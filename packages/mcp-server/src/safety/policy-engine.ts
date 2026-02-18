/**
 * Core safety evaluation engine.
 * All commands pass through here before reaching the bridge.
 */

import type { SafetyPolicy, SafetyCheckResult, SafetyViolation, VelocityLimits, AccelerationLimits, GeofenceBounds, DeadmanSwitchConfig, CommandApprovalConfig } from './types.js';
import { CommandApprovalManager } from './command-approval.js';
import { checkGeofence, type Position } from './geofence.js';
import { RateLimiter } from './rate-limiter.js';
import { AuditLogger } from './audit-logger.js';
import { SafetyScoreTracker } from './safety-score.js';
import { loadPolicy } from './policy-loader.js';

/**
 * Core safety evaluation engine.
 *
 * All publish, service, and action commands pass through the policy engine
 * before reaching the ROS2 bridge. The engine enforces velocity limits,
 * geofence boundaries, rate limits, blocked topics/services, emergency stop,
 * and the deadman switch.
 *
 * Every command evaluation is recorded in the audit log regardless of outcome.
 */
export class PolicyEngine {
  private policy: SafetyPolicy;
  private rateLimiter: RateLimiter;
  private auditLogger: AuditLogger;
  private scoreTracker: SafetyScoreTracker;
  private emergencyStopActive = false;
  private lastHeartbeat = Date.now();
  private deadmanTimer: ReturnType<typeof setInterval> | null = null;
  private approvalManager: CommandApprovalManager;
  private lastLinearSpeed = 0;
  private lastAngularRate = 0;
  private lastVelocityTime = 0;

  constructor(policyPath?: string) {
    this.policy = loadPolicy(policyPath);
    this.rateLimiter = new RateLimiter(this.policy.rateLimits);
    this.auditLogger = new AuditLogger();
    this.scoreTracker = new SafetyScoreTracker();
    this.approvalManager = new CommandApprovalManager(this.policy.commandApproval);
    console.error(`[PolicyEngine] Loaded policy: ${this.policy.name}`);
    if (this.policy.deadmanSwitch.enabled) {
      this.startDeadmanSwitch();
    }
  }

  /**
   * Check whether a topic publish is allowed by the safety policy.
   *
   * Evaluates: e-stop → blocked topics → velocity limits → rate limits.
   * In clamp mode, over-limit velocities are reduced to the maximum and
   * the command is allowed with a warning. The message object may be
   * mutated in-place when clamping.
   *
   * @param topic - The ROS2 topic name (e.g., "/cmd_vel")
   * @param message - The message payload to publish
   * @returns Safety check result with allowed flag and any violations
   */
  checkPublish(topic: string, message: Record<string, unknown>): SafetyCheckResult {
    const violations: SafetyViolation[] = [];

    if (this.emergencyStopActive) {
      violations.push({
        type: 'emergency_stop_active',
        message: 'Emergency stop is active. Release e-stop before publishing.',
        details: {},
        timestamp: Date.now(),
      });
    }

    if (this.isTopicBlocked(topic)) {
      violations.push({
        type: 'blocked_topic',
        message: `Topic "${topic}" is blocked by safety policy`,
        details: { topic, blockedTopics: this.policy.blockedTopics },
        timestamp: Date.now(),
      });
    }

    // Check velocity limits for cmd_vel-like topics
    const clampWarnings: SafetyViolation[] = [];
    if (topic.includes('cmd_vel')) {
      const velViolation = this.checkVelocityMessage(message, topic);
      if (velViolation) {
        if (velViolation.type === 'velocity_clamped') {
          clampWarnings.push(velViolation);
        } else {
          violations.push(velViolation);
        }
      }

      // Check acceleration limits
      const accelViolation = this.checkAcceleration(message);
      if (accelViolation) {
        violations.push(accelViolation);
      }
    }

    const rateViolation = this.rateLimiter.checkPublish(topic);
    if (rateViolation) violations.push(rateViolation);

    const result: SafetyCheckResult = {
      allowed: violations.length === 0,
      violations: [...violations, ...clampWarnings],
    };

    if (result.allowed) {
      this.scoreTracker.recordAllowed();
    } else {
      this.scoreTracker.recordBlocked(violations.map(v => v.type));
    }

    this.auditLogger.log('publish', topic, message, result);
    return result;
  }

  /**
   * Check whether a service call is allowed by the safety policy.
   *
   * Evaluates: e-stop → blocked services → rate limits.
   *
   * @param service - The ROS2 service name (e.g., "/set_parameters")
   * @param args - The service request arguments
   * @returns Safety check result with allowed flag and any violations
   */
  checkServiceCall(service: string, args: Record<string, unknown>): SafetyCheckResult {
    const violations: SafetyViolation[] = [];

    if (this.emergencyStopActive) {
      violations.push({
        type: 'emergency_stop_active',
        message: 'Emergency stop is active. Release e-stop before calling services.',
        details: {},
        timestamp: Date.now(),
      });
    }

    if (this.isServiceBlocked(service)) {
      violations.push({
        type: 'blocked_service',
        message: `Service "${service}" is blocked by safety policy`,
        details: { service, blockedServices: this.policy.blockedServices },
        timestamp: Date.now(),
      });
    }

    const rateViolation = this.rateLimiter.checkService(service);
    if (rateViolation) violations.push(rateViolation);

    const result: SafetyCheckResult = {
      allowed: violations.length === 0,
      violations,
    };

    if (result.allowed) {
      this.scoreTracker.recordAllowed();
    } else {
      this.scoreTracker.recordBlocked(violations.map(v => v.type));
    }

    this.auditLogger.log('service_call', service, args, result);
    return result;
  }

  /**
   * Check whether an action goal is allowed by the safety policy.
   *
   * Evaluates: e-stop → rate limits.
   *
   * @param action - The ROS2 action name (e.g., "/navigate_to_pose")
   * @param goal - The action goal parameters
   * @returns Safety check result with allowed flag and any violations
   */
  checkActionGoal(action: string, goal: Record<string, unknown>): SafetyCheckResult {
    const violations: SafetyViolation[] = [];

    if (this.emergencyStopActive) {
      violations.push({
        type: 'emergency_stop_active',
        message: 'Emergency stop is active. Release e-stop before sending goals.',
        details: {},
        timestamp: Date.now(),
      });
    }

    const rateViolation = this.rateLimiter.checkAction(action);
    if (rateViolation) violations.push(rateViolation);

    const result: SafetyCheckResult = {
      allowed: violations.length === 0,
      violations,
    };

    if (result.allowed) {
      this.scoreTracker.recordAllowed();
    } else {
      this.scoreTracker.recordBlocked(violations.map(v => v.type));
    }

    this.auditLogger.log('action_goal', action, goal, result);
    return result;
  }

  /** Get velocity limits for a specific topic, with per-topic overrides applied. */
  private getVelocityLimitsForTopic(topic: string): VelocityLimits {
    const base = this.policy.velocity;
    const overrides = this.policy.topicVelocityOverrides;
    if (!overrides || overrides.length === 0) return base;

    const match = overrides.find(o => topic.includes(o.topic) || topic === o.topic);
    if (!match) return base;

    return {
      linearMax: match.linearMax ?? base.linearMax,
      angularMax: match.angularMax ?? base.angularMax,
      clampMode: base.clampMode,
    };
  }

  private checkVelocityMessage(message: Record<string, unknown>, topic = '/cmd_vel'): SafetyViolation | null {
    const linear = message.linear as Record<string, number> | undefined;
    const angular = message.angular as Record<string, number> | undefined;
    const limits = this.getVelocityLimitsForTopic(topic);

    if (linear) {
      const speed = Math.sqrt((linear.x || 0) ** 2 + (linear.y || 0) ** 2 + (linear.z || 0) ** 2);
      if (speed > limits.linearMax) {
        if (limits.clampMode && speed > 0) {
          // Clamp: scale down to max instead of blocking
          const scale = limits.linearMax / speed;
          linear.x = (linear.x || 0) * scale;
          linear.y = (linear.y || 0) * scale;
          linear.z = (linear.z || 0) * scale;
          // Return a warning violation but still allow (caller checks clampMode)
          return {
            type: 'velocity_clamped',
            message: `Linear velocity ${speed.toFixed(2)} m/s clamped to ${limits.linearMax} m/s`,
            details: { original: speed, clamped: limits.linearMax, linear },
            timestamp: Date.now(),
          };
        }
        return {
          type: 'velocity_exceeded',
          message: `Linear velocity ${speed.toFixed(2)} m/s exceeds limit of ${limits.linearMax} m/s`,
          details: { linear, limit: limits.linearMax, actual: speed },
          timestamp: Date.now(),
        };
      }
    }

    if (angular) {
      const rate = Math.sqrt((angular.x || 0) ** 2 + (angular.y || 0) ** 2 + (angular.z || 0) ** 2);
      if (rate > limits.angularMax) {
        if (limits.clampMode && rate > 0) {
          const scale = limits.angularMax / rate;
          angular.x = (angular.x || 0) * scale;
          angular.y = (angular.y || 0) * scale;
          angular.z = (angular.z || 0) * scale;
          return {
            type: 'velocity_clamped',
            message: `Angular velocity ${rate.toFixed(2)} rad/s clamped to ${limits.angularMax} rad/s`,
            details: { original: rate, clamped: limits.angularMax, angular },
            timestamp: Date.now(),
          };
        }
        return {
          type: 'velocity_exceeded',
          message: `Angular velocity ${rate.toFixed(2)} rad/s exceeds limit of ${limits.angularMax} rad/s`,
          details: { angular, limit: limits.angularMax, actual: rate },
          timestamp: Date.now(),
        };
      }
    }

    return null;
  }

  private checkAcceleration(message: Record<string, unknown>): SafetyViolation | null {
    const accelLimits = this.policy.acceleration;
    if (!accelLimits.enabled) return null;

    const linear = message.linear as Record<string, number> | undefined;
    const angular = message.angular as Record<string, number> | undefined;
    const now = Date.now();
    const dt = this.lastVelocityTime > 0 ? (now - this.lastVelocityTime) / 1000 : 0;

    if (dt <= 0 || dt > 2) {
      // First command or too much time passed — just record state
      if (linear) {
        this.lastLinearSpeed = Math.sqrt((linear.x || 0) ** 2 + (linear.y || 0) ** 2 + (linear.z || 0) ** 2);
      }
      if (angular) {
        this.lastAngularRate = Math.sqrt((angular.x || 0) ** 2 + (angular.y || 0) ** 2 + (angular.z || 0) ** 2);
      }
      this.lastVelocityTime = now;
      return null;
    }

    const violations: string[] = [];

    if (linear) {
      const speed = Math.sqrt((linear.x || 0) ** 2 + (linear.y || 0) ** 2 + (linear.z || 0) ** 2);
      const linearAccel = Math.abs(speed - this.lastLinearSpeed) / dt;
      if (linearAccel > accelLimits.linearMaxAccel) {
        violations.push(`linear accel ${linearAccel.toFixed(2)} m/s² exceeds limit ${accelLimits.linearMaxAccel} m/s²`);
      }
      this.lastLinearSpeed = speed;
    }

    if (angular) {
      const rate = Math.sqrt((angular.x || 0) ** 2 + (angular.y || 0) ** 2 + (angular.z || 0) ** 2);
      const angularAccel = Math.abs(rate - this.lastAngularRate) / dt;
      if (angularAccel > accelLimits.angularMaxAccel) {
        violations.push(`angular accel ${angularAccel.toFixed(2)} rad/s² exceeds limit ${accelLimits.angularMaxAccel} rad/s²`);
      }
      this.lastAngularRate = rate;
    }

    this.lastVelocityTime = now;

    if (violations.length > 0) {
      return {
        type: 'acceleration_exceeded',
        message: `Acceleration limit exceeded: ${violations.join(', ')}`,
        details: { violations, dt },
        timestamp: now,
      };
    }

    return null;
  }

  /**
   * Check if a position is near the geofence boundary.
   * Returns a warning (non-blocking) if within the warning margin.
   */
  checkGeofenceProximity(position: Position): SafetyViolation | null {
    const margin = this.policy.geofenceWarningMargin;
    if (margin <= 0) return null;

    const b = this.policy.geofence;
    const warnings: string[] = [];

    if (position.x - b.xMin < margin) warnings.push(`x=${position.x} within ${margin}m of xMin=${b.xMin}`);
    if (b.xMax - position.x < margin) warnings.push(`x=${position.x} within ${margin}m of xMax=${b.xMax}`);
    if (position.y - b.yMin < margin) warnings.push(`y=${position.y} within ${margin}m of yMin=${b.yMin}`);
    if (b.yMax - position.y < margin) warnings.push(`y=${position.y} within ${margin}m of yMax=${b.yMax}`);
    if (position.z - b.zMin < margin) warnings.push(`z=${position.z} within ${margin}m of zMin=${b.zMin}`);
    if (b.zMax - position.z < margin) warnings.push(`z=${position.z} within ${margin}m of zMax=${b.zMax}`);

    if (warnings.length > 0) {
      return {
        type: 'geofence_warning',
        message: `Approaching geofence boundary: ${warnings.join(', ')}`,
        details: { position, margin, warnings },
        timestamp: Date.now(),
      };
    }

    return null;
  }

  private isTopicBlocked(topic: string): boolean {
    if (this.policy.allowedTopics) {
      return !this.policy.allowedTopics.some(t => topic.startsWith(t));
    }
    return this.policy.blockedTopics.some(t => topic.startsWith(t));
  }

  private isServiceBlocked(service: string): boolean {
    if (this.policy.allowedServices) {
      return !this.policy.allowedServices.some(s => service.startsWith(s));
    }
    return this.policy.blockedServices.some(s => service.startsWith(s));
  }

  /** Activate emergency stop. Blocks all commands until released. */
  activateEmergencyStop(): void {
    this.emergencyStopActive = true;
    console.error('[PolicyEngine] EMERGENCY STOP ACTIVATED');
    this.auditLogger.log('emergency_stop', 'system', {}, { allowed: true, violations: [] });
  }

  /** Release emergency stop and allow commands to resume. */
  releaseEmergencyStop(): void {
    this.emergencyStopActive = false;
    console.error('[PolicyEngine] Emergency stop released');
    this.auditLogger.log('emergency_stop_release', 'system', {}, { allowed: true, violations: [] });
  }

  get isEmergencyStopActive(): boolean {
    return this.emergencyStopActive;
  }

  // Deadman switch
  private startDeadmanSwitch(): void {
    this.stopDeadmanSwitch();
    this.lastHeartbeat = Date.now();
    const checkInterval = Math.max(1000, this.policy.deadmanSwitch.timeoutMs / 3);
    this.deadmanTimer = setInterval(() => {
      if (!this.policy.deadmanSwitch.enabled) return;
      const elapsed = Date.now() - this.lastHeartbeat;
      if (elapsed > this.policy.deadmanSwitch.timeoutMs && !this.emergencyStopActive) {
        console.error(`[PolicyEngine] Deadman switch timeout (${elapsed}ms) — activating e-stop`);
        this.activateEmergencyStop();
        this.auditLogger.log('deadman_switch_timeout', 'system', { elapsed }, { allowed: true, violations: [] });
      }
    }, checkInterval);
  }

  private stopDeadmanSwitch(): void {
    if (this.deadmanTimer) {
      clearInterval(this.deadmanTimer);
      this.deadmanTimer = null;
    }
  }

  /** Send a heartbeat to the deadman switch, resetting its timeout timer. */
  heartbeat(): void {
    this.lastHeartbeat = Date.now();
  }

  updateDeadmanSwitch(config: Partial<DeadmanSwitchConfig>): void {
    const wasEnabled = this.policy.deadmanSwitch.enabled;
    Object.assign(this.policy.deadmanSwitch, config);
    if (this.policy.deadmanSwitch.enabled && !wasEnabled) {
      this.startDeadmanSwitch();
      console.error(`[PolicyEngine] Deadman switch enabled (timeout: ${this.policy.deadmanSwitch.timeoutMs}ms)`);
    } else if (!this.policy.deadmanSwitch.enabled && wasEnabled) {
      this.stopDeadmanSwitch();
      console.error('[PolicyEngine] Deadman switch disabled');
    }
  }

  // Policy management
  getPolicy(): SafetyPolicy {
    return { ...this.policy };
  }

  updateVelocityLimits(limits: Partial<VelocityLimits>): void {
    Object.assign(this.policy.velocity, limits);
    console.error(`[PolicyEngine] Velocity limits updated: linear=${this.policy.velocity.linearMax}, angular=${this.policy.velocity.angularMax}, clamp=${this.policy.velocity.clampMode}`);
  }

  updateAccelerationLimits(limits: Partial<AccelerationLimits>): void {
    Object.assign(this.policy.acceleration, limits);
    console.error(`[PolicyEngine] Acceleration limits updated: linear=${this.policy.acceleration.linearMaxAccel}, angular=${this.policy.acceleration.angularMaxAccel}, enabled=${this.policy.acceleration.enabled}`);
  }

  updateGeofence(bounds: Partial<GeofenceBounds>): void {
    Object.assign(this.policy.geofence, bounds);
    console.error(`[PolicyEngine] Geofence updated`);
  }

  // Audit
  getAuditLog(options?: { limit?: number; command?: string; violationsOnly?: boolean }) {
    return this.auditLogger.getEntries(options);
  }

  getAuditStats() {
    return this.auditLogger.getStats();
  }

  exportAuditLog(filePath: string, options?: { violationsOnly?: boolean; command?: string }): number {
    return this.auditLogger.exportToFile(filePath, options);
  }

  /**
   * Check a position against geofence and return both violation and proximity warnings.
   */
  checkPosition(position: Position): { inside: boolean; violation: SafetyViolation | null; warning: SafetyViolation | null } {
    const violation = checkGeofence(position, this.policy.geofence);
    const warning = violation ? null : this.checkGeofenceProximity(position);
    return {
      inside: violation === null,
      violation,
      warning,
    };
  }

  getStatus() {
    return {
      policyName: this.policy.name,
      emergencyStopActive: this.emergencyStopActive,
      velocity: this.policy.velocity,
      acceleration: this.policy.acceleration,
      geofence: this.policy.geofence,
      geofenceWarningMargin: this.policy.geofenceWarningMargin,
      rateLimits: this.policy.rateLimits,
      deadmanSwitch: {
        ...this.policy.deadmanSwitch,
        lastHeartbeat: this.lastHeartbeat,
        timeSinceHeartbeat: Date.now() - this.lastHeartbeat,
      },
      rateLimiterStats: this.rateLimiter.getStats(),
      safetyScore: this.scoreTracker.getSnapshot(),
      auditStats: this.auditLogger.getStats(),
    };
  }

  /** Get the command approval manager for human-in-the-loop workflows. */
  get approval(): CommandApprovalManager {
    return this.approvalManager;
  }

  destroy(): void {
    this.stopDeadmanSwitch();
  }
}

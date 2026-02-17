/**
 * Core safety evaluation engine.
 * All commands pass through here before reaching the bridge.
 */

import type { SafetyPolicy, SafetyCheckResult, SafetyViolation, VelocityLimits, GeofenceBounds } from './types.js';
import { checkGeofence, type Position } from './geofence.js';
import { RateLimiter } from './rate-limiter.js';
import { AuditLogger } from './audit-logger.js';
import { loadPolicy } from './policy-loader.js';

export class PolicyEngine {
  private policy: SafetyPolicy;
  private rateLimiter: RateLimiter;
  private auditLogger: AuditLogger;
  private emergencyStopActive = false;

  constructor(policyPath?: string) {
    this.policy = loadPolicy(policyPath);
    this.rateLimiter = new RateLimiter(this.policy.rateLimits);
    this.auditLogger = new AuditLogger();
    console.error(`[PolicyEngine] Loaded policy: ${this.policy.name}`);
  }

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
    if (topic.includes('cmd_vel')) {
      const velViolation = this.checkVelocityMessage(message);
      if (velViolation) violations.push(velViolation);
    }

    const rateViolation = this.rateLimiter.checkPublish(topic);
    if (rateViolation) violations.push(rateViolation);

    const result: SafetyCheckResult = {
      allowed: violations.length === 0,
      violations,
    };

    this.auditLogger.log('publish', topic, message, result);
    return result;
  }

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

    this.auditLogger.log('service_call', service, args, result);
    return result;
  }

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

    this.auditLogger.log('action_goal', action, goal, result);
    return result;
  }

  private checkVelocityMessage(message: Record<string, unknown>): SafetyViolation | null {
    const linear = message.linear as Record<string, number> | undefined;
    const angular = message.angular as Record<string, number> | undefined;
    const limits = this.policy.velocity;

    if (linear) {
      const speed = Math.sqrt((linear.x || 0) ** 2 + (linear.y || 0) ** 2 + (linear.z || 0) ** 2);
      if (speed > limits.linearMax) {
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

  // Emergency stop
  activateEmergencyStop(): void {
    this.emergencyStopActive = true;
    console.error('[PolicyEngine] EMERGENCY STOP ACTIVATED');
    this.auditLogger.log('emergency_stop', 'system', {}, { allowed: true, violations: [] });
  }

  releaseEmergencyStop(): void {
    this.emergencyStopActive = false;
    console.error('[PolicyEngine] Emergency stop released');
    this.auditLogger.log('emergency_stop_release', 'system', {}, { allowed: true, violations: [] });
  }

  get isEmergencyStopActive(): boolean {
    return this.emergencyStopActive;
  }

  // Policy management
  getPolicy(): SafetyPolicy {
    return { ...this.policy };
  }

  updateVelocityLimits(limits: Partial<VelocityLimits>): void {
    Object.assign(this.policy.velocity, limits);
    console.error(`[PolicyEngine] Velocity limits updated: linear=${this.policy.velocity.linearMax}, angular=${this.policy.velocity.angularMax}`);
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

  getStatus() {
    return {
      policyName: this.policy.name,
      emergencyStopActive: this.emergencyStopActive,
      velocity: this.policy.velocity,
      geofence: this.policy.geofence,
      rateLimits: this.policy.rateLimits,
      auditStats: this.auditLogger.getStats(),
    };
  }
}

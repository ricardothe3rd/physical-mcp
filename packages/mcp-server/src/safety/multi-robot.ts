/**
 * Multi-robot safety isolation module.
 *
 * Manages multiple robots, each with its own ROS2 namespace and safety policy.
 * Ensures commands are namespaced correctly and per-robot policies are enforced.
 */

import type { SafetyPolicy } from './types.js';
import { getDefaultPolicy } from './policy-loader.js';

/** Maximum number of robots that can be registered simultaneously. */
const MAX_ROBOTS = 20;

/** Robots with commands within this window (ms) are considered active. */
const ACTIVE_WINDOW_MS = 60_000;

/** Profile for a registered robot. */
export interface RobotProfile {
  robotId: string;
  name: string;
  namespace: string;
  policy: SafetyPolicy;
  enabled: boolean;
  registeredAt: number;
  lastCommandAt: number | null;
}

/**
 * Manages registration, namespacing, and per-robot safety policies
 * for multi-robot deployments.
 */
export class MultiRobotManager {
  private robots: Map<string, RobotProfile> = new Map();

  /**
   * Register a robot with an optional per-robot safety policy.
   *
   * If a robot with the same ID already exists, the registration is updated.
   * Throws if the maximum robot limit has been reached (and this is a new robot).
   *
   * @param robotId  - Unique identifier for the robot
   * @param name     - Human-readable name
   * @param namespace - ROS2 namespace (e.g., "/robot1")
   * @param policy   - Per-robot safety policy; uses a conservative default if omitted
   * @returns The created or updated RobotProfile
   */
  registerRobot(
    robotId: string,
    name: string,
    namespace: string,
    policy?: SafetyPolicy,
  ): RobotProfile {
    const existing = this.robots.get(robotId);

    if (!existing && this.robots.size >= MAX_ROBOTS) {
      throw new Error(
        `Maximum robot limit (${MAX_ROBOTS}) reached. Unregister a robot before adding a new one.`,
      );
    }

    const profile: RobotProfile = {
      robotId,
      name,
      namespace,
      policy: policy ? deepClonePolicy(policy) : getDefaultPolicy(),
      enabled: true,
      registeredAt: existing?.registeredAt ?? Date.now(),
      lastCommandAt: existing?.lastCommandAt ?? null,
    };

    this.robots.set(robotId, profile);
    return { ...profile, policy: deepClonePolicy(profile.policy) };
  }

  /**
   * Unregister a robot by ID.
   * @returns true if the robot was found and removed, false otherwise.
   */
  unregisterRobot(robotId: string): boolean {
    return this.robots.delete(robotId);
  }

  /**
   * Get a robot profile by ID.
   * @returns A copy of the profile, or null if not found.
   */
  getRobot(robotId: string): RobotProfile | null {
    const profile = this.robots.get(robotId);
    if (!profile) return null;
    return { ...profile, policy: deepClonePolicy(profile.policy) };
  }

  /**
   * List all registered robots.
   * @returns Array of robot profile copies.
   */
  listRobots(): RobotProfile[] {
    return Array.from(this.robots.values()).map((p) => ({
      ...p,
      policy: deepClonePolicy(p.policy),
    }));
  }

  /**
   * Enable a robot, allowing commands to be sent to it.
   * @returns true if the robot was found and enabled, false otherwise.
   */
  enableRobot(robotId: string): boolean {
    const profile = this.robots.get(robotId);
    if (!profile) return false;
    profile.enabled = true;
    return true;
  }

  /**
   * Disable a robot, blocking all commands to it.
   * @returns true if the robot was found and disabled, false otherwise.
   */
  disableRobot(robotId: string): boolean {
    const profile = this.robots.get(robotId);
    if (!profile) return false;
    profile.enabled = false;
    return true;
  }

  /**
   * Get the safety policy for a specific robot.
   * @returns A deep copy of the policy, or null if the robot is not found.
   */
  getPolicy(robotId: string): SafetyPolicy | null {
    const profile = this.robots.get(robotId);
    if (!profile) return null;
    return deepClonePolicy(profile.policy);
  }

  /**
   * Update the safety policy for a specific robot.
   * @returns true if the robot was found and the policy updated, false otherwise.
   */
  updatePolicy(robotId: string, policy: SafetyPolicy): boolean {
    const profile = this.robots.get(robotId);
    if (!profile) return false;
    profile.policy = deepClonePolicy(policy);
    return true;
  }

  /**
   * Resolve a ROS2 topic name within a robot's namespace.
   *
   * Handles edge cases:
   * - Empty namespace: returns the topic unchanged
   * - Double slashes: collapsed (e.g., "/robot1/" + "/cmd_vel" => "/robot1/cmd_vel")
   * - Missing leading slash on topic: inserted automatically
   *
   * @param robotId - The robot whose namespace to use
   * @param topic   - The ROS2 topic name (e.g., "/cmd_vel")
   * @returns The fully resolved topic string
   * @throws Error if the robot is not registered
   */
  resolveNamespace(robotId: string, topic: string): string {
    const profile = this.robots.get(robotId);
    if (!profile) {
      throw new Error(`Robot "${robotId}" is not registered`);
    }

    const ns = profile.namespace;

    // Empty namespace — return topic as-is (ensure leading slash)
    if (!ns || ns === '') {
      return topic.startsWith('/') ? topic : `/${topic}`;
    }

    // Ensure topic part has a leading slash for concatenation
    const normalizedTopic = topic.startsWith('/') ? topic : `/${topic}`;

    // Concatenate and collapse any double slashes
    const raw = `${ns}${normalizedTopic}`;
    return raw.replace(/\/\/+/g, '/');
  }

  /**
   * Check whether a command is allowed for the given robot.
   *
   * A command is disallowed if:
   * - The robot is not registered
   * - The robot is disabled
   */
  isCommandAllowed(robotId: string): { allowed: boolean; reason?: string } {
    const profile = this.robots.get(robotId);
    if (!profile) {
      return { allowed: false, reason: `Robot "${robotId}" is not registered` };
    }
    if (!profile.enabled) {
      return { allowed: false, reason: `Robot "${robotId}" is disabled` };
    }
    return { allowed: true };
  }

  /**
   * Record that a command was sent to the given robot.
   * Updates the lastCommandAt timestamp.
   */
  recordCommand(robotId: string): void {
    const profile = this.robots.get(robotId);
    if (!profile) return;
    profile.lastCommandAt = Date.now();
  }

  /** Return the total number of registered robots. */
  getRobotCount(): number {
    return this.robots.size;
  }

  /**
   * Return robots that have received a command within the last 60 seconds.
   */
  getActiveRobots(): RobotProfile[] {
    const cutoff = Date.now() - ACTIVE_WINDOW_MS;
    return Array.from(this.robots.values())
      .filter((p) => p.lastCommandAt !== null && p.lastCommandAt >= cutoff)
      .map((p) => ({ ...p, policy: deepClonePolicy(p.policy) }));
  }
}

/** Deep-clone a SafetyPolicy so mutations to the copy never affect the original. */
function deepClonePolicy(policy: SafetyPolicy): SafetyPolicy {
  return {
    ...policy,
    velocity: { ...policy.velocity },
    acceleration: { ...policy.acceleration },
    geofence: { ...policy.geofence },
    rateLimits: { ...policy.rateLimits },
    deadmanSwitch: { ...policy.deadmanSwitch },
    blockedTopics: [...policy.blockedTopics],
    blockedServices: [...policy.blockedServices],
    allowedTopics: policy.allowedTopics ? [...policy.allowedTopics] : undefined,
    allowedServices: policy.allowedServices ? [...policy.allowedServices] : undefined,
    topicVelocityOverrides: policy.topicVelocityOverrides
      ? policy.topicVelocityOverrides.map((o) => ({ ...o }))
      : undefined,
    commandApproval: {
      ...policy.commandApproval,
      requireApprovalFor: [...policy.commandApproval.requireApprovalFor],
    },
  };
}

/**
 * Safety policy inheritance: merge base + override policies.
 *
 * Enables layered safety configurations where a base policy defines
 * defaults and specialized policies override specific fields.
 *
 * Example: A "turtlebot3" policy extends "conservative" base with
 * robot-specific velocity limits and geofence.
 */

import type { SafetyPolicy } from './types.js';
import { getDefaultPolicy } from './policy-loader.js';

/**
 * Deep merge two policy objects. Override values take precedence.
 * Arrays from override completely replace base arrays.
 */
export function mergePolicies(base: SafetyPolicy, override: Partial<SafetyPolicy>): SafetyPolicy {
  const result = getDefaultPolicy();

  // Copy base
  Object.assign(result, base);
  result.velocity = { ...base.velocity };
  result.acceleration = { ...base.acceleration };
  result.geofence = { ...base.geofence };
  result.rateLimits = { ...base.rateLimits };
  result.deadmanSwitch = { ...base.deadmanSwitch };
  result.commandApproval = {
    ...base.commandApproval,
    requireApprovalFor: [...base.commandApproval.requireApprovalFor],
  };
  result.blockedTopics = [...base.blockedTopics];
  result.blockedServices = [...base.blockedServices];
  if (base.topicVelocityOverrides) {
    result.topicVelocityOverrides = base.topicVelocityOverrides.map(o => ({ ...o }));
  }
  if (base.allowedTopics) result.allowedTopics = [...base.allowedTopics];
  if (base.allowedServices) result.allowedServices = [...base.allowedServices];

  // Apply overrides
  if (override.name !== undefined) result.name = override.name;
  if (override.description !== undefined) result.description = override.description;

  if (override.velocity) {
    Object.assign(result.velocity, override.velocity);
  }
  if (override.acceleration) {
    Object.assign(result.acceleration, override.acceleration);
  }
  if (override.geofence) {
    Object.assign(result.geofence, override.geofence);
  }
  if (override.geofenceWarningMargin !== undefined) {
    result.geofenceWarningMargin = override.geofenceWarningMargin;
  }
  if (override.rateLimits) {
    Object.assign(result.rateLimits, override.rateLimits);
  }
  if (override.deadmanSwitch) {
    Object.assign(result.deadmanSwitch, override.deadmanSwitch);
  }
  if (override.commandApproval) {
    Object.assign(result.commandApproval, override.commandApproval);
  }

  // Arrays: override replaces entirely
  if (override.blockedTopics) result.blockedTopics = [...override.blockedTopics];
  if (override.blockedServices) result.blockedServices = [...override.blockedServices];
  if (override.allowedTopics !== undefined) result.allowedTopics = override.allowedTopics ? [...override.allowedTopics] : undefined;
  if (override.allowedServices !== undefined) result.allowedServices = override.allowedServices ? [...override.allowedServices] : undefined;
  if (override.topicVelocityOverrides !== undefined) {
    result.topicVelocityOverrides = override.topicVelocityOverrides
      ? override.topicVelocityOverrides.map(o => ({ ...o }))
      : undefined;
  }

  return result;
}

/**
 * Build a policy by chaining multiple layers of overrides.
 * Later layers take precedence over earlier ones.
 */
export function buildPolicyChain(layers: Array<Partial<SafetyPolicy>>): SafetyPolicy {
  let result = getDefaultPolicy();
  for (const layer of layers) {
    result = mergePolicies(result, layer);
  }
  return result;
}

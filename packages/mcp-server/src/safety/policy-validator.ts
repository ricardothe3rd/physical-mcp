/**
 * Validate safety policy configuration for correctness and sanity.
 */

import type { SafetyPolicy } from './types.js';

export interface ValidationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
}

export function validatePolicy(policy: SafetyPolicy): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  // Name and description
  if (!policy.name || policy.name.trim() === '') {
    errors.push('Policy name is required');
  }

  // Velocity limits
  if (policy.velocity.linearMax <= 0) {
    errors.push(`linearMax must be positive, got ${policy.velocity.linearMax}`);
  }
  if (policy.velocity.angularMax <= 0) {
    errors.push(`angularMax must be positive, got ${policy.velocity.angularMax}`);
  }
  if (policy.velocity.linearMax > 10) {
    warnings.push(`linearMax of ${policy.velocity.linearMax} m/s is very high — most robots max out at ~2 m/s`);
  }
  if (policy.velocity.angularMax > 10) {
    warnings.push(`angularMax of ${policy.velocity.angularMax} rad/s is very high`);
  }

  // Acceleration limits
  if (policy.acceleration.enabled) {
    if (policy.acceleration.linearMaxAccel <= 0) {
      errors.push(`linearMaxAccel must be positive, got ${policy.acceleration.linearMaxAccel}`);
    }
    if (policy.acceleration.angularMaxAccel <= 0) {
      errors.push(`angularMaxAccel must be positive, got ${policy.acceleration.angularMaxAccel}`);
    }
  }

  // Geofence
  if (policy.geofence.xMin >= policy.geofence.xMax) {
    errors.push(`Geofence xMin (${policy.geofence.xMin}) must be less than xMax (${policy.geofence.xMax})`);
  }
  if (policy.geofence.yMin >= policy.geofence.yMax) {
    errors.push(`Geofence yMin (${policy.geofence.yMin}) must be less than yMax (${policy.geofence.yMax})`);
  }
  if (policy.geofence.zMin >= policy.geofence.zMax) {
    errors.push(`Geofence zMin (${policy.geofence.zMin}) must be less than zMax (${policy.geofence.zMax})`);
  }

  const xRange = policy.geofence.xMax - policy.geofence.xMin;
  const yRange = policy.geofence.yMax - policy.geofence.yMin;
  if (xRange > 1000 || yRange > 1000) {
    warnings.push(`Geofence is very large (${xRange}m x ${yRange}m) — consider tighter bounds`);
  }
  if (xRange < 0.5 || yRange < 0.5) {
    warnings.push(`Geofence is very small (${xRange}m x ${yRange}m) — robot may not have room to move`);
  }

  // Geofence warning margin
  if (policy.geofenceWarningMargin < 0) {
    errors.push(`geofenceWarningMargin must be non-negative, got ${policy.geofenceWarningMargin}`);
  }
  const minDim = Math.min(xRange, yRange);
  if (policy.geofenceWarningMargin > minDim / 2) {
    warnings.push(`geofenceWarningMargin (${policy.geofenceWarningMargin}m) is more than half the smallest geofence dimension — warnings will fire constantly`);
  }

  // Rate limits
  if (policy.rateLimits.publishHz <= 0) {
    errors.push(`publishHz must be positive, got ${policy.rateLimits.publishHz}`);
  }
  if (policy.rateLimits.servicePerMinute <= 0) {
    errors.push(`servicePerMinute must be positive, got ${policy.rateLimits.servicePerMinute}`);
  }
  if (policy.rateLimits.actionPerMinute <= 0) {
    errors.push(`actionPerMinute must be positive, got ${policy.rateLimits.actionPerMinute}`);
  }
  if (policy.rateLimits.publishHz > 1000) {
    warnings.push(`publishHz of ${policy.rateLimits.publishHz} is very high — may overwhelm the bridge`);
  }

  // Deadman switch
  if (policy.deadmanSwitch.enabled && policy.deadmanSwitch.timeoutMs < 1000) {
    warnings.push(`Deadman switch timeout of ${policy.deadmanSwitch.timeoutMs}ms is very short — may trigger accidentally`);
  }

  // Blocked lists
  if (policy.blockedTopics.length === 0) {
    warnings.push('No blocked topics — consider blocking /rosout and /parameter_events');
  }
  if (policy.blockedServices.length === 0) {
    warnings.push('No blocked services — consider blocking /kill and /shutdown');
  }

  // Allowed lists sanity
  if (policy.allowedTopics && policy.allowedTopics.length === 0) {
    errors.push('allowedTopics is empty — no topics would be allowed');
  }
  if (policy.allowedServices && policy.allowedServices.length === 0) {
    errors.push('allowedServices is empty — no services would be allowed');
  }

  return {
    valid: errors.length === 0,
    errors,
    warnings,
  };
}

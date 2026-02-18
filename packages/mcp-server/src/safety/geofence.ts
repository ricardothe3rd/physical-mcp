/**
 * Geofence boundary checking.
 * Supports rectangular bounds and circular zones.
 */

import type { GeofenceBounds, SafetyViolation } from './types.js';

export interface Position {
  x: number;
  y: number;
  z: number;
}

export interface CircularGeofence {
  centerX: number;
  centerY: number;
  radius: number;
  zMin: number;
  zMax: number;
}

/**
 * Check if a position is within rectangular geofence bounds.
 */
export function checkGeofence(position: Position, bounds: GeofenceBounds): SafetyViolation | null {
  const violations: string[] = [];

  if (position.x < bounds.xMin || position.x > bounds.xMax) {
    violations.push(`x=${position.x} outside [${bounds.xMin}, ${bounds.xMax}]`);
  }
  if (position.y < bounds.yMin || position.y > bounds.yMax) {
    violations.push(`y=${position.y} outside [${bounds.yMin}, ${bounds.yMax}]`);
  }
  if (position.z < bounds.zMin || position.z > bounds.zMax) {
    violations.push(`z=${position.z} outside [${bounds.zMin}, ${bounds.zMax}]`);
  }

  if (violations.length > 0) {
    return {
      type: 'geofence_violation',
      message: `Position outside geofence: ${violations.join(', ')}`,
      details: { position, bounds, violations },
      timestamp: Date.now(),
    };
  }

  return null;
}

/**
 * Check if a position is within a circular geofence.
 */
export function checkCircularGeofence(position: Position, fence: CircularGeofence): SafetyViolation | null {
  const dx = position.x - fence.centerX;
  const dy = position.y - fence.centerY;
  const distance = Math.sqrt(dx * dx + dy * dy);

  const violations: string[] = [];

  if (distance > fence.radius) {
    violations.push(`distance ${distance.toFixed(2)}m from center exceeds radius ${fence.radius}m`);
  }
  if (position.z < fence.zMin || position.z > fence.zMax) {
    violations.push(`z=${position.z} outside [${fence.zMin}, ${fence.zMax}]`);
  }

  if (violations.length > 0) {
    return {
      type: 'geofence_violation',
      message: `Position outside circular geofence: ${violations.join(', ')}`,
      details: { position, fence, distance, violations },
      timestamp: Date.now(),
    };
  }

  return null;
}

export function isInsideGeofence(position: Position, bounds: GeofenceBounds): boolean {
  return checkGeofence(position, bounds) === null;
}

export function isInsideCircularGeofence(position: Position, fence: CircularGeofence): boolean {
  return checkCircularGeofence(position, fence) === null;
}

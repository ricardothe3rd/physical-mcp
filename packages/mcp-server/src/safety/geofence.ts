/**
 * Geofence boundary checking.
 * Supports rectangular bounds and circular zones.
 */

import type { GeofenceBounds, SafetyViolation } from './types.js';

/** A 3D position in meters, typically in the robot's reference frame. */
export interface Position {
  x: number;
  y: number;
  z: number;
}

/** A cylindrical geofence defined by a center point, radius, and vertical bounds. */
export interface CircularGeofence {
  centerX: number;
  centerY: number;
  radius: number;
  zMin: number;
  zMax: number;
}

/**
 * Check if a position is within rectangular geofence bounds.
 * @param position - The 3D position to check
 * @param bounds - The axis-aligned bounding box defining the allowed region
 * @returns A geofence violation if outside bounds, or null if inside
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
 * Check if a position is within a circular (cylindrical) geofence.
 * @param position - The 3D position to check
 * @param fence - The circular geofence definition (center, radius, z bounds)
 * @returns A geofence violation if outside the fence, or null if inside
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

/** Convenience check: returns true if the position is inside the rectangular geofence. */
export function isInsideGeofence(position: Position, bounds: GeofenceBounds): boolean {
  return checkGeofence(position, bounds) === null;
}

/** Convenience check: returns true if the position is inside the circular geofence. */
export function isInsideCircularGeofence(position: Position, fence: CircularGeofence): boolean {
  return checkCircularGeofence(position, fence) === null;
}

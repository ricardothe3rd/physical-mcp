/**
 * Geofence boundary checking.
 */

import type { GeofenceBounds, SafetyViolation } from './types.js';

export interface Position {
  x: number;
  y: number;
  z: number;
}

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

export function isInsideGeofence(position: Position, bounds: GeofenceBounds): boolean {
  return checkGeofence(position, bounds) === null;
}

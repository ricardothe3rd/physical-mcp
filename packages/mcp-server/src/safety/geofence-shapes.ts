/**
 * Extended geofence shape support: rectangle, circle, and polygon.
 * Builds on the existing rectangular geofence in geofence.ts.
 */

import type { GeofenceBounds, SafetyViolation } from './types.js';
import { checkGeofence, type Position } from './geofence.js';

export type GeofenceShape =
  | { type: 'rectangle'; bounds: GeofenceBounds }
  | { type: 'circle'; center: { x: number; y: number }; radius: number; zMin: number; zMax: number }
  | { type: 'polygon'; vertices: Array<{ x: number; y: number }>; zMin: number; zMax: number };

/**
 * Determine whether a point (px, py) lies inside a polygon defined by vertices,
 * using the ray-casting algorithm. A ray is cast from the point in the +x direction
 * and the number of edge crossings is counted. An odd count means inside.
 */
export function isPointInPolygon(px: number, py: number, vertices: Array<{ x: number; y: number }>): boolean {
  const n = vertices.length;
  if (n < 3) return false;

  let inside = false;

  for (let i = 0, j = n - 1; i < n; j = i++) {
    const xi = vertices[i].x;
    const yi = vertices[i].y;
    const xj = vertices[j].x;
    const yj = vertices[j].y;

    // Check if the ray from (px, py) in the +x direction crosses edge (i, j).
    // The edge must straddle the y-coordinate of the point.
    const intersects =
      (yi > py) !== (yj > py) &&
      px < ((xj - xi) * (py - yi)) / (yj - yi) + xi;

    if (intersects) {
      inside = !inside;
    }
  }

  return inside;
}

/**
 * Check if a position is within a geofence shape.
 * Returns a SafetyViolation if the position is outside the shape, or null if inside.
 */
export function checkGeofenceShape(position: Position, shape: GeofenceShape): SafetyViolation | null {
  switch (shape.type) {
    case 'rectangle':
      return checkGeofence(position, shape.bounds);

    case 'circle':
      return checkCircleShape(position, shape);

    case 'polygon':
      return checkPolygonShape(position, shape);
  }
}

function checkCircleShape(
  position: Position,
  shape: { type: 'circle'; center: { x: number; y: number }; radius: number; zMin: number; zMax: number },
): SafetyViolation | null {
  const dx = position.x - shape.center.x;
  const dy = position.y - shape.center.y;
  const distance = Math.sqrt(dx * dx + dy * dy);

  const violations: string[] = [];

  if (distance > shape.radius) {
    violations.push(
      `distance ${distance.toFixed(2)}m from center exceeds radius ${shape.radius}m`,
    );
  }
  if (position.z < shape.zMin || position.z > shape.zMax) {
    violations.push(`z=${position.z} outside [${shape.zMin}, ${shape.zMax}]`);
  }

  if (violations.length > 0) {
    return {
      type: 'geofence_violation',
      message: `Position outside circular geofence: ${violations.join(', ')}`,
      details: { position, shape, distance, violations },
      timestamp: Date.now(),
    };
  }

  return null;
}

function checkPolygonShape(
  position: Position,
  shape: { type: 'polygon'; vertices: Array<{ x: number; y: number }>; zMin: number; zMax: number },
): SafetyViolation | null {
  const violations: string[] = [];

  if (!isPointInPolygon(position.x, position.y, shape.vertices)) {
    violations.push(
      `position (${position.x}, ${position.y}) is outside polygon boundary`,
    );
  }
  if (position.z < shape.zMin || position.z > shape.zMax) {
    violations.push(`z=${position.z} outside [${shape.zMin}, ${shape.zMax}]`);
  }

  if (violations.length > 0) {
    return {
      type: 'geofence_violation',
      message: `Position outside polygon geofence: ${violations.join(', ')}`,
      details: { position, shape, violations },
      timestamp: Date.now(),
    };
  }

  return null;
}

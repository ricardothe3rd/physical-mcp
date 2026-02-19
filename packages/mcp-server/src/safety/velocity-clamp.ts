/**
 * Velocity auto-clamping module.
 *
 * Instead of blocking commands that exceed velocity limits, this module
 * scales them down to the maximum allowed magnitude while preserving
 * direction. This provides a smoother experience for AI-controlled robots
 * while still enforcing safety constraints.
 */

import type { VelocityLimits } from './types.js';

/** A 3D velocity split into linear and angular components (Twist-like). */
export interface VelocityVector {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

/** Result of a velocity clamp operation. */
export interface ClampResult {
  /** Whether any component was clamped. */
  clamped: boolean;
  /** The original velocity before clamping. */
  original: VelocityVector;
  /** The resulting velocity after clamping. */
  result: VelocityVector;
  /** Human-readable descriptions of what was reduced. */
  reductions: string[];
}

/**
 * Compute the Euclidean magnitude of a 3D vector.
 * @param x - X component
 * @param y - Y component
 * @param z - Z component
 * @returns The magnitude sqrt(x^2 + y^2 + z^2)
 */
export function vectorMagnitude(x: number, y: number, z: number): number {
  return Math.sqrt(x * x + y * y + z * z);
}

/**
 * Scale a 3D vector so its magnitude does not exceed maxMagnitude.
 * If the vector's magnitude is already within the limit or is zero,
 * the original values are returned unchanged.
 *
 * @param x - X component
 * @param y - Y component
 * @param z - Z component
 * @param maxMagnitude - Maximum allowed magnitude (must be >= 0)
 * @returns The (possibly scaled) vector components
 */
export function scaleVector(
  x: number,
  y: number,
  z: number,
  maxMagnitude: number,
): { x: number; y: number; z: number } {
  const mag = vectorMagnitude(x, y, z);
  if (mag === 0 || mag <= maxMagnitude) {
    return { x, y, z };
  }
  const scale = maxMagnitude / mag;
  return {
    x: x * scale,
    y: y * scale,
    z: z * scale,
  };
}

/**
 * Clamp a Twist-like velocity message to the given limits.
 *
 * Linear and angular components are clamped independently by magnitude
 * (not per-axis), which preserves the direction of movement while
 * reducing speed to the allowed maximum.
 *
 * @param message - A record containing `linear` and `angular` sub-objects
 *   with `x`, `y`, `z` number fields (standard ROS2 Twist layout)
 * @param limits - The velocity limits to enforce
 * @returns A ClampResult describing what happened
 */
export function clampVelocity(
  message: Record<string, unknown>,
  limits: VelocityLimits,
): ClampResult {
  const linear = message.linear as Record<string, number> | undefined;
  const angular = message.angular as Record<string, number> | undefined;

  const lx = linear?.x ?? 0;
  const ly = linear?.y ?? 0;
  const lz = linear?.z ?? 0;
  const ax = angular?.x ?? 0;
  const ay = angular?.y ?? 0;
  const az = angular?.z ?? 0;

  const original: VelocityVector = {
    linear: { x: lx, y: ly, z: lz },
    angular: { x: ax, y: ay, z: az },
  };

  const reductions: string[] = [];

  // Clamp linear velocity by magnitude
  const linearMag = vectorMagnitude(lx, ly, lz);
  let resultLinear = { x: lx, y: ly, z: lz };
  if (linearMag > limits.linearMax) {
    resultLinear = scaleVector(lx, ly, lz, limits.linearMax);
    reductions.push(
      `linear velocity ${linearMag.toFixed(2)} m/s clamped to ${limits.linearMax} m/s`,
    );
  }

  // Clamp angular velocity by magnitude
  const angularMag = vectorMagnitude(ax, ay, az);
  let resultAngular = { x: ax, y: ay, z: az };
  if (angularMag > limits.angularMax) {
    resultAngular = scaleVector(ax, ay, az, limits.angularMax);
    reductions.push(
      `angular velocity ${angularMag.toFixed(2)} rad/s clamped to ${limits.angularMax} rad/s`,
    );
  }

  const clamped = reductions.length > 0;

  const result: VelocityVector = {
    linear: resultLinear,
    angular: resultAngular,
  };

  return { clamped, original, result, reductions };
}

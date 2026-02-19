import { describe, it, expect } from 'vitest';
import {
  clampVelocity,
  vectorMagnitude,
  scaleVector,
  type VelocityVector,
  type ClampResult,
} from './velocity-clamp.js';
import type { VelocityLimits } from './types.js';

const defaultLimits: VelocityLimits = {
  linearMax: 0.5,
  angularMax: 1.5,
  clampMode: true,
};

describe('vectorMagnitude', () => {
  it('returns 0 for zero vector', () => {
    expect(vectorMagnitude(0, 0, 0)).toBe(0);
  });

  it('returns correct magnitude for single-axis vector', () => {
    expect(vectorMagnitude(3, 0, 0)).toBe(3);
    expect(vectorMagnitude(0, -4, 0)).toBe(4);
    expect(vectorMagnitude(0, 0, 5)).toBe(5);
  });

  it('returns correct magnitude for 3-4-5 triangle', () => {
    // sqrt(3^2 + 4^2) = 5
    expect(vectorMagnitude(3, 4, 0)).toBe(5);
  });

  it('returns correct magnitude for 3D vector', () => {
    // sqrt(1^2 + 2^2 + 2^2) = sqrt(9) = 3
    expect(vectorMagnitude(1, 2, 2)).toBe(3);
  });

  it('handles negative values correctly', () => {
    expect(vectorMagnitude(-3, -4, 0)).toBe(5);
    expect(vectorMagnitude(-1, -2, -2)).toBe(3);
  });
});

describe('scaleVector', () => {
  it('returns original vector when magnitude is within limit', () => {
    const result = scaleVector(0.3, 0.4, 0, 1.0);
    expect(result.x).toBe(0.3);
    expect(result.y).toBe(0.4);
    expect(result.z).toBe(0);
  });

  it('returns original vector when magnitude equals limit', () => {
    const result = scaleVector(3, 4, 0, 5);
    expect(result.x).toBe(3);
    expect(result.y).toBe(4);
    expect(result.z).toBe(0);
  });

  it('scales down vector exceeding limit while preserving direction', () => {
    // magnitude = 10, max = 5, scale = 0.5
    const result = scaleVector(6, 8, 0, 5);
    expect(result.x).toBeCloseTo(3, 10);
    expect(result.y).toBeCloseTo(4, 10);
    expect(result.z).toBe(0);
    const mag = vectorMagnitude(result.x, result.y, result.z);
    expect(mag).toBeCloseTo(5, 10);
  });

  it('preserves direction ratio after scaling', () => {
    const result = scaleVector(6, 8, 0, 5);
    // direction ratio x:y should be 6:8 = 3:4
    expect(result.x / result.y).toBeCloseTo(6 / 8, 10);
  });

  it('returns zero vector unchanged', () => {
    const result = scaleVector(0, 0, 0, 5);
    expect(result.x).toBe(0);
    expect(result.y).toBe(0);
    expect(result.z).toBe(0);
  });

  it('handles negative components correctly', () => {
    // magnitude = 5, max = 2.5, scale = 0.5
    const result = scaleVector(-3, 4, 0, 2.5);
    expect(result.x).toBeCloseTo(-1.5, 10);
    expect(result.y).toBeCloseTo(2.0, 10);
    expect(result.z).toBe(0);
    const mag = vectorMagnitude(result.x, result.y, result.z);
    expect(mag).toBeCloseTo(2.5, 10);
  });

  it('handles 3D vector scaling', () => {
    // magnitude = sqrt(1+4+4) = 3, max = 1.5
    const result = scaleVector(1, 2, 2, 1.5);
    const mag = vectorMagnitude(result.x, result.y, result.z);
    expect(mag).toBeCloseTo(1.5, 10);
    // direction preserved: ratios 1:2:2
    expect(result.y / result.x).toBeCloseTo(2, 10);
    expect(result.z / result.x).toBeCloseTo(2, 10);
  });
});

describe('clampVelocity', () => {
  it('returns unclamped when velocity is within limits', () => {
    const message = {
      linear: { x: 0.1, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0.5 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(false);
    expect(result.reductions).toHaveLength(0);
    expect(result.result.linear.x).toBe(0.1);
    expect(result.result.angular.z).toBe(0.5);
  });

  it('clamps linear velocity over limit proportionally', () => {
    const message = {
      linear: { x: 2.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    const linearMag = vectorMagnitude(
      result.result.linear.x,
      result.result.linear.y,
      result.result.linear.z,
    );
    expect(linearMag).toBeCloseTo(0.5, 10);
    // Single axis: should be exactly 0.5
    expect(result.result.linear.x).toBeCloseTo(0.5, 10);
  });

  it('clamps angular velocity over limit proportionally', () => {
    const message = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 5.0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    const angularMag = vectorMagnitude(
      result.result.angular.x,
      result.result.angular.y,
      result.result.angular.z,
    );
    expect(angularMag).toBeCloseTo(1.5, 10);
    expect(result.result.angular.z).toBeCloseTo(1.5, 10);
  });

  it('clamps both linear and angular when both exceed limits', () => {
    const message = {
      linear: { x: 2.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 5.0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    expect(result.reductions).toHaveLength(2);

    const linearMag = vectorMagnitude(
      result.result.linear.x,
      result.result.linear.y,
      result.result.linear.z,
    );
    const angularMag = vectorMagnitude(
      result.result.angular.x,
      result.result.angular.y,
      result.result.angular.z,
    );
    expect(linearMag).toBeCloseTo(0.5, 10);
    expect(angularMag).toBeCloseTo(1.5, 10);
  });

  it('returns zero for zero velocity input', () => {
    const message = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(false);
    expect(result.result.linear).toEqual({ x: 0, y: 0, z: 0 });
    expect(result.result.angular).toEqual({ x: 0, y: 0, z: 0 });
    expect(result.reductions).toHaveLength(0);
  });

  it('clamps single-axis linear velocity', () => {
    const message = {
      linear: { x: 1.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    expect(result.result.linear.x).toBeCloseTo(0.5, 10);
    expect(result.result.linear.y).toBe(0);
    expect(result.result.linear.z).toBe(0);
  });

  it('clamps multi-axis linear (diagonal movement) preserving direction', () => {
    // magnitude = 5, limit = 0.5
    const message = {
      linear: { x: 3.0, y: 4.0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    const linearMag = vectorMagnitude(
      result.result.linear.x,
      result.result.linear.y,
      result.result.linear.z,
    );
    expect(linearMag).toBeCloseTo(0.5, 10);

    // Direction preserved: 3:4 ratio
    expect(result.result.linear.x / result.result.linear.y).toBeCloseTo(3 / 4, 10);
  });

  it('handles negative velocities correctly', () => {
    const message = {
      linear: { x: -2.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: -5.0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    // Linear: should be -0.5 (direction preserved)
    expect(result.result.linear.x).toBeCloseTo(-0.5, 10);
    // Angular: should be -1.5 (direction preserved)
    expect(result.result.angular.z).toBeCloseTo(-1.5, 10);
  });

  it('handles mixed positive/negative multi-axis velocities', () => {
    // magnitude = sqrt(9+16) = 5, limit = 0.5
    const message = {
      linear: { x: -3.0, y: 4.0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    const mag = vectorMagnitude(
      result.result.linear.x,
      result.result.linear.y,
      result.result.linear.z,
    );
    expect(mag).toBeCloseTo(0.5, 10);
    // Sign is preserved
    expect(result.result.linear.x).toBeLessThan(0);
    expect(result.result.linear.y).toBeGreaterThan(0);
  });

  it('preserves original values in the result', () => {
    const message = {
      linear: { x: 2.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 5.0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.original.linear.x).toBe(2.0);
    expect(result.original.angular.z).toBe(5.0);
  });

  it('lists reductions for linear clamping', () => {
    const message = {
      linear: { x: 2.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.reductions).toHaveLength(1);
    expect(result.reductions[0]).toContain('linear velocity');
    expect(result.reductions[0]).toContain('clamped to');
    expect(result.reductions[0]).toContain('0.5');
  });

  it('lists reductions for angular clamping', () => {
    const message = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 5.0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.reductions).toHaveLength(1);
    expect(result.reductions[0]).toContain('angular velocity');
    expect(result.reductions[0]).toContain('clamped to');
    expect(result.reductions[0]).toContain('1.5');
  });

  it('lists both reductions when both are clamped', () => {
    const message = {
      linear: { x: 2.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 5.0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.reductions).toHaveLength(2);
    expect(result.reductions[0]).toContain('linear');
    expect(result.reductions[1]).toContain('angular');
  });

  it('does not clamp when velocity is exactly at the limit', () => {
    const message = {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 1.5 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(false);
    expect(result.reductions).toHaveLength(0);
    expect(result.result.linear.x).toBe(0.5);
    expect(result.result.angular.z).toBe(1.5);
  });

  it('handles message with missing linear/angular fields', () => {
    const message = {};
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(false);
    expect(result.result.linear).toEqual({ x: 0, y: 0, z: 0 });
    expect(result.result.angular).toEqual({ x: 0, y: 0, z: 0 });
  });

  it('clamps 3D linear velocity correctly', () => {
    // magnitude = sqrt(1+4+4) = 3, limit = 0.5
    const message = {
      linear: { x: 1, y: 2, z: 2 },
      angular: { x: 0, y: 0, z: 0 },
    };
    const result = clampVelocity(message, defaultLimits);

    expect(result.clamped).toBe(true);
    const mag = vectorMagnitude(
      result.result.linear.x,
      result.result.linear.y,
      result.result.linear.z,
    );
    expect(mag).toBeCloseTo(0.5, 10);
    // Ratios 1:2:2 preserved
    expect(result.result.linear.y / result.result.linear.x).toBeCloseTo(2, 10);
    expect(result.result.linear.z / result.result.linear.x).toBeCloseTo(2, 10);
  });

  it('works with custom limits', () => {
    const customLimits: VelocityLimits = {
      linearMax: 2.0,
      angularMax: 3.0,
      clampMode: true,
    };
    const message = {
      linear: { x: 4.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 6.0 },
    };
    const result = clampVelocity(message, customLimits);

    expect(result.clamped).toBe(true);
    expect(result.result.linear.x).toBeCloseTo(2.0, 10);
    expect(result.result.angular.z).toBeCloseTo(3.0, 10);
  });
});

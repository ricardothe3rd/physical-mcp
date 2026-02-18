import { describe, it, expect } from 'vitest';
import { checkGeofence, isInsideGeofence, checkCircularGeofence, isInsideCircularGeofence, type Position, type CircularGeofence } from './geofence.js';
import type { GeofenceBounds } from './types.js';

const bounds: GeofenceBounds = {
  xMin: -5, xMax: 5,
  yMin: -5, yMax: 5,
  zMin: 0, zMax: 2,
};

describe('checkGeofence', () => {
  it('returns null for position inside bounds', () => {
    const pos: Position = { x: 0, y: 0, z: 1 };
    expect(checkGeofence(pos, bounds)).toBeNull();
  });

  it('returns violation for x below min', () => {
    const pos: Position = { x: -6, y: 0, z: 1 };
    const result = checkGeofence(pos, bounds);
    expect(result).not.toBeNull();
    expect(result!.type).toBe('geofence_violation');
    expect(result!.message).toContain('x=-6');
  });

  it('returns violation for x above max', () => {
    const pos: Position = { x: 6, y: 0, z: 1 };
    const result = checkGeofence(pos, bounds);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('x=6');
  });

  it('returns violation for y below min', () => {
    const pos: Position = { x: 0, y: -10, z: 1 };
    const result = checkGeofence(pos, bounds);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('y=-10');
  });

  it('returns violation for y above max', () => {
    const pos: Position = { x: 0, y: 8, z: 1 };
    const result = checkGeofence(pos, bounds);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('y=8');
  });

  it('returns violation for z below min', () => {
    const pos: Position = { x: 0, y: 0, z: -1 };
    const result = checkGeofence(pos, bounds);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=-1');
  });

  it('returns violation for z above max', () => {
    const pos: Position = { x: 0, y: 0, z: 5 };
    const result = checkGeofence(pos, bounds);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=5');
  });

  it('reports multiple axis violations at once', () => {
    const pos: Position = { x: -10, y: 10, z: -1 };
    const result = checkGeofence(pos, bounds);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('x=-10');
    expect(result!.message).toContain('y=10');
    expect(result!.message).toContain('z=-1');
  });

  it('allows positions on exact boundary (min)', () => {
    const pos: Position = { x: -5, y: -5, z: 0 };
    expect(checkGeofence(pos, bounds)).toBeNull();
  });

  it('allows positions on exact boundary (max)', () => {
    const pos: Position = { x: 5, y: 5, z: 2 };
    expect(checkGeofence(pos, bounds)).toBeNull();
  });

  it('includes position and bounds in violation details', () => {
    const pos: Position = { x: -10, y: 0, z: 1 };
    const result = checkGeofence(pos, bounds);
    expect(result!.details).toHaveProperty('position');
    expect(result!.details).toHaveProperty('bounds');
    expect(result!.details).toHaveProperty('violations');
  });
});

describe('isInsideGeofence', () => {
  it('returns true for position inside', () => {
    expect(isInsideGeofence({ x: 0, y: 0, z: 1 }, bounds)).toBe(true);
  });

  it('returns false for position outside', () => {
    expect(isInsideGeofence({ x: 100, y: 0, z: 1 }, bounds)).toBe(false);
  });
});

// --- Circular geofence tests ---

const circle: CircularGeofence = {
  centerX: 0,
  centerY: 0,
  radius: 5,
  zMin: 0,
  zMax: 2,
};

describe('checkCircularGeofence', () => {
  it('returns null for position at center', () => {
    expect(checkCircularGeofence({ x: 0, y: 0, z: 1 }, circle)).toBeNull();
  });

  it('returns null for position inside radius', () => {
    expect(checkCircularGeofence({ x: 3, y: 3, z: 1 }, circle)).toBeNull();
  });

  it('returns null for position on exact radius', () => {
    expect(checkCircularGeofence({ x: 5, y: 0, z: 1 }, circle)).toBeNull();
  });

  it('returns violation for position outside radius', () => {
    const result = checkCircularGeofence({ x: 6, y: 0, z: 1 }, circle);
    expect(result).not.toBeNull();
    expect(result!.type).toBe('geofence_violation');
    expect(result!.message).toContain('circular geofence');
    expect(result!.message).toContain('distance');
  });

  it('checks diagonal distance correctly', () => {
    // sqrt(4^2 + 4^2) = sqrt(32) ≈ 5.66 > 5
    const result = checkCircularGeofence({ x: 4, y: 4, z: 1 }, circle);
    expect(result).not.toBeNull();
  });

  it('returns violation for z outside bounds', () => {
    const result = checkCircularGeofence({ x: 0, y: 0, z: 5 }, circle);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=5');
  });

  it('reports both distance and z violations', () => {
    const result = checkCircularGeofence({ x: 10, y: 0, z: -1 }, circle);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('distance');
    expect(result!.message).toContain('z=-1');
  });

  it('works with offset center', () => {
    const offsetCircle: CircularGeofence = {
      centerX: 10,
      centerY: 10,
      radius: 3,
      zMin: 0,
      zMax: 2,
    };
    // Position at (10, 10) is at center
    expect(checkCircularGeofence({ x: 10, y: 10, z: 1 }, offsetCircle)).toBeNull();
    // Position at (14, 10) is 4m away > 3m radius
    expect(checkCircularGeofence({ x: 14, y: 10, z: 1 }, offsetCircle)).not.toBeNull();
  });

  it('includes distance in violation details', () => {
    const result = checkCircularGeofence({ x: 10, y: 0, z: 1 }, circle);
    expect(result!.details).toHaveProperty('distance');
    expect(result!.details.distance).toBeCloseTo(10, 1);
  });
});

describe('isInsideCircularGeofence', () => {
  it('returns true for position inside', () => {
    expect(isInsideCircularGeofence({ x: 0, y: 0, z: 1 }, circle)).toBe(true);
  });

  it('returns false for position outside', () => {
    expect(isInsideCircularGeofence({ x: 100, y: 0, z: 1 }, circle)).toBe(false);
  });
});

import { describe, it, expect } from 'vitest';
import { checkGeofence, isInsideGeofence, type Position } from './geofence.js';
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

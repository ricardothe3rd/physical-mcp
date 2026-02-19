import { describe, it, expect } from 'vitest';
import { checkGeofenceShape, isPointInPolygon, type GeofenceShape } from './geofence-shapes.js';
import type { Position } from './geofence.js';
import type { GeofenceBounds } from './types.js';

// ─── Rectangle shape tests ─────────────────────────────────────────────────────

describe('checkGeofenceShape — rectangle', () => {
  const rectShape: GeofenceShape = {
    type: 'rectangle',
    bounds: { xMin: -5, xMax: 5, yMin: -5, yMax: 5, zMin: 0, zMax: 2 },
  };

  it('returns null for position inside rectangular bounds', () => {
    const pos: Position = { x: 0, y: 0, z: 1 };
    expect(checkGeofenceShape(pos, rectShape)).toBeNull();
  });

  it('returns violation for position outside rectangular bounds (x)', () => {
    const pos: Position = { x: 10, y: 0, z: 1 };
    const result = checkGeofenceShape(pos, rectShape);
    expect(result).not.toBeNull();
    expect(result!.type).toBe('geofence_violation');
    expect(result!.message).toContain('x=10');
  });

  it('returns violation for position outside rectangular bounds (z)', () => {
    const pos: Position = { x: 0, y: 0, z: 5 };
    const result = checkGeofenceShape(pos, rectShape);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=5');
  });

  it('allows position on exact boundary', () => {
    const pos: Position = { x: 5, y: -5, z: 0 };
    expect(checkGeofenceShape(pos, rectShape)).toBeNull();
  });
});

// ─── Circle shape tests ────────────────────────────────────────────────────────

describe('checkGeofenceShape — circle', () => {
  const circleShape: GeofenceShape = {
    type: 'circle',
    center: { x: 0, y: 0 },
    radius: 5,
    zMin: 0,
    zMax: 3,
  };

  it('returns null for position at center', () => {
    expect(checkGeofenceShape({ x: 0, y: 0, z: 1 }, circleShape)).toBeNull();
  });

  it('returns null for position inside radius', () => {
    expect(checkGeofenceShape({ x: 3, y: 0, z: 1 }, circleShape)).toBeNull();
  });

  it('returns null for position on exact radius boundary', () => {
    expect(checkGeofenceShape({ x: 5, y: 0, z: 1 }, circleShape)).toBeNull();
  });

  it('returns violation for position outside radius', () => {
    const result = checkGeofenceShape({ x: 6, y: 0, z: 1 }, circleShape);
    expect(result).not.toBeNull();
    expect(result!.type).toBe('geofence_violation');
    expect(result!.message).toContain('circular geofence');
    expect(result!.message).toContain('distance');
  });

  it('correctly checks diagonal distance', () => {
    // sqrt(4^2 + 4^2) = sqrt(32) ≈ 5.66 > 5
    const result = checkGeofenceShape({ x: 4, y: 4, z: 1 }, circleShape);
    expect(result).not.toBeNull();
  });

  it('returns violation when z is below zMin', () => {
    const result = checkGeofenceShape({ x: 0, y: 0, z: -1 }, circleShape);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=-1');
  });

  it('returns violation when z is above zMax', () => {
    const result = checkGeofenceShape({ x: 0, y: 0, z: 5 }, circleShape);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=5');
  });

  it('reports both distance and z violations', () => {
    const result = checkGeofenceShape({ x: 10, y: 0, z: -1 }, circleShape);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('distance');
    expect(result!.message).toContain('z=-1');
  });

  it('works with an offset center', () => {
    const offsetCircle: GeofenceShape = {
      type: 'circle',
      center: { x: 10, y: 10 },
      radius: 3,
      zMin: 0,
      zMax: 2,
    };
    // At center
    expect(checkGeofenceShape({ x: 10, y: 10, z: 1 }, offsetCircle)).toBeNull();
    // 4m away > 3m radius
    expect(checkGeofenceShape({ x: 14, y: 10, z: 1 }, offsetCircle)).not.toBeNull();
    // Just inside (2m away < 3m radius)
    expect(checkGeofenceShape({ x: 12, y: 10, z: 1 }, offsetCircle)).toBeNull();
  });

  it('includes distance in violation details', () => {
    const result = checkGeofenceShape({ x: 10, y: 0, z: 1 }, circleShape);
    expect(result).not.toBeNull();
    expect(result!.details).toHaveProperty('distance');
    expect(result!.details.distance).toBeCloseTo(10, 1);
  });
});

// ─── Polygon shape tests ───────────────────────────────────────────────────────

describe('checkGeofenceShape — polygon', () => {
  // Simple triangle: (0,0), (10,0), (5,10)
  const triangle: GeofenceShape = {
    type: 'polygon',
    vertices: [
      { x: 0, y: 0 },
      { x: 10, y: 0 },
      { x: 5, y: 10 },
    ],
    zMin: 0,
    zMax: 2,
  };

  // Unit square: (0,0), (10,0), (10,10), (0,10)
  const square: GeofenceShape = {
    type: 'polygon',
    vertices: [
      { x: 0, y: 0 },
      { x: 10, y: 0 },
      { x: 10, y: 10 },
      { x: 0, y: 10 },
    ],
    zMin: 0,
    zMax: 5,
  };

  // L-shape (concave polygon):
  //  (0,0) -> (6,0) -> (6,3) -> (3,3) -> (3,6) -> (0,6)
  const lShape: GeofenceShape = {
    type: 'polygon',
    vertices: [
      { x: 0, y: 0 },
      { x: 6, y: 0 },
      { x: 6, y: 3 },
      { x: 3, y: 3 },
      { x: 3, y: 6 },
      { x: 0, y: 6 },
    ],
    zMin: 0,
    zMax: 2,
  };

  it('returns null for position inside a triangle', () => {
    // Centroid of the triangle
    expect(checkGeofenceShape({ x: 5, y: 3, z: 1 }, triangle)).toBeNull();
  });

  it('returns violation for position outside a triangle', () => {
    const result = checkGeofenceShape({ x: 0, y: 10, z: 1 }, triangle);
    expect(result).not.toBeNull();
    expect(result!.type).toBe('geofence_violation');
    expect(result!.message).toContain('polygon geofence');
  });

  it('returns null for position inside a square', () => {
    expect(checkGeofenceShape({ x: 5, y: 5, z: 2 }, square)).toBeNull();
  });

  it('returns violation for position outside a square', () => {
    const result = checkGeofenceShape({ x: 15, y: 5, z: 2 }, square);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('polygon geofence');
  });

  it('returns null inside the bottom-left portion of L-shape', () => {
    // Inside the lower horizontal part
    expect(checkGeofenceShape({ x: 1, y: 1, z: 1 }, lShape)).toBeNull();
  });

  it('returns null inside the upper-left portion of L-shape', () => {
    // Inside the left vertical part
    expect(checkGeofenceShape({ x: 1, y: 5, z: 1 }, lShape)).toBeNull();
  });

  it('returns violation for position in the concave notch of L-shape', () => {
    // The point (5, 5) is in the upper-right area which is the notch
    const result = checkGeofenceShape({ x: 5, y: 5, z: 1 }, lShape);
    expect(result).not.toBeNull();
  });

  it('returns violation when z is outside polygon zMin', () => {
    const result = checkGeofenceShape({ x: 5, y: 3, z: -1 }, triangle);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=-1');
  });

  it('returns violation when z is outside polygon zMax', () => {
    const result = checkGeofenceShape({ x: 5, y: 3, z: 10 }, triangle);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('z=10');
  });

  it('reports both polygon boundary and z violations', () => {
    const result = checkGeofenceShape({ x: -5, y: -5, z: -1 }, triangle);
    expect(result).not.toBeNull();
    expect(result!.message).toContain('outside polygon boundary');
    expect(result!.message).toContain('z=-1');
  });

  it('returns null for position on exact z boundary', () => {
    expect(checkGeofenceShape({ x: 5, y: 3, z: 0 }, triangle)).toBeNull();
    expect(checkGeofenceShape({ x: 5, y: 3, z: 2 }, triangle)).toBeNull();
  });
});

// ─── isPointInPolygon tests ────────────────────────────────────────────────────

describe('isPointInPolygon', () => {
  // Triangle: (0,0), (10,0), (5,10)
  const triangle = [
    { x: 0, y: 0 },
    { x: 10, y: 0 },
    { x: 5, y: 10 },
  ];

  // Square: (0,0), (10,0), (10,10), (0,10)
  const square = [
    { x: 0, y: 0 },
    { x: 10, y: 0 },
    { x: 10, y: 10 },
    { x: 0, y: 10 },
  ];

  // Concave arrow-shaped polygon
  // Looks like a chevron pointing right: (0,0) (5,5) (0,10) (3,5)
  const concave = [
    { x: 0, y: 0 },
    { x: 5, y: 5 },
    { x: 0, y: 10 },
    { x: 3, y: 5 },
  ];

  it('detects point inside triangle', () => {
    expect(isPointInPolygon(5, 3, triangle)).toBe(true);
  });

  it('detects point outside triangle', () => {
    expect(isPointInPolygon(0, 10, triangle)).toBe(false);
  });

  it('detects point far outside triangle', () => {
    expect(isPointInPolygon(-10, -10, triangle)).toBe(false);
  });

  it('detects point inside square', () => {
    expect(isPointInPolygon(5, 5, square)).toBe(true);
  });

  it('detects point outside square (right)', () => {
    expect(isPointInPolygon(15, 5, square)).toBe(false);
  });

  it('detects point outside square (above)', () => {
    expect(isPointInPolygon(5, 15, square)).toBe(false);
  });

  it('detects point just inside square near corner', () => {
    expect(isPointInPolygon(0.1, 0.1, square)).toBe(true);
  });

  it('detects point inside concave polygon (in the wider area)', () => {
    // Near the tip at (5, 5), slightly left
    expect(isPointInPolygon(4, 5, concave)).toBe(true);
  });

  it('detects point outside concave polygon (in the concavity)', () => {
    // Point at (1, 5) is in the concave indent area between (3,5) and the left edges
    expect(isPointInPolygon(1, 5, concave)).toBe(false);
  });

  it('detects point well outside concave polygon', () => {
    expect(isPointInPolygon(10, 10, concave)).toBe(false);
  });

  it('returns false for degenerate polygon with fewer than 3 vertices', () => {
    expect(isPointInPolygon(0, 0, [])).toBe(false);
    expect(isPointInPolygon(0, 0, [{ x: 0, y: 0 }])).toBe(false);
    expect(isPointInPolygon(0, 0, [{ x: 0, y: 0 }, { x: 1, y: 1 }])).toBe(false);
  });

  it('handles large polygon correctly', () => {
    // Diamond shape centered at origin: (100,0) (0,100) (-100,0) (0,-100)
    const diamond = [
      { x: 100, y: 0 },
      { x: 0, y: 100 },
      { x: -100, y: 0 },
      { x: 0, y: -100 },
    ];
    expect(isPointInPolygon(0, 0, diamond)).toBe(true);
    // (49, 49) -> |49|+|49| = 98 < 100, inside diamond
    expect(isPointInPolygon(49, 49, diamond)).toBe(true);
    // (25, 25) clearly inside
    expect(isPointInPolygon(25, 25, diamond)).toBe(true);
    // (51, 51) -> |51|+|51| = 102 > 100, outside diamond
    expect(isPointInPolygon(51, 51, diamond)).toBe(false);
    expect(isPointInPolygon(200, 200, diamond)).toBe(false);
  });
});

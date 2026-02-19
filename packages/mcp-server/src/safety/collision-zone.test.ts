import { describe, it, expect, beforeEach } from 'vitest';
import {
  CollisionZoneManager,
  type CollisionZone,
  type CollisionCheckResult,
} from './collision-zone.js';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function circleZone(overrides: Partial<CollisionZone> = {}): CollisionZone {
  return {
    id: 'c1',
    name: 'Pillar',
    shape: 'circle',
    center: { x: 0, y: 0 },
    radius: 1,
    warningDistance: 3,
    blockDistance: 1,
    ...overrides,
  };
}

function rectZone(overrides: Partial<CollisionZone> = {}): CollisionZone {
  return {
    id: 'r1',
    name: 'Table',
    shape: 'rectangle',
    center: { x: 5, y: 5 },
    width: 4,
    height: 2,
    warningDistance: 2,
    blockDistance: 0.5,
    ...overrides,
  };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('CollisionZoneManager', () => {
  let manager: CollisionZoneManager;

  beforeEach(() => {
    manager = new CollisionZoneManager();
  });

  // ---- Zone lifecycle ----

  it('addZone / removeZone lifecycle', () => {
    const zone = circleZone();
    manager.addZone(zone);
    expect(manager.getZoneCount()).toBe(1);

    const removed = manager.removeZone(zone.id);
    expect(removed).toBe(true);
    expect(manager.getZoneCount()).toBe(0);
  });

  it('removeZone returns false for non-existing zone', () => {
    expect(manager.removeZone('nonexistent')).toBe(false);
  });

  it('listZones returns all added zones', () => {
    const c = circleZone();
    const r = rectZone();
    manager.addZone(c);
    manager.addZone(r);

    const listed = manager.listZones();
    expect(listed).toHaveLength(2);
    expect(listed.map((z) => z.id).sort()).toEqual(['c1', 'r1']);
  });

  it('getZoneCount reflects current count', () => {
    expect(manager.getZoneCount()).toBe(0);
    manager.addZone(circleZone());
    expect(manager.getZoneCount()).toBe(1);
    manager.addZone(rectZone());
    expect(manager.getZoneCount()).toBe(2);
  });

  it('clearZones empties the manager', () => {
    manager.addZone(circleZone());
    manager.addZone(rectZone());
    expect(manager.getZoneCount()).toBe(2);

    manager.clearZones();
    expect(manager.getZoneCount()).toBe(0);
    expect(manager.listZones()).toEqual([]);
  });

  // ---- Position checks: clear ----

  it('checkPosition far from all zones returns clear', () => {
    manager.addZone(circleZone()); // circle at origin, radius 1, warn 3, block 1
    // Point at (100, 100) is far away.
    const result = manager.checkPosition(100, 100);
    expect(result.status).toBe('clear');
    expect(result.zones).toHaveLength(1);
    expect(result.zones[0].status).toBe('clear');
  });

  it('checkPosition with no zones returns clear with null closestZone', () => {
    const result = manager.checkPosition(0, 0);
    expect(result.status).toBe('clear');
    expect(result.zones).toHaveLength(0);
    expect(result.closestZone).toBeNull();
  });

  // ---- Position checks: warning ----

  it('checkPosition in warning distance returns warning (circle)', () => {
    // circle at origin, radius 1, warnDist 3, blockDist 1
    // Point at (3, 0) => distance to boundary = sqrt(9) - 1 = 2
    // 2 <= warningDistance(3) but 2 > blockDistance(1) => warning
    manager.addZone(circleZone());
    const result = manager.checkPosition(3, 0);
    expect(result.status).toBe('warning');
    expect(result.zones[0].status).toBe('warning');
  });

  it('checkPosition in warning distance returns warning (rectangle)', () => {
    // rect center (5,5), width 4 (x: 3..7), height 2 (y: 4..6)
    // warnDist 2, blockDist 0.5
    // Point at (8, 5) => distance to nearest edge = 8 - 7 = 1
    // 1 <= warnDist(2) but 1 > blockDist(0.5) => warning
    manager.addZone(rectZone());
    const result = manager.checkPosition(8, 5);
    expect(result.status).toBe('warning');
    expect(result.zones[0].status).toBe('warning');
  });

  // ---- Position checks: blocked ----

  it('checkPosition in block distance returns blocked (circle)', () => {
    // circle at origin, radius 1, blockDist 1
    // Point at (1.5, 0) => distance = 1.5 - 1 = 0.5
    // 0.5 <= blockDistance(1) => blocked
    manager.addZone(circleZone());
    const result = manager.checkPosition(1.5, 0);
    expect(result.status).toBe('blocked');
    expect(result.zones[0].status).toBe('blocked');
  });

  it('checkPosition in block distance returns blocked (rectangle)', () => {
    // rect center (5,5), width 4 (x: 3..7), height 2 (y: 4..6)
    // blockDist 0.5
    // Point at (7.3, 5) => distance to nearest edge = 7.3 - 7 = 0.3
    // 0.3 <= blockDist(0.5) => blocked
    manager.addZone(rectZone());
    const result = manager.checkPosition(7.3, 5);
    expect(result.status).toBe('blocked');
    expect(result.zones[0].status).toBe('blocked');
  });

  // ---- Inside a zone ----

  it('position inside a circular zone (negative distance) returns blocked', () => {
    // circle at origin, radius 1
    // Point at (0, 0) => distance = 0 - 1 = -1 (inside)
    manager.addZone(circleZone());
    const result = manager.checkPosition(0, 0);
    expect(result.status).toBe('blocked');
    expect(result.zones[0].distance).toBeLessThan(0);
    expect(result.zones[0].status).toBe('blocked');
  });

  it('position inside a rectangular zone (negative distance) returns blocked', () => {
    // rect center (5,5), width 4 (x: 3..7), height 2 (y: 4..6)
    // Point at (5, 5) is at the center => inside, negative distance
    manager.addZone(rectZone());
    const result = manager.checkPosition(5, 5);
    expect(result.status).toBe('blocked');
    expect(result.zones[0].distance).toBeLessThan(0);
    expect(result.zones[0].status).toBe('blocked');
  });

  // ---- Circular distance calculation ----

  it('circular zone distance calculation is correct', () => {
    // circle at (10, 0), radius 5
    // Point at (20, 0) => distance = 10 - 5 = 5
    manager.addZone(
      circleZone({
        id: 'c2',
        center: { x: 10, y: 0 },
        radius: 5,
        warningDistance: 10,
        blockDistance: 2,
      }),
    );
    const result = manager.checkPosition(20, 0);
    expect(result.zones[0].distance).toBeCloseTo(5, 5);
  });

  // ---- Rectangular distance calculation ----

  it('rectangular zone distance calculation is correct (side region)', () => {
    // rect center (0, 0), width 6 (x: -3..3), height 4 (y: -2..2)
    // Point at (5, 0) => distance = 5 - 3 = 2 (nearest edge is right side)
    manager.addZone(
      rectZone({
        id: 'r2',
        center: { x: 0, y: 0 },
        width: 6,
        height: 4,
        warningDistance: 10,
        blockDistance: 1,
      }),
    );
    const result = manager.checkPosition(5, 0);
    expect(result.zones[0].distance).toBeCloseTo(2, 5);
  });

  it('rectangular zone distance calculation is correct (corner region)', () => {
    // rect center (0, 0), width 6 (x: -3..3), height 4 (y: -2..2)
    // Point at (6, 5) => outsideX = 6-3 = 3, outsideY = 5-2 = 3
    // distance = sqrt(9 + 9) = sqrt(18) ~ 4.2426
    manager.addZone(
      rectZone({
        id: 'r3',
        center: { x: 0, y: 0 },
        width: 6,
        height: 4,
        warningDistance: 10,
        blockDistance: 1,
      }),
    );
    const result = manager.checkPosition(6, 5);
    expect(result.zones[0].distance).toBeCloseTo(Math.sqrt(18), 5);
  });

  // ---- Multiple zones - closest reported ----

  it('multiple zones - closest one is reported in closestZone', () => {
    // Zone A: circle at (0,0), radius 1 => point (5,0) dist = 4
    // Zone B: circle at (10,0), radius 1 => point (5,0) dist = 4
    // Zone C: circle at (4,0), radius 1 => point (5,0) dist = 0 (on boundary)
    manager.addZone(
      circleZone({
        id: 'zA',
        name: 'Zone A',
        center: { x: 0, y: 0 },
        radius: 1,
        warningDistance: 10,
        blockDistance: 0,
      }),
    );
    manager.addZone(
      circleZone({
        id: 'zB',
        name: 'Zone B',
        center: { x: 10, y: 0 },
        radius: 1,
        warningDistance: 10,
        blockDistance: 0,
      }),
    );
    manager.addZone(
      circleZone({
        id: 'zC',
        name: 'Zone C',
        center: { x: 4, y: 0 },
        radius: 1,
        warningDistance: 10,
        blockDistance: 1,
      }),
    );

    const result = manager.checkPosition(5, 0);
    expect(result.closestZone).not.toBeNull();
    expect(result.closestZone!.zoneId).toBe('zC');
    expect(result.closestZone!.distance).toBeCloseTo(0, 5);
  });

  it('multiple zones - overall status is the worst of all zones', () => {
    // Zone A far away => clear
    // Zone B in warning range => warning
    // Zone C in block range => blocked
    // Overall should be blocked
    manager.addZone(
      circleZone({
        id: 'zA',
        center: { x: 100, y: 100 },
        radius: 1,
        warningDistance: 3,
        blockDistance: 1,
      }),
    );
    manager.addZone(
      circleZone({
        id: 'zB',
        center: { x: 5, y: 0 },
        radius: 1,
        warningDistance: 5,
        blockDistance: 0.5,
      }),
    );
    manager.addZone(
      circleZone({
        id: 'zC',
        center: { x: 0, y: 0 },
        radius: 1,
        warningDistance: 5,
        blockDistance: 2,
      }),
    );

    // Point at (2, 0): zC distance = 2-1=1 <= blockDist(2) => blocked
    const result = manager.checkPosition(2, 0);
    expect(result.status).toBe('blocked');
  });

  // ---- Edge cases ----

  it('point exactly at boundary of circle (distance 0) is blocked when blockDistance >= 0', () => {
    // circle at (0,0), radius 5, blockDistance 0
    // Point at (5,0) => distance = 0
    // 0 <= blockDistance(0) => blocked
    manager.addZone(
      circleZone({
        radius: 5,
        warningDistance: 3,
        blockDistance: 0,
      }),
    );
    const result = manager.checkPosition(5, 0);
    expect(result.zones[0].distance).toBeCloseTo(0, 5);
    expect(result.zones[0].status).toBe('blocked');
  });

  it('addZone with same id replaces the previous zone', () => {
    manager.addZone(circleZone({ id: 'dup', name: 'First' }));
    manager.addZone(circleZone({ id: 'dup', name: 'Second' }));
    expect(manager.getZoneCount()).toBe(1);
    expect(manager.listZones()[0].name).toBe('Second');
  });

  it('closestZone reports correct data with single zone', () => {
    manager.addZone(circleZone({ center: { x: 10, y: 0 }, radius: 2 }));
    const result = manager.checkPosition(0, 0);
    expect(result.closestZone).not.toBeNull();
    expect(result.closestZone!.zoneId).toBe('c1');
    // distance = 10 - 2 = 8
    expect(result.closestZone!.distance).toBeCloseTo(8, 5);
  });

  it('zone status entries include correct zoneName', () => {
    manager.addZone(circleZone({ name: 'MyPillar' }));
    const result = manager.checkPosition(100, 100);
    expect(result.zones[0].zoneName).toBe('MyPillar');
  });
});

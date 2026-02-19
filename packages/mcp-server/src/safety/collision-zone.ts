/**
 * Collision zone proximity warning module.
 * Manages named zones (circles and rectangles) with configurable warning
 * and block distances, and checks whether a 2D position is clear, within
 * a warning range, or blocked by any registered zone.
 */

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** A collision zone defined as either a circle or an axis-aligned rectangle. */
export interface CollisionZone {
  id: string;
  name: string;
  shape: 'circle' | 'rectangle';
  center: { x: number; y: number };
  /** Radius in meters (required when shape === 'circle'). */
  radius?: number;
  /** Width in meters along the x-axis (required when shape === 'rectangle'). */
  width?: number;
  /** Height in meters along the y-axis (required when shape === 'rectangle'). */
  height?: number;
  /** Meters - warn when closer than this distance to the zone boundary. */
  warningDistance: number;
  /** Meters - block movement when closer than this distance to the zone boundary. */
  blockDistance: number;
}

/** Per-zone status entry inside a CollisionCheckResult. */
export interface ZoneStatus {
  zoneId: string;
  zoneName: string;
  /** Distance from the queried point to the nearest boundary of the zone (meters). Negative means inside. */
  distance: number;
  status: 'clear' | 'warning' | 'blocked';
}

/** Aggregate result for a position check against all registered zones. */
export interface CollisionCheckResult {
  /** Overall status - the worst (most severe) of all individual zone statuses. */
  status: 'clear' | 'warning' | 'blocked';
  /** Per-zone breakdown. */
  zones: ZoneStatus[];
  /** The single closest zone, or null when no zones are registered. */
  closestZone: { zoneId: string; distance: number } | null;
}

// ---------------------------------------------------------------------------
// Distance helpers
// ---------------------------------------------------------------------------

/**
 * Compute the signed distance from a point (px, py) to the boundary of a
 * circular zone.  Positive means outside the circle; negative means inside.
 */
function distanceToCircle(px: number, py: number, zone: CollisionZone): number {
  const dx = px - zone.center.x;
  const dy = py - zone.center.y;
  const distToCenter = Math.sqrt(dx * dx + dy * dy);
  return distToCenter - (zone.radius ?? 0);
}

/**
 * Compute the signed distance from a point (px, py) to the nearest edge of
 * an axis-aligned rectangle defined by center, width, and height.
 * Positive means outside the rectangle; negative means inside.
 */
function distanceToRectangle(px: number, py: number, zone: CollisionZone): number {
  const halfW = (zone.width ?? 0) / 2;
  const halfH = (zone.height ?? 0) / 2;

  // Distances from the point to each edge (positive = outside that edge).
  const dxMin = zone.center.x - halfW - px; // left edge
  const dxMax = px - (zone.center.x + halfW); // right edge
  const dyMin = zone.center.y - halfH - py; // bottom edge
  const dyMax = py - (zone.center.y + halfH); // top edge

  // For each axis, the penetration depth (negative when inside).
  const outsideX = Math.max(dxMin, dxMax);
  const outsideY = Math.max(dyMin, dyMax);

  if (outsideX > 0 && outsideY > 0) {
    // Point is outside in both axes (corner region) - Euclidean distance.
    return Math.sqrt(outsideX * outsideX + outsideY * outsideY);
  }
  if (outsideX > 0) return outsideX;
  if (outsideY > 0) return outsideY;

  // Inside the rectangle - distance is negative (closest edge).
  return Math.max(outsideX, outsideY);
}

// ---------------------------------------------------------------------------
// Manager
// ---------------------------------------------------------------------------

/**
 * Manages a collection of collision zones and checks arbitrary 2D positions
 * against them, returning aggregated warning/block statuses.
 */
export class CollisionZoneManager {
  private zones: Map<string, CollisionZone> = new Map();

  /** Register a new collision zone. */
  addZone(zone: CollisionZone): void {
    this.zones.set(zone.id, zone);
  }

  /** Remove a zone by id.  Returns true if it existed, false otherwise. */
  removeZone(id: string): boolean {
    return this.zones.delete(id);
  }

  /** Return all registered zones as an array. */
  listZones(): CollisionZone[] {
    return Array.from(this.zones.values());
  }

  /** Return the number of registered zones. */
  getZoneCount(): number {
    return this.zones.size;
  }

  /** Remove all zones. */
  clearZones(): void {
    this.zones.clear();
  }

  /**
   * Check a 2D position against every registered zone.
   * The overall status is the most severe individual status encountered
   * (blocked > warning > clear).
   */
  checkPosition(x: number, y: number): CollisionCheckResult {
    const zoneStatuses: ZoneStatus[] = [];
    let closestZone: { zoneId: string; distance: number } | null = null;
    let overallStatus: 'clear' | 'warning' | 'blocked' = 'clear';

    for (const zone of this.zones.values()) {
      const distance =
        zone.shape === 'circle'
          ? distanceToCircle(x, y, zone)
          : distanceToRectangle(x, y, zone);

      let status: 'clear' | 'warning' | 'blocked';
      if (distance <= zone.blockDistance) {
        status = 'blocked';
      } else if (distance <= zone.warningDistance) {
        status = 'warning';
      } else {
        status = 'clear';
      }

      zoneStatuses.push({
        zoneId: zone.id,
        zoneName: zone.name,
        distance,
        status,
      });

      // Track closest zone.
      if (closestZone === null || distance < closestZone.distance) {
        closestZone = { zoneId: zone.id, distance };
      }

      // Escalate overall status.
      if (status === 'blocked') {
        overallStatus = 'blocked';
      } else if (status === 'warning' && overallStatus !== 'blocked') {
        overallStatus = 'warning';
      }
    }

    return {
      status: overallStatus,
      zones: zoneStatuses,
      closestZone,
    };
  }
}

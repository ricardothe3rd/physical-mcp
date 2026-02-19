/**
 * Position tracking via odometry data for live geofence enforcement.
 *
 * Subscribes to odometry updates (e.g. /odom topic) and maintains a
 * ring buffer of recent positions. Enables real-time geofence checks
 * against the robot's *actual* position, not just commanded targets.
 */

export interface Position {
  x: number;
  y: number;
  z: number;
  timestamp: number;
}

export interface GeofenceBoundary {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

export interface GeofenceCheckResult {
  inside: boolean;
  distanceToBoundary: number; // Negative if outside
  nearestEdge: 'minX' | 'maxX' | 'minY' | 'maxY';
}

export interface PositionTrackerConfig {
  maxHistorySize?: number; // Default 100
  staleThresholdMs?: number; // Default 5000 (5 seconds)
  onPositionUpdate?: (pos: Position) => void;
  onGeofenceViolation?: (pos: Position, boundary: GeofenceBoundary) => void;
}

export class PositionTracker {
  private history: Position[] = [];
  private totalUpdates = 0;
  private readonly maxHistorySize: number;
  private readonly staleThresholdMs: number;
  private readonly onPositionUpdate?: (pos: Position) => void;
  private readonly onGeofenceViolation?: (pos: Position, boundary: GeofenceBoundary) => void;

  constructor(config?: PositionTrackerConfig) {
    this.maxHistorySize = config?.maxHistorySize ?? 100;
    this.staleThresholdMs = config?.staleThresholdMs ?? 5000;
    this.onPositionUpdate = config?.onPositionUpdate;
    this.onGeofenceViolation = config?.onGeofenceViolation;
  }

  /**
   * Update position from odometry data.
   * Called when an /odom message arrives.
   * @param x - X coordinate in meters
   * @param y - Y coordinate in meters
   * @param z - Z coordinate in meters (defaults to 0)
   */
  updatePosition(x: number, y: number, z = 0): void {
    const position: Position = {
      x,
      y,
      z,
      timestamp: Date.now(),
    };

    this.history.push(position);
    this.totalUpdates++;

    // Ring buffer: drop oldest when full
    if (this.history.length > this.maxHistorySize) {
      this.history = this.history.slice(this.history.length - this.maxHistorySize);
    }

    if (this.onPositionUpdate) {
      this.onPositionUpdate(position);
    }
  }

  /**
   * Get current position.
   * @returns The most recent position, or null if no data or data is stale.
   */
  getCurrentPosition(): Position | null {
    if (this.history.length === 0) {
      return null;
    }

    const latest = this.history[this.history.length - 1];

    if (this.isPositionStale(latest)) {
      return null;
    }

    return latest;
  }

  /**
   * Check if position data is stale (older than staleThresholdMs).
   * Returns true if no positions have been recorded.
   */
  isStale(): boolean {
    if (this.history.length === 0) {
      return true;
    }

    const latest = this.history[this.history.length - 1];
    return this.isPositionStale(latest);
  }

  /**
   * Get position history, most recent first.
   */
  getHistory(): Position[] {
    return [...this.history].reverse();
  }

  /**
   * Calculate total distance traveled from position history.
   * Sums Euclidean distances between consecutive points.
   */
  getDistanceTraveled(): number {
    if (this.history.length < 2) {
      return 0;
    }

    let total = 0;
    for (let i = 1; i < this.history.length; i++) {
      const prev = this.history[i - 1];
      const curr = this.history[i];
      total += this.euclideanDistance(prev, curr);
    }

    return total;
  }

  /**
   * Estimate current velocity from the two most recent positions (m/s).
   * @returns Velocity in m/s, or null if fewer than 2 positions exist
   *   or the time delta is zero.
   */
  getEstimatedVelocity(): number | null {
    if (this.history.length < 2) {
      return null;
    }

    const curr = this.history[this.history.length - 1];
    const prev = this.history[this.history.length - 2];
    const dt = (curr.timestamp - prev.timestamp) / 1000; // Convert ms to seconds

    if (dt === 0) {
      return null;
    }

    const distance = this.euclideanDistance(prev, curr);
    return distance / dt;
  }

  /**
   * Check if current position is within a 2D geofence boundary.
   * @param boundary - The rectangular boundary to check against
   * @returns Geofence check result with inside status, distance, and nearest edge
   */
  checkGeofence(boundary: GeofenceBoundary): GeofenceCheckResult {
    const pos = this.history.length > 0
      ? this.history[this.history.length - 1]
      : { x: 0, y: 0, z: 0, timestamp: 0 };

    // Compute signed distance to each edge (positive = inside, negative = outside)
    const distToMinX = pos.x - boundary.minX;
    const distToMaxX = boundary.maxX - pos.x;
    const distToMinY = pos.y - boundary.minY;
    const distToMaxY = boundary.maxY - pos.y;

    const edges: { edge: 'minX' | 'maxX' | 'minY' | 'maxY'; dist: number }[] = [
      { edge: 'minX', dist: distToMinX },
      { edge: 'maxX', dist: distToMaxX },
      { edge: 'minY', dist: distToMinY },
      { edge: 'maxY', dist: distToMaxY },
    ];

    // Nearest edge is the one with the smallest signed distance
    edges.sort((a, b) => a.dist - b.dist);
    const nearest = edges[0];

    const inside = distToMinX >= 0 && distToMaxX >= 0 && distToMinY >= 0 && distToMaxY >= 0;

    // distanceToBoundary: positive when inside (min distance to any edge),
    // negative when outside (using the most negative signed distance)
    let distanceToBoundary: number;
    if (inside) {
      distanceToBoundary = nearest.dist;
    } else {
      // When outside, find the most negative distance (furthest outside)
      distanceToBoundary = nearest.dist;
    }

    // Fire violation callback if outside
    if (!inside && this.onGeofenceViolation && this.history.length > 0) {
      this.onGeofenceViolation(pos, boundary);
    }

    return {
      inside,
      distanceToBoundary,
      nearestEdge: nearest.edge,
    };
  }

  /**
   * Clear all history and reset update counter.
   */
  reset(): void {
    this.history = [];
    this.totalUpdates = 0;
  }

  /**
   * Get summary statistics.
   */
  getStats(): {
    totalUpdates: number;
    historySize: number;
    isStale: boolean;
    distanceTraveled: number;
    currentPosition: Position | null;
  } {
    return {
      totalUpdates: this.totalUpdates,
      historySize: this.history.length,
      isStale: this.isStale(),
      distanceTraveled: this.getDistanceTraveled(),
      currentPosition: this.getCurrentPosition(),
    };
  }

  // --- Private helpers ---

  private isPositionStale(pos: Position): boolean {
    return Date.now() - pos.timestamp > this.staleThresholdMs;
  }

  private euclideanDistance(a: Position, b: Position): number {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const dz = b.z - a.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}

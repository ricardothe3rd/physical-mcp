import { describe, it, expect, vi, beforeEach } from 'vitest';
import { PositionTracker } from './position-tracker.js';
import type { Position, GeofenceBoundary } from './position-tracker.js';

describe('PositionTracker', () => {
  let tracker: PositionTracker;

  beforeEach(() => {
    tracker = new PositionTracker();
  });

  // --- updatePosition ---

  describe('updatePosition', () => {
    it('stores position with timestamp', () => {
      const before = Date.now();
      tracker.updatePosition(1, 2, 3);
      const after = Date.now();

      const pos = tracker.getCurrentPosition();
      expect(pos).not.toBeNull();
      expect(pos!.x).toBe(1);
      expect(pos!.y).toBe(2);
      expect(pos!.z).toBe(3);
      expect(pos!.timestamp).toBeGreaterThanOrEqual(before);
      expect(pos!.timestamp).toBeLessThanOrEqual(after);
    });

    it('z coordinate defaults to 0 when not provided', () => {
      tracker.updatePosition(5, 10);
      const pos = tracker.getCurrentPosition();
      expect(pos).not.toBeNull();
      expect(pos!.z).toBe(0);
    });
  });

  // --- getCurrentPosition ---

  describe('getCurrentPosition', () => {
    it('returns null before any updates', () => {
      expect(tracker.getCurrentPosition()).toBeNull();
    });

    it('returns latest position after updates', () => {
      tracker.updatePosition(1, 2, 3);
      tracker.updatePosition(4, 5, 6);

      const pos = tracker.getCurrentPosition();
      expect(pos).not.toBeNull();
      expect(pos!.x).toBe(4);
      expect(pos!.y).toBe(5);
      expect(pos!.z).toBe(6);
    });

    it('returns null when position is stale', () => {
      const staleTracker = new PositionTracker({ staleThresholdMs: 0 });
      staleTracker.updatePosition(1, 2, 3);

      // With threshold of 0, anything > 0ms old is stale
      // Date.now() will have advanced at least 0ms, so this may or may not be stale
      // Use vi.useFakeTimers for deterministic behavior
      vi.useFakeTimers();
      try {
        const t = new PositionTracker({ staleThresholdMs: 100 });
        vi.setSystemTime(1000);
        t.updatePosition(1, 2, 3);

        // Advance time beyond threshold
        vi.setSystemTime(1200);
        expect(t.getCurrentPosition()).toBeNull();
      } finally {
        vi.useRealTimers();
      }
    });
  });

  // --- isStale ---

  describe('isStale', () => {
    it('returns true when no updates have been received', () => {
      expect(tracker.isStale()).toBe(true);
    });

    it('returns false right after an update', () => {
      tracker.updatePosition(1, 2, 3);
      expect(tracker.isStale()).toBe(false);
    });

    it('returns true when no updates received within threshold', () => {
      vi.useFakeTimers();
      try {
        const t = new PositionTracker({ staleThresholdMs: 500 });

        vi.setSystemTime(1000);
        t.updatePosition(1, 2, 3);
        expect(t.isStale()).toBe(false);

        // Advance past threshold
        vi.setSystemTime(1600);
        expect(t.isStale()).toBe(true);
      } finally {
        vi.useRealTimers();
      }
    });
  });

  // --- getHistory ---

  describe('getHistory', () => {
    it('returns positions in reverse chronological order', () => {
      vi.useFakeTimers();
      try {
        vi.setSystemTime(1000);
        tracker.updatePosition(1, 0, 0);
        vi.setSystemTime(2000);
        tracker.updatePosition(2, 0, 0);
        vi.setSystemTime(3000);
        tracker.updatePosition(3, 0, 0);

        const history = tracker.getHistory();
        expect(history).toHaveLength(3);
        expect(history[0].x).toBe(3); // Most recent first
        expect(history[1].x).toBe(2);
        expect(history[2].x).toBe(1);
      } finally {
        vi.useRealTimers();
      }
    });

    it('respects maxHistorySize (ring buffer behavior)', () => {
      const smallTracker = new PositionTracker({ maxHistorySize: 3 });

      smallTracker.updatePosition(1, 0, 0);
      smallTracker.updatePosition(2, 0, 0);
      smallTracker.updatePosition(3, 0, 0);
      smallTracker.updatePosition(4, 0, 0); // Should push out position 1
      smallTracker.updatePosition(5, 0, 0); // Should push out position 2

      const history = smallTracker.getHistory();
      expect(history).toHaveLength(3);
      expect(history[0].x).toBe(5); // Most recent
      expect(history[1].x).toBe(4);
      expect(history[2].x).toBe(3); // Oldest retained
    });

    it('returns empty array when no updates', () => {
      expect(tracker.getHistory()).toEqual([]);
    });
  });

  // --- getDistanceTraveled ---

  describe('getDistanceTraveled', () => {
    it('returns 0 for single position', () => {
      tracker.updatePosition(1, 2, 3);
      expect(tracker.getDistanceTraveled()).toBe(0);
    });

    it('returns 0 for no positions', () => {
      expect(tracker.getDistanceTraveled()).toBe(0);
    });

    it('calculates correctly for known path', () => {
      // Move along x-axis: 0 -> 3 -> 3 (right angle, then up along y)
      tracker.updatePosition(0, 0, 0);
      tracker.updatePosition(3, 0, 0); // Distance: 3
      tracker.updatePosition(3, 4, 0); // Distance: 4

      // Total: 3 + 4 = 7
      expect(tracker.getDistanceTraveled()).toBe(7);
    });

    it('calculates correctly for 3D path', () => {
      tracker.updatePosition(0, 0, 0);
      tracker.updatePosition(1, 0, 0); // Distance: 1
      tracker.updatePosition(1, 1, 0); // Distance: 1
      tracker.updatePosition(1, 1, 1); // Distance: 1

      // Total: 3
      expect(tracker.getDistanceTraveled()).toBe(3);
    });

    it('calculates diagonal distance using Euclidean formula', () => {
      tracker.updatePosition(0, 0, 0);
      tracker.updatePosition(3, 4, 0); // Distance: sqrt(9+16) = 5

      expect(tracker.getDistanceTraveled()).toBeCloseTo(5, 10);
    });
  });

  // --- getEstimatedVelocity ---

  describe('getEstimatedVelocity', () => {
    it('returns null with fewer than 2 positions', () => {
      expect(tracker.getEstimatedVelocity()).toBeNull();

      tracker.updatePosition(1, 2, 3);
      expect(tracker.getEstimatedVelocity()).toBeNull();
    });

    it('calculates correctly for known positions', () => {
      vi.useFakeTimers();
      try {
        vi.setSystemTime(0);
        tracker.updatePosition(0, 0, 0);

        vi.setSystemTime(1000); // 1 second later
        tracker.updatePosition(3, 4, 0); // Distance = 5

        // Velocity = 5m / 1s = 5 m/s
        expect(tracker.getEstimatedVelocity()).toBeCloseTo(5, 10);
      } finally {
        vi.useRealTimers();
      }
    });

    it('returns null when time delta is zero', () => {
      vi.useFakeTimers();
      try {
        vi.setSystemTime(1000);
        tracker.updatePosition(0, 0, 0);
        tracker.updatePosition(1, 0, 0); // Same timestamp

        expect(tracker.getEstimatedVelocity()).toBeNull();
      } finally {
        vi.useRealTimers();
      }
    });
  });

  // --- checkGeofence ---

  describe('checkGeofence', () => {
    const boundary: GeofenceBoundary = {
      minX: -10,
      maxX: 10,
      minY: -10,
      maxY: 10,
    };

    it('returns inside:true when within bounds', () => {
      tracker.updatePosition(0, 0, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(true);
    });

    it('returns inside:true on exact boundary', () => {
      tracker.updatePosition(10, 10, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(true);
    });

    it('returns inside:false when outside bounds (x exceeds maxX)', () => {
      tracker.updatePosition(15, 0, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(false);
    });

    it('returns inside:false when outside bounds (x below minX)', () => {
      tracker.updatePosition(-15, 0, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(false);
    });

    it('returns inside:false when outside bounds (y exceeds maxY)', () => {
      tracker.updatePosition(0, 15, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(false);
    });

    it('returns inside:false when outside bounds (y below minY)', () => {
      tracker.updatePosition(0, -15, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(false);
    });

    it('returns correct nearestEdge when inside', () => {
      // Position at (8, 0): closest to maxX edge (distance 2)
      tracker.updatePosition(8, 0, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(true);
      expect(result.nearestEdge).toBe('maxX');
      expect(result.distanceToBoundary).toBeCloseTo(2);
    });

    it('returns correct nearestEdge for minY', () => {
      // Position at (0, -8): closest to minY edge (distance 2)
      tracker.updatePosition(0, -8, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(true);
      expect(result.nearestEdge).toBe('minY');
      expect(result.distanceToBoundary).toBeCloseTo(2);
    });

    it('returns correct distanceToBoundary (negative when outside)', () => {
      // Position at (12, 0): 2m outside maxX edge
      tracker.updatePosition(12, 0, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(false);
      expect(result.distanceToBoundary).toBe(-2);
      expect(result.nearestEdge).toBe('maxX');
    });

    it('returns correct negative distance for minX violation', () => {
      // Position at (-13, 0): 3m outside minX edge
      tracker.updatePosition(-13, 0, 0);
      const result = tracker.checkGeofence(boundary);
      expect(result.inside).toBe(false);
      expect(result.distanceToBoundary).toBe(-3);
      expect(result.nearestEdge).toBe('minX');
    });
  });

  // --- onPositionUpdate callback ---

  describe('onPositionUpdate callback', () => {
    it('fires on each updatePosition call', () => {
      const callback = vi.fn();
      const t = new PositionTracker({ onPositionUpdate: callback });

      t.updatePosition(1, 2, 3);
      t.updatePosition(4, 5, 6);

      expect(callback).toHaveBeenCalledTimes(2);

      const firstCall = callback.mock.calls[0][0] as Position;
      expect(firstCall.x).toBe(1);
      expect(firstCall.y).toBe(2);
      expect(firstCall.z).toBe(3);

      const secondCall = callback.mock.calls[1][0] as Position;
      expect(secondCall.x).toBe(4);
      expect(secondCall.y).toBe(5);
      expect(secondCall.z).toBe(6);
    });
  });

  // --- onGeofenceViolation callback ---

  describe('onGeofenceViolation callback', () => {
    it('fires when checkGeofence detects position outside boundary', () => {
      const callback = vi.fn();
      const t = new PositionTracker({ onGeofenceViolation: callback });

      const boundary: GeofenceBoundary = { minX: -5, maxX: 5, minY: -5, maxY: 5 };

      t.updatePosition(10, 0, 0); // Outside
      t.checkGeofence(boundary);

      expect(callback).toHaveBeenCalledTimes(1);
      expect(callback.mock.calls[0][0].x).toBe(10);
      expect(callback.mock.calls[0][1]).toEqual(boundary);
    });

    it('does not fire when position is inside boundary', () => {
      const callback = vi.fn();
      const t = new PositionTracker({ onGeofenceViolation: callback });

      const boundary: GeofenceBoundary = { minX: -5, maxX: 5, minY: -5, maxY: 5 };

      t.updatePosition(0, 0, 0); // Inside
      t.checkGeofence(boundary);

      expect(callback).not.toHaveBeenCalled();
    });
  });

  // --- reset ---

  describe('reset', () => {
    it('clears history and stats', () => {
      tracker.updatePosition(1, 2, 3);
      tracker.updatePosition(4, 5, 6);
      tracker.updatePosition(7, 8, 9);

      tracker.reset();

      expect(tracker.getHistory()).toEqual([]);
      expect(tracker.getCurrentPosition()).toBeNull();
      expect(tracker.getDistanceTraveled()).toBe(0);
      expect(tracker.isStale()).toBe(true);

      const stats = tracker.getStats();
      expect(stats.totalUpdates).toBe(0);
      expect(stats.historySize).toBe(0);
    });
  });

  // --- getStats ---

  describe('getStats', () => {
    it('returns correct summary', () => {
      tracker.updatePosition(0, 0, 0);
      tracker.updatePosition(3, 4, 0); // Distance = 5

      const stats = tracker.getStats();
      expect(stats.totalUpdates).toBe(2);
      expect(stats.historySize).toBe(2);
      expect(stats.isStale).toBe(false);
      expect(stats.distanceTraveled).toBeCloseTo(5, 10);
      expect(stats.currentPosition).not.toBeNull();
      expect(stats.currentPosition!.x).toBe(3);
      expect(stats.currentPosition!.y).toBe(4);
    });

    it('reflects stale state correctly', () => {
      vi.useFakeTimers();
      try {
        const t = new PositionTracker({ staleThresholdMs: 100 });

        vi.setSystemTime(1000);
        t.updatePosition(1, 2, 3);

        vi.setSystemTime(1200); // Past threshold
        const stats = t.getStats();

        expect(stats.isStale).toBe(true);
        expect(stats.currentPosition).toBeNull();
        expect(stats.totalUpdates).toBe(1);
        expect(stats.historySize).toBe(1);
      } finally {
        vi.useRealTimers();
      }
    });

    it('returns empty stats before any updates', () => {
      const stats = tracker.getStats();
      expect(stats.totalUpdates).toBe(0);
      expect(stats.historySize).toBe(0);
      expect(stats.isStale).toBe(true);
      expect(stats.distanceTraveled).toBe(0);
      expect(stats.currentPosition).toBeNull();
    });
  });
});

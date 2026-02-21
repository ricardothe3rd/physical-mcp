import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { RateLimiterCleanup, CleanupStats } from './rate-limiter-cleanup.js';

describe('RateLimiterCleanup', () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  describe('constructor', () => {
    it('uses default config values', () => {
      const cleanup = new RateLimiterCleanup();
      const config = cleanup.getConfig();
      expect(config.intervalMs).toBe(30000);
      expect(config.maxAgeMs).toBe(60000);
      expect(config.maxKeys).toBe(1000);
      expect(config.onCleanup).toBeUndefined();
    });

    it('accepts custom config', () => {
      const onCleanup = vi.fn();
      const cleanup = new RateLimiterCleanup({
        intervalMs: 5000,
        maxAgeMs: 10000,
        maxKeys: 50,
        onCleanup,
      });
      const config = cleanup.getConfig();
      expect(config.intervalMs).toBe(5000);
      expect(config.maxAgeMs).toBe(10000);
      expect(config.maxKeys).toBe(50);
      expect(config.onCleanup).toBe(onCleanup);
    });
  });

  describe('cleanup()', () => {
    it('removes entries older than maxAgeMs', () => {
      const cleanup = new RateLimiterCleanup({ maxAgeMs: 10000 });
      const now = Date.now();
      const windows = new Map<string, number[]>();
      // Mix of old and recent timestamps
      windows.set('topic:a', [now - 15000, now - 12000, now - 5000, now - 1000]);

      const stats = cleanup.cleanup(windows);

      // The two old entries (15s and 12s ago) should be removed
      expect(stats.entriesRemoved).toBe(2);
      expect(windows.get('topic:a')).toEqual([now - 5000, now - 1000]);
    });

    it('removes keys with no remaining entries', () => {
      const cleanup = new RateLimiterCleanup({ maxAgeMs: 10000 });
      const now = Date.now();
      const windows = new Map<string, number[]>();
      // All timestamps are older than maxAgeMs
      windows.set('topic:stale', [now - 20000, now - 15000]);
      // This key has recent timestamps
      windows.set('topic:active', [now - 1000]);

      const stats = cleanup.cleanup(windows);

      expect(stats.keysRemoved).toBe(1);
      expect(stats.keysRemaining).toBe(1);
      expect(windows.has('topic:stale')).toBe(false);
      expect(windows.has('topic:active')).toBe(true);
    });

    it('respects maxKeys limit and removes oldest keys', () => {
      const cleanup = new RateLimiterCleanup({ maxAgeMs: 60000, maxKeys: 2 });
      const now = Date.now();
      const windows = new Map<string, number[]>();
      // Key with oldest most-recent timestamp
      windows.set('topic:oldest', [now - 30000]);
      // Key with middle most-recent timestamp
      windows.set('topic:middle', [now - 20000]);
      // Key with newest most-recent timestamp
      windows.set('topic:newest', [now - 5000]);

      const stats = cleanup.cleanup(windows);

      // Should have removed the oldest key to get down to maxKeys=2
      expect(stats.keysRemoved).toBe(1);
      expect(stats.keysRemaining).toBe(2);
      expect(windows.has('topic:oldest')).toBe(false);
      expect(windows.has('topic:middle')).toBe(true);
      expect(windows.has('topic:newest')).toBe(true);
    });

    it('returns correct stats', () => {
      const cleanup = new RateLimiterCleanup({ maxAgeMs: 10000 });
      const now = Date.now();
      const windows = new Map<string, number[]>();
      windows.set('topic:a', [now - 15000, now - 5000]);
      windows.set('topic:b', [now - 20000]);
      windows.set('topic:c', [now - 1000, now - 2000]);

      const stats = cleanup.cleanup(windows);

      // topic:a: 1 old entry removed, 1 remains
      // topic:b: 1 old entry removed, key removed
      // topic:c: 0 old entries, 2 remain
      expect(stats.keysRemoved).toBe(1);
      expect(stats.entriesRemoved).toBe(2);
      expect(stats.keysRemaining).toBe(2);
      expect(stats.timestamp).toBe(now);
    });

    it('fires onCleanup callback', () => {
      const onCleanup = vi.fn();
      const cleanup = new RateLimiterCleanup({ maxAgeMs: 10000, onCleanup });
      const now = Date.now();
      const windows = new Map<string, number[]>();
      windows.set('topic:a', [now - 15000]);

      cleanup.cleanup(windows);

      expect(onCleanup).toHaveBeenCalledOnce();
      const callArg: CleanupStats = onCleanup.mock.calls[0][0];
      expect(callArg.keysRemoved).toBe(1);
      expect(callArg.entriesRemoved).toBe(1);
      expect(callArg.keysRemaining).toBe(0);
    });

    it('handles empty map', () => {
      const cleanup = new RateLimiterCleanup();
      const windows = new Map<string, number[]>();

      const stats = cleanup.cleanup(windows);

      expect(stats.keysRemoved).toBe(0);
      expect(stats.entriesRemoved).toBe(0);
      expect(stats.keysRemaining).toBe(0);
    });
  });

  describe('start()', () => {
    it('begins periodic cleanup', () => {
      const cleanup = new RateLimiterCleanup({ intervalMs: 5000, maxAgeMs: 10000 });
      const now = Date.now();
      const windows = new Map<string, number[]>();
      windows.set('topic:a', [now - 15000]);

      cleanup.start(windows);
      expect(cleanup.isRunning()).toBe(true);

      // Key still present before interval fires
      expect(windows.has('topic:a')).toBe(true);

      // Advance past the interval
      vi.advanceTimersByTime(5000);

      // Now the stale key should be cleaned up
      expect(windows.has('topic:a')).toBe(false);

      cleanup.stop();
    });

    it('throws if already running', () => {
      const cleanup = new RateLimiterCleanup();
      const windows = new Map<string, number[]>();

      cleanup.start(windows);

      expect(() => cleanup.start(windows)).toThrow(
        'Cleanup is already running. Call stop() before starting again.'
      );

      cleanup.stop();
    });
  });

  describe('stop()', () => {
    it('clears the interval', () => {
      const cleanup = new RateLimiterCleanup({ intervalMs: 5000, maxAgeMs: 10000 });
      const now = Date.now();
      const windows = new Map<string, number[]>();
      windows.set('topic:a', [now - 15000]);

      cleanup.start(windows);
      cleanup.stop();

      expect(cleanup.isRunning()).toBe(false);

      // Advance time - the stale entry should NOT be cleaned because we stopped
      vi.advanceTimersByTime(10000);
      expect(windows.has('topic:a')).toBe(true);
    });

    it('is safe to call when not running', () => {
      const cleanup = new RateLimiterCleanup();

      // Should not throw
      expect(() => cleanup.stop()).not.toThrow();
      expect(cleanup.isRunning()).toBe(false);
    });
  });

  describe('isRunning()', () => {
    it('returns correct state', () => {
      const cleanup = new RateLimiterCleanup();
      const windows = new Map<string, number[]>();

      expect(cleanup.isRunning()).toBe(false);

      cleanup.start(windows);
      expect(cleanup.isRunning()).toBe(true);

      cleanup.stop();
      expect(cleanup.isRunning()).toBe(false);
    });
  });

  describe('getTotalStats()', () => {
    it('accumulates across multiple cleanups', () => {
      const cleanup = new RateLimiterCleanup({ maxAgeMs: 10000 });
      const now = Date.now();

      // First cleanup pass
      const windows1 = new Map<string, number[]>();
      windows1.set('topic:a', [now - 15000]);
      windows1.set('topic:b', [now - 12000, now - 11000]);
      cleanup.cleanup(windows1);

      // Second cleanup pass
      const windows2 = new Map<string, number[]>();
      windows2.set('topic:c', [now - 20000]);
      cleanup.cleanup(windows2);

      const totalStats = cleanup.getTotalStats();
      expect(totalStats.totalKeysRemoved).toBe(3);     // a + b + c
      expect(totalStats.totalEntriesRemoved).toBe(4);   // 1 + 2 + 1
      expect(totalStats.lastCleanup).toBe(now);
      expect(totalStats.isRunning).toBe(false);
    });
  });

  describe('progressive cleanup', () => {
    it('multiple cleanup passes correctly clean up progressively', () => {
      const cleanup = new RateLimiterCleanup({ intervalMs: 5000, maxAgeMs: 10000 });
      const windows = new Map<string, number[]>();

      // Set initial time
      const startTime = Date.now();

      // Add entries at t=0
      windows.set('topic:a', [startTime]);
      windows.set('topic:b', [startTime]);

      cleanup.start(windows);

      // At t=5s (first interval fires), entries are 5s old - still valid (< 10s maxAge)
      vi.advanceTimersByTime(5000);
      expect(windows.size).toBe(2);
      expect(windows.has('topic:a')).toBe(true);
      expect(windows.has('topic:b')).toBe(true);

      // Add a new entry for topic:a at t=5s, keeping it fresh
      windows.get('topic:a')!.push(Date.now());

      // Advance to t=15s (intervals fire at t=10s and t=15s)
      // At t=10s: original entries from t=0 are exactly 10s old.
      //   cutoff = 10000 - 10000 = 0, timestamps >= 0 pass, so they survive.
      // At t=15s: original entries from t=0 are 15s old.
      //   cutoff = 15000 - 10000 = 5000, timestamp 0 < 5000, so removed.
      //   topic:a still has entry from t=5s (10s old, cutoff=5000, 5000 >= 5000 passes).
      //   topic:b has no remaining entries, key removed.
      vi.advanceTimersByTime(10000);

      expect(windows.has('topic:a')).toBe(true);
      expect(windows.has('topic:b')).toBe(false);

      // Advance to t=20s (interval fires)
      // topic:a's entry from t=5s is now 15s old (> 10s maxAge), removed
      vi.advanceTimersByTime(5000);

      expect(windows.size).toBe(0);

      const totalStats = cleanup.getTotalStats();
      expect(totalStats.totalKeysRemoved).toBe(2);

      cleanup.stop();
    });
  });
});

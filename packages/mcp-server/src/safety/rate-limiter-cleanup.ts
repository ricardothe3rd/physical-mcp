/**
 * Periodic cleanup for rate limiter sliding windows.
 * Prevents memory leaks from abandoned topics accumulating entries.
 */

export interface CleanupConfig {
  intervalMs?: number;      // How often to run cleanup (default 30000 = 30s)
  maxAgeMs?: number;        // Remove entries older than this (default 60000 = 60s)
  maxKeys?: number;         // Maximum number of rate limit keys to keep (default 1000)
  onCleanup?: (stats: CleanupStats) => void;
}

export interface CleanupStats {
  keysRemoved: number;
  entriesRemoved: number;
  keysRemaining: number;
  timestamp: number;
}

const DEFAULT_INTERVAL_MS = 30000;
const DEFAULT_MAX_AGE_MS = 60000;
const DEFAULT_MAX_KEYS = 1000;

export class RateLimiterCleanup {
  private config: Required<Omit<CleanupConfig, 'onCleanup'>> & { onCleanup?: (stats: CleanupStats) => void };
  private timer: ReturnType<typeof setInterval> | null = null;
  private lastCleanup: number = 0;
  private totalKeysRemoved: number = 0;
  private totalEntriesRemoved: number = 0;

  constructor(config?: CleanupConfig) {
    this.config = {
      intervalMs: config?.intervalMs ?? DEFAULT_INTERVAL_MS,
      maxAgeMs: config?.maxAgeMs ?? DEFAULT_MAX_AGE_MS,
      maxKeys: config?.maxKeys ?? DEFAULT_MAX_KEYS,
      onCleanup: config?.onCleanup,
    };
  }

  /**
   * Start periodic cleanup using a Map of key -> timestamps.
   * Throws if cleanup is already running.
   */
  start(windows: Map<string, number[]>): void {
    if (this.timer !== null) {
      throw new Error('Cleanup is already running. Call stop() before starting again.');
    }

    this.timer = setInterval(() => {
      this.cleanup(windows);
    }, this.config.intervalMs);
  }

  /** Stop periodic cleanup. Safe to call when not running. */
  stop(): void {
    if (this.timer !== null) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }

  /**
   * Run a single cleanup pass manually.
   * 1. Remove timestamps older than maxAgeMs from each key.
   * 2. Remove keys with no remaining timestamps.
   * 3. If keys exceed maxKeys, evict the oldest keys (by most recent timestamp).
   * 4. Fire onCleanup callback with stats.
   */
  cleanup(windows: Map<string, number[]>): CleanupStats {
    const now = Date.now();
    const cutoff = now - this.config.maxAgeMs;
    let keysRemoved = 0;
    let entriesRemoved = 0;

    // Step 1 & 2: Remove old timestamps and empty keys
    for (const [key, timestamps] of windows) {
      const originalLength = timestamps.length;
      const filtered = timestamps.filter(t => t >= cutoff);
      const removedCount = originalLength - filtered.length;

      if (removedCount > 0) {
        entriesRemoved += removedCount;
      }

      if (filtered.length === 0) {
        windows.delete(key);
        keysRemoved++;
      } else {
        // Update the array in place by replacing its contents
        windows.set(key, filtered);
      }
    }

    // Step 3: Evict oldest keys if over maxKeys
    if (windows.size > this.config.maxKeys) {
      // Build a list of (key, mostRecentTimestamp) and sort ascending by most recent
      const keysByRecency: Array<{ key: string; mostRecent: number }> = [];
      for (const [key, timestamps] of windows) {
        const mostRecent = timestamps.length > 0 ? Math.max(...timestamps) : 0;
        keysByRecency.push({ key, mostRecent });
      }
      // Sort ascending: oldest (smallest mostRecent) first
      keysByRecency.sort((a, b) => a.mostRecent - b.mostRecent);

      const toRemove = windows.size - this.config.maxKeys;
      for (let i = 0; i < toRemove; i++) {
        const entry = keysByRecency[i];
        const timestamps = windows.get(entry.key);
        if (timestamps) {
          entriesRemoved += timestamps.length;
        }
        windows.delete(entry.key);
        keysRemoved++;
      }
    }

    const stats: CleanupStats = {
      keysRemoved,
      entriesRemoved,
      keysRemaining: windows.size,
      timestamp: now,
    };

    this.lastCleanup = now;
    this.totalKeysRemoved += keysRemoved;
    this.totalEntriesRemoved += entriesRemoved;

    if (this.config.onCleanup) {
      this.config.onCleanup(stats);
    }

    return stats;
  }

  /** Check if periodic cleanup is running. */
  isRunning(): boolean {
    return this.timer !== null;
  }

  /** Get cumulative stats about all cleanup activity. */
  getTotalStats(): {
    totalKeysRemoved: number;
    totalEntriesRemoved: number;
    lastCleanup: number;
    isRunning: boolean;
  } {
    return {
      totalKeysRemoved: this.totalKeysRemoved,
      totalEntriesRemoved: this.totalEntriesRemoved,
      lastCleanup: this.lastCleanup,
      isRunning: this.isRunning(),
    };
  }

  /** Get the current cleanup configuration. */
  getConfig(): CleanupConfig {
    return { ...this.config };
  }
}

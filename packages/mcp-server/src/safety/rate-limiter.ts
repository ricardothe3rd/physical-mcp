/**
 * Command rate limiting.
 */

import type { RateLimitConfig, SafetyViolation } from './types.js';

interface RateWindow {
  timestamps: number[];
}

export class RateLimiter {
  private windows = new Map<string, RateWindow>();
  private config: RateLimitConfig;

  constructor(config: RateLimitConfig) {
    this.config = config;
  }

  checkPublish(topic: string): SafetyViolation | null {
    return this.check(`publish:${topic}`, this.config.publishHz, 1000);
  }

  checkService(service: string): SafetyViolation | null {
    return this.check(`service:${service}`, this.config.servicePerMinute, 60000);
  }

  checkAction(action: string): SafetyViolation | null {
    return this.check(`action:${action}`, this.config.actionPerMinute, 60000);
  }

  private check(key: string, maxCount: number, windowMs: number): SafetyViolation | null {
    const now = Date.now();
    let window = this.windows.get(key);

    if (!window) {
      window = { timestamps: [] };
      this.windows.set(key, window);
    }

    // Remove expired timestamps
    window.timestamps = window.timestamps.filter(t => now - t < windowMs);

    if (window.timestamps.length >= maxCount) {
      return {
        type: 'rate_limit_exceeded',
        message: `Rate limit exceeded for ${key}: ${window.timestamps.length}/${maxCount} in ${windowMs}ms`,
        details: { key, count: window.timestamps.length, limit: maxCount, windowMs },
        timestamp: now,
      };
    }

    window.timestamps.push(now);
    return null;
  }

  updateConfig(config: Partial<RateLimitConfig>) {
    Object.assign(this.config, config);
  }

  getStats(): { activeWindows: number; entries: { key: string; count: number; limit: number }[] } {
    const now = Date.now();
    const entries: { key: string; count: number; limit: number }[] = [];

    for (const [key, window] of this.windows) {
      // Determine the correct window and limit based on key prefix
      let windowMs = 1000;
      let limit = this.config.publishHz;
      if (key.startsWith('service:')) {
        windowMs = 60000;
        limit = this.config.servicePerMinute;
      } else if (key.startsWith('action:')) {
        windowMs = 60000;
        limit = this.config.actionPerMinute;
      }

      const validTimestamps = window.timestamps.filter(t => now - t < windowMs);
      if (validTimestamps.length > 0) {
        entries.push({ key, count: validTimestamps.length, limit });
      }
    }

    return { activeWindows: entries.length, entries };
  }

  /**
   * Remove stale windows that have no active timestamps.
   * Call periodically to prevent memory leaks from abandoned topics.
   */
  cleanup(): number {
    const now = Date.now();
    let removed = 0;

    for (const [key, window] of this.windows) {
      // Use a conservative 60s window for cleanup
      window.timestamps = window.timestamps.filter(t => now - t < 60000);
      if (window.timestamps.length === 0) {
        this.windows.delete(key);
        removed++;
      }
    }

    return removed;
  }

  reset() {
    this.windows.clear();
  }
}

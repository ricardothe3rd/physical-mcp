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

  reset() {
    this.windows.clear();
  }
}

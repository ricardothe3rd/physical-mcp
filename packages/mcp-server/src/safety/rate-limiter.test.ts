import { describe, it, expect, beforeEach, vi } from 'vitest';
import { RateLimiter } from './rate-limiter.js';

describe('RateLimiter', () => {
  let limiter: RateLimiter;

  beforeEach(() => {
    limiter = new RateLimiter({
      publishHz: 3,          // 3 publishes per second
      servicePerMinute: 5,   // 5 service calls per minute
      actionPerMinute: 2,    // 2 action goals per minute
    });
  });

  describe('checkPublish', () => {
    it('allows publishes under the limit', () => {
      expect(limiter.checkPublish('/cmd_vel')).toBeNull();
      expect(limiter.checkPublish('/cmd_vel')).toBeNull();
      expect(limiter.checkPublish('/cmd_vel')).toBeNull();
    });

    it('blocks when publish rate exceeded', () => {
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      const result = limiter.checkPublish('/cmd_vel');
      expect(result).not.toBeNull();
      expect(result!.type).toBe('rate_limit_exceeded');
    });

    it('tracks different topics independently', () => {
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      // /cmd_vel is at limit, but /other_topic is fresh
      expect(limiter.checkPublish('/other_topic')).toBeNull();
    });
  });

  describe('checkService', () => {
    it('allows service calls under the limit', () => {
      for (let i = 0; i < 5; i++) {
        expect(limiter.checkService('/my_service')).toBeNull();
      }
    });

    it('blocks when service rate exceeded', () => {
      for (let i = 0; i < 5; i++) {
        limiter.checkService('/my_service');
      }
      const result = limiter.checkService('/my_service');
      expect(result).not.toBeNull();
      expect(result!.type).toBe('rate_limit_exceeded');
    });
  });

  describe('checkAction', () => {
    it('allows action goals under the limit', () => {
      expect(limiter.checkAction('/navigate')).toBeNull();
      expect(limiter.checkAction('/navigate')).toBeNull();
    });

    it('blocks when action rate exceeded', () => {
      limiter.checkAction('/navigate');
      limiter.checkAction('/navigate');
      const result = limiter.checkAction('/navigate');
      expect(result).not.toBeNull();
      expect(result!.type).toBe('rate_limit_exceeded');
    });
  });

  describe('window expiry', () => {
    it('resets after window expires', () => {
      vi.useFakeTimers();

      // Fill up the publish limit
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();

      // Advance past the 1-second window
      vi.advanceTimersByTime(1100);

      // Should be allowed again
      expect(limiter.checkPublish('/cmd_vel')).toBeNull();

      vi.useRealTimers();
    });
  });

  describe('reset', () => {
    it('clears all rate limits', () => {
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();

      limiter.reset();
      expect(limiter.checkPublish('/cmd_vel')).toBeNull();
    });
  });

  describe('updateConfig', () => {
    it('applies new rate limits', () => {
      limiter.updateConfig({ publishHz: 1 });
      limiter.checkPublish('/cmd_vel');
      const result = limiter.checkPublish('/cmd_vel');
      expect(result).not.toBeNull();
    });
  });
});

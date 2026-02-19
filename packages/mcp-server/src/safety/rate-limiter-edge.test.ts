/**
 * Edge case tests for the RateLimiter.
 *
 * Covers high-load scenarios, window boundary timing, memory leak
 * prevention via cleanup, and topic/service independence.
 */

import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest';
import { RateLimiter } from './rate-limiter.js';

describe('RateLimiter Edge Cases', () => {
  afterEach(() => {
    vi.useRealTimers();
  });

  describe('100+ rapid calls (high load)', () => {
    it('enforces publish rate limit of 10 Hz across 200 rapid calls', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      let allowed = 0;
      let blocked = 0;

      for (let i = 0; i < 200; i++) {
        const result = limiter.checkPublish('/cmd_vel');
        if (result === null) {
          allowed++;
        } else {
          blocked++;
          expect(result.type).toBe('rate_limit_exceeded');
        }
      }

      expect(allowed).toBe(10);
      expect(blocked).toBe(190);
    });

    it('enforces service rate limit of 5/min across 100 rapid calls', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 5,
        actionPerMinute: 30,
      });

      let allowed = 0;
      let blocked = 0;

      for (let i = 0; i < 100; i++) {
        const result = limiter.checkService('/my_service');
        if (result === null) {
          allowed++;
        } else {
          blocked++;
        }
      }

      expect(allowed).toBe(5);
      expect(blocked).toBe(95);
    });

    it('enforces action rate limit of 2/min across 100 rapid calls', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 2,
      });

      let allowed = 0;
      let blocked = 0;

      for (let i = 0; i < 100; i++) {
        const result = limiter.checkAction('/navigate');
        if (result === null) {
          allowed++;
        } else {
          blocked++;
        }
      }

      expect(allowed).toBe(2);
      expect(blocked).toBe(98);
    });

    it('getStats reflects correct counts under high load', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Send 50 calls across 5 topics
      for (let t = 0; t < 5; t++) {
        for (let c = 0; c < 10; c++) {
          limiter.checkPublish(`/topic_${t}`);
        }
      }

      const stats = limiter.getStats();
      expect(stats.activeWindows).toBe(5);

      // Each topic should have 10 entries recorded (all 10 allowed within limit,
      // but only 10 timestamps are added since blocked calls don't add timestamps)
      for (const entry of stats.entries) {
        expect(entry.count).toBe(10);
        expect(entry.limit).toBe(10);
      }
    });

    it('violation messages contain correct details', () => {
      const limiter = new RateLimiter({
        publishHz: 3,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Fill up the limit
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');

      // This should produce a violation
      const violation = limiter.checkPublish('/cmd_vel');
      expect(violation).not.toBeNull();
      expect(violation!.type).toBe('rate_limit_exceeded');
      expect(violation!.message).toContain('publish:/cmd_vel');
      expect(violation!.message).toContain('3/3');
      expect(violation!.details.key).toBe('publish:/cmd_vel');
      expect(violation!.details.count).toBe(3);
      expect(violation!.details.limit).toBe(3);
      expect(violation!.details.windowMs).toBe(1000);
    });
  });

  describe('window expiry edge case (calls right at window boundary)', () => {
    it('calls just before window expiry are still counted', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 3,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Use 2 of 3 slots
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');

      // Advance to just before the 1s window expires (999ms)
      vi.advanceTimersByTime(999);

      // Third call at 999ms should still be within the window
      const result = limiter.checkPublish('/cmd_vel');
      expect(result).toBeNull(); // allowed (3rd of 3)

      // Fourth call should be blocked (still within the same window)
      const blocked = limiter.checkPublish('/cmd_vel');
      expect(blocked).not.toBeNull();
    });

    it('calls exactly at window expiry allow new calls', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 3,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Fill up the limit
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');

      // Blocked immediately
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();

      // Advance exactly 1000ms — the original timestamps become expired
      vi.advanceTimersByTime(1000);

      // Should be allowed again (old timestamps are now >= 1000ms old)
      const afterExpiry = limiter.checkPublish('/cmd_vel');
      expect(afterExpiry).toBeNull();
    });

    it('service window expiry at 60-second boundary', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 3,
        actionPerMinute: 30,
      });

      // Fill up service limit
      limiter.checkService('/svc');
      limiter.checkService('/svc');
      limiter.checkService('/svc');
      expect(limiter.checkService('/svc')).not.toBeNull();

      // Advance just short of 60s
      vi.advanceTimersByTime(59999);
      expect(limiter.checkService('/svc')).not.toBeNull(); // still blocked

      // Advance 1 more ms (total 60000ms)
      vi.advanceTimersByTime(1);

      // Now the original timestamps should be expired
      expect(limiter.checkService('/svc')).toBeNull();
    });

    it('mixed timing: some timestamps expire while others remain', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 5,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Add 3 calls at t=0
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');

      // Advance 600ms
      vi.advanceTimersByTime(600);

      // Add 2 more calls at t=600ms
      limiter.checkPublish('/cmd_vel');
      limiter.checkPublish('/cmd_vel');

      // At t=600ms we have 5 timestamps (3 at t=0, 2 at t=600)
      // Limit is 5, so next should be blocked
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();

      // Advance to t=1001ms — the 3 calls at t=0 expire
      vi.advanceTimersByTime(401);

      // Now only 2 timestamps remain (from t=600), so we have room
      const result = limiter.checkPublish('/cmd_vel');
      expect(result).toBeNull();
    });

    it('timestamps slide continuously within the window', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 2,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // t=0: call 1
      limiter.checkPublish('/cmd_vel');

      // t=400ms: call 2
      vi.advanceTimersByTime(400);
      limiter.checkPublish('/cmd_vel');

      // t=400ms: blocked (2 calls within 1s)
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();

      // t=1001ms: call 1 (from t=0) expires, but call 2 (from t=400) is still valid
      vi.advanceTimersByTime(601);
      const result = limiter.checkPublish('/cmd_vel');
      expect(result).toBeNull(); // allowed: only 1 existing + this new one = 2

      // Now blocked again
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();
    });
  });

  describe('rate limiter cleanup (no memory leak)', () => {
    it('cleanup removes all windows after sufficient time', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Create windows for 50 different topics
      for (let i = 0; i < 50; i++) {
        limiter.checkPublish(`/topic_${i}`);
      }

      expect(limiter.getStats().activeWindows).toBe(50);

      // Advance past the 60s cleanup window
      vi.advanceTimersByTime(61000);

      const removed = limiter.cleanup();
      expect(removed).toBe(50);
      expect(limiter.getStats().activeWindows).toBe(0);
    });

    it('cleanup preserves recently-active windows', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Create old windows
      for (let i = 0; i < 10; i++) {
        limiter.checkPublish(`/old_${i}`);
      }

      // Advance past cleanup window
      vi.advanceTimersByTime(61000);

      // Create fresh windows
      for (let i = 0; i < 5; i++) {
        limiter.checkPublish(`/new_${i}`);
      }

      const removed = limiter.cleanup();
      expect(removed).toBe(10); // only old ones removed

      const stats = limiter.getStats();
      expect(stats.activeWindows).toBe(5); // new ones remain
    });

    it('cleanup handles mixed publish, service, and action windows', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      limiter.checkPublish('/topic_a');
      limiter.checkService('/service_a');
      limiter.checkAction('/action_a');

      // Advance past cleanup window
      vi.advanceTimersByTime(61000);

      // Add fresh entries
      limiter.checkPublish('/topic_b');
      limiter.checkService('/service_b');

      const removed = limiter.cleanup();
      expect(removed).toBe(3); // old topic_a, service_a, action_a

      const stats = limiter.getStats();
      expect(stats.activeWindows).toBe(2); // topic_b and service_b
    });

    it('repeated cleanup calls are idempotent', () => {
      vi.useFakeTimers();

      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      for (let i = 0; i < 10; i++) {
        limiter.checkPublish(`/topic_${i}`);
      }

      vi.advanceTimersByTime(61000);

      const first = limiter.cleanup();
      expect(first).toBe(10);

      const second = limiter.cleanup();
      expect(second).toBe(0);

      const third = limiter.cleanup();
      expect(third).toBe(0);
    });

    it('reset clears everything instantly regardless of timing', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      for (let i = 0; i < 20; i++) {
        limiter.checkPublish(`/topic_${i}`);
        limiter.checkService(`/svc_${i}`);
      }

      expect(limiter.getStats().activeWindows).toBeGreaterThan(0);

      limiter.reset();
      expect(limiter.getStats().activeWindows).toBe(0);
    });
  });

  describe('different topics/services have independent limits', () => {
    it('publish rate limits are per-topic', () => {
      const limiter = new RateLimiter({
        publishHz: 2,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Topic A: 2 allowed
      expect(limiter.checkPublish('/topicA')).toBeNull();
      expect(limiter.checkPublish('/topicA')).toBeNull();
      expect(limiter.checkPublish('/topicA')).not.toBeNull(); // blocked

      // Topic B: still has its own 2 slots
      expect(limiter.checkPublish('/topicB')).toBeNull();
      expect(limiter.checkPublish('/topicB')).toBeNull();
      expect(limiter.checkPublish('/topicB')).not.toBeNull(); // blocked

      // Topic C: fresh
      expect(limiter.checkPublish('/topicC')).toBeNull();
    });

    it('service rate limits are per-service', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 3,
        actionPerMinute: 30,
      });

      // Service A: 3 allowed
      expect(limiter.checkService('/svcA')).toBeNull();
      expect(limiter.checkService('/svcA')).toBeNull();
      expect(limiter.checkService('/svcA')).toBeNull();
      expect(limiter.checkService('/svcA')).not.toBeNull(); // blocked

      // Service B: independent, 3 allowed
      expect(limiter.checkService('/svcB')).toBeNull();
      expect(limiter.checkService('/svcB')).toBeNull();
      expect(limiter.checkService('/svcB')).toBeNull();
      expect(limiter.checkService('/svcB')).not.toBeNull(); // blocked
    });

    it('action rate limits are per-action', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 2,
      });

      expect(limiter.checkAction('/nav')).toBeNull();
      expect(limiter.checkAction('/nav')).toBeNull();
      expect(limiter.checkAction('/nav')).not.toBeNull(); // blocked

      // Different action is independent
      expect(limiter.checkAction('/pick')).toBeNull();
      expect(limiter.checkAction('/pick')).toBeNull();
      expect(limiter.checkAction('/pick')).not.toBeNull(); // blocked
    });

    it('publish, service, and action on same name are independent', () => {
      const limiter = new RateLimiter({
        publishHz: 2,
        servicePerMinute: 2,
        actionPerMinute: 2,
      });

      // All three categories share the name "/robot" but are independent
      expect(limiter.checkPublish('/robot')).toBeNull();
      expect(limiter.checkPublish('/robot')).toBeNull();
      expect(limiter.checkPublish('/robot')).not.toBeNull(); // blocked

      expect(limiter.checkService('/robot')).toBeNull();
      expect(limiter.checkService('/robot')).toBeNull();
      expect(limiter.checkService('/robot')).not.toBeNull(); // blocked

      expect(limiter.checkAction('/robot')).toBeNull();
      expect(limiter.checkAction('/robot')).toBeNull();
      expect(limiter.checkAction('/robot')).not.toBeNull(); // blocked

      // Stats should show 3 independent windows
      const stats = limiter.getStats();
      expect(stats.activeWindows).toBe(3);
      expect(stats.entries.find(e => e.key === 'publish:/robot')).toBeDefined();
      expect(stats.entries.find(e => e.key === 'service:/robot')).toBeDefined();
      expect(stats.entries.find(e => e.key === 'action:/robot')).toBeDefined();
    });

    it('updateConfig changes limits for all future checks', () => {
      const limiter = new RateLimiter({
        publishHz: 5,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Allow 5 publishes
      for (let i = 0; i < 5; i++) {
        expect(limiter.checkPublish('/cmd_vel')).toBeNull();
      }
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();

      // Update to higher limit — but old timestamps still count
      limiter.updateConfig({ publishHz: 20 });

      // Now we can publish more (5 existing + up to 15 more = 20 total)
      for (let i = 0; i < 15; i++) {
        expect(limiter.checkPublish('/cmd_vel')).toBeNull();
      }
      // Now at 20 total — should be blocked again
      expect(limiter.checkPublish('/cmd_vel')).not.toBeNull();
    });
  });
});

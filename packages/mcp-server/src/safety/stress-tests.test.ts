/**
 * Stress and edge case tests for safety subsystems.
 *
 * Tests high-load scenarios, memory caps, concurrent operations,
 * and boundary conditions that wouldn't be covered in unit tests.
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';
import { AuditLogger } from './audit-logger.js';
import { RateLimiter } from './rate-limiter.js';

describe('Stress Tests', () => {
  describe('Audit Logger Memory Cap', () => {
    it('caps entries at maxEntries (default 10000)', () => {
      const logger = new AuditLogger(100); // small cap for testing

      for (let i = 0; i < 200; i++) {
        logger.log('publish', '/cmd_vel', { i }, { allowed: true, violations: [] });
      }

      const entries = logger.getEntries();
      expect(entries.length).toBe(100);
    });

    it('keeps newest entries when capped', () => {
      const logger = new AuditLogger(5);

      for (let i = 0; i < 10; i++) {
        logger.log('publish', `/topic_${i}`, { i }, { allowed: true, violations: [] });
      }

      const entries = logger.getEntries();
      // entries are reversed (newest first)
      expect(entries[0].target).toBe('/topic_9');
      expect(entries[4].target).toBe('/topic_5');
    });

    it('handles rapid concurrent-like writes', () => {
      const logger = new AuditLogger(1000);

      // Simulate rapid writes
      for (let i = 0; i < 500; i++) {
        logger.log(
          i % 3 === 0 ? 'publish' : i % 3 === 1 ? 'service_call' : 'action_goal',
          `/target_${i}`,
          { iteration: i },
          {
            allowed: i % 5 !== 0,
            violations: i % 5 === 0
              ? [{ type: 'velocity_exceeded' as any, message: 'test', details: {}, timestamp: Date.now() }]
              : [],
          }
        );
      }

      const stats = logger.getStats();
      expect(stats.total).toBe(500);
      expect(stats.blocked).toBe(100); // every 5th is blocked
      expect(stats.allowed).toBe(400);
    });

    it('filters correctly with large dataset', () => {
      const logger = new AuditLogger(1000);

      for (let i = 0; i < 200; i++) {
        const command = i % 2 === 0 ? 'publish' : 'service_call';
        const allowed = i % 3 !== 0;
        logger.log(command, '/test', {}, {
          allowed,
          violations: allowed ? [] : [{ type: 'rate_limit_exceeded' as any, message: 'test', details: {}, timestamp: Date.now() }],
        });
      }

      const pubOnly = logger.getEntries({ command: 'publish' });
      expect(pubOnly.length).toBe(100);

      const violationsOnly = logger.getEntries({ violationsOnly: true });
      expect(violationsOnly.length).toBeGreaterThan(0);

      const limited = logger.getEntries({ limit: 10 });
      expect(limited.length).toBe(10);
    });
  });

  describe('Rate Limiter High Load', () => {
    it('handles 100+ rapid publish checks', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      let allowed = 0;
      let blocked = 0;

      for (let i = 0; i < 100; i++) {
        const result = limiter.checkPublish('/cmd_vel');
        if (result) {
          blocked++;
        } else {
          allowed++;
        }
      }

      // With 10 Hz limit, first ~10 should be allowed in the same second window
      expect(allowed).toBeLessThanOrEqual(10);
      expect(blocked).toBeGreaterThanOrEqual(90);
    });

    it('handles multiple topics independently', () => {
      const limiter = new RateLimiter({
        publishHz: 5,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      let topic1Allowed = 0;
      let topic2Allowed = 0;

      for (let i = 0; i < 20; i++) {
        if (!limiter.checkPublish('/cmd_vel')) topic1Allowed++;
        if (!limiter.checkPublish('/other_topic')) topic2Allowed++;
      }

      // Each topic gets its own rate limit window
      expect(topic1Allowed).toBeLessThanOrEqual(5);
      expect(topic2Allowed).toBeLessThanOrEqual(5);
    });

    it('handles service rate limit over minute boundary', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 5,
        actionPerMinute: 30,
      });

      let allowed = 0;

      for (let i = 0; i < 20; i++) {
        const result = limiter.checkService('/test_service');
        if (!result) allowed++;
      }

      expect(allowed).toBeLessThanOrEqual(5);
    });

    it('cleanup removes stale windows', () => {
      const limiter = new RateLimiter({
        publishHz: 10,
        servicePerMinute: 60,
        actionPerMinute: 30,
      });

      // Create some windows
      for (let i = 0; i < 10; i++) {
        limiter.checkPublish(`/topic_${i}`);
      }

      const statsBefore = limiter.getStats();
      expect(statsBefore.activeWindows).toBeGreaterThan(0);

      // Cleanup shouldn't remove active windows
      limiter.cleanup();
      const statsAfter = limiter.getStats();
      expect(statsAfter.activeWindows).toBe(statsBefore.activeWindows);
    });
  });

  describe('Policy Engine Concurrent Safety', () => {
    let engine: PolicyEngine;

    beforeEach(() => {
      engine = new PolicyEngine();
    });

    it('handles many publishes without state corruption', () => {
      const results = [];
      for (let i = 0; i < 50; i++) {
        const speed = 0.1 + (i * 0.02); // gradually increasing speed
        const result = engine.checkPublish('/cmd_vel', {
          linear: { x: speed, y: 0, z: 0 },
        });
        results.push({ speed, allowed: result.allowed });
      }

      // Speed exceeds 0.5 at index ~20 (0.1 + 20*0.02 = 0.5)
      const firstBlock = results.findIndex(r => !r.allowed);
      expect(firstBlock).toBeGreaterThan(0);
      expect(firstBlock).toBeLessThan(25);

      // All after should be blocked
      for (let i = firstBlock; i < results.length; i++) {
        expect(results[i].allowed).toBe(false);
      }
    });

    it('e-stop blocks everything regardless of velocity', () => {
      engine.activateEmergencyStop();

      for (let i = 0; i < 20; i++) {
        const result = engine.checkPublish('/cmd_vel', {
          linear: { x: 0.01, y: 0, z: 0 },
        });
        expect(result.allowed).toBe(false);
        expect(result.violations[0].type).toBe('emergency_stop_active');
      }
    });

    it('e-stop release allows commands again', () => {
      engine.activateEmergencyStop();

      const blocked = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
      });
      expect(blocked.allowed).toBe(false);

      engine.releaseEmergencyStop();

      const allowed = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
      });
      expect(allowed.allowed).toBe(true);
    });

    it('safety score tracks correctly across operations', () => {
      // Publish safe commands on different topics to avoid rate limits
      for (let i = 0; i < 5; i++) {
        engine.checkPublish(`/robot_${i}/cmd_vel`, { linear: { x: 0.1 } });
      }

      // Publish velocity-exceeded commands on different topics
      for (let i = 0; i < 5; i++) {
        engine.checkPublish(`/fast_${i}/cmd_vel`, { linear: { x: 5.0 } });
      }

      const status = engine.getStatus();
      expect(status.safetyScore.totalCommands).toBe(10);
      expect(status.safetyScore.allowedCommands).toBe(5);
      expect(status.safetyScore.blockedCommands).toBe(5);
      expect(status.safetyScore.safetyScore).toBe(50);
    });

    it('event emitter fires on violations', () => {
      const events: unknown[] = [];
      engine.events.on('violation', (e) => events.push(e));

      engine.checkPublish('/cmd_vel', { linear: { x: 5.0 } });
      engine.checkPublish('/cmd_vel', { linear: { x: 0.1 } }); // safe, no event

      expect(events).toHaveLength(1);
    });

    it('event emitter fires on e-stop', () => {
      const events: unknown[] = [];
      engine.events.on('emergency_stop_activated', (e) => events.push(e));
      engine.events.on('emergency_stop_released', (e) => events.push(e));

      engine.activateEmergencyStop();
      engine.releaseEmergencyStop();

      expect(events).toHaveLength(2);
    });
  });
});

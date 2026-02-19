/**
 * Tests for concurrent/race condition scenarios in the safety system.
 *
 * Validates that the PolicyEngine, RateLimiter, and AuditLogger maintain
 * consistency under concurrent-like access patterns (rapid interleaved calls,
 * state changes mid-check, etc.).
 */

import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Concurrent Safety', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  afterEach(() => {
    engine.destroy();
  });

  describe('multiple simultaneous safety checks', () => {
    it('handles interleaved publish, service, and action checks without corruption', () => {
      const results: { type: string; allowed: boolean }[] = [];

      for (let i = 0; i < 30; i++) {
        // Interleave all three check types
        const pubResult = engine.checkPublish(`/robot_${i}/cmd_vel`, {
          linear: { x: 0.1, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
        results.push({ type: 'publish', allowed: pubResult.allowed });

        const svcResult = engine.checkServiceCall(`/service_${i}`, { param: i });
        results.push({ type: 'service', allowed: svcResult.allowed });

        const actResult = engine.checkActionGoal(`/action_${i}`, { goal: i });
        results.push({ type: 'action', allowed: actResult.allowed });
      }

      // All should succeed (unique topics/services/actions, within rate limits)
      expect(results.every(r => r.allowed)).toBe(true);

      // Audit log should have exactly 90 entries (30 * 3)
      const stats = engine.getAuditStats();
      expect(stats.total).toBe(90);
    });

    it('concurrent publishes on the same topic hit rate limits correctly', () => {
      const results: boolean[] = [];

      // Default rate limit is 10 Hz (10 per second)
      for (let i = 0; i < 25; i++) {
        const result = engine.checkPublish('/cmd_vel', {
          linear: { x: 0.1, y: 0, z: 0 },
        });
        results.push(result.allowed);
      }

      const allowedCount = results.filter(r => r).length;
      const blockedCount = results.filter(r => !r).length;

      // First 10 should be allowed (10 Hz), then rate-limited
      expect(allowedCount).toBe(10);
      expect(blockedCount).toBe(15);
    });

    it('parallel-style Promise.all safety checks maintain state consistency', async () => {
      // Simulate concurrent checks using Promise.all with microtask-based async
      const checks = Array.from({ length: 20 }, (_, i) =>
        Promise.resolve().then(() =>
          engine.checkPublish(`/topic_${i}/cmd_vel`, {
            linear: { x: 0.1, y: 0, z: 0 },
          })
        )
      );

      const results = await Promise.all(checks);

      // All distinct topics, all under velocity limit -> all should be allowed
      expect(results.every(r => r.allowed)).toBe(true);

      // Verify audit log captured all
      const stats = engine.getAuditStats();
      expect(stats.total).toBe(20);
    });

    it('mixed allowed and blocked checks interleaved correctly', () => {
      const results: { topic: string; allowed: boolean; violationTypes: string[] }[] = [];

      for (let i = 0; i < 10; i++) {
        // Safe publish
        const safe = engine.checkPublish(`/safe_${i}/cmd_vel`, {
          linear: { x: 0.1, y: 0, z: 0 },
        });
        results.push({ topic: `safe_${i}`, allowed: safe.allowed, violationTypes: safe.violations.map(v => v.type) });

        // Blocked publish (exceeds velocity)
        const blocked = engine.checkPublish(`/fast_${i}/cmd_vel`, {
          linear: { x: 5.0, y: 0, z: 0 },
        });
        results.push({ topic: `fast_${i}`, allowed: blocked.allowed, violationTypes: blocked.violations.map(v => v.type) });

        // Blocked topic
        const blockedTopic = engine.checkPublish('/rosout', { data: 'test' });
        results.push({ topic: '/rosout', allowed: blockedTopic.allowed, violationTypes: blockedTopic.violations.map(v => v.type) });
      }

      // Verify each category
      const safeResults = results.filter(r => r.topic.startsWith('safe_'));
      const fastResults = results.filter(r => r.topic.startsWith('fast_'));
      const rosoutResults = results.filter(r => r.topic === '/rosout');

      expect(safeResults.every(r => r.allowed)).toBe(true);
      expect(fastResults.every(r => !r.allowed)).toBe(true);
      expect(fastResults.every(r => r.violationTypes.includes('velocity_exceeded'))).toBe(true);
      expect(rosoutResults.every(r => !r.allowed)).toBe(true);
    });
  });

  describe('e-stop during active publish check', () => {
    it('activating e-stop between sequential checks blocks subsequent ones', () => {
      // First check should pass
      const before = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
      });
      expect(before.allowed).toBe(true);

      // Activate e-stop
      engine.activateEmergencyStop();

      // All subsequent checks should fail
      for (let i = 0; i < 10; i++) {
        const after = engine.checkPublish(`/topic_${i}/cmd_vel`, {
          linear: { x: 0.01, y: 0, z: 0 },
        });
        expect(after.allowed).toBe(false);
        expect(after.violations[0].type).toBe('emergency_stop_active');
      }
    });

    it('e-stop blocks services and actions simultaneously', () => {
      engine.activateEmergencyStop();

      const pub = engine.checkPublish('/cmd_vel', { linear: { x: 0.1 } });
      const svc = engine.checkServiceCall('/my_service', {});
      const act = engine.checkActionGoal('/navigate', { goal: {} });

      expect(pub.allowed).toBe(false);
      expect(svc.allowed).toBe(false);
      expect(act.allowed).toBe(false);

      expect(pub.violations[0].type).toBe('emergency_stop_active');
      expect(svc.violations[0].type).toBe('emergency_stop_active');
      expect(act.violations[0].type).toBe('emergency_stop_active');
    });

    it('rapid e-stop toggle does not corrupt state', () => {
      for (let i = 0; i < 50; i++) {
        engine.activateEmergencyStop();
        expect(engine.isEmergencyStopActive).toBe(true);

        engine.releaseEmergencyStop();
        expect(engine.isEmergencyStopActive).toBe(false);
      }

      // After toggling, should be in released state
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('e-stop activation is logged in audit trail', () => {
      engine.activateEmergencyStop();
      engine.releaseEmergencyStop();

      const entries = engine.getAuditLog();
      const estopEntries = entries.filter(
        e => e.command === 'emergency_stop' || e.command === 'emergency_stop_release'
      );
      expect(estopEntries.length).toBe(2);
    });

    it('e-stop produces violations alongside other violations', () => {
      engine.activateEmergencyStop();

      // Blocked topic + e-stop
      const result = engine.checkPublish('/rosout', { data: 'test' });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('emergency_stop_active');
      expect(types).toContain('blocked_topic');
    });

    it('e-stop during velocity-exceeded check produces both violations', () => {
      engine.activateEmergencyStop();

      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 10.0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('emergency_stop_active');
      expect(types).toContain('velocity_exceeded');
    });
  });

  describe('concurrent rate limiter access', () => {
    it('rate limiter stays consistent across interleaved publish checks on same topic', () => {
      let allowedCount = 0;

      // Default is 10 Hz, fire 30 checks on the same topic
      for (let i = 0; i < 30; i++) {
        const result = engine.checkPublish('/cmd_vel', {
          linear: { x: 0.1, y: 0, z: 0 },
        });
        if (result.allowed) allowedCount++;
      }

      // Exactly 10 should be allowed
      expect(allowedCount).toBe(10);
    });

    it('rate limiter for services is independent from publish rate limiter', () => {
      // Exhaust publish rate limit on /cmd_vel (10 per second)
      for (let i = 0; i < 15; i++) {
        engine.checkPublish('/cmd_vel', { linear: { x: 0.1 } });
      }

      // Service calls should still work (different rate limiter key)
      const svcResult = engine.checkServiceCall('/my_service', {});
      expect(svcResult.allowed).toBe(true);
    });

    it('rate limiter for actions is independent from both publish and service', () => {
      // Exhaust publish rate limit
      for (let i = 0; i < 15; i++) {
        engine.checkPublish('/cmd_vel', { linear: { x: 0.1 } });
      }

      // Exhaust service rate limit (60 per minute)
      for (let i = 0; i < 65; i++) {
        engine.checkServiceCall('/my_service', {});
      }

      // Action calls should still work
      const actResult = engine.checkActionGoal('/navigate', { goal: {} });
      expect(actResult.allowed).toBe(true);
    });

    it('rate limiting across many distinct topics does not cross-contaminate', () => {
      // Create 20 different topics, each with 8 calls (under 10 Hz limit)
      for (let t = 0; t < 20; t++) {
        for (let c = 0; c < 8; c++) {
          const result = engine.checkPublish(`/topic_${t}`, { data: c });
          expect(result.allowed).toBe(true);
        }
      }

      // Now push one topic over the limit
      for (let c = 0; c < 5; c++) {
        engine.checkPublish('/topic_0', { data: 'extra' });
      }

      // topic_0 should now be rate-limited (8 + 5 = 13 > 10)
      const overLimit = engine.checkPublish('/topic_0', { data: 'one_more' });
      expect(overLimit.allowed).toBe(false);
      expect(overLimit.violations.some(v => v.type === 'rate_limit_exceeded')).toBe(true);

      // Other topics should still be fine
      const otherResult = engine.checkPublish('/topic_1', { data: 'still_ok' });
      expect(otherResult.allowed).toBe(true);
    });
  });

  describe('policy update during active check', () => {
    it('velocity limit change takes effect on the next check', () => {
      // Should be allowed at 0.4 m/s (limit is 0.5)
      const before = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.4, y: 0, z: 0 },
      });
      expect(before.allowed).toBe(true);

      // Lower the limit
      engine.updateVelocityLimits({ linearMax: 0.3 });

      // Same velocity should now be blocked
      const after = engine.checkPublish('/v2/cmd_vel', {
        linear: { x: 0.4, y: 0, z: 0 },
      });
      expect(after.allowed).toBe(false);
      expect(after.violations.some(v => v.type === 'velocity_exceeded')).toBe(true);
    });

    it('raising velocity limit unblocks previously-blocked velocities', () => {
      // Block at default limit (0.5 m/s)
      const blocked = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.8, y: 0, z: 0 },
      });
      expect(blocked.allowed).toBe(false);

      // Raise the limit
      engine.updateVelocityLimits({ linearMax: 1.0 });

      // Same velocity should now be allowed
      const allowed = engine.checkPublish('/v2/cmd_vel', {
        linear: { x: 0.8, y: 0, z: 0 },
      });
      expect(allowed.allowed).toBe(true);
    });

    it('geofence update takes effect immediately for position checks', () => {
      // Default geofence: x [-5, 5]
      const insideBefore = engine.checkPosition({ x: 4.5, y: 0, z: 1 });
      expect(insideBefore.inside).toBe(true);

      // Shrink geofence
      engine.updateGeofence({ xMax: 3 });

      // Same position is now outside
      const outsideAfter = engine.checkPosition({ x: 4.5, y: 0, z: 1 });
      expect(outsideAfter.inside).toBe(false);
      expect(outsideAfter.violation).not.toBeNull();
    });

    it('policy name and limits are reflected in getPolicy after update', () => {
      engine.updateVelocityLimits({ linearMax: 2.0, angularMax: 5.0 });
      engine.updateGeofence({ xMin: -100, xMax: 100 });

      const policy = engine.getPolicy();
      expect(policy.velocity.linearMax).toBe(2.0);
      expect(policy.velocity.angularMax).toBe(5.0);
      expect(policy.geofence.xMin).toBe(-100);
      expect(policy.geofence.xMax).toBe(100);
    });

    it('multiple rapid policy updates do not leave stale state', () => {
      for (let i = 0; i < 50; i++) {
        engine.updateVelocityLimits({ linearMax: 0.1 + i * 0.01 });
      }

      // Final limit should be 0.1 + 49*0.01 = 0.59
      const policy = engine.getPolicy();
      expect(policy.velocity.linearMax).toBeCloseTo(0.59, 5);

      // A velocity of 0.55 should be allowed
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.55, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('audit log records checks made under different policy versions', () => {
      // Check under original policy
      engine.checkPublish('/cmd_vel', { linear: { x: 0.4, y: 0, z: 0 } });

      // Update policy
      engine.updateVelocityLimits({ linearMax: 0.3 });

      // Check under new policy (same velocity, now blocked)
      engine.checkPublish('/v2/cmd_vel', { linear: { x: 0.4, y: 0, z: 0 } });

      const entries = engine.getAuditLog({ limit: 2 });
      // Newest first: the blocked one is first
      expect(entries[0].safetyResult.allowed).toBe(false);
      expect(entries[1].safetyResult.allowed).toBe(true);
    });
  });
});

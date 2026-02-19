/**
 * Safety whitelist mode tests.
 *
 * Tests the allowedTopics and allowedServices whitelist behavior in
 * the PolicyEngine. When these arrays are set on the policy, only
 * matching topics/services are permitted; everything else is blocked
 * as if it were on the blocked list.
 *
 * Implementation reference (policy-engine.ts):
 *   isTopicBlocked: if allowedTopics is set, blocks topics that do NOT
 *     start with any entry in allowedTopics. Otherwise falls back to
 *     blockedTopics list.
 *   isServiceBlocked: same logic with allowedServices / blockedServices.
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Whitelist Mode — allowedTopics', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  describe('when allowedTopics is undefined (default)', () => {
    it('allows any topic not in blockedTopics', () => {
      const result = engine.checkPublish('/my_custom_topic', { data: 'hello' });
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
    });

    it('blocks topics in blockedTopics', () => {
      const result = engine.checkPublish('/rosout', { data: 'test' });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('blocks /parameter_events (in default blockedTopics)', () => {
      const result = engine.checkPublish('/parameter_events', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('allows arbitrary topic names', () => {
      const result = engine.checkPublish('/sensor/lidar', { range: 2.5 });
      expect(result.allowed).toBe(true);
    });
  });

  describe('when allowedTopics is set', () => {
    beforeEach(() => {
      (engine as any).policy.allowedTopics = ['/cmd_vel', '/sensor'];
    });

    it('allows topics that start with an allowed entry', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('allows topics that are prefixed by an allowed entry', () => {
      // "/sensor" is allowed, so "/sensor/lidar" should also pass (startsWith check)
      const result = engine.checkPublish('/sensor/lidar', { range: 2.5 });
      expect(result.allowed).toBe(true);
    });

    it('blocks topics not in allowed list', () => {
      const result = engine.checkPublish('/my_custom_topic', { data: 'hello' });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('blocks topics that only partially match', () => {
      // "/command" does not start with "/cmd_vel" or "/sensor"
      const result = engine.checkPublish('/command', { data: 1 });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('whitelist overrides blockedTopics — blockedTopics is ignored in whitelist mode', () => {
      // /rosout is in blockedTopics by default, but whitelist mode bypasses blockedTopics check.
      // Since /rosout is NOT in allowedTopics, it should be blocked.
      const result = engine.checkPublish('/rosout', { data: 'test' });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('velocity checks still apply even when topic is whitelisted', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 10.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      // Should have both blocked_topic=no (it passes whitelist) and velocity_exceeded
      const types = result.violations.map(v => v.type);
      expect(types).toContain('velocity_exceeded');
    });
  });

  describe('when allowedTopics is an empty array', () => {
    beforeEach(() => {
      (engine as any).policy.allowedTopics = [];
    });

    it('blocks all topics', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('blocks even normally allowed topics', () => {
      const result = engine.checkPublish('/my_topic', { data: 1 });
      expect(result.allowed).toBe(false);
    });

    it('blocks every topic attempted', () => {
      const topics = ['/cmd_vel', '/sensor', '/rosout', '/anything'];
      for (const topic of topics) {
        const result = engine.checkPublish(topic, {});
        expect(result.allowed).toBe(false);
      }
    });
  });

  describe('emergency stop still works in whitelist mode', () => {
    beforeEach(() => {
      (engine as any).policy.allowedTopics = ['/cmd_vel'];
    });

    it('blocks whitelisted topic when e-stop is active', () => {
      engine.activateEmergencyStop();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('emergency_stop_active');
    });
  });
});

describe('Whitelist Mode — allowedServices', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  describe('when allowedServices is undefined (default)', () => {
    it('allows any service not in blockedServices', () => {
      const result = engine.checkServiceCall('/set_parameters', { data: true });
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
    });

    it('blocks services in blockedServices', () => {
      const result = engine.checkServiceCall('/kill', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });

    it('blocks /shutdown (in default blockedServices)', () => {
      const result = engine.checkServiceCall('/shutdown', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });

    it('allows arbitrary service names', () => {
      const result = engine.checkServiceCall('/my_service', { arg: 1 });
      expect(result.allowed).toBe(true);
    });
  });

  describe('when allowedServices is set', () => {
    beforeEach(() => {
      (engine as any).policy.allowedServices = ['/set_parameters', '/spawn'];
    });

    it('allows services that start with an allowed entry', () => {
      const result = engine.checkServiceCall('/set_parameters', { data: true });
      expect(result.allowed).toBe(true);
    });

    it('allows services that are prefixed by an allowed entry', () => {
      // "/spawn" is allowed, so "/spawn_entity" should pass (startsWith)
      const result = engine.checkServiceCall('/spawn_entity', { name: 'robot' });
      expect(result.allowed).toBe(true);
    });

    it('blocks services not in allowed list', () => {
      const result = engine.checkServiceCall('/my_service', { arg: 1 });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });

    it('whitelist overrides blockedServices — blockedServices is ignored in whitelist mode', () => {
      // /kill is in blockedServices by default, but whitelist mode checks allowedServices instead.
      // Since /kill is NOT in allowedServices, it should be blocked.
      const result = engine.checkServiceCall('/kill', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });

    it('blocks services with no prefix match', () => {
      const result = engine.checkServiceCall('/reset', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });
  });

  describe('when allowedServices is an empty array', () => {
    beforeEach(() => {
      (engine as any).policy.allowedServices = [];
    });

    it('blocks all services', () => {
      const result = engine.checkServiceCall('/set_parameters', { data: true });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });

    it('blocks every service attempted', () => {
      const services = ['/set_parameters', '/spawn', '/kill', '/anything'];
      for (const service of services) {
        const result = engine.checkServiceCall(service, {});
        expect(result.allowed).toBe(false);
      }
    });
  });

  describe('emergency stop still works in service whitelist mode', () => {
    beforeEach(() => {
      (engine as any).policy.allowedServices = ['/set_parameters'];
    });

    it('blocks whitelisted service when e-stop is active', () => {
      engine.activateEmergencyStop();
      const result = engine.checkServiceCall('/set_parameters', { data: true });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('emergency_stop_active');
    });
  });
});

describe('Whitelist Mode — combined scenarios', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  describe('allowedTopics set while allowedServices is undefined', () => {
    beforeEach(() => {
      (engine as any).policy.allowedTopics = ['/cmd_vel'];
    });

    it('whitelists topics but uses blockedServices for services', () => {
      // Topic: only /cmd_vel allowed
      const topicAllowed = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(topicAllowed.allowed).toBe(true);

      const topicBlocked = engine.checkPublish('/other', {});
      expect(topicBlocked.allowed).toBe(false);

      // Service: blockedServices still applies, not whitelist
      const serviceAllowed = engine.checkServiceCall('/set_parameters', {});
      expect(serviceAllowed.allowed).toBe(true);

      const serviceBlocked = engine.checkServiceCall('/kill', {});
      expect(serviceBlocked.allowed).toBe(false);
    });
  });

  describe('allowedServices set while allowedTopics is undefined', () => {
    beforeEach(() => {
      (engine as any).policy.allowedServices = ['/set_parameters'];
    });

    it('whitelists services but uses blockedTopics for topics', () => {
      // Service: only /set_parameters allowed
      const serviceAllowed = engine.checkServiceCall('/set_parameters', {});
      expect(serviceAllowed.allowed).toBe(true);

      const serviceBlocked = engine.checkServiceCall('/other', {});
      expect(serviceBlocked.allowed).toBe(false);

      // Topic: blockedTopics still applies, not whitelist
      const topicAllowed = engine.checkPublish('/my_topic', {});
      expect(topicAllowed.allowed).toBe(true);

      const topicBlocked = engine.checkPublish('/rosout', {});
      expect(topicBlocked.allowed).toBe(false);
    });
  });

  describe('both allowedTopics and allowedServices set', () => {
    beforeEach(() => {
      (engine as any).policy.allowedTopics = ['/cmd_vel', '/sensor'];
      (engine as any).policy.allowedServices = ['/set_parameters'];
    });

    it('both whitelists apply independently', () => {
      // Allowed topic
      const t1 = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(t1.allowed).toBe(true);

      // Blocked topic (not in whitelist)
      const t2 = engine.checkPublish('/other', {});
      expect(t2.allowed).toBe(false);

      // Allowed service
      const s1 = engine.checkServiceCall('/set_parameters', {});
      expect(s1.allowed).toBe(true);

      // Blocked service (not in whitelist)
      const s2 = engine.checkServiceCall('/other', {});
      expect(s2.allowed).toBe(false);
    });
  });

  describe('whitelist mode with velocity checks combined', () => {
    beforeEach(() => {
      (engine as any).policy.allowedTopics = ['/cmd_vel'];
    });

    it('allowed topic with safe velocity passes', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
    });

    it('allowed topic with excessive velocity is blocked by velocity check', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 5.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('velocity_exceeded');
      expect(types).not.toContain('blocked_topic');
    });

    it('non-allowed topic is blocked even with safe velocity', () => {
      const result = engine.checkPublish('/other_cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('blocked_topic');
    });

    it('non-allowed topic with excessive velocity gets both violations', () => {
      // /other_cmd_vel contains "cmd_vel" so velocity checks apply,
      // AND it is not in allowedTopics so it gets blocked_topic too
      const result = engine.checkPublish('/other_cmd_vel', {
        linear: { x: 5.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('blocked_topic');
      expect(types).toContain('velocity_exceeded');
    });
  });

  describe('whitelist mode with rate limiting', () => {
    let engine: PolicyEngine;

    beforeEach(() => {
      engine = new PolicyEngine();
      (engine as any).policy.allowedTopics = ['/fast_topic'];
      // Set rate limit very low so we can trigger it
      (engine as any).policy.rateLimits.publishHz = 1;
    });

    it('allowed topic still subject to rate limits', () => {
      // First publish should succeed
      const first = engine.checkPublish('/fast_topic', { data: 1 });
      expect(first.allowed).toBe(true);

      // Rapid second publish should hit rate limit
      const second = engine.checkPublish('/fast_topic', { data: 2 });
      expect(second.allowed).toBe(false);
      const types = second.violations.map(v => v.type);
      expect(types).toContain('rate_limit_exceeded');
    });
  });

  describe('getPolicy reflects whitelist settings', () => {
    it('getPolicy returns allowedTopics when set', () => {
      (engine as any).policy.allowedTopics = ['/cmd_vel'];
      const policy = engine.getPolicy();
      expect(policy.allowedTopics).toEqual(['/cmd_vel']);
    });

    it('getPolicy returns allowedServices when set', () => {
      (engine as any).policy.allowedServices = ['/set_parameters'];
      const policy = engine.getPolicy();
      expect(policy.allowedServices).toEqual(['/set_parameters']);
    });

    it('getPolicy returns undefined for allowedTopics when not set', () => {
      const policy = engine.getPolicy();
      expect(policy.allowedTopics).toBeUndefined();
    });

    it('getPolicy returns undefined for allowedServices when not set', () => {
      const policy = engine.getPolicy();
      expect(policy.allowedServices).toBeUndefined();
    });
  });
});

/**
 * Tests for safety policy inheritance.
 */

import { describe, it, expect } from 'vitest';
import { mergePolicies, buildPolicyChain } from './policy-inheritance.js';
import { getDefaultPolicy } from './policy-loader.js';

describe('Policy Inheritance', () => {
  describe('mergePolicies', () => {
    it('returns base when override is empty', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {});

      expect(result.name).toBe('default');
      expect(result.velocity.linearMax).toBe(0.5);
    });

    it('overrides name and description', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        name: 'turtlebot3',
        description: 'TurtleBot3 specific policy',
      });

      expect(result.name).toBe('turtlebot3');
      expect(result.description).toBe('TurtleBot3 specific policy');
    });

    it('overrides velocity limits partially', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        velocity: { linearMax: 0.22 } as any,
      });

      expect(result.velocity.linearMax).toBe(0.22);
      expect(result.velocity.angularMax).toBe(1.5); // kept from base
      expect(result.velocity.clampMode).toBe(false); // kept from base
    });

    it('overrides geofence partially', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        geofence: { xMax: 10, yMax: 10 } as any,
      });

      expect(result.geofence.xMax).toBe(10);
      expect(result.geofence.yMax).toBe(10);
      expect(result.geofence.xMin).toBe(-5); // kept from base
    });

    it('overrides blocked topics entirely', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        blockedTopics: ['/rosout', '/clock'],
      });

      expect(result.blockedTopics).toEqual(['/rosout', '/clock']);
      expect(result.blockedTopics).not.toContain('/parameter_events');
    });

    it('overrides rate limits partially', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        rateLimits: { publishHz: 20 } as any,
      });

      expect(result.rateLimits.publishHz).toBe(20);
      expect(result.rateLimits.servicePerMinute).toBe(60); // from base
    });

    it('overrides deadman switch', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        deadmanSwitch: { enabled: true, timeoutMs: 10000 },
      });

      expect(result.deadmanSwitch.enabled).toBe(true);
      expect(result.deadmanSwitch.timeoutMs).toBe(10000);
    });

    it('overrides acceleration limits', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        acceleration: { enabled: true, linearMaxAccel: 2.0 } as any,
      });

      expect(result.acceleration.enabled).toBe(true);
      expect(result.acceleration.linearMaxAccel).toBe(2.0);
      expect(result.acceleration.angularMaxAccel).toBe(3.0); // from base
    });

    it('overrides geofence warning margin', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        geofenceWarningMargin: 2.0,
      });

      expect(result.geofenceWarningMargin).toBe(2.0);
    });

    it('overrides command approval', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        commandApproval: { enabled: true, requireApprovalFor: ['ros2_topic_publish'], pendingTimeout: 5000 },
      });

      expect(result.commandApproval.enabled).toBe(true);
      expect(result.commandApproval.requireApprovalFor).toContain('ros2_topic_publish');
    });

    it('sets allowed topics', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        allowedTopics: ['/cmd_vel', '/odom'],
      });

      expect(result.allowedTopics).toEqual(['/cmd_vel', '/odom']);
    });

    it('preserves base allowed topics when not overridden', () => {
      const base = getDefaultPolicy();
      base.allowedTopics = ['/cmd_vel'];
      const result = mergePolicies(base, {
        name: 'override-without-allowed',
      });

      expect(result.allowedTopics).toEqual(['/cmd_vel']);
    });

    it('does not mutate base policy', () => {
      const base = getDefaultPolicy();
      const originalMax = base.velocity.linearMax;

      mergePolicies(base, {
        velocity: { linearMax: 10.0 } as any,
      });

      expect(base.velocity.linearMax).toBe(originalMax);
    });

    it('adds topic velocity overrides', () => {
      const base = getDefaultPolicy();
      const result = mergePolicies(base, {
        topicVelocityOverrides: [
          { topic: '/arm/cmd_vel', linearMax: 0.1 },
        ],
      });

      expect(result.topicVelocityOverrides).toHaveLength(1);
      expect(result.topicVelocityOverrides![0].linearMax).toBe(0.1);
    });
  });

  describe('buildPolicyChain', () => {
    it('applies layers in order', () => {
      const result = buildPolicyChain([
        { name: 'base', velocity: { linearMax: 1.0 } as any },
        { velocity: { linearMax: 0.5 } as any },
        { velocity: { linearMax: 0.3 } as any },
      ]);

      expect(result.velocity.linearMax).toBe(0.3); // last layer wins
    });

    it('returns default for empty layers', () => {
      const result = buildPolicyChain([]);
      expect(result.name).toBe('default');
    });

    it('builds a realistic policy chain', () => {
      const result = buildPolicyChain([
        // Layer 1: Base conservative policy
        {
          name: 'conservative',
          velocity: { linearMax: 0.3, angularMax: 1.0, clampMode: false },
          geofence: { xMin: -2, xMax: 2, yMin: -2, yMax: 2, zMin: 0, zMax: 1.5 },
        },
        // Layer 2: Robot-specific overrides
        {
          name: 'turtlebot3-conservative',
          velocity: { linearMax: 0.22 } as any, // TurtleBot3 max is 0.22
          deadmanSwitch: { enabled: true, timeoutMs: 15000 },
        },
        // Layer 3: Environment-specific overrides
        {
          geofenceWarningMargin: 0.5,
          blockedTopics: ['/rosout', '/parameter_events', '/clock'],
        },
      ]);

      expect(result.name).toBe('turtlebot3-conservative');
      expect(result.velocity.linearMax).toBe(0.22);
      expect(result.velocity.angularMax).toBe(1.0); // from layer 1
      expect(result.deadmanSwitch.enabled).toBe(true);
      expect(result.geofenceWarningMargin).toBe(0.5);
      expect(result.blockedTopics).toContain('/clock');
    });
  });
});

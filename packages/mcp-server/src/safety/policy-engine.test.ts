import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('PolicyEngine', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  describe('checkPublish', () => {
    it('allows safe velocity commands', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0.5 },
      });
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
    });

    it('blocks linear velocity exceeding limit', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 5.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('velocity_exceeded');
      expect(result.violations[0].message).toContain('Linear velocity');
    });

    it('blocks angular velocity exceeding limit', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 10.0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('velocity_exceeded');
      expect(result.violations[0].message).toContain('Angular velocity');
    });

    it('checks velocity magnitude not individual components', () => {
      // 0.4 on x and 0.4 on y = sqrt(0.16 + 0.16) = 0.566 > 0.5 limit
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.4, y: 0.4, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
    });

    it('blocks publish to blocked topics', () => {
      const result = engine.checkPublish('/rosout', { data: 'test' });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('blocks publish to /parameter_events', () => {
      const result = engine.checkPublish('/parameter_events', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_topic');
    });

    it('allows publish to non-blocked topics', () => {
      const result = engine.checkPublish('/my_custom_topic', { data: 'hello' });
      expect(result.allowed).toBe(true);
    });

    it('does not check velocity on non-cmd_vel topics', () => {
      const result = engine.checkPublish('/other_topic', {
        linear: { x: 100, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });
  });

  describe('checkServiceCall', () => {
    it('allows calls to non-blocked services', () => {
      const result = engine.checkServiceCall('/set_parameters', { data: true });
      expect(result.allowed).toBe(true);
    });

    it('blocks calls to /kill', () => {
      const result = engine.checkServiceCall('/kill', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });

    it('blocks calls to /shutdown', () => {
      const result = engine.checkServiceCall('/shutdown', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('blocked_service');
    });
  });

  describe('checkActionGoal', () => {
    it('allows action goals when no violations', () => {
      const result = engine.checkActionGoal('/navigate_to_pose', {
        pose: { x: 1.0, y: 2.0 },
      });
      expect(result.allowed).toBe(true);
    });
  });

  describe('emergency stop', () => {
    it('blocks all publishes when e-stop active', () => {
      engine.activateEmergencyStop();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('emergency_stop_active');
    });

    it('blocks all service calls when e-stop active', () => {
      engine.activateEmergencyStop();
      const result = engine.checkServiceCall('/set_parameters', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('emergency_stop_active');
    });

    it('blocks all action goals when e-stop active', () => {
      engine.activateEmergencyStop();
      const result = engine.checkActionGoal('/navigate_to_pose', {});
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('emergency_stop_active');
    });

    it('allows commands after e-stop release', () => {
      engine.activateEmergencyStop();
      expect(engine.isEmergencyStopActive).toBe(true);

      engine.releaseEmergencyStop();
      expect(engine.isEmergencyStopActive).toBe(false);

      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });
  });

  describe('policy management', () => {
    it('returns current policy', () => {
      const policy = engine.getPolicy();
      expect(policy.name).toBe('default');
      expect(policy.velocity.linearMax).toBe(0.5);
      expect(policy.velocity.angularMax).toBe(1.5);
    });

    it('updates velocity limits', () => {
      engine.updateVelocityLimits({ linearMax: 0.3 });
      const policy = engine.getPolicy();
      expect(policy.velocity.linearMax).toBe(0.3);
      expect(policy.velocity.angularMax).toBe(1.5); // unchanged
    });

    it('enforces updated velocity limits', () => {
      engine.updateVelocityLimits({ linearMax: 0.1 });
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.2, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
    });

    it('updates geofence bounds', () => {
      engine.updateGeofence({ xMin: -1, xMax: 1 });
      const policy = engine.getPolicy();
      expect(policy.geofence.xMin).toBe(-1);
      expect(policy.geofence.xMax).toBe(1);
      expect(policy.geofence.yMin).toBe(-5); // unchanged
    });
  });

  describe('status', () => {
    it('returns complete status', () => {
      const status = engine.getStatus();
      expect(status.policyName).toBe('default');
      expect(status.emergencyStopActive).toBe(false);
      expect(status.velocity).toBeDefined();
      expect(status.geofence).toBeDefined();
      expect(status.rateLimits).toBeDefined();
      expect(status.auditStats).toBeDefined();
    });

    it('reflects e-stop state in status', () => {
      engine.activateEmergencyStop();
      expect(engine.getStatus().emergencyStopActive).toBe(true);
    });
  });
});

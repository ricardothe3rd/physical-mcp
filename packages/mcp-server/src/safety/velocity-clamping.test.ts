import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Velocity Clamping', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  describe('when clampMode is disabled (default)', () => {
    it('blocks over-limit linear velocity', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 2.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('velocity_exceeded');
    });

    it('blocks over-limit angular velocity', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 5.0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('velocity_exceeded');
    });
  });

  describe('when clampMode is enabled', () => {
    beforeEach(() => {
      engine.updateVelocityLimits({ clampMode: true });
    });

    it('allows and clamps over-limit linear velocity', () => {
      const message = {
        linear: { x: 2.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
      const result = engine.checkPublish('/cmd_vel', message);

      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(1);
      expect(result.violations[0].type).toBe('velocity_clamped');
      expect(result.violations[0].message).toContain('clamped');

      // Message should be modified in-place
      const linear = message.linear;
      const speed = Math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2);
      expect(speed).toBeCloseTo(0.5, 4);
    });

    it('clamps multi-axis linear velocity correctly', () => {
      const message = {
        linear: { x: 3.0, y: 4.0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
      const result = engine.checkPublish('/cmd_vel', message);

      expect(result.allowed).toBe(true);
      expect(result.violations[0].type).toBe('velocity_clamped');

      const linear = message.linear;
      const speed = Math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2);
      expect(speed).toBeCloseTo(0.5, 4);

      // Direction should be preserved (3:4 ratio)
      expect(linear.x / linear.y).toBeCloseTo(3 / 4, 4);
    });

    it('allows and clamps over-limit angular velocity', () => {
      const message = {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 5.0 },
      };
      const result = engine.checkPublish('/cmd_vel', message);

      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(1);
      expect(result.violations[0].type).toBe('velocity_clamped');

      const angular = message.angular;
      const rate = Math.sqrt(angular.x ** 2 + angular.y ** 2 + angular.z ** 2);
      expect(rate).toBeCloseTo(1.5, 4);
    });

    it('does not clamp velocities within limits', () => {
      const message = {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0.5 },
      };
      const result = engine.checkPublish('/cmd_vel', message);

      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
      expect(message.linear.x).toBe(0.1);
    });

    it('does not clamp velocities exactly at the limit', () => {
      const message = {
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
      const result = engine.checkPublish('/cmd_vel', message);

      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
    });

    it('can be toggled at runtime', () => {
      // Clamp mode is on — should allow with clamping
      const msg1 = {
        linear: { x: 2.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
      expect(engine.checkPublish('/cmd_vel', msg1).allowed).toBe(true);

      // Turn clamp mode off — should block
      engine.updateVelocityLimits({ clampMode: false });
      const msg2 = {
        linear: { x: 2.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
      expect(engine.checkPublish('/cmd_vel', msg2).allowed).toBe(false);
    });
  });
});

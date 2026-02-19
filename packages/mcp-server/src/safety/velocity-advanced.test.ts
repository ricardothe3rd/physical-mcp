/**
 * Advanced velocity checking tests: diagonal movement, 3D, edge cases.
 */

import { describe, it, expect } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Advanced Velocity Checks', () => {
  describe('diagonal movement', () => {
    it('allows diagonal movement within combined limit', () => {
      const engine = new PolicyEngine();
      // sqrt(0.3^2 + 0.3^2) ≈ 0.424, which is < 0.5 (default linearMax)
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.3, y: 0.3, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('blocks diagonal movement exceeding combined limit', () => {
      const engine = new PolicyEngine();
      // sqrt(0.4^2 + 0.4^2) ≈ 0.566, which exceeds 0.5 (default linearMax)
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.4, y: 0.4, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('velocity_exceeded');
    });

    it('checks 3D diagonal movement', () => {
      const engine = new PolicyEngine();
      // sqrt(0.3^2 + 0.3^2 + 0.3^2) ≈ 0.520, which exceeds 0.5
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.3, y: 0.3, z: 0.3 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('velocity_exceeded');
    });

    it('allows pure Y axis movement within limit', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0.4, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('allows pure Z axis movement within limit', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0.4 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });
  });

  describe('angular velocity multi-axis', () => {
    it('blocks combined angular velocity exceeding limit', () => {
      const engine = new PolicyEngine();
      // sqrt(1.0^2 + 1.0^2 + 1.0^2) ≈ 1.732, which exceeds 1.5 (default angularMax)
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 1.0, y: 1.0, z: 1.0 },
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('velocity_exceeded');
    });

    it('allows combined angular velocity within limit', () => {
      const engine = new PolicyEngine();
      // sqrt(0.5^2 + 0.5^2 + 0.5^2) ≈ 0.866, which is < 1.5
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0.5, y: 0.5, z: 0.5 },
      });
      expect(result.allowed).toBe(true);
    });

    it('allows pure X axis angular within limit', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 1.4, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });
  });

  describe('edge cases', () => {
    it('allows zero velocity', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('allows message without linear or angular fields', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        data: 'something else',
      });
      expect(result.allowed).toBe(true);
    });

    it('handles negative velocities correctly', () => {
      const engine = new PolicyEngine();
      // sqrt((-0.3)^2 + (-0.3)^2) ≈ 0.424, within limit
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: -0.3, y: -0.3, z: 0 },
        angular: { x: 0, y: 0, z: -1.0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('blocks negative velocities that exceed limit', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: -5.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
    });

    it('handles velocity exactly at limit', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles velocity just barely over limit', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.501, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
    });

    it('handles angular velocity exactly at limit', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 1.5 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles missing axes (partial linear)', () => {
      const engine = new PolicyEngine();
      // When only x is provided, y and z default to 0
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.3 },
        angular: { z: 0.5 },
      });
      expect(result.allowed).toBe(true);
    });

    it('non cmd_vel topics skip velocity checks', () => {
      const engine = new PolicyEngine();
      const result = engine.checkPublish('/other_topic', {
        linear: { x: 100, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 100 },
      });
      expect(result.allowed).toBe(true);
    });
  });

  describe('combined linear + angular violations', () => {
    it('reports linear violation even with angular also over', () => {
      const engine = new PolicyEngine();
      // Linear exceeds limit, angular also exceeds, but only first violation reported
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 5.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 10.0 },
      });
      expect(result.allowed).toBe(false);
      // Only the first violation (linear) is reported since the check returns early
      expect(result.violations.some(v => v.type === 'velocity_exceeded')).toBe(true);
    });
  });
});

/**
 * Edge case tests for safety system.
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Safety Edge Cases', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  describe('velocity checks', () => {
    it('handles zero velocity correctly', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles velocity with only angular component', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 1.0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles velocity with only angular exceeding limit', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 2.0 },
      });
      expect(result.allowed).toBe(false);
    });

    it('handles diagonal movement (3D velocity)', () => {
      // sqrt(0.3^2 + 0.3^2 + 0.3^2) = sqrt(0.27) ≈ 0.52 > 0.5
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.3, y: 0.3, z: 0.3 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
    });

    it('handles velocity at exact limit', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles negative velocity values', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: -0.3, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -0.5 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles missing linear field', () => {
      const result = engine.checkPublish('/cmd_vel', {
        angular: { x: 0, y: 0, z: 0.5 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles missing angular field', () => {
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 0.3, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(true);
    });

    it('handles empty message on cmd_vel', () => {
      const result = engine.checkPublish('/cmd_vel', {});
      expect(result.allowed).toBe(true);
    });

    it('checks velocity on namespaced cmd_vel topics', () => {
      const result = engine.checkPublish('/robot1/cmd_vel', {
        linear: { x: 5.0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
    });
  });

  describe('blocked topic matching', () => {
    it('blocks exact match', () => {
      const result = engine.checkPublish('/rosout', { data: 'test' });
      expect(result.allowed).toBe(false);
    });

    it('blocks prefix match', () => {
      const result = engine.checkPublish('/rosout/agg', { data: 'test' });
      expect(result.allowed).toBe(false);
    });

    it('does not block partial name match', () => {
      const result = engine.checkPublish('/my_rosout', { data: 'test' });
      expect(result.allowed).toBe(true);
    });
  });

  describe('multiple violations', () => {
    it('reports e-stop + blocked topic simultaneously', () => {
      engine.activateEmergencyStop();
      const result = engine.checkPublish('/rosout', { data: 'test' });
      expect(result.allowed).toBe(false);
      expect(result.violations.length).toBeGreaterThanOrEqual(2);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('emergency_stop_active');
      expect(types).toContain('blocked_topic');
    });

    it('reports e-stop + velocity violation simultaneously', () => {
      engine.activateEmergencyStop();
      const result = engine.checkPublish('/cmd_vel', {
        linear: { x: 10, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      expect(result.allowed).toBe(false);
      const types = result.violations.map(v => v.type);
      expect(types).toContain('emergency_stop_active');
      expect(types).toContain('velocity_exceeded');
    });
  });

  describe('concurrent operations', () => {
    it('handles rapid sequential publish checks', () => {
      for (let i = 0; i < 100; i++) {
        const result = engine.checkPublish('/cmd_vel', {
          linear: { x: 0.1, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
        // First 10 should pass (10 Hz limit), then rate limited
        if (i < 10) {
          expect(result.allowed).toBe(true);
        }
      }
    });

    it('handles rapid sequential service checks', () => {
      // 60 per minute = should allow 60 calls
      for (let i = 0; i < 70; i++) {
        engine.checkServiceCall('/my_service', {});
      }
      // Just ensure no crashes — rate limiting may kick in
    });
  });

  describe('audit log integrity', () => {
    it('logs every check regardless of result', () => {
      engine.checkPublish('/cmd_vel', { linear: { x: 0.1 } }); // allowed
      engine.checkPublish('/rosout', {}); // blocked
      engine.checkServiceCall('/my_service', {}); // allowed
      engine.checkServiceCall('/kill', {}); // blocked

      const stats = engine.getAuditStats();
      expect(stats.total).toBe(4);
      expect(stats.blocked).toBe(2);
    });
  });
});

/**
 * Tests for per-topic velocity limits.
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Per-Topic Velocity Limits', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  it('uses default limits when no overrides configured', () => {
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0.3, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);
  });

  it('uses default limits and blocks exceeding velocity', () => {
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 1.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(false);
    expect(result.violations[0].type).toBe('velocity_exceeded');
  });

  it('applies per-topic override with higher limit', () => {
    // Set an override allowing 2.0 m/s for arm cmd_vel
    const policy = engine.getPolicy();
    policy.topicVelocityOverrides = [
      { topic: '/arm/cmd_vel', linearMax: 2.0, angularMax: 3.0 },
    ];
    // We need to update the policy internal state
    engine.updateVelocityLimits({}); // Trigger recalculation if needed

    // Create a new engine with the override (since we can't easily mutate internal policy)
    const engine2 = new PolicyEngine();
    // Access internal policy to set overrides for testing
    (engine2 as any).policy.topicVelocityOverrides = [
      { topic: '/arm/cmd_vel', linearMax: 2.0, angularMax: 3.0 },
    ];

    // 1.5 m/s should pass on /arm/cmd_vel (limit is 2.0) but fail on /cmd_vel (limit is 0.5)
    const armResult = engine2.checkPublish('/arm/cmd_vel', {
      linear: { x: 1.5, y: 0, z: 0 },
    });
    expect(armResult.allowed).toBe(true);

    const baseResult = engine2.checkPublish('/cmd_vel', {
      linear: { x: 1.5, y: 0, z: 0 },
    });
    expect(baseResult.allowed).toBe(false);
  });

  it('applies per-topic override with lower limit', () => {
    const engine2 = new PolicyEngine();
    (engine2 as any).policy.topicVelocityOverrides = [
      { topic: '/slow_robot/cmd_vel', linearMax: 0.1, angularMax: 0.5 },
    ];

    // 0.3 m/s should fail on /slow_robot/cmd_vel (limit is 0.1) but pass on /cmd_vel (limit is 0.5)
    const slowResult = engine2.checkPublish('/slow_robot/cmd_vel', {
      linear: { x: 0.3, y: 0, z: 0 },
    });
    expect(slowResult.allowed).toBe(false);

    const baseResult = engine2.checkPublish('/cmd_vel', {
      linear: { x: 0.3, y: 0, z: 0 },
    });
    expect(baseResult.allowed).toBe(true);
  });

  it('applies angular override', () => {
    const engine2 = new PolicyEngine();
    (engine2 as any).policy.topicVelocityOverrides = [
      { topic: '/arm/cmd_vel', angularMax: 5.0 },
    ];

    // 3.0 rad/s should pass on /arm/cmd_vel (limit is 5.0) but fail on /cmd_vel (limit is 1.5)
    const armResult = engine2.checkPublish('/arm/cmd_vel', {
      angular: { x: 0, y: 0, z: 3.0 },
    });
    expect(armResult.allowed).toBe(true);

    const baseResult = engine2.checkPublish('/cmd_vel', {
      angular: { x: 0, y: 0, z: 3.0 },
    });
    expect(baseResult.allowed).toBe(false);
  });

  it('only overrides specified fields', () => {
    const engine2 = new PolicyEngine();
    (engine2 as any).policy.topicVelocityOverrides = [
      { topic: '/custom/cmd_vel', linearMax: 3.0 },
      // angularMax not specified, should use default (1.5)
    ];

    // High angular on /custom/cmd_vel should still use default angular limit
    const result = engine2.checkPublish('/custom/cmd_vel', {
      angular: { x: 0, y: 0, z: 2.0 },
    });
    expect(result.allowed).toBe(false);
    expect(result.violations[0].type).toBe('velocity_exceeded');
  });

  it('handles empty overrides array', () => {
    const engine2 = new PolicyEngine();
    (engine2 as any).policy.topicVelocityOverrides = [];

    const result = engine2.checkPublish('/cmd_vel', {
      linear: { x: 0.3, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);
  });

  it('handles no overrides (undefined)', () => {
    const engine2 = new PolicyEngine();
    (engine2 as any).policy.topicVelocityOverrides = undefined;

    const result = engine2.checkPublish('/cmd_vel', {
      linear: { x: 0.3, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);
  });

  it('matches topic by substring', () => {
    const engine2 = new PolicyEngine();
    (engine2 as any).policy.topicVelocityOverrides = [
      { topic: '/robot1', linearMax: 1.0 },
    ];

    // /robot1/cmd_vel should match the /robot1 override
    const result = engine2.checkPublish('/robot1/cmd_vel', {
      linear: { x: 0.8, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true); // limit is 1.0, speed is 0.8
  });
});

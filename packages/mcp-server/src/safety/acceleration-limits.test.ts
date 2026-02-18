import { describe, it, expect, beforeEach, vi } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Acceleration Limits', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
    // Enable acceleration limits with known values
    engine.updateAccelerationLimits({
      enabled: true,
      linearMaxAccel: 1.0,   // 1 m/s²
      angularMaxAccel: 3.0,  // 3 rad/s²
    });
  });

  it('allows first command without acceleration check (no previous state)', () => {
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);
  });

  it('allows gradual velocity increase within acceleration limit', () => {
    vi.useFakeTimers();

    // First command: 0 → 0.3 m/s
    engine.checkPublish('/cmd_vel', {
      linear: { x: 0.3, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });

    // 500ms later: 0.3 → 0.5 m/s (accel = 0.4 m/s², under 1.0 limit)
    vi.advanceTimersByTime(500);
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);

    vi.useRealTimers();
  });

  it('blocks sudden velocity jump exceeding linear acceleration limit', () => {
    vi.useFakeTimers();

    // First command: 0 m/s
    engine.checkPublish('/cmd_vel', {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });

    // 100ms later: 0 → 0.5 m/s (accel = 5.0 m/s², exceeds 1.0 limit)
    vi.advanceTimersByTime(100);
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(false);
    const accelViolation = result.violations.find(v => v.type === 'acceleration_exceeded');
    expect(accelViolation).toBeDefined();
    expect(accelViolation!.message).toContain('linear accel');

    vi.useRealTimers();
  });

  it('blocks sudden angular velocity change exceeding limit', () => {
    vi.useFakeTimers();

    // First command: 0 rad/s
    engine.checkPublish('/cmd_vel', {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 1.0 },
    });

    // 100ms later: 1.0 → 1.5 rad/s (accel = 5.0 rad/s², exceeds 3.0 limit)
    vi.advanceTimersByTime(100);
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 1.5 },
    });
    expect(result.allowed).toBe(false);
    const accelViolation = result.violations.find(v => v.type === 'acceleration_exceeded');
    expect(accelViolation).toBeDefined();
    expect(accelViolation!.message).toContain('angular accel');

    vi.useRealTimers();
  });

  it('does not check acceleration when disabled', () => {
    vi.useFakeTimers();

    engine.updateAccelerationLimits({ enabled: false });

    engine.checkPublish('/cmd_vel', {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });

    vi.advanceTimersByTime(100);
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);
    expect(result.violations.find(v => v.type === 'acceleration_exceeded')).toBeUndefined();

    vi.useRealTimers();
  });

  it('resets tracking after long gap (> 2 seconds)', () => {
    vi.useFakeTimers();

    engine.checkPublish('/cmd_vel', {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });

    // 3 seconds later — gap too large, should not check acceleration
    vi.advanceTimersByTime(3000);
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);

    vi.useRealTimers();
  });

  it('allows deceleration within limits', () => {
    vi.useFakeTimers();

    // Start at 0.5 m/s
    engine.checkPublish('/cmd_vel', {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });

    // 1 second later: 0.5 → 0 m/s (decel = 0.5 m/s², under 1.0 limit)
    vi.advanceTimersByTime(1000);
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(true);

    vi.useRealTimers();
  });

  it('blocks sudden deceleration exceeding limit', () => {
    vi.useFakeTimers();

    // Start at 0.5 m/s
    engine.checkPublish('/cmd_vel', {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });

    // 50ms later: 0.5 → 0 m/s (decel = 10.0 m/s², exceeds 1.0 limit)
    vi.advanceTimersByTime(50);
    const result = engine.checkPublish('/cmd_vel', {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    expect(result.allowed).toBe(false);

    vi.useRealTimers();
  });

  it('can update acceleration limits at runtime', () => {
    engine.updateAccelerationLimits({ linearMaxAccel: 10.0 });
    const policy = engine.getPolicy();
    expect(policy.acceleration.linearMaxAccel).toBe(10.0);
    expect(policy.acceleration.enabled).toBe(true);
  });

  it('reports acceleration in status', () => {
    const status = engine.getStatus();
    expect(status.acceleration).toBeDefined();
    expect(status.acceleration.enabled).toBe(true);
    expect(status.acceleration.linearMaxAccel).toBe(1.0);
  });
});

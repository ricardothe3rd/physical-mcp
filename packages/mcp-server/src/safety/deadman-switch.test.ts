import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Deadman Switch', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    vi.useFakeTimers();
    engine = new PolicyEngine();
  });

  afterEach(() => {
    engine.destroy();
    vi.useRealTimers();
  });

  it('is disabled by default', () => {
    const status = engine.getStatus();
    expect(status.deadmanSwitch.enabled).toBe(false);
  });

  it('can be enabled at runtime', () => {
    engine.updateDeadmanSwitch({ enabled: true, timeoutMs: 5000 });
    const status = engine.getStatus();
    expect(status.deadmanSwitch.enabled).toBe(true);
    expect(status.deadmanSwitch.timeoutMs).toBe(5000);
  });

  it('triggers e-stop after timeout with no heartbeat', () => {
    engine.updateDeadmanSwitch({ enabled: true, timeoutMs: 5000 });
    expect(engine.isEmergencyStopActive).toBe(false);

    // checkInterval = max(1000, 5000/3) ≈ 1666ms
    // Need to advance past timeout + one check interval to ensure the timer fires after timeout
    vi.advanceTimersByTime(7000);

    expect(engine.isEmergencyStopActive).toBe(true);
  });

  it('does not trigger e-stop if heartbeat is sent', () => {
    engine.updateDeadmanSwitch({ enabled: true, timeoutMs: 5000 });

    // Send heartbeat before timeout
    vi.advanceTimersByTime(4000);
    engine.heartbeat();

    // 4000ms since start, but only 0ms since last heartbeat
    vi.advanceTimersByTime(4000);
    // 4000ms since last heartbeat — still within 5000ms timeout
    expect(engine.isEmergencyStopActive).toBe(false);

    // But if we stop sending heartbeats and wait long enough...
    vi.advanceTimersByTime(7000);
    expect(engine.isEmergencyStopActive).toBe(true);
  });

  it('heartbeat resets the timer', () => {
    engine.updateDeadmanSwitch({ enabled: true, timeoutMs: 5000 });

    // Send heartbeats repeatedly
    for (let i = 0; i < 10; i++) {
      vi.advanceTimersByTime(4000);
      engine.heartbeat();
    }

    expect(engine.isEmergencyStopActive).toBe(false);
  });

  it('can be disabled after being enabled', () => {
    engine.updateDeadmanSwitch({ enabled: true, timeoutMs: 5000 });
    engine.updateDeadmanSwitch({ enabled: false });

    // Advance well past timeout — should NOT trigger
    vi.advanceTimersByTime(60000);
    expect(engine.isEmergencyStopActive).toBe(false);
  });

  it('does not trigger e-stop twice if already active', () => {
    engine.updateDeadmanSwitch({ enabled: true, timeoutMs: 5000 });

    vi.advanceTimersByTime(7000);
    expect(engine.isEmergencyStopActive).toBe(true);

    // Release e-stop and send heartbeat
    engine.releaseEmergencyStop();
    engine.heartbeat();
    expect(engine.isEmergencyStopActive).toBe(false);

    // Should trigger again after timeout
    vi.advanceTimersByTime(7000);
    expect(engine.isEmergencyStopActive).toBe(true);
  });
});

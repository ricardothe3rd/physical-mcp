import { describe, it, expect } from 'vitest';
import { getHealthStatus, type BridgeInfo, type SafetyInfo, type HealthStatus } from './health-check.js';

function makeBridge(connected: boolean): BridgeInfo {
  return { isConnected: connected };
}

function makeSafety(estop: boolean, blocked: number): SafetyInfo {
  return {
    isEmergencyStopActive: estop,
    getAuditStats: () => ({ blocked }),
  };
}

describe('getHealthStatus', () => {
  it('returns healthy when bridge connected and no estop', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 0));
    expect(result.status).toBe('healthy');
    expect(result.bridge.connected).toBe(true);
    expect(result.safety.estopActive).toBe(false);
    expect(result.safety.violationCount).toBe(0);
  });

  it('returns unhealthy when bridge is disconnected', () => {
    const result = getHealthStatus(makeBridge(false), makeSafety(false, 0));
    expect(result.status).toBe('unhealthy');
    expect(result.bridge.connected).toBe(false);
  });

  it('returns degraded when estop is active', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(true, 3));
    expect(result.status).toBe('degraded');
    expect(result.safety.estopActive).toBe(true);
    expect(result.safety.violationCount).toBe(3);
  });

  it('returns unhealthy over degraded when bridge disconnected and estop active', () => {
    const result = getHealthStatus(makeBridge(false), makeSafety(true, 5));
    expect(result.status).toBe('unhealthy');
  });

  it('includes bridge latency when provided', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 0), 42);
    expect(result.bridge.latencyMs).toBe(42);
  });

  it('sets bridge latency to null when not provided', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 0));
    expect(result.bridge.latencyMs).toBeNull();
  });

  it('includes memory usage as positive numbers', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 0));
    expect(result.memory.heapUsedMB).toBeGreaterThan(0);
    expect(result.memory.heapTotalMB).toBeGreaterThan(0);
    expect(result.memory.heapTotalMB).toBeGreaterThanOrEqual(result.memory.heapUsedMB);
  });

  it('includes a timestamp close to now', () => {
    const before = Date.now();
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 0));
    const after = Date.now();
    expect(result.timestamp).toBeGreaterThanOrEqual(before);
    expect(result.timestamp).toBeLessThanOrEqual(after);
  });

  it('includes non-negative uptime', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 0));
    expect(result.uptime).toBeGreaterThanOrEqual(0);
  });

  it('reports violation count from audit stats', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 42));
    expect(result.safety.violationCount).toBe(42);
  });

  it('matches the HealthStatus interface shape', () => {
    const result = getHealthStatus(makeBridge(true), makeSafety(false, 0));
    // Verify all required keys are present
    const keys: (keyof HealthStatus)[] = ['status', 'uptime', 'bridge', 'safety', 'memory', 'timestamp'];
    for (const key of keys) {
      expect(result).toHaveProperty(key);
    }
    expect(result.bridge).toHaveProperty('connected');
    expect(result.bridge).toHaveProperty('latencyMs');
    expect(result.safety).toHaveProperty('estopActive');
    expect(result.safety).toHaveProperty('violationCount');
    expect(result.memory).toHaveProperty('heapUsedMB');
    expect(result.memory).toHaveProperty('heapTotalMB');
  });
});

/**
 * Runtime health status for the PhysicalMCP server.
 *
 * Provides a snapshot of server health including bridge connectivity,
 * safety state, and memory usage.
 */

export interface HealthStatus {
  status: 'healthy' | 'degraded' | 'unhealthy';
  uptime: number;
  bridge: { connected: boolean; latencyMs: number | null };
  safety: { estopActive: boolean; violationCount: number };
  memory: { heapUsedMB: number; heapTotalMB: number };
  timestamp: number;
}

/** Minimal interface for the bridge connection — matches ConnectionManager. */
export interface BridgeInfo {
  isConnected: boolean;
}

/** Minimal interface for safety engine status — matches PolicyEngine. */
export interface SafetyInfo {
  isEmergencyStopActive: boolean;
  getAuditStats(): { blocked: number };
}

const startTime = Date.now();

/**
 * Build a health status snapshot from the current server state.
 *
 * @param bridge - Object with `isConnected` (e.g., ConnectionManager)
 * @param safety - Object with `isEmergencyStopActive` and `getAuditStats()` (e.g., PolicyEngine)
 * @param bridgeLatencyMs - Optional measured round-trip latency to the bridge in ms
 * @returns A {@link HealthStatus} snapshot
 */
export function getHealthStatus(
  bridge: BridgeInfo,
  safety: SafetyInfo,
  bridgeLatencyMs: number | null = null,
): HealthStatus {
  const mem = process.memoryUsage();
  const heapUsedMB = Math.round((mem.heapUsed / 1024 / 1024) * 100) / 100;
  const heapTotalMB = Math.round((mem.heapTotal / 1024 / 1024) * 100) / 100;

  const bridgeConnected = bridge.isConnected;
  const estopActive = safety.isEmergencyStopActive;
  const violationCount = safety.getAuditStats().blocked;

  let status: HealthStatus['status'] = 'healthy';
  if (!bridgeConnected) {
    status = 'unhealthy';
  } else if (estopActive) {
    status = 'degraded';
  }

  return {
    status,
    uptime: Date.now() - startTime,
    bridge: { connected: bridgeConnected, latencyMs: bridgeLatencyMs },
    safety: { estopActive, violationCount },
    memory: { heapUsedMB, heapTotalMB },
    timestamp: Date.now(),
  };
}

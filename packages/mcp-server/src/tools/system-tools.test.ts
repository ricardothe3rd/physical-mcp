/**
 * Tests for system tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { handleSystemTool } from './system-tools.js';

function createMockConnection(overrides: Record<string, unknown> = {}) {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
    ...overrides,
  } as any;
}

function createMockSafety(overrides: Record<string, unknown> = {}) {
  return {
    isEmergencyStopActive: false,
    getAuditStats: vi.fn().mockReturnValue({ total: 10, allowed: 8, blocked: 2, errors: 0 }),
    ...overrides,
  } as any;
}

describe('System Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;
  let safety: ReturnType<typeof createMockSafety>;

  beforeEach(() => {
    connection = createMockConnection();
    safety = createMockSafety();
  });

  describe('system_bridge_status', () => {
    it('returns connected status with latency', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: { version: '0.1.0' }, timestamp: Date.now() });

      const result = await handleSystemTool('system_bridge_status', {}, connection, safety);
      expect(result.isError).toBeUndefined();
      const data = JSON.parse(result.content[0].text);
      expect(data.connected).toBe(true);
      expect(data.available).toBe(true);
      expect(data.latencyMs).toBeDefined();
    });

    it('returns disconnected status when not connected', async () => {
      connection = createMockConnection({ isConnected: false });

      const result = await handleSystemTool('system_bridge_status', {}, connection, safety);
      const data = JSON.parse(result.content[0].text);
      expect(data.connected).toBe(false);
      expect(data.available).toBe(false);
    });

    it('handles ping failure gracefully', async () => {
      connection.send.mockRejectedValue(new Error('connection lost'));

      const result = await handleSystemTool('system_bridge_status', {}, connection, safety);
      const data = JSON.parse(result.content[0].text);
      expect(data.connected).toBe(true);
      expect(data.error).toBeDefined();
    });
  });

  describe('system_node_list', () => {
    it('returns node list from bridge', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: ['/turtlebot3', '/gazebo'],
        timestamp: Date.now(),
      });

      const result = await handleSystemTool('system_node_list', {}, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/turtlebot3');
    });
  });

  describe('system_node_info', () => {
    it('sends node name to bridge', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: { name: '/turtlebot3', publishers: [], subscribers: [], services: [] },
        timestamp: Date.now(),
      });

      const result = await handleSystemTool('system_node_info', { nodeName: '/turtlebot3' }, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('node.info', { node_name: '/turtlebot3' });
    });
  });

  describe('system_health_status', () => {
    it('returns healthy status when bridge is connected and no e-stop', async () => {
      const result = await handleSystemTool('system_health_status', {}, connection, safety);
      expect(result.isError).toBeUndefined();
      const data = JSON.parse(result.content[0].text);
      expect(data.status).toBe('healthy');
      expect(data.bridge.connected).toBe(true);
      expect(data.safety.estopActive).toBe(false);
      expect(data.safety.violationCount).toBe(2);
      expect(data.uptime).toBeGreaterThanOrEqual(0);
      expect(data.memory.heapUsedMB).toBeDefined();
      expect(data.memory.heapTotalMB).toBeDefined();
      expect(data.timestamp).toBeDefined();
    });

    it('returns unhealthy status when bridge is disconnected', async () => {
      connection = createMockConnection({ isConnected: false });

      const result = await handleSystemTool('system_health_status', {}, connection, safety);
      const data = JSON.parse(result.content[0].text);
      expect(data.status).toBe('unhealthy');
      expect(data.bridge.connected).toBe(false);
    });

    it('returns degraded status when e-stop is active', async () => {
      safety = createMockSafety({ isEmergencyStopActive: true });

      const result = await handleSystemTool('system_health_status', {}, connection, safety);
      const data = JSON.parse(result.content[0].text);
      expect(data.status).toBe('degraded');
      expect(data.safety.estopActive).toBe(true);
    });
  });

  it('returns error for unknown system tool', async () => {
    const result = await handleSystemTool('unknown_tool', {}, connection, safety);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown system tool');
  });
});

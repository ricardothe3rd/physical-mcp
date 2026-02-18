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

describe('System Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  describe('system_bridge_status', () => {
    it('returns connected status with latency', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: { version: '0.1.0' }, timestamp: Date.now() });

      const result = await handleSystemTool('system_bridge_status', {}, connection);
      expect(result.isError).toBeUndefined();
      const data = JSON.parse(result.content[0].text);
      expect(data.connected).toBe(true);
      expect(data.available).toBe(true);
      expect(data.latencyMs).toBeDefined();
    });

    it('returns disconnected status when not connected', async () => {
      connection = createMockConnection({ isConnected: false });

      const result = await handleSystemTool('system_bridge_status', {}, connection);
      const data = JSON.parse(result.content[0].text);
      expect(data.connected).toBe(false);
      expect(data.available).toBe(false);
    });

    it('handles ping failure gracefully', async () => {
      connection.send.mockRejectedValue(new Error('connection lost'));

      const result = await handleSystemTool('system_bridge_status', {}, connection);
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

      const result = await handleSystemTool('system_node_list', {}, connection);
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

      const result = await handleSystemTool('system_node_info', { nodeName: '/turtlebot3' }, connection);
      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('node.info', { node_name: '/turtlebot3' });
    });
  });

  describe('ros2_param_list', () => {
    it('lists parameters for a node', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: ['use_sim_time', 'max_velocity'],
        timestamp: Date.now(),
      });

      const result = await handleSystemTool('ros2_param_list', { nodeName: '/turtlebot3' }, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('use_sim_time');
      expect(connection.send).toHaveBeenCalledWith('params.list', { node_name: '/turtlebot3' });
    });
  });

  describe('ros2_param_get', () => {
    it('gets a parameter value', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: true, timestamp: Date.now(),
      });

      const result = await handleSystemTool('ros2_param_get', {
        nodeName: '/turtlebot3',
        paramName: 'use_sim_time',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('use_sim_time');
      expect(result.content[0].text).toContain('true');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'param not found' }, timestamp: Date.now() });

      const result = await handleSystemTool('ros2_param_get', {
        nodeName: '/turtlebot3',
        paramName: 'nonexistent',
      }, connection);

      expect(result.isError).toBe(true);
    });
  });

  describe('ros2_param_set', () => {
    it('sets a parameter value', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() });

      const result = await handleSystemTool('ros2_param_set', {
        nodeName: '/turtlebot3',
        paramName: 'max_velocity',
        value: 0.5,
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('max_velocity');
      expect(result.content[0].text).toContain('0.5');
      expect(connection.send).toHaveBeenCalledWith('params.set', {
        node_name: '/turtlebot3',
        param_name: 'max_velocity',
        value: 0.5,
      });
    });
  });

  it('returns error for unknown system tool', async () => {
    const result = await handleSystemTool('unknown_tool', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown system tool');
  });
});

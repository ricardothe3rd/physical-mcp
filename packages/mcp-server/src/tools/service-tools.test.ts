/**
 * Tests for service tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { handleServiceTool } from './service-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('Service Tool Handlers', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  describe('ros2_service_list', () => {
    it('returns service list from bridge', async () => {
      const services = [{ name: '/set_bool', type: 'std_srvs/srv/SetBool' }];
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: services, timestamp: Date.now() });

      const result = await handleServiceTool('ros2_service_list', {}, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/set_bool');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'unavailable' }, timestamp: Date.now() });

      const result = await handleServiceTool('ros2_service_list', {}, connection, safety);
      expect(result.isError).toBe(true);
    });
  });

  describe('ros2_service_info', () => {
    it('sends service name to bridge', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: { name: '/spawn_entity', type: 'gazebo_msgs/srv/SpawnEntity' },
        timestamp: Date.now(),
      });

      const result = await handleServiceTool('ros2_service_info', { service: '/spawn_entity' }, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('service.info', { service: '/spawn_entity' });
    });
  });

  describe('ros2_service_call', () => {
    it('calls allowed service through bridge', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: { success: true }, timestamp: Date.now(),
      });

      const result = await handleServiceTool('ros2_service_call', {
        service: '/set_bool',
        serviceType: 'std_srvs/srv/SetBool',
        args: { data: true },
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('service.call', {
        service: '/set_bool',
        service_type: 'std_srvs/srv/SetBool',
        args: { data: true },
      });
    });

    it('blocks call to blocked service', async () => {
      const result = await handleServiceTool('ros2_service_call', {
        service: '/kill',
        serviceType: 'std_srvs/srv/Empty',
        args: {},
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('SAFETY BLOCKED');
      expect(result.content[0].text).toContain('blocked');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('blocks service call when e-stop active', async () => {
      safety.activateEmergencyStop();

      const result = await handleServiceTool('ros2_service_call', {
        service: '/set_bool',
        serviceType: 'std_srvs/srv/SetBool',
        args: { data: true },
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('SAFETY BLOCKED');
    });

    it('handles missing args with default empty object', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: { success: true }, timestamp: Date.now(),
      });

      const result = await handleServiceTool('ros2_service_call', {
        service: '/trigger',
        serviceType: 'std_srvs/srv/Trigger',
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('service.call', expect.objectContaining({
        args: {},
      }));
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'timeout' }, timestamp: Date.now() });

      const result = await handleServiceTool('ros2_service_call', {
        service: '/set_bool',
        serviceType: 'std_srvs/srv/SetBool',
        args: { data: true },
      }, connection, safety);

      expect(result.isError).toBe(true);
    });
  });

  it('returns error for unknown service tool', async () => {
    const result = await handleServiceTool('unknown_tool', {}, connection, safety);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown service tool');
  });
});

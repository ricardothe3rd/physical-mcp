/**
 * Tests for fleet management tool handlers.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getFleetTools, handleFleetTool, fleetRegistry } from './fleet-tools.js';

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

describe('Fleet Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
    fleetRegistry.clear();
  });

  describe('getFleetTools', () => {
    it('returns 3 tools', () => {
      const tools = getFleetTools();
      expect(tools).toHaveLength(3);
      expect(tools.map(t => t.name)).toContain('ros2_fleet_status');
      expect(tools.map(t => t.name)).toContain('ros2_fleet_add');
      expect(tools.map(t => t.name)).toContain('ros2_fleet_remove');
    });
  });

  describe('ros2_fleet_status', () => {
    it('returns empty fleet when no robots registered', async () => {
      const result = await handleFleetTool('ros2_fleet_status', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Fleet robots (0)');
    });

    it('lists registered robots', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot2' }, connection);

      const result = await handleFleetTool('ros2_fleet_status', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Fleet robots (2)');
      expect(result.content[0].text).toContain('/robot1');
      expect(result.content[0].text).toContain('/robot2');
    });
  });

  describe('ros2_fleet_add', () => {
    it('adds a robot to the fleet', async () => {
      const result = await handleFleetTool('ros2_fleet_add', {
        namespace: '/robot1',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/robot1');
      expect(result.content[0].text).toContain('added to fleet');
      expect(fleetRegistry.size).toBe(1);
      expect(fleetRegistry.has('/robot1')).toBe(true);
    });

    it('stores robot with initial unknown status', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);

      const robot = fleetRegistry.get('/robot1');
      expect(robot).toBeDefined();
      expect(robot!.status).toBe('unknown');
      expect(robot!.addedAt).toBeDefined();
    });

    it('rejects duplicate namespace', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      const result = await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('already registered');
      expect(fleetRegistry.size).toBe(1);
    });

    it('returns error when namespace is missing', async () => {
      const result = await handleFleetTool('ros2_fleet_add', {}, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('namespace');
    });
  });

  describe('ros2_fleet_remove', () => {
    it('removes a registered robot', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      expect(fleetRegistry.size).toBe(1);

      const result = await handleFleetTool('ros2_fleet_remove', {
        namespace: '/robot1',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/robot1');
      expect(result.content[0].text).toContain('removed from fleet');
      expect(fleetRegistry.size).toBe(0);
    });

    it('returns error for nonexistent robot', async () => {
      const result = await handleFleetTool('ros2_fleet_remove', {
        namespace: '/nonexistent',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('not found');
    });

    it('returns error when namespace is missing', async () => {
      const result = await handleFleetTool('ros2_fleet_remove', {}, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('namespace');
    });

    it('updates count after removal', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot2' }, connection);
      expect(fleetRegistry.size).toBe(2);

      const result = await handleFleetTool('ros2_fleet_remove', {
        namespace: '/robot1',
      }, connection);

      expect(result.content[0].text).toContain('Remaining robots: 1');
      expect(fleetRegistry.size).toBe(1);
      expect(fleetRegistry.has('/robot2')).toBe(true);
    });
  });

  it('returns error for unknown fleet tool', async () => {
    const result = await handleFleetTool('unknown_tool', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown fleet tool');
  });
});

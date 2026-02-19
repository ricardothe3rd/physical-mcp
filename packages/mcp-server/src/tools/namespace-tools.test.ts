/**
 * Tests for ROS2 namespace management tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import {
  getNamespaceTools,
  handleNamespaceTool,
  remapStore,
  resetRemapStore,
} from './namespace-tools.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('Namespace Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
    resetRemapStore();
  });

  describe('getNamespaceTools', () => {
    it('returns 3 tools', () => {
      const tools = getNamespaceTools();
      expect(tools).toHaveLength(3);
    });

    it('includes ros2_namespace_list', () => {
      const tools = getNamespaceTools();
      expect(tools.map(t => t.name)).toContain('ros2_namespace_list');
    });

    it('includes ros2_namespace_remap', () => {
      const tools = getNamespaceTools();
      expect(tools.map(t => t.name)).toContain('ros2_namespace_remap');
    });

    it('includes ros2_namespace_clear_remaps', () => {
      const tools = getNamespaceTools();
      expect(tools.map(t => t.name)).toContain('ros2_namespace_clear_remaps');
    });

    it('all tools have descriptions and input schemas', () => {
      const tools = getNamespaceTools();
      for (const tool of tools) {
        expect(tool.description).toBeTruthy();
        expect(tool.inputSchema).toBeDefined();
      }
    });
  });

  describe('ros2_namespace_list', () => {
    it('returns namespace list from bridge', async () => {
      const nsData = {
        namespaces: ['/', '/robot1', '/robot2', '/sensors'],
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: nsData, timestamp: Date.now(),
      });

      const result = await handleNamespaceTool('ros2_namespace_list', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/robot1');
      expect(result.content[0].text).toContain('/sensors');
      expect(connection.send).toHaveBeenCalledWith('namespace.list');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'bridge unavailable' }, timestamp: Date.now(),
      });

      const result = await handleNamespaceTool('ros2_namespace_list', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_namespace_remap', () => {
    it('stores a namespace remapping', async () => {
      const result = await handleNamespaceTool('ros2_namespace_remap', {
        from_namespace: '/robot1',
        to_namespace: '/robot2',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/robot1');
      expect(result.content[0].text).toContain('/robot2');
      expect(remapStore.get('/robot1')).toBe('/robot2');
    });

    it('overwrites existing remapping for same source namespace', async () => {
      await handleNamespaceTool('ros2_namespace_remap', {
        from_namespace: '/robot1',
        to_namespace: '/robot2',
      }, connection);

      await handleNamespaceTool('ros2_namespace_remap', {
        from_namespace: '/robot1',
        to_namespace: '/robot3',
      }, connection);

      expect(remapStore.get('/robot1')).toBe('/robot3');
      expect(remapStore.size).toBe(1);
    });

    it('stores multiple remappings', async () => {
      await handleNamespaceTool('ros2_namespace_remap', {
        from_namespace: '/robot1',
        to_namespace: '/robot2',
      }, connection);

      const result = await handleNamespaceTool('ros2_namespace_remap', {
        from_namespace: '/sensors',
        to_namespace: '/alt_sensors',
      }, connection);

      expect(remapStore.size).toBe(2);
      expect(result.content[0].text).toContain('2');
    });

    it('returns error when from_namespace is missing', async () => {
      const result = await handleNamespaceTool('ros2_namespace_remap', {
        to_namespace: '/robot2',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('from_namespace');
    });

    it('returns error when to_namespace is missing', async () => {
      const result = await handleNamespaceTool('ros2_namespace_remap', {
        from_namespace: '/robot1',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('to_namespace');
    });
  });

  describe('ros2_namespace_clear_remaps', () => {
    it('clears all remappings', async () => {
      remapStore.set('/robot1', '/robot2');
      remapStore.set('/sensors', '/alt_sensors');

      const result = await handleNamespaceTool('ros2_namespace_clear_remaps', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Cleared 2');
      expect(remapStore.size).toBe(0);
    });

    it('reports zero when no remappings exist', async () => {
      const result = await handleNamespaceTool('ros2_namespace_clear_remaps', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Cleared 0');
    });
  });

  it('returns error for unknown namespace tool', async () => {
    const result = await handleNamespaceTool('unknown_tool', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown namespace tool');
  });
});

/**
 * Tests for TF2 transform tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getTfTools, handleTfTool } from './tf-tools.js';

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

describe('TF Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  describe('getTfTools', () => {
    it('returns 2 tools', () => {
      const tools = getTfTools();
      expect(tools).toHaveLength(2);
      expect(tools.map(t => t.name)).toContain('ros2_tf_tree');
      expect(tools.map(t => t.name)).toContain('ros2_tf_lookup');
    });
  });

  describe('ros2_tf_tree', () => {
    it('returns the TF tree from bridge', async () => {
      const treeData = {
        frames: [
          { frame: 'base_link', parent: 'odom' },
          { frame: 'laser_link', parent: 'base_link' },
        ],
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: treeData, timestamp: Date.now(),
      });

      const result = await handleTfTool('ros2_tf_tree', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('base_link');
      expect(result.content[0].text).toContain('odom');
      expect(connection.send).toHaveBeenCalledWith('tf.tree');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'tf2 not available' }, timestamp: Date.now(),
      });

      const result = await handleTfTool('ros2_tf_tree', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_tf_lookup', () => {
    it('looks up transform between two frames', async () => {
      const transformData = {
        translation: { x: 1.0, y: 0.0, z: 0.5 },
        rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: transformData, timestamp: Date.now(),
      });

      const result = await handleTfTool('ros2_tf_lookup', {
        sourceFrame: 'base_link',
        targetFrame: 'odom',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('base_link');
      expect(result.content[0].text).toContain('odom');
      expect(result.content[0].text).toContain('translation');
      expect(connection.send).toHaveBeenCalledWith('tf.lookup', {
        source_frame: 'base_link',
        target_frame: 'odom',
      });
    });

    it('returns error when sourceFrame is missing', async () => {
      const result = await handleTfTool('ros2_tf_lookup', {
        targetFrame: 'odom',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('sourceFrame');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error when targetFrame is missing', async () => {
      const result = await handleTfTool('ros2_tf_lookup', {
        sourceFrame: 'base_link',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('targetFrame');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'transform not found' }, timestamp: Date.now(),
      });

      const result = await handleTfTool('ros2_tf_lookup', {
        sourceFrame: 'base_link',
        targetFrame: 'nonexistent_frame',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  it('returns error for unknown TF tool', async () => {
    const result = await handleTfTool('unknown_tool', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown TF tool');
  });
});

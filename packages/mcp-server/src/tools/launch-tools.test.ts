/**
 * Tests for launch file management tools.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getLaunchTools, handleLaunchTool, resetLaunchProcesses } from './launch-tools.js';

function createMockConnection() {
  return {
    isConnected: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({
      id: '1',
      status: 'ok',
      data: {},
      timestamp: Date.now(),
    }),
    disconnect: vi.fn(),
  } as any;
}

describe('Launch Tools', () => {
  beforeEach(() => {
    resetLaunchProcesses();
  });

  describe('tool definitions', () => {
    it('returns 3 tools', () => {
      const tools = getLaunchTools();
      expect(tools.length).toBe(3);
    });

    it('has ros2_launch_start tool', () => {
      const tools = getLaunchTools();
      expect(tools.map(t => t.name)).toContain('ros2_launch_start');
    });

    it('has ros2_launch_stop tool', () => {
      const tools = getLaunchTools();
      expect(tools.map(t => t.name)).toContain('ros2_launch_stop');
    });

    it('has ros2_launch_status tool', () => {
      const tools = getLaunchTools();
      expect(tools.map(t => t.name)).toContain('ros2_launch_status');
    });

    it('all tools have descriptions', () => {
      for (const tool of getLaunchTools()) {
        expect(tool.description.length).toBeGreaterThan(10);
      }
    });
  });

  describe('ros2_launch_start', () => {
    it('starts a launch process', async () => {
      const connection = createMockConnection();
      const result = await handleLaunchTool('ros2_launch_start', {
        packageName: 'turtlebot3_gazebo',
        launchFile: 'turtlebot3_world.launch.py',
      }, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Launch started');
      expect(result.content[0].text).toContain('turtlebot3_gazebo');
    });

    it('passes launch args to bridge', async () => {
      const connection = createMockConnection();
      await handleLaunchTool('ros2_launch_start', {
        packageName: 'nav2_bringup',
        launchFile: 'navigation_launch.py',
        args: { use_sim_time: 'true', map: '/maps/test.yaml' },
      }, connection);
      expect(connection.send).toHaveBeenCalledWith('launch.start', expect.objectContaining({
        package: 'nav2_bringup',
        launch_file: 'navigation_launch.py',
        args: { use_sim_time: 'true', map: '/maps/test.yaml' },
      }));
    });

    it('handles bridge error response', async () => {
      const connection = createMockConnection();
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: 'Package not found', timestamp: Date.now() });
      const result = await handleLaunchTool('ros2_launch_start', {
        packageName: 'nonexistent',
        launchFile: 'test.launch.py',
      }, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Failed to start launch');
    });

    it('handles bridge connection error', async () => {
      const connection = createMockConnection();
      connection.send.mockRejectedValue(new Error('Not connected'));
      const result = await handleLaunchTool('ros2_launch_start', {
        packageName: 'pkg',
        launchFile: 'test.launch.py',
      }, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Not connected');
    });
  });

  describe('ros2_launch_stop', () => {
    it('stops a running launch process', async () => {
      const connection = createMockConnection();
      await handleLaunchTool('ros2_launch_start', {
        packageName: 'pkg',
        launchFile: 'test.launch.py',
      }, connection);
      const result = await handleLaunchTool('ros2_launch_stop', {
        processId: 'launch_1',
      }, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Launch stopped');
    });

    it('returns error for unknown process ID', async () => {
      const connection = createMockConnection();
      const result = await handleLaunchTool('ros2_launch_stop', {
        processId: 'nonexistent',
      }, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('not found');
    });
  });

  describe('ros2_launch_status', () => {
    it('returns empty when no processes', async () => {
      const connection = createMockConnection();
      const result = await handleLaunchTool('ros2_launch_status', {}, connection);
      expect(result.content[0].text).toContain('No active launch processes');
    });

    it('lists active processes', async () => {
      const connection = createMockConnection();
      await handleLaunchTool('ros2_launch_start', {
        packageName: 'pkg1',
        launchFile: 'a.launch.py',
      }, connection);
      await handleLaunchTool('ros2_launch_start', {
        packageName: 'pkg2',
        launchFile: 'b.launch.py',
      }, connection);
      const result = await handleLaunchTool('ros2_launch_status', {}, connection);
      const data = JSON.parse(result.content[0].text);
      expect(data.length).toBe(2);
      expect(data[0].package).toBe('pkg1');
      expect(data[1].package).toBe('pkg2');
    });
  });

  describe('unknown tool', () => {
    it('returns error for unknown launch tool', async () => {
      const connection = createMockConnection();
      const result = await handleLaunchTool('nonexistent', {}, connection);
      expect(result.isError).toBe(true);
    });
  });
});

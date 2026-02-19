/**
 * Tests for ROS2 map tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getMapTools, handleMapTool } from './map-tools.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('getMapTools', () => {
  it('returns exactly three tools', () => {
    const tools = getMapTools();
    expect(tools).toHaveLength(3);
  });

  it('includes ros2_map_info tool', () => {
    const tools = getMapTools();
    const tool = tools.find(t => t.name === 'ros2_map_info');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('map metadata');
  });

  it('includes ros2_costmap_info tool', () => {
    const tools = getMapTools();
    const tool = tools.find(t => t.name === 'ros2_costmap_info');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('costmap metadata');
  });

  it('includes ros2_map_topics tool', () => {
    const tools = getMapTools();
    const tool = tools.find(t => t.name === 'ros2_map_topics');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('map-related messages');
  });

  it('all tools have valid inputSchema', () => {
    const tools = getMapTools();
    for (const tool of tools) {
      expect(tool.inputSchema).toBeDefined();
      expect(tool.inputSchema.type).toBe('object');
    }
  });
});

describe('handleMapTool - ros2_map_info', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns map info on success (nested info field)', async () => {
    const data = {
      info: {
        resolution: 0.05,
        width: 4000,
        height: 4000,
        origin: {
          position: { x: -100.0, y: -100.0, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        },
      },
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_map_info', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content).toHaveLength(1);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Map info');
    expect(result.content[0].text).toContain('0.05');
    expect(result.content[0].text).toContain('4000');
    expect(result.content[0].text).toContain('origin');
  });

  it('returns map info on success (flat fields)', async () => {
    const data = {
      resolution: 0.1,
      width: 2000,
      height: 2000,
      origin: { position: { x: 0.0, y: 0.0, z: 0.0 } },
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_map_info', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('0.1');
    expect(result.content[0].text).toContain('2000');
  });

  it('uses default topic /map when none provided', async () => {
    await handleMapTool('ros2_map_info', {}, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/map',
      count: 1,
    });
  });

  it('uses custom topic when provided', async () => {
    await handleMapTool('ros2_map_info', { topic: '/robot1/map' }, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/robot1/map',
      count: 1,
    });
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Topic not found' }, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_map_info', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('Topic not found');
  });

  it('returns error for invalid topic name (missing leading slash)', async () => {
    const result = await handleMapTool('ros2_map_info', { topic: 'map' }, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid topic');
    expect(connection.send).not.toHaveBeenCalled();
  });
});

describe('handleMapTool - ros2_costmap_info', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns costmap info on success (nested info field)', async () => {
    const data = {
      info: {
        resolution: 0.05,
        width: 200,
        height: 200,
        origin: {
          position: { x: -5.0, y: -5.0, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        },
      },
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_costmap_info', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content).toHaveLength(1);
    expect(result.content[0].text).toContain('Costmap info');
    expect(result.content[0].text).toContain('0.05');
    expect(result.content[0].text).toContain('200');
    expect(result.content[0].text).toContain('origin');
  });

  it('returns costmap info on success (flat fields)', async () => {
    const data = {
      resolution: 0.025,
      width: 400,
      height: 400,
      origin: { position: { x: 0.0, y: 0.0, z: 0.0 } },
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_costmap_info', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('0.025');
    expect(result.content[0].text).toContain('400');
  });

  it('uses default topic /local_costmap/costmap when none provided', async () => {
    await handleMapTool('ros2_costmap_info', {}, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/local_costmap/costmap',
      count: 1,
    });
  });

  it('uses custom topic when provided', async () => {
    await handleMapTool('ros2_costmap_info', { topic: '/global_costmap/costmap' }, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/global_costmap/costmap',
      count: 1,
    });
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Timeout waiting for costmap' }, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_costmap_info', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('Timeout');
  });

  it('returns error for invalid topic name (missing leading slash)', async () => {
    const result = await handleMapTool('ros2_costmap_info', { topic: 'local_costmap/costmap' }, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid topic');
    expect(connection.send).not.toHaveBeenCalled();
  });
});

describe('handleMapTool - ros2_map_topics', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns only map topics from full topic list', async () => {
    const allTopics = [
      { name: '/map', type: 'nav_msgs/msg/OccupancyGrid' },
      { name: '/map_metadata', type: 'nav_msgs/msg/MapMetaData' },
      { name: '/local_costmap/costmap', type: 'nav_msgs/msg/OccupancyGrid' },
      { name: '/cmd_vel', type: 'geometry_msgs/msg/Twist' },
      { name: '/camera/image_raw', type: 'sensor_msgs/msg/Image' },
    ];
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: allTopics, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_map_topics', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('3 found');
    expect(result.content[0].text).toContain('/map');
    expect(result.content[0].text).toContain('/map_metadata');
    expect(result.content[0].text).toContain('/local_costmap/costmap');
    expect(result.content[0].text).not.toContain('/cmd_vel');
    expect(result.content[0].text).not.toContain('/camera/image_raw');
    expect(connection.send).toHaveBeenCalledWith('topic.list');
  });

  it('returns empty list when no map topics exist', async () => {
    const allTopics = [
      { name: '/cmd_vel', type: 'geometry_msgs/msg/Twist' },
      { name: '/odom', type: 'nav_msgs/msg/Odometry' },
    ];
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: allTopics, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_map_topics', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('0 found');
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Bridge unavailable' }, timestamp: Date.now(),
    });

    const result = await handleMapTool('ros2_map_topics', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
  });
});

describe('handleMapTool - unknown tool', () => {
  it('returns error for unknown tool name', async () => {
    const connection = createMockConnection();

    const result = await handleMapTool('ros2_nonexistent_map_tool', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown map tool');
    expect(result.content[0].text).toContain('ros2_nonexistent_map_tool');
  });
});

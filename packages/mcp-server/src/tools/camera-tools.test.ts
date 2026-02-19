/**
 * Tests for ROS2 camera tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getCameraTools, handleCameraTool } from './camera-tools.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('getCameraTools', () => {
  it('returns exactly three tools', () => {
    const tools = getCameraTools();
    expect(tools).toHaveLength(3);
  });

  it('includes ros2_camera_info tool', () => {
    const tools = getCameraTools();
    const tool = tools.find(t => t.name === 'ros2_camera_info');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('camera calibration');
  });

  it('includes ros2_image_preview tool', () => {
    const tools = getCameraTools();
    const tool = tools.find(t => t.name === 'ros2_image_preview');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('image metadata');
  });

  it('includes ros2_image_topics tool', () => {
    const tools = getCameraTools();
    const tool = tools.find(t => t.name === 'ros2_image_topics');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('image messages');
  });

  it('all tools have valid inputSchema', () => {
    const tools = getCameraTools();
    for (const tool of tools) {
      expect(tool.inputSchema).toBeDefined();
      expect(tool.inputSchema.type).toBe('object');
    }
  });
});

describe('handleCameraTool - ros2_camera_info', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns camera info on success', async () => {
    const data = {
      width: 640,
      height: 480,
      distortion_model: 'plumb_bob',
      k: [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0],
      d: [0.0, 0.0, 0.0, 0.0, 0.0],
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data, timestamp: Date.now(),
    });

    const result = await handleCameraTool('ros2_camera_info', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content).toHaveLength(1);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Camera info');
    expect(result.content[0].text).toContain('640');
    expect(result.content[0].text).toContain('plumb_bob');
  });

  it('uses default topic /camera/camera_info when none provided', async () => {
    await handleCameraTool('ros2_camera_info', {}, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/camera/camera_info',
      count: 1,
    });
  });

  it('uses custom topic when provided', async () => {
    await handleCameraTool('ros2_camera_info', { topic: '/front_camera/camera_info' }, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/front_camera/camera_info',
      count: 1,
    });
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Topic not found' }, timestamp: Date.now(),
    });

    const result = await handleCameraTool('ros2_camera_info', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('Topic not found');
  });

  it('returns error for invalid topic name (missing leading slash)', async () => {
    const result = await handleCameraTool('ros2_camera_info', { topic: 'camera/camera_info' }, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid topic');
    expect(connection.send).not.toHaveBeenCalled();
  });
});

describe('handleCameraTool - ros2_image_preview', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns image metadata on success', async () => {
    const data = {
      width: 1920,
      height: 1080,
      encoding: 'rgb8',
      step: 5760,
      data: new Array(1920 * 1080 * 3),
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data, timestamp: Date.now(),
    });

    const result = await handleCameraTool('ros2_image_preview', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content).toHaveLength(1);
    expect(result.content[0].text).toContain('Image metadata');
    expect(result.content[0].text).toContain('1920');
    expect(result.content[0].text).toContain('1080');
    expect(result.content[0].text).toContain('rgb8');
    expect(result.content[0].text).toContain('5760');
    expect(result.content[0].text).toContain('data_length');
    expect(result.content[0].text).toContain('MCP text protocol limitations');
  });

  it('uses default topic /camera/image_raw when none provided', async () => {
    await handleCameraTool('ros2_image_preview', {}, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/camera/image_raw',
      count: 1,
    });
  });

  it('uses custom topic when provided', async () => {
    await handleCameraTool('ros2_image_preview', { topic: '/stereo/left/image_raw' }, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/stereo/left/image_raw',
      count: 1,
    });
  });

  it('passes encoding parameter when provided', async () => {
    await handleCameraTool('ros2_image_preview', { encoding: 'bgr8' }, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/camera/image_raw',
      count: 1,
      encoding: 'bgr8',
    });
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Timeout waiting for image' }, timestamp: Date.now(),
    });

    const result = await handleCameraTool('ros2_image_preview', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('Timeout');
  });

  it('returns error for invalid topic name (missing leading slash)', async () => {
    const result = await handleCameraTool('ros2_image_preview', { topic: 'camera/image_raw' }, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid topic');
    expect(connection.send).not.toHaveBeenCalled();
  });
});

describe('handleCameraTool - ros2_image_topics', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns only image topics from full topic list', async () => {
    const allTopics = [
      { name: '/camera/image_raw', type: 'sensor_msgs/msg/Image' },
      { name: '/camera/image_compressed', type: 'sensor_msgs/msg/CompressedImage' },
      { name: '/cmd_vel', type: 'geometry_msgs/msg/Twist' },
      { name: '/odom', type: 'nav_msgs/msg/Odometry' },
      { name: '/depth/image_raw', type: 'sensor_msgs/msg/Image' },
    ];
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: allTopics, timestamp: Date.now(),
    });

    const result = await handleCameraTool('ros2_image_topics', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('3 found');
    expect(result.content[0].text).toContain('/camera/image_raw');
    expect(result.content[0].text).toContain('/camera/image_compressed');
    expect(result.content[0].text).toContain('/depth/image_raw');
    expect(result.content[0].text).not.toContain('/cmd_vel');
    expect(result.content[0].text).not.toContain('/odom');
    expect(connection.send).toHaveBeenCalledWith('topic.list');
  });

  it('returns empty list when no image topics exist', async () => {
    const allTopics = [
      { name: '/cmd_vel', type: 'geometry_msgs/msg/Twist' },
      { name: '/odom', type: 'nav_msgs/msg/Odometry' },
    ];
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: allTopics, timestamp: Date.now(),
    });

    const result = await handleCameraTool('ros2_image_topics', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('0 found');
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Bridge unavailable' }, timestamp: Date.now(),
    });

    const result = await handleCameraTool('ros2_image_topics', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
  });
});

describe('handleCameraTool - unknown tool', () => {
  it('returns error for unknown tool name', async () => {
    const connection = createMockConnection();

    const result = await handleCameraTool('ros2_nonexistent_camera_tool', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown camera tool');
    expect(result.content[0].text).toContain('ros2_nonexistent_camera_tool');
  });
});

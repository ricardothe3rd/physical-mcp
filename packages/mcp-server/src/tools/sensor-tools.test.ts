/**
 * Tests for ROS2 sensor tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getSensorTools, handleSensorTool } from './sensor-tools.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('Sensor Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  describe('getSensorTools', () => {
    it('returns 2 tools', () => {
      const tools = getSensorTools();
      expect(tools).toHaveLength(2);
    });

    it('includes ros2_sensor_summary', () => {
      const tools = getSensorTools();
      expect(tools.map(t => t.name)).toContain('ros2_sensor_summary');
    });

    it('includes ros2_sensor_read', () => {
      const tools = getSensorTools();
      expect(tools.map(t => t.name)).toContain('ros2_sensor_read');
    });

    it('all tools have descriptions and input schemas', () => {
      const tools = getSensorTools();
      for (const tool of tools) {
        expect(tool.description).toBeTruthy();
        expect(tool.inputSchema).toBeDefined();
      }
    });
  });

  describe('ros2_sensor_summary', () => {
    it('returns sensor summary from bridge', async () => {
      const summaryData = {
        categories: {
          camera: [{ topic: '/camera/image_raw', type: 'sensor_msgs/msg/Image' }],
          lidar: [{ topic: '/scan', type: 'sensor_msgs/msg/LaserScan' }],
          imu: [{ topic: '/imu/data', type: 'sensor_msgs/msg/Imu' }],
        },
        total: 3,
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: summaryData, timestamp: Date.now(),
      });

      const result = await handleSensorTool('ros2_sensor_summary', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('camera');
      expect(result.content[0].text).toContain('lidar');
      expect(result.content[0].text).toContain('imu');
      expect(connection.send).toHaveBeenCalledWith('sensor.summary');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'sensor data unavailable' }, timestamp: Date.now(),
      });

      const result = await handleSensorTool('ros2_sensor_summary', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_sensor_read', () => {
    it('returns sensor reading from bridge', async () => {
      const readingData = {
        topic: '/imu/data',
        timestamp: 1700000000.0,
        data: {
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
          angular_velocity: { x: 0.01, y: 0.02, z: 0.0 },
          linear_acceleration: { x: 0.0, y: 0.0, z: 9.81 },
        },
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: readingData, timestamp: Date.now(),
      });

      const result = await handleSensorTool('ros2_sensor_read', {
        topic: '/imu/data',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/imu/data');
      expect(result.content[0].text).toContain('orientation');
      expect(connection.send).toHaveBeenCalledWith('sensor.read', {
        topic: '/imu/data',
        timeout_ms: 5000,
      });
    });

    it('uses custom timeout_ms when provided', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: { topic: '/scan', data: {} }, timestamp: Date.now(),
      });

      const result = await handleSensorTool('ros2_sensor_read', {
        topic: '/scan',
        timeout_ms: 10000,
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('sensor.read', {
        topic: '/scan',
        timeout_ms: 10000,
      });
    });

    it('returns error when topic is missing', async () => {
      const result = await handleSensorTool('ros2_sensor_read', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('topic');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'timeout waiting for sensor data' }, timestamp: Date.now(),
      });

      const result = await handleSensorTool('ros2_sensor_read', {
        topic: '/camera/image_raw',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  it('returns error for unknown sensor tool', async () => {
    const result = await handleSensorTool('unknown_tool', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown sensor tool');
  });
});

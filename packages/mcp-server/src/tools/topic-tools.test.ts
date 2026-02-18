/**
 * Tests for topic tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { handleTopicTool } from './topic-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

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

describe('Topic Tool Handlers', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  describe('ros2_topic_list', () => {
    it('returns topic list from bridge', async () => {
      const topics = [{ name: '/cmd_vel', type: 'geometry_msgs/msg/Twist' }];
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: topics, timestamp: Date.now() });

      const result = await handleTopicTool('ros2_topic_list', {}, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/cmd_vel');
      expect(connection.send).toHaveBeenCalledWith('topic.list');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'bridge down' }, timestamp: Date.now() });

      const result = await handleTopicTool('ros2_topic_list', {}, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_topic_info', () => {
    it('sends topic name to bridge', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: { topic: '/odom', type: 'nav_msgs/msg/Odometry', publishers: 1, subscribers: 2 },
        timestamp: Date.now(),
      });

      const result = await handleTopicTool('ros2_topic_info', { topic: '/odom' }, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('topic.info', { topic: '/odom' });
    });
  });

  describe('ros2_topic_subscribe', () => {
    it('sends subscribe params to bridge', async () => {
      const messages = [{ linear: { x: 0.1 } }];
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: messages, timestamp: Date.now() });

      const result = await handleTopicTool('ros2_topic_subscribe', {
        topic: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist',
        count: 3,
        timeoutSec: 10,
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
        topic: '/cmd_vel',
        message_type: 'geometry_msgs/msg/Twist',
        count: 3,
        timeout_sec: 10,
      });
    });
  });

  describe('ros2_topic_publish', () => {
    it('publishes safe velocity command', async () => {
      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist',
        message: { linear: { x: 0.2, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.5 } },
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Published to /cmd_vel');
      expect(connection.send).toHaveBeenCalledWith('topic.publish', expect.objectContaining({
        topic: '/cmd_vel',
      }));
    });

    it('blocks over-limit velocity', async () => {
      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist',
        message: { linear: { x: 10.0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('SAFETY BLOCKED');
      expect(result.content[0].text).toContain('velocity');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('blocks publish to blocked topic', async () => {
      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '/rosout',
        messageType: 'rcl_interfaces/msg/Log',
        message: { msg: 'test' },
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('SAFETY BLOCKED');
      expect(result.content[0].text).toContain('blocked');
    });

    it('blocks publish when e-stop active', async () => {
      safety.activateEmergencyStop();

      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist',
        message: { linear: { x: 0.1 } },
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('SAFETY BLOCKED');
      expect(result.content[0].text).toContain('emergency');
    });

    it('returns error on bridge failure after safety pass', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'timeout' }, timestamp: Date.now() });

      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist',
        message: { linear: { x: 0.1, y: 0, z: 0 } },
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_topic_echo', () => {
    it('sends echo request with defaults', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: { linear: { x: 0.5 }, angular: { z: 0.3 } },
        timestamp: Date.now(),
      });

      const result = await handleTopicTool('ros2_topic_echo', {
        topic: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist',
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('topic.echo', expect.objectContaining({
        topic: '/cmd_vel',
        timeout_sec: 3,
      }));
    });
  });

  it('returns error for unknown topic tool', async () => {
    const result = await handleTopicTool('unknown_tool', {}, connection, safety);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown topic tool');
  });
});

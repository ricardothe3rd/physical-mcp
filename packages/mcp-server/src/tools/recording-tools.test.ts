/**
 * Tests for topic recording tools.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import {
  getRecordingTools,
  handleRecordingTool,
  resetRecordingSessions,
} from './recording-tools.js';

function createMockConnection(overrides: Record<string, unknown> = {}) {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: [], timestamp: Date.now() }),
    disconnect: vi.fn(),
    ...overrides,
  } as any;
}

describe('Recording Tools', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
    resetRecordingSessions();
  });

  describe('getRecordingTools', () => {
    it('returns 3 recording tools', () => {
      const tools = getRecordingTools();
      expect(tools).toHaveLength(3);
      expect(tools.map(t => t.name)).toContain('ros2_topic_record_start');
      expect(tools.map(t => t.name)).toContain('ros2_topic_record_stop');
      expect(tools.map(t => t.name)).toContain('ros2_topic_record_status');
    });
  });

  describe('ros2_topic_record_start', () => {
    it('starts a recording session', async () => {
      const result = await handleRecordingTool('ros2_topic_record_start', {
        topic: '/odom',
        messageType: 'nav_msgs/msg/Odometry',
        maxMessages: 500,
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Recording started');
      expect(result.content[0].text).toContain('rec_');
      expect(result.content[0].text).toContain('/odom');
    });

    it('collects messages from bridge response', async () => {
      const messages = [
        { pose: { x: 1 } },
        { pose: { x: 2 } },
      ];
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: messages, timestamp: Date.now() });

      await handleRecordingTool('ros2_topic_record_start', {
        topic: '/odom',
        messageType: 'nav_msgs/msg/Odometry',
      }, connection);

      // Check status shows the messages
      const status = await handleRecordingTool('ros2_topic_record_status', {}, connection);
      const data = JSON.parse(status.content[0].text.replace('Active recordings (1):\n\n', ''));
      expect(data[0].messageCount).toBe(2);
    });

    it('handles bridge failure gracefully', async () => {
      connection.send.mockRejectedValue(new Error('connection lost'));

      const result = await handleRecordingTool('ros2_topic_record_start', {
        topic: '/odom',
        messageType: 'nav_msgs/msg/Odometry',
      }, connection);

      expect(result.isError).toBeUndefined(); // Should still succeed
      expect(result.content[0].text).toContain('Recording started');
    });
  });

  describe('ros2_topic_record_stop', () => {
    it('stops recording and returns messages', async () => {
      const messages = [{ data: 'test' }];
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: messages, timestamp: Date.now() });

      const start = await handleRecordingTool('ros2_topic_record_start', {
        topic: '/scan',
        messageType: 'sensor_msgs/msg/LaserScan',
      }, connection);

      // Extract session ID from response
      const match = start.content[0].text.match(/session "(\w+)"/);
      const sessionId = match![1];

      const stop = await handleRecordingTool('ros2_topic_record_stop', {
        sessionId,
      }, connection);

      expect(stop.isError).toBeUndefined();
      const data = JSON.parse(stop.content[0].text);
      expect(data.topic).toBe('/scan');
      expect(data.messageCount).toBe(1);
      expect(data.durationMs).toBeGreaterThanOrEqual(0);
    });

    it('returns error for nonexistent session', async () => {
      const result = await handleRecordingTool('ros2_topic_record_stop', {
        sessionId: 'nonexistent',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('not found');
    });
  });

  describe('ros2_topic_record_status', () => {
    it('returns empty when no active recordings', async () => {
      const result = await handleRecordingTool('ros2_topic_record_status', {}, connection);
      expect(result.content[0].text).toContain('No active recordings');
    });

    it('shows active recording sessions', async () => {
      await handleRecordingTool('ros2_topic_record_start', {
        topic: '/odom',
        messageType: 'nav_msgs/msg/Odometry',
      }, connection);

      await handleRecordingTool('ros2_topic_record_start', {
        topic: '/scan',
        messageType: 'sensor_msgs/msg/LaserScan',
      }, connection);

      const result = await handleRecordingTool('ros2_topic_record_status', {}, connection);
      expect(result.content[0].text).toContain('Active recordings (2)');
    });
  });

  it('returns error for unknown recording tool', async () => {
    const result = await handleRecordingTool('unknown', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown recording tool');
  });
});

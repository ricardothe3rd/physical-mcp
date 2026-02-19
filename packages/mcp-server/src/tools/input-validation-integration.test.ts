/**
 * Tests that input validation is properly integrated into tool handlers.
 */

import { describe, it, expect, vi } from 'vitest';
import { handleTopicTool } from './topic-tools.js';
import { handleServiceTool } from './service-tools.js';
import { handleActionTool } from './action-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

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

describe('Input Validation Integration', () => {
  const safety = new PolicyEngine();

  describe('topic tools validation', () => {
    it('rejects invalid topic name in ros2_topic_info', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_info', {
        topic: 'no_leading_slash',
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid topic name');
    });

    it('rejects invalid topic name in ros2_topic_subscribe', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_subscribe', {
        topic: 'bad name',
        messageType: 'std_msgs/msg/String',
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid topic name');
    });

    it('rejects invalid topic name in ros2_topic_publish', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '//double_slash',
        messageType: 'geometry_msgs/msg/Twist',
        message: { linear: { x: 0 } },
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid topic name');
    });

    it('rejects null message payload in ros2_topic_publish', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist',
        message: null,
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid message');
    });

    it('rejects invalid topic name in ros2_topic_echo', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_echo', {
        topic: '/trailing/',
        messageType: 'std_msgs/msg/String',
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid topic name');
    });

    it('accepts valid topic names', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_info', {
        topic: '/cmd_vel',
      }, connection, safety);
      expect(result.isError).toBeUndefined();
    });

    it('accepts valid publish with valid topic and message', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_publish', {
        topic: '/status',
        messageType: 'std_msgs/msg/String',
        message: { data: 'hello' },
      }, connection, safety);
      expect(result.isError).toBeUndefined();
    });

    it('topic_list does not validate topic name', async () => {
      const connection = createMockConnection();
      const result = await handleTopicTool('ros2_topic_list', {}, connection, safety);
      expect(result.isError).toBeUndefined();
    });
  });

  describe('service tools validation', () => {
    it('rejects invalid service name in ros2_service_info', async () => {
      const connection = createMockConnection();
      const result = await handleServiceTool('ros2_service_info', {
        service: 'no_slash',
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid service name');
    });

    it('rejects invalid service name in ros2_service_call', async () => {
      const connection = createMockConnection();
      const result = await handleServiceTool('ros2_service_call', {
        service: '//bad',
        serviceType: 'std_srvs/srv/SetBool',
        args: {},
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid service name');
    });

    it('accepts valid service names', async () => {
      const connection = createMockConnection();
      const result = await handleServiceTool('ros2_service_info', {
        service: '/set_parameters',
      }, connection, safety);
      expect(result.isError).toBeUndefined();
    });

    it('service_list does not validate service name', async () => {
      const connection = createMockConnection();
      const result = await handleServiceTool('ros2_service_list', {}, connection, safety);
      expect(result.isError).toBeUndefined();
    });
  });

  describe('action tools validation', () => {
    it('rejects invalid action name in ros2_action_send_goal', async () => {
      const connection = createMockConnection();
      const result = await handleActionTool('ros2_action_send_goal', {
        action: 'no_slash',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: {},
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Invalid action name');
    });

    it('accepts valid action names', async () => {
      const connection = createMockConnection();
      const result = await handleActionTool('ros2_action_send_goal', {
        action: '/navigate_to_pose',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { x: 1, y: 2 },
      }, connection, safety);
      expect(result.isError).toBeUndefined();
    });

    it('action_list does not validate action name', async () => {
      const connection = createMockConnection();
      const result = await handleActionTool('ros2_action_list', {}, connection, safety);
      expect(result.isError).toBeUndefined();
    });
  });
});

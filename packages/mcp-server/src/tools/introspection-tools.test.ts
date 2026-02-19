/**
 * Tests for ROS2 type introspection tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getIntrospectionTools, handleIntrospectionTool } from './introspection-tools.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('Introspection Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  describe('getIntrospectionTools', () => {
    it('returns 3 tools', () => {
      const tools = getIntrospectionTools();
      expect(tools).toHaveLength(3);
    });

    it('includes ros2_msg_type_info', () => {
      const tools = getIntrospectionTools();
      expect(tools.map(t => t.name)).toContain('ros2_msg_type_info');
    });

    it('includes ros2_srv_type_info', () => {
      const tools = getIntrospectionTools();
      expect(tools.map(t => t.name)).toContain('ros2_srv_type_info');
    });

    it('includes ros2_action_type_info', () => {
      const tools = getIntrospectionTools();
      expect(tools.map(t => t.name)).toContain('ros2_action_type_info');
    });

    it('all tools have descriptions and input schemas', () => {
      const tools = getIntrospectionTools();
      for (const tool of tools) {
        expect(tool.description).toBeTruthy();
        expect(tool.inputSchema).toBeDefined();
      }
    });
  });

  describe('ros2_msg_type_info', () => {
    it('returns message type definition from bridge', async () => {
      const msgData = {
        type: 'geometry_msgs/msg/Twist',
        fields: [
          { name: 'linear', type: 'geometry_msgs/msg/Vector3' },
          { name: 'angular', type: 'geometry_msgs/msg/Vector3' },
        ],
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: msgData, timestamp: Date.now(),
      });

      const result = await handleIntrospectionTool('ros2_msg_type_info', {
        type_name: 'geometry_msgs/msg/Twist',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('geometry_msgs/msg/Twist');
      expect(result.content[0].text).toContain('linear');
      expect(connection.send).toHaveBeenCalledWith('msg.type_info', {
        type_name: 'geometry_msgs/msg/Twist',
      });
    });

    it('returns error when type_name is missing', async () => {
      const result = await handleIntrospectionTool('ros2_msg_type_info', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('type_name');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'type not found' }, timestamp: Date.now(),
      });

      const result = await handleIntrospectionTool('ros2_msg_type_info', {
        type_name: 'nonexistent_msgs/msg/Fake',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_srv_type_info', () => {
    it('returns service type definition from bridge', async () => {
      const srvData = {
        type: 'std_srvs/srv/SetBool',
        request: [{ name: 'data', type: 'bool' }],
        response: [{ name: 'success', type: 'bool' }, { name: 'message', type: 'string' }],
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: srvData, timestamp: Date.now(),
      });

      const result = await handleIntrospectionTool('ros2_srv_type_info', {
        type_name: 'std_srvs/srv/SetBool',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('std_srvs/srv/SetBool');
      expect(connection.send).toHaveBeenCalledWith('srv.type_info', {
        type_name: 'std_srvs/srv/SetBool',
      });
    });

    it('returns error when type_name is missing', async () => {
      const result = await handleIntrospectionTool('ros2_srv_type_info', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('type_name');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'service type not found' }, timestamp: Date.now(),
      });

      const result = await handleIntrospectionTool('ros2_srv_type_info', {
        type_name: 'nonexistent_srvs/srv/Fake',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_action_type_info', () => {
    it('returns action type definition from bridge', async () => {
      const actionData = {
        type: 'nav2_msgs/action/NavigateToPose',
        goal: [{ name: 'pose', type: 'geometry_msgs/msg/PoseStamped' }],
        result: [{ name: 'result', type: 'std_msgs/msg/Empty' }],
        feedback: [{ name: 'current_pose', type: 'geometry_msgs/msg/PoseStamped' }],
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: actionData, timestamp: Date.now(),
      });

      const result = await handleIntrospectionTool('ros2_action_type_info', {
        type_name: 'nav2_msgs/action/NavigateToPose',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('nav2_msgs/action/NavigateToPose');
      expect(result.content[0].text).toContain('goal');
      expect(connection.send).toHaveBeenCalledWith('action.type_info', {
        type_name: 'nav2_msgs/action/NavigateToPose',
      });
    });

    it('returns error when type_name is missing', async () => {
      const result = await handleIntrospectionTool('ros2_action_type_info', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('type_name');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'action type not found' }, timestamp: Date.now(),
      });

      const result = await handleIntrospectionTool('ros2_action_type_info', {
        type_name: 'nonexistent_msgs/action/Fake',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  it('returns error for unknown introspection tool', async () => {
    const result = await handleIntrospectionTool('unknown_tool', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown introspection tool');
  });
});

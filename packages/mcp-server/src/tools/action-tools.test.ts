/**
 * Tests for action tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { handleActionTool } from './action-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('Action Tool Handlers', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  describe('ros2_action_list', () => {
    it('returns action list from bridge', async () => {
      const actions = [{ name: '/navigate_to_pose', type: 'nav2_msgs/action/NavigateToPose' }];
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: actions, timestamp: Date.now() });

      const result = await handleActionTool('ros2_action_list', {}, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('/navigate_to_pose');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'no actions' }, timestamp: Date.now() });

      const result = await handleActionTool('ros2_action_list', {}, connection, safety);
      expect(result.isError).toBe(true);
    });
  });

  describe('ros2_action_send_goal', () => {
    it('sends goal through bridge', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: { goal_id: 'abc-123', accepted: true },
        timestamp: Date.now(),
      });

      const result = await handleActionTool('ros2_action_send_goal', {
        action: '/navigate_to_pose',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { pose: { position: { x: 1, y: 2 } } },
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('action.send_goal', {
        action: '/navigate_to_pose',
        action_type: 'nav2_msgs/action/NavigateToPose',
        goal: { pose: { position: { x: 1, y: 2 } } },
      });
    });

    it('blocks goal when e-stop active', async () => {
      safety.activateEmergencyStop();

      const result = await handleActionTool('ros2_action_send_goal', {
        action: '/navigate_to_pose',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { pose: {} },
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('SAFETY BLOCKED');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error on bridge rejection', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'goal rejected' }, timestamp: Date.now() });

      const result = await handleActionTool('ros2_action_send_goal', {
        action: '/navigate_to_pose',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { pose: {} },
      }, connection, safety);

      expect(result.isError).toBe(true);
    });
  });

  describe('ros2_action_cancel', () => {
    it('cancels action with goal ID', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() });

      const result = await handleActionTool('ros2_action_cancel', {
        action: '/navigate_to_pose',
        goalId: 'abc-123',
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('cancelled');
      expect(connection.send).toHaveBeenCalledWith('action.cancel', {
        action: '/navigate_to_pose',
        goal_id: 'abc-123',
      });
    });

    it('cancels all goals when no goalId', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() });

      const result = await handleActionTool('ros2_action_cancel', {
        action: '/navigate_to_pose',
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(connection.send).toHaveBeenCalledWith('action.cancel', {
        action: '/navigate_to_pose',
        goal_id: undefined,
      });
    });

    it('returns error on cancel failure', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'error', data: { error: 'no active goal' }, timestamp: Date.now() });

      const result = await handleActionTool('ros2_action_cancel', {
        action: '/navigate_to_pose',
      }, connection, safety);

      expect(result.isError).toBe(true);
    });
  });

  describe('ros2_action_status', () => {
    it('gets action status from bridge', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: { goals: [{ id: 'abc', status: 'EXECUTING' }] },
        timestamp: Date.now(),
      });

      const result = await handleActionTool('ros2_action_status', {
        action: '/navigate_to_pose',
      }, connection, safety);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('EXECUTING');
    });
  });

  it('returns error for unknown action tool', async () => {
    const result = await handleActionTool('unknown_tool', {}, connection, safety);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown action tool');
  });
});

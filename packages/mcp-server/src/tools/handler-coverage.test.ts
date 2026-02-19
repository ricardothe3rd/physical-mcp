/**
 * Additional handler coverage tests focusing on error handling and edge cases.
 *
 * These tests complement the existing tests in tools.test.ts (safety handlers)
 * and response-format.test.ts (response structure) by exercising specific
 * error paths, safety-violation flows, and edge cases for:
 *   - Topic handlers (safety violation via checkPublish)
 *   - Service handlers (safety violation via checkServiceCall, e-stop)
 *   - Action handlers (cancel flow, e-stop blocked action goal)
 *   - System handlers (bridge status disconnected details, param formatting)
 *   - Batch handlers (empty commands, mixed success/failure, safety tool dispatch)
 */

import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { handleTopicTool } from './topic-tools.js';
import { handleServiceTool } from './service-tools.js';
import { handleActionTool } from './action-tools.js';
import { handleSystemTool } from './system-tools.js';
import { handleParamTool } from './param-tools.js';
import { handleBatchTool } from './batch-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

// ---------------------------------------------------------------------------
// Mock connection factory (same pattern as tools.test.ts)
// ---------------------------------------------------------------------------

function createMockConnection() {
  return {
    isConnected: false,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
    isBridgeAvailable: false,
  } as any;
}

// ---------------------------------------------------------------------------
// Topic handler edge cases
// ---------------------------------------------------------------------------

describe('Topic handler edge cases', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('handleTopicTool with unknown tool name returns error', async () => {
    const result = await handleTopicTool('ros2_topic_nonexistent', {}, connection, safety);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Unknown topic tool');
    expect(result.isError).toBe(true);
  });

  it('handleTopicTool("ros2_topic_publish") with safety violation for velocity exceeded', async () => {
    // The default policy has velocity limits; publish an excessively fast cmd_vel
    const result = await handleTopicTool('ros2_topic_publish', {
      topic: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist',
      message: {
        linear: { x: 100, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.content[0].text).toContain('velocity');
    expect(result.isError).toBe(true);
  });

  it('handleTopicTool("ros2_topic_publish") with e-stop active is blocked', async () => {
    safety.activateEmergencyStop();

    const result = await handleTopicTool('ros2_topic_publish', {
      topic: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist',
      message: {
        linear: { x: 0.1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.content[0].text).toContain('emergency_stop_active');
    expect(result.isError).toBe(true);

    safety.releaseEmergencyStop();
  });

  it('handleTopicTool("ros2_topic_publish") blocked topic is rejected', async () => {
    const result = await handleTopicTool('ros2_topic_publish', {
      topic: '/rosout',
      messageType: 'std_msgs/msg/String',
      message: { data: 'test' },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.isError).toBe(true);
  });

  it('handleTopicTool("ros2_topic_publish") bridge error response', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'error',
      data: { message: 'bridge publish failed' },
      timestamp: Date.now(),
    });

    const result = await handleTopicTool('ros2_topic_publish', {
      topic: '/safe_topic',
      messageType: 'std_msgs/msg/String',
      message: { data: 'hello' },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Error');
    expect(result.isError).toBe(true);
  });

  it('handleTopicTool("ros2_topic_info") with invalid topic name', async () => {
    const result = await handleTopicTool('ros2_topic_info', {
      topic: 'no-leading-slash',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Invalid topic name');
    expect(result.isError).toBe(true);
  });

  it('handleTopicTool("ros2_topic_echo") with valid topic returns data', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { data: 'hello world' },
      timestamp: Date.now(),
    });

    const result = await handleTopicTool('ros2_topic_echo', {
      topic: '/chatter',
      messageType: 'std_msgs/msg/String',
      timeoutSec: 3,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('hello world');
    expect(result.isError).toBeUndefined();
  });
});

// ---------------------------------------------------------------------------
// Service handler edge cases
// ---------------------------------------------------------------------------

describe('Service handler edge cases', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('handleServiceTool with unknown tool name returns error', async () => {
    const result = await handleServiceTool('ros2_service_nonexistent', {}, connection, safety);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Unknown service tool');
    expect(result.isError).toBe(true);
  });

  it('handleServiceTool("ros2_service_call") with e-stop active is blocked', async () => {
    safety.activateEmergencyStop();

    const result = await handleServiceTool('ros2_service_call', {
      service: '/test_service',
      serviceType: 'std_srvs/srv/SetBool',
      args: { data: true },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.content[0].text).toContain('emergency_stop_active');
    expect(result.isError).toBe(true);

    safety.releaseEmergencyStop();
  });

  it('handleServiceTool("ros2_service_call") blocked service is rejected', async () => {
    // /kill is in the default blocked services list
    const result = await handleServiceTool('ros2_service_call', {
      service: '/kill',
      serviceType: 'std_srvs/srv/Empty',
      args: {},
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.isError).toBe(true);
  });

  it('handleServiceTool("ros2_service_call") with invalid service name', async () => {
    const result = await handleServiceTool('ros2_service_call', {
      service: 'no-slash',
      serviceType: 'std_srvs/srv/SetBool',
      args: { data: true },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Invalid service name');
    expect(result.isError).toBe(true);
  });

  it('handleServiceTool("ros2_service_call") success returns bridge data', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { success: true, message: 'done' },
      timestamp: Date.now(),
    });

    const result = await handleServiceTool('ros2_service_call', {
      service: '/test_service',
      serviceType: 'std_srvs/srv/SetBool',
      args: { data: true },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('success');
    expect(result.isError).toBeUndefined();
  });

  it('handleServiceTool("ros2_service_info") with invalid name', async () => {
    const result = await handleServiceTool('ros2_service_info', {
      service: 'bad-name',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Invalid service name');
    expect(result.isError).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// Action handler edge cases
// ---------------------------------------------------------------------------

describe('Action handler edge cases', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('handleActionTool with unknown tool name returns error', async () => {
    const result = await handleActionTool('ros2_action_nonexistent', {}, connection, safety);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Unknown action tool');
    expect(result.isError).toBe(true);
  });

  it('handleActionTool("ros2_action_cancel") basic flow succeeds', async () => {
    const result = await handleActionTool('ros2_action_cancel', {
      action: '/navigate_to_pose',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('cancelled');
    expect(result.isError).toBeUndefined();
  });

  it('handleActionTool("ros2_action_cancel") with specific goalId', async () => {
    const result = await handleActionTool('ros2_action_cancel', {
      action: '/navigate_to_pose',
      goalId: 'goal-123',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('cancelled');
    expect(result.isError).toBeUndefined();
    // Verify the goalId was passed to the connection
    expect(connection.send).toHaveBeenCalledWith(
      expect.anything(),
      expect.objectContaining({ goal_id: 'goal-123' }),
    );
  });

  it('handleActionTool("ros2_action_cancel") bridge error', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'error',
      data: { message: 'no active goal' },
      timestamp: Date.now(),
    });

    const result = await handleActionTool('ros2_action_cancel', {
      action: '/navigate_to_pose',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Error');
    expect(result.isError).toBe(true);
  });

  it('handleActionTool("ros2_action_send_goal") with e-stop blocks the goal', async () => {
    safety.activateEmergencyStop();

    const result = await handleActionTool('ros2_action_send_goal', {
      action: '/navigate_to_pose',
      actionType: 'nav2_msgs/action/NavigateToPose',
      goal: { x: 1, y: 2 },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.isError).toBe(true);

    safety.releaseEmergencyStop();
  });

  it('handleActionTool("ros2_action_send_goal") with invalid action name', async () => {
    const result = await handleActionTool('ros2_action_send_goal', {
      action: 'bad-name',
      actionType: 'nav2_msgs/action/NavigateToPose',
      goal: { x: 1 },
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Invalid action name');
    expect(result.isError).toBe(true);
  });

  it('handleActionTool("ros2_action_status") returns data', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { goals: [{ id: 'g1', status: 'executing' }] },
      timestamp: Date.now(),
    });

    const result = await handleActionTool('ros2_action_status', {
      action: '/navigate_to_pose',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('executing');
    expect(result.isError).toBeUndefined();
  });
});

// ---------------------------------------------------------------------------
// System handler edge cases
// ---------------------------------------------------------------------------

describe('System handler edge cases', () => {
  let connection: ReturnType<typeof createMockConnection>;
  let safety: PolicyEngine;

  beforeEach(() => {
    connection = createMockConnection();
    safety = new PolicyEngine();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('handleSystemTool with unknown tool name returns error', async () => {
    const result = await handleSystemTool('system_nonexistent', {}, connection, safety);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Unknown system tool');
    expect(result.isError).toBe(true);
  });

  it('handleSystemTool("system_bridge_status") when disconnected returns not connected info', async () => {
    connection.isConnected = false;
    connection.isBridgeAvailable = false;

    const result = await handleSystemTool('system_bridge_status', {}, connection, safety);
    expect(result.content[0].type).toBe('text');

    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.connected).toBe(false);
    expect(parsed.available).toBe(false);
    expect(parsed.message).toBe('Bridge not connected');
    expect(result.isError).toBeUndefined();
  });

  it('handleSystemTool("system_bridge_status") when connected returns latency info', async () => {
    connection.isConnected = true;
    connection.isBridgeAvailable = true;
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { version: '1.0' },
      timestamp: Date.now(),
    });

    const result = await handleSystemTool('system_bridge_status', {}, connection, safety);
    expect(result.content[0].type).toBe('text');

    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.connected).toBe(true);
    expect(parsed.available).toBe(true);
    expect(typeof parsed.latencyMs).toBe('number');
    expect(result.isError).toBeUndefined();
  });

  it('handleSystemTool("system_bridge_status") when send throws returns error info', async () => {
    connection.isConnected = true;
    connection.isBridgeAvailable = true;
    connection.send.mockRejectedValue(new Error('ping timeout'));

    const result = await handleSystemTool('system_bridge_status', {}, connection, safety);
    expect(result.content[0].type).toBe('text');

    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.connected).toBe(true);
    expect(parsed.error).toContain('ping timeout');
    expect(result.isError).toBeUndefined();
  });

  it('handleParamTool("ros2_param_get") formats output correctly', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: 42,
      timestamp: Date.now(),
    });

    const result = await handleParamTool('ros2_param_get', {
      nodeName: '/my_node',
      paramName: 'speed',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('/my_node/speed');
    expect(result.content[0].text).toContain('42');
    expect(result.isError).toBeUndefined();
  });

  it('handleParamTool("ros2_param_set") formats output correctly', async () => {
    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/my_node',
      paramName: 'speed',
      value: 1.5,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Set /my_node/speed');
    expect(result.content[0].text).toContain('1.5');
    expect(result.isError).toBeUndefined();
  });

  it('handleSystemTool("system_node_info") bridge error', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'error',
      data: { message: 'node not found' },
      timestamp: Date.now(),
    });

    const result = await handleSystemTool('system_node_info', {
      nodeName: '/nonexistent_node',
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Error');
    expect(result.isError).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// Batch handler edge cases
// ---------------------------------------------------------------------------

describe('Batch handler edge cases', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('handleBatchTool with unknown tool name returns error', async () => {
    const result = await handleBatchTool('ros2_batch_nonexistent', {}, connection, safety);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Unknown batch tool');
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool with empty commands array returns error', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('No commands provided');
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool with null commands returns error', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: null,
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('No commands provided');
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool with undefined commands returns error', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('No commands provided');
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool exceeding 20 commands returns error', async () => {
    const commands = Array.from({ length: 21 }, () => ({
      tool: 'ros2_topic_list',
      args: {},
    }));

    const result = await handleBatchTool('ros2_batch_execute', {
      commands,
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Maximum 20 commands');
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool dispatches to safety tool handler for safety_ prefixed tools', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [{ tool: 'safety_status', args: {} }],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.summary).toContain('1 succeeded');
    expect(parsed.results[0].success).toBe(true);
    expect(result.isError).toBeUndefined();
  });

  it('handleBatchTool with unknown tool in batch marks that command as failed', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [{ tool: 'completely_unknown_tool', args: {} }],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.summary).toContain('0 succeeded');
    expect(parsed.summary).toContain('1 failed');
    expect(parsed.results[0].success).toBe(false);
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool with mixed success and failure commands', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [
        { tool: 'ros2_topic_list', args: {} },
        { tool: 'completely_unknown_tool', args: {} },
        { tool: 'ros2_service_list', args: {} },
      ],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.summary).toContain('2 succeeded');
    expect(parsed.summary).toContain('1 failed');
    expect(parsed.results).toHaveLength(3);
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool stopOnError stops at first failure', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [
        { tool: 'ros2_topic_list', args: {} },
        { tool: 'completely_unknown_tool', args: {} },
        { tool: 'ros2_service_list', args: {} },
      ],
      stopOnError: true,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    // Should stop after 2 commands (1 success + 1 failure), not run the 3rd
    expect(parsed.results).toHaveLength(2);
    expect(parsed.summary).toContain('2/3 executed');
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool dispatches to action handler', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [{ tool: 'ros2_action_list', args: {} }],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.results[0].success).toBe(true);
    expect(parsed.results[0].tool).toBe('ros2_action_list');
    expect(result.isError).toBeUndefined();
  });

  it('handleBatchTool dispatches to system handler', async () => {
    connection.isConnected = true;
    connection.isBridgeAvailable = true;

    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [{ tool: 'system_bridge_status', args: {} }],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.results[0].success).toBe(true);
    expect(parsed.results[0].tool).toBe('system_bridge_status');
    expect(result.isError).toBeUndefined();
  });

  it('handleBatchTool handles exception thrown by a command gracefully', async () => {
    connection.send.mockRejectedValueOnce(new Error('connection lost'));

    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [{ tool: 'ros2_topic_list', args: {} }],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.results[0].success).toBe(false);
    expect(parsed.results[0].result).toContain('connection lost');
    expect(result.isError).toBe(true);
  });

  it('handleBatchTool all commands succeed has isError undefined', async () => {
    const result = await handleBatchTool('ros2_batch_execute', {
      commands: [
        { tool: 'ros2_topic_list', args: {} },
        { tool: 'ros2_service_list', args: {} },
      ],
      stopOnError: false,
    }, connection, safety);

    expect(result.content[0].type).toBe('text');
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.summary).toContain('2 succeeded, 0 failed');
    expect(result.isError).toBeUndefined();
  });
});

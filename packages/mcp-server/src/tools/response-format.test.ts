/**
 * Response format consistency tests for ALL tool handlers.
 *
 * Verifies that every handler returns properly structured MCP responses:
 * - { content: [{ type: 'text', text: string }] }
 * - Error responses have isError: true
 * - Success responses don't have isError or have isError: undefined
 */

import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { handleTopicTool } from './topic-tools.js';
import { handleServiceTool } from './service-tools.js';
import { handleActionTool } from './action-tools.js';
import { handleSafetyTool } from './safety-tools.js';
import { handleSystemTool } from './system-tools.js';
import { handleBatchTool } from './batch-tools.js';
import { handleRecordingTool, resetRecordingSessions } from './recording-tools.js';
import { handleConditionalTool } from './conditional-tools.js';
import { handleScheduledTool, jobs } from './scheduled-tools.js';
import { handleTfTool } from './tf-tools.js';
import { handleDiagnosticTool } from './diagnostic-tools.js';
import { handleFleetTool, fleetRegistry } from './fleet-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
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

function createErrorConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({
      id: '1',
      status: 'error',
      data: { message: 'bridge error' },
      timestamp: Date.now(),
    }),
    disconnect: vi.fn(),
  } as any;
}

function createMockSafety(): PolicyEngine {
  return new PolicyEngine();
}

// ---------------------------------------------------------------------------
// Shared validators for the MCP response structure
// ---------------------------------------------------------------------------

interface McpResponse {
  content: Array<{ type: string; text: string }>;
  isError?: boolean;
}

function assertValidMcpResponse(response: McpResponse): void {
  // content must be an array
  expect(Array.isArray(response.content)).toBe(true);
  expect(response.content.length).toBeGreaterThanOrEqual(1);

  for (const item of response.content) {
    expect(item).toHaveProperty('type');
    expect(item.type).toBe('text');
    expect(item).toHaveProperty('text');
    expect(typeof item.text).toBe('string');
    expect(item.text.length).toBeGreaterThan(0);
  }
}

function assertSuccessResponse(response: McpResponse): void {
  assertValidMcpResponse(response);
  // Success: isError must be absent or undefined
  expect(response.isError === undefined || response.isError === true).toBe(
    response.isError === undefined,
  );
}

function assertErrorResponse(response: McpResponse): void {
  assertValidMcpResponse(response);
  expect(response.isError).toBe(true);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('Response format consistency', () => {
  let connection: ReturnType<typeof createMockConnection>;
  let errorConnection: ReturnType<typeof createErrorConnection>;
  let safety: PolicyEngine;

  beforeEach(() => {
    connection = createMockConnection();
    errorConnection = createErrorConnection();
    safety = createMockSafety();
  });

  afterEach(() => {
    safety.destroy();
    // Clean up scheduled jobs timers
    for (const [, job] of jobs) {
      if (job.timer) {
        clearInterval(job.timer as any);
        clearTimeout(job.timer as any);
      }
    }
    jobs.clear();
    resetRecordingSessions();
    fleetRegistry.clear();
  });

  // =========================================================================
  // Topic Tools
  // =========================================================================
  describe('Topic tools', () => {
    it('ros2_topic_list success response format', async () => {
      const res = await handleTopicTool('ros2_topic_list', {}, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_topic_list error response format', async () => {
      const res = await handleTopicTool('ros2_topic_list', {}, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_info success response format', async () => {
      const res = await handleTopicTool('ros2_topic_info', { topic: '/test' }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_topic_info error from bridge', async () => {
      const res = await handleTopicTool('ros2_topic_info', { topic: '/test' }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_info invalid name error', async () => {
      const res = await handleTopicTool('ros2_topic_info', { topic: 'bad-name' }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_subscribe success response format', async () => {
      const res = await handleTopicTool('ros2_topic_subscribe', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        count: 1,
        timeoutSec: 1,
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_topic_subscribe error from bridge', async () => {
      const res = await handleTopicTool('ros2_topic_subscribe', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
      }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_subscribe invalid topic', async () => {
      const res = await handleTopicTool('ros2_topic_subscribe', {
        topic: 'no-slash',
        messageType: 'std_msgs/msg/String',
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_publish success response format', async () => {
      const res = await handleTopicTool('ros2_topic_publish', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        message: { data: 'hello' },
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_topic_publish error from bridge', async () => {
      const res = await handleTopicTool('ros2_topic_publish', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        message: { data: 'hello' },
      }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_publish invalid topic name', async () => {
      const res = await handleTopicTool('ros2_topic_publish', {
        topic: 'invalid',
        messageType: 'std_msgs/msg/String',
        message: { data: 'hello' },
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_publish invalid message payload', async () => {
      const res = await handleTopicTool('ros2_topic_publish', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        message: null,
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_publish safety blocked (blocked topic)', async () => {
      const res = await handleTopicTool('ros2_topic_publish', {
        topic: '/rosout',
        messageType: 'std_msgs/msg/String',
        message: { data: 'hello' },
      }, connection, safety);
      assertErrorResponse(res);
      expect(res.content[0].text).toContain('SAFETY BLOCKED');
    });

    it('ros2_topic_echo success response format', async () => {
      const res = await handleTopicTool('ros2_topic_echo', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        timeoutSec: 1,
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_topic_echo error from bridge', async () => {
      const res = await handleTopicTool('ros2_topic_echo', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
      }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_topic_echo invalid topic', async () => {
      const res = await handleTopicTool('ros2_topic_echo', {
        topic: 'bad',
        messageType: 'std_msgs/msg/String',
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('unknown topic tool returns error', async () => {
      const res = await handleTopicTool('ros2_topic_unknown', {}, connection, safety);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Service Tools
  // =========================================================================
  describe('Service tools', () => {
    it('ros2_service_list success response format', async () => {
      const res = await handleServiceTool('ros2_service_list', {}, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_service_list error from bridge', async () => {
      const res = await handleServiceTool('ros2_service_list', {}, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_service_info success response format', async () => {
      const res = await handleServiceTool('ros2_service_info', { service: '/test_service' }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_service_info error from bridge', async () => {
      const res = await handleServiceTool('ros2_service_info', { service: '/test_service' }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_service_info invalid service name', async () => {
      const res = await handleServiceTool('ros2_service_info', { service: 'no-slash' }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_service_call success response format', async () => {
      const res = await handleServiceTool('ros2_service_call', {
        service: '/test_service',
        serviceType: 'std_srvs/srv/SetBool',
        args: { data: true },
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_service_call error from bridge', async () => {
      const res = await handleServiceTool('ros2_service_call', {
        service: '/test_service',
        serviceType: 'std_srvs/srv/SetBool',
        args: { data: true },
      }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_service_call invalid service name', async () => {
      const res = await handleServiceTool('ros2_service_call', {
        service: 'bad',
        serviceType: 'std_srvs/srv/SetBool',
        args: {},
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_service_call safety blocked (blocked service)', async () => {
      const res = await handleServiceTool('ros2_service_call', {
        service: '/kill',
        serviceType: 'std_srvs/srv/Empty',
        args: {},
      }, connection, safety);
      assertErrorResponse(res);
      expect(res.content[0].text).toContain('SAFETY BLOCKED');
    });

    it('unknown service tool returns error', async () => {
      const res = await handleServiceTool('ros2_service_unknown', {}, connection, safety);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Action Tools
  // =========================================================================
  describe('Action tools', () => {
    it('ros2_action_list success response format', async () => {
      const res = await handleActionTool('ros2_action_list', {}, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_action_list error from bridge', async () => {
      const res = await handleActionTool('ros2_action_list', {}, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_action_send_goal success response format', async () => {
      const res = await handleActionTool('ros2_action_send_goal', {
        action: '/navigate',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { x: 1 },
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_action_send_goal error from bridge', async () => {
      const res = await handleActionTool('ros2_action_send_goal', {
        action: '/navigate',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { x: 1 },
      }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_action_send_goal invalid action name', async () => {
      const res = await handleActionTool('ros2_action_send_goal', {
        action: 'bad',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { x: 1 },
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_action_send_goal safety blocked (estop)', async () => {
      safety.activateEmergencyStop();
      const res = await handleActionTool('ros2_action_send_goal', {
        action: '/navigate',
        actionType: 'nav2_msgs/action/NavigateToPose',
        goal: { x: 1 },
      }, connection, safety);
      assertErrorResponse(res);
      expect(res.content[0].text).toContain('SAFETY BLOCKED');
      safety.releaseEmergencyStop();
    });

    it('ros2_action_cancel success response format', async () => {
      const res = await handleActionTool('ros2_action_cancel', {
        action: '/navigate',
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_action_cancel error from bridge', async () => {
      const res = await handleActionTool('ros2_action_cancel', {
        action: '/navigate',
      }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('ros2_action_status success response format', async () => {
      const res = await handleActionTool('ros2_action_status', {
        action: '/navigate',
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('ros2_action_status error from bridge', async () => {
      const res = await handleActionTool('ros2_action_status', {
        action: '/navigate',
      }, errorConnection, safety);
      assertErrorResponse(res);
    });

    it('unknown action tool returns error', async () => {
      const res = await handleActionTool('ros2_action_unknown', {}, connection, safety);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Safety Tools
  // =========================================================================
  describe('Safety tools', () => {
    it('safety_status success response format', async () => {
      const res = await handleSafetyTool('safety_status', {}, connection, safety);
      assertSuccessResponse(res);
    });

    it('safety_emergency_stop success response format', async () => {
      const res = await handleSafetyTool('safety_emergency_stop', { reason: 'test' }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('EMERGENCY STOP ACTIVATED');
      safety.releaseEmergencyStop();
    });

    it('safety_emergency_stop without reason', async () => {
      const res = await handleSafetyTool('safety_emergency_stop', {}, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Manual activation');
      safety.releaseEmergencyStop();
    });

    it('safety_emergency_stop_release success', async () => {
      safety.activateEmergencyStop();
      const res = await handleSafetyTool('safety_emergency_stop_release', {
        confirmation: 'CONFIRM_RELEASE',
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('safety_emergency_stop_release wrong confirmation', async () => {
      const res = await handleSafetyTool('safety_emergency_stop_release', {
        confirmation: 'WRONG',
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('safety_get_policy success response format', async () => {
      const res = await handleSafetyTool('safety_get_policy', {}, connection, safety);
      assertSuccessResponse(res);
    });

    it('safety_update_velocity_limits success', async () => {
      const res = await handleSafetyTool('safety_update_velocity_limits', {
        linearMax: 1.0,
        angularMax: 2.0,
      }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Velocity limits updated');
    });

    it('safety_update_geofence success', async () => {
      const res = await handleSafetyTool('safety_update_geofence', {
        xMin: -10,
        xMax: 10,
      }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Geofence updated');
    });

    it('safety_audit_log success', async () => {
      const res = await handleSafetyTool('safety_audit_log', {
        limit: 5,
        violationsOnly: false,
      }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Audit Log');
    });

    it('safety_set_clamp_mode enabled', async () => {
      const res = await handleSafetyTool('safety_set_clamp_mode', { enabled: true }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('ENABLED');
    });

    it('safety_set_clamp_mode disabled', async () => {
      const res = await handleSafetyTool('safety_set_clamp_mode', { enabled: false }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('DISABLED');
    });

    it('safety_deadman_switch success', async () => {
      const res = await handleSafetyTool('safety_deadman_switch', {
        enabled: false,
        timeoutMs: 30000,
      }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Deadman switch');
    });

    it('safety_heartbeat success', async () => {
      const res = await handleSafetyTool('safety_heartbeat', {}, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Heartbeat received');
    });

    it('safety_update_acceleration_limits success', async () => {
      const res = await handleSafetyTool('safety_update_acceleration_limits', {
        enabled: true,
        linearMaxAccel: 2.0,
        angularMaxAccel: 5.0,
      }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Acceleration limits updated');
    });

    it('safety_export_audit_log error (bad path)', async () => {
      const res = await handleSafetyTool('safety_export_audit_log', {
        filePath: '/nonexistent/dir/audit.json',
        violationsOnly: false,
      }, connection, safety);
      assertErrorResponse(res);
      expect(res.content[0].text).toContain('Failed to export');
    });

    it('safety_check_position success', async () => {
      const res = await handleSafetyTool('safety_check_position', {
        x: 0,
        y: 0,
        z: 1,
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('safety_validate_policy success', async () => {
      const res = await handleSafetyTool('safety_validate_policy', {}, connection, safety);
      assertValidMcpResponse(res);
      // The default policy is valid, so isError should be undefined
      expect(res.content[0].text).toContain('VALID');
    });

    it('safety_approval_list success (empty)', async () => {
      const res = await handleSafetyTool('safety_approval_list', {}, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('No pending approvals');
    });

    it('safety_approval_approve non-existent', async () => {
      const res = await handleSafetyTool('safety_approval_approve', {
        approvalId: 'nonexistent',
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('safety_approval_deny non-existent', async () => {
      const res = await handleSafetyTool('safety_approval_deny', {
        approvalId: 'nonexistent',
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('safety_approval_config success', async () => {
      const res = await handleSafetyTool('safety_approval_config', {}, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Command approval config');
    });

    it('safety_violation_mode success (get)', async () => {
      const res = await handleSafetyTool('safety_violation_mode', {}, connection, safety);
      assertSuccessResponse(res);
    });

    it('safety_violation_mode success (set to warn)', async () => {
      const res = await handleSafetyTool('safety_violation_mode', { mode: 'warn' }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('warn');
      // Reset back
      safety.setViolationMode('block');
    });

    it('safety_time_policy success (get)', async () => {
      const res = await handleSafetyTool('safety_time_policy', {}, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Time-based policies');
    });

    it('safety_time_policy success (set)', async () => {
      const res = await handleSafetyTool('safety_time_policy', {
        enabled: true,
        schedules: [],
      }, connection, safety);
      assertSuccessResponse(res);
    });

    it('safety_time_policy_status success', async () => {
      const res = await handleSafetyTool('safety_time_policy_status', {}, connection, safety);
      assertSuccessResponse(res);
    });

    it('unknown safety tool returns error', async () => {
      const res = await handleSafetyTool('safety_unknown', {}, connection, safety);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // System Tools
  // =========================================================================
  describe('System tools', () => {
    it('system_bridge_status connected success', async () => {
      const res = await handleSystemTool('system_bridge_status', {}, connection);
      assertSuccessResponse(res);
    });

    it('system_bridge_status disconnected', async () => {
      const disconnected = createMockConnection();
      disconnected.isConnected = false;
      const res = await handleSystemTool('system_bridge_status', {}, disconnected);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('false');
    });

    it('system_node_list success', async () => {
      const res = await handleSystemTool('system_node_list', {}, connection);
      assertSuccessResponse(res);
    });

    it('system_node_list error from bridge', async () => {
      const res = await handleSystemTool('system_node_list', {}, errorConnection);
      assertErrorResponse(res);
    });

    it('system_node_info success', async () => {
      const res = await handleSystemTool('system_node_info', {
        nodeName: '/test_node',
      }, connection);
      assertSuccessResponse(res);
    });

    it('system_node_info error from bridge', async () => {
      const res = await handleSystemTool('system_node_info', {
        nodeName: '/test_node',
      }, errorConnection);
      assertErrorResponse(res);
    });

    it('ros2_param_list success', async () => {
      const res = await handleSystemTool('ros2_param_list', {
        nodeName: '/test_node',
      }, connection);
      assertSuccessResponse(res);
    });

    it('ros2_param_list error from bridge', async () => {
      const res = await handleSystemTool('ros2_param_list', {
        nodeName: '/test_node',
      }, errorConnection);
      assertErrorResponse(res);
    });

    it('ros2_param_get success', async () => {
      const res = await handleSystemTool('ros2_param_get', {
        nodeName: '/test_node',
        paramName: 'speed',
      }, connection);
      assertSuccessResponse(res);
    });

    it('ros2_param_get error from bridge', async () => {
      const res = await handleSystemTool('ros2_param_get', {
        nodeName: '/test_node',
        paramName: 'speed',
      }, errorConnection);
      assertErrorResponse(res);
    });

    it('ros2_param_set success', async () => {
      const res = await handleSystemTool('ros2_param_set', {
        nodeName: '/test_node',
        paramName: 'speed',
        value: 1.5,
      }, connection);
      assertSuccessResponse(res);
    });

    it('ros2_param_set error from bridge', async () => {
      const res = await handleSystemTool('ros2_param_set', {
        nodeName: '/test_node',
        paramName: 'speed',
        value: 1.5,
      }, errorConnection);
      assertErrorResponse(res);
    });

    it('unknown system tool returns error', async () => {
      const res = await handleSystemTool('system_unknown', {}, connection);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Batch Tools
  // =========================================================================
  describe('Batch tools', () => {
    it('ros2_batch_execute single success command', async () => {
      const res = await handleBatchTool('ros2_batch_execute', {
        commands: [{ tool: 'ros2_topic_list', args: {} }],
        stopOnError: false,
      }, connection, safety);
      assertSuccessResponse(res);
      const parsed = JSON.parse(res.content[0].text);
      expect(parsed.summary).toContain('1 succeeded');
    });

    it('ros2_batch_execute multiple commands', async () => {
      const res = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_list', args: {} },
          { tool: 'ros2_service_list', args: {} },
        ],
        stopOnError: false,
      }, connection, safety);
      assertValidMcpResponse(res);
      const parsed = JSON.parse(res.content[0].text);
      expect(parsed.summary).toContain('2 succeeded');
    });

    it('ros2_batch_execute with error command', async () => {
      const res = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_list', args: {} },
          { tool: 'unknown_tool', args: {} },
        ],
        stopOnError: false,
      }, connection, safety);
      assertValidMcpResponse(res);
      // hasErrors = true due to unknown tool
      expect(res.isError).toBe(true);
    });

    it('ros2_batch_execute stopOnError stops early', async () => {
      const res = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'unknown_tool', args: {} },
          { tool: 'ros2_topic_list', args: {} },
        ],
        stopOnError: true,
      }, connection, safety);
      assertValidMcpResponse(res);
      expect(res.isError).toBe(true);
      const parsed = JSON.parse(res.content[0].text);
      // Only 1 of 2 commands executed because stopOnError
      expect(parsed.results.length).toBe(1);
    });

    it('ros2_batch_execute no commands provided', async () => {
      const res = await handleBatchTool('ros2_batch_execute', {
        commands: [],
        stopOnError: false,
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_batch_execute too many commands', async () => {
      const commands = Array.from({ length: 21 }, () => ({
        tool: 'ros2_topic_list',
        args: {},
      }));
      const res = await handleBatchTool('ros2_batch_execute', {
        commands,
        stopOnError: false,
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('unknown batch tool returns error', async () => {
      const res = await handleBatchTool('ros2_batch_unknown', {}, connection, safety);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Recording Tools
  // =========================================================================
  describe('Recording tools', () => {
    it('ros2_topic_record_start success', async () => {
      const res = await handleRecordingTool('ros2_topic_record_start', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        maxMessages: 10,
      }, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Recording started');
    });

    it('ros2_topic_record_stop success', async () => {
      // First start a session
      await handleRecordingTool('ros2_topic_record_start', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        maxMessages: 10,
      }, connection);

      const res = await handleRecordingTool('ros2_topic_record_stop', {
        sessionId: 'rec_1',
      }, connection);
      assertSuccessResponse(res);
    });

    it('ros2_topic_record_stop not found', async () => {
      const res = await handleRecordingTool('ros2_topic_record_stop', {
        sessionId: 'nonexistent',
      }, connection);
      assertErrorResponse(res);
    });

    it('ros2_topic_record_status success (empty)', async () => {
      const res = await handleRecordingTool('ros2_topic_record_status', {}, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('No active recordings');
    });

    it('ros2_topic_record_status success (with sessions)', async () => {
      await handleRecordingTool('ros2_topic_record_start', {
        topic: '/test',
        messageType: 'std_msgs/msg/String',
        maxMessages: 10,
      }, connection);

      const res = await handleRecordingTool('ros2_topic_record_status', {}, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Active recordings');
    });

    it('unknown recording tool returns error', async () => {
      const res = await handleRecordingTool('ros2_topic_record_unknown', {}, connection);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Conditional Tools
  // =========================================================================
  describe('Conditional tools', () => {
    it('ros2_conditional_execute condition true with then-tool', async () => {
      connection.send.mockResolvedValue({
        id: '1',
        status: 'ok',
        data: { message: { percentage: 50 } },
        timestamp: Date.now(),
      });
      const res = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/battery',
        field: 'percentage',
        operator: 'gt',
        value: 20,
        thenTool: 'ros2_topic_list',
        thenArgs: {},
      }, connection, safety);
      assertValidMcpResponse(res);
    });

    it('ros2_conditional_execute condition false no else-tool', async () => {
      connection.send.mockResolvedValue({
        id: '1',
        status: 'ok',
        data: { message: { percentage: 10 } },
        timestamp: Date.now(),
      });
      const res = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/battery',
        field: 'percentage',
        operator: 'gt',
        value: 20,
        thenTool: 'ros2_topic_list',
        thenArgs: {},
      }, connection, safety);
      assertSuccessResponse(res);
      const parsed = JSON.parse(res.content[0].text);
      expect(parsed.conditionMet).toBe(false);
      expect(parsed.action).toContain('none');
    });

    it('ros2_conditional_execute topic read failure', async () => {
      connection.send.mockRejectedValue(new Error('connection failed'));
      const res = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/battery',
        field: 'percentage',
        operator: 'gt',
        value: 20,
        thenTool: 'ros2_topic_list',
        thenArgs: {},
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_wait_for_condition success (met immediately)', async () => {
      connection.send.mockResolvedValue({
        id: '1',
        status: 'ok',
        data: { message: { value: 100 } },
        timestamp: Date.now(),
      });
      const res = await handleConditionalTool('ros2_wait_for_condition', {
        topic: '/sensor',
        field: 'value',
        operator: 'gte',
        value: 50,
        timeoutMs: 1000,
        pollIntervalMs: 100,
      }, connection, safety);
      assertSuccessResponse(res);
      const parsed = JSON.parse(res.content[0].text);
      expect(parsed.conditionMet).toBe(true);
    });

    it('ros2_wait_for_condition timeout', async () => {
      connection.send.mockResolvedValue({
        id: '1',
        status: 'ok',
        data: { message: { value: 10 } },
        timestamp: Date.now(),
      });
      const res = await handleConditionalTool('ros2_wait_for_condition', {
        topic: '/sensor',
        field: 'value',
        operator: 'gte',
        value: 1000,
        timeoutMs: 200,
        pollIntervalMs: 50,
      }, connection, safety);
      assertErrorResponse(res);
      const parsed = JSON.parse(res.content[0].text);
      expect(parsed.conditionMet).toBe(false);
      expect(parsed.reason).toBe('timeout');
    });

    it('unknown conditional tool returns error', async () => {
      const res = await handleConditionalTool('ros2_conditional_unknown', {}, connection, safety);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Scheduled Tools
  // =========================================================================
  describe('Scheduled tools', () => {
    it('ros2_schedule_command success', async () => {
      const res = await handleScheduledTool('ros2_schedule_command', {
        tool: 'ros2_topic_list',
        args: {},
        delayMs: 60000, // Long delay so it won't fire during tests
        repeat: false,
      }, connection, safety);
      assertSuccessResponse(res);
      const parsed = JSON.parse(res.content[0].text);
      expect(parsed.jobId).toBeDefined();
    });

    it('ros2_schedule_command repeat without intervalMs', async () => {
      const res = await handleScheduledTool('ros2_schedule_command', {
        tool: 'ros2_topic_list',
        args: {},
        delayMs: 1000,
        repeat: true,
        // intentionally no intervalMs
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_schedule_cancel success', async () => {
      // Schedule first and extract the generated jobId
      const schedRes = await handleScheduledTool('ros2_schedule_command', {
        tool: 'ros2_topic_list',
        args: {},
        delayMs: 60000,
        repeat: false,
      }, connection, safety);
      const jobId = JSON.parse(schedRes.content[0].text).jobId;

      const res = await handleScheduledTool('ros2_schedule_cancel', {
        jobId,
      }, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('cancelled');
    });

    it('ros2_schedule_cancel not found', async () => {
      const res = await handleScheduledTool('ros2_schedule_cancel', {
        jobId: 'nonexistent',
      }, connection, safety);
      assertErrorResponse(res);
    });

    it('ros2_schedule_list success', async () => {
      const res = await handleScheduledTool('ros2_schedule_list', {}, connection, safety);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Scheduled jobs');
    });

    it('unknown scheduled tool returns error', async () => {
      const res = await handleScheduledTool('ros2_schedule_unknown', {}, connection, safety);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // TF Tools
  // =========================================================================
  describe('TF tools', () => {
    it('ros2_tf_tree success', async () => {
      const res = await handleTfTool('ros2_tf_tree', {}, connection);
      assertSuccessResponse(res);
    });

    it('ros2_tf_tree error from bridge', async () => {
      const res = await handleTfTool('ros2_tf_tree', {}, errorConnection);
      assertErrorResponse(res);
    });

    it('ros2_tf_lookup success', async () => {
      const res = await handleTfTool('ros2_tf_lookup', {
        sourceFrame: 'base_link',
        targetFrame: 'odom',
      }, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('base_link');
      expect(res.content[0].text).toContain('odom');
    });

    it('ros2_tf_lookup error from bridge', async () => {
      const res = await handleTfTool('ros2_tf_lookup', {
        sourceFrame: 'base_link',
        targetFrame: 'odom',
      }, errorConnection);
      assertErrorResponse(res);
    });

    it('ros2_tf_lookup missing frames', async () => {
      const res = await handleTfTool('ros2_tf_lookup', {
        sourceFrame: '',
        targetFrame: '',
      }, connection);
      assertErrorResponse(res);
    });

    it('unknown TF tool returns error', async () => {
      const res = await handleTfTool('ros2_tf_unknown', {}, connection);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Diagnostic Tools
  // =========================================================================
  describe('Diagnostic tools', () => {
    it('ros2_diagnostics_summary success', async () => {
      const res = await handleDiagnosticTool('ros2_diagnostics_summary', {}, connection);
      assertSuccessResponse(res);
    });

    it('ros2_diagnostics_summary error from bridge', async () => {
      const res = await handleDiagnosticTool('ros2_diagnostics_summary', {}, errorConnection);
      assertErrorResponse(res);
    });

    it('ros2_diagnostics_detail success', async () => {
      const res = await handleDiagnosticTool('ros2_diagnostics_detail', {
        name: 'motor_controller',
      }, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('motor_controller');
    });

    it('ros2_diagnostics_detail error from bridge', async () => {
      const res = await handleDiagnosticTool('ros2_diagnostics_detail', {
        name: 'motor_controller',
      }, errorConnection);
      assertErrorResponse(res);
    });

    it('ros2_diagnostics_detail missing name', async () => {
      const res = await handleDiagnosticTool('ros2_diagnostics_detail', {
        name: '',
      }, connection);
      assertErrorResponse(res);
    });

    it('unknown diagnostic tool returns error', async () => {
      const res = await handleDiagnosticTool('ros2_diagnostics_unknown', {}, connection);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Fleet Tools
  // =========================================================================
  describe('Fleet tools', () => {
    it('ros2_fleet_status success (empty)', async () => {
      const res = await handleFleetTool('ros2_fleet_status', {}, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Fleet robots (0)');
    });

    it('ros2_fleet_add success', async () => {
      const res = await handleFleetTool('ros2_fleet_add', {
        namespace: '/robot1',
      }, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('robot1');
      expect(res.content[0].text).toContain('added');
    });

    it('ros2_fleet_add missing namespace', async () => {
      const res = await handleFleetTool('ros2_fleet_add', {
        namespace: '',
      }, connection);
      assertErrorResponse(res);
    });

    it('ros2_fleet_add duplicate', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      const res = await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      assertErrorResponse(res);
      expect(res.content[0].text).toContain('already registered');
    });

    it('ros2_fleet_remove success', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      const res = await handleFleetTool('ros2_fleet_remove', { namespace: '/robot1' }, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('removed');
    });

    it('ros2_fleet_remove missing namespace', async () => {
      const res = await handleFleetTool('ros2_fleet_remove', { namespace: '' }, connection);
      assertErrorResponse(res);
    });

    it('ros2_fleet_remove not found', async () => {
      const res = await handleFleetTool('ros2_fleet_remove', { namespace: '/nonexistent' }, connection);
      assertErrorResponse(res);
    });

    it('ros2_fleet_status with robots', async () => {
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot1' }, connection);
      await handleFleetTool('ros2_fleet_add', { namespace: '/robot2' }, connection);
      const res = await handleFleetTool('ros2_fleet_status', {}, connection);
      assertSuccessResponse(res);
      expect(res.content[0].text).toContain('Fleet robots (2)');
    });

    it('unknown fleet tool returns error', async () => {
      const res = await handleFleetTool('ros2_fleet_unknown', {}, connection);
      assertErrorResponse(res);
    });
  });

  // =========================================================================
  // Cross-cutting: every handler rejects unknown tool names with error format
  // =========================================================================
  describe('All handlers reject unknown tools with proper format', () => {
    const cases: Array<{
      name: string;
      handler: (...args: any[]) => Promise<McpResponse>;
      argsFactory: () => any[];
    }> = [
      {
        name: 'topic',
        handler: handleTopicTool,
        argsFactory: () => ['unknown_topic_tool', {}, connection, safety],
      },
      {
        name: 'service',
        handler: handleServiceTool,
        argsFactory: () => ['unknown_service_tool', {}, connection, safety],
      },
      {
        name: 'action',
        handler: handleActionTool,
        argsFactory: () => ['unknown_action_tool', {}, connection, safety],
      },
      {
        name: 'safety',
        handler: handleSafetyTool,
        argsFactory: () => ['unknown_safety_tool', {}, connection, safety],
      },
      {
        name: 'system',
        handler: handleSystemTool,
        argsFactory: () => ['unknown_system_tool', {}, connection],
      },
      {
        name: 'batch',
        handler: handleBatchTool,
        argsFactory: () => ['unknown_batch_tool', {}, connection, safety],
      },
      {
        name: 'recording',
        handler: handleRecordingTool,
        argsFactory: () => ['unknown_recording_tool', {}, connection],
      },
      {
        name: 'conditional',
        handler: handleConditionalTool,
        argsFactory: () => ['unknown_conditional_tool', {}, connection, safety],
      },
      {
        name: 'scheduled',
        handler: handleScheduledTool,
        argsFactory: () => ['unknown_scheduled_tool', {}, connection, safety],
      },
      {
        name: 'tf',
        handler: handleTfTool,
        argsFactory: () => ['unknown_tf_tool', {}, connection],
      },
      {
        name: 'diagnostic',
        handler: handleDiagnosticTool,
        argsFactory: () => ['unknown_diagnostic_tool', {}, connection],
      },
      {
        name: 'fleet',
        handler: handleFleetTool,
        argsFactory: () => ['unknown_fleet_tool', {}, connection],
      },
    ];

    for (const { name, handler, argsFactory } of cases) {
      it(`${name} handler rejects unknown tool with isError: true`, async () => {
        const args = argsFactory();
        const res = await (handler as any)(...args);
        assertErrorResponse(res);
        expect(res.content[0].text).toContain('Unknown');
      });
    }
  });

  // =========================================================================
  // Cross-cutting: success responses never have isError: true
  // =========================================================================
  describe('Success responses have isError undefined', () => {
    it('topic_list success has no isError', async () => {
      const res = await handleTopicTool('ros2_topic_list', {}, connection, safety);
      expect(res.isError).toBeUndefined();
    });

    it('service_list success has no isError', async () => {
      const res = await handleServiceTool('ros2_service_list', {}, connection, safety);
      expect(res.isError).toBeUndefined();
    });

    it('action_list success has no isError', async () => {
      const res = await handleActionTool('ros2_action_list', {}, connection, safety);
      expect(res.isError).toBeUndefined();
    });

    it('safety_status success has no isError', async () => {
      const res = await handleSafetyTool('safety_status', {}, connection, safety);
      expect(res.isError).toBeUndefined();
    });

    it('system_node_list success has no isError', async () => {
      const res = await handleSystemTool('system_node_list', {}, connection);
      expect(res.isError).toBeUndefined();
    });

    it('fleet_status success has no isError', async () => {
      const res = await handleFleetTool('ros2_fleet_status', {}, connection);
      expect(res.isError).toBeUndefined();
    });

    it('tf_tree success has no isError', async () => {
      const res = await handleTfTool('ros2_tf_tree', {}, connection);
      expect(res.isError).toBeUndefined();
    });

    it('diagnostics_summary success has no isError', async () => {
      const res = await handleDiagnosticTool('ros2_diagnostics_summary', {}, connection);
      expect(res.isError).toBeUndefined();
    });

    it('recording_status success has no isError', async () => {
      const res = await handleRecordingTool('ros2_topic_record_status', {}, connection);
      expect(res.isError).toBeUndefined();
    });

    it('schedule_list success has no isError', async () => {
      const res = await handleScheduledTool('ros2_schedule_list', {}, connection, safety);
      expect(res.isError).toBeUndefined();
    });
  });
});

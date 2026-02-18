/**
 * Tests for MCP tool definitions and safety tool handlers.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getTopicTools } from './topic-tools.js';
import { getServiceTools } from './service-tools.js';
import { getActionTools } from './action-tools.js';
import { getSafetyTools, handleSafetyTool } from './safety-tools.js';
import { getSystemTools } from './system-tools.js';
import { getBatchTools } from './batch-tools.js';
import { getRecordingTools } from './recording-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

// Create a simple mock object instead of mocking the module
function createMockConnection() {
  return {
    isConnected: false,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
    isBridgeAvailable: false,
  } as any;
}

describe('Tool Definitions', () => {
  it('topic tools have correct count', () => {
    const tools = getTopicTools();
    expect(tools.length).toBe(5);
    expect(tools.map(t => t.name)).toContain('ros2_topic_list');
    expect(tools.map(t => t.name)).toContain('ros2_topic_publish');
  });

  it('service tools have correct count', () => {
    const tools = getServiceTools();
    expect(tools.length).toBe(3);
    expect(tools.map(t => t.name)).toContain('ros2_service_list');
    expect(tools.map(t => t.name)).toContain('ros2_service_call');
  });

  it('action tools have correct count', () => {
    const tools = getActionTools();
    expect(tools.length).toBe(4);
    expect(tools.map(t => t.name)).toContain('ros2_action_send_goal');
  });

  it('safety tools have correct count', () => {
    const tools = getSafetyTools();
    expect(tools.length).toBe(21);
    expect(tools.map(t => t.name)).toContain('safety_emergency_stop');
    expect(tools.map(t => t.name)).toContain('safety_heartbeat');
    expect(tools.map(t => t.name)).toContain('safety_update_acceleration_limits');
    expect(tools.map(t => t.name)).toContain('safety_export_audit_log');
    expect(tools.map(t => t.name)).toContain('safety_check_position');
  });

  it('system tools have correct count', () => {
    const tools = getSystemTools();
    expect(tools.length).toBe(6);
    expect(tools.map(t => t.name)).toContain('ros2_param_get');
    expect(tools.map(t => t.name)).toContain('ros2_param_set');
  });

  it('batch tools have correct count', () => {
    const tools = getBatchTools();
    expect(tools.length).toBe(1);
    expect(tools.map(t => t.name)).toContain('ros2_batch_execute');
  });

  it('recording tools have correct count', () => {
    const tools = getRecordingTools();
    expect(tools.length).toBe(3);
    expect(tools.map(t => t.name)).toContain('ros2_topic_record_start');
    expect(tools.map(t => t.name)).toContain('ros2_topic_record_stop');
  });

  it('all tools have unique names', () => {
    const allTools = [
      ...getTopicTools(),
      ...getServiceTools(),
      ...getActionTools(),
      ...getSafetyTools(),
      ...getSystemTools(),
      ...getBatchTools(),
      ...getRecordingTools(),
    ];
    const names = allTools.map(t => t.name);
    const uniqueNames = new Set(names);
    expect(uniqueNames.size).toBe(names.length);
  });

  it('all tools have descriptions', () => {
    const allTools = [
      ...getTopicTools(),
      ...getServiceTools(),
      ...getActionTools(),
      ...getSafetyTools(),
      ...getSystemTools(),
      ...getBatchTools(),
      ...getRecordingTools(),
    ];
    for (const tool of allTools) {
      expect(tool.description).toBeTruthy();
      expect(tool.description.length).toBeGreaterThan(10);
    }
  });

  it('all tools have input schemas', () => {
    const allTools = [
      ...getTopicTools(),
      ...getServiceTools(),
      ...getActionTools(),
      ...getSafetyTools(),
      ...getSystemTools(),
      ...getBatchTools(),
      ...getRecordingTools(),
    ];
    for (const tool of allTools) {
      expect(tool.inputSchema).toBeDefined();
      expect(tool.inputSchema.type).toBe('object');
    }
  });

  it('total tool count is 40', () => {
    const total =
      getTopicTools().length +
      getServiceTools().length +
      getActionTools().length +
      getSafetyTools().length +
      getSystemTools().length +
      getBatchTools().length +
      getRecordingTools().length;
    expect(total).toBe(43);
  });
});

describe('Safety Tool Handlers', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    safety = new PolicyEngine();
    connection = createMockConnection();
  });

  it('safety_status returns JSON status', async () => {
    const result = await handleSafetyTool('safety_status', {}, connection, safety);
    expect(result.content[0].type).toBe('text');
    const status = JSON.parse(result.content[0].text);
    expect(status.policyName).toBe('default');
    expect(status.emergencyStopActive).toBe(false);
    expect(status.acceleration).toBeDefined();
    expect(status.rateLimiterStats).toBeDefined();
  });

  it('safety_emergency_stop activates e-stop', async () => {
    const result = await handleSafetyTool('safety_emergency_stop', { reason: 'test' }, connection, safety);
    expect(result.content[0].text).toContain('EMERGENCY STOP');
    expect(safety.isEmergencyStopActive).toBe(true);
  });

  it('safety_emergency_stop_release requires confirmation', async () => {
    safety.activateEmergencyStop();
    const result = await handleSafetyTool('safety_emergency_stop_release', { confirmation: 'wrong' }, connection, safety);
    expect(result.isError).toBe(true);
    expect(safety.isEmergencyStopActive).toBe(true);
  });

  it('safety_emergency_stop_release works with correct confirmation', async () => {
    safety.activateEmergencyStop();
    const result = await handleSafetyTool('safety_emergency_stop_release', { confirmation: 'CONFIRM_RELEASE' }, connection, safety);
    expect(result.isError).toBeUndefined();
    expect(safety.isEmergencyStopActive).toBe(false);
  });

  it('safety_set_clamp_mode enables clamping', async () => {
    const result = await handleSafetyTool('safety_set_clamp_mode', { enabled: true }, connection, safety);
    expect(result.content[0].text).toContain('ENABLED');
    expect(safety.getPolicy().velocity.clampMode).toBe(true);
  });

  it('safety_heartbeat resets deadman timer', async () => {
    const result = await handleSafetyTool('safety_heartbeat', {}, connection, safety);
    expect(result.content[0].text).toContain('Heartbeat received');
  });

  it('safety_update_acceleration_limits updates config', async () => {
    const result = await handleSafetyTool('safety_update_acceleration_limits', {
      enabled: true,
      linearMaxAccel: 2.0,
    }, connection, safety);
    expect(result.content[0].text).toContain('Acceleration limits updated');
    expect(safety.getPolicy().acceleration.linearMaxAccel).toBe(2.0);
    expect(safety.getPolicy().acceleration.enabled).toBe(true);
  });

  it('safety_check_position returns geofence check', async () => {
    const result = await handleSafetyTool('safety_check_position', {
      x: 0, y: 0, z: 1,
    }, connection, safety);
    const data = JSON.parse(result.content[0].text);
    expect(data.inside).toBe(true);
    expect(data.violation).toBeNull();
  });

  it('safety_check_position detects proximity warning', async () => {
    const result = await handleSafetyTool('safety_check_position', {
      x: 4.5, y: 0, z: 1,
    }, connection, safety);
    const data = JSON.parse(result.content[0].text);
    expect(data.inside).toBe(true);
    expect(data.warning).not.toBeNull();
    expect(data.warning.type).toBe('geofence_warning');
  });

  it('safety_check_position detects violation', async () => {
    const result = await handleSafetyTool('safety_check_position', {
      x: 10, y: 0, z: 1,
    }, connection, safety);
    const data = JSON.parse(result.content[0].text);
    expect(data.inside).toBe(false);
    expect(data.violation).not.toBeNull();
  });

  it('unknown safety tool returns error', async () => {
    const result = await handleSafetyTool('nonexistent_tool', {}, connection, safety);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown safety tool');
  });
});

/**
 * Tests for ROS2 parameter tools: list, get, set.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getParamTools, handleParamTool } from './param-tools.js';

// --- Mocks ---

function createMockConnection(response: { status: string; data: unknown } = { status: 'ok', data: {} }) {
  return {
    send: vi.fn().mockResolvedValue(response),
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    disconnect: vi.fn(),
  } as any;
}

function createMockSafety(allowed = true, violations: any[] = []) {
  return {
    checkServiceCall: vi.fn().mockReturnValue({ allowed, violations }),
    checkPublish: vi.fn().mockReturnValue({ allowed: true, violations: [] }),
    checkActionGoal: vi.fn().mockReturnValue({ allowed: true, violations: [] }),
  } as any;
}

// --- Tool definition tests ---

describe('getParamTools', () => {
  it('should return exactly 3 tools', () => {
    const tools = getParamTools();
    expect(tools).toHaveLength(3);
  });

  it('should include ros2_param_list', () => {
    const tools = getParamTools();
    const tool = tools.find(t => t.name === 'ros2_param_list');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('List');
    expect(tool!.description).toContain('parameters');
  });

  it('should include ros2_param_get', () => {
    const tools = getParamTools();
    const tool = tools.find(t => t.name === 'ros2_param_get');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('Get');
    expect(tool!.description).toContain('parameter');
  });

  it('should include ros2_param_set', () => {
    const tools = getParamTools();
    const tool = tools.find(t => t.name === 'ros2_param_set');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('Set');
    expect(tool!.description).toContain('SAFETY CHECKED');
  });

  it('should have valid inputSchema for all tools', () => {
    const tools = getParamTools();
    for (const tool of tools) {
      expect(tool.inputSchema).toBeDefined();
      expect(tool.inputSchema.type).toBe('object');
    }
  });
});

// --- ros2_param_list handler tests ---

describe('handleParamTool - ros2_param_list', () => {
  it('should list parameters for a valid node', async () => {
    const params = ['use_sim_time', 'background_r', 'background_g', 'background_b'];
    const conn = createMockConnection({ status: 'ok', data: { parameters: params } });
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_list', { nodeName: '/turtlesim' }, conn, safety);

    expect(result.isError).toBeUndefined();
    expect(conn.send).toHaveBeenCalledWith('params.list', { node_name: '/turtlesim' });
    const parsed = JSON.parse(result.content[0].text);
    expect(parsed.parameters).toEqual(params);
  });

  it('should reject invalid node name (no leading slash)', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_list', { nodeName: 'turtlesim' }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid node name');
    expect(conn.send).not.toHaveBeenCalled();
  });

  it('should return error on bridge error response', async () => {
    const conn = createMockConnection({ status: 'error', data: 'Node not found' });
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_list', { nodeName: '/nonexistent' }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error:');
  });
});

// --- ros2_param_get handler tests ---

describe('handleParamTool - ros2_param_get', () => {
  it('should get a parameter value', async () => {
    const conn = createMockConnection({ status: 'ok', data: { value: 69 } });
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_get', {
      nodeName: '/turtlesim',
      paramName: 'background_r',
    }, conn, safety);

    expect(result.isError).toBeUndefined();
    expect(conn.send).toHaveBeenCalledWith('params.get', {
      node_name: '/turtlesim',
      param_name: 'background_r',
    });
    expect(result.content[0].text).toContain('/turtlesim/background_r');
    expect(result.content[0].text).toContain('69');
  });

  it('should reject invalid node name', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_get', {
      nodeName: 'bad_name',
      paramName: 'background_r',
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid node name');
    expect(conn.send).not.toHaveBeenCalled();
  });

  it('should reject invalid parameter name (special chars)', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_get', {
      nodeName: '/turtlesim',
      paramName: 'bad param!',
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid parameter name');
    expect(conn.send).not.toHaveBeenCalled();
  });

  it('should accept dotted parameter names (nested params)', async () => {
    const conn = createMockConnection({ status: 'ok', data: { value: 1.0 } });
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_get', {
      nodeName: '/controller',
      paramName: 'pid.kp',
    }, conn, safety);

    expect(result.isError).toBeUndefined();
    expect(conn.send).toHaveBeenCalledWith('params.get', {
      node_name: '/controller',
      param_name: 'pid.kp',
    });
  });

  it('should return error on bridge error response', async () => {
    const conn = createMockConnection({ status: 'error', data: 'Parameter not found' });
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_get', {
      nodeName: '/turtlesim',
      paramName: 'nonexistent',
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error:');
  });
});

// --- ros2_param_set handler tests ---

describe('handleParamTool - ros2_param_set', () => {
  it('should set a parameter value when safety allows', async () => {
    const conn = createMockConnection({ status: 'ok', data: {} });
    const safety = createMockSafety(true);

    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/turtlesim',
      paramName: 'background_r',
      value: 255,
    }, conn, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Set /turtlesim/background_r');
    expect(result.content[0].text).toContain('255');
    expect(conn.send).toHaveBeenCalledWith('params.set', {
      node_name: '/turtlesim',
      param_name: 'background_r',
      value: 255,
    });
  });

  it('should pass safety check through checkServiceCall', async () => {
    const conn = createMockConnection({ status: 'ok', data: {} });
    const safety = createMockSafety(true);

    await handleParamTool('ros2_param_set', {
      nodeName: '/turtlesim',
      paramName: 'background_r',
      value: 100,
    }, conn, safety);

    expect(safety.checkServiceCall).toHaveBeenCalledWith(
      '/turtlesim/set_parameters',
      { param_name: 'background_r', value: 100 }
    );
  });

  it('should block when safety denies the operation', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety(false, [
      {
        type: 'blocked_service',
        message: 'Service "/turtlesim/set_parameters" is blocked by safety policy',
        details: {},
        timestamp: Date.now(),
      },
    ]);

    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/turtlesim',
      paramName: 'background_r',
      value: 255,
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.content[0].text).toContain('blocked_service');
    expect(conn.send).not.toHaveBeenCalled();
  });

  it('should block when emergency stop is active', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety(false, [
      {
        type: 'emergency_stop_active',
        message: 'Emergency stop is active. Release e-stop before calling services.',
        details: {},
        timestamp: Date.now(),
      },
    ]);

    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/turtlesim',
      paramName: 'background_r',
      value: 255,
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('SAFETY BLOCKED');
    expect(result.content[0].text).toContain('emergency_stop_active');
    expect(conn.send).not.toHaveBeenCalled();
  });

  it('should reject invalid node name', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_set', {
      nodeName: 'no_slash',
      paramName: 'background_r',
      value: 100,
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid node name');
    expect(conn.send).not.toHaveBeenCalled();
    expect(safety.checkServiceCall).not.toHaveBeenCalled();
  });

  it('should reject invalid parameter name', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/turtlesim',
      paramName: '',
      value: 100,
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Invalid parameter name');
    expect(conn.send).not.toHaveBeenCalled();
  });

  it('should handle string values', async () => {
    const conn = createMockConnection({ status: 'ok', data: {} });
    const safety = createMockSafety(true);

    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/robot',
      paramName: 'frame_id',
      value: 'base_link',
    }, conn, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('"base_link"');
    expect(conn.send).toHaveBeenCalledWith('params.set', {
      node_name: '/robot',
      param_name: 'frame_id',
      value: 'base_link',
    });
  });

  it('should handle boolean values', async () => {
    const conn = createMockConnection({ status: 'ok', data: {} });
    const safety = createMockSafety(true);

    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/robot',
      paramName: 'use_sim_time',
      value: true,
    }, conn, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('true');
  });

  it('should return error on bridge error response', async () => {
    const conn = createMockConnection({ status: 'error', data: 'Parameter is read-only' });
    const safety = createMockSafety(true);

    const result = await handleParamTool('ros2_param_set', {
      nodeName: '/turtlesim',
      paramName: 'background_r',
      value: 255,
    }, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error:');
    expect(result.content[0].text).toContain('read-only');
  });
});

// --- Unknown tool name ---

describe('handleParamTool - unknown tool', () => {
  it('should return error for unknown tool name', async () => {
    const conn = createMockConnection();
    const safety = createMockSafety();

    const result = await handleParamTool('ros2_param_delete', {}, conn, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown param tool');
  });
});

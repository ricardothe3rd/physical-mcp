/**
 * Tests for path planning integration tools with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { getPathTools, handlePathTool } from './path-tools.js';
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

describe('getPathTools', () => {
  it('returns exactly four tools', () => {
    const tools = getPathTools();
    expect(tools).toHaveLength(4);
  });

  it('includes ros2_plan_path tool', () => {
    const tools = getPathTools();
    const tool = tools.find(t => t.name === 'ros2_plan_path');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('path plan');
  });

  it('includes ros2_path_info tool', () => {
    const tools = getPathTools();
    const tool = tools.find(t => t.name === 'ros2_path_info');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('/plan');
  });

  it('includes ros2_costmap_update tool', () => {
    const tools = getPathTools();
    const tool = tools.find(t => t.name === 'ros2_costmap_update');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('costmap');
  });

  it('includes ros2_navigation_status tool', () => {
    const tools = getPathTools();
    const tool = tools.find(t => t.name === 'ros2_navigation_status');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('navigation status');
  });

  it('all tools have valid inputSchema', () => {
    const tools = getPathTools();
    for (const tool of tools) {
      expect(tool.inputSchema).toBeDefined();
      expect(tool.inputSchema.type).toBe('object');
    }
  });
});

describe('handlePathTool - ros2_plan_path', () => {
  let connection: ReturnType<typeof createMockConnection>;
  let safety: PolicyEngine;

  beforeEach(() => {
    connection = createMockConnection();
    safety = new PolicyEngine();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('sends path planning goal on success', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok',
      data: { path: { poses: [{ pose: { position: { x: 0, y: 0 } } }] } },
      timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, goal_x: 2, goal_y: 3,
    }, connection, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Path planned');
    expect(result.content[0].text).toContain('(0, 0)');
    expect(result.content[0].text).toContain('(2, 3)');
    expect(connection.send).toHaveBeenCalledWith('action.send_goal', expect.objectContaining({
      action: '/compute_path_to_pose',
      action_type: 'nav2_msgs/action/ComputePathToPose',
    }));
  });

  it('uses default planner_id GridBased when not specified', async () => {
    await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, goal_x: 1, goal_y: 1,
    }, connection, safety);

    expect(connection.send).toHaveBeenCalledWith('action.send_goal', expect.objectContaining({
      goal: expect.objectContaining({ planner_id: 'GridBased' }),
    }));
  });

  it('uses custom planner_id when provided', async () => {
    await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, goal_x: 1, goal_y: 1, planner_id: 'SmacPlanner',
    }, connection, safety);

    expect(connection.send).toHaveBeenCalledWith('action.send_goal', expect.objectContaining({
      goal: expect.objectContaining({ planner_id: 'SmacPlanner' }),
    }));
  });

  it('blocks path planning when start is outside geofence', async () => {
    const result = await handlePathTool('ros2_plan_path', {
      start_x: 100, start_y: 100, goal_x: 1, goal_y: 1,
    }, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('start position');
    expect(result.content[0].text).toContain('outside geofence');
    expect(connection.send).not.toHaveBeenCalled();
  });

  it('blocks path planning when goal is outside geofence', async () => {
    const result = await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, goal_x: 100, goal_y: 100,
    }, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('goal position');
    expect(result.content[0].text).toContain('outside geofence');
    expect(connection.send).not.toHaveBeenCalled();
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Planner failed' }, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, goal_x: 1, goal_y: 1,
    }, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Path planning error');
    expect(result.content[0].text).toContain('Planner failed');
  });

  it('handles bridge connection failure', async () => {
    connection.send.mockRejectedValue(new Error('Bridge unavailable'));

    const result = await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, goal_x: 1, goal_y: 1,
    }, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Path planning failed');
    expect(result.content[0].text).toContain('Bridge unavailable');
  });

  it('sets use_start false when start_theta is not provided', async () => {
    await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, goal_x: 1, goal_y: 1,
    }, connection, safety);

    expect(connection.send).toHaveBeenCalledWith('action.send_goal', expect.objectContaining({
      goal: expect.objectContaining({ use_start: false }),
    }));
  });

  it('does not set use_start when start_theta is provided', async () => {
    await handlePathTool('ros2_plan_path', {
      start_x: 0, start_y: 0, start_theta: 1.57, goal_x: 1, goal_y: 1,
    }, connection, safety);

    const call = connection.send.mock.calls[0];
    expect(call[1].goal.use_start).toBeUndefined();
  });
});

describe('handlePathTool - ros2_path_info', () => {
  let connection: ReturnType<typeof createMockConnection>;
  let safety: PolicyEngine;

  beforeEach(() => {
    connection = createMockConnection();
    safety = new PolicyEngine();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('returns path info with poses, distance, and bounding box', async () => {
    const pathData = {
      poses: [
        { pose: { position: { x: 0, y: 0 } } },
        { pose: { position: { x: 1, y: 0 } } },
        { pose: { position: { x: 1, y: 1 } } },
      ],
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: pathData, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_path_info', {}, connection, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Path info');
    const text = result.content[0].text;
    expect(text).toContain('"num_poses": 3');
    expect(text).toContain('"total_distance_m": 2');
    expect(text).toContain('"min_x": 0');
    expect(text).toContain('"max_x": 1');
    expect(text).toContain('"min_y": 0');
    expect(text).toContain('"max_y": 1');
  });

  it('subscribes to /plan topic', async () => {
    await handlePathTool('ros2_path_info', {}, connection, safety);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/plan',
      count: 1,
    });
  });

  it('handles empty path (no poses)', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { poses: [] }, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_path_info', {}, connection, safety);

    expect(result.isError).toBeUndefined();
    const text = result.content[0].text;
    expect(text).toContain('"num_poses": 0');
    expect(text).toContain('"total_distance_m": 0');
    expect(text).toContain('"bounding_box": null');
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'No path available' }, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_path_info', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('No path available');
  });

  it('handles bridge connection failure', async () => {
    connection.send.mockRejectedValue(new Error('Connection lost'));

    const result = await handlePathTool('ros2_path_info', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Failed to get path info');
    expect(result.content[0].text).toContain('Connection lost');
  });
});

describe('handlePathTool - ros2_costmap_update', () => {
  let connection: ReturnType<typeof createMockConnection>;
  let safety: PolicyEngine;

  beforeEach(() => {
    connection = createMockConnection();
    safety = new PolicyEngine();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('clears costmap successfully with default name', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: {}, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_costmap_update', {}, connection, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('local_costmap');
    expect(result.content[0].text).toContain('cleared successfully');
    expect(connection.send).toHaveBeenCalledWith('service.call', expect.objectContaining({
      service: '/local_costmap/clear_entirely_local_costmap',
      service_type: 'nav2_msgs/srv/ClearEntireCostmap',
    }));
  });

  it('clears costmap with custom name', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: {}, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_costmap_update', {
      costmap_name: 'global_costmap',
    }, connection, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('global_costmap');
    expect(connection.send).toHaveBeenCalledWith('service.call', expect.objectContaining({
      service: '/global_costmap/clear_entirely_global_costmap',
    }));
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Service not found' }, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_costmap_update', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Costmap update error');
    expect(result.content[0].text).toContain('Service not found');
  });

  it('handles bridge connection failure', async () => {
    connection.send.mockRejectedValue(new Error('Timeout'));

    const result = await handlePathTool('ros2_costmap_update', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Costmap update failed');
    expect(result.content[0].text).toContain('Timeout');
  });

  it('blocks costmap update when e-stop is active', async () => {
    safety.activateEmergencyStop();

    const result = await handlePathTool('ros2_costmap_update', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('blocked');
    expect(connection.send).not.toHaveBeenCalled();

    safety.releaseEmergencyStop();
  });
});

describe('handlePathTool - ros2_navigation_status', () => {
  let connection: ReturnType<typeof createMockConnection>;
  let safety: PolicyEngine;

  beforeEach(() => {
    connection = createMockConnection();
    safety = new PolicyEngine();
  });

  afterEach(() => {
    safety.destroy();
  });

  it('returns navigation status with active goals', async () => {
    const statusData = {
      status_list: [
        { goal_info: { goal_id: 'goal_1' }, status: 2 },
        { goal_info: { goal_id: 'goal_2' }, status: 4 },
      ],
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: statusData, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_navigation_status', {}, connection, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Navigation status');
    expect(result.content[0].text).toContain('"active_goals": 2');
    expect(result.content[0].text).toContain('goal_1');
    expect(result.content[0].text).toContain('goal_2');
  });

  it('subscribes to correct status topic', async () => {
    await handlePathTool('ros2_navigation_status', {}, connection, safety);

    expect(connection.send).toHaveBeenCalledWith('topic.subscribe', {
      topic: '/navigate_to_pose/_action/status',
      count: 1,
    });
  });

  it('handles empty status list', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { status_list: [] }, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_navigation_status', {}, connection, safety);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('"active_goals": 0');
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Topic not found' }, timestamp: Date.now(),
    });

    const result = await handlePathTool('ros2_navigation_status', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('Topic not found');
  });

  it('handles bridge connection failure', async () => {
    connection.send.mockRejectedValue(new Error('WebSocket closed'));

    const result = await handlePathTool('ros2_navigation_status', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Failed to get navigation status');
    expect(result.content[0].text).toContain('WebSocket closed');
  });
});

describe('handlePathTool - unknown tool', () => {
  it('returns error for unknown tool name', async () => {
    const connection = createMockConnection();
    const safety = new PolicyEngine();

    const result = await handlePathTool('ros2_nonexistent_path_tool', {}, connection, safety);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown path tool');
    expect(result.content[0].text).toContain('ros2_nonexistent_path_tool');

    safety.destroy();
  });
});

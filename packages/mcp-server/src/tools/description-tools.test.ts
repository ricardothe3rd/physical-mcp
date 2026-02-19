/**
 * Tests for robot description tool handlers.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import {
  getDescriptionTools,
  handleDescriptionTool,
  parseJointsFromUrdf,
} from './description-tools.js';

// ---------------------------------------------------------------------------
// Mock connection factory
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Sample URDF snippets
// ---------------------------------------------------------------------------

const SAMPLE_URDF = `<?xml version="1.0" ?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="wheel_left"/>
  <link name="wheel_right"/>
  <link name="arm_link"/>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
  </joint>

  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <joint name="fixed_mount" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
  </joint>
</robot>`;

const MINIMAL_URDF = `<robot name="tiny"><link name="a"/></robot>`;

// ---------------------------------------------------------------------------
// Tool definition tests
// ---------------------------------------------------------------------------

describe('getDescriptionTools', () => {
  it('returns exactly 2 tools', () => {
    expect(getDescriptionTools()).toHaveLength(2);
  });

  it('returns ros2_robot_description tool', () => {
    const tools = getDescriptionTools();
    expect(tools.find(t => t.name === 'ros2_robot_description')).toBeDefined();
  });

  it('returns ros2_robot_joints tool', () => {
    const tools = getDescriptionTools();
    expect(tools.find(t => t.name === 'ros2_robot_joints')).toBeDefined();
  });

  it('each tool has name, description, and inputSchema', () => {
    for (const tool of getDescriptionTools()) {
      expect(tool.name).toBeTruthy();
      expect(tool.description).toBeTruthy();
      expect(tool.inputSchema).toBeDefined();
    }
  });
});

// ---------------------------------------------------------------------------
// URDF parser unit tests
// ---------------------------------------------------------------------------

describe('parseJointsFromUrdf', () => {
  it('extracts all joints from a well-formed URDF', () => {
    const joints = parseJointsFromUrdf(SAMPLE_URDF);
    expect(joints).toHaveLength(4);
  });

  it('extracts joint names correctly', () => {
    const joints = parseJointsFromUrdf(SAMPLE_URDF);
    const names = joints.map(j => j.name);
    expect(names).toContain('wheel_left_joint');
    expect(names).toContain('wheel_right_joint');
    expect(names).toContain('arm_joint');
    expect(names).toContain('fixed_mount');
  });

  it('extracts joint types correctly', () => {
    const joints = parseJointsFromUrdf(SAMPLE_URDF);
    const arm = joints.find(j => j.name === 'arm_joint');
    expect(arm?.type).toBe('revolute');

    const wheel = joints.find(j => j.name === 'wheel_left_joint');
    expect(wheel?.type).toBe('continuous');

    const fixed = joints.find(j => j.name === 'fixed_mount');
    expect(fixed?.type).toBe('fixed');
  });

  it('extracts parent and child links', () => {
    const joints = parseJointsFromUrdf(SAMPLE_URDF);
    const arm = joints.find(j => j.name === 'arm_joint');
    expect(arm?.parent).toBe('base_link');
    expect(arm?.child).toBe('arm_link');
  });

  it('extracts limit attributes when present', () => {
    const joints = parseJointsFromUrdf(SAMPLE_URDF);
    const arm = joints.find(j => j.name === 'arm_joint');
    expect(arm?.limits).toBeDefined();
    expect(arm?.limits?.lower).toBeCloseTo(-1.57);
    expect(arm?.limits?.upper).toBeCloseTo(1.57);
    expect(arm?.limits?.effort).toBe(10.0);
    expect(arm?.limits?.velocity).toBe(1.0);
  });

  it('omits limits when not present in joint', () => {
    const joints = parseJointsFromUrdf(SAMPLE_URDF);
    const wheel = joints.find(j => j.name === 'wheel_left_joint');
    expect(wheel?.limits).toBeUndefined();
  });

  it('returns empty array for URDF with no joints', () => {
    const joints = parseJointsFromUrdf(MINIMAL_URDF);
    expect(joints).toHaveLength(0);
  });

  it('returns empty array for empty string', () => {
    const joints = parseJointsFromUrdf('');
    expect(joints).toHaveLength(0);
  });

  it('handles partial limit attributes', () => {
    const urdf = `<robot name="r">
      <joint name="j1" type="prismatic">
        <parent link="a"/>
        <child link="b"/>
        <limit effort="5.0"/>
      </joint>
    </robot>`;
    const joints = parseJointsFromUrdf(urdf);
    expect(joints).toHaveLength(1);
    expect(joints[0].limits?.effort).toBe(5.0);
    expect(joints[0].limits?.lower).toBeUndefined();
    expect(joints[0].limits?.upper).toBeUndefined();
  });
});

// ---------------------------------------------------------------------------
// Handler tests: ros2_robot_description
// ---------------------------------------------------------------------------

describe('handleDescriptionTool: ros2_robot_description', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns URDF from topic echo', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { data: SAMPLE_URDF },
      timestamp: Date.now(),
    });

    const result = await handleDescriptionTool('ros2_robot_description', {}, connection);
    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('<robot name="test_robot">');
    expect(connection.send).toHaveBeenCalledWith('topic.echo', expect.objectContaining({
      topic: '/robot_description',
      message_type: 'std_msgs/msg/String',
    }));
  });

  it('uses custom topic when provided', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { data: SAMPLE_URDF },
      timestamp: Date.now(),
    });

    await handleDescriptionTool('ros2_robot_description', {
      topic: '/my_robot/description',
      timeoutSec: 10,
    }, connection);

    expect(connection.send).toHaveBeenCalledWith('topic.echo', expect.objectContaining({
      topic: '/my_robot/description',
      timeout_sec: 10,
    }));
  });

  it('falls back to parameter when topic echo fails', async () => {
    // First call (topic echo) fails, second call (param get) succeeds
    connection.send
      .mockRejectedValueOnce(new Error('topic not available'))
      .mockResolvedValueOnce({
        id: '2',
        status: 'ok',
        data: SAMPLE_URDF,
        timestamp: Date.now(),
      });

    const result = await handleDescriptionTool('ros2_robot_description', {}, connection);
    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('<robot name="test_robot">');
    // Second call should be params.get
    expect(connection.send).toHaveBeenCalledWith('params.get', {
      node_name: '/robot_state_publisher',
      param_name: 'robot_description',
    });
  });

  it('falls back to parameter when topic echo returns error status', async () => {
    connection.send
      .mockResolvedValueOnce({
        id: '1',
        status: 'error',
        data: { error: 'no publishers' },
        timestamp: Date.now(),
      })
      .mockResolvedValueOnce({
        id: '2',
        status: 'ok',
        data: SAMPLE_URDF,
        timestamp: Date.now(),
      });

    const result = await handleDescriptionTool('ros2_robot_description', {}, connection);
    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('<robot name="test_robot">');
  });

  it('returns error when both strategies fail', async () => {
    connection.send
      .mockRejectedValueOnce(new Error('topic not available'))
      .mockRejectedValueOnce(new Error('param not available'));

    const result = await handleDescriptionTool('ros2_robot_description', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Could not fetch robot description');
  });

  it('returns error when topic returns empty data and param fails', async () => {
    connection.send
      .mockResolvedValueOnce({
        id: '1',
        status: 'ok',
        data: { data: '' },
        timestamp: Date.now(),
      })
      .mockRejectedValueOnce(new Error('param not found'));

    const result = await handleDescriptionTool('ros2_robot_description', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Could not fetch robot description');
  });
});

// ---------------------------------------------------------------------------
// Handler tests: ros2_robot_joints
// ---------------------------------------------------------------------------

describe('handleDescriptionTool: ros2_robot_joints', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns parsed joints from URDF', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { data: SAMPLE_URDF },
      timestamp: Date.now(),
    });

    const result = await handleDescriptionTool('ros2_robot_joints', {}, connection);
    expect(result.isError).toBeUndefined();

    const joints = JSON.parse(result.content[0].text);
    expect(joints).toHaveLength(4);
    expect(joints[0].name).toBeDefined();
    expect(joints[0].type).toBeDefined();
    expect(joints[0].parent).toBeDefined();
    expect(joints[0].child).toBeDefined();
  });

  it('includes joint limits in output', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { data: SAMPLE_URDF },
      timestamp: Date.now(),
    });

    const result = await handleDescriptionTool('ros2_robot_joints', {}, connection);
    const joints = JSON.parse(result.content[0].text);
    const arm = joints.find((j: any) => j.name === 'arm_joint');
    expect(arm.limits).toBeDefined();
    expect(arm.limits.lower).toBeCloseTo(-1.57);
  });

  it('returns message when URDF has no joints', async () => {
    connection.send.mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { data: MINIMAL_URDF },
      timestamp: Date.now(),
    });

    const result = await handleDescriptionTool('ros2_robot_joints', {}, connection);
    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('No joints found');
  });

  it('returns error when URDF cannot be fetched', async () => {
    connection.send
      .mockRejectedValueOnce(new Error('topic not available'))
      .mockRejectedValueOnce(new Error('param not available'));

    const result = await handleDescriptionTool('ros2_robot_joints', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Could not fetch robot description');
  });
});

// ---------------------------------------------------------------------------
// Unknown tool
// ---------------------------------------------------------------------------

describe('handleDescriptionTool: unknown tool', () => {
  it('returns error for unknown description tool', async () => {
    const connection = createMockConnection();
    const result = await handleDescriptionTool('ros2_robot_unknown', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown description tool');
  });
});

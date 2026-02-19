import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getPowerTools, handlePowerTool } from './power-tools.js';
import type { ConnectionManager } from '../bridge/connection-manager.js';

// Helper to create a mock ConnectionManager
function mockConnection(response: { status: string; data: unknown }): ConnectionManager {
  return {
    send: vi.fn().mockResolvedValue(response),
  } as unknown as ConnectionManager;
}

describe('getPowerTools', () => {
  it('returns exactly two tools', () => {
    const tools = getPowerTools();
    expect(tools).toHaveLength(2);
  });

  it('includes ros2_battery_status tool', () => {
    const tools = getPowerTools();
    const battery = tools.find(t => t.name === 'ros2_battery_status');
    expect(battery).toBeDefined();
    expect(battery!.description).toContain('battery');
  });

  it('includes ros2_power_supply_status tool', () => {
    const tools = getPowerTools();
    const supply = tools.find(t => t.name === 'ros2_power_supply_status');
    expect(supply).toBeDefined();
    expect(supply!.description).toContain('power supply');
  });

  it('all tools have valid inputSchema', () => {
    const tools = getPowerTools();
    for (const tool of tools) {
      expect(tool.inputSchema).toBeDefined();
      expect(tool.inputSchema.type).toBe('object');
    }
  });
});

describe('handlePowerTool - ros2_battery_status', () => {
  it('returns battery data on success', async () => {
    const data = {
      percentage: 0.85,
      voltage: 12.6,
      current: -1.2,
      temperature: 35.0,
      power_supply_status: 2,
    };
    const conn = mockConnection({ status: 'ok', data });

    const result = await handlePowerTool('ros2_battery_status', {}, conn);

    expect(result.isError).toBeUndefined();
    expect(result.content).toHaveLength(1);
    expect(result.content[0].type).toBe('text');
    expect(result.content[0].text).toContain('Battery status');
    expect(result.content[0].text).toContain('0.85');
    expect(result.content[0].text).toContain('12.6');
  });

  it('uses default topic /battery_state when none provided', async () => {
    const conn = mockConnection({ status: 'ok', data: {} });

    await handlePowerTool('ros2_battery_status', {}, conn);

    expect(conn.send).toHaveBeenCalledWith('power.battery_status', {
      topic: '/battery_state',
      timeout_ms: 5000,
    });
  });

  it('uses custom topic when provided', async () => {
    const conn = mockConnection({ status: 'ok', data: {} });

    await handlePowerTool('ros2_battery_status', { topic: '/robot1/battery' }, conn);

    expect(conn.send).toHaveBeenCalledWith('power.battery_status', {
      topic: '/robot1/battery',
      timeout_ms: 5000,
    });
  });

  it('uses custom timeout when provided', async () => {
    const conn = mockConnection({ status: 'ok', data: {} });

    await handlePowerTool('ros2_battery_status', { timeout_ms: 10000 }, conn);

    expect(conn.send).toHaveBeenCalledWith('power.battery_status', {
      topic: '/battery_state',
      timeout_ms: 10000,
    });
  });

  it('returns error when bridge responds with error', async () => {
    const conn = mockConnection({ status: 'error', data: { message: 'Topic not found' } });

    const result = await handlePowerTool('ros2_battery_status', {}, conn);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('Topic not found');
  });
});

describe('handlePowerTool - ros2_power_supply_status', () => {
  it('returns power supply data on success', async () => {
    const data = {
      power_supply_status: 1,
      power_supply_health: 1,
      power_supply_technology: 2,
      cell_voltage: [3.7, 3.7, 3.7],
      cell_temperature: [30.0, 31.0, 29.5],
      present: true,
    };
    const conn = mockConnection({ status: 'ok', data });

    const result = await handlePowerTool('ros2_power_supply_status', {}, conn);

    expect(result.isError).toBeUndefined();
    expect(result.content).toHaveLength(1);
    expect(result.content[0].text).toContain('Power supply info');
    expect(result.content[0].text).toContain('cell_voltage');
  });

  it('uses default topic /battery_state when none provided', async () => {
    const conn = mockConnection({ status: 'ok', data: {} });

    await handlePowerTool('ros2_power_supply_status', {}, conn);

    expect(conn.send).toHaveBeenCalledWith('power.supply_status', {
      topic: '/battery_state',
      timeout_ms: 5000,
    });
  });

  it('uses custom topic when provided', async () => {
    const conn = mockConnection({ status: 'ok', data: {} });

    await handlePowerTool('ros2_power_supply_status', { topic: '/ups/status' }, conn);

    expect(conn.send).toHaveBeenCalledWith('power.supply_status', {
      topic: '/ups/status',
      timeout_ms: 5000,
    });
  });

  it('returns error when bridge responds with error', async () => {
    const conn = mockConnection({ status: 'error', data: { message: 'Timeout waiting for message' } });

    const result = await handlePowerTool('ros2_power_supply_status', {}, conn);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
    expect(result.content[0].text).toContain('Timeout');
  });
});

describe('handlePowerTool - unknown tool', () => {
  it('returns error for unknown tool name', async () => {
    const conn = mockConnection({ status: 'ok', data: {} });

    const result = await handlePowerTool('ros2_nonexistent_power_tool', {}, conn);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown power tool');
    expect(result.content[0].text).toContain('ros2_nonexistent_power_tool');
  });
});

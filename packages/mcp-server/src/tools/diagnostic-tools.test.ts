/**
 * Tests for diagnostic tool handlers with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getDiagnosticTools, handleDiagnosticTool } from './diagnostic-tools.js';

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

describe('Diagnostic Tool Handlers', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  describe('getDiagnosticTools', () => {
    it('returns 2 tools', () => {
      const tools = getDiagnosticTools();
      expect(tools).toHaveLength(2);
      expect(tools.map(t => t.name)).toContain('ros2_diagnostics_summary');
      expect(tools.map(t => t.name)).toContain('ros2_diagnostics_detail');
    });
  });

  describe('ros2_diagnostics_summary', () => {
    it('returns diagnostics summary from bridge', async () => {
      const summaryData = {
        status: 'OK',
        devices: [
          { name: 'motor_controller', level: 0, message: 'Running' },
          { name: 'imu_sensor', level: 0, message: 'OK' },
        ],
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: summaryData, timestamp: Date.now(),
      });

      const result = await handleDiagnosticTool('ros2_diagnostics_summary', {}, connection);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('motor_controller');
      expect(result.content[0].text).toContain('imu_sensor');
      expect(connection.send).toHaveBeenCalledWith('diagnostics.summary');
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'diagnostics unavailable' }, timestamp: Date.now(),
      });

      const result = await handleDiagnosticTool('ros2_diagnostics_summary', {}, connection);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  describe('ros2_diagnostics_detail', () => {
    it('returns detail for a specific device', async () => {
      const detailData = {
        name: 'motor_controller',
        hardware_id: 'mc_001',
        level: 0,
        message: 'Running',
        values: [
          { key: 'temperature', value: '42.5' },
          { key: 'voltage', value: '12.1' },
        ],
      };
      connection.send.mockResolvedValue({
        id: '1', status: 'ok', data: detailData, timestamp: Date.now(),
      });

      const result = await handleDiagnosticTool('ros2_diagnostics_detail', {
        name: 'motor_controller',
      }, connection);

      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('motor_controller');
      expect(result.content[0].text).toContain('temperature');
      expect(connection.send).toHaveBeenCalledWith('diagnostics.detail', {
        name: 'motor_controller',
      });
    });

    it('returns error when name is missing', async () => {
      const result = await handleDiagnosticTool('ros2_diagnostics_detail', {}, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('name');
      expect(connection.send).not.toHaveBeenCalled();
    });

    it('returns error on bridge failure', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'error', data: { error: 'device not found' }, timestamp: Date.now(),
      });

      const result = await handleDiagnosticTool('ros2_diagnostics_detail', {
        name: 'nonexistent_device',
      }, connection);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });
  });

  it('returns error for unknown diagnostic tool', async () => {
    const result = await handleDiagnosticTool('unknown_tool', {}, connection);
    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown diagnostic tool');
  });
});

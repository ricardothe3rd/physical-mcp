/**
 * Tests for network quality monitoring tools with mock bridge connection.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getNetworkTools, handleNetworkTool } from './network-tools.js';

function createMockConnection() {
  return {
    isConnected: true,
    isBridgeAvailable: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() }),
    disconnect: vi.fn(),
  } as any;
}

describe('getNetworkTools', () => {
  it('returns exactly 3 tools', () => {
    const tools = getNetworkTools();
    expect(tools).toHaveLength(3);
  });

  it('each tool has name, description, and inputSchema', () => {
    const tools = getNetworkTools();
    for (const tool of tools) {
      expect(tool.name).toBeDefined();
      expect(tool.description).toBeTruthy();
      expect(tool.inputSchema).toBeDefined();
    }
  });

  it('all inputSchema.type === "object"', () => {
    const tools = getNetworkTools();
    for (const tool of tools) {
      expect(tool.inputSchema.type).toBe('object');
    }
  });

  it('includes ros2_network_stats tool', () => {
    const tools = getNetworkTools();
    const tool = tools.find(t => t.name === 'ros2_network_stats');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('network statistics');
  });

  it('includes ros2_network_bandwidth tool', () => {
    const tools = getNetworkTools();
    const tool = tools.find(t => t.name === 'ros2_network_bandwidth');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('bandwidth');
  });

  it('includes ros2_network_latency_test tool', () => {
    const tools = getNetworkTools();
    const tool = tools.find(t => t.name === 'ros2_network_latency_test');
    expect(tool).toBeDefined();
    expect(tool!.description).toContain('latency test');
  });
});

describe('handleNetworkTool - ros2_network_stats', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns network stats on success', async () => {
    const statsData = {
      latency_ms: 12,
      avg_latency_ms: 15,
      packet_loss_pct: 0.1,
      bytes_sent: 102400,
      bytes_received: 204800,
      uptime_s: 3600,
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: statsData, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_stats', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Network statistics');
    expect(result.content[0].text).toContain('"latency_ms": 12');
    expect(result.content[0].text).toContain('"avg_latency_ms": 15');
    expect(result.content[0].text).toContain('"packet_loss_pct": 0.1');
    expect(result.content[0].text).toContain('"bytes_sent": 102400');
    expect(result.content[0].text).toContain('"bytes_received": 204800');
    expect(result.content[0].text).toContain('"uptime_s": 3600');
    expect(result.content[0].text).toContain('"connection_quality": "excellent"');
  });

  it('sends system.network_stats to bridge', async () => {
    await handleNetworkTool('ros2_network_stats', {}, connection);

    expect(connection.send).toHaveBeenCalledWith('system.network_stats');
  });

  it('returns "excellent" for latency < 20ms', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { latency_ms: 10 }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_stats', {}, connection);
    expect(result.content[0].text).toContain('"connection_quality": "excellent"');
  });

  it('returns "good" for latency 20-49ms', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { latency_ms: 35 }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_stats', {}, connection);
    expect(result.content[0].text).toContain('"connection_quality": "good"');
  });

  it('returns "fair" for latency 50-99ms', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { latency_ms: 75 }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_stats', {}, connection);
    expect(result.content[0].text).toContain('"connection_quality": "fair"');
  });

  it('returns "poor" for latency >= 100ms', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { latency_ms: 150 }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_stats', {}, connection);
    expect(result.content[0].text).toContain('"connection_quality": "poor"');
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Stats unavailable' }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_stats', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Network stats error');
    expect(result.content[0].text).toContain('Stats unavailable');
  });

  it('handles bridge connection failure', async () => {
    connection.send.mockRejectedValue(new Error('Bridge unavailable'));

    const result = await handleNetworkTool('ros2_network_stats', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Network stats failed');
    expect(result.content[0].text).toContain('Bridge unavailable');
  });
});

describe('handleNetworkTool - ros2_network_bandwidth', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('returns bandwidth data for all topics', async () => {
    const bandwidthData = {
      topics: [
        { topic: '/cmd_vel', msg_count: 100, bytes_per_sec: 2400, avg_msg_size: 24 },
        { topic: '/scan', msg_count: 50, bytes_per_sec: 50000, avg_msg_size: 1000 },
      ],
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: bandwidthData, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_bandwidth', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Bandwidth usage (all topics)');
    expect(result.content[0].text).toContain('/cmd_vel');
    expect(result.content[0].text).toContain('/scan');
  });

  it('filters by topic when specified', async () => {
    const bandwidthData = {
      topics: [
        { topic: '/cmd_vel', msg_count: 100, bytes_per_sec: 2400, avg_msg_size: 24 },
      ],
    };
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: bandwidthData, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_bandwidth', { topic: '/cmd_vel' }, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Bandwidth usage for "/cmd_vel"');
  });

  it('sends system.bandwidth to bridge', async () => {
    await handleNetworkTool('ros2_network_bandwidth', {}, connection);

    expect(connection.send).toHaveBeenCalledWith('system.bandwidth', {});
  });

  it('sends system.bandwidth with topic filter to bridge', async () => {
    await handleNetworkTool('ros2_network_bandwidth', { topic: '/scan' }, connection);

    expect(connection.send).toHaveBeenCalledWith('system.bandwidth', { topic: '/scan' });
  });

  it('returns error when bridge responds with error', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'Bandwidth data unavailable' }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_bandwidth', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Bandwidth query error');
    expect(result.content[0].text).toContain('Bandwidth data unavailable');
  });

  it('handles bridge connection failure', async () => {
    connection.send.mockRejectedValue(new Error('Connection refused'));

    const result = await handleNetworkTool('ros2_network_bandwidth', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Bandwidth query failed');
    expect(result.content[0].text).toContain('Connection refused');
  });
});

describe('handleNetworkTool - ros2_network_latency_test', () => {
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    connection = createMockConnection();
  });

  it('runs default 5 pings and calculates stats', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { pong: true }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_latency_test', {}, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('Latency test results');
    expect(result.content[0].text).toContain('"pings_sent": 5');
    expect(result.content[0].text).toContain('"pings_successful": 5');
    expect(result.content[0].text).toContain('"pings_failed": 0');
    expect(result.content[0].text).toContain('min_ms');
    expect(result.content[0].text).toContain('max_ms');
    expect(result.content[0].text).toContain('avg_ms');
    expect(result.content[0].text).toContain('stddev_ms');
    expect(connection.send).toHaveBeenCalledTimes(5);
  });

  it('runs custom count of pings', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { pong: true }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_latency_test', { count: 10 }, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('"pings_sent": 10');
    expect(connection.send).toHaveBeenCalledTimes(10);
  });

  it('caps count at 20', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { pong: true }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_latency_test', { count: 50 }, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('"pings_sent": 20');
    expect(connection.send).toHaveBeenCalledTimes(20);
  });

  it('calculates min/max/avg/stddev correctly', async () => {
    // Mock Date.now() to control latency measurements
    let callCount = 0;
    // Simulate latencies: 10, 20, 30, 40, 50 ms
    const times = [
      1000, 1010,  // ping 1: 10ms
      1010, 1030,  // ping 2: 20ms
      1030, 1060,  // ping 3: 30ms
      1060, 1100,  // ping 4: 40ms
      1100, 1150,  // ping 5: 50ms
    ];
    const originalDateNow = Date.now;
    Date.now = vi.fn(() => times[callCount++]);

    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { pong: true }, timestamp: 0,
    });

    const result = await handleNetworkTool('ros2_network_latency_test', { count: 5 }, connection);

    Date.now = originalDateNow;

    expect(result.isError).toBeUndefined();
    const text = result.content[0].text;
    // avg = (10+20+30+40+50)/5 = 30
    expect(text).toContain('"min_ms": 10');
    expect(text).toContain('"max_ms": 50');
    expect(text).toContain('"avg_ms": 30');
    // variance = ((10-30)^2+(20-30)^2+(30-30)^2+(40-30)^2+(50-30)^2)/5 = (400+100+0+100+400)/5 = 200
    // stddev = sqrt(200) = 14.142... => 14.14
    expect(text).toContain('"stddev_ms": 14.14');
  });

  it('sends system.ping to bridge', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'ok', data: { pong: true }, timestamp: Date.now(),
    });

    await handleNetworkTool('ros2_network_latency_test', { count: 3 }, connection);

    expect(connection.send).toHaveBeenCalledWith('system.ping');
    expect(connection.send).toHaveBeenCalledTimes(3);
  });

  it('returns error when all pings fail', async () => {
    connection.send.mockRejectedValue(new Error('Connection lost'));

    const result = await handleNetworkTool('ros2_network_latency_test', { count: 3 }, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('all 3 pings failed');
  });

  it('handles partial ping failure (some pings succeed, some fail)', async () => {
    let callNum = 0;
    connection.send.mockImplementation(() => {
      callNum++;
      if (callNum === 2 || callNum === 4) {
        return Promise.reject(new Error('Timeout'));
      }
      return Promise.resolve({ id: '1', status: 'ok', data: { pong: true }, timestamp: Date.now() });
    });

    const result = await handleNetworkTool('ros2_network_latency_test', { count: 5 }, connection);

    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('"pings_sent": 5');
    expect(result.content[0].text).toContain('"pings_successful": 3');
    expect(result.content[0].text).toContain('"pings_failed": 2');
  });

  it('handles bridge error response in pings', async () => {
    connection.send.mockResolvedValue({
      id: '1', status: 'error', data: { message: 'ping failed' }, timestamp: Date.now(),
    });

    const result = await handleNetworkTool('ros2_network_latency_test', { count: 3 }, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('all 3 pings failed');
  });
});

describe('handleNetworkTool - unknown tool', () => {
  it('returns error for unknown tool name', async () => {
    const connection = createMockConnection();

    const result = await handleNetworkTool('ros2_nonexistent_network_tool', {}, connection);

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Unknown network tool');
    expect(result.content[0].text).toContain('ros2_nonexistent_network_tool');
  });
});

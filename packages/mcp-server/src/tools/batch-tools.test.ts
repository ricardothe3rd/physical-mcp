/**
 * Tests for batch command execution tool.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getBatchTools, handleBatchTool } from './batch-tools.js';
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

describe('Batch Tools', () => {
  describe('getBatchTools', () => {
    it('returns batch tool definitions', () => {
      const tools = getBatchTools();
      expect(tools).toHaveLength(1);
      expect(tools[0].name).toBe('ros2_batch_execute');
      expect(tools[0].inputSchema.type).toBe('object');
    });
  });

  describe('handleBatchTool', () => {
    let safety: PolicyEngine;
    let connection: ReturnType<typeof createMockConnection>;

    beforeEach(() => {
      safety = new PolicyEngine();
      connection = createMockConnection();
    });

    it('returns error for unknown batch tool', async () => {
      const result = await handleBatchTool('unknown', {}, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Unknown batch tool');
    });

    it('returns error for empty commands', async () => {
      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [],
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('No commands');
    });

    it('returns error for missing commands', async () => {
      const result = await handleBatchTool('ros2_batch_execute', {}, connection, safety);
      expect(result.isError).toBe(true);
    });

    it('returns error for too many commands', async () => {
      const commands = Array(21).fill({ tool: 'ros2_topic_list', args: {} });
      const result = await handleBatchTool('ros2_batch_execute', {
        commands,
      }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Maximum 20');
    });

    it('executes multiple topic commands', async () => {
      const topicList = [{ name: '/cmd_vel', type: 'Twist' }];
      connection.send
        .mockResolvedValueOnce({ id: '1', status: 'ok', data: topicList, timestamp: Date.now() })
        .mockResolvedValueOnce({ id: '2', status: 'ok', data: topicList, timestamp: Date.now() });

      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_list', args: {} },
          { tool: 'ros2_topic_list', args: {} },
        ],
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.summary).toContain('2 succeeded');
      expect(output.results).toHaveLength(2);
      expect(output.results[0].success).toBe(true);
      expect(output.results[1].success).toBe(true);
      expect(result.isError).toBeUndefined();
    });

    it('handles mixed tool categories', async () => {
      connection.send.mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() });

      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_list', args: {} },
          { tool: 'ros2_service_list', args: {} },
          { tool: 'ros2_action_list', args: {} },
        ],
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.summary).toContain('3 succeeded');
    });

    it('continues on error when stopOnError is false', async () => {
      connection.send
        .mockResolvedValueOnce({ id: '1', status: 'error', data: { error: 'fail' }, timestamp: Date.now() })
        .mockResolvedValueOnce({ id: '2', status: 'ok', data: {}, timestamp: Date.now() });

      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_list', args: {} },
          { tool: 'ros2_topic_list', args: {} },
        ],
        stopOnError: false,
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.summary).toContain('1 succeeded');
      expect(output.summary).toContain('1 failed');
      expect(output.results).toHaveLength(2);
    });

    it('stops on first error when stopOnError is true', async () => {
      connection.send
        .mockResolvedValueOnce({ id: '1', status: 'error', data: { error: 'fail' }, timestamp: Date.now() })
        .mockResolvedValueOnce({ id: '2', status: 'ok', data: {}, timestamp: Date.now() });

      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_list', args: {} },
          { tool: 'ros2_topic_list', args: {} },
        ],
        stopOnError: true,
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.results).toHaveLength(1);
      expect(output.summary).toContain('1/2 executed');
    });

    it('handles safety blocks in batch', async () => {
      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_publish', args: {
            topic: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist',
            message: { linear: { x: 10.0 } },
          }},
        ],
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.results[0].success).toBe(false);
      expect(output.results[0].result).toContain('SAFETY BLOCKED');
    });

    it('handles unknown tool names in batch', async () => {
      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'nonexistent_tool', args: {} },
        ],
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.results[0].success).toBe(false);
      expect(output.results[0].result).toContain('Unknown tool');
    });

    it('handles safety tools in batch', async () => {
      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'safety_status', args: {} },
          { tool: 'safety_heartbeat', args: {} },
        ],
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.summary).toContain('2 succeeded');
    });

    it('handles system tools in batch', async () => {
      connection.send.mockResolvedValue({
        id: '1', status: 'ok',
        data: ['/turtlebot3'],
        timestamp: Date.now(),
      });

      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'system_node_list', args: {} },
        ],
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.results[0].success).toBe(true);
    });

    it('handles exception during command execution', async () => {
      connection.send.mockRejectedValueOnce(new Error('connection lost'));

      const result = await handleBatchTool('ros2_batch_execute', {
        commands: [
          { tool: 'ros2_topic_list', args: {} },
        ],
      }, connection, safety);

      const output = JSON.parse(result.content[0].text);
      expect(output.results[0].success).toBe(false);
      expect(output.results[0].result).toContain('connection lost');
    });
  });
});

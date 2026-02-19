/**
 * Tests for scheduled command execution tools.
 */

import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { getScheduledTools, handleScheduledTool, jobs } from './scheduled-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

function createMockConnection() {
  return {
    isConnected: true,
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

describe('Scheduled Tools', () => {
  let safety: PolicyEngine;
  let connection: ReturnType<typeof createMockConnection>;

  beforeEach(() => {
    vi.useFakeTimers();
    safety = new PolicyEngine();
    connection = createMockConnection();
    // Clear all jobs between tests
    jobs.clear();
  });

  afterEach(() => {
    // Cancel all jobs to clear timers
    for (const job of jobs.values()) {
      if (job.timer) {
        clearTimeout(job.timer as any);
        clearInterval(job.timer as any);
      }
    }
    jobs.clear();
    vi.useRealTimers();
  });

  describe('getScheduledTools', () => {
    it('returns 3 tools', () => {
      const tools = getScheduledTools();
      expect(tools).toHaveLength(3);
      expect(tools.map(t => t.name)).toContain('ros2_schedule_command');
      expect(tools.map(t => t.name)).toContain('ros2_schedule_cancel');
      expect(tools.map(t => t.name)).toContain('ros2_schedule_list');
    });
  });

  describe('ros2_schedule_command', () => {
    it('schedules a one-shot command', async () => {
      const result = await handleScheduledTool('ros2_schedule_command', {
        tool: 'safety_status',
        args: {},
        delayMs: 5000,
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.jobId).toMatch(/^sched_/);
      expect(data.tool).toBe('safety_status');
      expect(data.delayMs).toBe(5000);
      expect(data.repeat).toBeFalsy();
    });

    it('schedules a repeating command', async () => {
      const result = await handleScheduledTool('ros2_schedule_command', {
        tool: 'safety_heartbeat',
        args: {},
        delayMs: 1000,
        repeat: true,
        intervalMs: 5000,
        maxRepeats: 10,
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.repeat).toBe(true);
      expect(data.intervalMs).toBe(5000);
      expect(data.maxRepeats).toBe(10);
    });

    it('rejects repeat without intervalMs', async () => {
      const result = await handleScheduledTool('ros2_schedule_command', {
        tool: 'safety_status',
        args: {},
        delayMs: 1000,
        repeat: true,
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('intervalMs is required');
    });

    it('adds job to the jobs map', async () => {
      await handleScheduledTool('ros2_schedule_command', {
        tool: 'safety_status',
        args: {},
        delayMs: 1000,
      }, connection, safety);

      expect(jobs.size).toBe(1);
      const job = Array.from(jobs.values())[0];
      expect(job.status).toBe('pending');
      expect(job.executionCount).toBe(0);
    });
  });

  describe('ros2_schedule_cancel', () => {
    it('cancels a pending job', async () => {
      const schedResult = await handleScheduledTool('ros2_schedule_command', {
        tool: 'safety_status',
        args: {},
        delayMs: 10000,
      }, connection, safety);
      const { jobId } = JSON.parse(schedResult.content[0].text);

      const cancelResult = await handleScheduledTool('ros2_schedule_cancel', {
        jobId,
      }, connection, safety);

      expect(cancelResult.content[0].text).toContain('cancelled');
      expect(jobs.get(jobId)!.status).toBe('cancelled');
    });

    it('returns error for nonexistent job', async () => {
      const result = await handleScheduledTool('ros2_schedule_cancel', {
        jobId: 'sched_999',
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('not found');
    });
  });

  describe('ros2_schedule_list', () => {
    it('lists empty when no jobs', async () => {
      const result = await handleScheduledTool('ros2_schedule_list', {}, connection, safety);
      expect(result.content[0].text).toContain('Scheduled jobs (0)');
    });

    it('lists scheduled jobs', async () => {
      await handleScheduledTool('ros2_schedule_command', {
        tool: 'safety_status',
        args: {},
        delayMs: 5000,
      }, connection, safety);

      await handleScheduledTool('ros2_schedule_command', {
        tool: 'safety_heartbeat',
        args: {},
        delayMs: 10000,
      }, connection, safety);

      const result = await handleScheduledTool('ros2_schedule_list', {}, connection, safety);
      expect(result.content[0].text).toContain('Scheduled jobs (2)');
    });
  });

  describe('unknown tool', () => {
    it('returns error for unknown scheduled tool', async () => {
      const result = await handleScheduledTool('nonexistent', {}, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Unknown scheduled tool');
    });
  });
});

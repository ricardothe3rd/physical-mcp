/**
 * Scheduled command execution: run commands after a delay or at intervals.
 *
 * Enables behaviors like:
 * - "Publish stop command in 5 seconds"
 * - "Send heartbeat every 10 seconds"
 * - "Schedule a status check after 30 seconds"
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

interface ScheduledJob {
  id: string;
  tool: string;
  args: Record<string, unknown>;
  delayMs: number;
  repeat: boolean;
  intervalMs?: number;
  maxRepeats?: number;
  createdAt: number;
  nextRunAt: number;
  executionCount: number;
  status: 'pending' | 'running' | 'completed' | 'cancelled';
  lastResult?: string;
  timer?: ReturnType<typeof setTimeout> | ReturnType<typeof setInterval>;
}

let nextJobId = 1;
const jobs = new Map<string, ScheduledJob>();

function generateJobId(): string {
  return `sched_${nextJobId++}`;
}

export function getScheduledTools(): Tool[] {
  return [
    {
      name: 'ros2_schedule_command',
      description: 'Schedule a command to execute after a delay. Optionally repeat at an interval.',
      inputSchema: toInputSchema(z.object({
        tool: z.string().describe('Tool name to execute (e.g., ros2_topic_publish)'),
        args: z.record(z.unknown()).default({}).describe('Arguments for the tool'),
        delayMs: z.number().min(100).max(300000).describe('Delay before first execution in ms (100ms - 5min)'),
        repeat: z.boolean().default(false).describe('Repeat the command at intervalMs'),
        intervalMs: z.number().min(100).max(60000).optional().describe('Repeat interval in ms (only if repeat=true)'),
        maxRepeats: z.number().min(1).max(100).optional().describe('Max number of repeats (default: unlimited up to 100)'),
      })),
    },
    {
      name: 'ros2_schedule_cancel',
      description: 'Cancel a scheduled command by its job ID',
      inputSchema: toInputSchema(z.object({
        jobId: z.string().describe('The job ID to cancel'),
      })),
    },
    {
      name: 'ros2_schedule_list',
      description: 'List all scheduled commands and their status',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handleScheduledTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine,
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_schedule_command': {
      const tool = args.tool as string;
      const toolArgs = (args.args as Record<string, unknown>) || {};
      const delayMs = args.delayMs as number;
      const repeat = (args.repeat as boolean) || false;
      const intervalMs = args.intervalMs as number | undefined;
      const maxRepeats = (args.maxRepeats as number) || 100;

      if (repeat && !intervalMs) {
        return {
          content: [{ type: 'text', text: 'intervalMs is required when repeat=true' }],
          isError: true,
        };
      }

      if (jobs.size >= 50) {
        return {
          content: [{ type: 'text', text: 'Maximum 50 scheduled jobs. Cancel some before adding more.' }],
          isError: true,
        };
      }

      const jobId = generateJobId();
      const now = Date.now();
      const job: ScheduledJob = {
        id: jobId,
        tool,
        args: toolArgs,
        delayMs,
        repeat,
        intervalMs,
        maxRepeats,
        createdAt: now,
        nextRunAt: now + delayMs,
        executionCount: 0,
        status: 'pending',
      };

      const executeJob = async () => {
        job.status = 'running';
        try {
          // Dynamic import to avoid circular deps
          const { handleTopicTool } = await import('./topic-tools.js');
          const { handleServiceTool } = await import('./service-tools.js');
          const { handleSafetyTool } = await import('./safety-tools.js');

          let result: { content: { type: string; text: string }[]; isError?: boolean };
          if (tool.startsWith('ros2_topic_')) {
            result = await handleTopicTool(tool, toolArgs, connection, safety);
          } else if (tool.startsWith('ros2_service_')) {
            result = await handleServiceTool(tool, toolArgs, connection, safety);
          } else if (tool.startsWith('safety_')) {
            result = await handleSafetyTool(tool, toolArgs, connection, safety);
          } else {
            result = { content: [{ type: 'text', text: `Unsupported tool: ${tool}` }], isError: true };
          }

          job.lastResult = result.content[0]?.text?.slice(0, 200) || '';
          job.executionCount++;
        } catch (err) {
          job.lastResult = `Error: ${err instanceof Error ? err.message : err}`;
          job.executionCount++;
        }

        if (!repeat || job.executionCount >= maxRepeats || (job.status as string) === 'cancelled') {
          job.status = 'completed';
          if (job.timer) {
            clearInterval(job.timer as ReturnType<typeof setInterval>);
            clearTimeout(job.timer as ReturnType<typeof setTimeout>);
          }
        } else {
          job.status = 'pending';
          job.nextRunAt = Date.now() + (intervalMs || delayMs);
        }
      };

      if (repeat && intervalMs) {
        // Initial delay, then repeating interval
        job.timer = setTimeout(() => {
          executeJob();
          job.timer = setInterval(executeJob, intervalMs);
        }, delayMs);
      } else {
        job.timer = setTimeout(executeJob, delayMs);
      }

      jobs.set(jobId, job);

      return {
        content: [{
          type: 'text',
          text: JSON.stringify({
            jobId,
            tool,
            delayMs,
            repeat,
            intervalMs: repeat ? intervalMs : undefined,
            maxRepeats: repeat ? maxRepeats : undefined,
            scheduledFor: new Date(now + delayMs).toISOString(),
          }, null, 2),
        }],
      };
    }

    case 'ros2_schedule_cancel': {
      const jobId = args.jobId as string;
      const job = jobs.get(jobId);

      if (!job) {
        return {
          content: [{ type: 'text', text: `Job "${jobId}" not found` }],
          isError: true,
        };
      }

      job.status = 'cancelled';
      if (job.timer) {
        clearInterval(job.timer as ReturnType<typeof setInterval>);
        clearTimeout(job.timer as ReturnType<typeof setTimeout>);
      }

      return {
        content: [{
          type: 'text',
          text: `Job "${jobId}" cancelled. Executed ${job.executionCount} times.`,
        }],
      };
    }

    case 'ros2_schedule_list': {
      const jobList = Array.from(jobs.values()).map(j => ({
        id: j.id,
        tool: j.tool,
        status: j.status,
        executionCount: j.executionCount,
        repeat: j.repeat,
        createdAt: new Date(j.createdAt).toISOString(),
        nextRunAt: j.status === 'pending' ? new Date(j.nextRunAt).toISOString() : null,
        lastResult: j.lastResult?.slice(0, 100),
      }));

      return {
        content: [{
          type: 'text',
          text: `Scheduled jobs (${jobList.length}):\n${JSON.stringify(jobList, null, 2)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown scheduled tool: ${name}` }], isError: true };
  }
}

// Export for testing
export { jobs, ScheduledJob };

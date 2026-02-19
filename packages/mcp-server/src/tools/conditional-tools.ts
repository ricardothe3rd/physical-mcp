/**
 * Conditional command execution: if sensor X reads Y, then do Z.
 *
 * Enables autonomous robot behaviors like:
 * - "If battery < 20%, navigate to charging station"
 * - "If distance sensor < 0.5m, stop moving"
 * - "If temperature > 80C, activate cooling"
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';
import { CommandType } from '../bridge/protocol.js';
import { handleTopicTool } from './topic-tools.js';
import { handleServiceTool } from './service-tools.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

type ComparisonOp = 'eq' | 'neq' | 'gt' | 'gte' | 'lt' | 'lte' | 'contains' | 'exists';

function evaluateCondition(
  actual: unknown,
  operator: ComparisonOp,
  expected: unknown,
): boolean {
  switch (operator) {
    case 'eq': return actual === expected;
    case 'neq': return actual !== expected;
    case 'gt': return typeof actual === 'number' && typeof expected === 'number' && actual > expected;
    case 'gte': return typeof actual === 'number' && typeof expected === 'number' && actual >= expected;
    case 'lt': return typeof actual === 'number' && typeof expected === 'number' && actual < expected;
    case 'lte': return typeof actual === 'number' && typeof expected === 'number' && actual <= expected;
    case 'contains': return typeof actual === 'string' && typeof expected === 'string' && actual.includes(expected);
    case 'exists': return actual !== undefined && actual !== null;
    default: return false;
  }
}

function extractField(obj: unknown, path: string): unknown {
  const parts = path.split('.');
  let current: unknown = obj;
  for (const part of parts) {
    if (current === null || current === undefined || typeof current !== 'object') {
      return undefined;
    }
    current = (current as Record<string, unknown>)[part];
  }
  return current;
}

export function getConditionalTools(): Tool[] {
  return [
    {
      name: 'ros2_conditional_execute',
      description: 'Execute a command conditionally based on a topic value. Reads the latest message from a topic, evaluates a condition, and if true, executes the specified action.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Topic to read the condition value from (e.g., "/battery_state")'),
        field: z.string().describe('Dot-path to the field in the message (e.g., "percentage" or "linear.x")'),
        operator: z.enum(['eq', 'neq', 'gt', 'gte', 'lt', 'lte', 'contains', 'exists'])
          .describe('Comparison operator'),
        value: z.unknown().optional().describe('Value to compare against (not needed for "exists")'),
        thenTool: z.string().describe('Tool to execute if condition is true'),
        thenArgs: z.record(z.unknown()).default({}).describe('Arguments for the then-tool'),
        elseTool: z.string().optional().describe('Tool to execute if condition is false (optional)'),
        elseArgs: z.record(z.unknown()).optional().describe('Arguments for the else-tool'),
      })),
    },
    {
      name: 'ros2_wait_for_condition',
      description: 'Wait until a topic value meets a condition, polling at a specified interval. Returns when the condition is met or timeout is reached.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Topic to monitor'),
        field: z.string().describe('Dot-path to the field in the message'),
        operator: z.enum(['eq', 'neq', 'gt', 'gte', 'lt', 'lte', 'contains', 'exists'])
          .describe('Comparison operator'),
        value: z.unknown().optional().describe('Value to compare against'),
        timeoutMs: z.number().default(10000).describe('Maximum wait time in ms (default: 10000)'),
        pollIntervalMs: z.number().default(500).describe('Poll interval in ms (default: 500)'),
      })),
    },
  ];
}

export async function handleConditionalTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine,
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_conditional_execute': {
      const topic = args.topic as string;
      const field = args.field as string;
      const operator = args.operator as ComparisonOp;
      const value = args.value;
      const thenTool = args.thenTool as string;
      const thenArgs = (args.thenArgs as Record<string, unknown>) || {};
      const elseTool = args.elseTool as string | undefined;
      const elseArgs = (args.elseArgs as Record<string, unknown>) || {};

      // Read latest message from topic
      let message: unknown;
      try {
        const response = await connection.send(CommandType.TOPIC_ECHO, { topic });
        message = response.data?.message ?? response.data;
      } catch (err) {
        return {
          content: [{ type: 'text', text: `Failed to read topic "${topic}": ${err instanceof Error ? err.message : err}` }],
          isError: true,
        };
      }

      const actualValue = extractField(message, field);
      const conditionMet = evaluateCondition(actualValue, operator, value);

      const conditionText = `${field} ${operator} ${JSON.stringify(value)} → actual: ${JSON.stringify(actualValue)} → ${conditionMet ? 'TRUE' : 'FALSE'}`;

      // Execute the appropriate tool
      const toolToRun = conditionMet ? thenTool : elseTool;
      const argsToUse = conditionMet ? thenArgs : elseArgs;

      if (!toolToRun) {
        return {
          content: [{
            type: 'text',
            text: JSON.stringify({
              condition: conditionText,
              conditionMet,
              action: 'none (no else-tool specified)',
            }, null, 2),
          }],
        };
      }

      // Dispatch to the right handler
      let result: { content: { type: string; text: string }[]; isError?: boolean };
      try {
        if (toolToRun.startsWith('ros2_topic_')) {
          result = await handleTopicTool(toolToRun, argsToUse, connection, safety);
        } else if (toolToRun.startsWith('ros2_service_')) {
          result = await handleServiceTool(toolToRun, argsToUse, connection, safety);
        } else if (toolToRun.startsWith('safety_')) {
          const { handleSafetyTool } = await import('./safety-tools.js');
          result = await handleSafetyTool(toolToRun, argsToUse, connection, safety);
        } else {
          result = { content: [{ type: 'text', text: `Unsupported tool for conditional: ${toolToRun}` }], isError: true };
        }
      } catch (err) {
        result = {
          content: [{ type: 'text', text: `Error executing ${toolToRun}: ${err instanceof Error ? err.message : err}` }],
          isError: true,
        };
      }

      return {
        content: [{
          type: 'text',
          text: JSON.stringify({
            condition: conditionText,
            conditionMet,
            executedTool: toolToRun,
            result: result.content[0]?.text || '',
            isError: result.isError,
          }, null, 2),
        }],
        isError: result.isError,
      };
    }

    case 'ros2_wait_for_condition': {
      const topic = args.topic as string;
      const field = args.field as string;
      const operator = args.operator as ComparisonOp;
      const value = args.value;
      const timeoutMs = (args.timeoutMs as number) || 10000;
      const pollIntervalMs = (args.pollIntervalMs as number) || 500;

      const startTime = Date.now();
      let lastValue: unknown;
      let attempts = 0;

      while (Date.now() - startTime < timeoutMs) {
        attempts++;
        try {
          const response = await connection.send(CommandType.TOPIC_ECHO, { topic });
          const message = response.data?.message ?? response.data;
          lastValue = extractField(message, field);

          if (evaluateCondition(lastValue, operator, value)) {
            return {
              content: [{
                type: 'text',
                text: JSON.stringify({
                  conditionMet: true,
                  field,
                  finalValue: lastValue,
                  attempts,
                  elapsedMs: Date.now() - startTime,
                }, null, 2),
              }],
            };
          }
        } catch {
          // Continue polling even if individual reads fail
        }

        await new Promise(resolve => setTimeout(resolve, pollIntervalMs));
      }

      return {
        content: [{
          type: 'text',
          text: JSON.stringify({
            conditionMet: false,
            field,
            lastValue,
            attempts,
            elapsedMs: Date.now() - startTime,
            reason: 'timeout',
          }, null, 2),
        }],
        isError: true,
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown conditional tool: ${name}` }], isError: true };
  }
}

// Export for testing
export { evaluateCondition, extractField };

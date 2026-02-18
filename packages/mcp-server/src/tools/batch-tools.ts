/**
 * Batch command execution tool: execute multiple ROS2 commands in a single call.
 *
 * Useful for AI agents that need to:
 * - Read multiple sensor topics at once
 * - Execute a sequence of commands atomically
 * - Reduce MCP round-trips for multi-step operations
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';
import { handleTopicTool } from './topic-tools.js';
import { handleServiceTool } from './service-tools.js';
import { handleActionTool } from './action-tools.js';
import { handleSystemTool } from './system-tools.js';
import { handleSafetyTool } from './safety-tools.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

const TOPIC_TOOL_NAMES = new Set([
  'ros2_topic_list', 'ros2_topic_info', 'ros2_topic_subscribe',
  'ros2_topic_publish', 'ros2_topic_echo',
]);
const SERVICE_TOOL_NAMES = new Set([
  'ros2_service_list', 'ros2_service_info', 'ros2_service_call',
]);
const ACTION_TOOL_NAMES = new Set([
  'ros2_action_list', 'ros2_action_send_goal', 'ros2_action_cancel', 'ros2_action_status',
]);
const SYSTEM_TOOL_NAMES = new Set([
  'system_bridge_status', 'system_node_list', 'system_node_info',
  'ros2_param_list', 'ros2_param_get', 'ros2_param_set',
]);

export function getBatchTools(): Tool[] {
  return [
    {
      name: 'ros2_batch_execute',
      description: 'Execute multiple ROS2 commands in a single call. Commands run sequentially. If stopOnError is true, execution stops at the first failure. Each command includes its own safety checks.',
      inputSchema: toInputSchema(z.object({
        commands: z.array(z.object({
          tool: z.string().describe('Tool name (e.g., ros2_topic_echo, ros2_service_call)'),
          args: z.record(z.unknown()).default({}).describe('Arguments for the tool'),
        })).min(1).max(20).describe('Array of commands to execute (1-20)'),
        stopOnError: z.boolean().default(false).describe('Stop execution on first error'),
      })),
    },
  ];
}

export async function handleBatchTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  if (name !== 'ros2_batch_execute') {
    return { content: [{ type: 'text', text: `Unknown batch tool: ${name}` }], isError: true };
  }

  const commands = args.commands as Array<{ tool: string; args: Record<string, unknown> }>;
  const stopOnError = (args.stopOnError as boolean) || false;

  if (!commands || !Array.isArray(commands) || commands.length === 0) {
    return { content: [{ type: 'text', text: 'No commands provided' }], isError: true };
  }

  if (commands.length > 20) {
    return { content: [{ type: 'text', text: 'Maximum 20 commands per batch' }], isError: true };
  }

  const results: Array<{ tool: string; success: boolean; result: string }> = [];
  let hasErrors = false;

  for (const command of commands) {
    const toolName = command.tool;
    const toolArgs = command.args || {};

    try {
      let result: { content: { type: string; text: string }[]; isError?: boolean };

      if (TOPIC_TOOL_NAMES.has(toolName)) {
        result = await handleTopicTool(toolName, toolArgs, connection, safety);
      } else if (SERVICE_TOOL_NAMES.has(toolName)) {
        result = await handleServiceTool(toolName, toolArgs, connection, safety);
      } else if (ACTION_TOOL_NAMES.has(toolName)) {
        result = await handleActionTool(toolName, toolArgs, connection, safety);
      } else if (SYSTEM_TOOL_NAMES.has(toolName)) {
        result = await handleSystemTool(toolName, toolArgs, connection);
      } else if (toolName.startsWith('safety_')) {
        result = await handleSafetyTool(toolName, toolArgs, connection, safety);
      } else {
        result = { content: [{ type: 'text', text: `Unknown tool: ${toolName}` }], isError: true };
      }

      const success = !result.isError;
      if (!success) hasErrors = true;

      results.push({
        tool: toolName,
        success,
        result: result.content[0]?.text || '',
      });

      if (!success && stopOnError) {
        break;
      }
    } catch (err) {
      hasErrors = true;
      results.push({
        tool: toolName,
        success: false,
        result: `Error: ${err instanceof Error ? err.message : String(err)}`,
      });

      if (stopOnError) {
        break;
      }
    }
  }

  const succeeded = results.filter(r => r.success).length;
  const failed = results.filter(r => !r.success).length;
  const summary = `Batch: ${succeeded} succeeded, ${failed} failed, ${results.length}/${commands.length} executed`;

  const output = {
    summary,
    results,
  };

  return {
    content: [{ type: 'text', text: JSON.stringify(output, null, 2) }],
    isError: hasErrors ? true : undefined,
  };
}

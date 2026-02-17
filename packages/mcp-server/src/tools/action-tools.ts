/**
 * ROS2 action MCP tools: list, send_goal, cancel, status.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';

export function getActionTools(): Tool[] {
  return [
    {
      name: 'ros2_action_list',
      description: 'List all available ROS2 action servers',
      inputSchema: zodToJsonSchema(z.object({})) as Tool['inputSchema'],
    },
    {
      name: 'ros2_action_send_goal',
      description: 'Send a goal to a ROS2 action server. Subject to safety checks.',
      inputSchema: zodToJsonSchema(z.object({
        action: z.string().describe('Full action name (e.g. /navigate_to_pose)'),
        actionType: z.string().describe('Action type (e.g. nav2_msgs/action/NavigateToPose)'),
        goal: z.record(z.unknown()).describe('Goal data as JSON'),
      })) as Tool['inputSchema'],
    },
    {
      name: 'ros2_action_cancel',
      description: 'Cancel an active action goal',
      inputSchema: zodToJsonSchema(z.object({
        action: z.string().describe('Full action name'),
        goalId: z.string().optional().describe('Specific goal ID to cancel (cancels all if omitted)'),
      })) as Tool['inputSchema'],
    },
    {
      name: 'ros2_action_status',
      description: 'Get the status of action goals',
      inputSchema: zodToJsonSchema(z.object({
        action: z.string().describe('Full action name'),
      })) as Tool['inputSchema'],
    },
  ];
}

export async function handleActionTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_action_list': {
      const response = await connection.send(CommandType.ACTION_LIST);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_action_send_goal': {
      const action = args.action as string;
      const goal = args.goal as Record<string, unknown>;

      // SAFETY CHECK
      const check = safety.checkActionGoal(action, goal);
      if (!check.allowed) {
        const violationText = check.violations
          .map(v => `- [${v.type}] ${v.message}`)
          .join('\n');
        return {
          content: [{ type: 'text', text: `SAFETY BLOCKED: Action goal to ${action} denied.\n\nViolations:\n${violationText}` }],
          isError: true,
        };
      }

      const response = await connection.send(CommandType.ACTION_SEND_GOAL, {
        action,
        action_type: args.actionType,
        goal,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_action_cancel': {
      const response = await connection.send(CommandType.ACTION_CANCEL, {
        action: args.action,
        goal_id: args.goalId,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: `Action cancelled successfully` }] };
    }

    case 'ros2_action_status': {
      const response = await connection.send(CommandType.ACTION_STATUS, { action: args.action });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown action tool: ${name}` }], isError: true };
  }
}

/**
 * ROS2 type introspection MCP tools: message, service, and action type info.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getIntrospectionTools(): Tool[] {
  return [
    {
      name: 'ros2_msg_type_info',
      description: 'Get the full definition of a ROS2 message type (fields, types, defaults)',
      inputSchema: toInputSchema(z.object({
        type_name: z.string().describe('Fully qualified message type (e.g. "geometry_msgs/msg/Twist")'),
      })),
    },
    {
      name: 'ros2_srv_type_info',
      description: 'Get the full definition of a ROS2 service type (request and response fields)',
      inputSchema: toInputSchema(z.object({
        type_name: z.string().describe('Fully qualified service type (e.g. "std_srvs/srv/SetBool")'),
      })),
    },
    {
      name: 'ros2_action_type_info',
      description: 'Get the full definition of a ROS2 action type (goal, result, and feedback fields)',
      inputSchema: toInputSchema(z.object({
        type_name: z.string().describe('Fully qualified action type (e.g. "nav2_msgs/action/NavigateToPose")'),
      })),
    },
  ];
}

export async function handleIntrospectionTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_msg_type_info': {
      const typeName = args.type_name as string;

      if (!typeName) {
        return {
          content: [{ type: 'text', text: 'Parameter "type_name" is required' }],
          isError: true,
        };
      }

      const response = await connection.send('msg.type_info' as any, { type_name: typeName });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_srv_type_info': {
      const typeName = args.type_name as string;

      if (!typeName) {
        return {
          content: [{ type: 'text', text: 'Parameter "type_name" is required' }],
          isError: true,
        };
      }

      const response = await connection.send('srv.type_info' as any, { type_name: typeName });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_action_type_info': {
      const typeName = args.type_name as string;

      if (!typeName) {
        return {
          content: [{ type: 'text', text: 'Parameter "type_name" is required' }],
          isError: true,
        };
      }

      const response = await connection.send('action.type_info' as any, { type_name: typeName });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown introspection tool: ${name}` }], isError: true };
  }
}

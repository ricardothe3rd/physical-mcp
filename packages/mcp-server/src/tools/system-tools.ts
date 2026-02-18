/**
 * System MCP tools: bridge status, node list.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getSystemTools(): Tool[] {
  return [
    {
      name: 'system_bridge_status',
      description: 'Get the connection status of the ROS2 bridge (WebSocket health, latency)',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'system_node_list',
      description: 'List all active ROS2 nodes',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'system_node_info',
      description: 'Get detailed information about a specific ROS2 node (subscribers, publishers, services)',
      inputSchema: toInputSchema(z.object({
        nodeName: z.string().describe('Full node name (e.g., "/turtlebot3/controller")'),
      })),
    },
    {
      name: 'ros2_param_list',
      description: 'List all parameters for a ROS2 node',
      inputSchema: toInputSchema(z.object({
        nodeName: z.string().describe('Full node name'),
      })),
    },
    {
      name: 'ros2_param_get',
      description: 'Get the value of a ROS2 parameter',
      inputSchema: toInputSchema(z.object({
        nodeName: z.string().describe('Full node name'),
        paramName: z.string().describe('Parameter name'),
      })),
    },
    {
      name: 'ros2_param_set',
      description: 'Set the value of a ROS2 parameter. SAFETY CHECKED — blocked parameters are protected.',
      inputSchema: toInputSchema(z.object({
        nodeName: z.string().describe('Full node name'),
        paramName: z.string().describe('Parameter name'),
        value: z.unknown().describe('New parameter value'),
      })),
    },
  ];
}

export async function handleSystemTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'system_bridge_status': {
      const connected = connection.isConnected;
      const available = connection.isBridgeAvailable;

      if (!connected) {
        return {
          content: [{ type: 'text', text: JSON.stringify({ connected: false, available: false, message: 'Bridge not connected' }, null, 2) }],
        };
      }

      // Ping the bridge for latency check
      try {
        const start = Date.now();
        const response = await connection.send(CommandType.PING);
        const latencyMs = Date.now() - start;

        return {
          content: [{
            type: 'text',
            text: JSON.stringify({
              connected,
              available,
              latencyMs,
              bridgeData: response.data,
            }, null, 2),
          }],
        };
      } catch (err) {
        return {
          content: [{ type: 'text', text: JSON.stringify({ connected, available, error: String(err) }, null, 2) }],
        };
      }
    }

    case 'system_node_list': {
      const response = await connection.send(CommandType.NODE_LIST);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'system_node_info': {
      const response = await connection.send(CommandType.NODE_INFO, {
        node_name: args.nodeName as string,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_param_list': {
      const response = await connection.send(CommandType.LIST_PARAMS, {
        node_name: args.nodeName as string,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_param_get': {
      const response = await connection.send(CommandType.GET_PARAMS, {
        node_name: args.nodeName as string,
        param_name: args.paramName as string,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `${args.nodeName}/${args.paramName} = ${JSON.stringify(response.data)}`,
        }],
      };
    }

    case 'ros2_param_set': {
      const response = await connection.send(CommandType.SET_PARAMS, {
        node_name: args.nodeName as string,
        param_name: args.paramName as string,
        value: args.value,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `Set ${args.nodeName}/${args.paramName} = ${JSON.stringify(args.value)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown system tool: ${name}` }], isError: true };
  }
}

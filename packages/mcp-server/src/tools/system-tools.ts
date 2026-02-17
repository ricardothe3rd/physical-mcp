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
  ];
}

export async function handleSystemTool(
  name: string,
  _args: Record<string, unknown>,
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

    default:
      return { content: [{ type: 'text', text: `Unknown system tool: ${name}` }], isError: true };
  }
}

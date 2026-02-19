/**
 * ROS2 diagnostic MCP tools: system summary, per-device detail.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getDiagnosticTools(): Tool[] {
  return [
    {
      name: 'ros2_diagnostics_summary',
      description: 'Get an overall summary of system diagnostics (all hardware and software status)',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_diagnostics_detail',
      description: 'Get detailed diagnostics for a specific hardware component by name or hardware ID',
      inputSchema: toInputSchema(z.object({
        name: z.string().describe('Hardware name or ID to query (e.g., "motor_controller", "imu_sensor")'),
      })),
    },
  ];
}

export async function handleDiagnosticTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_diagnostics_summary': {
      const response = await connection.send('diagnostics.summary' as any);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_diagnostics_detail': {
      const deviceName = args.name as string;

      if (!deviceName) {
        return {
          content: [{ type: 'text', text: 'Parameter "name" is required for diagnostics detail' }],
          isError: true,
        };
      }

      const response = await connection.send('diagnostics.detail' as any, {
        name: deviceName,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `Diagnostics for "${deviceName}":\n${JSON.stringify(response.data, null, 2)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown diagnostic tool: ${name}` }], isError: true };
  }
}

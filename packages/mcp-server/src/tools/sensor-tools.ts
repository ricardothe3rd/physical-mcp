/**
 * ROS2 sensor data MCP tools: summary of all sensors, read specific sensor.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getSensorTools(): Tool[] {
  return [
    {
      name: 'ros2_sensor_summary',
      description: 'Get a summary of all available sensor topics categorized by type (camera, lidar, IMU, etc.)',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_sensor_read',
      description: 'Read the latest value from a specific sensor topic',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Full sensor topic name (e.g. "/imu/data")'),
        timeout_ms: z.number().default(5000).describe('Timeout in milliseconds to wait for a reading'),
      })),
    },
  ];
}

export async function handleSensorTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_sensor_summary': {
      const response = await connection.send('sensor.summary' as any);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_sensor_read': {
      const topic = args.topic as string;
      const timeoutMs = (args.timeout_ms as number) || 5000;

      if (!topic) {
        return {
          content: [{ type: 'text', text: 'Parameter "topic" is required' }],
          isError: true,
        };
      }

      const response = await connection.send('sensor.read' as any, {
        topic,
        timeout_ms: timeoutMs,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `Sensor reading from "${topic}":\n${JSON.stringify(response.data, null, 2)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown sensor tool: ${name}` }], isError: true };
  }
}

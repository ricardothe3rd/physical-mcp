/**
 * ROS2 battery/power monitoring MCP tools: battery status, power supply info.
 *
 * Many robots publish battery status on standard ROS2 topics such as
 * /battery_state (sensor_msgs/msg/BatteryState).
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getPowerTools(): Tool[] {
  return [
    {
      name: 'ros2_battery_status',
      description:
        'Get battery status including percentage, voltage, current, temperature, and charging state. ' +
        'Subscribes to a BatteryState topic (sensor_msgs/msg/BatteryState).',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/battery_state').describe('Battery state topic (default: "/battery_state")'),
        timeout_ms: z.number().default(5000).describe('Timeout in milliseconds to wait for a reading'),
      })),
    },
    {
      name: 'ros2_power_supply_status',
      description:
        'Get power supply information including supply status, health, technology, and cell voltages. ' +
        'Reads extended fields from a BatteryState topic.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/battery_state').describe('Battery state topic (default: "/battery_state")'),
        timeout_ms: z.number().default(5000).describe('Timeout in milliseconds to wait for a reading'),
      })),
    },
  ];
}

export async function handlePowerTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_battery_status': {
      const topic = (args.topic as string) || '/battery_state';
      const timeoutMs = (args.timeout_ms as number) || 5000;

      const response = await connection.send('power.battery_status' as any, {
        topic,
        timeout_ms: timeoutMs,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `Battery status from "${topic}":\n${JSON.stringify(response.data, null, 2)}`,
        }],
      };
    }

    case 'ros2_power_supply_status': {
      const topic = (args.topic as string) || '/battery_state';
      const timeoutMs = (args.timeout_ms as number) || 5000;

      const response = await connection.send('power.supply_status' as any, {
        topic,
        timeout_ms: timeoutMs,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `Power supply info from "${topic}":\n${JSON.stringify(response.data, null, 2)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown power tool: ${name}` }], isError: true };
  }
}

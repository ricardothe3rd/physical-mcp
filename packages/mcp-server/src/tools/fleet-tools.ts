/**
 * Fleet management MCP tools: status, add, remove robots.
 *
 * Maintains a local registry of robots (namespace -> status) for fleet-level
 * awareness. Does not require bridge communication — purely local state.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

interface FleetRobot {
  namespace: string;
  status: 'online' | 'offline' | 'unknown';
  addedAt: number;
  lastSeen?: number;
}

const fleetRegistry = new Map<string, FleetRobot>();

export function getFleetTools(): Tool[] {
  return [
    {
      name: 'ros2_fleet_status',
      description: 'List all robots in the fleet registry and their connection status',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_fleet_add',
      description: 'Register a robot in the fleet registry by its namespace',
      inputSchema: toInputSchema(z.object({
        namespace: z.string().describe('Robot namespace (e.g., "/robot1", "/turtlebot3")'),
      })),
    },
    {
      name: 'ros2_fleet_remove',
      description: 'Remove a robot from the fleet registry by its namespace',
      inputSchema: toInputSchema(z.object({
        namespace: z.string().describe('Robot namespace to remove'),
      })),
    },
  ];
}

export async function handleFleetTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_fleet_status': {
      const robots = Array.from(fleetRegistry.values()).map(r => ({
        namespace: r.namespace,
        status: r.status,
        addedAt: new Date(r.addedAt).toISOString(),
        lastSeen: r.lastSeen ? new Date(r.lastSeen).toISOString() : null,
      }));

      return {
        content: [{
          type: 'text',
          text: `Fleet robots (${robots.length}):\n${JSON.stringify(robots, null, 2)}`,
        }],
      };
    }

    case 'ros2_fleet_add': {
      const namespace = args.namespace as string;

      if (!namespace) {
        return {
          content: [{ type: 'text', text: 'Parameter "namespace" is required' }],
          isError: true,
        };
      }

      if (fleetRegistry.has(namespace)) {
        return {
          content: [{ type: 'text', text: `Robot "${namespace}" is already registered in the fleet` }],
          isError: true,
        };
      }

      const now = Date.now();
      const robot: FleetRobot = {
        namespace,
        status: 'unknown',
        addedAt: now,
      };

      fleetRegistry.set(namespace, robot);

      return {
        content: [{
          type: 'text',
          text: `Robot "${namespace}" added to fleet. Total robots: ${fleetRegistry.size}`,
        }],
      };
    }

    case 'ros2_fleet_remove': {
      const namespace = args.namespace as string;

      if (!namespace) {
        return {
          content: [{ type: 'text', text: 'Parameter "namespace" is required' }],
          isError: true,
        };
      }

      if (!fleetRegistry.has(namespace)) {
        return {
          content: [{ type: 'text', text: `Robot "${namespace}" not found in fleet registry` }],
          isError: true,
        };
      }

      fleetRegistry.delete(namespace);

      return {
        content: [{
          type: 'text',
          text: `Robot "${namespace}" removed from fleet. Remaining robots: ${fleetRegistry.size}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown fleet tool: ${name}` }], isError: true };
  }
}

// Export for testing
export { fleetRegistry, FleetRobot };

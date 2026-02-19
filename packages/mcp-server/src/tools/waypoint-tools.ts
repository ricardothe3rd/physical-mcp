/**
 * Waypoint management tools for robot navigation.
 *
 * Enables AI agents to save, list, delete, and navigate to named waypoints.
 * Waypoints are stored in memory and can be used with nav2 action goals.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';
import { CommandType } from '../bridge/protocol.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export interface Waypoint {
  name: string;
  x: number;
  y: number;
  z: number;
  orientation?: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
  frame: string;
  createdAt: number;
}

export const waypointStore = new Map<string, Waypoint>();

export function getWaypointTools(): Tool[] {
  return [
    {
      name: 'ros2_waypoint_save',
      description: 'Save a named waypoint with position and optional orientation. Use with navigation tools to revisit locations.',
      inputSchema: toInputSchema(z.object({
        name: z.string().describe('Unique name for this waypoint'),
        x: z.number().describe('X position in meters'),
        y: z.number().describe('Y position in meters'),
        z: z.number().default(0).describe('Z position in meters (default: 0)'),
        orientationX: z.number().optional().describe('Quaternion X (optional)'),
        orientationY: z.number().optional().describe('Quaternion Y (optional)'),
        orientationZ: z.number().optional().describe('Quaternion Z (optional)'),
        orientationW: z.number().optional().describe('Quaternion W (optional)'),
        frame: z.string().default('map').describe('Coordinate frame (default: "map")'),
      })),
    },
    {
      name: 'ros2_waypoint_list',
      description: 'List all saved waypoints with their positions',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_waypoint_delete',
      description: 'Delete a saved waypoint by name',
      inputSchema: toInputSchema(z.object({
        name: z.string().describe('Name of the waypoint to delete'),
      })),
    },
    {
      name: 'ros2_waypoint_navigate',
      description: 'Send a navigation goal to a saved waypoint using nav2 NavigateToPose action. Safety-checked against geofence.',
      inputSchema: toInputSchema(z.object({
        name: z.string().describe('Name of the waypoint to navigate to'),
      })),
    },
  ];
}

export async function handleWaypointTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_waypoint_save': {
      const wpName = args.name as string;
      const x = args.x as number;
      const y = args.y as number;
      const zPos = (args.z as number) ?? 0;
      const frame = (args.frame as string) || 'map';

      const waypoint: Waypoint = {
        name: wpName,
        x,
        y,
        z: zPos,
        frame,
        createdAt: Date.now(),
      };

      if (args.orientationW !== undefined) {
        waypoint.orientation = {
          x: (args.orientationX as number) ?? 0,
          y: (args.orientationY as number) ?? 0,
          z: (args.orientationZ as number) ?? 0,
          w: (args.orientationW as number) ?? 1,
        };
      }

      const existed = waypointStore.has(wpName);
      waypointStore.set(wpName, waypoint);

      return {
        content: [{
          type: 'text',
          text: existed
            ? `Waypoint "${wpName}" updated: (${x}, ${y}, ${zPos}) in frame "${frame}"`
            : `Waypoint "${wpName}" saved: (${x}, ${y}, ${zPos}) in frame "${frame}"`,
        }],
      };
    }

    case 'ros2_waypoint_list': {
      const waypoints = Array.from(waypointStore.values()).map(wp => ({
        name: wp.name,
        position: { x: wp.x, y: wp.y, z: wp.z },
        orientation: wp.orientation || null,
        frame: wp.frame,
        createdAt: new Date(wp.createdAt).toISOString(),
      }));

      return {
        content: [{
          type: 'text',
          text: waypoints.length > 0
            ? JSON.stringify(waypoints, null, 2)
            : 'No waypoints saved.',
        }],
      };
    }

    case 'ros2_waypoint_delete': {
      const wpName = args.name as string;

      if (!waypointStore.has(wpName)) {
        return {
          content: [{ type: 'text', text: `Waypoint "${wpName}" not found` }],
          isError: true,
        };
      }

      waypointStore.delete(wpName);
      return {
        content: [{ type: 'text', text: `Waypoint "${wpName}" deleted` }],
      };
    }

    case 'ros2_waypoint_navigate': {
      const wpName = args.name as string;
      const waypoint = waypointStore.get(wpName);

      if (!waypoint) {
        return {
          content: [{ type: 'text', text: `Waypoint "${wpName}" not found` }],
          isError: true,
        };
      }

      // Safety check: verify waypoint position is within geofence
      const positionCheck = safety.checkPosition({ x: waypoint.x, y: waypoint.y, z: waypoint.z });
      if (!positionCheck.inside) {
        return {
          content: [{
            type: 'text',
            text: `Navigation blocked: waypoint "${wpName}" at (${waypoint.x}, ${waypoint.y}, ${waypoint.z}) is outside geofence boundary`,
          }],
          isError: true,
        };
      }

      // Send nav2 goal via action
      const goal = {
        pose: {
          header: { frame_id: waypoint.frame },
          pose: {
            position: { x: waypoint.x, y: waypoint.y, z: waypoint.z },
            orientation: waypoint.orientation || { x: 0, y: 0, z: 0, w: 1 },
          },
        },
      };

      try {
        const response = await connection.send(CommandType.ACTION_SEND_GOAL, {
          action: '/navigate_to_pose',
          action_type: 'nav2_msgs/action/NavigateToPose',
          goal,
        });

        return {
          content: [{
            type: 'text',
            text: `Navigating to waypoint "${wpName}" at (${waypoint.x}, ${waypoint.y}, ${waypoint.z})\n${JSON.stringify(response.data, null, 2)}`,
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Navigation failed: ${message}` }],
          isError: true,
        };
      }
    }

    default:
      return { content: [{ type: 'text', text: `Unknown waypoint tool: ${name}` }], isError: true };
  }
}

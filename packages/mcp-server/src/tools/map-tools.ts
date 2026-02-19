/**
 * ROS2 map MCP tools: map info, costmap info, map topic discovery.
 *
 * These tools help AI agents inspect occupancy grid metadata, costmap
 * parameters, and discover available map-related topics on a ROS2 system.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getMapTools(): Tool[] {
  return [
    {
      name: 'ros2_map_info',
      description:
        'Get map metadata (resolution, width, height, origin) by subscribing to an OccupancyGrid topic ' +
        '(nav_msgs/msg/OccupancyGrid).',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/map').describe('Map topic (default: "/map")'),
      })),
    },
    {
      name: 'ros2_costmap_info',
      description:
        'Get costmap metadata (resolution, width, height, origin) by subscribing to a costmap topic ' +
        '(nav_msgs/msg/OccupancyGrid).',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/local_costmap/costmap').describe('Costmap topic (default: "/local_costmap/costmap")'),
      })),
    },
    {
      name: 'ros2_map_topics',
      description:
        'List all topics that publish map-related messages (nav_msgs/msg/OccupancyGrid or nav_msgs/msg/MapMetaData).',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handleMapTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_map_info': {
      const topic = (args.topic as string) || '/map';

      if (!topic.startsWith('/')) {
        return {
          content: [{ type: 'text', text: 'Invalid topic: topic name must start with "/"' }],
          isError: true,
        };
      }

      const response = await connection.send(CommandType.TOPIC_SUBSCRIBE, {
        topic,
        count: 1,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }

      const data = response.data as Record<string, unknown> | undefined;
      const info: Record<string, unknown> = {};
      if (data) {
        if (data.info !== undefined) {
          // OccupancyGrid has an `info` field with MapMetaData
          const meta = data.info as Record<string, unknown>;
          if (meta.resolution !== undefined) info.resolution = meta.resolution;
          if (meta.width !== undefined) info.width = meta.width;
          if (meta.height !== undefined) info.height = meta.height;
          if (meta.origin !== undefined) info.origin = meta.origin;
        } else {
          // Fallback: fields may be at top level
          if (data.resolution !== undefined) info.resolution = data.resolution;
          if (data.width !== undefined) info.width = data.width;
          if (data.height !== undefined) info.height = data.height;
          if (data.origin !== undefined) info.origin = data.origin;
        }
      }

      return {
        content: [{
          type: 'text',
          text: `Map info from "${topic}":\n${JSON.stringify(info, null, 2)}`,
        }],
      };
    }

    case 'ros2_costmap_info': {
      const topic = (args.topic as string) || '/local_costmap/costmap';

      if (!topic.startsWith('/')) {
        return {
          content: [{ type: 'text', text: 'Invalid topic: topic name must start with "/"' }],
          isError: true,
        };
      }

      const response = await connection.send(CommandType.TOPIC_SUBSCRIBE, {
        topic,
        count: 1,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }

      const data = response.data as Record<string, unknown> | undefined;
      const info: Record<string, unknown> = {};
      if (data) {
        if (data.info !== undefined) {
          const meta = data.info as Record<string, unknown>;
          if (meta.resolution !== undefined) info.resolution = meta.resolution;
          if (meta.width !== undefined) info.width = meta.width;
          if (meta.height !== undefined) info.height = meta.height;
          if (meta.origin !== undefined) info.origin = meta.origin;
        } else {
          if (data.resolution !== undefined) info.resolution = data.resolution;
          if (data.width !== undefined) info.width = data.width;
          if (data.height !== undefined) info.height = data.height;
          if (data.origin !== undefined) info.origin = data.origin;
        }
      }

      return {
        content: [{
          type: 'text',
          text: `Costmap info from "${topic}":\n${JSON.stringify(info, null, 2)}`,
        }],
      };
    }

    case 'ros2_map_topics': {
      const response = await connection.send(CommandType.TOPIC_LIST);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }

      const allTopics = response.data as Array<{ name: string; type: string }> | undefined;
      const mapTypes = ['nav_msgs/msg/OccupancyGrid', 'nav_msgs/msg/MapMetaData'];
      const filtered = (allTopics || []).filter(
        (t) => mapTypes.includes(t.type)
      );

      return {
        content: [{
          type: 'text',
          text: `Map topics (${filtered.length} found):\n${JSON.stringify(filtered, null, 2)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown map tool: ${name}` }], isError: true };
  }
}

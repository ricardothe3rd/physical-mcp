/**
 * ROS2 camera/image MCP tools: camera info, image preview metadata, image topic discovery.
 *
 * These tools help AI agents inspect camera calibration data, image metadata,
 * and discover available image topics on a ROS2 system.  Because MCP is a
 * text-based protocol, raw pixel data is NOT transferred -- only metadata.
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

export function getCameraTools(): Tool[] {
  return [
    {
      name: 'ros2_camera_info',
      description:
        'Get camera calibration data (intrinsics, distortion model, resolution) by subscribing to a CameraInfo topic ' +
        '(sensor_msgs/msg/CameraInfo).',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/camera/camera_info').describe('CameraInfo topic (default: "/camera/camera_info")'),
      })),
    },
    {
      name: 'ros2_image_preview',
      description:
        'Get image metadata (width, height, encoding, step, data length) from an Image topic. ' +
        'Raw pixel data is NOT returned because MCP is a text-based protocol.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/camera/image_raw').describe('Image topic (default: "/camera/image_raw")'),
        encoding: z.string().optional().describe('Expected encoding (e.g. "rgb8", "bgr8", "mono8")'),
      })),
    },
    {
      name: 'ros2_image_topics',
      description:
        'List all topics that publish image messages (sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage).',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handleCameraTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_camera_info': {
      const topic = (args.topic as string) || '/camera/camera_info';

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
      return {
        content: [{
          type: 'text',
          text: `Camera info from "${topic}":\n${JSON.stringify(response.data, null, 2)}`,
        }],
      };
    }

    case 'ros2_image_preview': {
      const topic = (args.topic as string) || '/camera/image_raw';
      const encoding = args.encoding as string | undefined;

      if (!topic.startsWith('/')) {
        return {
          content: [{ type: 'text', text: 'Invalid topic: topic name must start with "/"' }],
          isError: true,
        };
      }

      const params: Record<string, unknown> = { topic, count: 1 };
      if (encoding) {
        params.encoding = encoding;
      }

      const response = await connection.send(CommandType.TOPIC_SUBSCRIBE, params);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }

      const data = response.data as Record<string, unknown> | undefined;
      const metadata: Record<string, unknown> = {};
      if (data) {
        if (data.width !== undefined) metadata.width = data.width;
        if (data.height !== undefined) metadata.height = data.height;
        if (data.encoding !== undefined) metadata.encoding = data.encoding;
        if (data.step !== undefined) metadata.step = data.step;
        if (data.data !== undefined) {
          // Report byte count rather than transferring raw pixels
          const raw = data.data;
          metadata.data_length = Array.isArray(raw) ? raw.length : typeof raw === 'string' ? raw.length : 0;
        }
      }

      return {
        content: [{
          type: 'text',
          text:
            `Image metadata from "${topic}":\n${JSON.stringify(metadata, null, 2)}\n\n` +
            'Note: Raw pixel data is not returned due to MCP text protocol limitations.',
        }],
      };
    }

    case 'ros2_image_topics': {
      const response = await connection.send(CommandType.TOPIC_LIST);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }

      const allTopics = response.data as Array<{ name: string; type: string }> | undefined;
      const imageTypes = ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'];
      const filtered = (allTopics || []).filter(
        (t) => imageTypes.includes(t.type)
      );

      return {
        content: [{
          type: 'text',
          text: `Image topics (${filtered.length} found):\n${JSON.stringify(filtered, null, 2)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown camera tool: ${name}` }], isError: true };
  }
}

/**
 * ROS2 TF2 transform MCP tools: tree view, transform lookup.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getTfTools(): Tool[] {
  return [
    {
      name: 'ros2_tf_tree',
      description: 'Get the TF2 transform tree showing all frames and their parent-child relationships',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_tf_lookup',
      description: 'Look up the transform (translation + rotation) between two TF2 frames',
      inputSchema: toInputSchema(z.object({
        sourceFrame: z.string().describe('Source frame ID (e.g., "base_link")'),
        targetFrame: z.string().describe('Target frame ID (e.g., "odom")'),
      })),
    },
  ];
}

export async function handleTfTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_tf_tree': {
      const response = await connection.send('tf.tree' as any);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_tf_lookup': {
      const sourceFrame = args.sourceFrame as string;
      const targetFrame = args.targetFrame as string;

      if (!sourceFrame || !targetFrame) {
        return {
          content: [{ type: 'text', text: 'Both sourceFrame and targetFrame are required' }],
          isError: true,
        };
      }

      const response = await connection.send('tf.lookup' as any, {
        source_frame: sourceFrame,
        target_frame: targetFrame,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `Transform from "${sourceFrame}" to "${targetFrame}":\n${JSON.stringify(response.data, null, 2)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown TF tool: ${name}` }], isError: true };
  }
}

/**
 * ROS2 namespace management MCP tools: list, remap, clear remappings.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

/** In-memory store for namespace remappings. Exported for testing. */
export const remapStore: Map<string, string> = new Map();

/** Reset the remap store. Exported for test cleanup. */
export function resetRemapStore(): void {
  remapStore.clear();
}

export function getNamespaceTools(): Tool[] {
  return [
    {
      name: 'ros2_namespace_list',
      description: 'List all discovered ROS2 namespaces in the system',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_namespace_remap',
      description: 'Set a temporary namespace remapping (from one namespace to another)',
      inputSchema: toInputSchema(z.object({
        from_namespace: z.string().describe('Source namespace to remap from (e.g. "/robot1")'),
        to_namespace: z.string().describe('Target namespace to remap to (e.g. "/robot2")'),
      })),
    },
    {
      name: 'ros2_namespace_clear_remaps',
      description: 'Clear all active namespace remappings',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handleNamespaceTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_namespace_list': {
      const response = await connection.send('namespace.list' as any);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_namespace_remap': {
      const fromNs = args.from_namespace as string;
      const toNs = args.to_namespace as string;

      if (!fromNs || !toNs) {
        return {
          content: [{ type: 'text', text: 'Both "from_namespace" and "to_namespace" are required' }],
          isError: true,
        };
      }

      remapStore.set(fromNs, toNs);
      return {
        content: [{
          type: 'text',
          text: `Namespace remapping set: "${fromNs}" -> "${toNs}". Active remappings: ${remapStore.size}`,
        }],
      };
    }

    case 'ros2_namespace_clear_remaps': {
      const count = remapStore.size;
      remapStore.clear();
      return {
        content: [{
          type: 'text',
          text: `Cleared ${count} namespace remapping(s)`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown namespace tool: ${name}` }], isError: true };
  }
}

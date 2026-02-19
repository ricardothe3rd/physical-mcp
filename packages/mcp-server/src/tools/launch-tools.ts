/**
 * ROS2 launch file management tools.
 *
 * Enables AI agents to list, start, and stop ROS2 launch processes
 * via the bridge agent.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

interface LaunchProcess {
  id: string;
  packageName: string;
  launchFile: string;
  args: Record<string, string>;
  startedAt: number;
  status: 'running' | 'stopped' | 'error';
}

const activeProcesses = new Map<string, LaunchProcess>();
let nextProcessId = 1;

export function getLaunchTools(): Tool[] {
  return [
    {
      name: 'ros2_launch_start',
      description: 'Start a ROS2 launch file. Executes `ros2 launch <package> <launch_file>` on the bridge host.',
      inputSchema: toInputSchema(z.object({
        packageName: z.string().describe('ROS2 package name containing the launch file'),
        launchFile: z.string().describe('Launch file name (e.g., "robot.launch.py")'),
        args: z.record(z.string()).optional().describe('Launch arguments as key-value pairs'),
      })),
    },
    {
      name: 'ros2_launch_stop',
      description: 'Stop a running launch process by its process ID',
      inputSchema: toInputSchema(z.object({
        processId: z.string().describe('Launch process ID returned by ros2_launch_start'),
      })),
    },
    {
      name: 'ros2_launch_status',
      description: 'List all active launch processes and their status',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handleLaunchTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_launch_start': {
      const packageName = args.packageName as string;
      const launchFile = args.launchFile as string;
      const launchArgs = (args.args as Record<string, string>) || {};

      const id = `launch_${nextProcessId++}`;

      try {
        const response = await connection.send('launch.start' as any, {
          package: packageName,
          launch_file: launchFile,
          args: launchArgs,
          process_id: id,
        });

        const process: LaunchProcess = {
          id,
          packageName,
          launchFile,
          args: launchArgs,
          startedAt: Date.now(),
          status: response.status === 'ok' ? 'running' : 'error',
        };
        activeProcesses.set(id, process);

        if (response.status === 'error') {
          return {
            content: [{ type: 'text', text: `Failed to start launch: ${JSON.stringify(response.data)}` }],
            isError: true,
          };
        }

        return {
          content: [{
            type: 'text',
            text: `Launch started: ${packageName}/${launchFile} (process ID: ${id})`,
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Failed to start launch: ${message}` }],
          isError: true,
        };
      }
    }

    case 'ros2_launch_stop': {
      const processId = args.processId as string;
      const process = activeProcesses.get(processId);

      if (!process) {
        return {
          content: [{ type: 'text', text: `Launch process "${processId}" not found` }],
          isError: true,
        };
      }

      try {
        await connection.send('launch.stop' as any, { process_id: processId });
        process.status = 'stopped';
        activeProcesses.delete(processId);

        return {
          content: [{
            type: 'text',
            text: `Launch stopped: ${process.packageName}/${process.launchFile} (${processId})`,
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Failed to stop launch: ${message}` }],
          isError: true,
        };
      }
    }

    case 'ros2_launch_status': {
      const processes = Array.from(activeProcesses.values()).map(p => ({
        id: p.id,
        package: p.packageName,
        launchFile: p.launchFile,
        status: p.status,
        uptimeMs: Date.now() - p.startedAt,
        args: p.args,
      }));

      return {
        content: [{
          type: 'text',
          text: processes.length > 0
            ? JSON.stringify(processes, null, 2)
            : 'No active launch processes.',
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown launch tool: ${name}` }], isError: true };
  }
}

/** Reset all processes (for testing). */
export function resetLaunchProcesses(): void {
  activeProcesses.clear();
  nextProcessId = 1;
}

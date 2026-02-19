/**
 * Network quality monitoring tools for the ROS2 bridge connection.
 *
 * Provides tools to check bridge connection health, monitor bandwidth
 * usage per topic, and run latency tests.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getNetworkTools(): Tool[] {
  return [
    {
      name: 'ros2_network_stats',
      description:
        'Get network statistics for the ROS2 bridge connection including latency, packet loss, ' +
        'bytes transferred, uptime, and overall connection quality rating.',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_network_bandwidth',
      description:
        'Get bandwidth usage per topic showing message counts, bytes per second, and average message size. ' +
        'Optionally filter to a specific topic.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().optional().describe('Filter to a specific topic (e.g. "/cmd_vel")'),
      })),
    },
    {
      name: 'ros2_network_latency_test',
      description:
        'Run a latency test by pinging the ROS2 bridge multiple times. ' +
        'Returns min, max, average, and standard deviation of round-trip times.',
      inputSchema: toInputSchema(z.object({
        count: z.number().default(5).describe('Number of pings to send (default: 5, max: 20)'),
      })),
    },
  ];
}

/**
 * Determine connection quality label from latency in milliseconds.
 */
function getConnectionQuality(latencyMs: number): string {
  if (latencyMs < 20) return 'excellent';
  if (latencyMs < 50) return 'good';
  if (latencyMs < 100) return 'fair';
  return 'poor';
}

export async function handleNetworkTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_network_stats': {
      try {
        const response = await connection.send('system.network_stats' as any);
        if (response.status === 'error') {
          return {
            content: [{ type: 'text', text: `Network stats error: ${JSON.stringify(response.data)}` }],
            isError: true,
          };
        }
        const data = response.data as Record<string, unknown>;
        const latency = (data.latency_ms as number) ?? 0;
        const result = {
          ...data,
          connection_quality: getConnectionQuality(latency),
        };
        return {
          content: [{ type: 'text', text: `Network statistics:\n${JSON.stringify(result, null, 2)}` }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Network stats failed: ${message}` }],
          isError: true,
        };
      }
    }

    case 'ros2_network_bandwidth': {
      const topic = args.topic as string | undefined;

      try {
        const params: Record<string, unknown> = {};
        if (topic) {
          params.topic = topic;
        }
        const response = await connection.send('system.bandwidth' as any, params);
        if (response.status === 'error') {
          return {
            content: [{ type: 'text', text: `Bandwidth query error: ${JSON.stringify(response.data)}` }],
            isError: true,
          };
        }
        const label = topic ? `Bandwidth usage for "${topic}"` : 'Bandwidth usage (all topics)';
        return {
          content: [{ type: 'text', text: `${label}:\n${JSON.stringify(response.data, null, 2)}` }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Bandwidth query failed: ${message}` }],
          isError: true,
        };
      }
    }

    case 'ros2_network_latency_test': {
      let count = (args.count as number) || 5;
      if (count > 20) count = 20;
      if (count < 1) count = 1;

      const results: { seq: number; latency_ms: number; success: boolean }[] = [];

      try {
        for (let i = 0; i < count; i++) {
          const startTime = Date.now();
          try {
            const response = await connection.send('system.ping' as any);
            const endTime = Date.now();
            const latency = endTime - startTime;
            if (response.status === 'error') {
              results.push({ seq: i + 1, latency_ms: -1, success: false });
            } else {
              results.push({ seq: i + 1, latency_ms: latency, success: true });
            }
          } catch {
            results.push({ seq: i + 1, latency_ms: -1, success: false });
          }
        }

        const successfulResults = results.filter(r => r.success);

        if (successfulResults.length === 0) {
          return {
            content: [{
              type: 'text',
              text: `Latency test failed: all ${count} pings failed\n${JSON.stringify({ results }, null, 2)}`,
            }],
            isError: true,
          };
        }

        const latencies = successfulResults.map(r => r.latency_ms);
        const minMs = Math.min(...latencies);
        const maxMs = Math.max(...latencies);
        const avgMs = latencies.reduce((sum, v) => sum + v, 0) / latencies.length;
        const variance = latencies.reduce((sum, v) => sum + (v - avgMs) ** 2, 0) / latencies.length;
        const stddevMs = Math.sqrt(variance);

        const summary = {
          pings_sent: count,
          pings_successful: successfulResults.length,
          pings_failed: count - successfulResults.length,
          min_ms: Math.round(minMs * 100) / 100,
          max_ms: Math.round(maxMs * 100) / 100,
          avg_ms: Math.round(avgMs * 100) / 100,
          stddev_ms: Math.round(stddevMs * 100) / 100,
          results,
        };

        return {
          content: [{ type: 'text', text: `Latency test results:\n${JSON.stringify(summary, null, 2)}` }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Latency test failed: ${message}` }],
          isError: true,
        };
      }
    }

    default:
      return { content: [{ type: 'text', text: `Unknown network tool: ${name}` }], isError: true };
  }
}

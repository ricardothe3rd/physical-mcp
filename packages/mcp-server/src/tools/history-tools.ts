/**
 * Command history/replay MCP tools: track, query, and replay past commands.
 *
 * All history is stored in-memory (module-level array). No bridge connection
 * is required since this operates entirely within the MCP server process.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

// ---------------------------------------------------------------------------
// History entry type & in-memory storage
// ---------------------------------------------------------------------------

export interface HistoryEntry {
  id: number;
  timestamp: number;
  tool: string;
  args: Record<string, unknown>;
  result: 'success' | 'error';
  duration: number;
  safetyBlocked: boolean;
}

const MAX_ENTRIES = 1000;
let history: HistoryEntry[] = [];
let nextId = 1;

// ---------------------------------------------------------------------------
// Public helpers
// ---------------------------------------------------------------------------

export function recordCommand(
  entry: Omit<HistoryEntry, 'id' | 'timestamp'>,
): HistoryEntry {
  const record: HistoryEntry = {
    id: nextId++,
    timestamp: Date.now(),
    ...entry,
  };
  history.push(record);
  if (history.length > MAX_ENTRIES) {
    history = history.slice(history.length - MAX_ENTRIES);
  }
  return record;
}

export function resetHistory(): void {
  history = [];
  nextId = 1;
}

// ---------------------------------------------------------------------------
// Tool definitions
// ---------------------------------------------------------------------------

export function getHistoryTools(): Tool[] {
  return [
    {
      name: 'ros2_command_history',
      description:
        'List recent command history. Returns entries in reverse chronological order ' +
        'with optional filters by tool name prefix and error-only mode. ' +
        'Includes a summary of total, success, error, and safety-blocked counts.',
      inputSchema: toInputSchema(
        z.object({
          limit: z
            .number()
            .default(20)
            .describe('Maximum number of entries to return (default: 20)'),
          toolFilter: z
            .string()
            .optional()
            .describe('Filter entries whose tool name starts with this prefix'),
          errorsOnly: z
            .boolean()
            .optional()
            .describe('When true, only return entries with result "error"'),
        }),
      ),
    },
    {
      name: 'ros2_command_stats',
      description:
        'Get command usage statistics: per-tool call counts, success rates, ' +
        'average durations, most common errors, total commands, time range, ' +
        'and safety block rate.',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_command_replay',
      description:
        'Look up a previously executed command by its history ID. ' +
        'Does NOT re-execute the command — returns the command details ' +
        '(tool name and arguments) so the user can review and optionally ' +
        'ask to re-execute it.',
      inputSchema: toInputSchema(
        z.object({
          commandId: z.number().describe('The history entry ID to look up'),
        }),
      ),
    },
  ];
}

// ---------------------------------------------------------------------------
// Handler
// ---------------------------------------------------------------------------

export async function handleHistoryTool(
  name: string,
  args: Record<string, unknown>,
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    // -----------------------------------------------------------------------
    case 'ros2_command_history': {
      const limit = (args.limit as number | undefined) ?? 20;
      const toolFilter = args.toolFilter as string | undefined;
      const errorsOnly = args.errorsOnly as boolean | undefined;

      let filtered = [...history];

      if (toolFilter) {
        filtered = filtered.filter((e) => e.tool.startsWith(toolFilter));
      }
      if (errorsOnly) {
        filtered = filtered.filter((e) => e.result === 'error');
      }

      // Most recent first
      filtered.reverse();
      const entries = filtered.slice(0, limit);

      const totalCommands = filtered.length;
      const successCount = filtered.filter((e) => e.result === 'success').length;
      const errorCount = filtered.filter((e) => e.result === 'error').length;
      const safetyBlockedCount = filtered.filter((e) => e.safetyBlocked).length;

      const payload = {
        summary: {
          totalCommands,
          successCount,
          errorCount,
          safetyBlockedCount,
        },
        entries,
      };

      return {
        content: [
          { type: 'text', text: JSON.stringify(payload, null, 2) },
        ],
      };
    }

    // -----------------------------------------------------------------------
    case 'ros2_command_stats': {
      if (history.length === 0) {
        const empty = {
          totalCommands: 0,
          timeRange: null,
          safetyBlockRate: 0,
          perTool: {},
        };
        return {
          content: [{ type: 'text', text: JSON.stringify(empty, null, 2) }],
        };
      }

      const perTool: Record<
        string,
        {
          calls: number;
          successes: number;
          errors: number;
          totalDuration: number;
          averageDuration: number;
          successRate: number;
        }
      > = {};

      for (const entry of history) {
        if (!perTool[entry.tool]) {
          perTool[entry.tool] = {
            calls: 0,
            successes: 0,
            errors: 0,
            totalDuration: 0,
            averageDuration: 0,
            successRate: 0,
          };
        }
        const t = perTool[entry.tool];
        t.calls++;
        if (entry.result === 'success') t.successes++;
        else t.errors++;
        t.totalDuration += entry.duration;
      }

      for (const tool of Object.keys(perTool)) {
        const t = perTool[tool];
        t.averageDuration = Math.round(t.totalDuration / t.calls);
        t.successRate = Number(((t.successes / t.calls) * 100).toFixed(1));
      }

      const totalCommands = history.length;
      const safetyBlocked = history.filter((e) => e.safetyBlocked).length;
      const timestamps = history.map((e) => e.timestamp);
      const timeRange = {
        from: Math.min(...timestamps),
        to: Math.max(...timestamps),
      };
      const safetyBlockRate = Number(
        ((safetyBlocked / totalCommands) * 100).toFixed(1),
      );

      // Most common errors: per-tool error counts, sorted descending
      const errorTools = Object.entries(perTool)
        .filter(([, v]) => v.errors > 0)
        .sort((a, b) => b[1].errors - a[1].errors)
        .map(([tool, v]) => ({ tool, errorCount: v.errors }));

      const payload = {
        totalCommands,
        timeRange,
        safetyBlockRate,
        mostCommonErrors: errorTools,
        perTool,
      };

      return {
        content: [{ type: 'text', text: JSON.stringify(payload, null, 2) }],
      };
    }

    // -----------------------------------------------------------------------
    case 'ros2_command_replay': {
      const commandId = args.commandId as number;
      const entry = history.find((e) => e.id === commandId);

      if (!entry) {
        return {
          content: [
            {
              type: 'text',
              text: `Error: Command with ID ${commandId} not found in history.`,
            },
          ],
          isError: true,
        };
      }

      const payload = {
        id: entry.id,
        timestamp: entry.timestamp,
        tool: entry.tool,
        args: entry.args,
        result: entry.result,
        duration: entry.duration,
        safetyBlocked: entry.safetyBlocked,
        note: 'This command was NOT re-executed. Review the details and ask to re-execute if desired.',
      };

      return {
        content: [{ type: 'text', text: JSON.stringify(payload, null, 2) }],
      };
    }

    // -----------------------------------------------------------------------
    default:
      return {
        content: [{ type: 'text', text: `Unknown history tool: ${name}` }],
        isError: true,
      };
  }
}

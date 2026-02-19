import { describe, it, expect, beforeEach } from 'vitest';
import {
  recordCommand,
  resetHistory,
  getHistoryTools,
  handleHistoryTool,
} from './history-tools.js';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function makeEntry(overrides: {
  tool?: string;
  args?: Record<string, unknown>;
  result?: 'success' | 'error';
  duration?: number;
  safetyBlocked?: boolean;
} = {}) {
  return {
    tool: overrides.tool ?? 'ros2_topic_echo',
    args: overrides.args ?? { topic: '/cmd_vel' },
    result: overrides.result ?? 'success' as const,
    duration: overrides.duration ?? 120,
    safetyBlocked: overrides.safetyBlocked ?? false,
  };
}

function parseResult(res: { content: { type: string; text: string }[] }) {
  return JSON.parse(res.content[0].text);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('history-tools', () => {
  beforeEach(() => {
    resetHistory();
  });

  // -----------------------------------------------------------------------
  // recordCommand
  // -----------------------------------------------------------------------

  describe('recordCommand', () => {
    it('stores entries with auto-incremented ids', () => {
      const a = recordCommand(makeEntry());
      const b = recordCommand(makeEntry());
      expect(a.id).toBe(1);
      expect(b.id).toBe(2);
      expect(a.timestamp).toBeLessThanOrEqual(b.timestamp);
    });

    it('limits history to 1000 entries by dropping oldest', () => {
      for (let i = 0; i < 1005; i++) {
        recordCommand(makeEntry({ tool: `tool_${i}` }));
      }
      // After recording 1005 entries, we should have exactly 1000.
      // The first 5 (tool_0..tool_4) should have been dropped.
      const res = handleHistoryTool('ros2_command_history', { limit: 1100 });
      return res.then((r) => {
        const data = parseResult(r);
        expect(data.entries.length).toBe(1000);
        // Entries are returned most-recent-first; the last one returned
        // should be the oldest kept entry (tool_5).
        const oldest = data.entries[data.entries.length - 1];
        expect(oldest.tool).toBe('tool_5');
      });
    });
  });

  // -----------------------------------------------------------------------
  // ros2_command_history
  // -----------------------------------------------------------------------

  describe('ros2_command_history', () => {
    it('returns empty when no history exists', async () => {
      const res = await handleHistoryTool('ros2_command_history', {});
      const data = parseResult(res);
      expect(data.entries).toEqual([]);
      expect(data.summary.totalCommands).toBe(0);
    });

    it('returns entries in reverse chronological order', async () => {
      recordCommand(makeEntry({ tool: 'first' }));
      recordCommand(makeEntry({ tool: 'second' }));
      recordCommand(makeEntry({ tool: 'third' }));

      const res = await handleHistoryTool('ros2_command_history', {});
      const data = parseResult(res);

      expect(data.entries[0].tool).toBe('third');
      expect(data.entries[1].tool).toBe('second');
      expect(data.entries[2].tool).toBe('first');
    });

    it('respects the limit parameter', async () => {
      for (let i = 0; i < 10; i++) {
        recordCommand(makeEntry({ tool: `tool_${i}` }));
      }

      const res = await handleHistoryTool('ros2_command_history', { limit: 3 });
      const data = parseResult(res);
      expect(data.entries.length).toBe(3);
      // Most recent 3: tool_9, tool_8, tool_7
      expect(data.entries[0].tool).toBe('tool_9');
      expect(data.entries[2].tool).toBe('tool_7');
    });

    it('filters by tool name prefix with toolFilter', async () => {
      recordCommand(makeEntry({ tool: 'ros2_topic_echo' }));
      recordCommand(makeEntry({ tool: 'ros2_topic_list' }));
      recordCommand(makeEntry({ tool: 'ros2_service_call' }));
      recordCommand(makeEntry({ tool: 'ros2_topic_pub' }));

      const res = await handleHistoryTool('ros2_command_history', {
        toolFilter: 'ros2_topic',
      });
      const data = parseResult(res);
      expect(data.entries.length).toBe(3);
      expect(data.entries.every((e: { tool: string }) => e.tool.startsWith('ros2_topic'))).toBe(true);
    });

    it('filters to errors only with errorsOnly', async () => {
      recordCommand(makeEntry({ result: 'success' }));
      recordCommand(makeEntry({ result: 'error' }));
      recordCommand(makeEntry({ result: 'success' }));
      recordCommand(makeEntry({ result: 'error' }));

      const res = await handleHistoryTool('ros2_command_history', {
        errorsOnly: true,
      });
      const data = parseResult(res);
      expect(data.entries.length).toBe(2);
      expect(data.entries.every((e: { result: string }) => e.result === 'error')).toBe(true);
    });

    it('combines toolFilter and errorsOnly', async () => {
      recordCommand(makeEntry({ tool: 'ros2_topic_echo', result: 'success' }));
      recordCommand(makeEntry({ tool: 'ros2_topic_echo', result: 'error' }));
      recordCommand(makeEntry({ tool: 'ros2_service_call', result: 'error' }));

      const res = await handleHistoryTool('ros2_command_history', {
        toolFilter: 'ros2_topic',
        errorsOnly: true,
      });
      const data = parseResult(res);
      expect(data.entries.length).toBe(1);
      expect(data.entries[0].tool).toBe('ros2_topic_echo');
      expect(data.entries[0].result).toBe('error');
    });

    it('includes correct summary counts', async () => {
      recordCommand(makeEntry({ result: 'success', safetyBlocked: false }));
      recordCommand(makeEntry({ result: 'error', safetyBlocked: false }));
      recordCommand(makeEntry({ result: 'error', safetyBlocked: true }));

      const res = await handleHistoryTool('ros2_command_history', {});
      const data = parseResult(res);
      expect(data.summary.totalCommands).toBe(3);
      expect(data.summary.successCount).toBe(1);
      expect(data.summary.errorCount).toBe(2);
      expect(data.summary.safetyBlockedCount).toBe(1);
    });
  });

  // -----------------------------------------------------------------------
  // ros2_command_stats
  // -----------------------------------------------------------------------

  describe('ros2_command_stats', () => {
    it('returns zeros when no history exists', async () => {
      const res = await handleHistoryTool('ros2_command_stats', {});
      const data = parseResult(res);
      expect(data.totalCommands).toBe(0);
      expect(data.timeRange).toBeNull();
      expect(data.safetyBlockRate).toBe(0);
      expect(data.perTool).toEqual({});
    });

    it('computes per-tool stats with mixed success/error', async () => {
      recordCommand(makeEntry({ tool: 'ros2_topic_echo', result: 'success', duration: 100 }));
      recordCommand(makeEntry({ tool: 'ros2_topic_echo', result: 'success', duration: 200 }));
      recordCommand(makeEntry({ tool: 'ros2_topic_echo', result: 'error', duration: 50 }));
      recordCommand(makeEntry({ tool: 'ros2_service_call', result: 'success', duration: 300 }));
      recordCommand(makeEntry({ tool: 'ros2_service_call', result: 'error', duration: 100, safetyBlocked: true }));

      const res = await handleHistoryTool('ros2_command_stats', {});
      const data = parseResult(res);

      expect(data.totalCommands).toBe(5);
      expect(data.safetyBlockRate).toBe(20);
      expect(data.timeRange).not.toBeNull();
      expect(data.timeRange.from).toBeLessThanOrEqual(data.timeRange.to);

      // per-tool checks
      const echo = data.perTool['ros2_topic_echo'];
      expect(echo.calls).toBe(3);
      expect(echo.successes).toBe(2);
      expect(echo.errors).toBe(1);
      expect(echo.averageDuration).toBe(117); // Math.round((100+200+50)/3) = 117
      expect(echo.successRate).toBe(66.7);

      const svc = data.perTool['ros2_service_call'];
      expect(svc.calls).toBe(2);
      expect(svc.successes).toBe(1);
      expect(svc.errors).toBe(1);
      expect(svc.averageDuration).toBe(200);
      expect(svc.successRate).toBe(50);

      // most common errors
      expect(data.mostCommonErrors.length).toBe(2);
      // Both have 1 error each; order depends on insertion but both should be present
      const errorTools = data.mostCommonErrors.map((e: { tool: string }) => e.tool);
      expect(errorTools).toContain('ros2_topic_echo');
      expect(errorTools).toContain('ros2_service_call');
    });
  });

  // -----------------------------------------------------------------------
  // ros2_command_replay
  // -----------------------------------------------------------------------

  describe('ros2_command_replay', () => {
    it('returns command details for a valid ID', async () => {
      const entry = recordCommand(
        makeEntry({ tool: 'ros2_topic_pub', args: { topic: '/cmd_vel', value: '{}' } }),
      );

      const res = await handleHistoryTool('ros2_command_replay', {
        commandId: entry.id,
      });
      expect(res.isError).toBeUndefined();

      const data = parseResult(res);
      expect(data.id).toBe(entry.id);
      expect(data.tool).toBe('ros2_topic_pub');
      expect(data.args.topic).toBe('/cmd_vel');
      expect(data.note).toContain('NOT re-executed');
    });

    it('returns error for an invalid ID', async () => {
      recordCommand(makeEntry());

      const res = await handleHistoryTool('ros2_command_replay', {
        commandId: 9999,
      });
      expect(res.isError).toBe(true);
      expect(res.content[0].text).toContain('9999');
      expect(res.content[0].text).toContain('not found');
    });
  });

  // -----------------------------------------------------------------------
  // resetHistory
  // -----------------------------------------------------------------------

  describe('resetHistory', () => {
    it('clears all history and resets ID counter', async () => {
      recordCommand(makeEntry());
      recordCommand(makeEntry());

      resetHistory();

      const res = await handleHistoryTool('ros2_command_history', {});
      const data = parseResult(res);
      expect(data.entries.length).toBe(0);

      // After reset, IDs should start from 1 again
      const fresh = recordCommand(makeEntry());
      expect(fresh.id).toBe(1);
    });
  });

  // -----------------------------------------------------------------------
  // Unknown tool
  // -----------------------------------------------------------------------

  describe('unknown tool', () => {
    it('returns error for unrecognised tool name', async () => {
      const res = await handleHistoryTool('ros2_nonexistent_tool', {});
      expect(res.isError).toBe(true);
      expect(res.content[0].text).toContain('Unknown history tool');
    });
  });

  // -----------------------------------------------------------------------
  // getHistoryTools
  // -----------------------------------------------------------------------

  describe('getHistoryTools', () => {
    it('returns three tool definitions', () => {
      const tools = getHistoryTools();
      expect(tools.length).toBe(3);
      const names = tools.map((t) => t.name);
      expect(names).toContain('ros2_command_history');
      expect(names).toContain('ros2_command_stats');
      expect(names).toContain('ros2_command_replay');
    });

    it('each tool has a name, description, and inputSchema', () => {
      const tools = getHistoryTools();
      for (const tool of tools) {
        expect(tool.name).toBeTruthy();
        expect(tool.description).toBeTruthy();
        expect(tool.inputSchema).toBeTruthy();
      }
    });
  });
});

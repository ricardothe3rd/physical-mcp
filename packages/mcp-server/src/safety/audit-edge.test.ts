/**
 * Edge case tests for the AuditLogger.
 *
 * Covers memory cap behavior, concurrent writes, JSON export format,
 * and entry format consistency.
 */

import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest';
import { writeFileSync } from 'fs';
import { AuditLogger } from './audit-logger.js';
import type { SafetyCheckResult } from './types.js';

// Mock fs.writeFileSync to capture output without writing to disk
vi.mock('fs', async () => {
  const actual = await vi.importActual<typeof import('fs')>('fs');
  return {
    ...actual,
    writeFileSync: vi.fn(),
  };
});

describe('AuditLogger Edge Cases', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  describe('memory cap behavior (>10000 entries)', () => {
    it('respects the default 10000 entry cap', () => {
      const logger = new AuditLogger(); // default maxEntries = 10000

      for (let i = 0; i < 10050; i++) {
        logger.log('publish', `/topic_${i}`, { i }, { allowed: true, violations: [] });
      }

      const entries = logger.getEntries();
      expect(entries.length).toBe(10000);

      const stats = logger.getStats();
      expect(stats.total).toBe(10000);
    });

    it('keeps the newest entries when cap is exceeded', () => {
      const logger = new AuditLogger(100);

      for (let i = 0; i < 200; i++) {
        logger.log('publish', `/topic_${i}`, { i }, { allowed: true, violations: [] });
      }

      const entries = logger.getEntries();
      expect(entries.length).toBe(100);

      // Newest first (reversed), so first entry should be topic_199
      expect(entries[0].target).toBe('/topic_199');
      // Oldest kept entry should be topic_100
      expect(entries[99].target).toBe('/topic_100');
    });

    it('eviction happens on every insert above cap, not in batches', () => {
      const logger = new AuditLogger(5);

      // Fill to cap
      for (let i = 0; i < 5; i++) {
        logger.log('publish', `/topic_${i}`, {}, { allowed: true, violations: [] });
      }
      expect(logger.getEntries().length).toBe(5);

      // Add one more — should evict oldest
      logger.log('publish', '/topic_5', {}, { allowed: true, violations: [] });
      const entries = logger.getEntries();
      expect(entries.length).toBe(5);
      // topic_0 should be gone, topic_1 should be the oldest
      expect(entries[entries.length - 1].target).toBe('/topic_1');
      expect(entries[0].target).toBe('/topic_5');
    });

    it('stats remain accurate after eviction', () => {
      const logger = new AuditLogger(10);

      // Log 5 blocked, then 10 allowed
      for (let i = 0; i < 5; i++) {
        logger.log('publish', `/blocked_${i}`, {}, {
          allowed: false,
          violations: [{ type: 'velocity_exceeded', message: 'too fast', details: {}, timestamp: Date.now() }],
        });
      }
      for (let i = 0; i < 10; i++) {
        logger.log('publish', `/allowed_${i}`, {}, { allowed: true, violations: [] });
      }

      // Cap is 10, so the 5 blocked entries should be evicted
      const stats = logger.getStats();
      expect(stats.total).toBe(10);
      expect(stats.blocked).toBe(0);
      expect(stats.allowed).toBe(10);
    });

    it('cap of 1 keeps only the most recent entry', () => {
      const logger = new AuditLogger(1);

      logger.log('publish', '/first', {}, { allowed: true, violations: [] });
      logger.log('service_call', '/second', {}, { allowed: false, violations: [
        { type: 'blocked_service', message: 'blocked', details: {}, timestamp: Date.now() },
      ] });

      const entries = logger.getEntries();
      expect(entries.length).toBe(1);
      expect(entries[0].target).toBe('/second');
      expect(entries[0].command).toBe('service_call');
    });

    it('very large cap accepts many entries without issue', () => {
      const logger = new AuditLogger(50000);

      for (let i = 0; i < 1000; i++) {
        logger.log('publish', `/topic_${i}`, { i }, { allowed: true, violations: [] });
      }

      expect(logger.getEntries().length).toBe(1000);
      expect(logger.getStats().total).toBe(1000);
    });
  });

  describe('concurrent writes', () => {
    it('rapid sequential writes maintain correct order', () => {
      const logger = new AuditLogger();

      for (let i = 0; i < 100; i++) {
        logger.log('publish', `/topic_${i}`, { seq: i }, { allowed: true, violations: [] });
      }

      const entries = logger.getEntries();
      // Entries are reversed (newest first)
      for (let i = 0; i < 100; i++) {
        expect(entries[i].target).toBe(`/topic_${99 - i}`);
        expect((entries[i].params as { seq: number }).seq).toBe(99 - i);
      }
    });

    it('interleaved command types maintain correct count', () => {
      const logger = new AuditLogger();

      for (let i = 0; i < 100; i++) {
        const command = i % 3 === 0 ? 'publish' : i % 3 === 1 ? 'service_call' : 'action_goal';
        const allowed = i % 4 !== 0;
        logger.log(command, `/target_${i}`, { i }, {
          allowed,
          violations: allowed ? [] : [
            { type: 'rate_limit_exceeded', message: 'limit hit', details: {}, timestamp: Date.now() },
          ],
        });
      }

      const stats = logger.getStats();
      expect(stats.total).toBe(100);
      expect(stats.blocked).toBe(25); // every 4th

      // Filter by command type
      const publishes = logger.getEntries({ command: 'publish' });
      expect(publishes.length).toBe(34); // ceil(100/3)

      const services = logger.getEntries({ command: 'service_call' });
      expect(services.length).toBe(33);

      const actions = logger.getEntries({ command: 'action_goal' });
      expect(actions.length).toBe(33);
    });

    it('updateResult calls during rapid writes update the correct entry', () => {
      const logger = new AuditLogger();

      const entryIds: string[] = [];
      for (let i = 0; i < 50; i++) {
        const entry = logger.log('publish', `/topic_${i}`, {}, { allowed: true, violations: [] });
        entryIds.push(entry.id);
      }

      // Update every other entry
      for (let i = 0; i < 50; i += 2) {
        logger.updateResult(entryIds[i], 'success');
      }
      for (let i = 1; i < 50; i += 2) {
        logger.updateResult(entryIds[i], 'error', `error_${i}`);
      }

      const entries = logger.getEntries();
      for (const entry of entries) {
        const idx = parseInt(entry.target.replace('/topic_', ''));
        if (idx % 2 === 0) {
          expect(entry.executionResult).toBe('success');
          expect(entry.executionError).toBeUndefined();
        } else {
          expect(entry.executionResult).toBe('error');
          expect(entry.executionError).toBe(`error_${idx}`);
        }
      }
    });

    it('concurrent-style Promise.all writes all get recorded', async () => {
      const logger = new AuditLogger();

      const writes = Array.from({ length: 50 }, (_, i) =>
        Promise.resolve().then(() =>
          logger.log('publish', `/async_${i}`, {}, { allowed: true, violations: [] })
        )
      );

      await Promise.all(writes);

      expect(logger.getEntries().length).toBe(50);
      expect(logger.getStats().total).toBe(50);
    });

    it('clear during writes resets cleanly', () => {
      const logger = new AuditLogger();

      for (let i = 0; i < 50; i++) {
        logger.log('publish', `/topic_${i}`, {}, { allowed: true, violations: [] });
      }

      logger.clear();

      // Log more after clear
      for (let i = 0; i < 10; i++) {
        logger.log('service_call', `/svc_${i}`, {}, { allowed: true, violations: [] });
      }

      const entries = logger.getEntries();
      expect(entries.length).toBe(10);
      expect(entries.every(e => e.command === 'service_call')).toBe(true);
    });
  });

  describe('export to JSON format', () => {
    it('exports all entries to JSON via writeFileSync', () => {
      const logger = new AuditLogger();

      logger.log('publish', '/cmd_vel', { linear: { x: 0.1 } }, { allowed: true, violations: [] });
      logger.log('service_call', '/svc', {}, {
        allowed: false,
        violations: [{ type: 'blocked_service', message: 'blocked', details: {}, timestamp: Date.now() }],
      });

      const count = logger.exportToFile('/tmp/audit.json');
      expect(count).toBe(2);

      expect(writeFileSync).toHaveBeenCalledTimes(1);
      expect(writeFileSync).toHaveBeenCalledWith(
        '/tmp/audit.json',
        expect.any(String)
      );

      // Parse the exported JSON to verify format
      const exportedJson = (writeFileSync as ReturnType<typeof vi.fn>).mock.calls[0][1] as string;
      const data = JSON.parse(exportedJson);

      expect(data.exportedAt).toBeDefined();
      expect(typeof data.exportedAt).toBe('string');
      expect(data.stats).toBeDefined();
      expect(data.stats.total).toBe(2);
      expect(data.stats.blocked).toBe(1);
      expect(data.entries).toBeInstanceOf(Array);
      expect(data.entries.length).toBe(2);
    });

    it('exports with violationsOnly filter', () => {
      const logger = new AuditLogger();

      logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.log('publish', '/rosout', {}, {
        allowed: false,
        violations: [{ type: 'blocked_topic', message: 'blocked', details: {}, timestamp: Date.now() }],
      });
      logger.log('publish', '/odom', {}, { allowed: true, violations: [] });

      const count = logger.exportToFile('/tmp/violations.json', { violationsOnly: true });
      expect(count).toBe(1);

      const exportedJson = (writeFileSync as ReturnType<typeof vi.fn>).mock.calls[0][1] as string;
      const data = JSON.parse(exportedJson);
      expect(data.entries.length).toBe(1);
      expect(data.entries[0].target).toBe('/rosout');
    });

    it('exports with command filter', () => {
      const logger = new AuditLogger();

      logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.log('service_call', '/svc', {}, { allowed: true, violations: [] });
      logger.log('action_goal', '/nav', {}, { allowed: true, violations: [] });

      const count = logger.exportToFile('/tmp/services.json', { command: 'service_call' });
      expect(count).toBe(1);

      const exportedJson = (writeFileSync as ReturnType<typeof vi.fn>).mock.calls[0][1] as string;
      const data = JSON.parse(exportedJson);
      expect(data.entries.length).toBe(1);
      expect(data.entries[0].command).toBe('service_call');
    });

    it('exported JSON is properly formatted (indented)', () => {
      const logger = new AuditLogger();

      logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.exportToFile('/tmp/formatted.json');

      const exportedJson = (writeFileSync as ReturnType<typeof vi.fn>).mock.calls[0][1] as string;
      // JSON.stringify with indent 2 produces lines starting with spaces
      expect(exportedJson).toContain('\n');
      expect(exportedJson).toContain('  ');
    });

    it('export of empty log produces valid JSON', () => {
      const logger = new AuditLogger();

      const count = logger.exportToFile('/tmp/empty.json');
      expect(count).toBe(0);

      const exportedJson = (writeFileSync as ReturnType<typeof vi.fn>).mock.calls[0][1] as string;
      const data = JSON.parse(exportedJson);
      expect(data.exportedAt).toBeDefined();
      expect(data.stats.total).toBe(0);
      expect(data.entries).toEqual([]);
    });

    it('export includes stats that reflect the full log, not just filtered entries', () => {
      const logger = new AuditLogger();

      logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.log('publish', '/rosout', {}, {
        allowed: false,
        violations: [{ type: 'blocked_topic', message: 'blocked', details: {}, timestamp: Date.now() }],
      });
      logger.log('service_call', '/svc', {}, { allowed: true, violations: [] });

      logger.exportToFile('/tmp/filtered.json', { violationsOnly: true });

      const exportedJson = (writeFileSync as ReturnType<typeof vi.fn>).mock.calls[0][1] as string;
      const data = JSON.parse(exportedJson);

      // Stats should reflect the full log
      expect(data.stats.total).toBe(3);
      expect(data.stats.allowed).toBe(2);
      expect(data.stats.blocked).toBe(1);

      // But entries should be filtered
      expect(data.entries.length).toBe(1);
    });
  });

  describe('entry format consistency', () => {
    it('every entry has a unique UUID id', () => {
      const logger = new AuditLogger();

      const ids = new Set<string>();
      for (let i = 0; i < 100; i++) {
        const entry = logger.log('publish', `/topic_${i}`, {}, { allowed: true, violations: [] });
        expect(entry.id).toBeDefined();
        expect(typeof entry.id).toBe('string');
        expect(entry.id.length).toBeGreaterThan(0);
        ids.add(entry.id);
      }

      // All IDs should be unique
      expect(ids.size).toBe(100);
    });

    it('every entry has a valid timestamp', () => {
      const logger = new AuditLogger();
      const before = Date.now();

      for (let i = 0; i < 10; i++) {
        logger.log('publish', `/topic_${i}`, {}, { allowed: true, violations: [] });
      }

      const after = Date.now();
      const entries = logger.getEntries();

      for (const entry of entries) {
        expect(entry.timestamp).toBeGreaterThanOrEqual(before);
        expect(entry.timestamp).toBeLessThanOrEqual(after);
      }
    });

    it('entry preserves command, target, and params exactly', () => {
      const logger = new AuditLogger();

      const params = { linear: { x: 0.5, y: 0.3 }, metadata: { source: 'test' } };
      const entry = logger.log('publish', '/cmd_vel', params, {
        allowed: true,
        violations: [],
      });

      expect(entry.command).toBe('publish');
      expect(entry.target).toBe('/cmd_vel');
      expect(entry.params).toEqual(params);
    });

    it('entry preserves safety result with violations', () => {
      const logger = new AuditLogger();

      const safetyResult: SafetyCheckResult = {
        allowed: false,
        violations: [
          {
            type: 'velocity_exceeded',
            message: 'Linear velocity 5.00 m/s exceeds limit of 0.50 m/s',
            details: { linear: { x: 5.0 }, limit: 0.5, actual: 5.0 },
            timestamp: Date.now(),
          },
          {
            type: 'rate_limit_exceeded',
            message: 'Rate limit exceeded for publish:/cmd_vel',
            details: { key: 'publish:/cmd_vel', count: 11, limit: 10, windowMs: 1000 },
            timestamp: Date.now(),
          },
        ],
      };

      const entry = logger.log('publish', '/cmd_vel', {}, safetyResult);

      expect(entry.safetyResult.allowed).toBe(false);
      expect(entry.safetyResult.violations).toHaveLength(2);
      expect(entry.safetyResult.violations[0].type).toBe('velocity_exceeded');
      expect(entry.safetyResult.violations[1].type).toBe('rate_limit_exceeded');
    });

    it('entry executionResult and executionError are undefined by default', () => {
      const logger = new AuditLogger();

      const entry = logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });

      expect(entry.executionResult).toBeUndefined();
      expect(entry.executionError).toBeUndefined();
    });

    it('updateResult with nonexistent ID does not throw', () => {
      const logger = new AuditLogger();

      // Should not throw even with an invalid ID
      expect(() => {
        logger.updateResult('nonexistent-uuid', 'error', 'should not crash');
      }).not.toThrow();
    });

    it('getEntriesByTimeRange returns entries within the range', () => {
      const logger = new AuditLogger();

      const t1 = Date.now();
      logger.log('publish', '/early', {}, { allowed: true, violations: [] });

      const t2 = Date.now();
      logger.log('publish', '/middle', {}, { allowed: true, violations: [] });

      const t3 = Date.now();
      logger.log('publish', '/late', {}, { allowed: true, violations: [] });

      const t4 = Date.now();

      // Get all entries (t1 to t4 inclusive)
      const all = logger.getEntriesByTimeRange(t1, t4);
      expect(all.length).toBe(3);

      // Entries are in insertion order (not reversed)
      expect(all[0].target).toBe('/early');
      expect(all[2].target).toBe('/late');
    });

    it('getEntriesByTimeRange with narrow window may return subset', () => {
      vi.useFakeTimers();

      const logger = new AuditLogger();

      logger.log('publish', '/t0', {}, { allowed: true, violations: [] });
      vi.advanceTimersByTime(1000);

      const midStart = Date.now();
      logger.log('publish', '/t1000', {}, { allowed: true, violations: [] });
      const midEnd = Date.now();

      vi.advanceTimersByTime(1000);
      logger.log('publish', '/t2000', {}, { allowed: true, violations: [] });

      const midEntries = logger.getEntriesByTimeRange(midStart, midEnd);
      expect(midEntries.length).toBe(1);
      expect(midEntries[0].target).toBe('/t1000');

      vi.useRealTimers();
    });

    it('entries from different command types all follow the same schema', () => {
      const logger = new AuditLogger();

      const pub = logger.log('publish', '/cmd_vel', { data: 1 }, { allowed: true, violations: [] });
      const svc = logger.log('service_call', '/set_param', { param: 'a' }, {
        allowed: false,
        violations: [{ type: 'blocked_service', message: 'blocked', details: {}, timestamp: Date.now() }],
      });
      const act = logger.log('action_goal', '/navigate', { goal: { x: 1 } }, { allowed: true, violations: [] });

      for (const entry of [pub, svc, act]) {
        // All entries must have the required fields
        expect(typeof entry.id).toBe('string');
        expect(typeof entry.timestamp).toBe('number');
        expect(typeof entry.command).toBe('string');
        expect(typeof entry.target).toBe('string');
        expect(typeof entry.params).toBe('object');
        expect(typeof entry.safetyResult).toBe('object');
        expect(typeof entry.safetyResult.allowed).toBe('boolean');
        expect(Array.isArray(entry.safetyResult.violations)).toBe(true);
      }
    });
  });
});

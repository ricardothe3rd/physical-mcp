import { describe, it, expect, beforeEach } from 'vitest';
import { AuditLogger } from './audit-logger.js';

describe('AuditLogger', () => {
  let logger: AuditLogger;

  beforeEach(() => {
    logger = new AuditLogger();
  });

  describe('log', () => {
    it('logs an allowed command', () => {
      const entry = logger.log('publish', '/cmd_vel', { data: 'test' }, {
        allowed: true,
        violations: [],
      });
      expect(entry.id).toBeDefined();
      expect(entry.command).toBe('publish');
      expect(entry.target).toBe('/cmd_vel');
      expect(entry.safetyResult.allowed).toBe(true);
      expect(entry.timestamp).toBeGreaterThan(0);
    });

    it('logs a blocked command', () => {
      const entry = logger.log('publish', '/rosout', {}, {
        allowed: false,
        violations: [{
          type: 'blocked_topic',
          message: 'Topic /rosout is blocked',
          details: {},
          timestamp: Date.now(),
        }],
      });
      expect(entry.safetyResult.allowed).toBe(false);
      expect(entry.safetyResult.violations).toHaveLength(1);
    });
  });

  describe('getEntries', () => {
    beforeEach(() => {
      // Log 5 entries: 3 allowed, 2 blocked
      logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.log('publish', '/rosout', {}, {
        allowed: false,
        violations: [{ type: 'blocked_topic', message: 'blocked', details: {}, timestamp: Date.now() }],
      });
      logger.log('service_call', '/set_param', {}, { allowed: true, violations: [] });
      logger.log('service_call', '/kill', {}, {
        allowed: false,
        violations: [{ type: 'blocked_service', message: 'blocked', details: {}, timestamp: Date.now() }],
      });
      logger.log('publish', '/odom', {}, { allowed: true, violations: [] });
    });

    it('returns all entries newest first', () => {
      const entries = logger.getEntries();
      expect(entries).toHaveLength(5);
      expect(entries[0].target).toBe('/odom'); // newest
      expect(entries[4].target).toBe('/cmd_vel'); // oldest
    });

    it('limits number of returned entries', () => {
      const entries = logger.getEntries({ limit: 2 });
      expect(entries).toHaveLength(2);
    });

    it('filters by violations only', () => {
      const entries = logger.getEntries({ violationsOnly: true });
      expect(entries).toHaveLength(2);
      expect(entries.every(e => !e.safetyResult.allowed)).toBe(true);
    });

    it('filters by command type', () => {
      const entries = logger.getEntries({ command: 'service_call' });
      expect(entries).toHaveLength(2);
      expect(entries.every(e => e.command === 'service_call')).toBe(true);
    });

    it('combines filters', () => {
      const entries = logger.getEntries({ command: 'service_call', violationsOnly: true });
      expect(entries).toHaveLength(1);
      expect(entries[0].target).toBe('/kill');
    });
  });

  describe('updateResult', () => {
    it('updates execution result on an entry', () => {
      const entry = logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.updateResult(entry.id, 'success');
      const entries = logger.getEntries();
      expect(entries[0].executionResult).toBe('success');
    });

    it('records error details', () => {
      const entry = logger.log('service_call', '/svc', {}, { allowed: true, violations: [] });
      logger.updateResult(entry.id, 'error', 'connection timeout');
      const entries = logger.getEntries();
      expect(entries[0].executionResult).toBe('error');
      expect(entries[0].executionError).toBe('connection timeout');
    });
  });

  describe('getStats', () => {
    it('returns correct stats', () => {
      logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.log('publish', '/rosout', {}, {
        allowed: false,
        violations: [{ type: 'blocked_topic', message: 'blocked', details: {}, timestamp: Date.now() }],
      });
      const entry = logger.log('service_call', '/svc', {}, { allowed: true, violations: [] });
      logger.updateResult(entry.id, 'error', 'timeout');

      const stats = logger.getStats();
      expect(stats.total).toBe(3);
      expect(stats.allowed).toBe(2);
      expect(stats.blocked).toBe(1);
      expect(stats.errors).toBe(1);
    });

    it('returns zeros for empty logger', () => {
      const stats = logger.getStats();
      expect(stats).toEqual({ total: 0, allowed: 0, blocked: 0, errors: 0 });
    });
  });

  describe('max entries cap', () => {
    it('evicts oldest entries when exceeding max', () => {
      const smallLogger = new AuditLogger(5);
      for (let i = 0; i < 10; i++) {
        smallLogger.log('publish', `/topic_${i}`, {}, { allowed: true, violations: [] });
      }
      const entries = smallLogger.getEntries();
      expect(entries.length).toBeLessThanOrEqual(5);
      // Should have the newest entries (topic_5 through topic_9)
      expect(entries[0].target).toBe('/topic_9');
    });
  });

  describe('clear', () => {
    it('removes all entries', () => {
      logger.log('publish', '/cmd_vel', {}, { allowed: true, violations: [] });
      logger.log('publish', '/odom', {}, { allowed: true, violations: [] });
      expect(logger.getEntries()).toHaveLength(2);

      logger.clear();
      expect(logger.getEntries()).toHaveLength(0);
      expect(logger.getStats().total).toBe(0);
    });
  });
});

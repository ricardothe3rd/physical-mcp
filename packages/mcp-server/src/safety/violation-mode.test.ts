/**
 * Tests for configurable violation mode (block vs warn).
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { applyViolationMode, ViolationModeManager, type ViolationMode } from './violation-mode.js';
import type { SafetyCheckResult } from './types.js';

describe('Violation Mode', () => {
  describe('applyViolationMode', () => {
    const blockedResult: SafetyCheckResult = {
      allowed: false,
      violations: [
        {
          type: 'velocity_exceeded',
          message: 'Linear velocity 2.00 m/s exceeds limit of 0.50 m/s',
          details: { actual: 2.0, limit: 0.5 },
          timestamp: Date.now(),
        },
      ],
    };

    const allowedResult: SafetyCheckResult = {
      allowed: true,
      violations: [],
    };

    it('returns result as-is in block mode', () => {
      const result = applyViolationMode(blockedResult, 'block');
      expect(result.allowed).toBe(false);
      expect(result.violations[0].message).not.toContain('[WARN MODE]');
    });

    it('allows blocked commands in warn mode', () => {
      const result = applyViolationMode(blockedResult, 'warn');
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(1);
      expect(result.violations[0].message).toContain('[WARN MODE]');
    });

    it('preserves violation type in warn mode', () => {
      const result = applyViolationMode(blockedResult, 'warn');
      expect(result.violations[0].type).toBe('velocity_exceeded');
    });

    it('passes through already-allowed results in warn mode', () => {
      const result = applyViolationMode(allowedResult, 'warn');
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
    });

    it('passes through already-allowed results in block mode', () => {
      const result = applyViolationMode(allowedResult, 'block');
      expect(result.allowed).toBe(true);
    });

    it('handles multiple violations in warn mode', () => {
      const multiViolation: SafetyCheckResult = {
        allowed: false,
        violations: [
          { type: 'velocity_exceeded', message: 'Too fast', details: {}, timestamp: Date.now() },
          { type: 'rate_limit_exceeded', message: 'Too many', details: {}, timestamp: Date.now() },
        ],
      };
      const result = applyViolationMode(multiViolation, 'warn');
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(2);
      expect(result.violations[0].message).toContain('[WARN MODE]');
      expect(result.violations[1].message).toContain('[WARN MODE]');
    });
  });

  describe('ViolationModeManager', () => {
    let manager: ViolationModeManager;

    beforeEach(() => {
      manager = new ViolationModeManager();
    });

    it('defaults to block mode', () => {
      expect(manager.getMode()).toBe('block');
    });

    it('switches to warn mode', () => {
      manager.setMode('warn');
      expect(manager.getMode()).toBe('warn');
    });

    it('blocks violations in block mode', () => {
      const result = manager.process({
        allowed: false,
        violations: [{ type: 'velocity_exceeded', message: 'test', details: {}, timestamp: Date.now() }],
      });
      expect(result.allowed).toBe(false);
    });

    it('allows violations in warn mode', () => {
      manager.setMode('warn');
      const result = manager.process({
        allowed: false,
        violations: [{ type: 'velocity_exceeded', message: 'test', details: {}, timestamp: Date.now() }],
      });
      expect(result.allowed).toBe(true);
    });

    it('tracks warned count', () => {
      manager.setMode('warn');
      manager.process({
        allowed: false,
        violations: [{ type: 'velocity_exceeded', message: 'test', details: {}, timestamp: Date.now() }],
      });
      manager.process({
        allowed: false,
        violations: [{ type: 'rate_limit_exceeded', message: 'test', details: {}, timestamp: Date.now() }],
      });

      const stats = manager.getStats();
      expect(stats.warnedCount).toBe(2);
      expect(stats.blockedCount).toBe(0);
    });

    it('tracks blocked count', () => {
      manager.process({
        allowed: false,
        violations: [{ type: 'velocity_exceeded', message: 'test', details: {}, timestamp: Date.now() }],
      });

      const stats = manager.getStats();
      expect(stats.blockedCount).toBe(1);
      expect(stats.warnedCount).toBe(0);
    });

    it('does not count allowed results', () => {
      manager.process({ allowed: true, violations: [] });
      manager.process({ allowed: true, violations: [] });

      const stats = manager.getStats();
      expect(stats.blockedCount).toBe(0);
      expect(stats.warnedCount).toBe(0);
    });

    it('resets counts', () => {
      manager.process({
        allowed: false,
        violations: [{ type: 'velocity_exceeded', message: 'test', details: {}, timestamp: Date.now() }],
      });
      manager.reset();

      const stats = manager.getStats();
      expect(stats.blockedCount).toBe(0);
      expect(stats.warnedCount).toBe(0);
      expect(stats.mode).toBe('block'); // mode is preserved
    });

    it('returns stats with mode', () => {
      manager.setMode('warn');
      const stats = manager.getStats();
      expect(stats.mode).toBe('warn');
    });
  });
});

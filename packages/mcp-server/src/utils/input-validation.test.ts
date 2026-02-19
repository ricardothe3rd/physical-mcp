/**
 * Tests for ROS2 input validation utilities.
 */

import { describe, it, expect } from 'vitest';
import {
  validateTopicName,
  validateServiceName,
  validateActionName,
  validateMessagePayload,
  sanitizeString,
} from './input-validation.js';

describe('Input Validation', () => {
  // ── Topic Name Validation ──────────────────────────────────────────

  describe('validateTopicName', () => {
    it('accepts root topic /', () => {
      expect(validateTopicName('/')).toEqual({ valid: true });
    });

    it('accepts simple topic /cmd_vel', () => {
      expect(validateTopicName('/cmd_vel')).toEqual({ valid: true });
    });

    it('accepts nested topic /robot_1/cmd_vel', () => {
      expect(validateTopicName('/robot_1/cmd_vel')).toEqual({ valid: true });
    });

    it('accepts single character topic /a', () => {
      expect(validateTopicName('/a')).toEqual({ valid: true });
    });

    it('accepts deeply nested topic /a/b/c/d/e', () => {
      expect(validateTopicName('/a/b/c/d/e')).toEqual({ valid: true });
    });

    it('accepts topic with underscores /my_robot/cmd_vel_raw', () => {
      expect(validateTopicName('/my_robot/cmd_vel_raw')).toEqual({ valid: true });
    });

    it('accepts topic starting with underscore segment /_hidden', () => {
      expect(validateTopicName('/_hidden')).toEqual({ valid: true });
    });

    it('rejects topic without leading slash', () => {
      const result = validateTopicName('cmd_vel');
      expect(result.valid).toBe(false);
      expect(result.error).toContain("must start with '/'");
    });

    it('rejects topic with double slashes', () => {
      const result = validateTopicName('//double');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('double slashes');
    });

    it('rejects topic with trailing slash', () => {
      const result = validateTopicName('/trailing/');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('trailing');
    });

    it('rejects topic with special characters', () => {
      const result = validateTopicName('/special!chars');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('invalid segment');
    });

    it('rejects empty string', () => {
      const result = validateTopicName('');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('non-empty string');
    });

    it('rejects topic with spaces', () => {
      const result = validateTopicName('/with spaces');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('invalid segment');
    });

    it('rejects topic with hyphens', () => {
      const result = validateTopicName('/my-topic');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('invalid segment');
    });

    it('rejects topic segment starting with a digit', () => {
      const result = validateTopicName('/1bad');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('invalid segment');
    });

    it('rejects double slashes in the middle', () => {
      const result = validateTopicName('/a//b');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('double slashes');
    });
  });

  // ── Service Name Validation ────────────────────────────────────────

  describe('validateServiceName', () => {
    it('accepts valid service name /set_speed', () => {
      expect(validateServiceName('/set_speed')).toEqual({ valid: true });
    });

    it('accepts nested service /robot/set_speed', () => {
      expect(validateServiceName('/robot/set_speed')).toEqual({ valid: true });
    });

    it('rejects service without leading slash', () => {
      const result = validateServiceName('set_speed');
      expect(result.valid).toBe(false);
      expect(result.error).toContain("must start with '/'");
    });

    it('rejects service with trailing slash', () => {
      const result = validateServiceName('/set_speed/');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('trailing');
    });

    it('rejects empty service name', () => {
      const result = validateServiceName('');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('non-empty string');
    });

    it('includes "Service" in error messages', () => {
      const result = validateServiceName('bad');
      expect(result.error).toContain('Service');
    });
  });

  // ── Action Name Validation ─────────────────────────────────────────

  describe('validateActionName', () => {
    it('accepts valid action name /navigate_to_pose', () => {
      expect(validateActionName('/navigate_to_pose')).toEqual({ valid: true });
    });

    it('accepts nested action /robot/navigate_to_pose', () => {
      expect(validateActionName('/robot/navigate_to_pose')).toEqual({ valid: true });
    });

    it('rejects action without leading slash', () => {
      const result = validateActionName('navigate');
      expect(result.valid).toBe(false);
      expect(result.error).toContain("must start with '/'");
    });

    it('includes "Action" in error messages', () => {
      const result = validateActionName('bad');
      expect(result.error).toContain('Action');
    });
  });

  // ── Message Payload Validation ─────────────────────────────────────

  describe('validateMessagePayload', () => {
    it('accepts a plain object', () => {
      expect(validateMessagePayload({ linear: { x: 0.5 } })).toEqual({ valid: true });
    });

    it('accepts an empty object', () => {
      expect(validateMessagePayload({})).toEqual({ valid: true });
    });

    it('rejects null', () => {
      const result = validateMessagePayload(null);
      expect(result.valid).toBe(false);
      expect(result.error).toContain('null');
    });

    it('rejects undefined', () => {
      const result = validateMessagePayload(undefined);
      expect(result.valid).toBe(false);
      expect(result.error).toContain('null or undefined');
    });

    it('rejects arrays', () => {
      const result = validateMessagePayload([1, 2, 3]);
      expect(result.valid).toBe(false);
      expect(result.error).toContain('not an array');
    });

    it('rejects strings', () => {
      const result = validateMessagePayload('hello');
      expect(result.valid).toBe(false);
      expect(result.error).toContain('must be an object');
    });

    it('rejects numbers', () => {
      const result = validateMessagePayload(42);
      expect(result.valid).toBe(false);
      expect(result.error).toContain('must be an object');
    });

    it('rejects booleans', () => {
      const result = validateMessagePayload(true);
      expect(result.valid).toBe(false);
      expect(result.error).toContain('must be an object');
    });
  });

  // ── Sanitize String ────────────────────────────────────────────────

  describe('sanitizeString', () => {
    it('trims leading and trailing whitespace', () => {
      expect(sanitizeString('  hello  ')).toBe('hello');
    });

    it('removes control characters', () => {
      expect(sanitizeString('hello\x00world')).toBe('helloworld');
    });

    it('removes multiple control characters', () => {
      expect(sanitizeString('\x01\x02hello\x7Fworld\x1F')).toBe('helloworld');
    });

    it('preserves newlines and tabs', () => {
      expect(sanitizeString('hello\nworld\ttab')).toBe('hello\nworld\ttab');
    });

    it('enforces default max length of 1000', () => {
      const long = 'a'.repeat(2000);
      const result = sanitizeString(long);
      expect(result.length).toBe(1000);
    });

    it('enforces custom max length', () => {
      const result = sanitizeString('hello world', 5);
      expect(result).toBe('hello');
    });

    it('returns empty string when input is only whitespace', () => {
      expect(sanitizeString('   ')).toBe('');
    });

    it('does not truncate strings under max length', () => {
      expect(sanitizeString('short')).toBe('short');
    });

    it('handles empty string', () => {
      expect(sanitizeString('')).toBe('');
    });
  });
});

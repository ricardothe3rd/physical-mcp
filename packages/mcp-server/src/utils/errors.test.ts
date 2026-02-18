/**
 * Tests for custom error types.
 */

import { describe, it, expect } from 'vitest';
import {
  PhysicalMCPError,
  BridgeConnectionError,
  BridgeTimeoutError,
  SafetyViolationError,
  PolicyLoadError,
} from './errors.js';

describe('Custom Error Types', () => {
  describe('PhysicalMCPError', () => {
    it('creates error with correct name and message', () => {
      const err = new PhysicalMCPError('something went wrong');
      expect(err.name).toBe('PhysicalMCPError');
      expect(err.message).toBe('something went wrong');
      expect(err instanceof Error).toBe(true);
      expect(err instanceof PhysicalMCPError).toBe(true);
    });
  });

  describe('BridgeConnectionError', () => {
    it('creates error with URL', () => {
      const err = new BridgeConnectionError('connection refused', 'ws://localhost:9090');
      expect(err.name).toBe('BridgeConnectionError');
      expect(err.url).toBe('ws://localhost:9090');
      expect(err.message).toBe('connection refused');
      expect(err instanceof PhysicalMCPError).toBe(true);
    });

    it('works without URL', () => {
      const err = new BridgeConnectionError('unknown error');
      expect(err.url).toBeUndefined();
    });
  });

  describe('BridgeTimeoutError', () => {
    it('creates error with command type and timeout', () => {
      const err = new BridgeTimeoutError('topic.publish', 5000);
      expect(err.name).toBe('BridgeTimeoutError');
      expect(err.commandType).toBe('topic.publish');
      expect(err.timeoutMs).toBe(5000);
      expect(err.message).toContain('topic.publish');
      expect(err.message).toContain('5000');
      expect(err instanceof PhysicalMCPError).toBe(true);
    });
  });

  describe('SafetyViolationError', () => {
    it('creates error with violation type and details', () => {
      const err = new SafetyViolationError(
        'velocity exceeded',
        'velocity_exceeded',
        { actual: 2.0, limit: 0.5 }
      );
      expect(err.name).toBe('SafetyViolationError');
      expect(err.violationType).toBe('velocity_exceeded');
      expect(err.details.actual).toBe(2.0);
      expect(err.details.limit).toBe(0.5);
      expect(err instanceof PhysicalMCPError).toBe(true);
    });
  });

  describe('PolicyLoadError', () => {
    it('creates error with file path', () => {
      const err = new PolicyLoadError('invalid YAML', '/path/to/policy.yaml');
      expect(err.name).toBe('PolicyLoadError');
      expect(err.filePath).toBe('/path/to/policy.yaml');
      expect(err instanceof PhysicalMCPError).toBe(true);
    });

    it('works without file path', () => {
      const err = new PolicyLoadError('no file specified');
      expect(err.filePath).toBeUndefined();
    });
  });

  describe('error hierarchy', () => {
    it('all custom errors are instances of Error', () => {
      const errors = [
        new PhysicalMCPError('test'),
        new BridgeConnectionError('test'),
        new BridgeTimeoutError('ping', 1000),
        new SafetyViolationError('test', 'test_type', {}),
        new PolicyLoadError('test'),
      ];

      for (const err of errors) {
        expect(err instanceof Error).toBe(true);
        expect(err instanceof PhysicalMCPError).toBe(true);
        expect(err.stack).toBeDefined();
      }
    });

    it('can be caught as PhysicalMCPError', () => {
      try {
        throw new BridgeTimeoutError('topic.publish', 5000);
      } catch (err) {
        expect(err instanceof PhysicalMCPError).toBe(true);
      }
    });
  });
});

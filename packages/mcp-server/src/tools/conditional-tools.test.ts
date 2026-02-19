/**
 * Tests for conditional command execution tools.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { evaluateCondition, extractField, getConditionalTools, handleConditionalTool } from './conditional-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

function createMockConnection(echoResponse: unknown = {}) {
  return {
    isConnected: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { message: echoResponse },
      timestamp: Date.now(),
    }),
    disconnect: vi.fn(),
  } as any;
}

describe('Conditional Tools', () => {
  describe('evaluateCondition', () => {
    it('eq: matches equal values', () => {
      expect(evaluateCondition(5, 'eq', 5)).toBe(true);
      expect(evaluateCondition('hello', 'eq', 'hello')).toBe(true);
      expect(evaluateCondition(5, 'eq', 6)).toBe(false);
    });

    it('neq: matches unequal values', () => {
      expect(evaluateCondition(5, 'neq', 6)).toBe(true);
      expect(evaluateCondition(5, 'neq', 5)).toBe(false);
    });

    it('gt: greater than', () => {
      expect(evaluateCondition(10, 'gt', 5)).toBe(true);
      expect(evaluateCondition(5, 'gt', 10)).toBe(false);
      expect(evaluateCondition(5, 'gt', 5)).toBe(false);
    });

    it('gte: greater than or equal', () => {
      expect(evaluateCondition(10, 'gte', 5)).toBe(true);
      expect(evaluateCondition(5, 'gte', 5)).toBe(true);
      expect(evaluateCondition(4, 'gte', 5)).toBe(false);
    });

    it('lt: less than', () => {
      expect(evaluateCondition(3, 'lt', 5)).toBe(true);
      expect(evaluateCondition(5, 'lt', 5)).toBe(false);
    });

    it('lte: less than or equal', () => {
      expect(evaluateCondition(3, 'lte', 5)).toBe(true);
      expect(evaluateCondition(5, 'lte', 5)).toBe(true);
      expect(evaluateCondition(6, 'lte', 5)).toBe(false);
    });

    it('contains: string contains', () => {
      expect(evaluateCondition('hello world', 'contains', 'world')).toBe(true);
      expect(evaluateCondition('hello', 'contains', 'xyz')).toBe(false);
    });

    it('exists: value exists', () => {
      expect(evaluateCondition(42, 'exists', undefined)).toBe(true);
      expect(evaluateCondition('', 'exists', undefined)).toBe(true);
      expect(evaluateCondition(null, 'exists', undefined)).toBe(false);
      expect(evaluateCondition(undefined, 'exists', undefined)).toBe(false);
    });

    it('returns false for type mismatches in numeric ops', () => {
      expect(evaluateCondition('5', 'gt', 3)).toBe(false);
      expect(evaluateCondition(5, 'gt', '3')).toBe(false);
    });

    it('returns false for unknown operator', () => {
      expect(evaluateCondition(5, 'unknown' as any, 5)).toBe(false);
    });
  });

  describe('extractField', () => {
    it('extracts top-level field', () => {
      expect(extractField({ x: 10 }, 'x')).toBe(10);
    });

    it('extracts nested field', () => {
      expect(extractField({ linear: { x: 0.5 } }, 'linear.x')).toBe(0.5);
    });

    it('extracts deeply nested field', () => {
      expect(extractField({ a: { b: { c: 42 } } }, 'a.b.c')).toBe(42);
    });

    it('returns undefined for missing field', () => {
      expect(extractField({ x: 1 }, 'y')).toBeUndefined();
    });

    it('returns undefined for null/undefined intermediate', () => {
      expect(extractField({ a: null }, 'a.b')).toBeUndefined();
      expect(extractField(null, 'a')).toBeUndefined();
    });
  });

  describe('getConditionalTools', () => {
    it('returns 2 tools', () => {
      const tools = getConditionalTools();
      expect(tools).toHaveLength(2);
      expect(tools.map(t => t.name)).toContain('ros2_conditional_execute');
      expect(tools.map(t => t.name)).toContain('ros2_wait_for_condition');
    });
  });

  describe('handleConditionalTool - ros2_conditional_execute', () => {
    let safety: PolicyEngine;

    beforeEach(() => {
      safety = new PolicyEngine();
    });

    it('executes then-tool when condition is true', async () => {
      const connection = createMockConnection({ percentage: 15 });
      const result = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/battery_state',
        field: 'percentage',
        operator: 'lt',
        value: 20,
        thenTool: 'ros2_topic_echo',
        thenArgs: { topic: '/cmd_vel' },
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.conditionMet).toBe(true);
      expect(data.executedTool).toBe('ros2_topic_echo');
    });

    it('returns no action when condition is false and no else-tool', async () => {
      const connection = createMockConnection({ percentage: 80 });
      const result = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/battery_state',
        field: 'percentage',
        operator: 'lt',
        value: 20,
        thenTool: 'ros2_topic_echo',
        thenArgs: { topic: '/cmd_vel' },
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.conditionMet).toBe(false);
      expect(data.action).toContain('none');
    });

    it('executes else-tool when condition is false', async () => {
      const connection = createMockConnection({ percentage: 80 });
      const result = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/battery_state',
        field: 'percentage',
        operator: 'lt',
        value: 20,
        thenTool: 'ros2_topic_echo',
        thenArgs: { topic: '/charger' },
        elseTool: 'ros2_topic_echo',
        elseArgs: { topic: '/status' },
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.conditionMet).toBe(false);
      expect(data.executedTool).toBe('ros2_topic_echo');
    });

    it('handles topic read failure', async () => {
      const connection = createMockConnection();
      connection.send.mockRejectedValue(new Error('Bridge timeout'));

      const result = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/battery_state',
        field: 'percentage',
        operator: 'lt',
        value: 20,
        thenTool: 'ros2_topic_echo',
        thenArgs: {},
      }, connection, safety);

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Failed to read topic');
    });

    it('works with nested fields', async () => {
      const connection = createMockConnection({ linear: { x: 0.3 } });
      const result = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/cmd_vel',
        field: 'linear.x',
        operator: 'gt',
        value: 0.1,
        thenTool: 'safety_status',
        thenArgs: {},
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.conditionMet).toBe(true);
    });

    it('returns error for unknown tool', async () => {
      const connection = createMockConnection({ value: 1 });
      const result = await handleConditionalTool('ros2_conditional_execute', {
        topic: '/sensor',
        field: 'value',
        operator: 'eq',
        value: 1,
        thenTool: 'nonexistent_tool',
        thenArgs: {},
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.isError).toBe(true);
    });
  });

  describe('handleConditionalTool - ros2_wait_for_condition', () => {
    let safety: PolicyEngine;

    beforeEach(() => {
      safety = new PolicyEngine();
    });

    it('returns immediately when condition is already met', async () => {
      const connection = createMockConnection({ value: 100 });
      const result = await handleConditionalTool('ros2_wait_for_condition', {
        topic: '/sensor',
        field: 'value',
        operator: 'gte',
        value: 50,
        timeoutMs: 5000,
        pollIntervalMs: 100,
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.conditionMet).toBe(true);
      expect(data.attempts).toBe(1);
    });

    it('times out when condition is never met', async () => {
      const connection = createMockConnection({ value: 10 });
      const result = await handleConditionalTool('ros2_wait_for_condition', {
        topic: '/sensor',
        field: 'value',
        operator: 'gte',
        value: 1000,
        timeoutMs: 300,
        pollIntervalMs: 100,
      }, connection, safety);

      const data = JSON.parse(result.content[0].text);
      expect(data.conditionMet).toBe(false);
      expect(data.reason).toBe('timeout');
      expect(result.isError).toBe(true);
    });

    it('returns unknown conditional tool error', async () => {
      const connection = createMockConnection();
      const result = await handleConditionalTool('nonexistent', {}, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Unknown conditional tool');
    });
  });
});

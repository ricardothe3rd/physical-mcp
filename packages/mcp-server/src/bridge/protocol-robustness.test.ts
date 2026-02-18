/**
 * Protocol robustness tests: malformed data, edge cases, schema validation.
 */

import { describe, it, expect } from 'vitest';
import { BridgeCommandSchema, BridgeResponseSchema, createCommand, CommandType } from './protocol.js';

describe('Protocol Robustness', () => {
  describe('BridgeCommandSchema edge cases', () => {
    it('accepts command with extra unknown fields', () => {
      const result = BridgeCommandSchema.parse({
        id: '1',
        type: 'topic.list',
        params: {},
        extraField: 'should be ignored',
      });
      expect(result.id).toBe('1');
      expect(result.type).toBe('topic.list');
    });

    it('rejects empty string id', () => {
      // Empty string is still a valid string
      const result = BridgeCommandSchema.parse({
        id: '',
        type: 'topic.list',
      });
      expect(result.id).toBe('');
    });

    it('rejects numeric id', () => {
      expect(() => BridgeCommandSchema.parse({
        id: 123,
        type: 'topic.list',
      })).toThrow();
    });

    it('rejects null type', () => {
      expect(() => BridgeCommandSchema.parse({
        id: '1',
        type: null,
      })).toThrow();
    });

    it('accepts params with nested objects', () => {
      const result = BridgeCommandSchema.parse({
        id: '1',
        type: 'topic.publish',
        params: {
          topic: '/cmd_vel',
          message: {
            linear: { x: 0.5, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0.3 },
          },
        },
      });
      expect((result.params as any).topic).toBe('/cmd_vel');
    });

    it('accepts params with arrays', () => {
      const result = BridgeCommandSchema.parse({
        id: '1',
        type: 'topic.subscribe',
        params: { topics: ['/cmd_vel', '/odom'] },
      });
      expect((result.params as any).topics).toHaveLength(2);
    });
  });

  describe('BridgeResponseSchema edge cases', () => {
    it('accepts data with any type', () => {
      // string data
      const r1 = BridgeResponseSchema.parse({ id: '1', status: 'ok', data: 'hello', timestamp: 1 });
      expect(r1.data).toBe('hello');

      // number data
      const r2 = BridgeResponseSchema.parse({ id: '1', status: 'ok', data: 42, timestamp: 1 });
      expect(r2.data).toBe(42);

      // array data
      const r3 = BridgeResponseSchema.parse({ id: '1', status: 'ok', data: [1, 2, 3], timestamp: 1 });
      expect(r3.data).toEqual([1, 2, 3]);

      // null data
      const r4 = BridgeResponseSchema.parse({ id: '1', status: 'ok', data: null, timestamp: 1 });
      expect(r4.data).toBeNull();
    });

    it('rejects missing timestamp', () => {
      expect(() => BridgeResponseSchema.parse({
        id: '1',
        status: 'ok',
        data: {},
      })).toThrow();
    });

    it('rejects non-numeric timestamp', () => {
      expect(() => BridgeResponseSchema.parse({
        id: '1',
        status: 'ok',
        data: {},
        timestamp: '2024-01-01',
      })).toThrow();
    });

    it('rejects unknown status values', () => {
      expect(() => BridgeResponseSchema.parse({
        id: '1',
        status: 'pending',
        data: {},
        timestamp: Date.now(),
      })).toThrow();
    });

    it('accepts zero timestamp', () => {
      const result = BridgeResponseSchema.parse({
        id: '1', status: 'ok', data: {}, timestamp: 0,
      });
      expect(result.timestamp).toBe(0);
    });

    it('accepts negative timestamp', () => {
      const result = BridgeResponseSchema.parse({
        id: '1', status: 'ok', data: {}, timestamp: -1,
      });
      expect(result.timestamp).toBe(-1);
    });
  });

  describe('createCommand', () => {
    it('creates command for every CommandType', () => {
      const types = Object.values(CommandType);
      for (const type of types) {
        const cmd = createCommand(type);
        expect(cmd.type).toBe(type);
        expect(cmd.params).toEqual({});
      }
    });

    it('preserves complex params structure', () => {
      const cmd = createCommand(CommandType.TOPIC_PUBLISH, {
        topic: '/cmd_vel',
        message_type: 'geometry_msgs/msg/Twist',
        message: {
          linear: { x: 0.5, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 1.0 },
        },
      });
      expect((cmd.params as any).message.linear.x).toBe(0.5);
    });
  });

  describe('CommandType completeness', () => {
    it('has all expected command types', () => {
      const expectedTypes = [
        'topic.list', 'topic.info', 'topic.subscribe', 'topic.publish', 'topic.echo',
        'service.list', 'service.info', 'service.call',
        'action.list', 'action.send_goal', 'action.cancel', 'action.status',
        'node.list', 'node.info',
        'params.get', 'params.set', 'params.list',
        'ping', 'emergency_stop',
      ];
      const actualTypes = Object.values(CommandType);
      for (const expected of expectedTypes) {
        expect(actualTypes).toContain(expected);
      }
    });

    it('has correct number of command types', () => {
      expect(Object.keys(CommandType)).toHaveLength(19);
    });
  });
});

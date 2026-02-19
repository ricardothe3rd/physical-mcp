import { describe, it, expect } from 'vitest';
import {
  CommandType,
  BridgeCommandSchema,
  BridgeResponseSchema,
  createCommand,
} from './protocol.js';

// ---------------------------------------------------------------------------
// BridgeCommandSchema validation
// ---------------------------------------------------------------------------
describe('BridgeCommandSchema', () => {
  describe('valid commands', () => {
    it('parses a command with all fields', () => {
      const input = { id: 'abc-123', type: 'topic.list', params: { foo: 'bar' } };
      const result = BridgeCommandSchema.parse(input);
      expect(result).toEqual(input);
    });

    it('defaults params to empty object when omitted', () => {
      const result = BridgeCommandSchema.parse({ id: '456', type: 'ping' });
      expect(result.params).toEqual({});
    });

    it('accepts empty params object', () => {
      const result = BridgeCommandSchema.parse({ id: 'x', type: 'ping', params: {} });
      expect(result.params).toEqual({});
    });

    it('allows params with nested unknown values', () => {
      const input = {
        id: 'x',
        type: 'service.call',
        params: { deep: { nested: [1, 2, 3] } },
      };
      const result = BridgeCommandSchema.parse(input);
      expect(result.params).toEqual({ deep: { nested: [1, 2, 3] } });
    });
  });

  describe('missing fields', () => {
    it('rejects a command missing id', () => {
      expect(() => BridgeCommandSchema.parse({ type: 'ping', params: {} })).toThrow();
    });

    it('rejects a command missing type', () => {
      expect(() => BridgeCommandSchema.parse({ id: '789', params: {} })).toThrow();
    });

    it('rejects completely empty object', () => {
      expect(() => BridgeCommandSchema.parse({})).toThrow();
    });
  });

  describe('wrong field types', () => {
    it('rejects non-string id', () => {
      expect(() => BridgeCommandSchema.parse({ id: 123, type: 'ping' })).toThrow();
    });

    it('rejects non-string type', () => {
      expect(() => BridgeCommandSchema.parse({ id: 'abc', type: 42 })).toThrow();
    });

    it('rejects null id', () => {
      expect(() => BridgeCommandSchema.parse({ id: null, type: 'ping' })).toThrow();
    });
  });

  describe('extra fields', () => {
    it('strips unrecognised fields (zod default strip behaviour)', () => {
      const input = { id: 'abc', type: 'ping', params: {}, extraField: 'nope' };
      const result = BridgeCommandSchema.parse(input);
      expect(result).toEqual({ id: 'abc', type: 'ping', params: {} });
      expect((result as Record<string, unknown>).extraField).toBeUndefined();
    });
  });

  describe('oversized messages', () => {
    it('handles a very long type string', () => {
      const longType = 'a'.repeat(100_000);
      const result = BridgeCommandSchema.parse({ id: 'id', type: longType, params: {} });
      expect(result.type).toHaveLength(100_000);
    });

    it('handles a very long id string', () => {
      const longId = 'b'.repeat(100_000);
      const result = BridgeCommandSchema.parse({ id: longId, type: 'ping', params: {} });
      expect(result.id).toHaveLength(100_000);
    });

    it('handles a huge payload in params', () => {
      const bigPayload = 'x'.repeat(1_000_000);
      const result = BridgeCommandSchema.parse({
        id: 'id',
        type: 'topic.publish',
        params: { data: bigPayload },
      });
      expect((result.params as Record<string, string>).data).toHaveLength(1_000_000);
    });
  });

  describe('malformed JSON scenarios', () => {
    it('rejects a plain string', () => {
      expect(() => BridgeCommandSchema.parse('not json')).toThrow();
    });

    it('rejects a number', () => {
      expect(() => BridgeCommandSchema.parse(42)).toThrow();
    });

    it('rejects an array', () => {
      expect(() => BridgeCommandSchema.parse([1, 2, 3])).toThrow();
    });

    it('rejects null', () => {
      expect(() => BridgeCommandSchema.parse(null)).toThrow();
    });

    it('rejects undefined', () => {
      expect(() => BridgeCommandSchema.parse(undefined)).toThrow();
    });

    it('rejects boolean', () => {
      expect(() => BridgeCommandSchema.parse(true)).toThrow();
    });
  });
});

// ---------------------------------------------------------------------------
// BridgeResponseSchema validation
// ---------------------------------------------------------------------------
describe('BridgeResponseSchema', () => {
  describe('valid responses', () => {
    it('parses a valid ok response', () => {
      const input = { id: '123', status: 'ok', data: { topics: [] }, timestamp: 1700000000 };
      const result = BridgeResponseSchema.parse(input);
      expect(result).toEqual(input);
    });

    it('parses a valid error response', () => {
      const input = { id: '123', status: 'error', data: 'something went wrong', timestamp: 1700000000 };
      const result = BridgeResponseSchema.parse(input);
      expect(result.status).toBe('error');
    });

    it('allows null id', () => {
      const result = BridgeResponseSchema.parse({
        id: null,
        status: 'ok',
        data: null,
        timestamp: 0,
      });
      expect(result.id).toBeNull();
    });

    it('accepts any type for data field', () => {
      const cases = [null, undefined, 42, 'hello', [1, 2, 3], { nested: true }, true];
      for (const data of cases) {
        const result = BridgeResponseSchema.parse({
          id: 'id',
          status: 'ok',
          data,
          timestamp: 123,
        });
        expect(result.data).toEqual(data);
      }
    });
  });

  describe('missing fields', () => {
    it('rejects missing id', () => {
      expect(() => BridgeResponseSchema.parse({
        status: 'ok', data: null, timestamp: 0,
      })).toThrow();
    });

    it('rejects missing status', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', data: null, timestamp: 0,
      })).toThrow();
    });

    it('accepts missing data (z.unknown() permits undefined)', () => {
      // data is typed as z.unknown() which allows undefined / missing keys
      const result = BridgeResponseSchema.parse({
        id: 'abc', status: 'ok', timestamp: 0,
      });
      expect(result.data).toBeUndefined();
    });

    it('rejects missing timestamp', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', status: 'ok', data: null,
      })).toThrow();
    });
  });

  describe('wrong status values', () => {
    it('rejects status "pending"', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', status: 'pending', data: null, timestamp: 0,
      })).toThrow();
    });

    it('rejects status "success"', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', status: 'success', data: null, timestamp: 0,
      })).toThrow();
    });

    it('rejects status "failure"', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', status: 'failure', data: null, timestamp: 0,
      })).toThrow();
    });

    it('rejects numeric status', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', status: 200, data: null, timestamp: 0,
      })).toThrow();
    });
  });

  describe('wrong field types', () => {
    it('rejects non-number timestamp', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', status: 'ok', data: null, timestamp: '2024-01-01',
      })).toThrow();
    });

    it('rejects boolean timestamp', () => {
      expect(() => BridgeResponseSchema.parse({
        id: 'abc', status: 'ok', data: null, timestamp: true,
      })).toThrow();
    });
  });

  describe('extra fields', () => {
    it('strips unrecognised fields', () => {
      const result = BridgeResponseSchema.parse({
        id: 'abc', status: 'ok', data: null, timestamp: 0, extra: 'nope',
      });
      expect((result as Record<string, unknown>).extra).toBeUndefined();
    });
  });

  describe('malformed inputs', () => {
    it('rejects a string', () => {
      expect(() => BridgeResponseSchema.parse('not json')).toThrow();
    });

    it('rejects a number', () => {
      expect(() => BridgeResponseSchema.parse(42)).toThrow();
    });

    it('rejects boolean', () => {
      expect(() => BridgeResponseSchema.parse(true)).toThrow();
    });

    it('rejects null', () => {
      expect(() => BridgeResponseSchema.parse(null)).toThrow();
    });
  });
});

// ---------------------------------------------------------------------------
// createCommand helper
// ---------------------------------------------------------------------------
describe('createCommand', () => {
  it('creates a command with type and default empty params', () => {
    const cmd = createCommand(CommandType.PING);
    expect(cmd).toEqual({ type: 'ping', params: {} });
  });

  it('creates a command with type and custom params', () => {
    const cmd = createCommand(CommandType.TOPIC_PUBLISH, { topic: '/cmd_vel', data: { x: 1 } });
    expect(cmd).toEqual({
      type: 'topic.publish',
      params: { topic: '/cmd_vel', data: { x: 1 } },
    });
  });

  it('creates a command with empty params object explicitly', () => {
    const cmd = createCommand(CommandType.NODE_LIST, {});
    expect(cmd).toEqual({ type: 'node.list', params: {} });
  });

  it('does not include an id field', () => {
    const cmd = createCommand(CommandType.TOPIC_LIST);
    expect(cmd).not.toHaveProperty('id');
  });

  it('maps each CommandType value correctly', () => {
    expect(createCommand(CommandType.TOPIC_LIST).type).toBe('topic.list');
    expect(createCommand(CommandType.TOPIC_INFO).type).toBe('topic.info');
    expect(createCommand(CommandType.TOPIC_SUBSCRIBE).type).toBe('topic.subscribe');
    expect(createCommand(CommandType.SERVICE_CALL).type).toBe('service.call');
    expect(createCommand(CommandType.ACTION_SEND_GOAL).type).toBe('action.send_goal');
    expect(createCommand(CommandType.EMERGENCY_STOP).type).toBe('emergency_stop');
  });
});

// ---------------------------------------------------------------------------
// CommandType constants
// ---------------------------------------------------------------------------
describe('CommandType constants', () => {
  it('has all expected command type values', () => {
    expect(CommandType.TOPIC_LIST).toBe('topic.list');
    expect(CommandType.TOPIC_INFO).toBe('topic.info');
    expect(CommandType.TOPIC_SUBSCRIBE).toBe('topic.subscribe');
    expect(CommandType.TOPIC_PUBLISH).toBe('topic.publish');
    expect(CommandType.TOPIC_ECHO).toBe('topic.echo');
    expect(CommandType.SERVICE_LIST).toBe('service.list');
    expect(CommandType.SERVICE_INFO).toBe('service.info');
    expect(CommandType.SERVICE_CALL).toBe('service.call');
    expect(CommandType.ACTION_LIST).toBe('action.list');
    expect(CommandType.ACTION_SEND_GOAL).toBe('action.send_goal');
    expect(CommandType.ACTION_CANCEL).toBe('action.cancel');
    expect(CommandType.ACTION_STATUS).toBe('action.status');
    expect(CommandType.NODE_LIST).toBe('node.list');
    expect(CommandType.GET_PARAMS).toBe('params.get');
    expect(CommandType.SET_PARAMS).toBe('params.set');
    expect(CommandType.LIST_PARAMS).toBe('params.list');
    expect(CommandType.NODE_INFO).toBe('node.info');
    expect(CommandType.PING).toBe('ping');
    expect(CommandType.EMERGENCY_STOP).toBe('emergency_stop');
  });

  it('contains all unique values (no duplicates)', () => {
    const values = Object.values(CommandType);
    const unique = new Set(values);
    expect(unique.size).toBe(values.length);
  });

  it('has exactly 19 command types', () => {
    expect(Object.keys(CommandType)).toHaveLength(19);
  });
});

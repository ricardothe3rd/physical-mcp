import { describe, it, expect } from 'vitest';
import {
  CommandType,
  BridgeCommandSchema,
  BridgeResponseSchema,
  createCommand,
} from './protocol.js';

describe('CommandType', () => {
  it('has all expected command types', () => {
    expect(CommandType.TOPIC_LIST).toBe('topic.list');
    expect(CommandType.TOPIC_PUBLISH).toBe('topic.publish');
    expect(CommandType.SERVICE_CALL).toBe('service.call');
    expect(CommandType.ACTION_SEND_GOAL).toBe('action.send_goal');
    expect(CommandType.PING).toBe('ping');
    expect(CommandType.EMERGENCY_STOP).toBe('emergency_stop');
  });
});

describe('BridgeCommandSchema', () => {
  it('parses valid command', () => {
    const result = BridgeCommandSchema.parse({
      id: '123',
      type: 'topic.list',
      params: {},
    });
    expect(result.id).toBe('123');
    expect(result.type).toBe('topic.list');
    expect(result.params).toEqual({});
  });

  it('defaults params to empty object', () => {
    const result = BridgeCommandSchema.parse({
      id: '456',
      type: 'ping',
    });
    expect(result.params).toEqual({});
  });

  it('rejects missing id', () => {
    expect(() => BridgeCommandSchema.parse({
      type: 'ping',
    })).toThrow();
  });

  it('rejects missing type', () => {
    expect(() => BridgeCommandSchema.parse({
      id: '789',
    })).toThrow();
  });
});

describe('BridgeResponseSchema', () => {
  it('parses valid ok response', () => {
    const result = BridgeResponseSchema.parse({
      id: '123',
      status: 'ok',
      data: { topics: [] },
      timestamp: Date.now(),
    });
    expect(result.status).toBe('ok');
    expect(result.data).toEqual({ topics: [] });
  });

  it('parses valid error response', () => {
    const result = BridgeResponseSchema.parse({
      id: '123',
      status: 'error',
      data: { error: 'not found' },
      timestamp: Date.now(),
    });
    expect(result.status).toBe('error');
  });

  it('allows null id', () => {
    const result = BridgeResponseSchema.parse({
      id: null,
      status: 'ok',
      data: {},
      timestamp: Date.now(),
    });
    expect(result.id).toBeNull();
  });

  it('rejects invalid status', () => {
    expect(() => BridgeResponseSchema.parse({
      id: '123',
      status: 'invalid',
      data: {},
      timestamp: Date.now(),
    })).toThrow();
  });
});

describe('createCommand', () => {
  it('creates a command with type and params', () => {
    const cmd = createCommand(CommandType.TOPIC_PUBLISH, { topic: '/cmd_vel' });
    expect(cmd.type).toBe('topic.publish');
    expect(cmd.params).toEqual({ topic: '/cmd_vel' });
  });

  it('defaults params to empty object', () => {
    const cmd = createCommand(CommandType.PING);
    expect(cmd.params).toEqual({});
  });
});

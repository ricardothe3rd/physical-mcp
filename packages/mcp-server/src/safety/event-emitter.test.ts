/**
 * Tests for safety event emitter.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { SafetyEventEmitter, type SafetyEvent } from './event-emitter.js';

describe('SafetyEventEmitter', () => {
  let emitter: SafetyEventEmitter;

  beforeEach(() => {
    emitter = new SafetyEventEmitter();
  });

  describe('on', () => {
    it('subscribes to specific event type', () => {
      const listener = vi.fn();
      emitter.on('violation', listener);
      emitter.emit('violation', { type: 'velocity_exceeded' });

      expect(listener).toHaveBeenCalledTimes(1);
      expect(listener).toHaveBeenCalledWith(expect.objectContaining({
        type: 'violation',
        data: { type: 'velocity_exceeded' },
      }));
    });

    it('does not fire for other event types', () => {
      const listener = vi.fn();
      emitter.on('violation', listener);
      emitter.emit('emergency_stop_activated', {});

      expect(listener).not.toHaveBeenCalled();
    });

    it('returns unsubscribe function', () => {
      const listener = vi.fn();
      const unsub = emitter.on('violation', listener);

      emitter.emit('violation');
      expect(listener).toHaveBeenCalledTimes(1);

      unsub();
      emitter.emit('violation');
      expect(listener).toHaveBeenCalledTimes(1);
    });

    it('supports multiple listeners for same type', () => {
      const l1 = vi.fn();
      const l2 = vi.fn();
      emitter.on('violation', l1);
      emitter.on('violation', l2);
      emitter.emit('violation');

      expect(l1).toHaveBeenCalledTimes(1);
      expect(l2).toHaveBeenCalledTimes(1);
    });
  });

  describe('onAll', () => {
    it('receives all event types', () => {
      const listener = vi.fn();
      emitter.onAll(listener);

      emitter.emit('violation');
      emitter.emit('emergency_stop_activated');
      emitter.emit('policy_updated');

      expect(listener).toHaveBeenCalledTimes(3);
    });

    it('returns unsubscribe function', () => {
      const listener = vi.fn();
      const unsub = emitter.onAll(listener);

      emitter.emit('violation');
      unsub();
      emitter.emit('violation');

      expect(listener).toHaveBeenCalledTimes(1);
    });
  });

  describe('emit', () => {
    it('includes timestamp in event', () => {
      const listener = vi.fn();
      emitter.on('violation', listener);
      emitter.emit('violation');

      const event: SafetyEvent = listener.mock.calls[0][0];
      expect(event.timestamp).toBeGreaterThan(0);
    });

    it('defaults data to empty object', () => {
      const listener = vi.fn();
      emitter.on('violation', listener);
      emitter.emit('violation');

      const event: SafetyEvent = listener.mock.calls[0][0];
      expect(event.data).toEqual({});
    });

    it('catches listener errors without propagating', () => {
      const badListener = vi.fn(() => { throw new Error('boom'); });
      const goodListener = vi.fn();

      emitter.on('violation', badListener);
      emitter.on('violation', goodListener);

      expect(() => emitter.emit('violation')).not.toThrow();
      expect(goodListener).toHaveBeenCalledTimes(1);
    });

    it('catches onAll listener errors', () => {
      const badListener = vi.fn(() => { throw new Error('boom'); });
      emitter.onAll(badListener);

      expect(() => emitter.emit('violation')).not.toThrow();
    });
  });

  describe('getHistory', () => {
    it('returns all events', () => {
      emitter.emit('violation', { v: 1 });
      emitter.emit('emergency_stop_activated', { reason: 'test' });
      emitter.emit('violation', { v: 2 });

      const history = emitter.getHistory();
      expect(history).toHaveLength(3);
    });

    it('limits results', () => {
      for (let i = 0; i < 10; i++) emitter.emit('violation');

      const history = emitter.getHistory(5);
      expect(history).toHaveLength(5);
    });

    it('filters by event type', () => {
      emitter.emit('violation');
      emitter.emit('emergency_stop_activated');
      emitter.emit('violation');

      const history = emitter.getHistory(undefined, 'violation');
      expect(history).toHaveLength(2);
    });

    it('respects max history limit', () => {
      const smallEmitter = new SafetyEventEmitter(5);
      for (let i = 0; i < 10; i++) smallEmitter.emit('violation', { i });

      const history = smallEmitter.getHistory();
      expect(history).toHaveLength(5);
      expect((history[0].data as any).i).toBe(5); // first 5 events dropped
    });
  });

  describe('listenerCount', () => {
    it('counts type-specific listeners', () => {
      emitter.on('violation', vi.fn());
      emitter.on('violation', vi.fn());
      emitter.on('emergency_stop_activated', vi.fn());

      expect(emitter.listenerCount('violation')).toBe(2);
      expect(emitter.listenerCount('emergency_stop_activated')).toBe(1);
    });

    it('includes onAll listeners in type count', () => {
      emitter.on('violation', vi.fn());
      emitter.onAll(vi.fn());

      expect(emitter.listenerCount('violation')).toBe(2);
    });

    it('counts total listeners', () => {
      emitter.on('violation', vi.fn());
      emitter.on('emergency_stop_activated', vi.fn());
      emitter.onAll(vi.fn());

      expect(emitter.listenerCount()).toBe(3);
    });
  });

  describe('removeAllListeners', () => {
    it('removes all listeners', () => {
      const l1 = vi.fn();
      const l2 = vi.fn();
      emitter.on('violation', l1);
      emitter.onAll(l2);

      emitter.removeAllListeners();
      emitter.emit('violation');

      expect(l1).not.toHaveBeenCalled();
      expect(l2).not.toHaveBeenCalled();
    });
  });

  describe('clearHistory', () => {
    it('clears all history', () => {
      emitter.emit('violation');
      emitter.emit('violation');
      expect(emitter.getHistory()).toHaveLength(2);

      emitter.clearHistory();
      expect(emitter.getHistory()).toHaveLength(0);
    });
  });
});

/**
 * Tests for safety event publish/subscribe system.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import {
  SafetyEventType,
  SafetyEventBus,
  type SafetyEvent,
} from './safety-events.js';

describe('SafetyEventBus', () => {
  let bus: SafetyEventBus;

  beforeEach(() => {
    bus = new SafetyEventBus();
  });

  // ── subscribe + emit ──────────────────────────────────────────────

  describe('subscribe and emit', () => {
    it('delivers events to a type-specific subscriber', () => {
      const cb = vi.fn();
      bus.subscribe(SafetyEventType.VIOLATION, cb);
      bus.emit(SafetyEventType.VIOLATION, { reason: 'speed' }, 'critical');

      expect(cb).toHaveBeenCalledTimes(1);
      expect(cb).toHaveBeenCalledWith(
        expect.objectContaining({
          type: SafetyEventType.VIOLATION,
          details: { reason: 'speed' },
          severity: 'critical',
        }),
      );
    });

    it('does not deliver events of a different type', () => {
      const cb = vi.fn();
      bus.subscribe(SafetyEventType.VIOLATION, cb);
      bus.emit(SafetyEventType.ESTOP_ACTIVATED, { source: 'manual' }, 'critical');

      expect(cb).not.toHaveBeenCalled();
    });

    it('delivers all events to a wildcard subscriber', () => {
      const cb = vi.fn();
      bus.subscribe('*', cb);

      bus.emit(SafetyEventType.VIOLATION, {}, 'warning');
      bus.emit(SafetyEventType.ESTOP_ACTIVATED, {}, 'critical');
      bus.emit(SafetyEventType.GEOFENCE_WARNING, {}, 'info');

      expect(cb).toHaveBeenCalledTimes(3);
    });

    it('supports multiple subscribers on the same event type', () => {
      const cb1 = vi.fn();
      const cb2 = vi.fn();
      bus.subscribe(SafetyEventType.POLICY_CHANGED, cb1);
      bus.subscribe(SafetyEventType.POLICY_CHANGED, cb2);

      bus.emit(SafetyEventType.POLICY_CHANGED, { name: 'strict' }, 'info');

      expect(cb1).toHaveBeenCalledTimes(1);
      expect(cb2).toHaveBeenCalledTimes(1);
    });

    it('does not throw when emitting with no subscribers', () => {
      expect(() =>
        bus.emit(SafetyEventType.ROBOT_DISABLED, { reason: 'hw' }, 'critical'),
      ).not.toThrow();
    });

    it('catches subscriber errors without propagating', () => {
      const bad = vi.fn(() => {
        throw new Error('boom');
      });
      const good = vi.fn();

      bus.subscribe(SafetyEventType.VIOLATION, bad);
      bus.subscribe(SafetyEventType.VIOLATION, good);

      expect(() =>
        bus.emit(SafetyEventType.VIOLATION, {}, 'warning'),
      ).not.toThrow();
      expect(good).toHaveBeenCalledTimes(1);
    });
  });

  // ── unsubscribe ───────────────────────────────────────────────────

  describe('unsubscribe', () => {
    it('stops delivering events after unsubscribe', () => {
      const cb = vi.fn();
      const id = bus.subscribe(SafetyEventType.ESTOP_ACTIVATED, cb);

      bus.emit(SafetyEventType.ESTOP_ACTIVATED, {}, 'critical');
      expect(cb).toHaveBeenCalledTimes(1);

      bus.unsubscribe(id);
      bus.emit(SafetyEventType.ESTOP_ACTIVATED, {}, 'critical');
      expect(cb).toHaveBeenCalledTimes(1);
    });

    it('returns true when a valid subscription is removed', () => {
      const id = bus.subscribe(SafetyEventType.VIOLATION, vi.fn());
      expect(bus.unsubscribe(id)).toBe(true);
    });

    it('returns false for an unknown subscription ID', () => {
      expect(bus.unsubscribe('nonexistent-id')).toBe(false);
    });
  });

  // ── severity ──────────────────────────────────────────────────────

  describe('severity', () => {
    it('correctly sets info severity', () => {
      const cb = vi.fn();
      bus.subscribe(SafetyEventType.POLICY_CHANGED, cb);
      bus.emit(SafetyEventType.POLICY_CHANGED, {}, 'info');

      const event: SafetyEvent = cb.mock.calls[0][0];
      expect(event.severity).toBe('info');
    });

    it('correctly sets warning severity', () => {
      const cb = vi.fn();
      bus.subscribe(SafetyEventType.GEOFENCE_WARNING, cb);
      bus.emit(SafetyEventType.GEOFENCE_WARNING, { distance: 0.3 }, 'warning');

      const event: SafetyEvent = cb.mock.calls[0][0];
      expect(event.severity).toBe('warning');
    });

    it('correctly sets critical severity', () => {
      const cb = vi.fn();
      bus.subscribe(SafetyEventType.ESTOP_ACTIVATED, cb);
      bus.emit(SafetyEventType.ESTOP_ACTIVATED, {}, 'critical');

      const event: SafetyEvent = cb.mock.calls[0][0];
      expect(event.severity).toBe('critical');
    });
  });

  // ── timestamp ─────────────────────────────────────────────────────

  describe('timestamp', () => {
    it('sets a reasonable timestamp on emitted events', () => {
      const cb = vi.fn();
      bus.subscribe(SafetyEventType.VIOLATION, cb);

      const before = Date.now();
      bus.emit(SafetyEventType.VIOLATION, {}, 'warning');
      const after = Date.now();

      const event: SafetyEvent = cb.mock.calls[0][0];
      expect(event.timestamp).toBeGreaterThanOrEqual(before);
      expect(event.timestamp).toBeLessThanOrEqual(after);
    });
  });

  // ── getHistory ────────────────────────────────────────────────────

  describe('getHistory', () => {
    it('returns events in reverse chronological order', () => {
      bus.emit(SafetyEventType.VIOLATION, { seq: 1 }, 'warning');
      bus.emit(SafetyEventType.VIOLATION, { seq: 2 }, 'warning');
      bus.emit(SafetyEventType.VIOLATION, { seq: 3 }, 'warning');

      const history = bus.getHistory();
      expect(history).toHaveLength(3);
      expect(history[0].details.seq).toBe(3);
      expect(history[1].details.seq).toBe(2);
      expect(history[2].details.seq).toBe(1);
    });

    it('respects the limit parameter', () => {
      for (let i = 0; i < 10; i++) {
        bus.emit(SafetyEventType.VIOLATION, { i }, 'info');
      }

      const history = bus.getHistory(3);
      expect(history).toHaveLength(3);
      // Should be the 3 most recent (newest first)
      expect(history[0].details.i).toBe(9);
      expect(history[1].details.i).toBe(8);
      expect(history[2].details.i).toBe(7);
    });

    it('returns all events when no limit is given', () => {
      for (let i = 0; i < 20; i++) {
        bus.emit(SafetyEventType.RATE_LIMIT_HIT, { i }, 'warning');
      }
      expect(bus.getHistory()).toHaveLength(20);
    });

    it('caps stored history at maxHistory (500 default)', () => {
      const smallBus = new SafetyEventBus(10);
      for (let i = 0; i < 25; i++) {
        smallBus.emit(SafetyEventType.VIOLATION, { i }, 'info');
      }

      const history = smallBus.getHistory();
      expect(history).toHaveLength(10);
      // The oldest retained event should be i=15 (first 15 evicted)
      expect(history[history.length - 1].details.i).toBe(15);
    });
  });

  // ── getHistoryByType ──────────────────────────────────────────────

  describe('getHistoryByType', () => {
    it('filters events by type', () => {
      bus.emit(SafetyEventType.VIOLATION, { a: 1 }, 'warning');
      bus.emit(SafetyEventType.ESTOP_ACTIVATED, { b: 2 }, 'critical');
      bus.emit(SafetyEventType.VIOLATION, { c: 3 }, 'warning');
      bus.emit(SafetyEventType.GEOFENCE_BREACH, { d: 4 }, 'critical');

      const violations = bus.getHistoryByType(SafetyEventType.VIOLATION);
      expect(violations).toHaveLength(2);
      expect(violations[0].details.c).toBe(3); // newest first
      expect(violations[1].details.a).toBe(1);
    });

    it('respects the limit parameter', () => {
      for (let i = 0; i < 10; i++) {
        bus.emit(SafetyEventType.VELOCITY_CLAMPED, { i }, 'warning');
      }
      bus.emit(SafetyEventType.VIOLATION, {}, 'critical');

      const clamped = bus.getHistoryByType(SafetyEventType.VELOCITY_CLAMPED, 3);
      expect(clamped).toHaveLength(3);
    });

    it('returns empty array when no events of that type exist', () => {
      bus.emit(SafetyEventType.VIOLATION, {}, 'warning');
      const result = bus.getHistoryByType(SafetyEventType.ROBOT_DISABLED);
      expect(result).toEqual([]);
    });
  });

  // ── clearHistory ──────────────────────────────────────────────────

  describe('clearHistory', () => {
    it('removes all stored events', () => {
      bus.emit(SafetyEventType.VIOLATION, {}, 'warning');
      bus.emit(SafetyEventType.ESTOP_ACTIVATED, {}, 'critical');
      expect(bus.getHistory()).toHaveLength(2);

      bus.clearHistory();
      expect(bus.getHistory()).toHaveLength(0);
    });
  });

  // ── getSubscriberCount ────────────────────────────────────────────

  describe('getSubscriberCount', () => {
    it('returns 0 when no subscribers exist', () => {
      expect(bus.getSubscriberCount()).toBe(0);
    });

    it('counts all subscribers including wildcard', () => {
      bus.subscribe(SafetyEventType.VIOLATION, vi.fn());
      bus.subscribe(SafetyEventType.ESTOP_ACTIVATED, vi.fn());
      bus.subscribe('*', vi.fn());

      expect(bus.getSubscriberCount()).toBe(3);
    });

    it('decrements after unsubscribe', () => {
      const id = bus.subscribe(SafetyEventType.VIOLATION, vi.fn());
      bus.subscribe(SafetyEventType.ESTOP_ACTIVATED, vi.fn());
      expect(bus.getSubscriberCount()).toBe(2);

      bus.unsubscribe(id);
      expect(bus.getSubscriberCount()).toBe(1);
    });
  });

  // ── destroy ───────────────────────────────────────────────────────

  describe('destroy', () => {
    it('removes all subscribers and clears history', () => {
      bus.subscribe(SafetyEventType.VIOLATION, vi.fn());
      bus.subscribe('*', vi.fn());
      bus.emit(SafetyEventType.VIOLATION, {}, 'warning');
      bus.emit(SafetyEventType.ESTOP_ACTIVATED, {}, 'critical');

      bus.destroy();

      expect(bus.getSubscriberCount()).toBe(0);
      expect(bus.getHistory()).toHaveLength(0);
    });

    it('stops delivering events after destroy', () => {
      const cb = vi.fn();
      bus.subscribe(SafetyEventType.VIOLATION, cb);

      bus.destroy();
      bus.emit(SafetyEventType.VIOLATION, {}, 'warning');

      expect(cb).not.toHaveBeenCalled();
    });
  });
});

/**
 * Safety event publish/subscribe system.
 *
 * Provides a typed event bus for safety-related events (violations,
 * e-stop changes, geofence breaches, etc.) with subscription management,
 * bounded history, and wildcard support.
 */

import { randomUUID } from 'crypto';

/** Enumeration of all safety event types emitted by the system. */
export enum SafetyEventType {
  VIOLATION = 'safety.violation',
  ESTOP_ACTIVATED = 'safety.estop.activated',
  ESTOP_RELEASED = 'safety.estop.released',
  POLICY_CHANGED = 'safety.policy.changed',
  GEOFENCE_WARNING = 'safety.geofence.warning',
  GEOFENCE_BREACH = 'safety.geofence.breach',
  RATE_LIMIT_HIT = 'safety.rate_limit.hit',
  VELOCITY_CLAMPED = 'safety.velocity.clamped',
  COMMAND_BLOCKED = 'safety.command.blocked',
  ROBOT_DISABLED = 'safety.robot.disabled',
}

/** A safety event delivered to subscribers. */
export interface SafetyEvent {
  type: SafetyEventType;
  timestamp: number;
  details: Record<string, unknown>;
  severity: 'info' | 'warning' | 'critical';
}

export type SafetyEventCallback = (event: SafetyEvent) => void;

interface Subscription {
  id: string;
  type: SafetyEventType | '*';
  callback: SafetyEventCallback;
}

/**
 * Centralized event bus for safety system events.
 *
 * Supports subscribing to specific event types or all events via wildcard ('*').
 * Maintains a bounded event history (default 500) for retrospective queries.
 */
export class SafetyEventBus {
  private subscriptions = new Map<string, Subscription>();
  private history: SafetyEvent[] = [];
  private readonly maxHistory: number;

  constructor(maxHistory = 500) {
    this.maxHistory = maxHistory;
  }

  /**
   * Subscribe to a specific event type or all events ('*').
   * @returns A unique subscription ID used for unsubscribing.
   */
  subscribe(type: SafetyEventType | '*', callback: SafetyEventCallback): string {
    const id = randomUUID();
    this.subscriptions.set(id, { id, type, callback });
    return id;
  }

  /**
   * Remove a subscription by its ID.
   * @returns true if the subscription existed and was removed, false otherwise.
   */
  unsubscribe(id: string): boolean {
    return this.subscriptions.delete(id);
  }

  /**
   * Emit a safety event to all matching subscribers.
   * The event is recorded in history and then dispatched to subscribers
   * whose type matches or who subscribed with '*'. Listener errors are
   * caught so they never propagate to the emitter.
   */
  emit(
    type: SafetyEventType,
    details: Record<string, unknown>,
    severity: SafetyEvent['severity'],
  ): void {
    const event: SafetyEvent = {
      type,
      timestamp: Date.now(),
      details,
      severity,
    };

    this.history.push(event);
    if (this.history.length > this.maxHistory) {
      this.history = this.history.slice(-this.maxHistory);
    }

    for (const sub of this.subscriptions.values()) {
      if (sub.type === type || sub.type === '*') {
        try {
          sub.callback(event);
        } catch {
          // Don't let subscriber errors propagate
        }
      }
    }
  }

  /**
   * Get recent events in reverse chronological order (newest first).
   * @param limit - Maximum number of events to return (defaults to all stored).
   */
  getHistory(limit?: number): SafetyEvent[] {
    const events = [...this.history].reverse();
    if (limit !== undefined && limit > 0) {
      return events.slice(0, limit);
    }
    return events;
  }

  /**
   * Get recent events of a specific type in reverse chronological order.
   * @param type - The event type to filter by.
   * @param limit - Maximum number of events to return.
   */
  getHistoryByType(type: SafetyEventType, limit?: number): SafetyEvent[] {
    const events = this.history.filter(e => e.type === type).reverse();
    if (limit !== undefined && limit > 0) {
      return events.slice(0, limit);
    }
    return events;
  }

  /** Remove all stored event history. */
  clearHistory(): void {
    this.history = [];
  }

  /** Return the total number of active subscribers. */
  getSubscriberCount(): number {
    return this.subscriptions.size;
  }

  /** Remove all subscribers and clear event history. */
  destroy(): void {
    this.subscriptions.clear();
    this.history = [];
  }
}

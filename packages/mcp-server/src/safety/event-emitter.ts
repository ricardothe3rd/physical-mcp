/**
 * Safety event emitter — subscribe to safety violations and state changes.
 *
 * Enables external systems and MCP clients to receive real-time
 * notifications about safety events (violations, e-stop, policy changes).
 */

export type SafetyEventType =
  | 'violation'
  | 'emergency_stop_activated'
  | 'emergency_stop_released'
  | 'policy_updated'
  | 'deadman_timeout'
  | 'approval_requested'
  | 'approval_resolved';

export interface SafetyEvent {
  type: SafetyEventType;
  timestamp: number;
  data: Record<string, unknown>;
}

export type SafetyEventListener = (event: SafetyEvent) => void;

export class SafetyEventEmitter {
  private listeners = new Map<SafetyEventType, Set<SafetyEventListener>>();
  private allListeners = new Set<SafetyEventListener>();
  private history: SafetyEvent[] = [];
  private maxHistory: number;

  constructor(maxHistory = 100) {
    this.maxHistory = maxHistory;
  }

  /** Subscribe to a specific event type. Returns an unsubscribe function. */
  on(type: SafetyEventType, listener: SafetyEventListener): () => void {
    let set = this.listeners.get(type);
    if (!set) {
      set = new Set();
      this.listeners.set(type, set);
    }
    set.add(listener);
    return () => set!.delete(listener);
  }

  /** Subscribe to all events. Returns an unsubscribe function. */
  onAll(listener: SafetyEventListener): () => void {
    this.allListeners.add(listener);
    return () => this.allListeners.delete(listener);
  }

  /** Emit an event to all matching listeners. */
  emit(type: SafetyEventType, data: Record<string, unknown> = {}): void {
    const event: SafetyEvent = { type, timestamp: Date.now(), data };

    this.history.push(event);
    if (this.history.length > this.maxHistory) {
      this.history.shift();
    }

    const typeListeners = this.listeners.get(type);
    if (typeListeners) {
      for (const listener of typeListeners) {
        try {
          listener(event);
        } catch {
          // Don't let listener errors propagate
        }
      }
    }

    for (const listener of this.allListeners) {
      try {
        listener(event);
      } catch {
        // Don't let listener errors propagate
      }
    }
  }

  /** Get recent events. */
  getHistory(limit?: number, type?: SafetyEventType): SafetyEvent[] {
    let events = type ? this.history.filter(e => e.type === type) : [...this.history];
    if (limit && limit > 0) {
      events = events.slice(-limit);
    }
    return events;
  }

  /** Get count of listeners for a given type. */
  listenerCount(type?: SafetyEventType): number {
    if (type) {
      return (this.listeners.get(type)?.size || 0) + this.allListeners.size;
    }
    let total = this.allListeners.size;
    for (const set of this.listeners.values()) {
      total += set.size;
    }
    return total;
  }

  /** Remove all listeners. */
  removeAllListeners(): void {
    this.listeners.clear();
    this.allListeners.clear();
  }

  /** Clear event history. */
  clearHistory(): void {
    this.history = [];
  }
}

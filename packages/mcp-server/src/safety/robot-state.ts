/**
 * Robot state machine for tracking operational state.
 *
 * States:
 * - idle: Robot is stationary, ready for commands
 * - moving: Robot is executing a velocity command or action goal
 * - e_stopped: Emergency stop is active
 * - error: Bridge disconnected or unrecoverable error
 */

export type RobotState = 'idle' | 'moving' | 'e_stopped' | 'error';

export interface StateTransition {
  from: RobotState;
  to: RobotState;
  reason: string;
  timestamp: number;
}

export type StateChangeCallback = (transition: StateTransition) => void;

export class RobotStateMachine {
  private state: RobotState = 'idle';
  private lastTransition: StateTransition | null = null;
  private history: StateTransition[] = [];
  private maxHistory: number;
  private listeners: StateChangeCallback[] = [];

  constructor(maxHistory = 100) {
    this.maxHistory = maxHistory;
  }

  get currentState(): RobotState {
    return this.state;
  }

  get lastChange(): StateTransition | null {
    return this.lastTransition;
  }

  transition(to: RobotState, reason: string): boolean {
    if (to === this.state) return false;

    // Validate transitions
    if (!this.isValidTransition(this.state, to)) {
      return false;
    }

    const transition: StateTransition = {
      from: this.state,
      to,
      reason,
      timestamp: Date.now(),
    };

    this.state = to;
    this.lastTransition = transition;
    this.history.push(transition);

    if (this.history.length > this.maxHistory) {
      this.history = this.history.slice(-this.maxHistory);
    }

    for (const listener of this.listeners) {
      try {
        listener(transition);
      } catch {
        // Listener errors should not affect state machine
      }
    }

    return true;
  }

  private isValidTransition(from: RobotState, to: RobotState): boolean {
    // E-stop can be entered from any state
    if (to === 'e_stopped') return true;
    // Error can be entered from any state
    if (to === 'error') return true;

    switch (from) {
      case 'idle':
        return to === 'moving';
      case 'moving':
        return to === 'idle';
      case 'e_stopped':
        return to === 'idle';
      case 'error':
        return to === 'idle';
      default:
        return false;
    }
  }

  onStateChange(callback: StateChangeCallback): () => void {
    this.listeners.push(callback);
    return () => {
      this.listeners = this.listeners.filter(l => l !== callback);
    };
  }

  getHistory(limit?: number): StateTransition[] {
    const result = [...this.history].reverse();
    return limit ? result.slice(0, limit) : result;
  }

  getStatus(): {
    state: RobotState;
    lastTransition: StateTransition | null;
    uptime: number;
    transitionCount: number;
  } {
    return {
      state: this.state,
      lastTransition: this.lastTransition,
      uptime: this.lastTransition ? Date.now() - this.lastTransition.timestamp : 0,
      transitionCount: this.history.length,
    };
  }
}

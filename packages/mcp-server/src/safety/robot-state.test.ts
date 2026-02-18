import { describe, it, expect, beforeEach, vi } from 'vitest';
import { RobotStateMachine, type StateTransition } from './robot-state.js';

describe('RobotStateMachine', () => {
  let sm: RobotStateMachine;

  beforeEach(() => {
    sm = new RobotStateMachine();
  });

  it('starts in idle state', () => {
    expect(sm.currentState).toBe('idle');
  });

  it('transitions from idle to moving', () => {
    const result = sm.transition('moving', 'cmd_vel published');
    expect(result).toBe(true);
    expect(sm.currentState).toBe('moving');
  });

  it('transitions from moving to idle', () => {
    sm.transition('moving', 'start');
    const result = sm.transition('idle', 'zero velocity');
    expect(result).toBe(true);
    expect(sm.currentState).toBe('idle');
  });

  it('transitions to e_stopped from any state', () => {
    expect(sm.transition('e_stopped', 'manual stop')).toBe(true);
    expect(sm.currentState).toBe('e_stopped');
  });

  it('transitions to e_stopped from moving', () => {
    sm.transition('moving', 'start');
    expect(sm.transition('e_stopped', 'safety violation')).toBe(true);
    expect(sm.currentState).toBe('e_stopped');
  });

  it('transitions from e_stopped to idle', () => {
    sm.transition('e_stopped', 'stop');
    expect(sm.transition('idle', 'e-stop released')).toBe(true);
    expect(sm.currentState).toBe('idle');
  });

  it('does not transition from e_stopped to moving directly', () => {
    sm.transition('e_stopped', 'stop');
    expect(sm.transition('moving', 'attempt')).toBe(false);
    expect(sm.currentState).toBe('e_stopped');
  });

  it('transitions to error from any state', () => {
    expect(sm.transition('error', 'bridge disconnected')).toBe(true);
    expect(sm.currentState).toBe('error');
  });

  it('transitions from error to idle (recovery)', () => {
    sm.transition('error', 'disconnected');
    expect(sm.transition('idle', 'reconnected')).toBe(true);
    expect(sm.currentState).toBe('idle');
  });

  it('does not transition to same state', () => {
    expect(sm.transition('idle', 'already idle')).toBe(false);
  });

  it('does not allow invalid transition idle -> idle', () => {
    expect(sm.transition('idle', 'noop')).toBe(false);
  });

  it('records transition history', () => {
    sm.transition('moving', 'start');
    sm.transition('idle', 'stop');
    sm.transition('e_stopped', 'emergency');

    const history = sm.getHistory();
    expect(history).toHaveLength(3);
    expect(history[0].to).toBe('e_stopped'); // newest first
    expect(history[2].to).toBe('moving');    // oldest last
  });

  it('limits history size', () => {
    const smallSm = new RobotStateMachine(3);
    smallSm.transition('moving', '1');
    smallSm.transition('idle', '2');
    smallSm.transition('moving', '3');
    smallSm.transition('idle', '4');
    smallSm.transition('moving', '5');

    const history = smallSm.getHistory();
    expect(history.length).toBeLessThanOrEqual(3);
  });

  it('calls state change listeners', () => {
    const listener = vi.fn();
    sm.onStateChange(listener);

    sm.transition('moving', 'start');
    expect(listener).toHaveBeenCalledTimes(1);
    expect(listener).toHaveBeenCalledWith(expect.objectContaining({
      from: 'idle',
      to: 'moving',
      reason: 'start',
    }));
  });

  it('allows removing listeners', () => {
    const listener = vi.fn();
    const unsubscribe = sm.onStateChange(listener);

    sm.transition('moving', 'start');
    expect(listener).toHaveBeenCalledTimes(1);

    unsubscribe();
    sm.transition('idle', 'stop');
    expect(listener).toHaveBeenCalledTimes(1); // not called again
  });

  it('getStatus returns current state info', () => {
    sm.transition('moving', 'start');

    const status = sm.getStatus();
    expect(status.state).toBe('moving');
    expect(status.lastTransition).not.toBeNull();
    expect(status.transitionCount).toBe(1);
    expect(status.uptime).toBeGreaterThanOrEqual(0);
  });

  it('getStatus returns null lastTransition for fresh machine', () => {
    const status = sm.getStatus();
    expect(status.lastTransition).toBeNull();
    expect(status.transitionCount).toBe(0);
  });

  it('getHistory respects limit parameter', () => {
    sm.transition('moving', '1');
    sm.transition('idle', '2');
    sm.transition('e_stopped', '3');

    const history = sm.getHistory(2);
    expect(history).toHaveLength(2);
  });

  it('listener errors do not break state machine', () => {
    sm.onStateChange(() => {
      throw new Error('listener error');
    });

    // Should not throw
    expect(() => sm.transition('moving', 'start')).not.toThrow();
    expect(sm.currentState).toBe('moving');
  });
});

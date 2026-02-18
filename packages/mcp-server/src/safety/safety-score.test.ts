import { describe, it, expect, beforeEach } from 'vitest';
import { SafetyScoreTracker } from './safety-score.js';

describe('SafetyScoreTracker', () => {
  let tracker: SafetyScoreTracker;

  beforeEach(() => {
    tracker = new SafetyScoreTracker();
  });

  it('starts with perfect score', () => {
    const snapshot = tracker.getSnapshot();
    expect(snapshot.totalCommands).toBe(0);
    expect(snapshot.safetyScore).toBe(100);
    expect(snapshot.blockRate).toBe(0);
  });

  it('tracks allowed commands', () => {
    tracker.recordAllowed();
    tracker.recordAllowed();
    tracker.recordAllowed();

    const snapshot = tracker.getSnapshot();
    expect(snapshot.totalCommands).toBe(3);
    expect(snapshot.allowedCommands).toBe(3);
    expect(snapshot.blockedCommands).toBe(0);
    expect(snapshot.safetyScore).toBe(100);
  });

  it('tracks blocked commands', () => {
    tracker.recordBlocked(['velocity_exceeded']);

    const snapshot = tracker.getSnapshot();
    expect(snapshot.totalCommands).toBe(1);
    expect(snapshot.blockedCommands).toBe(1);
    expect(snapshot.safetyScore).toBe(0);
    expect(snapshot.blockRate).toBe(1);
  });

  it('calculates mixed score correctly', () => {
    // 7 allowed, 3 blocked = 70% safety score
    for (let i = 0; i < 7; i++) tracker.recordAllowed();
    for (let i = 0; i < 3; i++) tracker.recordBlocked(['velocity_exceeded']);

    const snapshot = tracker.getSnapshot();
    expect(snapshot.totalCommands).toBe(10);
    expect(snapshot.allowedCommands).toBe(7);
    expect(snapshot.blockedCommands).toBe(3);
    expect(snapshot.safetyScore).toBe(70);
    expect(snapshot.blockRate).toBe(0.3);
  });

  it('tracks violation types', () => {
    tracker.recordBlocked(['velocity_exceeded']);
    tracker.recordBlocked(['velocity_exceeded', 'rate_limit_exceeded']);
    tracker.recordBlocked(['blocked_topic']);

    const snapshot = tracker.getSnapshot();
    expect(snapshot.violationsByType.velocity_exceeded).toBe(2);
    expect(snapshot.violationsByType.rate_limit_exceeded).toBe(1);
    expect(snapshot.violationsByType.blocked_topic).toBe(1);
  });

  it('records session start time', () => {
    const snapshot = tracker.getSnapshot();
    expect(snapshot.sessionStartedAt).toBeGreaterThan(0);
    expect(snapshot.sessionDurationMs).toBeGreaterThanOrEqual(0);
  });

  it('resets all tracking', () => {
    tracker.recordAllowed();
    tracker.recordBlocked(['velocity_exceeded']);

    tracker.reset();

    const snapshot = tracker.getSnapshot();
    expect(snapshot.totalCommands).toBe(0);
    expect(snapshot.blockedCommands).toBe(0);
    expect(snapshot.safetyScore).toBe(100);
    expect(Object.keys(snapshot.violationsByType)).toHaveLength(0);
  });

  it('handles multiple violation types per block', () => {
    tracker.recordBlocked(['emergency_stop_active', 'blocked_topic', 'velocity_exceeded']);

    const snapshot = tracker.getSnapshot();
    expect(snapshot.blockedCommands).toBe(1); // 1 command blocked
    expect(Object.keys(snapshot.violationsByType)).toHaveLength(3); // 3 violation types
  });

  it('rounds block rate to 3 decimals', () => {
    // 1 blocked out of 3 = 0.333...
    tracker.recordAllowed();
    tracker.recordAllowed();
    tracker.recordBlocked(['velocity_exceeded']);

    const snapshot = tracker.getSnapshot();
    expect(snapshot.blockRate).toBe(0.333);
  });
});

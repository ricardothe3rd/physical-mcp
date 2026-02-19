import { describe, it, expect, beforeEach, vi } from 'vitest';
import { MultiRobotManager } from './multi-robot.js';
import { getDefaultPolicy } from './policy-loader.js';
import type { SafetyPolicy } from './types.js';

function makePolicy(overrides: Partial<SafetyPolicy> = {}): SafetyPolicy {
  return { ...getDefaultPolicy(), ...overrides };
}

describe('MultiRobotManager', () => {
  let manager: MultiRobotManager;

  beforeEach(() => {
    manager = new MultiRobotManager();
  });

  // ── Registration ──────────────────────────────────────────────────

  it('registers a robot and returns its profile', () => {
    const profile = manager.registerRobot('r1', 'Robot One', '/robot1');

    expect(profile.robotId).toBe('r1');
    expect(profile.name).toBe('Robot One');
    expect(profile.namespace).toBe('/robot1');
    expect(profile.enabled).toBe(true);
    expect(profile.lastCommandAt).toBeNull();
    expect(profile.registeredAt).toBeGreaterThan(0);
    expect(profile.policy.name).toBe('default');
  });

  it('registers a robot with a custom policy', () => {
    const custom = makePolicy({ name: 'strict', velocity: { linearMax: 0.1, angularMax: 0.5, clampMode: true } });
    const profile = manager.registerRobot('r1', 'Robot One', '/robot1', custom);

    expect(profile.policy.name).toBe('strict');
    expect(profile.policy.velocity.linearMax).toBe(0.1);
  });

  it('uses a conservative default policy when none is provided', () => {
    const profile = manager.registerRobot('r1', 'Robot One', '/robot1');
    const defaultPolicy = getDefaultPolicy();

    expect(profile.policy.velocity.linearMax).toBe(defaultPolicy.velocity.linearMax);
    expect(profile.policy.velocity.angularMax).toBe(defaultPolicy.velocity.angularMax);
  });

  it('updates an existing robot when re-registered with the same ID', () => {
    manager.registerRobot('r1', 'Old Name', '/old');
    const updated = manager.registerRobot('r1', 'New Name', '/new');

    expect(updated.name).toBe('New Name');
    expect(updated.namespace).toBe('/new');
    // Only one robot should exist
    expect(manager.getRobotCount()).toBe(1);
  });

  it('preserves registeredAt and lastCommandAt when re-registering', () => {
    const first = manager.registerRobot('r1', 'Robot One', '/robot1');
    manager.recordCommand('r1');
    const second = manager.registerRobot('r1', 'Robot One Updated', '/robot1');

    expect(second.registeredAt).toBe(first.registeredAt);
    expect(second.lastCommandAt).not.toBeNull();
  });

  // ── Unregistration ────────────────────────────────────────────────

  it('unregisters an existing robot and returns true', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    expect(manager.unregisterRobot('r1')).toBe(true);
    expect(manager.getRobot('r1')).toBeNull();
    expect(manager.getRobotCount()).toBe(0);
  });

  it('returns false when unregistering a non-existing robot', () => {
    expect(manager.unregisterRobot('ghost')).toBe(false);
  });

  // ── Get robot ─────────────────────────────────────────────────────

  it('returns a profile for an existing robot', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    const profile = manager.getRobot('r1');

    expect(profile).not.toBeNull();
    expect(profile!.robotId).toBe('r1');
  });

  it('returns null for a non-existing robot', () => {
    expect(manager.getRobot('ghost')).toBeNull();
  });

  it('returns a copy that does not affect internal state when mutated', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    const copy = manager.getRobot('r1')!;
    copy.name = 'MUTATED';
    copy.policy.velocity.linearMax = 999;

    const original = manager.getRobot('r1')!;
    expect(original.name).toBe('Robot One');
    expect(original.policy.velocity.linearMax).toBe(getDefaultPolicy().velocity.linearMax);
  });

  // ── List robots ───────────────────────────────────────────────────

  it('lists all registered robots', () => {
    manager.registerRobot('r1', 'One', '/r1');
    manager.registerRobot('r2', 'Two', '/r2');
    manager.registerRobot('r3', 'Three', '/r3');

    const list = manager.listRobots();
    expect(list).toHaveLength(3);

    const ids = list.map((r) => r.robotId).sort();
    expect(ids).toEqual(['r1', 'r2', 'r3']);
  });

  it('returns an empty array when no robots are registered', () => {
    expect(manager.listRobots()).toHaveLength(0);
  });

  // ── Enable / disable ─────────────────────────────────────────────

  it('disables a robot', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    expect(manager.disableRobot('r1')).toBe(true);

    const profile = manager.getRobot('r1')!;
    expect(profile.enabled).toBe(false);
  });

  it('enables a previously disabled robot', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    manager.disableRobot('r1');
    expect(manager.enableRobot('r1')).toBe(true);

    const profile = manager.getRobot('r1')!;
    expect(profile.enabled).toBe(true);
  });

  it('returns false when enabling a non-existing robot', () => {
    expect(manager.enableRobot('ghost')).toBe(false);
  });

  it('returns false when disabling a non-existing robot', () => {
    expect(manager.disableRobot('ghost')).toBe(false);
  });

  // ── Policy management ─────────────────────────────────────────────

  it('returns the robot-specific policy via getPolicy', () => {
    const custom = makePolicy({ name: 'custom-bot' });
    manager.registerRobot('r1', 'Robot One', '/robot1', custom);

    const policy = manager.getPolicy('r1');
    expect(policy).not.toBeNull();
    expect(policy!.name).toBe('custom-bot');
  });

  it('returns null from getPolicy for a non-existing robot', () => {
    expect(manager.getPolicy('ghost')).toBeNull();
  });

  it('updates a robot policy via updatePolicy', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    const newPolicy = makePolicy({ name: 'updated-policy', velocity: { linearMax: 2.0, angularMax: 3.0, clampMode: true } });

    expect(manager.updatePolicy('r1', newPolicy)).toBe(true);

    const policy = manager.getPolicy('r1')!;
    expect(policy.name).toBe('updated-policy');
    expect(policy.velocity.linearMax).toBe(2.0);
  });

  it('returns false from updatePolicy for a non-existing robot', () => {
    const policy = getDefaultPolicy();
    expect(manager.updatePolicy('ghost', policy)).toBe(false);
  });

  it('deep-clones the policy on update so external mutations are safe', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    const newPolicy = makePolicy({ name: 'safe' });
    manager.updatePolicy('r1', newPolicy);

    // Mutate the object we passed in
    newPolicy.name = 'MUTATED';
    newPolicy.velocity.linearMax = 999;

    const stored = manager.getPolicy('r1')!;
    expect(stored.name).toBe('safe');
    expect(stored.velocity.linearMax).toBe(getDefaultPolicy().velocity.linearMax);
  });

  // ── Namespace resolution ──────────────────────────────────────────

  it('resolves namespace + topic correctly ("/robot1" + "/cmd_vel")', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    expect(manager.resolveNamespace('r1', '/cmd_vel')).toBe('/robot1/cmd_vel');
  });

  it('resolves empty namespace returning topic unchanged', () => {
    manager.registerRobot('r1', 'Robot One', '');
    expect(manager.resolveNamespace('r1', '/cmd_vel')).toBe('/cmd_vel');
  });

  it('resolves topic without leading slash ("/robot1" + "cmd_vel")', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    expect(manager.resolveNamespace('r1', 'cmd_vel')).toBe('/robot1/cmd_vel');
  });

  it('collapses double slashes ("/robot1/" + "/cmd_vel")', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1/');
    expect(manager.resolveNamespace('r1', '/cmd_vel')).toBe('/robot1/cmd_vel');
  });

  it('handles empty namespace with topic without leading slash', () => {
    manager.registerRobot('r1', 'Robot One', '');
    expect(manager.resolveNamespace('r1', 'cmd_vel')).toBe('/cmd_vel');
  });

  it('throws when resolving namespace for a non-existing robot', () => {
    expect(() => manager.resolveNamespace('ghost', '/cmd_vel')).toThrow(
      'Robot "ghost" is not registered',
    );
  });

  // ── Command allowed ───────────────────────────────────────────────

  it('allows commands when robot is enabled', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    const result = manager.isCommandAllowed('r1');

    expect(result.allowed).toBe(true);
    expect(result.reason).toBeUndefined();
  });

  it('blocks commands when robot is disabled', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    manager.disableRobot('r1');
    const result = manager.isCommandAllowed('r1');

    expect(result.allowed).toBe(false);
    expect(result.reason).toContain('disabled');
  });

  it('blocks commands for a non-existing robot', () => {
    const result = manager.isCommandAllowed('ghost');

    expect(result.allowed).toBe(false);
    expect(result.reason).toContain('not registered');
  });

  // ── Record command ────────────────────────────────────────────────

  it('updates lastCommandAt when recording a command', () => {
    manager.registerRobot('r1', 'Robot One', '/robot1');
    expect(manager.getRobot('r1')!.lastCommandAt).toBeNull();

    const before = Date.now();
    manager.recordCommand('r1');
    const after = Date.now();

    const profile = manager.getRobot('r1')!;
    expect(profile.lastCommandAt).toBeGreaterThanOrEqual(before);
    expect(profile.lastCommandAt).toBeLessThanOrEqual(after);
  });

  it('does nothing when recording a command for a non-existing robot', () => {
    // Should not throw
    expect(() => manager.recordCommand('ghost')).not.toThrow();
  });

  // ── Robot count ───────────────────────────────────────────────────

  it('returns correct robot count', () => {
    expect(manager.getRobotCount()).toBe(0);
    manager.registerRobot('r1', 'One', '/r1');
    expect(manager.getRobotCount()).toBe(1);
    manager.registerRobot('r2', 'Two', '/r2');
    expect(manager.getRobotCount()).toBe(2);
    manager.unregisterRobot('r1');
    expect(manager.getRobotCount()).toBe(1);
  });

  // ── Active robots ─────────────────────────────────────────────────

  it('returns only robots with commands in the last 60 seconds', () => {
    manager.registerRobot('r1', 'One', '/r1');
    manager.registerRobot('r2', 'Two', '/r2');
    manager.registerRobot('r3', 'Three', '/r3');

    manager.recordCommand('r1');
    manager.recordCommand('r3');

    const active = manager.getActiveRobots();
    expect(active).toHaveLength(2);

    const ids = active.map((r) => r.robotId).sort();
    expect(ids).toEqual(['r1', 'r3']);
  });

  it('excludes robots whose last command is older than 60 seconds', () => {
    manager.registerRobot('r1', 'One', '/r1');
    manager.recordCommand('r1');

    // Manually override lastCommandAt to simulate an old command
    vi.spyOn(Date, 'now').mockReturnValue(Date.now() + 61_000);

    const active = manager.getActiveRobots();
    // r1's lastCommandAt is ~61s in the past from the mocked "now"
    expect(active).toHaveLength(0);

    vi.restoreAllMocks();
  });

  it('returns empty array when no robots have received commands', () => {
    manager.registerRobot('r1', 'One', '/r1');
    manager.registerRobot('r2', 'Two', '/r2');

    expect(manager.getActiveRobots()).toHaveLength(0);
  });

  // ── Max robot limit ───────────────────────────────────────────────

  it('enforces a maximum of 20 registered robots', () => {
    for (let i = 0; i < 20; i++) {
      manager.registerRobot(`r${i}`, `Robot ${i}`, `/r${i}`);
    }
    expect(manager.getRobotCount()).toBe(20);

    expect(() => manager.registerRobot('r20', 'Robot 20', '/r20')).toThrow(
      /Maximum robot limit/,
    );
    expect(manager.getRobotCount()).toBe(20);
  });

  it('allows re-registration when at max limit (does not increase count)', () => {
    for (let i = 0; i < 20; i++) {
      manager.registerRobot(`r${i}`, `Robot ${i}`, `/r${i}`);
    }

    // Re-registering an existing robot should NOT throw
    expect(() =>
      manager.registerRobot('r0', 'Robot Zero Updated', '/r0-new'),
    ).not.toThrow();
    expect(manager.getRobotCount()).toBe(20);

    const updated = manager.getRobot('r0')!;
    expect(updated.name).toBe('Robot Zero Updated');
  });

  it('allows registering after unregistering when at max limit', () => {
    for (let i = 0; i < 20; i++) {
      manager.registerRobot(`r${i}`, `Robot ${i}`, `/r${i}`);
    }
    manager.unregisterRobot('r0');

    expect(() =>
      manager.registerRobot('r20', 'Robot 20', '/r20'),
    ).not.toThrow();
    expect(manager.getRobotCount()).toBe(20);
  });

  // ── Edge cases ────────────────────────────────────────────────────

  it('handles nested namespace resolution ("/fleet/robot1" + "/cmd_vel")', () => {
    manager.registerRobot('r1', 'Robot One', '/fleet/robot1');
    expect(manager.resolveNamespace('r1', '/cmd_vel')).toBe('/fleet/robot1/cmd_vel');
  });

  it('different robots can have different policies', () => {
    const fastPolicy = makePolicy({ name: 'fast', velocity: { linearMax: 5.0, angularMax: 10.0, clampMode: false } });
    const slowPolicy = makePolicy({ name: 'slow', velocity: { linearMax: 0.1, angularMax: 0.3, clampMode: true } });

    manager.registerRobot('fast-bot', 'Fast Bot', '/fast', fastPolicy);
    manager.registerRobot('slow-bot', 'Slow Bot', '/slow', slowPolicy);

    expect(manager.getPolicy('fast-bot')!.velocity.linearMax).toBe(5.0);
    expect(manager.getPolicy('slow-bot')!.velocity.linearMax).toBe(0.1);
  });
});

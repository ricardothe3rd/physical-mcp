import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyVersionManager } from './policy-versioning.js';
import { getDefaultPolicy } from './policy-loader.js';
import type { SafetyPolicy } from './types.js';

function makePolicy(overrides: Partial<SafetyPolicy> = {}): SafetyPolicy {
  return { ...getDefaultPolicy(), ...overrides };
}

describe('PolicyVersionManager', () => {
  let manager: PolicyVersionManager;

  beforeEach(() => {
    manager = new PolicyVersionManager();
  });

  // ── Recording versions ─────────────────────────────────────────────

  it('records an initial version and returns version number 1', () => {
    const policy = getDefaultPolicy();
    const version = manager.recordVersion(policy, 'Initial policy');
    expect(version).toBe(1);
  });

  it('records subsequent versions with incrementing numbers', () => {
    const p1 = getDefaultPolicy();
    const p2 = makePolicy({ name: 'updated' });
    const p3 = makePolicy({ name: 'updated-again' });

    expect(manager.recordVersion(p1, 'v1')).toBe(1);
    expect(manager.recordVersion(p2, 'v2')).toBe(2);
    expect(manager.recordVersion(p3, 'v3')).toBe(3);
  });

  it('records the reason for each change', () => {
    manager.recordVersion(getDefaultPolicy(), 'Bootstrap');
    manager.recordVersion(makePolicy({ name: 'v2' }), 'Tighten velocity');

    const versions = manager.getVersions();
    expect(versions[0].reason).toBe('Tighten velocity');
    expect(versions[1].reason).toBe('Bootstrap');
  });

  // ── Changed-field tracking ─────────────────────────────────────────

  it('lists all top-level fields as changed for the first version', () => {
    const policy = getDefaultPolicy();
    manager.recordVersion(policy, 'Initial');

    const v = manager.getVersion(1);
    expect(v).not.toBeNull();
    // The first version should list every top-level key as "changed"
    const policyKeys = Object.keys(policy).sort();
    expect(v!.changedFields).toEqual(policyKeys);
  });

  it('tracks changed fields correctly for subsequent versions', () => {
    const p1 = getDefaultPolicy();
    const p2 = makePolicy({ name: 'updated-name', description: 'new desc' });

    manager.recordVersion(p1, 'Initial');
    manager.recordVersion(p2, 'Rename');

    const v2 = manager.getVersion(2);
    expect(v2).not.toBeNull();
    expect(v2!.changedFields).toContain('name');
    expect(v2!.changedFields).toContain('description');
    // Fields that did NOT change should not appear
    expect(v2!.changedFields).not.toContain('velocity');
    expect(v2!.changedFields).not.toContain('geofence');
  });

  it('detects nested object field changes', () => {
    const p1 = getDefaultPolicy();
    const p2 = makePolicy();
    p2.velocity = { ...p2.velocity, linearMax: 9.9 };

    manager.recordVersion(p1, 'Initial');
    manager.recordVersion(p2, 'Increase speed');

    const v2 = manager.getVersion(2);
    expect(v2!.changedFields).toContain('velocity');
    expect(v2!.changedFields).not.toContain('name');
  });

  // ── Retrieving versions ────────────────────────────────────────────

  it('retrieves a specific version by number', () => {
    manager.recordVersion(makePolicy({ name: 'alpha' }), 'First');
    manager.recordVersion(makePolicy({ name: 'beta' }), 'Second');

    const v1 = manager.getVersion(1);
    const v2 = manager.getVersion(2);

    expect(v1).not.toBeNull();
    expect(v1!.policy.name).toBe('alpha');

    expect(v2).not.toBeNull();
    expect(v2!.policy.name).toBe('beta');
  });

  it('returns null for a nonexistent version number', () => {
    manager.recordVersion(getDefaultPolicy(), 'Initial');
    expect(manager.getVersion(999)).toBeNull();
  });

  it('lists versions newest-first', () => {
    manager.recordVersion(makePolicy({ name: 'first' }), 'v1');
    manager.recordVersion(makePolicy({ name: 'second' }), 'v2');
    manager.recordVersion(makePolicy({ name: 'third' }), 'v3');

    const versions = manager.getVersions();
    expect(versions).toHaveLength(3);
    expect(versions[0].version).toBe(3);
    expect(versions[1].version).toBe(2);
    expect(versions[2].version).toBe(1);
  });

  // ── Diff ───────────────────────────────────────────────────────────

  it('diff shows field changes between two versions', () => {
    const p1 = getDefaultPolicy();
    const p2 = makePolicy({
      name: 'strict',
      velocity: { linearMax: 0.1, angularMax: 0.5, clampMode: true },
    });

    manager.recordVersion(p1, 'Initial');
    manager.recordVersion(p2, 'Strict mode');

    const diffs = manager.diff(1, 2);
    expect(diffs).not.toBeNull();
    expect(diffs!.length).toBeGreaterThan(0);

    const nameChange = diffs!.find((d) => d.field === 'name');
    expect(nameChange).toBeDefined();
    expect(nameChange!.before).toBe('default');
    expect(nameChange!.after).toBe('strict');

    const velocityChange = diffs!.find((d) => d.field === 'velocity');
    expect(velocityChange).toBeDefined();
    expect(velocityChange!.before).toEqual(p1.velocity);
    expect(velocityChange!.after).toEqual(p2.velocity);
  });

  it('diff returns empty array when versions are identical', () => {
    const policy = getDefaultPolicy();
    manager.recordVersion(policy, 'First');
    manager.recordVersion(policy, 'Duplicate');

    const diffs = manager.diff(1, 2);
    expect(diffs).not.toBeNull();
    expect(diffs).toHaveLength(0);
  });

  it('diff returns null when a version does not exist', () => {
    manager.recordVersion(getDefaultPolicy(), 'Initial');
    expect(manager.diff(1, 999)).toBeNull();
    expect(manager.diff(888, 1)).toBeNull();
    expect(manager.diff(888, 999)).toBeNull();
  });

  // ── Rollback ───────────────────────────────────────────────────────

  it('rollback returns the correct policy for a given version', () => {
    const p1 = makePolicy({ name: 'original' });
    const p2 = makePolicy({ name: 'modified' });

    manager.recordVersion(p1, 'Original');
    manager.recordVersion(p2, 'Modified');

    const rolled = manager.rollback(1);
    expect(rolled).not.toBeNull();
    expect(rolled!.name).toBe('original');
  });

  it('rollback returns a deep clone that cannot mutate stored version', () => {
    const policy = getDefaultPolicy();
    manager.recordVersion(policy, 'Initial');

    const rolled = manager.rollback(1)!;
    rolled.name = 'MUTATED';
    rolled.velocity.linearMax = 999;

    const stored = manager.getVersion(1)!;
    expect(stored.policy.name).toBe('default');
    expect(stored.policy.velocity.linearMax).toBe(0.5);
  });

  it('rollback returns null for an invalid version', () => {
    manager.recordVersion(getDefaultPolicy(), 'Initial');
    expect(manager.rollback(42)).toBeNull();
    expect(manager.rollback(0)).toBeNull();
    expect(manager.rollback(-1)).toBeNull();
  });

  // ── getCurrentVersion ──────────────────────────────────────────────

  it('getCurrentVersion returns 0 when no versions exist', () => {
    expect(manager.getCurrentVersion()).toBe(0);
  });

  it('getCurrentVersion returns the latest version number', () => {
    manager.recordVersion(getDefaultPolicy(), 'First');
    expect(manager.getCurrentVersion()).toBe(1);

    manager.recordVersion(makePolicy({ name: 'v2' }), 'Second');
    expect(manager.getCurrentVersion()).toBe(2);
  });

  // ── maxVersions cap ────────────────────────────────────────────────

  it('respects maxVersions cap by dropping oldest versions', () => {
    const small = new PolicyVersionManager(3);

    small.recordVersion(makePolicy({ name: 'v1' }), 'First');
    small.recordVersion(makePolicy({ name: 'v2' }), 'Second');
    small.recordVersion(makePolicy({ name: 'v3' }), 'Third');
    small.recordVersion(makePolicy({ name: 'v4' }), 'Fourth');
    small.recordVersion(makePolicy({ name: 'v5' }), 'Fifth');

    const versions = small.getVersions();
    expect(versions).toHaveLength(3);
    // Only versions 3, 4, 5 should remain
    expect(versions.map((v) => v.version)).toEqual([5, 4, 3]);
    // Oldest versions should be gone
    expect(small.getVersion(1)).toBeNull();
    expect(small.getVersion(2)).toBeNull();
  });

  it('maxVersions of 1 keeps only the latest version', () => {
    const single = new PolicyVersionManager(1);

    single.recordVersion(makePolicy({ name: 'a' }), 'A');
    single.recordVersion(makePolicy({ name: 'b' }), 'B');

    expect(single.getVersions()).toHaveLength(1);
    expect(single.getVersion(1)).toBeNull();
    expect(single.getVersion(2)!.policy.name).toBe('b');
  });

  // ── Immutability ───────────────────────────────────────────────────

  it('stored policy is a deep clone, not affected by external mutation', () => {
    const policy = getDefaultPolicy();
    manager.recordVersion(policy, 'Snapshot');

    // Mutate the original policy object after recording
    policy.name = 'HACKED';
    policy.velocity.linearMax = 1000;

    const stored = manager.getVersion(1)!;
    expect(stored.policy.name).toBe('default');
    expect(stored.policy.velocity.linearMax).toBe(0.5);
  });

  // ── Timestamps ─────────────────────────────────────────────────────

  it('records a timestamp for each version', () => {
    const before = Date.now();
    manager.recordVersion(getDefaultPolicy(), 'Timestamped');
    const after = Date.now();

    const v = manager.getVersion(1)!;
    expect(v.timestamp).toBeGreaterThanOrEqual(before);
    expect(v.timestamp).toBeLessThanOrEqual(after);
  });

  // ── Array field changes ────────────────────────────────────────────

  it('detects changes to array fields like blockedTopics', () => {
    const p1 = getDefaultPolicy();
    const p2 = makePolicy({ blockedTopics: ['/rosout', '/parameter_events', '/new_topic'] });

    manager.recordVersion(p1, 'Initial');
    manager.recordVersion(p2, 'Add blocked topic');

    const v2 = manager.getVersion(2)!;
    expect(v2.changedFields).toContain('blockedTopics');
    expect(v2.changedFields).not.toContain('blockedServices');
  });
});

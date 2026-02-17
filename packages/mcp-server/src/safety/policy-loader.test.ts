import { describe, it, expect } from 'vitest';
import { join } from 'path';
import { loadPolicy, getDefaultPolicy } from './policy-loader.js';

// Resolve paths relative to the mcp-server package root
const policiesDir = join(import.meta.dirname, '..', '..', 'policies');

describe('loadPolicy', () => {
  it('returns default policy when no path provided', () => {
    const policy = loadPolicy();
    expect(policy.name).toBe('default');
    expect(policy.velocity.linearMax).toBe(0.5);
    expect(policy.velocity.angularMax).toBe(1.5);
    expect(policy.geofence.xMin).toBe(-5);
    expect(policy.geofence.xMax).toBe(5);
    expect(policy.rateLimits.publishHz).toBe(10);
    expect(policy.blockedTopics).toContain('/rosout');
    expect(policy.blockedServices).toContain('/kill');
  });

  it('loads default.yaml policy file', () => {
    const policy = loadPolicy(join(policiesDir, 'default.yaml'));
    expect(policy.name).toBe('default');
    expect(policy.velocity.linearMax).toBe(0.5);
  });

  it('loads turtlebot3.yaml policy file', () => {
    const policy = loadPolicy(join(policiesDir, 'turtlebot3.yaml'));
    expect(policy.name).toBe('turtlebot3');
    expect(policy.velocity.linearMax).toBe(0.22);
    expect(policy.velocity.angularMax).toBe(2.84);
    expect(policy.blockedServices).toContain('/delete_entity');
  });

  it('falls back to defaults for missing file', () => {
    const policy = loadPolicy('/nonexistent/path/nope.yaml');
    expect(policy.name).toBe('default');
    expect(policy.velocity.linearMax).toBe(0.5);
  });
});

describe('getDefaultPolicy', () => {
  it('returns a copy of the default policy', () => {
    const a = getDefaultPolicy();
    const b = getDefaultPolicy();
    expect(a).toEqual(b);
    expect(a).not.toBe(b); // different references
  });

  it('has all required fields', () => {
    const policy = getDefaultPolicy();
    expect(policy).toHaveProperty('name');
    expect(policy).toHaveProperty('description');
    expect(policy).toHaveProperty('velocity');
    expect(policy).toHaveProperty('geofence');
    expect(policy).toHaveProperty('rateLimits');
    expect(policy).toHaveProperty('blockedTopics');
    expect(policy).toHaveProperty('blockedServices');
  });
});

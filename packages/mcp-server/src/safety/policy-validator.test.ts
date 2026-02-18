import { describe, it, expect } from 'vitest';
import { validatePolicy } from './policy-validator.js';
import { getDefaultPolicy } from './policy-loader.js';

describe('PolicyValidator', () => {
  it('validates default policy without errors', () => {
    const result = validatePolicy(getDefaultPolicy());
    expect(result.valid).toBe(true);
    expect(result.errors).toHaveLength(0);
  });

  it('rejects empty policy name', () => {
    const policy = { ...getDefaultPolicy(), name: '' };
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
    expect(result.errors.some(e => e.includes('name'))).toBe(true);
  });

  it('rejects zero linearMax', () => {
    const policy = getDefaultPolicy();
    policy.velocity.linearMax = 0;
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
    expect(result.errors.some(e => e.includes('linearMax'))).toBe(true);
  });

  it('rejects negative angularMax', () => {
    const policy = getDefaultPolicy();
    policy.velocity.angularMax = -1;
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
    expect(result.errors.some(e => e.includes('angularMax'))).toBe(true);
  });

  it('warns about very high linearMax', () => {
    const policy = getDefaultPolicy();
    policy.velocity.linearMax = 15;
    const result = validatePolicy(policy);
    expect(result.warnings.some(w => w.includes('very high'))).toBe(true);
  });

  it('rejects invalid geofence (xMin >= xMax)', () => {
    const policy = getDefaultPolicy();
    policy.geofence.xMin = 5;
    policy.geofence.xMax = 3;
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
    expect(result.errors.some(e => e.includes('xMin'))).toBe(true);
  });

  it('rejects invalid geofence (yMin >= yMax)', () => {
    const policy = getDefaultPolicy();
    policy.geofence.yMin = 10;
    policy.geofence.yMax = 10;
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
  });

  it('rejects invalid geofence (zMin >= zMax)', () => {
    const policy = getDefaultPolicy();
    policy.geofence.zMin = 5;
    policy.geofence.zMax = 2;
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
  });

  it('warns about very large geofence', () => {
    const policy = getDefaultPolicy();
    policy.geofence.xMin = -1000;
    policy.geofence.xMax = 1000;
    const result = validatePolicy(policy);
    expect(result.warnings.some(w => w.includes('very large'))).toBe(true);
  });

  it('warns about very small geofence', () => {
    const policy = getDefaultPolicy();
    policy.geofence.xMin = -0.1;
    policy.geofence.xMax = 0.1;
    const result = validatePolicy(policy);
    expect(result.warnings.some(w => w.includes('very small'))).toBe(true);
  });

  it('rejects negative geofence warning margin', () => {
    const policy = getDefaultPolicy();
    policy.geofenceWarningMargin = -1;
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
    expect(result.errors.some(e => e.includes('geofenceWarningMargin'))).toBe(true);
  });

  it('warns about oversized warning margin', () => {
    const policy = getDefaultPolicy();
    // Default geofence is 10m wide, margin 6m would be > half
    policy.geofenceWarningMargin = 6;
    const result = validatePolicy(policy);
    expect(result.warnings.some(w => w.includes('more than half'))).toBe(true);
  });

  it('rejects zero publishHz', () => {
    const policy = getDefaultPolicy();
    policy.rateLimits.publishHz = 0;
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
  });

  it('warns about very high publishHz', () => {
    const policy = getDefaultPolicy();
    policy.rateLimits.publishHz = 5000;
    const result = validatePolicy(policy);
    expect(result.warnings.some(w => w.includes('very high'))).toBe(true);
  });

  it('warns about short deadman switch timeout', () => {
    const policy = getDefaultPolicy();
    policy.deadmanSwitch.enabled = true;
    policy.deadmanSwitch.timeoutMs = 500;
    const result = validatePolicy(policy);
    expect(result.warnings.some(w => w.includes('very short'))).toBe(true);
  });

  it('warns when no blocked topics', () => {
    const policy = getDefaultPolicy();
    policy.blockedTopics = [];
    const result = validatePolicy(policy);
    expect(result.warnings.some(w => w.includes('No blocked topics'))).toBe(true);
  });

  it('rejects empty allowedTopics list', () => {
    const policy = getDefaultPolicy();
    policy.allowedTopics = [];
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
    expect(result.errors.some(e => e.includes('allowedTopics'))).toBe(true);
  });

  it('validates acceleration limits when enabled', () => {
    const policy = getDefaultPolicy();
    policy.acceleration = { ...policy.acceleration, enabled: true, linearMaxAccel: 0 };
    const result = validatePolicy(policy);
    expect(result.valid).toBe(false);
    expect(result.errors.some(e => e.includes('linearMaxAccel'))).toBe(true);
  });

  it('skips acceleration validation when disabled', () => {
    const policy = getDefaultPolicy();
    policy.acceleration = { ...policy.acceleration, enabled: false, linearMaxAccel: 0 };
    const result = validatePolicy(policy);
    // Should be valid because acceleration is disabled
    expect(result.valid).toBe(true);
  });
});

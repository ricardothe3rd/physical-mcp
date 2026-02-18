import { describe, it, expect } from 'vitest';
import { runStartupChecks } from './startup-check.js';

describe('runStartupChecks', () => {
  it('passes with valid default config', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    expect(result.passed).toBe(true);
    expect(result.checks.every(c => c.status !== 'fail')).toBe(true);
  });

  it('fails with invalid bridge URL protocol', () => {
    const result = runStartupChecks({
      bridgeUrl: 'http://localhost:9090',
      verbose: false,
    });
    expect(result.passed).toBe(false);
    const urlCheck = result.checks.find(c => c.name === 'bridge-url');
    expect(urlCheck?.status).toBe('fail');
  });

  it('fails with invalid bridge URL', () => {
    const result = runStartupChecks({
      bridgeUrl: 'not a url',
      verbose: false,
    });
    expect(result.passed).toBe(false);
  });

  it('accepts wss:// protocol', () => {
    const result = runStartupChecks({
      bridgeUrl: 'wss://localhost:9090',
      verbose: false,
    });
    const urlCheck = result.checks.find(c => c.name === 'bridge-url');
    expect(urlCheck?.status).toBe('ok');
  });

  it('fails with nonexistent policy file', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      policyPath: '/nonexistent/policy.yaml',
      verbose: false,
    });
    expect(result.passed).toBe(false);
    const policyCheck = result.checks.find(c => c.name === 'policy-file');
    expect(policyCheck?.status).toBe('fail');
  });

  it('checks Node.js version', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const nodeCheck = result.checks.find(c => c.name === 'node-version');
    expect(nodeCheck?.status).toBe('ok');
  });

  it('reports default policy when no path specified', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const policyCheck = result.checks.find(c => c.name === 'policy-file');
    expect(policyCheck?.status).toBe('ok');
    expect(policyCheck?.message).toContain('default');
  });
});

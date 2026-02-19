import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { runStartupChecks } from './startup-check.js';

describe('runStartupChecks', () => {
  // Store original env vars so we can restore them after each test
  const originalEnv: Record<string, string | undefined> = {};

  beforeEach(() => {
    originalEnv.PHYSICAL_MCP_BRIDGE_URL = process.env.PHYSICAL_MCP_BRIDGE_URL;
    originalEnv.PHYSICAL_MCP_POLICY = process.env.PHYSICAL_MCP_POLICY;
    delete process.env.PHYSICAL_MCP_BRIDGE_URL;
    delete process.env.PHYSICAL_MCP_POLICY;
  });

  afterEach(() => {
    if (originalEnv.PHYSICAL_MCP_BRIDGE_URL !== undefined) {
      process.env.PHYSICAL_MCP_BRIDGE_URL = originalEnv.PHYSICAL_MCP_BRIDGE_URL;
    } else {
      delete process.env.PHYSICAL_MCP_BRIDGE_URL;
    }
    if (originalEnv.PHYSICAL_MCP_POLICY !== undefined) {
      process.env.PHYSICAL_MCP_POLICY = originalEnv.PHYSICAL_MCP_POLICY;
    } else {
      delete process.env.PHYSICAL_MCP_POLICY;
    }
  });

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

  // --- Bridge port validation ---

  it('validates bridge port is in range', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const portCheck = result.checks.find(c => c.name === 'bridge-port');
    expect(portCheck?.status).toBe('ok');
    expect(portCheck?.message).toContain('9090');
  });

  it('accepts default port when none specified', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost',
      verbose: false,
    });
    const portCheck = result.checks.find(c => c.name === 'bridge-port');
    expect(portCheck?.status).toBe('ok');
    expect(portCheck?.message).toContain('default');
  });

  it('fails bridge-port check when URL is completely invalid', () => {
    const result = runStartupChecks({
      bridgeUrl: 'not a url',
      verbose: false,
    });
    const portCheck = result.checks.find(c => c.name === 'bridge-port');
    expect(portCheck?.status).toBe('fail');
    expect(portCheck?.message).toContain('invalid');
  });

  // --- PHYSICAL_MCP_BRIDGE_URL env var ---

  it('validates PHYSICAL_MCP_BRIDGE_URL env var when set with valid ws://', () => {
    process.env.PHYSICAL_MCP_BRIDGE_URL = 'ws://robot:9090';
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const envCheck = result.checks.find(c => c.name === 'env-bridge-url');
    expect(envCheck?.status).toBe('ok');
  });

  it('validates PHYSICAL_MCP_BRIDGE_URL env var when set with valid wss://', () => {
    process.env.PHYSICAL_MCP_BRIDGE_URL = 'wss://robot.example.com:9090';
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const envCheck = result.checks.find(c => c.name === 'env-bridge-url');
    expect(envCheck?.status).toBe('ok');
  });

  it('fails when PHYSICAL_MCP_BRIDGE_URL uses http://', () => {
    process.env.PHYSICAL_MCP_BRIDGE_URL = 'http://robot:9090';
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    expect(result.passed).toBe(false);
    const envCheck = result.checks.find(c => c.name === 'env-bridge-url');
    expect(envCheck?.status).toBe('fail');
    expect(envCheck?.message).toContain('ws:// or wss://');
  });

  it('fails when PHYSICAL_MCP_BRIDGE_URL is not a valid URL', () => {
    process.env.PHYSICAL_MCP_BRIDGE_URL = 'garbage';
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    expect(result.passed).toBe(false);
    const envCheck = result.checks.find(c => c.name === 'env-bridge-url');
    expect(envCheck?.status).toBe('fail');
    expect(envCheck?.message).toContain('not a valid URL');
  });

  it('skips env-bridge-url check when PHYSICAL_MCP_BRIDGE_URL is not set', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const envCheck = result.checks.find(c => c.name === 'env-bridge-url');
    expect(envCheck).toBeUndefined();
  });

  // --- PHYSICAL_MCP_POLICY env var ---

  it('passes when PHYSICAL_MCP_POLICY points to existing file', () => {
    // Use this test file itself as the existing file
    process.env.PHYSICAL_MCP_POLICY = __filename;
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const envCheck = result.checks.find(c => c.name === 'env-policy-path');
    expect(envCheck?.status).toBe('ok');
  });

  it('fails when PHYSICAL_MCP_POLICY points to nonexistent file', () => {
    process.env.PHYSICAL_MCP_POLICY = '/nonexistent/path/policy.yaml';
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    expect(result.passed).toBe(false);
    const envCheck = result.checks.find(c => c.name === 'env-policy-path');
    expect(envCheck?.status).toBe('fail');
    expect(envCheck?.message).toContain('not found');
  });

  it('skips env-policy-path check when PHYSICAL_MCP_POLICY is not set', () => {
    const result = runStartupChecks({
      bridgeUrl: 'ws://localhost:9090',
      verbose: false,
    });
    const envCheck = result.checks.find(c => c.name === 'env-policy-path');
    expect(envCheck).toBeUndefined();
  });
});

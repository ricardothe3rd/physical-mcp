/**
 * Startup self-test: validates configuration before the server starts.
 */

import { existsSync } from 'fs';
import { loadPolicy } from '../safety/policy-loader.js';

export interface StartupCheckResult {
  passed: boolean;
  checks: { name: string; status: 'ok' | 'warn' | 'fail'; message: string }[];
}

export function runStartupChecks(config: {
  bridgeUrl: string;
  policyPath?: string;
  verbose: boolean;
}): StartupCheckResult {
  const checks: StartupCheckResult['checks'] = [];

  // 1. Validate bridge URL format
  try {
    const url = new URL(config.bridgeUrl);
    if (url.protocol !== 'ws:' && url.protocol !== 'wss:') {
      checks.push({
        name: 'bridge-url',
        status: 'fail',
        message: `Bridge URL must use ws:// or wss:// protocol, got: ${url.protocol}`,
      });
    } else {
      checks.push({ name: 'bridge-url', status: 'ok', message: `Bridge URL: ${config.bridgeUrl}` });
    }
  } catch {
    checks.push({
      name: 'bridge-url',
      status: 'fail',
      message: `Invalid bridge URL: ${config.bridgeUrl}`,
    });
  }

  // 2. Validate bridge port is in valid range (1-65535)
  try {
    const url = new URL(config.bridgeUrl);
    if (url.port) {
      const port = parseInt(url.port, 10);
      if (isNaN(port) || port < 1 || port > 65535) {
        checks.push({
          name: 'bridge-port',
          status: 'fail',
          message: `Bridge port must be between 1 and 65535, got: ${url.port}`,
        });
      } else {
        checks.push({ name: 'bridge-port', status: 'ok', message: `Bridge port: ${port}` });
      }
    } else {
      checks.push({ name: 'bridge-port', status: 'ok', message: 'Bridge port: default' });
    }
  } catch {
    // URL parse failure already caught by bridge-url check above
    checks.push({
      name: 'bridge-port',
      status: 'fail',
      message: 'Cannot validate port — bridge URL is invalid',
    });
  }

  // 3. Validate PHYSICAL_MCP_BRIDGE_URL env var format if set
  const envBridgeUrl = process.env.PHYSICAL_MCP_BRIDGE_URL;
  if (envBridgeUrl !== undefined) {
    try {
      const envUrl = new URL(envBridgeUrl);
      if (envUrl.protocol !== 'ws:' && envUrl.protocol !== 'wss:') {
        checks.push({
          name: 'env-bridge-url',
          status: 'fail',
          message: `PHYSICAL_MCP_BRIDGE_URL must use ws:// or wss://, got: ${envUrl.protocol}`,
        });
      } else {
        checks.push({ name: 'env-bridge-url', status: 'ok', message: `PHYSICAL_MCP_BRIDGE_URL: ${envBridgeUrl}` });
      }
    } catch {
      checks.push({
        name: 'env-bridge-url',
        status: 'fail',
        message: `PHYSICAL_MCP_BRIDGE_URL is not a valid URL: ${envBridgeUrl}`,
      });
    }
  }

  // 4. Validate PHYSICAL_MCP_POLICY env var file exists if set
  const envPolicyPath = process.env.PHYSICAL_MCP_POLICY;
  if (envPolicyPath !== undefined) {
    if (existsSync(envPolicyPath)) {
      checks.push({ name: 'env-policy-path', status: 'ok', message: `PHYSICAL_MCP_POLICY file exists: ${envPolicyPath}` });
    } else {
      checks.push({
        name: 'env-policy-path',
        status: 'fail',
        message: `PHYSICAL_MCP_POLICY file not found: ${envPolicyPath}`,
      });
    }
  }

  // 5. Validate policy file exists (if specified)
  if (config.policyPath) {
    if (existsSync(config.policyPath)) {
      try {
        const policy = loadPolicy(config.policyPath);
        checks.push({ name: 'policy-file', status: 'ok', message: `Policy loaded: ${policy.name}` });

        // 6. Validate policy values
        if (policy.velocity.linearMax <= 0) {
          checks.push({ name: 'policy-velocity', status: 'fail', message: 'linearMax must be positive' });
        } else if (policy.velocity.linearMax > 10) {
          checks.push({ name: 'policy-velocity', status: 'warn', message: `linearMax ${policy.velocity.linearMax} m/s is very high — are you sure?` });
        } else {
          checks.push({ name: 'policy-velocity', status: 'ok', message: `Velocity limits: ${policy.velocity.linearMax} m/s linear, ${policy.velocity.angularMax} rad/s angular` });
        }

        if (policy.geofence.xMin >= policy.geofence.xMax || policy.geofence.yMin >= policy.geofence.yMax) {
          checks.push({ name: 'policy-geofence', status: 'fail', message: 'Geofence min must be less than max' });
        } else {
          const width = policy.geofence.xMax - policy.geofence.xMin;
          const depth = policy.geofence.yMax - policy.geofence.yMin;
          checks.push({ name: 'policy-geofence', status: 'ok', message: `Geofence: ${width}m × ${depth}m` });
        }
      } catch (err) {
        checks.push({ name: 'policy-file', status: 'fail', message: `Failed to parse policy: ${err}` });
      }
    } else {
      checks.push({ name: 'policy-file', status: 'fail', message: `Policy file not found: ${config.policyPath}` });
    }
  } else {
    checks.push({ name: 'policy-file', status: 'ok', message: 'Using default policy' });
  }

  // 7. Check Node.js version
  const nodeVersion = parseInt(process.version.slice(1), 10);
  if (nodeVersion < 18) {
    checks.push({ name: 'node-version', status: 'fail', message: `Node.js >= 18 required, got: ${process.version}` });
  } else {
    checks.push({ name: 'node-version', status: 'ok', message: `Node.js ${process.version}` });
  }

  const passed = checks.every(c => c.status !== 'fail');

  return { passed, checks };
}

export function printStartupChecks(result: StartupCheckResult): void {
  for (const check of result.checks) {
    const icon = check.status === 'ok' ? '[OK]' : check.status === 'warn' ? '[WARN]' : '[FAIL]';
    console.error(`[StartupCheck] ${icon} ${check.name}: ${check.message}`);
  }
  if (!result.passed) {
    console.error('[StartupCheck] Startup checks FAILED. Fix the issues above and try again.');
  }
}

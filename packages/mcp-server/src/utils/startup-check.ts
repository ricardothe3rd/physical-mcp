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

  // 2. Validate policy file exists (if specified)
  if (config.policyPath) {
    if (existsSync(config.policyPath)) {
      try {
        const policy = loadPolicy(config.policyPath);
        checks.push({ name: 'policy-file', status: 'ok', message: `Policy loaded: ${policy.name}` });

        // 3. Validate policy values
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

  // 4. Check Node.js version
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

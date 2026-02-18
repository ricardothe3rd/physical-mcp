# Tutorial: How to Add a New Safety Check

This guide walks through the complete process of adding a new safety check to the PhysicalMCP policy engine. We use a concrete worked example throughout: adding **joint limit checking** that prevents commands from exceeding safe joint angle ranges on a robot arm.

---

## Table of Contents

1. [Safety Architecture Overview](#safety-architecture-overview)
2. [Step 1: Add the Violation Type to types.ts](#step-1-add-the-violation-type-to-typests)
3. [Step 2: Add Config to the SafetyPolicy Interface](#step-2-add-config-to-the-safetypolicy-interface)
4. [Step 3: Update policy-loader.ts Defaults](#step-3-update-policy-loaderts-defaults)
5. [Step 4: Implement the Check in policy-engine.ts](#step-4-implement-the-check-in-policy-enginets)
6. [Step 5: Wire It Into checkPublish / checkServiceCall](#step-5-wire-it-into-checkpublish--checkservicecall)
7. [Step 6: Add Runtime Update Method](#step-6-add-runtime-update-method)
8. [Step 7: Add a Safety Tool for Runtime Configuration](#step-7-add-a-safety-tool-for-runtime-configuration)
9. [Step 8: Add YAML Config Support](#step-8-add-yaml-config-support)
10. [Step 9: Write Tests](#step-9-write-tests)
11. [Complete Worked Example: Joint Limit Checking](#complete-worked-example-joint-limit-checking)
12. [Checklist](#checklist)

---

## Safety Architecture Overview

The safety system has four layers:

```
Tool Handler (e.g., handleTopicTool)
  |
  v
PolicyEngine (safety/policy-engine.ts)     <-- core safety evaluation
  |-- checkPublish()       checks: e-stop, blocked topics, velocity, acceleration, rate limits
  |-- checkServiceCall()   checks: e-stop, blocked services, rate limits
  |-- checkActionGoal()    checks: e-stop, rate limits
  |
  +-- Helper modules:
      |-- RateLimiter     (safety/rate-limiter.ts)
      |-- checkGeofence   (safety/geofence.ts)
      |-- AuditLogger     (safety/audit-logger.ts)
```

Key files:

| File | Purpose |
|------|---------|
| `src/safety/types.ts` | Type definitions: `SafetyViolationType`, `SafetyPolicy`, `SafetyViolation`, etc. |
| `src/safety/policy-engine.ts` | Core engine that evaluates every command against the active policy |
| `src/safety/policy-loader.ts` | Loads YAML policy files, merges with defaults |
| `src/safety/geofence.ts` | Geofence boundary check (example of a standalone check module) |
| `src/safety/rate-limiter.ts` | Rate limiting logic (example of a stateful check module) |
| `src/safety/audit-logger.ts` | Audit trail for all safety decisions |
| `src/tools/safety-tools.ts` | MCP tools for runtime safety configuration |

### How a safety check flows

1. A tool handler calls `safety.checkPublish(topic, message)` (or `checkServiceCall`, `checkActionGoal`).
2. The `PolicyEngine` runs all relevant checks in sequence, collecting violations.
3. If any violations are found, `allowed` is set to `false`.
4. The result (with violations) is logged to the audit trail.
5. The handler inspects `result.allowed` and either executes the command or returns a `SAFETY BLOCKED` error.

---

## Step 1: Add the Violation Type to types.ts

Every safety check produces a typed violation. Add your new violation type to the `SafetyViolationType` union in `src/safety/types.ts`:

```typescript
export type SafetyViolationType =
  | 'velocity_exceeded'
  | 'velocity_clamped'
  | 'acceleration_exceeded'
  | 'geofence_violation'
  | 'geofence_warning'
  | 'rate_limit_exceeded'
  | 'blocked_topic'
  | 'blocked_service'
  | 'emergency_stop_active'
  | 'deadman_switch_timeout'
  | 'joint_limit_exceeded';     // <-- NEW
```

### Naming conventions

- Use `snake_case` for violation type names.
- Use `_exceeded` for numeric limit violations (e.g., `velocity_exceeded`, `joint_limit_exceeded`).
- Use `_violation` for boundary/rule violations (e.g., `geofence_violation`).
- Use `_warning` for non-blocking warnings (e.g., `geofence_warning`, `velocity_clamped`).
- Use `_active` or `_timeout` for state-based checks (e.g., `emergency_stop_active`).

---

## Step 2: Add Config to the SafetyPolicy Interface

Define the configuration structure for your check and add it to the `SafetyPolicy` interface in `src/safety/types.ts`.

### 2a. Define the config interface

```typescript
export interface JointLimits {
  enabled: boolean;
  limits: Record<string, { min: number; max: number }>;  // joint_name -> {min, max} in radians
}
```

### 2b. Add it to SafetyPolicy

```typescript
export interface SafetyPolicy {
  name: string;
  description: string;
  velocity: VelocityLimits;
  acceleration: AccelerationLimits;
  geofence: GeofenceBounds;
  geofenceWarningMargin: number;
  rateLimits: RateLimitConfig;
  deadmanSwitch: DeadmanSwitchConfig;
  jointLimits: JointLimits;             // <-- NEW
  blockedTopics: string[];
  blockedServices: string[];
  allowedTopics?: string[];
  allowedServices?: string[];
}
```

### Design guidelines for config interfaces

- Always include an `enabled: boolean` field so the check can be toggled at runtime.
- Use descriptive field names with units in comments (e.g., `// radians`, `// m/s`).
- Keep the interface flat where possible. Use nested objects only when grouping related values (like per-joint limits).
- Make numeric limits configurable -- never hardcode threshold values in the engine.

---

## Step 3: Update policy-loader.ts Defaults

The policy loader must provide sensible defaults for every field in `SafetyPolicy`. Update two locations in `src/safety/policy-loader.ts`:

### 3a. Add defaults to `DEFAULT_POLICY`

```typescript
const DEFAULT_POLICY: SafetyPolicy = {
  name: 'default',
  description: 'Conservative default safety policy',
  velocity: { /* ... existing ... */ },
  acceleration: { /* ... existing ... */ },
  geofence: { /* ... existing ... */ },
  geofenceWarningMargin: 1.0,
  rateLimits: { /* ... existing ... */ },
  deadmanSwitch: { /* ... existing ... */ },
  // NEW: Joint limits disabled by default (not all robots have arms)
  jointLimits: {
    enabled: false,
    limits: {},
  },
  blockedTopics: ['/rosout', '/parameter_events'],
  blockedServices: ['/kill', '/shutdown'],
};
```

### 3b. Add merge logic to `mergePolicyWithDefaults`

```typescript
function mergePolicyWithDefaults(data: Record<string, unknown>): SafetyPolicy {
  return {
    // ... existing fields ...
    jointLimits: {
      ...DEFAULT_POLICY.jointLimits,
      ...(data.jointLimits as Record<string, unknown> || {}),
    },
    // ... remaining fields ...
  };
}
```

### Default value guidelines

- **Disabled by default** for checks that are robot-specific (like joint limits -- not every robot has joints).
- **Enabled by default** with conservative limits for universal checks (like velocity limits -- every mobile robot has a speed limit).
- Choose defaults that are safe for the most common use case.

---

## Step 4: Implement the Check in policy-engine.ts

Add a private method to `PolicyEngine` that performs the actual safety evaluation. The method should:

1. Return `null` if no violation is found.
2. Return a `SafetyViolation` object if the check fails.
3. Check the `enabled` flag first for early return.

```typescript
/**
 * Check whether joint positions in a trajectory command exceed configured limits.
 *
 * Inspects the `points` array in a JointTrajectory message for any position
 * values that fall outside the configured min/max range for each joint.
 *
 * @param message - The message payload (expected to contain joint trajectory data)
 * @returns A safety violation if any joint exceeds its limits, or null if all are within bounds
 */
private checkJointLimits(message: Record<string, unknown>): SafetyViolation | null {
  const config = this.policy.jointLimits;
  if (!config.enabled) return null;

  // Extract joint names and positions from the message
  // ROS2 JointTrajectory message format:
  //   { joint_names: string[], points: [{ positions: number[], ... }] }
  const jointNames = message.joint_names as string[] | undefined;
  const points = message.points as Array<{ positions: number[] }> | undefined;

  if (!jointNames || !points) return null;

  const violations: string[] = [];

  for (const point of points) {
    if (!point.positions) continue;

    for (let i = 0; i < jointNames.length; i++) {
      const jointName = jointNames[i];
      const position = point.positions[i];
      const limit = config.limits[jointName];

      if (limit && position !== undefined) {
        if (position < limit.min) {
          violations.push(
            `${jointName}: ${position.toFixed(3)} rad < min ${limit.min.toFixed(3)} rad`
          );
        }
        if (position > limit.max) {
          violations.push(
            `${jointName}: ${position.toFixed(3)} rad > max ${limit.max.toFixed(3)} rad`
          );
        }
      }
    }
  }

  if (violations.length > 0) {
    return {
      type: 'joint_limit_exceeded',
      message: `Joint limits exceeded: ${violations.join(', ')}`,
      details: { violations, limits: config.limits },
      timestamp: Date.now(),
    };
  }

  return null;
}
```

### Implementation patterns

Study the existing checks in `PolicyEngine` to follow established patterns:

| Check | Pattern | Returns |
|-------|---------|---------|
| `checkVelocityMessage()` | Computes magnitude, compares to limit | `SafetyViolation \| null` |
| `checkAcceleration()` | Stateful (tracks last velocity + time), computes rate of change | `SafetyViolation \| null` |
| `checkGeofenceProximity()` | Computes distance to boundary | `SafetyViolation \| null` |
| `isTopicBlocked()` | String matching against blocked list | `boolean` |

Your check should:
- Accept the minimum data it needs (usually the message payload).
- Return `null` early if the check is disabled or data is missing.
- Collect all violations before returning (do not short-circuit on the first violation).
- Include relevant numeric values in both `message` (human-readable) and `details` (machine-parseable).

---

## Step 5: Wire It Into checkPublish / checkServiceCall

Now integrate your check into the appropriate entry point(s) in the `PolicyEngine`. The three entry points are:

- `checkPublish(topic, message)` -- for topic publish commands
- `checkServiceCall(service, args)` -- for service calls
- `checkActionGoal(action, goal)` -- for action goals

For joint limits, we want to check topic publishes to joint trajectory topics.

### Adding to checkPublish

```typescript
checkPublish(topic: string, message: Record<string, unknown>): SafetyCheckResult {
  const violations: SafetyViolation[] = [];

  // ... existing e-stop check ...
  // ... existing blocked topic check ...

  // Existing: Check velocity limits for cmd_vel topics
  if (topic.includes('cmd_vel')) {
    // ... existing velocity and acceleration checks ...
  }

  // NEW: Check joint limits for joint trajectory topics
  if (topic.includes('joint_trajectory') || topic.includes('joint_command')) {
    const jointViolation = this.checkJointLimits(message);
    if (jointViolation) {
      violations.push(jointViolation);
    }
  }

  // ... existing rate limit check ...

  const result: SafetyCheckResult = {
    allowed: violations.length === 0,
    violations: [...violations, ...clampWarnings],
  };

  this.auditLogger.log('publish', topic, message, result);
  return result;
}
```

### Choosing the right entry point

| If your check applies to... | Wire it into... |
|------------------------------|-----------------|
| Topic publishes (e.g., velocity commands, joint trajectories) | `checkPublish()` |
| Service calls (e.g., parameter changes, spawning entities) | `checkServiceCall()` |
| Action goals (e.g., navigation goals) | `checkActionGoal()` |
| All command types | All three methods |

### Topic matching

Use `topic.includes('...')` for broad matching (like `cmd_vel` appearing in `/cmd_vel`, `/robot1/cmd_vel`, etc.). Use exact string comparison or `topic === '...'` for specific topics.

---

## Step 6: Add Runtime Update Method

Add a public method to `PolicyEngine` so the safety configuration can be updated at runtime via MCP tools:

```typescript
updateJointLimits(config: Partial<JointLimits>): void {
  if (config.enabled !== undefined) {
    this.policy.jointLimits.enabled = config.enabled;
  }
  if (config.limits) {
    Object.assign(this.policy.jointLimits.limits, config.limits);
  }
  console.error(
    `[PolicyEngine] Joint limits updated: enabled=${this.policy.jointLimits.enabled}, joints=${Object.keys(this.policy.jointLimits.limits).length}`
  );
}
```

Also update `getStatus()` to include the new check's state:

```typescript
getStatus() {
  return {
    // ... existing fields ...
    jointLimits: this.policy.jointLimits,  // <-- NEW
    auditStats: this.auditLogger.getStats(),
  };
}
```

### Update method guidelines

- Accept `Partial<YourConfig>` so callers can update individual fields.
- Use `Object.assign` for shallow merges (consistent with existing methods like `updateVelocityLimits`).
- Log the update to stderr with the `[PolicyEngine]` prefix.
- Expose the state in `getStatus()` so the AI agent can inspect it.

---

## Step 7: Add a Safety Tool for Runtime Configuration

Add an MCP tool in `src/tools/safety-tools.ts` that allows the AI agent to configure your check at runtime.

### 7a. Add the tool definition

```typescript
{
  name: 'safety_update_joint_limits',
  description: 'Configure joint angle limits. When enabled, blocks trajectory commands that would move joints beyond safe ranges.',
  inputSchema: toInputSchema(z.object({
    enabled: z.boolean().optional().describe('Enable/disable joint limit checking'),
    limits: z.record(z.object({
      min: z.number().describe('Minimum joint angle in radians'),
      max: z.number().describe('Maximum joint angle in radians'),
    })).optional().describe('Joint limits as { joint_name: { min, max } }'),
  })),
},
```

### 7b. Add the handler case

```typescript
case 'safety_update_joint_limits': {
  const updates: Record<string, unknown> = {};
  if (args.enabled !== undefined) updates.enabled = args.enabled;
  if (args.limits !== undefined) updates.limits = args.limits;
  safety.updateJointLimits(updates);
  const policy = safety.getPolicy();
  return {
    content: [{
      type: 'text',
      text: `Joint limits updated:\n${JSON.stringify(policy.jointLimits, null, 2)}`,
    }],
  };
}
```

---

## Step 8: Add YAML Config Support

Users can configure safety policies via YAML files. Add a section for your check in the YAML format.

### Example YAML configuration

Create or update a policy file (e.g., `policies/robot-arm.yaml`):

```yaml
name: robot-arm
description: Safety policy for a 6-DOF robot arm

velocity:
  linearMax: 0.5
  angularMax: 1.5
  clampMode: false

jointLimits:
  enabled: true
  limits:
    joint_1:
      min: -3.14
      max: 3.14
    joint_2:
      min: -1.57
      max: 1.57
    joint_3:
      min: -1.57
      max: 1.57
    joint_4:
      min: -3.14
      max: 3.14
    joint_5:
      min: -1.57
      max: 1.57
    joint_6:
      min: -3.14
      max: 3.14

blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
```

The `mergePolicyWithDefaults` function in `policy-loader.ts` (updated in Step 3b) handles merging this with the defaults automatically.

### Nested object merging

For deeply nested configs (like per-joint limits), the spread operator in `mergePolicyWithDefaults` performs a shallow merge. If you need deep merging for nested objects, handle it explicitly:

```typescript
jointLimits: {
  enabled: (data.jointLimits as Record<string, unknown>)?.enabled
    ?? DEFAULT_POLICY.jointLimits.enabled,
  limits: {
    ...DEFAULT_POLICY.jointLimits.limits,
    ...((data.jointLimits as Record<string, unknown>)?.limits as Record<string, unknown> || {}),
  },
},
```

---

## Step 9: Write Tests

Safety checks are critical code and require thorough testing. Place tests in `src/safety/` alongside the source files.

### 9a. Unit test the check method

Test the check in isolation through the `PolicyEngine`:

```typescript
// src/safety/joint-limits.test.ts
import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from './policy-engine.js';

describe('Joint Limit Checking', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
    // Enable joint limits with test configuration
    engine.updateJointLimits({
      enabled: true,
      limits: {
        joint_1: { min: -1.57, max: 1.57 },
        joint_2: { min: -0.78, max: 0.78 },
      },
    });
  });

  describe('when enabled', () => {
    it('allows joint positions within limits', () => {
      const result = engine.checkPublish('/joint_trajectory_controller/joint_trajectory', {
        joint_names: ['joint_1', 'joint_2'],
        points: [{ positions: [0.5, 0.3] }],
      });
      expect(result.allowed).toBe(true);
      expect(result.violations).toHaveLength(0);
    });

    it('blocks joint positions exceeding maximum', () => {
      const result = engine.checkPublish('/joint_trajectory_controller/joint_trajectory', {
        joint_names: ['joint_1', 'joint_2'],
        points: [{ positions: [2.0, 0.3] }],
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('joint_limit_exceeded');
      expect(result.violations[0].message).toContain('joint_1');
    });

    it('blocks joint positions below minimum', () => {
      const result = engine.checkPublish('/joint_trajectory_controller/joint_trajectory', {
        joint_names: ['joint_1', 'joint_2'],
        points: [{ positions: [-2.0, 0.3] }],
      });
      expect(result.allowed).toBe(false);
      expect(result.violations[0].type).toBe('joint_limit_exceeded');
    });

    it('checks all points in a trajectory', () => {
      const result = engine.checkPublish('/joint_trajectory_controller/joint_trajectory', {
        joint_names: ['joint_1'],
        points: [
          { positions: [0.5] },   // OK
          { positions: [2.0] },   // exceeds limit
        ],
      });
      expect(result.allowed).toBe(false);
    });

    it('ignores joints not in the limits config', () => {
      const result = engine.checkPublish('/joint_trajectory_controller/joint_trajectory', {
        joint_names: ['joint_1', 'joint_99'],
        points: [{ positions: [0.5, 999.0] }],
      });
      // joint_99 is not configured, so it should be allowed
      expect(result.allowed).toBe(true);
    });

    it('does not check non-trajectory topics', () => {
      const result = engine.checkPublish('/cmd_vel', {
        joint_names: ['joint_1'],
        points: [{ positions: [999.0] }],
      });
      // /cmd_vel is not a joint trajectory topic, so joint limits are not checked
      // (may fail on velocity check, but not joint limits)
      const jointViolations = result.violations.filter(v => v.type === 'joint_limit_exceeded');
      expect(jointViolations).toHaveLength(0);
    });
  });

  describe('when disabled', () => {
    beforeEach(() => {
      engine.updateJointLimits({ enabled: false });
    });

    it('allows any joint positions', () => {
      const result = engine.checkPublish('/joint_trajectory_controller/joint_trajectory', {
        joint_names: ['joint_1'],
        points: [{ positions: [999.0] }],
      });
      const jointViolations = result.violations.filter(v => v.type === 'joint_limit_exceeded');
      expect(jointViolations).toHaveLength(0);
    });
  });

  describe('runtime updates', () => {
    it('reflects updated limits immediately', () => {
      // First check passes with current limits
      const msg1 = {
        joint_names: ['joint_1'],
        points: [{ positions: [1.0] }],
      };
      expect(engine.checkPublish('/joint_trajectory', msg1).allowed).toBe(true);

      // Tighten the limits
      engine.updateJointLimits({
        limits: { joint_1: { min: -0.5, max: 0.5 } },
      });

      // Same position now exceeds the new limit
      const msg2 = {
        joint_names: ['joint_1'],
        points: [{ positions: [1.0] }],
      };
      expect(engine.checkPublish('/joint_trajectory', msg2).allowed).toBe(false);
    });
  });
});
```

### 9b. Test YAML loading

Add a test case to the policy loader tests:

```typescript
// In src/safety/policy-loader.test.ts
it('loads joint limits from YAML', () => {
  const policy = loadPolicy(join(policiesDir, 'robot-arm.yaml'));
  expect(policy.jointLimits.enabled).toBe(true);
  expect(policy.jointLimits.limits.joint_1).toBeDefined();
  expect(policy.jointLimits.limits.joint_1.min).toBe(-3.14);
  expect(policy.jointLimits.limits.joint_1.max).toBe(3.14);
});

it('defaults joint limits to disabled when not specified', () => {
  const policy = loadPolicy(join(policiesDir, 'default.yaml'));
  expect(policy.jointLimits.enabled).toBe(false);
  expect(policy.jointLimits.limits).toEqual({});
});
```

### 9c. Test the safety tool

```typescript
// In a safety-tools test or inline
it('updates joint limits via safety tool', async () => {
  const result = await handleSafetyTool(
    'safety_update_joint_limits',
    { enabled: true, limits: { joint_1: { min: -1.0, max: 1.0 } } },
    mockConnection,
    safety
  );
  expect(result.isError).toBeUndefined();
  expect(result.content[0].text).toContain('joint_1');

  const policy = safety.getPolicy();
  expect(policy.jointLimits.enabled).toBe(true);
});
```

### Running tests

```bash
# From packages/mcp-server/
npm test                                         # run all tests
npx vitest run src/safety/joint-limits.test.ts   # run just your new test
npx vitest run src/safety/                       # run all safety tests
```

---

## Complete Worked Example: Joint Limit Checking

Here is a summary of every file change needed, in order:

### File 1: `src/safety/types.ts`

Add the violation type and config interface:

```typescript
// Add to SafetyViolationType union:
export type SafetyViolationType =
  | 'velocity_exceeded'
  | 'velocity_clamped'
  | 'acceleration_exceeded'
  | 'geofence_violation'
  | 'geofence_warning'
  | 'rate_limit_exceeded'
  | 'blocked_topic'
  | 'blocked_service'
  | 'emergency_stop_active'
  | 'deadman_switch_timeout'
  | 'joint_limit_exceeded';       // NEW

// Add new interface:
export interface JointLimits {
  enabled: boolean;
  limits: Record<string, { min: number; max: number }>;  // joint_name -> range in radians
}

// Add to SafetyPolicy interface:
export interface SafetyPolicy {
  // ... existing fields ...
  jointLimits: JointLimits;       // NEW
  blockedTopics: string[];
  // ... remaining fields ...
}
```

### File 2: `src/safety/policy-loader.ts`

Add defaults and merge logic:

```typescript
const DEFAULT_POLICY: SafetyPolicy = {
  // ... existing ...
  jointLimits: {
    enabled: false,
    limits: {},
  },
  blockedTopics: ['/rosout', '/parameter_events'],
  blockedServices: ['/kill', '/shutdown'],
};

function mergePolicyWithDefaults(data: Record<string, unknown>): SafetyPolicy {
  return {
    // ... existing fields ...
    jointLimits: {
      ...DEFAULT_POLICY.jointLimits,
      ...(data.jointLimits as Record<string, unknown> || {}),
    },
    // ... remaining fields ...
  };
}
```

### File 3: `src/safety/policy-engine.ts`

Add the check method, wire it in, and add the update method:

```typescript
// Import the new type
import type { SafetyPolicy, SafetyCheckResult, SafetyViolation, VelocityLimits,
  AccelerationLimits, GeofenceBounds, DeadmanSwitchConfig, JointLimits } from './types.js';

// Add private check method:
private checkJointLimits(message: Record<string, unknown>): SafetyViolation | null {
  const config = this.policy.jointLimits;
  if (!config.enabled) return null;

  const jointNames = message.joint_names as string[] | undefined;
  const points = message.points as Array<{ positions: number[] }> | undefined;
  if (!jointNames || !points) return null;

  const violations: string[] = [];

  for (const point of points) {
    if (!point.positions) continue;
    for (let i = 0; i < jointNames.length; i++) {
      const jointName = jointNames[i];
      const position = point.positions[i];
      const limit = config.limits[jointName];
      if (limit && position !== undefined) {
        if (position < limit.min) {
          violations.push(`${jointName}: ${position.toFixed(3)} rad < min ${limit.min.toFixed(3)} rad`);
        }
        if (position > limit.max) {
          violations.push(`${jointName}: ${position.toFixed(3)} rad > max ${limit.max.toFixed(3)} rad`);
        }
      }
    }
  }

  if (violations.length > 0) {
    return {
      type: 'joint_limit_exceeded',
      message: `Joint limits exceeded: ${violations.join(', ')}`,
      details: { violations, limits: config.limits },
      timestamp: Date.now(),
    };
  }

  return null;
}

// Wire into checkPublish() — add after the cmd_vel block:
if (topic.includes('joint_trajectory') || topic.includes('joint_command')) {
  const jointViolation = this.checkJointLimits(message);
  if (jointViolation) {
    violations.push(jointViolation);
  }
}

// Add public update method:
updateJointLimits(config: Partial<JointLimits>): void {
  if (config.enabled !== undefined) {
    this.policy.jointLimits.enabled = config.enabled;
  }
  if (config.limits) {
    Object.assign(this.policy.jointLimits.limits, config.limits);
  }
  console.error(
    `[PolicyEngine] Joint limits updated: enabled=${this.policy.jointLimits.enabled}, joints=${Object.keys(this.policy.jointLimits.limits).length}`
  );
}

// Update getStatus() to include joint limits:
getStatus() {
  return {
    // ... existing fields ...
    jointLimits: this.policy.jointLimits,
    // ... remaining fields ...
  };
}
```

### File 4: `src/tools/safety-tools.ts`

Add the tool definition and handler:

```typescript
// In getSafetyTools(), add:
{
  name: 'safety_update_joint_limits',
  description: 'Configure joint angle limits. When enabled, blocks trajectory commands that would move joints beyond safe ranges.',
  inputSchema: toInputSchema(z.object({
    enabled: z.boolean().optional().describe('Enable/disable joint limit checking'),
    limits: z.record(z.object({
      min: z.number().describe('Minimum joint angle in radians'),
      max: z.number().describe('Maximum joint angle in radians'),
    })).optional().describe('Joint limits as { joint_name: { min, max } }'),
  })),
},

// In handleSafetyTool(), add case:
case 'safety_update_joint_limits': {
  const updates: Record<string, unknown> = {};
  if (args.enabled !== undefined) updates.enabled = args.enabled;
  if (args.limits !== undefined) updates.limits = args.limits;
  safety.updateJointLimits(updates);
  const policy = safety.getPolicy();
  return {
    content: [{
      type: 'text',
      text: `Joint limits updated:\n${JSON.stringify(policy.jointLimits, null, 2)}`,
    }],
  };
}
```

### File 5: `policies/robot-arm.yaml` (new policy file)

```yaml
name: robot-arm
description: Safety policy for a 6-DOF robot arm

velocity:
  linearMax: 0.5
  angularMax: 1.5
  clampMode: false

jointLimits:
  enabled: true
  limits:
    joint_1:
      min: -3.14
      max: 3.14
    joint_2:
      min: -1.57
      max: 1.57
    joint_3:
      min: -1.57
      max: 1.57
    joint_4:
      min: -3.14
      max: 3.14
    joint_5:
      min: -1.57
      max: 1.57
    joint_6:
      min: -3.14
      max: 3.14

blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
```

### File 6: `src/safety/joint-limits.test.ts` (new test file)

(See the full test file in [Step 9a](#9a-unit-test-the-check-method) above.)

---

## Checklist

Use this checklist when adding a new safety check:

- [ ] **Added violation type** to `SafetyViolationType` union in `types.ts`
- [ ] **Defined config interface** (e.g., `JointLimits`) in `types.ts`
- [ ] **Added config to `SafetyPolicy`** interface in `types.ts`
- [ ] **Added defaults** to `DEFAULT_POLICY` in `policy-loader.ts`
- [ ] **Added merge logic** to `mergePolicyWithDefaults` in `policy-loader.ts`
- [ ] **Implemented the check** as a private method in `policy-engine.ts`
- [ ] **Wired into the right entry point** (`checkPublish`, `checkServiceCall`, or `checkActionGoal`)
- [ ] **Added runtime update method** (e.g., `updateJointLimits`) to `PolicyEngine`
- [ ] **Updated `getStatus()`** to include new check state
- [ ] **Added MCP safety tool** in `safety-tools.ts` for runtime configuration
- [ ] **Added YAML config support** (with example policy file)
- [ ] **Wrote tests** covering:
  - [ ] Check allows valid values
  - [ ] Check blocks values exceeding limits
  - [ ] Check respects the `enabled` flag
  - [ ] Check ignores irrelevant topics/messages
  - [ ] Runtime updates take effect immediately
  - [ ] YAML loading works correctly
  - [ ] Default values are applied when not specified in YAML
  - [ ] Safety tool updates config correctly
- [ ] **Ran `npm test`** and all tests pass
- [ ] **Ran `npm run build`** and TypeScript compiles without errors

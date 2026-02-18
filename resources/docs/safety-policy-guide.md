# Safety Policy Authoring Guide

This guide explains how to create, customize, and deploy safety policies for PhysicalMCP.

---

## Table of Contents

- [Overview](#overview)
- [Policy File Format](#policy-file-format)
- [Velocity Limits](#velocity-limits)
- [Geofence Boundaries](#geofence-boundaries)
- [Rate Limits](#rate-limits)
- [Deadman Switch](#deadman-switch)
- [Blocked Topics and Services](#blocked-topics-and-services)
- [Allowlists](#allowlists)
- [Built-in Policies](#built-in-policies)
- [Creating a Custom Policy](#creating-a-custom-policy)
- [Testing Your Policy](#testing-your-policy)
- [Best Practices](#best-practices)
- [Common Robot Profiles](#common-robot-profiles)

---

## Overview

A safety policy is a YAML file that defines the safety constraints enforced by PhysicalMCP before any command reaches the robot. Every publish, service call, and action goal is checked against the active policy.

**Key principles:**

1. **Default deny for dangerous operations** — Anything not explicitly allowed can be restricted
2. **Partial policies** — Omit any field and it falls back to safe defaults
3. **Runtime updates** — Limits can be adjusted at runtime via MCP tools without restarting
4. **Audit everything** — All policy decisions are logged regardless of outcome

---

## Policy File Format

```yaml
name: my-robot
description: Short description of this policy

velocity:
  linearMax: 0.5     # m/s — maximum linear speed
  angularMax: 1.5    # rad/s — maximum angular speed
  clampMode: false   # true = reduce to max, false = block entirely

geofence:
  xMin: -5.0
  xMax: 5.0
  yMin: -5.0
  yMax: 5.0
  zMin: 0.0
  zMax: 2.0

rateLimits:
  publishHz: 10           # max topic publishes per second
  servicePerMinute: 60    # max service calls per minute
  actionPerMinute: 30     # max action goals per minute

deadmanSwitch:
  enabled: false     # auto e-stop if no heartbeat received
  timeoutMs: 30000   # timeout in milliseconds

blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
```

### Loading a Policy

```bash
# Via environment variable
export PHYSICAL_MCP_POLICY=/path/to/my-policy.yaml
npx @ricardothe3rd/physical-mcp

# Via CLI flag
npx @ricardothe3rd/physical-mcp --policy /path/to/my-policy.yaml
```

---

## Velocity Limits

Controls the maximum speed for `geometry_msgs/msg/Twist` messages on any topic containing `cmd_vel`.

### How it works

1. The policy engine extracts `linear.x`, `linear.y`, `linear.z` from the Twist message
2. Computes the magnitude: `sqrt(x² + y² + z²)`
3. Compares against `linearMax`
4. Same process for `angular.x/y/z` against `angularMax`

**Important:** The check uses vector magnitude, not individual axis values. A robot moving at 0.4 m/s on X and 0.4 m/s on Y has an actual speed of ~0.57 m/s, which exceeds a 0.5 m/s limit.

### Block mode vs Clamp mode

```yaml
velocity:
  linearMax: 0.5
  angularMax: 1.5
  clampMode: false   # BLOCK mode (default)
```

**Block mode (`clampMode: false`):** Any velocity exceeding the limit is rejected. The command never reaches the robot. The AI gets an error message.

```yaml
velocity:
  linearMax: 0.22
  angularMax: 2.84
  clampMode: true    # CLAMP mode
```

**Clamp mode (`clampMode: true`):** Over-limit velocities are scaled down to the maximum while preserving direction. The command is sent to the robot at the reduced speed. A warning is included in the response.

**When to use each:**

| Scenario | Recommended Mode |
|----------|-----------------|
| Real hardware, safety-critical | Block (default) |
| Simulation, iterative tuning | Clamp |
| Hardware with known max speeds | Clamp (set to hardware max) |
| Unknown robot capabilities | Block with conservative limits |

### How to find your robot's limits

| Robot | Linear Max | Angular Max |
|-------|-----------|-------------|
| TurtleBot3 Burger | 0.22 m/s | 2.84 rad/s |
| TurtleBot3 Waffle | 0.26 m/s | 1.82 rad/s |
| TurtleBot4 | 0.46 m/s | — |
| Clearpath Jackal | 2.0 m/s | 4.0 rad/s |
| Clearpath Husky | 1.0 m/s | 2.0 rad/s |

Check your robot's URDF or manufacturer specs for exact values.

---

## Geofence Boundaries

Defines a rectangular workspace boundary in 3D space.

```yaml
geofence:
  xMin: -5.0   # meters
  xMax: 5.0
  yMin: -5.0
  yMax: 5.0
  zMin: 0.0    # floor
  zMax: 2.0    # ceiling (for drones)
```

The geofence is checked against position data when available. For ground robots, `zMin: 0` and `zMax: 2` is a safe default.

### Sizing your geofence

- **Simulation:** Match the world dimensions (e.g., 20m × 20m = xMin/yMin: -10, xMax/yMax: 10)
- **Lab/workshop:** Measure the room and subtract 0.5m from walls for safety margin
- **Outdoor:** Start small and expand as you gain confidence

---

## Rate Limits

Prevents flooding the ROS2 graph.

```yaml
rateLimits:
  publishHz: 10          # max publishes per second (any topic)
  servicePerMinute: 60   # max service calls per minute
  actionPerMinute: 30    # max action goals per minute
```

### How it works

Uses a sliding window rate limiter. Each topic/service/action is tracked independently. Example: with `publishHz: 10`, you can publish to `/cmd_vel` at 10 Hz AND `/other_topic` at 10 Hz simultaneously — the limit is per-topic.

### Tuning tips

- **Control loops** need higher publish rates (30-50 Hz for smooth motion)
- **AI agent conversations** rarely need more than 5-10 Hz
- **Service calls** are usually infrequent — 60/min is generous
- **Action goals** are expensive — 30/min prevents goal flooding

---

## Deadman Switch

Auto-activates emergency stop if no heartbeat is received within the timeout.

```yaml
deadmanSwitch:
  enabled: false     # disabled by default
  timeoutMs: 30000   # 30 seconds
```

### When to use

Enable the deadman switch when:
- The AI agent might disconnect unexpectedly
- You're running autonomous operations on real hardware
- You want a fail-safe for network interruptions

### How it works

1. When enabled, a timer checks for heartbeats at regular intervals
2. The AI agent (or MCP client) calls `safety_heartbeat` periodically
3. If no heartbeat arrives within `timeoutMs`, the policy engine activates emergency stop
4. E-stop can be released normally after sending a heartbeat

### Usage in MCP

```
AI: "Enable deadman switch with 10 second timeout"
→ Calls safety_deadman_switch(enabled: true, timeoutMs: 10000)

AI: [periodically] "Heartbeat"
→ Calls safety_heartbeat()
```

---

## Blocked Topics and Services

Prevent access to sensitive system resources.

```yaml
blockedTopics:
  - /rosout
  - /parameter_events
  - /diagnostics

blockedServices:
  - /kill
  - /shutdown
  - /delete_entity
  - /set_parameters    # prevents reconfiguring nodes
```

### Default blocked topics

- `/rosout` — ROS2 logging (read via different mechanisms)
- `/parameter_events` — Parameter change events (can disrupt node behavior)

### Default blocked services

- `/kill` — Terminates nodes
- `/shutdown` — Shuts down nodes

### What to block

Think about what an AI agent could do that would be harmful:

| Risk | Topics/Services to Block |
|------|------------------------|
| Kill ROS2 nodes | `/kill`, `/shutdown` |
| Reconfigure safety-critical params | `/set_parameters` on safety nodes |
| Access raw logs | `/rosout` |
| Interfere with sensors | Sensor driver topics |
| Control multiple robots when only authorized for one | Other robot namespaces |

---

## Allowlists

For maximum restriction, use allowlists instead of blocklists:

```yaml
# Instead of blocking specific topics, only allow these:
allowedTopics:
  - /cmd_vel
  - /odom
  - /scan
  - /camera/image_raw

allowedServices:
  - /navigate_to_pose
  - /set_initial_pose
```

When `allowedTopics` or `allowedServices` is set, **only** listed items are accessible. Everything else is blocked. This is the most restrictive mode and is recommended for production deployments.

---

## Built-in Policies

### `default.yaml` — Conservative General Purpose

```yaml
velocity: { linearMax: 0.5, angularMax: 1.5, clampMode: false }
geofence: 10m × 10m × 2m
rateLimits: 10 Hz publish, 60 svc/min, 30 action/min
deadmanSwitch: disabled
```

Good for: Unknown robots, first-time setup, general testing.

### `turtlebot3.yaml` — TurtleBot3 Burger

```yaml
velocity: { linearMax: 0.22, angularMax: 2.84, clampMode: true }
geofence: 20m × 20m × 0.5m
rateLimits: 30 Hz publish, 120 svc/min, 60 action/min
deadmanSwitch: disabled
```

Good for: TurtleBot3 Burger in Gazebo or real hardware. Clamp mode enabled since these are hardware maximums.

---

## Creating a Custom Policy

### Step 1: Start from a template

```bash
cp packages/mcp-server/policies/default.yaml my-robot.yaml
```

### Step 2: Set your robot's limits

Look up your robot's maximum speeds and set `linearMax` and `angularMax` to those values (or lower for extra safety).

### Step 3: Define your workspace

Measure your workspace and set the geofence. Add a safety margin:

```yaml
# Real room: 6m × 4m
# With 0.5m margin:
geofence:
  xMin: -2.5
  xMax: 2.5
  yMin: -1.5
  yMax: 1.5
  zMin: 0.0
  zMax: 1.0
```

### Step 4: Block dangerous endpoints

List any topics and services the AI should never access:

```yaml
blockedTopics:
  - /rosout
  - /parameter_events
  - /diagnostics/self_test

blockedServices:
  - /kill
  - /shutdown
  - /my_robot/reset_hardware
```

### Step 5: Load and test

```bash
npx @ricardothe3rd/physical-mcp --policy ./my-robot.yaml
```

---

## Testing Your Policy

### Manual testing checklist

1. **Velocity block/clamp:** Try publishing a velocity above the limit → should be blocked or clamped
2. **Velocity allow:** Try publishing a velocity within limits → should succeed
3. **Geofence:** Check that position limits are correctly set
4. **Blocked topics:** Try publishing to a blocked topic → should be blocked
5. **Blocked services:** Try calling a blocked service → should be blocked
6. **Rate limiting:** Send rapid commands → should be rate limited after threshold
7. **E-stop:** Activate e-stop → all commands should be blocked
8. **E-stop release:** Release e-stop → commands should resume
9. **Audit log:** Check that all of the above appear in the audit trail

### Automated testing

The policy engine has comprehensive unit tests. To run them:

```bash
cd packages/mcp-server
npm test
```

---

## Best Practices

1. **Start conservative, loosen gradually.** Begin with low velocity limits and a small geofence. Expand as you gain confidence.

2. **Use hardware limits as absolute maximums.** Never set `linearMax` or `angularMax` above the robot's physical maximum — the motors can't exceed it anyway, and overshooting can cause current spikes.

3. **Enable clamp mode for known robots.** If you know the hardware limits, clamp mode provides a smoother experience than blocking. The AI gets told "I reduced your speed" instead of "I blocked your command."

4. **Block system topics.** `/rosout` and `/parameter_events` should always be blocked unless you have a specific reason to access them.

5. **Use allowlists in production.** Blocklists are good for development; allowlists are better for production. Only expose what the AI actually needs.

6. **Enable the deadman switch for autonomous operations.** If the AI is running unattended, the deadman switch prevents the robot from continuing after a disconnect.

7. **Keep policies in version control.** Safety policies are configuration-as-code. Track them in git, review changes, and deploy them through your CI/CD pipeline.

8. **One policy per robot model.** Don't reuse a TurtleBot3 policy on a Husky — the velocity limits are completely different.

---

## Common Robot Profiles

### Mobile ground robot (generic)

```yaml
name: generic-ground-robot
velocity: { linearMax: 0.5, angularMax: 1.5, clampMode: false }
geofence: { xMin: -5, xMax: 5, yMin: -5, yMax: 5, zMin: 0, zMax: 0.5 }
deadmanSwitch: { enabled: true, timeoutMs: 30000 }
blockedTopics: [/rosout, /parameter_events]
blockedServices: [/kill, /shutdown]
```

### Drone / aerial robot

```yaml
name: drone
velocity: { linearMax: 2.0, angularMax: 3.0, clampMode: true }
geofence: { xMin: -20, xMax: 20, yMin: -20, yMax: 20, zMin: 0, zMax: 30 }
deadmanSwitch: { enabled: true, timeoutMs: 10000 }
blockedTopics: [/rosout, /parameter_events]
blockedServices: [/kill, /shutdown, /emergency_land]
```

### Robot arm / manipulator

```yaml
name: robot-arm
velocity: { linearMax: 0.1, angularMax: 0.5, clampMode: false }
geofence: { xMin: -1, xMax: 1, yMin: -1, yMax: 1, zMin: 0, zMax: 1.5 }
deadmanSwitch: { enabled: true, timeoutMs: 5000 }
blockedTopics: [/rosout, /parameter_events]
blockedServices: [/kill, /shutdown]
```

Note: For robot arms, velocity limits apply to end-effector commands via `cmd_vel`-like topics. Joint velocity limits require custom safety checks (planned for future versions).

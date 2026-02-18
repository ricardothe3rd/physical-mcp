# I Built the First Safety-First MCP Server for Controlling Robots with AI

**TL;DR:** I built [PhysicalMCP](https://github.com/ricardothe3rd/physical-mcp) — an open-source MCP server that bridges AI agents (Claude, GPT, any LLM) to ROS2 robots with built-in velocity limits, geofence, emergency stop, rate limiting, and full audit logging. Every other AI-to-robot bridge passes commands straight through with zero guardrails. This is the first one that says "no."

---

## The Problem

AI agents are incredible at reasoning, planning, and using tools. Claude can write code, query databases, manage files, and now — thanks to MCP (Model Context Protocol) — interact with virtually any system through standardized tool interfaces.

But here's what keeps me up at night: **what happens when you connect an AI to a physical robot?**

There are already a few open-source MCP-to-ROS2 bridges out there. They work. You can tell Claude "move the robot forward" and it does. But they all share one critical flaw:

**There are no guardrails.**

The AI sends `cmd_vel` with a linear velocity of 10 m/s? It goes through. The AI floods the ROS2 graph with 1000 publish calls per second? It goes through. The AI calls `/shutdown` on the robot? It goes through.

For simulation and hobby projects, that's fine. For anything involving real hardware, other people, or expensive equipment, it's terrifying.

## The Solution: PhysicalMCP

PhysicalMCP is a safety-first MCP server that sits between the AI agent and your ROS2 robot. Every command passes through a policy engine before it reaches the robot:

```
AI Agent → MCP Server → [Safety Layer] → WebSocket → ROS2 Bridge → Robot
```

If a command violates any safety policy, it's **blocked** — the robot never receives it. The violation is logged in an audit trail. The AI gets a clear error message explaining what happened and why.

### What gets checked

**Velocity limits** — Every `geometry_msgs/msg/Twist` message is checked against configurable linear and angular velocity maximums. The check uses magnitude (sqrt of components), not just individual axis values.

```
"Move forward at 0.1 m/s"  → ✅ Published (under 0.5 m/s limit)
"Move forward at 5.0 m/s"  → ❌ BLOCKED: Linear velocity 5.00 m/s exceeds limit of 0.5 m/s
```

**Geofence** — Define rectangular workspace boundaries. Any command that would move the robot outside these bounds is blocked. Configurable per-robot.

**Rate limiting** — Sliding window rate limiter prevents flooding. Default: 10 Hz for publishes, 60 service calls/minute, 30 action goals/minute.

**Blocked topics and services** — System topics like `/rosout` and `/parameter_events` are off-limits. Dangerous services like `/kill` and `/shutdown` are blocked. Fully configurable.

**Emergency stop** — Dual-layer e-stop. The TypeScript server blocks all commands. The Python bridge publishes zero velocity to `/cmd_vel` and cancels all active action goals. Releasing the e-stop requires the explicit string `CONFIRM_RELEASE` — no accidental resumption.

**Audit logging** — Every command is recorded: timestamp, type, target, safety result, violations, execution outcome. Query it anytime with the `safety_audit_log` tool.

## The Architecture

PhysicalMCP is a hybrid TypeScript + Python system:

- **TypeScript MCP server** — Handles the MCP protocol, registers 21 tools, runs the safety policy engine. This is the npm package you install.
- **Python ROS2 bridge** — An rclpy node that actually talks to ROS2. Runs in a Docker container (or anywhere with ROS2 installed).
- **WebSocket** — Connects the two. Simple JSON protocol with request/response matching, heartbeat, and auto-reconnect.

The safety layer lives in the TypeScript server, which means it enforces limits **before** any command crosses the WebSocket to the bridge. The bridge has a secondary e-stop as a fail-safe, but the primary enforcement is server-side.

```
┌──────────────┐              ┌──────────────────────────────────┐
│   AI Agent   │◄────────────►│     MCP Server (TypeScript)      │
│  (Claude,    │  MCP         │                                  │
│  GPT, etc.)  │  protocol    │  ┌──────────────────────────┐    │
└──────────────┘              │  │      Safety Layer        │    │
                              │  │  Velocity · Geofence     │    │
                              │  │  Rate Limit · E-Stop     │    │
                              │  │  Blocked · Audit Log     │    │
                              │  └──────────────────────────┘    │
                              └───────────────┬──────────────────┘
                                              │ WebSocket
                              ┌───────────────▼──────────────────┐
                              │     ROS2 Bridge (Python)         │
                              │  rclpy · websockets              │
                              └───────────────┬──────────────────┘
                                              │ ROS2 DDS
                              ┌───────────────▼──────────────────┐
                              │          Robot                   │
                              └──────────────────────────────────┘
```

## 21 MCP Tools

The server exposes a complete set of ROS2 tools:

| Category | Tools | Safety Checked |
|----------|-------|:--------------:|
| **Topics** | list, info, subscribe, publish, echo | publish |
| **Services** | list, info, call | call |
| **Actions** | list, send_goal, cancel, status | send_goal |
| **Safety** | status, e-stop, release, get_policy, update_velocity, update_geofence, audit_log | — |
| **System** | bridge_status, node_list | — |

Read-only tools (list, subscribe, echo, info) pass through without safety checks. Write tools (publish, call, send_goal) go through the full policy engine.

## Safety Policies Are YAML

Safety limits are configured via simple YAML files:

```yaml
name: turtlebot3
description: TurtleBot3 Burger safety policy

velocity:
  linearMax: 0.22   # TurtleBot3 Burger hardware max
  angularMax: 2.84

geofence:
  xMin: -10.0
  xMax: 10.0
  yMin: -10.0
  yMax: 10.0
  zMin: 0.0
  zMax: 2.0

rateLimits:
  publishHz: 10
  servicePerMinute: 60
  actionPerMinute: 30

blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
```

Swap the policy file and the limits change instantly. No code changes. No recompilation. Partial policies are supported — omit any field and it falls back to safe defaults.

## Quick Start

### 1. Start the simulation

```bash
git clone https://github.com/ricardothe3rd/physical-mcp.git
cd physical-mcp/docker
docker compose up
```

This launches Gazebo with a TurtleBot3 and the Python ROS2 bridge.

### 2. Add PhysicalMCP to Claude

```bash
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp
```

### 3. Talk to your robot

Ask Claude: "List all ROS2 topics" or "Move the robot forward at 0.1 m/s" or "Activate emergency stop."

## Why I Built This

I've been working in robotics and AI for a while, and the gap between "AI can reason about the physical world" and "AI can safely act in the physical world" is enormous. The reasoning capabilities are there. The safety infrastructure is not.

Every major robotics company I've talked to has the same concern: "We'd love to use AI agents for robot operations, but we can't just let an LLM send arbitrary commands to a $50K robot." The safety layer is the missing piece.

PhysicalMCP is that missing piece. Open source, MIT licensed, and designed to be the standard safety layer for AI-to-robot communication.

## What's Next

- **Cloud relay** — Remote robot access via a hosted WebSocket relay (no VPN needed)
- **Dashboard** — Real-time visualization of safety events, audit trail, and robot state
- **More safety features** — Deadman switch, velocity clamping (reduce to max instead of blocking), workspace zones
- **Robot-specific policies** — Community-contributed policies for popular platforms (UR5, Spot, Franka, etc.)
- **Certification** — Formal safety analysis for industrial deployments

## Try It

GitHub: [github.com/ricardothe3rd/physical-mcp](https://github.com/ricardothe3rd/physical-mcp)

Install:
```bash
npx @ricardothe3rd/physical-mcp
```

If you're connecting AI to robots, you need a safety layer. This is it.

---

*If you have feedback, feature requests, or want to contribute robot-specific safety policies, open an issue on GitHub or leave a comment below. I'd especially love to hear from anyone running AI agents on real hardware — what safety constraints do you need?*

---

**Tags:** #mcp #ros2 #robotics #ai #safety #opensource #typescript #python

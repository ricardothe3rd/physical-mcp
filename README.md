# PhysicalMCP

[![CI](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/@ricardothe3rd/physical-mcp)](https://www.npmjs.com/package/@ricardothe3rd/physical-mcp)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Node.js](https://img.shields.io/badge/node-%3E%3D18-brightgreen)](https://nodejs.org)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.x-blue)](https://www.typescriptlang.org/)

**Safety-first MCP server for ROS2 robots.** Bridge AI agents to physical systems with built-in velocity limits, geofence boundaries, emergency stop, rate limiting, and full audit logging.

```bash
npx @ricardothe3rd/physical-mcp
```

## Table of Contents

- [Why PhysicalMCP?](#why-physicalmcp)
- [What It Looks Like](#what-it-looks-like)
- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [What It Looks Like In Practice](#what-it-looks-like-in-practice)
- [MCP Tools (40)](#mcp-tools-40)
- [Safety Layer](#safety-layer)
- [Configuration](#configuration)
- [Development](#development)
- [Requirements](#requirements)
- [FAQ](#faq)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Why PhysicalMCP?

Existing MCP-to-ROS2 bridges let AI agents send raw commands to robots with zero guardrails. That's fine for simulation — dangerous for real hardware.

PhysicalMCP enforces a safety layer **before** any command reaches your robot:

| Feature | PhysicalMCP | robotmcp | wise-vision-mcp |
|---------|:-----------:|:--------:|:----------------:|
| Velocity limits | Yes | No | No |
| Geofence boundaries | Yes | No | No |
| Emergency stop | Yes | No | No |
| Rate limiting | Yes | No | No |
| Audit logging | Yes | No | No |
| Blocked topics/services | Yes | No | No |
| YAML policy configs | Yes | No | No |
| Acceleration limits | Yes | No | No |
| Per-topic velocity limits | Yes | No | No |
| Command approval (human-in-loop) | Yes | No | No |
| Safety score tracking | Yes | No | No |
| Deadman switch | Yes | No | No |
| Batch command execution | Yes | No | No |
| Topic recording | Yes | No | No |
| MCP tools | **40** | 8 | 5 |
| ROS2 full coverage | Yes | Yes | Partial |

## What It Looks Like

### Safe command — allowed

```
You: "Move the robot forward at 0.1 m/s"

Claude calls ros2_topic_publish → Safety check passes (0.1 < 0.5 m/s limit)

> Published to /cmd_vel successfully
```

### Dangerous command — blocked

```
You: "Move the robot forward at 5 m/s"

Claude calls ros2_topic_publish → Safety check FAILS

> SAFETY BLOCKED: Publish to /cmd_vel denied.
>
> Violations:
> - [velocity_exceeded] Linear velocity 5.00 m/s exceeds limit of 0.5 m/s
```

The command never reaches the robot. The violation is logged in the audit trail.

### Emergency stop

```
You: "Activate emergency stop"

> EMERGENCY STOP ACTIVATED
>
> Reason: Manual activation
>
> All commands are now blocked. Zero velocity published to /cmd_vel.
> Use safety_emergency_stop_release with confirmation "CONFIRM_RELEASE" to resume.

You: "Move the robot forward at 0.1 m/s"

> SAFETY BLOCKED: Publish to /cmd_vel denied.
>
> Violations:
> - [emergency_stop_active] Emergency stop is active. Release e-stop before publishing.
```

Even a perfectly safe command is blocked while e-stop is active. Releasing requires the explicit string `CONFIRM_RELEASE`.

### Audit trail

```
You: "Show me the safety audit log"

> Audit Log (8 total, 3 blocked, 0 errors)
>
> [
>   { "command": "publish", "target": "/cmd_vel", "safetyResult": { "allowed": true } },
>   { "command": "emergency_stop_release", "target": "system", "safetyResult": { "allowed": true } },
>   { "command": "publish", "target": "/cmd_vel", "safetyResult": { "allowed": false,
>       "violations": [{ "type": "emergency_stop_active" }] } },
>   { "command": "emergency_stop", "target": "system", "safetyResult": { "allowed": true } },
>   { "command": "publish", "target": "/cmd_vel", "safetyResult": { "allowed": false,
>       "violations": [{ "type": "velocity_exceeded" }] } },
>   ...
> ]
```

Every command — allowed or blocked — is recorded with full context.

<!-- TODO: Replace text examples above with GIF demos once recorded -->
<!-- GIF 1: Split-screen showing Claude sending commands + Gazebo robot responding + safety block -->
<!-- GIF 2: Emergency stop activation + robot halting -->

## Architecture

```
┌──────────────┐              ┌────────────────────────────────────────┐
│   AI Agent   │◄────────────►│       MCP Server (TypeScript)          │
│  (Claude,    │  MCP         │                                        │
│  GPT, etc.)  │  protocol    │  ┌──────────────────────────────────┐  │
└──────────────┘              │  │           Safety Layer           │  │
                              │  │                                  │  │
                              │  │   ┌──────────┐  ┌────────────┐   │  │
                              │  │   │ Velocity │  │  Geofence  │   │  │
                              │  │   │  Limits  │  │ Boundaries │   │  │
                              │  │   └──────────┘  └────────────┘   │  │
                              │  │   ┌──────────┐  ┌────────────┐   │  │
                              │  │   │   Rate   │  │  E-Stop +  │   │  │
                              │  │   │  Limiter │  │ Audit Log  │   │  │
                              │  │   └──────────┘  └────────────┘   │  │
                              │  │                                  │  │
                              │  └──────────────────────────────────┘  │
                              └───────────────────┬────────────────────┘
                                                  │
                                         WebSocket (port 9090)
                                                  │
                              ┌───────────────────▼────────────────────┐
                              │        ROS2 Bridge (Python)            │
                              │  rclpy + websockets                    │
                              │  Secondary e-stop as fail-safe         │
                              └───────────────────┬────────────────────┘
                                                  │
                                             ROS2 DDS
                                                  │
                              ┌───────────────────▼────────────────────┐
                              │           ROS2 Robot                   │
                              │  Topics, Services, Actions             │
                              └────────────────────────────────────────┘
```

The safety layer sits in the TypeScript MCP server and evaluates **every** publish, service call, and action goal before it reaches the bridge. The Python bridge has a secondary emergency stop as an additional fail-safe.

## Quick Start

### 1. Start the simulation + bridge

```bash
cd docker
docker compose up
```

This launches a Gazebo simulation with TurtleBot3 and the ROS2 bridge on port 9090.

### 2. Add PhysicalMCP to Claude

```bash
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp
```

### 3. Talk to your robot

Ask Claude things like:
- "List all ROS2 topics"
- "What's the robot's current pose?"
- "Move the robot forward at 0.1 m/s"
- "Show me the safety status"
- "Activate emergency stop"

## What It Looks Like In Practice

Here is what you see in the terminal when Claude interacts with a ROS2 robot through PhysicalMCP.

### Listing topics

```
You: "What ROS2 topics are available?"

Claude calls ros2_topic_list

> Available topics (12):
>
>  Topic                          Type
>  ─────────────────────────────  ───────────────────────────────────
>  /cmd_vel                       geometry_msgs/msg/Twist
>  /odom                          nav_msgs/msg/Odometry
>  /scan                          sensor_msgs/msg/LaserScan
>  /camera/image_raw              sensor_msgs/msg/Image
>  /tf                            tf2_msgs/msg/TFMessage
>  /joint_states                  sensor_msgs/msg/JointState
>  /imu                           sensor_msgs/msg/Imu
>  /battery_state                 sensor_msgs/msg/BatteryState
>  /clock                         rosgraph_msgs/msg/Clock
>  /rosout                        rcl_interfaces/msg/Log
>  /parameter_events              rcl_interfaces/msg/ParameterEvent
>  /robot_description             std_msgs/msg/String
```

### Publishing a velocity command

```
You: "Move the robot forward slowly at 0.15 m/s"

Claude calls ros2_topic_publish with topic=/cmd_vel, msg={linear: {x: 0.15}, angular: {z: 0.0}}

> Safety check: PASSED
>   linear velocity 0.15 m/s within limit (0.5 m/s)
>   angular velocity 0.00 rad/s within limit (1.5 rad/s)
>
> Published to /cmd_vel successfully
```

### Safety layer blocking a dangerous command

```
You: "Send the robot forward at maximum speed, 10 m/s"

Claude calls ros2_topic_publish with topic=/cmd_vel, msg={linear: {x: 10.0}, angular: {z: 0.0}}

> SAFETY BLOCKED: Publish to /cmd_vel denied.
>
> Violations:
>   - [velocity_exceeded] Linear velocity 10.00 m/s exceeds limit of 0.5 m/s
>
> The command was NOT sent to the robot.
> Logged to audit trail: entry #47, timestamp 2026-02-19T14:23:01.442Z
```

The safety layer intercepts every command before it reaches the ROS2 bridge. Blocked commands never touch the robot, and every decision (allowed or blocked) is recorded in the audit log.

## MCP Tools (40)

### Topic Tools (5)

| Tool | Description |
|------|-------------|
| `ros2_topic_list` | List all available topics with their message types |
| `ros2_topic_info` | Get details for a topic (publishers, subscribers) |
| `ros2_topic_subscribe` | Collect N messages from a topic |
| `ros2_topic_publish` | Publish a message to a topic *(safety checked)* |
| `ros2_topic_echo` | Get the latest message from a topic |

### Service Tools (3)

| Tool | Description |
|------|-------------|
| `ros2_service_list` | List all available services |
| `ros2_service_info` | Get service type information |
| `ros2_service_call` | Call a ROS2 service *(safety checked)* |

### Action Tools (4)

| Tool | Description |
|------|-------------|
| `ros2_action_list` | List all action servers |
| `ros2_action_send_goal` | Send a goal to an action server *(safety checked)* |
| `ros2_action_cancel` | Cancel a specific or all active goals |
| `ros2_action_status` | Get status of active goals |

### Safety Tools (18)

| Tool | Description |
|------|-------------|
| `safety_status` | Get full safety system status |
| `safety_emergency_stop` | Activate emergency stop — blocks all commands |
| `safety_emergency_stop_release` | Release e-stop (requires `CONFIRM_RELEASE`) |
| `safety_get_policy` | Get current safety policy configuration |
| `safety_update_velocity_limits` | Adjust velocity limits at runtime |
| `safety_update_geofence` | Adjust geofence boundaries at runtime |
| `safety_audit_log` | Query the command audit trail with filtering |
| `safety_set_clamp_mode` | Toggle velocity clamping (reduce to max vs block) |
| `safety_deadman_switch` | Configure auto e-stop on heartbeat timeout |
| `safety_heartbeat` | Send heartbeat to prevent deadman switch e-stop |
| `safety_update_acceleration_limits` | Configure max acceleration/deceleration |
| `safety_export_audit_log` | Export audit trail to JSON file |
| `safety_check_position` | Check geofence compliance + proximity warnings |
| `safety_validate_policy` | Validate policy configuration for errors/warnings |
| `safety_approval_list` | List pending command approval requests |
| `safety_approval_approve` | Approve a pending command (human-in-the-loop) |
| `safety_approval_deny` | Deny a pending command execution |
| `safety_approval_config` | Configure command approval settings |

### System Tools (6)

| Tool | Description |
|------|-------------|
| `system_bridge_status` | Check bridge connection health + latency |
| `system_node_list` | List all ROS2 nodes |
| `system_node_info` | Get detailed info for a specific node |
| `ros2_param_list` | List parameters for a node |
| `ros2_param_get` | Get a parameter value |
| `ros2_param_set` | Set a parameter value *(safety checked)* |

### Batch & Recording Tools (4)

| Tool | Description |
|------|-------------|
| `ros2_batch_execute` | Execute multiple commands in a single call |
| `ros2_topic_record_start` | Start recording messages from a topic |
| `ros2_topic_record_stop` | Stop recording and retrieve messages |
| `ros2_topic_record_status` | Get status of all active recordings |

## Safety Layer

Every command that could move or affect the robot passes through the policy engine.

### Velocity Limits

Caps linear and angular velocity on any `geometry_msgs/msg/Twist` message:

```yaml
velocity:
  linearMax: 0.5   # m/s (default)
  angularMax: 1.5  # rad/s (default)
```

If an AI agent tries to publish a velocity above the limit, the command is **blocked** and the violation is logged.

### Geofence

Defines the workspace boundaries the robot must stay within:

```yaml
geofence:
  xMin: -5.0
  xMax: 5.0
  yMin: -5.0
  yMax: 5.0
  zMin: 0.0
  zMax: 2.0
```

### Rate Limiting

Prevents flooding the ROS2 graph with too many commands:

```yaml
rateLimits:
  publishHz: 10          # max publishes per second
  servicePerMinute: 60   # max service calls per minute
  actionPerMinute: 30    # max action goals per minute
```

### Blocked Topics & Services

Prevents access to sensitive system topics:

```yaml
blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
```

### Velocity Clamping

Instead of blocking over-limit velocities, clamp mode scales them down to the maximum while preserving direction:

```yaml
velocity:
  linearMax: 0.22
  clampMode: true   # reduce to max instead of blocking
```

Toggle at runtime with `safety_set_clamp_mode`.

### Deadman Switch

Auto-activates emergency stop if no heartbeat is received within the timeout:

```yaml
deadmanSwitch:
  enabled: true
  timeoutMs: 30000   # 30 seconds
```

Call `safety_heartbeat` periodically to keep the robot active. If the AI agent disconnects, the deadman switch stops the robot automatically.

### Acceleration Limits

Prevent jerky motion by limiting how fast velocity can change:

```yaml
acceleration:
  enabled: true
  linearMaxAccel: 1.0    # m/s²
  angularMaxAccel: 3.0   # rad/s²
```

When enabled, the engine tracks velocity over time and blocks commands that would require acceleration beyond the configured limits.

### Geofence Proximity Warnings

Get warned before hitting the boundary:

```yaml
geofenceWarningMargin: 1.0  # warn within 1m of boundary
```

Use `safety_check_position` to check any position against the geofence and get both violation status and proximity warnings.

### Emergency Stop

Dual-layer emergency stop:

1. **Primary (TypeScript):** Blocks all publish/service/action commands at the MCP server
2. **Secondary (Python):** Bridge-level e-stop publishes zero velocity to `/cmd_vel` and cancels all active action goals

Releasing requires explicit confirmation (`CONFIRM_RELEASE`) to prevent accidental resumption.

### Per-Topic Velocity Limits

Different speed limits for different robots or joints:

```yaml
topicVelocityOverrides:
  - topic: /arm/cmd_vel
    linearMax: 0.1        # Arm moves slowly
    angularMax: 0.5
  - topic: /base/cmd_vel
    linearMax: 1.0        # Base can go faster
```

### Command Approval (Human-in-the-Loop)

Require explicit human approval before executing dangerous commands:

```yaml
commandApproval:
  enabled: true
  requireApprovalFor:
    - ros2_topic_publish
    - ros2_action_send_goal
  pendingTimeout: 30000   # 30s before approval expires
```

Use `safety_approval_list` to see pending requests, and `safety_approval_approve` or `safety_approval_deny` to resolve them.

### Safety Score

Track how "safe" an AI agent session has been:

- **Score:** 0-100 (100 = all commands allowed, 0 = all blocked)
- **Block rate:** Percentage of commands blocked
- **Violations by type:** Breakdown of what went wrong
- **Session duration:** How long the session has been running

Check with `safety_status` — the `safetyScore` field shows the current snapshot.

### Safety Events

Subscribe to real-time safety events: violations, e-stop activations, policy changes. The event emitter provides both type-specific and catch-all listeners with automatic history.

### Audit Logging

Every command is logged with:
- Timestamp and unique ID
- Command type and target
- Safety check result (allowed/blocked)
- Violation details if blocked
- Execution result

Query the audit trail with `safety_audit_log` — filter by violations only, command type, or limit results.

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `PHYSICAL_MCP_BRIDGE_URL` | `ws://localhost:9090` | WebSocket URL for the ROS2 bridge |
| `PHYSICAL_MCP_POLICY` | *(built-in defaults)* | Path to a YAML safety policy file |

### Safety Policies

Create a YAML file and point to it with `PHYSICAL_MCP_POLICY`:

```yaml
name: my-robot
description: Custom policy for my robot

velocity:
  linearMax: 0.3
  angularMax: 1.0

geofence:
  xMin: -2.0
  xMax: 2.0
  yMin: -2.0
  yMax: 2.0
  zMin: 0.0
  zMax: 1.0

rateLimits:
  publishHz: 5
  servicePerMinute: 30
  actionPerMinute: 15

blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
```

Partial policies are supported — any field you omit falls back to the defaults.

Two built-in policies are included:
- **`default.yaml`** — Conservative limits for any ROS2 robot
- **`turtlebot3.yaml`** — Tuned for TurtleBot3 Burger in Gazebo (0.22 m/s linear, 2.84 rad/s angular)

### Claude Desktop Configuration

Add to your Claude Desktop `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "physical-mcp": {
      "command": "npx",
      "args": ["@ricardothe3rd/physical-mcp"],
      "env": {
        "PHYSICAL_MCP_BRIDGE_URL": "ws://localhost:9090",
        "PHYSICAL_MCP_POLICY": "/path/to/my-policy.yaml"
      }
    }
  }
}
```

## Development

### Project Structure

```
physical-mcp/
├── packages/
│   ├── mcp-server/              # TypeScript MCP server (npm package)
│   │   ├── src/
│   │   │   ├── index.ts         # Entry point, tool registration
│   │   │   ├── bridge/          # WebSocket client + connection manager
│   │   │   ├── tools/           # 21 MCP tool definitions
│   │   │   ├── safety/          # Policy engine, geofence, rate limiter, audit
│   │   │   ├── types/           # ROS2 type definitions
│   │   │   └── utils/           # Error recovery, circuit breaker
│   │   └── policies/            # YAML safety policy files
│   └── ros2-bridge/             # Python ROS2 bridge (rclpy + websockets)
│       └── physical_mcp_bridge/
│           ├── bridge_node.py   # Main node + WebSocket server
│           ├── protocol.py      # Shared protocol (mirrors TS)
│           ├── discovery.py     # ROS2 graph discovery
│           ├── topic_handler.py # Topic pub/sub/echo
│           ├── service_handler.py
│           ├── action_handler.py
│           └── telemetry.py     # Bridge health metrics
└── docker/
    ├── docker-compose.yml       # Full stack (sim + bridge)
    ├── Dockerfile.bridge        # ROS2 Humble + bridge
    └── Dockerfile.sim           # Gazebo + TurtleBot3
```

### Building

```bash
# TypeScript MCP server
cd packages/mcp-server
npm install
npm run build

# Python bridge (in ROS2 environment)
cd packages/ros2-bridge
pip install -e .
```

### Running Locally

```bash
# Terminal 1: Start the bridge (requires ROS2)
physical-mcp-bridge

# Terminal 2: Start the MCP server
cd packages/mcp-server
npm run dev
```

### Testing

```bash
cd packages/mcp-server
npm test
```

### Docker

```bash
# Build and start everything
cd docker
docker compose up --build

# The bridge runs on port 9090
# Gazebo sim launches with TurtleBot3 Burger
```

## Requirements

- **MCP Server:** Node.js >= 18
- **ROS2 Bridge:** Python >= 3.10, ROS2 Humble
- **Docker:** Docker + Docker Compose (for simulation)

## FAQ

**Q: How is this different from robotmcp?**
A: robotmcp and other MCP-ROS2 bridges pass commands directly to the robot with no safety checks. PhysicalMCP adds a full safety layer (velocity limits, geofence, rate limiting, e-stop, audit logging) that evaluates every command before it reaches the robot. See the [comparison table](#why-physicalmcp).

**Q: Does this work with local LLMs?**
A: Yes. PhysicalMCP is an MCP server, so it works with any MCP-compatible client — Claude, GPT, or local models running through an MCP client. The safety layer is the same regardless of which AI sends the commands.

**Q: Can I use this with real hardware (not just simulation)?**
A: Yes. The MCP server connects to any ROS2 system via the Python bridge. Start the bridge on your robot (or a machine with ROS2 access to it), point `PHYSICAL_MCP_BRIDGE_URL` at it, and it works. **Start with conservative safety limits** and test thoroughly before using real hardware.

**Q: What happens if the WebSocket connection drops?**
A: The connection manager automatically attempts reconnection every 5 seconds. A circuit breaker prevents flooding if the bridge is down. Commands sent while disconnected return a clear error. The bridge-side e-stop remains active independently.

**Q: Can I have different safety policies for different robots?**
A: Yes. Set `PHYSICAL_MCP_POLICY` to point at a YAML file with your robot-specific limits. Two built-in policies are included (default + TurtleBot3). You can also update limits at runtime via the `safety_update_velocity_limits` and `safety_update_geofence` tools.

**Q: Is the safety layer bypassable?**
A: Not through the MCP interface. All publish, service, and action commands pass through the policy engine in the TypeScript server. The Python bridge has a secondary e-stop as an additional fail-safe. There is no "skip safety" flag.

## Troubleshooting

### Bridge not connected

```
Bridge not connected. Start the ROS2 bridge and try again.
Expected bridge at: ws://localhost:9090
```

**Fix:** Make sure the Python bridge is running. If using Docker: `cd docker && docker compose up`. If running locally: `physical-mcp-bridge`. Check that port 9090 is not blocked by a firewall.

### tsc build hangs

If `npm run build` hangs, ensure you're using the version of the tool files with the `toInputSchema()` helper function. The `zodToJsonSchema` return type causes TypeScript type inference to loop when cast directly to `Tool['inputSchema']`.

### Docker Gazebo display issues

If Gazebo doesn't show a window on macOS/Linux, you need X11 forwarding:

```bash
# macOS: install XQuartz, then:
xhost +local:docker

# Linux:
xhost +local:root
```

### ROS2 topics not showing up

If `ros2_topic_list` returns empty, the bridge may have started before the simulation. Restart the bridge after the simulation is fully loaded:

```bash
docker compose restart bridge
```

### Safety violations blocking commands unexpectedly

If the safety layer is blocking commands you expect to succeed, check which policy is active:

```
You: "Show me the safety status"
```

Look at the velocity limits and geofence boundaries. If they are too restrictive for your use case, you can adjust them at runtime:

```
You: "Update the linear velocity limit to 1.0 m/s"
```

Or create a custom YAML policy file with your desired limits and set `PHYSICAL_MCP_POLICY` to point at it. See the [Safety Layer](#safety-layer) section for all configurable options.

### WebSocket connection drops

If you see frequent disconnections between the MCP server and the ROS2 bridge:

1. **Check network stability** between the machine running the MCP server and the machine running the bridge.
2. **Check the bridge process** is still running: `docker compose ps` (if using Docker) or check the process directly.
3. **Deadman switch timeout**: If you have the deadman switch enabled, make sure `safety_heartbeat` is being called within the configured `timeoutMs`. If the heartbeat lapses, the system activates emergency stop and the bridge may appear unresponsive.
4. **Increase reconnection tolerance**: The connection manager retries every 5 seconds with a circuit breaker. If the bridge takes longer to restart, you may see errors during the reconnection window. These resolve automatically once the bridge is back.

### Permission denied on Linux

If the bridge or Docker containers fail with permission errors on Linux:

```
PermissionError: [Errno 13] Permission denied
```

**Fix:** Do not run the bridge as root. Instead, add your user to the `docker` group:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

For the ROS2 bridge running natively (not in Docker), ensure your user has access to the ROS2 workspace and DDS middleware. Some DDS implementations create shared-memory files in `/dev/shm` that require appropriate permissions.

### Node.js version incompatibility

```
SyntaxError: Unexpected token '?.'
```

or

```
Error: PhysicalMCP requires Node.js >= 18
```

**Fix:** PhysicalMCP uses modern JavaScript features (optional chaining, top-level await, ES modules) that require Node.js 18 or later. Check your version:

```bash
node --version
```

If you are on an older version, upgrade using [nvm](https://github.com/nvm-sh/nvm):

```bash
nvm install 18
nvm use 18
```

### Rate limit errors during normal use

The default policy limits publishes to 10 Hz. If you're sending commands faster than that (e.g., in a control loop), either increase the limit in your policy YAML or use the `safety_update_velocity_limits` tool.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup and guidelines.

## License

MIT - see [LICENSE](LICENSE) for details.

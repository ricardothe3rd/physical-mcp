<div align="center">

# PhysicalMCP

### The safety layer between AI agents and real robots

[![CI](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/@ricardothe3rd/physical-mcp)](https://www.npmjs.com/package/@ricardothe3rd/physical-mcp)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Node.js](https://img.shields.io/badge/node-%3E%3D18-brightgreen)](https://nodejs.org)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.x-blue)](https://www.typescriptlang.org/)
[![Tests](https://img.shields.io/badge/tests-2167%20passing-brightgreen)](https://github.com/ricardothe3rd/physical-mcp)
[![Tools](https://img.shields.io/badge/MCP%20tools-100-blueviolet)](https://github.com/ricardothe3rd/physical-mcp)

**100 MCP tools** | **25 safety modules** | **2167 tests** | **Zero unsafe commands reach your robot**

[Quick Start](#quick-start) | [Why PhysicalMCP?](#why-physicalmcp) | [Safety Layer](#safety-layer) | [All Tools](#mcp-tools-100)

</div>

---

Bridge any AI agent (Claude, GPT, Gemini, local LLMs) to any ROS2 robot with built-in velocity limits, geofence boundaries, emergency stop, rate limiting, and full audit logging. Every command is safety-checked **before** it reaches your robot.

```bash
npx @ricardothe3rd/physical-mcp
```

## Table of Contents

- [Why PhysicalMCP?](#why-physicalmcp)
- [What It Looks Like](#what-it-looks-like)
- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [MCP Tools (100)](#mcp-tools-100)
- [Safety Layer](#safety-layer)
- [Configuration](#configuration)
- [Development](#development)
- [Requirements](#requirements)
- [FAQ & Troubleshooting](#faq)
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
| Fleet management | Yes | No | No |
| Path planning tools | Yes | No | No |
| Camera/sensor tools | Yes | No | No |
| MCP tools | **100** | 8 | 5 |
| Tests | **2167** | ~50 | ~20 |
| ROS2 full coverage | Yes | Yes | Partial |

## What It Looks Like

```
You: "Move the robot forward slowly"
Claude → ros2_topic_publish(topic=/cmd_vel, msg={linear: {x: 0.15}})
✅ Safety check PASSED → Published to /cmd_vel

You: "Go full speed, 10 m/s!"
Claude → ros2_topic_publish(topic=/cmd_vel, msg={linear: {x: 10.0}})
🛑 BLOCKED: Linear velocity 10.00 m/s exceeds limit of 0.5 m/s
   → Command never reaches the robot. Logged to audit trail.

You: "Emergency stop!"
Claude → safety_emergency_stop(reason="Manual activation")
🔴 E-STOP ACTIVE → Zero velocity sent. All commands blocked.
   → Release requires explicit "CONFIRM_RELEASE" string.
```

Every command — allowed or blocked — is recorded in the audit trail with timestamps, violations, and full context.

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


## MCP Tools (100)

100 tools across 25 categories. Every publish, service call, and action goal passes through the safety layer.

### Core ROS2 Tools

<details>
<summary><strong>Topic Tools (5)</strong> — publish, subscribe, echo messages</summary>

| Tool | Description |
|------|-------------|
| `ros2_topic_list` | List all available topics with their message types |
| `ros2_topic_info` | Get details for a topic (publishers, subscribers) |
| `ros2_topic_subscribe` | Collect N messages from a topic |
| `ros2_topic_publish` | Publish a message to a topic *(safety checked)* |
| `ros2_topic_echo` | Get the latest message from a topic |
</details>

<details>
<summary><strong>Service Tools (3)</strong> — call ROS2 services</summary>

| Tool | Description |
|------|-------------|
| `ros2_service_list` | List all available services |
| `ros2_service_info` | Get service type information |
| `ros2_service_call` | Call a ROS2 service *(safety checked)* |
</details>

<details>
<summary><strong>Action Tools (4)</strong> — send goals, cancel, track status</summary>

| Tool | Description |
|------|-------------|
| `ros2_action_list` | List all action servers |
| `ros2_action_send_goal` | Send a goal to an action server *(safety checked)* |
| `ros2_action_cancel` | Cancel a specific or all active goals |
| `ros2_action_status` | Get status of active goals |
</details>

<details>
<summary><strong>System & Parameter Tools (7)</strong> — nodes, params, health</summary>

| Tool | Description |
|------|-------------|
| `system_bridge_status` | Check bridge connection health + latency |
| `system_node_list` | List all ROS2 nodes |
| `system_node_info` | Get detailed info for a specific node |
| `system_health_status` | Overall system health check |
| `ros2_param_list` | List parameters for a node |
| `ros2_param_get` | Get a parameter value |
| `ros2_param_set` | Set a parameter value *(safety checked)* |
</details>

### Safety Tools (21)

<details>
<summary><strong>Full safety tool list</strong> — e-stop, geofence, audit, approval, policies</summary>

| Tool | Description |
|------|-------------|
| `safety_status` | Get full safety system status + safety score |
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
| `safety_violation_mode` | Set warn vs block mode for violations |
| `safety_time_policy` | Configure time-based safety policies |
| `safety_time_policy_status` | Get status of active time-based policies |
</details>

### Robot Perception & Sensing

<details>
<summary><strong>Camera Tools (3)</strong> — image topics, camera info, previews</summary>

| Tool | Description |
|------|-------------|
| `ros2_camera_info` | Get camera intrinsics and calibration |
| `ros2_image_preview` | Get latest image from a camera topic |
| `ros2_image_topics` | List all available image topics |
</details>

<details>
<summary><strong>Sensor Tools (2)</strong> — IMU, lidar, sensor summaries</summary>

| Tool | Description |
|------|-------------|
| `ros2_sensor_summary` | Get summary of all sensor data |
| `ros2_sensor_read` | Read latest data from a specific sensor |
</details>

<details>
<summary><strong>TF Tools (2)</strong> — coordinate transforms</summary>

| Tool | Description |
|------|-------------|
| `ros2_tf_tree` | Get the full TF2 transform tree |
| `ros2_tf_lookup` | Look up transform between two frames |
</details>

### Navigation & Planning

<details>
<summary><strong>Path Tools (4)</strong> — path planning, navigation status</summary>

| Tool | Description |
|------|-------------|
| `ros2_plan_path` | Plan a path between two poses |
| `ros2_path_info` | Get info about a planned path |
| `ros2_costmap_update` | Force costmap update |
| `ros2_navigation_status` | Get Nav2 navigation status |
</details>

<details>
<summary><strong>Map Tools (3)</strong> — occupancy grids, costmaps</summary>

| Tool | Description |
|------|-------------|
| `ros2_map_info` | Get occupancy grid map info |
| `ros2_costmap_info` | Get costmap details |
| `ros2_map_topics` | List all map-related topics |
</details>

<details>
<summary><strong>Waypoint Tools (4)</strong> — save, navigate, manage waypoints</summary>

| Tool | Description |
|------|-------------|
| `ros2_waypoint_save` | Save current position as a named waypoint |
| `ros2_waypoint_list` | List all saved waypoints |
| `ros2_waypoint_delete` | Delete a saved waypoint |
| `ros2_waypoint_navigate` | Navigate to a saved waypoint |
</details>

### Fleet & Device Management

<details>
<summary><strong>Fleet Tools (3)</strong> — multi-robot management</summary>

| Tool | Description |
|------|-------------|
| `ros2_fleet_status` | Get status of all robots in the fleet |
| `ros2_fleet_add` | Register a new robot to the fleet |
| `ros2_fleet_remove` | Remove a robot from the fleet |
</details>

<details>
<summary><strong>Launch Tools (3)</strong> — start/stop ROS2 launch files</summary>

| Tool | Description |
|------|-------------|
| `ros2_launch_start` | Start a ROS2 launch file |
| `ros2_launch_stop` | Stop a running launch process |
| `ros2_launch_status` | Get status of launch processes |
</details>

<details>
<summary><strong>Namespace Tools (3)</strong> — ROS2 namespace management</summary>

| Tool | Description |
|------|-------------|
| `ros2_namespace_list` | List all active namespaces |
| `ros2_namespace_remap` | Remap a topic/service to a different namespace |
| `ros2_namespace_clear_remaps` | Clear all active remappings |
</details>

### Advanced Capabilities

<details>
<summary><strong>Batch & Conditional Tools (3)</strong> — execute multiple commands, conditions</summary>

| Tool | Description |
|------|-------------|
| `ros2_batch_execute` | Execute multiple commands in a single call |
| `ros2_conditional_execute` | Execute a command only if a condition is met |
| `ros2_wait_for_condition` | Wait for a topic/condition before proceeding |
</details>

<details>
<summary><strong>Scheduled Tools (3)</strong> — cron-like command scheduling</summary>

| Tool | Description |
|------|-------------|
| `ros2_schedule_command` | Schedule a command for future execution |
| `ros2_schedule_cancel` | Cancel a scheduled command |
| `ros2_schedule_list` | List all scheduled commands |
</details>

<details>
<summary><strong>Recording Tools (3)</strong> — record and replay topic data</summary>

| Tool | Description |
|------|-------------|
| `ros2_topic_record_start` | Start recording messages from a topic |
| `ros2_topic_record_stop` | Stop recording and retrieve messages |
| `ros2_topic_record_status` | Get status of all active recordings |
</details>

<details>
<summary><strong>History Tools (3)</strong> — command history and replay</summary>

| Tool | Description |
|------|-------------|
| `ros2_command_history` | View past commands sent through PhysicalMCP |
| `ros2_command_stats` | Get statistics on command usage |
| `ros2_command_replay` | Replay a previous command |
</details>

<details>
<summary><strong>Introspection Tools (3)</strong> — message/service/action type info</summary>

| Tool | Description |
|------|-------------|
| `ros2_msg_type_info` | Get fields and structure of a message type |
| `ros2_srv_type_info` | Get fields and structure of a service type |
| `ros2_action_type_info` | Get fields and structure of an action type |
</details>

<details>
<summary><strong>Diagnostic & Network Tools (5)</strong> — system diagnostics, network monitoring</summary>

| Tool | Description |
|------|-------------|
| `ros2_diagnostics_summary` | Get system diagnostics summary |
| `ros2_diagnostics_detail` | Get detailed diagnostics for a component |
| `ros2_network_stats` | Get ROS2 network statistics |
| `ros2_network_bandwidth` | Monitor network bandwidth usage |
| `ros2_network_latency_test` | Test network latency to bridge |
</details>

<details>
<summary><strong>Power & Description Tools (4)</strong> — battery, URDF, joint info</summary>

| Tool | Description |
|------|-------------|
| `ros2_battery_status` | Get battery level and charging status |
| `ros2_power_supply_status` | Get power supply information |
| `ros2_robot_description` | Get robot URDF/description |
| `ros2_robot_joints` | Get joint names and states |
</details>

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
│   │   │   ├── index.ts         # Entry point — 100 tool registrations
│   │   │   ├── bridge/          # WebSocket client, connection manager, heartbeat
│   │   │   ├── tools/           # 25 tool category files (100 tools total)
│   │   │   ├── safety/          # Policy engine, geofence, rate limiter, audit, collision
│   │   │   ├── types/           # ROS2 type definitions
│   │   │   └── utils/           # Logger, health checks, input validation, profiles
│   │   └── policies/            # YAML safety policy files
│   └── ros2-bridge/             # Python ROS2 bridge (rclpy + websockets)
│       └── physical_mcp_bridge/
│           ├── bridge_node.py   # Main ROS2 node + WebSocket server
│           ├── ws_manager.py    # WebSocket manager with backpressure
│           ├── protocol.py      # Shared protocol (mirrors TS)
│           ├── discovery.py     # ROS2 graph discovery
│           ├── topic_handler.py # Topic pub/sub/echo
│           ├── service_handler.py
│           ├── action_handler.py
│           ├── telemetry.py     # Bridge health metrics
│           └── msg_validator.py # Message type validation
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

<details>
<summary><strong>How is this different from robotmcp?</strong></summary>

robotmcp and other MCP-ROS2 bridges pass commands directly to the robot with no safety checks. PhysicalMCP adds a full safety layer (velocity limits, geofence, rate limiting, e-stop, audit logging) that evaluates every command before it reaches the robot. See the [comparison table](#why-physicalmcp).
</details>

<details>
<summary><strong>Does this work with local LLMs?</strong></summary>

Yes. PhysicalMCP is an MCP server, so it works with any MCP-compatible client — Claude, GPT, or local models running through an MCP client. The safety layer is the same regardless of which AI sends the commands.
</details>

<details>
<summary><strong>Can I use this with real hardware (not just simulation)?</strong></summary>

Yes. The MCP server connects to any ROS2 system via the Python bridge. Start the bridge on your robot (or a machine with ROS2 access to it), point `PHYSICAL_MCP_BRIDGE_URL` at it, and it works. **Start with conservative safety limits** and test thoroughly before using real hardware.
</details>

<details>
<summary><strong>What happens if the WebSocket connection drops?</strong></summary>

The connection manager automatically attempts reconnection every 5 seconds. A circuit breaker prevents flooding if the bridge is down. Commands sent while disconnected return a clear error. The bridge-side e-stop remains active independently.
</details>

<details>
<summary><strong>Can I have different safety policies for different robots?</strong></summary>

Yes. Set `PHYSICAL_MCP_POLICY` to point at a YAML file with your robot-specific limits. Two built-in policies are included (default + TurtleBot3). You can also update limits at runtime via the `safety_update_velocity_limits` and `safety_update_geofence` tools.
</details>

<details>
<summary><strong>Is the safety layer bypassable?</strong></summary>

Not through the MCP interface. All publish, service, and action commands pass through the policy engine in the TypeScript server. The Python bridge has a secondary e-stop as an additional fail-safe. There is no "skip safety" flag.
</details>

<details>
<summary><strong>Troubleshooting</strong> (click to expand)</summary>

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

### Bridge Connection Refused
**Error:** `WebSocket connection failed: ECONNREFUSED`
- Ensure the ROS2 bridge is running: `docker compose up bridge`
- Check the bridge URL matches: default is `ws://localhost:9090`
- Verify no firewall blocking port 9090

### Docker: Gazebo Display Issues
**Error:** `cannot open display` or blank Gazebo window
- Set `DISPLAY` env var: `export DISPLAY=:0`
- On macOS, install XQuartz and run `xhost +localhost`
- For headless mode, set `GAZEBO_HEADLESS=1`

### Safety Policy Not Loading
**Error:** `Failed to load policy file`
- Check the YAML syntax: `npx yaml-lint policies/default.yaml`
- Ensure the file path is absolute or relative to the server working directory
- Verify the policy structure matches the schema (see Safety Policy Guide)

### E-Stop Won't Release
**Behavior:** `safety_emergency_stop_release` returns error
- E-stop release requires explicit confirmation: pass `confirm: true`
- If bridge-level e-stop is active, restart the bridge container
- Check audit log for the original e-stop trigger: `safety_audit_log`

### High Latency Commands
**Behavior:** Commands take >1 second
- Check bridge connection: `system_bridge_status`
- Verify Docker is not resource-starved: `docker stats`
- Reduce rate limiter windows if they're too restrictive

</details>

## Contributing

Contributions welcome! See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup and guidelines.

Areas where help is especially wanted:
- Safety policies for specific robot platforms (UR5, Spot, Franka, drones)
- Additional sensor/camera integrations
- Documentation improvements and tutorials
- Bug reports from real hardware deployments

## License

MIT - see [LICENSE](LICENSE) for details.

---

<div align="center">

**PhysicalMCP** is the safety layer the robotics industry needs.

If this project is useful to you, please consider giving it a star.

[Report Bug](https://github.com/ricardothe3rd/physical-mcp/issues) | [Request Feature](https://github.com/ricardothe3rd/physical-mcp/issues) | [Sponsor](https://github.com/sponsors/ricardothe3rd)

</div>

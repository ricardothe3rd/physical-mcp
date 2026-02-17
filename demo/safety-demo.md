# PhysicalMCP Safety Demo

A hands-on walkthrough demonstrating the PhysicalMCP safety layer -- velocity limits, blocked topics, emergency stop, audit trail, and runtime policy updates.

## Prerequisites

- **Docker + Docker Compose** -- for running the Gazebo simulation and ROS2 bridge
- **Node.js >= 18** -- for the MCP server
- **Claude Desktop or Claude Code** -- to interact with the MCP tools

## Setup

### 1. Start the Simulation and Bridge

Clone the repo and bring up the full stack (TurtleBot3 in Gazebo + the ROS2 WebSocket bridge):

```bash
git clone https://github.com/ricardothe3rd/physical-mcp.git
cd physical-mcp/docker
docker compose up
```

Wait until you see both `sim` and `bridge` containers are running. The bridge listens on `ws://localhost:9090`.

### 2. Add the MCP Server

Register PhysicalMCP so Claude can use it:

```bash
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp
```

Restart Claude Desktop (or reconnect in Claude Code) so it picks up the new server.

---

## Demo Walkthrough

### Step 1: Discovery -- List All ROS2 Topics

Ask Claude:

> "List all available ROS2 topics."

Claude calls the `ros2_topic_list` tool. Expected output:

```json
{
  "topics": [
    { "name": "/cmd_vel", "type": "geometry_msgs/msg/Twist" },
    { "name": "/odom", "type": "nav_msgs/msg/Odometry" },
    { "name": "/scan", "type": "sensor_msgs/msg/LaserScan" },
    { "name": "/tf", "type": "tf2_msgs/msg/TFMessage" },
    { "name": "/rosout", "type": "rcl_interfaces/msg/Log" },
    { "name": "/parameter_events", "type": "rcl_interfaces/msg/ParameterEvent" },
    { "name": "/joint_states", "type": "sensor_msgs/msg/JointState" },
    { "name": "/imu", "type": "sensor_msgs/msg/Imu" }
  ]
}
```

This is a read-only operation -- no safety checks needed.

---

### Step 2: Read Sensor Data -- Echo Odometry

Ask Claude:

> "What is the robot's current position? Echo the /odom topic."

Claude calls `ros2_topic_echo` with topic `/odom` and message type `nav_msgs/msg/Odometry`. Expected output:

```json
{
  "header": {
    "stamp": { "sec": 142, "nanosec": 300000000 },
    "frame_id": "odom"
  },
  "pose": {
    "pose": {
      "position": { "x": 0.012, "y": -0.003, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.001, "w": 1.0 }
    }
  },
  "twist": {
    "twist": {
      "linear": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
    }
  }
}
```

Again read-only -- the robot is near the origin and stationary.

---

### Step 3: Safe Movement -- Publish Under the Limit

Ask Claude:

> "Move the robot forward at 0.1 m/s."

Claude calls `ros2_topic_publish` with:
- topic: `/cmd_vel`
- messageType: `geometry_msgs/msg/Twist`
- message: `{ "linear": { "x": 0.1, "y": 0.0, "z": 0.0 }, "angular": { "x": 0.0, "y": 0.0, "z": 0.0 } }`

The default velocity limit is **0.5 m/s** linear. Since 0.1 < 0.5, the safety check passes.

Expected output:

```
Published to /cmd_vel successfully
```

The robot starts moving forward in the simulation.

---

### Step 4: Safety Block -- Velocity Exceeded

Ask Claude:

> "Move the robot forward at 5.0 m/s."

Claude calls `ros2_topic_publish` with `linear.x = 5.0`. The policy engine checks the velocity and blocks it.

Expected output:

```
SAFETY BLOCKED: Publish to /cmd_vel denied.

Violations:
- [velocity_exceeded] Linear velocity 5.00 m/s exceeds limit of 0.5 m/s
```

The command never reaches the bridge. The robot does not move.

---

### Step 5: Safety Block -- Blocked Topic

Ask Claude:

> "Publish a test message to /rosout."

Claude calls `ros2_topic_publish` with topic `/rosout`. The default policy blocks `/rosout` and `/parameter_events`.

Expected output:

```
SAFETY BLOCKED: Publish to /rosout denied.

Violations:
- [blocked_topic] Topic "/rosout" is blocked by safety policy
```

System topics are protected from accidental writes.

---

### Step 6: Emergency Stop -- Halt Everything

Ask Claude:

> "Activate emergency stop -- testing safety features."

Claude calls `safety_emergency_stop` with reason `"testing safety features"`. Expected output:

```
EMERGENCY STOP ACTIVATED

Reason: testing safety features

All commands are now blocked. Zero velocity published to /cmd_vel.
Use safety_emergency_stop_release with confirmation "CONFIRM_RELEASE" to resume.
```

Now try to publish any command. Ask Claude:

> "Move the robot forward at 0.1 m/s."

Expected output:

```
SAFETY BLOCKED: Publish to /cmd_vel denied.

Violations:
- [emergency_stop_active] Emergency stop is active. Release e-stop before publishing.
```

Even a perfectly valid command is blocked while e-stop is active.

---

### Step 7: E-Stop Release -- Resume Operations

Ask Claude:

> "Release the emergency stop."

Claude calls `safety_emergency_stop_release` with confirmation `"CONFIRM_RELEASE"`. Expected output:

```
Emergency stop released. Commands are now allowed.
```

Now retry the movement:

> "Move the robot forward at 0.1 m/s."

```
Published to /cmd_vel successfully
```

The deliberate confirmation string `CONFIRM_RELEASE` prevents accidental e-stop release.

---

### Step 8: Audit Trail -- Review All Commands

Ask Claude:

> "Show me the safety audit log."

Claude calls `safety_audit_log` with default parameters (last 20 entries). Expected output:

```
Audit Log (8 total, 3 blocked, 0 errors)

[
  {
    "id": "a1b2c3d4-...",
    "timestamp": 1739808000000,
    "command": "publish",
    "target": "/cmd_vel",
    "params": { "linear": { "x": 0.1 }, "angular": {} },
    "safetyResult": { "allowed": true, "violations": [] }
  },
  {
    "id": "e5f6g7h8-...",
    "timestamp": 1739807995000,
    "command": "emergency_stop_release",
    "target": "system",
    "params": {},
    "safetyResult": { "allowed": true, "violations": [] }
  },
  {
    "id": "i9j0k1l2-...",
    "timestamp": 1739807990000,
    "command": "publish",
    "target": "/cmd_vel",
    "params": { "linear": { "x": 0.1 }, "angular": {} },
    "safetyResult": {
      "allowed": false,
      "violations": [
        { "type": "emergency_stop_active", "message": "Emergency stop is active. Release e-stop before publishing." }
      ]
    }
  },
  {
    "id": "m3n4o5p6-...",
    "timestamp": 1739807985000,
    "command": "emergency_stop",
    "target": "system",
    "params": {},
    "safetyResult": { "allowed": true, "violations": [] }
  },
  ...
]
```

To see only the blocked commands:

> "Show me only the blocked commands in the audit log."

Claude calls `safety_audit_log` with `violationsOnly: true`.

Every command -- allowed or blocked -- is recorded with full context.

---

### Step 9: Policy Customization -- Update Velocity Limits at Runtime

Ask Claude:

> "Reduce the maximum linear velocity to 0.2 m/s."

Claude calls `safety_update_velocity_limits` with `linearMax: 0.2`. Expected output:

```
Velocity limits updated:
{
  "linearMax": 0.2,
  "angularMax": 1.5
}
```

Now test the new limit:

> "Move the robot forward at 0.3 m/s."

```
SAFETY BLOCKED: Publish to /cmd_vel denied.

Violations:
- [velocity_exceeded] Linear velocity 0.30 m/s exceeds limit of 0.2 m/s
```

0.3 m/s was allowed under the old 0.5 m/s limit but is now blocked. Policy changes take effect immediately with zero downtime.

---

## Default Safety Policy Reference

| Parameter | Default Value |
|-----------|---------------|
| Linear velocity limit | 0.5 m/s |
| Angular velocity limit | 1.5 rad/s |
| Geofence | -5 to +5 m (x, y), 0 to 2 m (z) |
| Publish rate limit | 10 Hz |
| Service call rate limit | 60/min |
| Action goal rate limit | 30/min |
| Blocked topics | `/rosout`, `/parameter_events` |
| Blocked services | `/kill`, `/shutdown` |

## Key Takeaways

- **Every command goes through safety checks** before reaching the ROS2 bridge. The policy engine evaluates velocity, geofence, rate limits, and blocked topics/services on every publish, service call, and action goal.
- **Violations are blocked with clear, actionable messages** that explain exactly which rule was violated and what the current limits are.
- **Full audit trail** records every command with its safety result, creating a complete log of what the AI agent attempted and what was allowed.
- **Emergency stop requires explicit confirmation to release** -- the literal string `CONFIRM_RELEASE` prevents accidental reactivation. While active, all commands are blocked regardless of their content.
- **Policy is tunable at runtime** -- velocity limits, geofence boundaries, and other parameters can be adjusted without restarting the server.

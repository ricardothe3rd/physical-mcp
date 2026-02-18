# PhysicalMCP API Reference

Complete reference for all 21 MCP tools provided by the PhysicalMCP server. These tools enable AI agents to interact with ROS2 robots through a safety-first interface.

**Version:** 1.0
**Protocol:** Model Context Protocol (MCP)
**Transport:** stdio (MCP server) / WebSocket (ROS2 bridge on port 9090)

---

## Table of Contents

- [Overview](#overview)
- [Safety Model](#safety-model)
- [Topic Tools](#topic-tools)
  - [ros2_topic_list](#ros2_topic_list)
  - [ros2_topic_info](#ros2_topic_info)
  - [ros2_topic_subscribe](#ros2_topic_subscribe)
  - [ros2_topic_publish](#ros2_topic_publish)
  - [ros2_topic_echo](#ros2_topic_echo)
- [Service Tools](#service-tools)
  - [ros2_service_list](#ros2_service_list)
  - [ros2_service_info](#ros2_service_info)
  - [ros2_service_call](#ros2_service_call)
- [Action Tools](#action-tools)
  - [ros2_action_list](#ros2_action_list)
  - [ros2_action_send_goal](#ros2_action_send_goal)
  - [ros2_action_cancel](#ros2_action_cancel)
  - [ros2_action_status](#ros2_action_status)
- [Safety Tools](#safety-tools)
  - [safety_status](#safety_status)
  - [safety_emergency_stop](#safety_emergency_stop)
  - [safety_emergency_stop_release](#safety_emergency_stop_release)
  - [safety_get_policy](#safety_get_policy)
  - [safety_update_velocity_limits](#safety_update_velocity_limits)
  - [safety_update_geofence](#safety_update_geofence)
  - [safety_audit_log](#safety_audit_log)
- [System Tools](#system-tools)
  - [system_bridge_status](#system_bridge_status)
  - [system_node_list](#system_node_list)

---

## Overview

PhysicalMCP exposes 21 MCP tools organized into five categories:

| Category | Tools | Safety Checked | Purpose |
|----------|:-----:|:--------------:|---------|
| Topic | 5 | 1 of 5 | Read from and publish to ROS2 topics |
| Service | 3 | 1 of 3 | Discover and call ROS2 services |
| Action | 4 | 1 of 4 | Send goals, cancel, and monitor ROS2 actions |
| Safety | 7 | N/A | Control the safety layer itself |
| System | 2 | 0 of 2 | Bridge health and ROS2 graph introspection |

All tools that write to the ROS2 graph (publish, service call, action goal) pass through the safety policy engine before execution. Read-only tools bypass safety checks entirely.

---

## Safety Model

Three tools are safety-checked before their commands reach the ROS2 bridge:

- **`ros2_topic_publish`** -- Velocity limits, blocked topics, rate limiting, e-stop
- **`ros2_service_call`** -- Blocked services, rate limiting, e-stop
- **`ros2_action_send_goal`** -- Rate limiting, e-stop

When a safety check fails, the command is **blocked** (never sent to the robot), the violation is recorded in the audit log, and a structured error is returned to the caller describing each violation.

Safety tools themselves (`safety_status`, `safety_emergency_stop`, etc.) are always available, even when the bridge is disconnected.

---

## Topic Tools

### ros2_topic_list

List all available ROS2 topics with their message types.

**Description:** Queries the ROS2 graph through the bridge and returns every active topic along with its message type. Useful for discovering what data the robot is publishing or what command interfaces are available.

**Safety:** Not checked (read-only operation)

#### Parameters

*This tool takes no parameters.*

#### Returns

An array of objects, each containing:

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The fully qualified topic name (e.g., `"/cmd_vel"`) |
| `type` | `string` | The ROS2 message type (e.g., `"geometry_msgs/msg/Twist"`) |

#### Example

**Request:**
```json
{
  "tool": "ros2_topic_list"
}
```

**Response:**
```json
[
  { "name": "/cmd_vel", "type": "geometry_msgs/msg/Twist" },
  { "name": "/odom", "type": "nav_msgs/msg/Odometry" },
  { "name": "/scan", "type": "sensor_msgs/msg/LaserScan" },
  { "name": "/camera/image_raw", "type": "sensor_msgs/msg/Image" },
  { "name": "/tf", "type": "tf2_msgs/msg/TFMessage" }
]
```

---

### ros2_topic_info

Get detailed information about a specific ROS2 topic, including the number of publishers and subscribers.

**Description:** Returns metadata for a single topic. Useful for understanding whether a topic is active, how many nodes are publishing to it, and how many are listening.

**Safety:** Not checked (read-only operation)

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `topic` | `string` | Yes | -- | The topic name to query (e.g., `"/cmd_vel"`) |

#### Returns

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The topic name |
| `type` | `string` | The ROS2 message type |
| `publisherCount` | `number` | Number of nodes publishing to this topic |
| `subscriberCount` | `number` | Number of nodes subscribed to this topic |

#### Example

**Request:**
```json
{
  "tool": "ros2_topic_info",
  "arguments": {
    "topic": "/cmd_vel"
  }
}
```

**Response:**
```json
{
  "name": "/cmd_vel",
  "type": "geometry_msgs/msg/Twist",
  "publisherCount": 1,
  "subscriberCount": 2
}
```

---

### ros2_topic_subscribe

Subscribe to a topic and collect a specified number of messages.

**Description:** Opens a temporary subscription to the given topic and accumulates messages until the requested count is reached or the timeout expires. Useful for collecting a batch of sensor readings, monitoring a data stream, or capturing periodic updates.

**Safety:** Not checked (read-only operation)

#### Parameters

| Parameter | Type | Required | Default | Constraints | Description |
|-----------|------|:--------:|---------|-------------|-------------|
| `topic` | `string` | Yes | -- | -- | The topic to subscribe to (e.g., `"/odom"`) |
| `message_count` | `number` | No | `1` | Max: `100` | Number of messages to collect before returning |
| `timeout_ms` | `number` | No | `5000` | -- | Maximum time in milliseconds to wait for messages |

#### Returns

An array of collected messages. Each message is the deserialized ROS2 message object with the same field structure as the topic's message type. If fewer than `message_count` messages arrive before the timeout, the array contains only the messages that were received.

#### Example

**Request:**
```json
{
  "tool": "ros2_topic_subscribe",
  "arguments": {
    "topic": "/odom",
    "message_count": 3,
    "timeout_ms": 10000
  }
}
```

**Response:**
```json
[
  {
    "header": { "stamp": { "sec": 100, "nanosec": 500000000 }, "frame_id": "odom" },
    "pose": {
      "pose": {
        "position": { "x": 1.23, "y": 0.45, "z": 0.0 },
        "orientation": { "x": 0.0, "y": 0.0, "z": 0.12, "w": 0.99 }
      }
    }
  },
  {
    "header": { "stamp": { "sec": 100, "nanosec": 600000000 }, "frame_id": "odom" },
    "pose": {
      "pose": {
        "position": { "x": 1.24, "y": 0.45, "z": 0.0 },
        "orientation": { "x": 0.0, "y": 0.0, "z": 0.12, "w": 0.99 }
      }
    }
  },
  {
    "header": { "stamp": { "sec": 100, "nanosec": 700000000 }, "frame_id": "odom" },
    "pose": {
      "pose": {
        "position": { "x": 1.25, "y": 0.46, "z": 0.0 },
        "orientation": { "x": 0.0, "y": 0.0, "z": 0.12, "w": 0.99 }
      }
    }
  }
]
```

---

### ros2_topic_publish

Publish a message to a ROS2 topic.

**Description:** Sends a single message to the specified topic. This is the primary tool for commanding robot motion (e.g., publishing velocity commands to `/cmd_vel`). Every call passes through the full safety pipeline before the message reaches the bridge.

**Safety:** CHECKED -- The following safety checks are applied before the message is forwarded to the bridge:

| Check | Description |
|-------|-------------|
| **E-stop** | If emergency stop is active, the command is immediately blocked. |
| **Blocked topics** | If the topic is on the blocked list (e.g., `/rosout`, `/parameter_events`), the command is blocked. |
| **Velocity limits** | For `geometry_msgs/msg/Twist` messages, linear and angular velocities are compared against configured maximums. |
| **Rate limiting** | Publish rate is capped (default: 10 Hz). Exceeding the rate blocks the command. |

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `topic` | `string` | Yes | -- | The topic to publish to (e.g., `"/cmd_vel"`) |
| `message_type` | `string` | Yes | -- | The ROS2 message type (e.g., `"geometry_msgs/msg/Twist"`) |
| `message` | `object` | Yes | -- | The message fields matching the specified type |

#### Returns

On success, returns a confirmation that the message was published. On safety violation, returns a structured error containing:

| Field | Type | Description |
|-------|------|-------------|
| `allowed` | `boolean` | Always `false` when blocked |
| `violations` | `array` | List of violation objects, each with `type` and `message` fields |

#### Violation Types

| Violation Type | Trigger |
|----------------|---------|
| `emergency_stop_active` | E-stop is engaged |
| `blocked_topic` | Topic is on the blocked list |
| `velocity_exceeded` | Linear or angular velocity exceeds configured limit |
| `rate_limit_exceeded` | Publish rate exceeds configured Hz limit |

#### Example: Successful Publish

**Request:**
```json
{
  "tool": "ros2_topic_publish",
  "arguments": {
    "topic": "/cmd_vel",
    "message_type": "geometry_msgs/msg/Twist",
    "message": {
      "linear": { "x": 0.1, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
    }
  }
}
```

**Response:**
```
Published to /cmd_vel successfully
```

#### Example: Blocked by Velocity Limit

**Request:**
```json
{
  "tool": "ros2_topic_publish",
  "arguments": {
    "topic": "/cmd_vel",
    "message_type": "geometry_msgs/msg/Twist",
    "message": {
      "linear": { "x": 5.0, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
    }
  }
}
```

**Response:**
```
SAFETY BLOCKED: Publish to /cmd_vel denied.

Violations:
- [velocity_exceeded] Linear velocity 5.00 m/s exceeds limit of 0.5 m/s
```

#### Example: Blocked by E-Stop

**Response:**
```
SAFETY BLOCKED: Publish to /cmd_vel denied.

Violations:
- [emergency_stop_active] Emergency stop is active. Release e-stop before publishing.
```

---

### ros2_topic_echo

Get the latest message from a topic as a single snapshot.

**Description:** Reads the most recently published message on the given topic. Unlike `ros2_topic_subscribe`, this tool returns only one message and is intended for quick checks (e.g., "What is the robot's current pose?").

**Safety:** Not checked (read-only operation)

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `topic` | `string` | Yes | -- | The topic to read from (e.g., `"/odom"`) |
| `timeout_ms` | `number` | No | `5000` | Maximum time in milliseconds to wait for a message |

#### Returns

The latest message content as a deserialized ROS2 message object. If no message is received within the timeout, an error is returned.

#### Example

**Request:**
```json
{
  "tool": "ros2_topic_echo",
  "arguments": {
    "topic": "/scan",
    "timeout_ms": 3000
  }
}
```

**Response:**
```json
{
  "header": { "stamp": { "sec": 105, "nanosec": 200000000 }, "frame_id": "base_scan" },
  "angle_min": -3.14,
  "angle_max": 3.14,
  "angle_increment": 0.0175,
  "range_min": 0.12,
  "range_max": 3.5,
  "ranges": [0.5, 0.52, 0.55, "..."]
}
```

---

## Service Tools

### ros2_service_list

List all available ROS2 services.

**Description:** Queries the ROS2 graph and returns every advertised service along with its service type. Useful for discovering what ROS2 services are available on the system.

**Safety:** Not checked (read-only operation)

#### Parameters

*This tool takes no parameters.*

#### Returns

An array of objects, each containing:

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The fully qualified service name (e.g., `"/spawn_entity"`) |
| `type` | `string` | The ROS2 service type (e.g., `"gazebo_msgs/srv/SpawnEntity"`) |

#### Example

**Request:**
```json
{
  "tool": "ros2_service_list"
}
```

**Response:**
```json
[
  { "name": "/spawn_entity", "type": "gazebo_msgs/srv/SpawnEntity" },
  { "name": "/delete_entity", "type": "gazebo_msgs/srv/DeleteEntity" },
  { "name": "/reset_simulation", "type": "std_srvs/srv/Empty" },
  { "name": "/get_model_list", "type": "gazebo_msgs/srv/GetModelList" }
]
```

---

### ros2_service_info

Get type information for a specific ROS2 service.

**Description:** Returns the service name and its type. Useful for confirming the correct service type before calling it with `ros2_service_call`.

**Safety:** Not checked (read-only operation)

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `service` | `string` | Yes | -- | The service name to query (e.g., `"/spawn_entity"`) |

#### Returns

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The service name |
| `type` | `string` | The ROS2 service type |

#### Example

**Request:**
```json
{
  "tool": "ros2_service_info",
  "arguments": {
    "service": "/reset_simulation"
  }
}
```

**Response:**
```json
{
  "name": "/reset_simulation",
  "type": "std_srvs/srv/Empty"
}
```

---

### ros2_service_call

Call a ROS2 service with a request payload.

**Description:** Sends a request to a ROS2 service and returns the response. Services are synchronous request-response interactions (as opposed to topics which are publish-subscribe). Every call passes through safety checks before reaching the bridge.

**Safety:** CHECKED -- The following safety checks are applied:

| Check | Description |
|-------|-------------|
| **E-stop** | If emergency stop is active, the call is immediately blocked. |
| **Blocked services** | If the service is on the blocked list (e.g., `/kill`, `/shutdown`), the call is blocked. |
| **Rate limiting** | Service call rate is capped (default: 60 per minute). Exceeding the rate blocks the call. |

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `service` | `string` | Yes | -- | The service to call (e.g., `"/reset_simulation"`) |
| `service_type` | `string` | Yes | -- | The ROS2 service type (e.g., `"std_srvs/srv/Empty"`) |
| `request` | `object` | No | `{}` | The request fields matching the service's request type |

#### Returns

The service response object with fields matching the service's response type. On safety violation, returns a structured error with violation details (same format as `ros2_topic_publish`).

#### Violation Types

| Violation Type | Trigger |
|----------------|---------|
| `emergency_stop_active` | E-stop is engaged |
| `blocked_service` | Service is on the blocked list |
| `rate_limit_exceeded` | Service call rate exceeds configured per-minute limit |

#### Example: Empty Request Service

**Request:**
```json
{
  "tool": "ros2_service_call",
  "arguments": {
    "service": "/reset_simulation",
    "service_type": "std_srvs/srv/Empty",
    "request": {}
  }
}
```

**Response:**
```json
{}
```

#### Example: Service with Request Fields

**Request:**
```json
{
  "tool": "ros2_service_call",
  "arguments": {
    "service": "/spawn_entity",
    "service_type": "gazebo_msgs/srv/SpawnEntity",
    "request": {
      "name": "my_robot",
      "xml": "<robot>...</robot>",
      "initial_pose": {
        "position": { "x": 1.0, "y": 2.0, "z": 0.0 }
      }
    }
  }
}
```

**Response:**
```json
{
  "success": true,
  "status_message": "SpawnEntity: Successfully spawned entity [my_robot]"
}
```

#### Example: Blocked Service

**Request:**
```json
{
  "tool": "ros2_service_call",
  "arguments": {
    "service": "/shutdown",
    "service_type": "std_srvs/srv/Empty",
    "request": {}
  }
}
```

**Response:**
```
SAFETY BLOCKED: Service call to /shutdown denied.

Violations:
- [blocked_service] Service /shutdown is on the blocked list.
```

---

## Action Tools

### ros2_action_list

List all available ROS2 action servers.

**Description:** Queries the ROS2 graph and returns every advertised action server along with its action type. Actions are used for long-running tasks like navigation goals that provide feedback during execution.

**Safety:** Not checked (read-only operation)

#### Parameters

*This tool takes no parameters.*

#### Returns

An array of objects, each containing:

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The action server name (e.g., `"/navigate_to_pose"`) |
| `type` | `string` | The ROS2 action type (e.g., `"nav2_msgs/action/NavigateToPose"`) |

#### Example

**Request:**
```json
{
  "tool": "ros2_action_list"
}
```

**Response:**
```json
[
  { "name": "/navigate_to_pose", "type": "nav2_msgs/action/NavigateToPose" },
  { "name": "/spin", "type": "nav2_msgs/action/Spin" },
  { "name": "/backup", "type": "nav2_msgs/action/BackUp" }
]
```

---

### ros2_action_send_goal

Send a goal to a ROS2 action server.

**Description:** Submits a goal to an action server. Actions represent long-running tasks (such as navigating to a waypoint) that provide feedback during execution and can be cancelled. The tool returns the goal acceptance status. Every goal submission passes through safety checks.

**Safety:** CHECKED -- The following safety checks are applied:

| Check | Description |
|-------|-------------|
| **E-stop** | If emergency stop is active, the goal is immediately blocked. |
| **Rate limiting** | Action goal rate is capped (default: 30 per minute). Exceeding the rate blocks the goal. |

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `action` | `string` | Yes | -- | The action server name (e.g., `"/navigate_to_pose"`) |
| `action_type` | `string` | Yes | -- | The ROS2 action type (e.g., `"nav2_msgs/action/NavigateToPose"`) |
| `goal` | `object` | Yes | -- | The goal fields matching the action's goal type |

#### Returns

The goal acceptance result from the action server. On safety violation, returns a structured error with violation details.

#### Violation Types

| Violation Type | Trigger |
|----------------|---------|
| `emergency_stop_active` | E-stop is engaged |
| `rate_limit_exceeded` | Action goal rate exceeds configured per-minute limit |

#### Example

**Request:**
```json
{
  "tool": "ros2_action_send_goal",
  "arguments": {
    "action": "/navigate_to_pose",
    "action_type": "nav2_msgs/action/NavigateToPose",
    "goal": {
      "pose": {
        "header": { "frame_id": "map" },
        "pose": {
          "position": { "x": 2.0, "y": 1.0, "z": 0.0 },
          "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
        }
      }
    }
  }
}
```

**Response:**
```json
{
  "accepted": true,
  "goal_id": "abc123-def456"
}
```

---

### ros2_action_cancel

Cancel an active goal or all active goals on an action server.

**Description:** Sends a cancellation request to an action server. Can cancel a specific goal by ID or all active goals if no `goal_id` is provided. Cancellation is always considered a safe operation and is not subject to safety checks.

**Safety:** Not checked (cancellation is always safe)

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `action` | `string` | Yes | -- | The action server name (e.g., `"/navigate_to_pose"`) |
| `goal_id` | `string` | No | -- | Specific goal ID to cancel. If omitted, all active goals on this action server are cancelled. |

#### Returns

The cancellation result from the action server.

#### Example: Cancel a Specific Goal

**Request:**
```json
{
  "tool": "ros2_action_cancel",
  "arguments": {
    "action": "/navigate_to_pose",
    "goal_id": "abc123-def456"
  }
}
```

**Response:**
```json
{
  "goals_cancelled": 1
}
```

#### Example: Cancel All Goals

**Request:**
```json
{
  "tool": "ros2_action_cancel",
  "arguments": {
    "action": "/navigate_to_pose"
  }
}
```

**Response:**
```json
{
  "goals_cancelled": 3
}
```

---

### ros2_action_status

Get the status of active goals on an action server.

**Description:** Queries the action server for the status of all currently tracked goals. Useful for monitoring whether a navigation goal is still executing, has succeeded, or has been cancelled.

**Safety:** Not checked (read-only operation)

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `action` | `string` | Yes | -- | The action server name (e.g., `"/navigate_to_pose"`) |

#### Returns

An array of goal status objects, each containing:

| Field | Type | Description |
|-------|------|-------------|
| `goal_id` | `string` | The unique identifier for the goal |
| `status` | `string` | Current status (e.g., `"executing"`, `"succeeded"`, `"canceled"`, `"aborted"`) |

#### Example

**Request:**
```json
{
  "tool": "ros2_action_status",
  "arguments": {
    "action": "/navigate_to_pose"
  }
}
```

**Response:**
```json
[
  { "goal_id": "abc123-def456", "status": "executing" },
  { "goal_id": "ghi789-jkl012", "status": "succeeded" }
]
```

---

## Safety Tools

### safety_status

Get the current status of the entire safety system.

**Description:** Returns a comprehensive snapshot of the safety system, including whether emergency stop is active, the current policy configuration, and a summary of the audit log. This tool is always available, even when the bridge is disconnected.

**Safety:** Always available

#### Parameters

*This tool takes no parameters.*

#### Returns

| Field | Type | Description |
|-------|------|-------------|
| `emergencyStop` | `boolean` | `true` if e-stop is currently active |
| `policy` | `object` | Current safety policy summary |
| `policy.name` | `string` | Policy name (e.g., `"default"`, `"turtlebot3"`) |
| `policy.velocity` | `object` | Velocity limits (`linearMax`, `angularMax`) |
| `policy.geofence` | `object` | Geofence bounds (`xMin`, `xMax`, `yMin`, `yMax`, `zMin`, `zMax`) |
| `policy.rateLimits` | `object` | Rate limits (`publishHz`, `servicePerMinute`, `actionPerMinute`) |
| `auditSummary` | `object` | Audit log statistics |
| `auditSummary.total` | `number` | Total number of commands logged |
| `auditSummary.blocked` | `number` | Number of commands blocked by safety |
| `auditSummary.errors` | `number` | Number of commands that resulted in errors |

#### Example

**Request:**
```json
{
  "tool": "safety_status"
}
```

**Response:**
```json
{
  "emergencyStop": false,
  "policy": {
    "name": "default",
    "velocity": { "linearMax": 0.5, "angularMax": 1.5 },
    "geofence": {
      "xMin": -5.0, "xMax": 5.0,
      "yMin": -5.0, "yMax": 5.0,
      "zMin": 0.0, "zMax": 2.0
    },
    "rateLimits": {
      "publishHz": 10,
      "servicePerMinute": 60,
      "actionPerMinute": 30
    }
  },
  "auditSummary": {
    "total": 42,
    "blocked": 3,
    "errors": 0
  }
}
```

---

### safety_emergency_stop

Activate the emergency stop, immediately blocking ALL commands.

**Description:** Engages the emergency stop. Once active, every publish, service call, and action goal is blocked until the e-stop is explicitly released. The bridge also publishes a zero-velocity `Twist` message to `/cmd_vel` and cancels all active action goals as a secondary fail-safe.

**Safety:** Always available (this IS a safety tool)

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `reason` | `string` | No | -- | Human-readable reason for activating the e-stop (logged in audit trail) |

#### Returns

A confirmation message indicating the emergency stop has been activated.

#### Side Effects

- All subsequent publish, service call, and action goal commands are blocked.
- The bridge publishes `{ linear: {x:0, y:0, z:0}, angular: {x:0, y:0, z:0} }` to `/cmd_vel`.
- All active action goals are cancelled.

#### Example

**Request:**
```json
{
  "tool": "safety_emergency_stop",
  "arguments": {
    "reason": "Robot approaching obstacle too fast"
  }
}
```

**Response:**
```
EMERGENCY STOP ACTIVATED

Reason: Robot approaching obstacle too fast

All commands are now blocked. Zero velocity published to /cmd_vel.
Use safety_emergency_stop_release with confirmation "CONFIRM_RELEASE" to resume.
```

---

### safety_emergency_stop_release

Release the emergency stop to resume normal operations.

**Description:** Disengages the emergency stop. Requires the caller to provide the exact confirmation string `"CONFIRM_RELEASE"` to prevent accidental release. After release, commands are processed normally through the safety pipeline again.

**Safety:** Requires explicit confirmation

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `confirmation` | `string` | Yes | -- | Must be the exact string `"CONFIRM_RELEASE"` |

#### Returns

A success message if the confirmation matches. An error message if the confirmation string is incorrect.

#### Example: Successful Release

**Request:**
```json
{
  "tool": "safety_emergency_stop_release",
  "arguments": {
    "confirmation": "CONFIRM_RELEASE"
  }
}
```

**Response:**
```
Emergency stop released. Normal operations resumed.
```

#### Example: Failed Release (Wrong Confirmation)

**Request:**
```json
{
  "tool": "safety_emergency_stop_release",
  "arguments": {
    "confirmation": "yes"
  }
}
```

**Response:**
```
ERROR: Invalid confirmation. You must provide the exact string "CONFIRM_RELEASE" to release the emergency stop.
```

---

### safety_get_policy

Get the full current safety policy configuration.

**Description:** Returns the complete safety policy that is currently in effect, including velocity limits, geofence boundaries, rate limits, and blocked topic/service lists. Useful for understanding what constraints are active before sending commands.

**Safety:** Always available

#### Parameters

*This tool takes no parameters.*

#### Returns

The full policy object:

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | Policy name |
| `description` | `string` | Human-readable description |
| `velocity` | `object` | `{ linearMax: number, angularMax: number }` |
| `geofence` | `object` | `{ xMin, xMax, yMin, yMax, zMin, zMax }` (all `number`) |
| `rateLimits` | `object` | `{ publishHz: number, servicePerMinute: number, actionPerMinute: number }` |
| `blockedTopics` | `string[]` | List of topic names that cannot be published to |
| `blockedServices` | `string[]` | List of service names that cannot be called |

#### Example

**Request:**
```json
{
  "tool": "safety_get_policy"
}
```

**Response:**
```json
{
  "name": "turtlebot3",
  "description": "Tuned for TurtleBot3 Burger in Gazebo",
  "velocity": {
    "linearMax": 0.22,
    "angularMax": 2.84
  },
  "geofence": {
    "xMin": -5.0, "xMax": 5.0,
    "yMin": -5.0, "yMax": 5.0,
    "zMin": 0.0, "zMax": 2.0
  },
  "rateLimits": {
    "publishHz": 10,
    "servicePerMinute": 60,
    "actionPerMinute": 30
  },
  "blockedTopics": ["/rosout", "/parameter_events"],
  "blockedServices": ["/kill", "/shutdown"]
}
```

---

### safety_update_velocity_limits

Update the velocity limits at runtime without restarting the server.

**Description:** Modifies the maximum allowed linear and/or angular velocity. The update takes effect immediately for all subsequent commands. Both parameters are optional -- you can update one without affecting the other.

**Safety:** Always available

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `linearMax` | `number` | No | -- | New maximum linear velocity in m/s |
| `angularMax` | `number` | No | -- | New maximum angular velocity in rad/s |

#### Returns

The updated velocity limits object showing the current values after the update.

#### Example

**Request:**
```json
{
  "tool": "safety_update_velocity_limits",
  "arguments": {
    "linearMax": 0.3,
    "angularMax": 1.0
  }
}
```

**Response:**
```json
{
  "linearMax": 0.3,
  "angularMax": 1.0
}
```

#### Example: Update Only One Limit

**Request:**
```json
{
  "tool": "safety_update_velocity_limits",
  "arguments": {
    "linearMax": 0.1
  }
}
```

**Response:**
```json
{
  "linearMax": 0.1,
  "angularMax": 1.0
}
```

---

### safety_update_geofence

Update the geofence boundaries at runtime without restarting the server.

**Description:** Modifies the workspace boundaries that define where the robot is allowed to operate. The update takes effect immediately. All six boundary parameters are optional -- you can update any subset without affecting the others.

**Safety:** Always available

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|:--------:|---------|-------------|
| `xMin` | `number` | No | -- | Minimum X boundary (meters) |
| `xMax` | `number` | No | -- | Maximum X boundary (meters) |
| `yMin` | `number` | No | -- | Minimum Y boundary (meters) |
| `yMax` | `number` | No | -- | Maximum Y boundary (meters) |
| `zMin` | `number` | No | -- | Minimum Z boundary (meters) |
| `zMax` | `number` | No | -- | Maximum Z boundary (meters) |

#### Returns

The updated geofence bounds object showing the current values after the update.

#### Example

**Request:**
```json
{
  "tool": "safety_update_geofence",
  "arguments": {
    "xMin": -2.0,
    "xMax": 2.0,
    "yMin": -2.0,
    "yMax": 2.0
  }
}
```

**Response:**
```json
{
  "xMin": -2.0,
  "xMax": 2.0,
  "yMin": -2.0,
  "yMax": 2.0,
  "zMin": 0.0,
  "zMax": 2.0
}
```

---

### safety_audit_log

Query the command audit trail.

**Description:** Retrieves logged entries from the audit trail. Every command processed by the safety system (whether allowed or blocked) is recorded. This tool supports filtering to show only violations and limiting the number of results returned.

**Safety:** Always available

#### Parameters

| Parameter | Type | Required | Default | Constraints | Description |
|-----------|------|:--------:|---------|-------------|-------------|
| `limit` | `number` | No | `50` | Max: `1000` | Maximum number of entries to return |
| `violations_only` | `boolean` | No | `false` | -- | When `true`, only returns entries where the command was blocked |

#### Returns

An array of audit log entries, each containing:

| Field | Type | Description |
|-------|------|-------------|
| `id` | `string` | Unique identifier for the audit entry |
| `timestamp` | `string` | ISO 8601 timestamp of when the command was processed |
| `command` | `string` | The command type (e.g., `"publish"`, `"service_call"`, `"action_goal"`, `"emergency_stop"`) |
| `target` | `string` | The target of the command (topic name, service name, or `"system"`) |
| `safetyResult` | `object` | The safety check outcome |
| `safetyResult.allowed` | `boolean` | Whether the command was allowed |
| `safetyResult.violations` | `array` | List of violations (empty if allowed) |

#### Example: Recent Entries

**Request:**
```json
{
  "tool": "safety_audit_log",
  "arguments": {
    "limit": 5
  }
}
```

**Response:**
```json
[
  {
    "id": "audit-001",
    "timestamp": "2026-02-18T14:30:00.000Z",
    "command": "publish",
    "target": "/cmd_vel",
    "safetyResult": { "allowed": true, "violations": [] }
  },
  {
    "id": "audit-002",
    "timestamp": "2026-02-18T14:30:05.000Z",
    "command": "publish",
    "target": "/cmd_vel",
    "safetyResult": {
      "allowed": false,
      "violations": [
        { "type": "velocity_exceeded", "message": "Linear velocity 2.00 m/s exceeds limit of 0.5 m/s" }
      ]
    }
  },
  {
    "id": "audit-003",
    "timestamp": "2026-02-18T14:30:10.000Z",
    "command": "emergency_stop",
    "target": "system",
    "safetyResult": { "allowed": true, "violations": [] }
  },
  {
    "id": "audit-004",
    "timestamp": "2026-02-18T14:30:15.000Z",
    "command": "publish",
    "target": "/cmd_vel",
    "safetyResult": {
      "allowed": false,
      "violations": [
        { "type": "emergency_stop_active", "message": "Emergency stop is active. Release e-stop before publishing." }
      ]
    }
  },
  {
    "id": "audit-005",
    "timestamp": "2026-02-18T14:30:20.000Z",
    "command": "emergency_stop_release",
    "target": "system",
    "safetyResult": { "allowed": true, "violations": [] }
  }
]
```

#### Example: Violations Only

**Request:**
```json
{
  "tool": "safety_audit_log",
  "arguments": {
    "violations_only": true,
    "limit": 10
  }
}
```

**Response:**
```json
[
  {
    "id": "audit-002",
    "timestamp": "2026-02-18T14:30:05.000Z",
    "command": "publish",
    "target": "/cmd_vel",
    "safetyResult": {
      "allowed": false,
      "violations": [
        { "type": "velocity_exceeded", "message": "Linear velocity 2.00 m/s exceeds limit of 0.5 m/s" }
      ]
    }
  },
  {
    "id": "audit-004",
    "timestamp": "2026-02-18T14:30:15.000Z",
    "command": "publish",
    "target": "/cmd_vel",
    "safetyResult": {
      "allowed": false,
      "violations": [
        { "type": "emergency_stop_active", "message": "Emergency stop is active. Release e-stop before publishing." }
      ]
    }
  }
]
```

---

## System Tools

### system_bridge_status

Check the health and latency of the ROS2 bridge connection.

**Description:** Reports whether the MCP server is connected to the Python ROS2 bridge over WebSocket. When connected, includes the round-trip latency from a ping measurement. Useful for diagnosing connectivity issues before sending commands.

**Safety:** Not checked (read-only operation)

#### Parameters

*This tool takes no parameters.*

#### Returns

| Field | Type | Description |
|-------|------|-------------|
| `connected` | `boolean` | `true` if the WebSocket connection to the bridge is active |
| `url` | `string` | The WebSocket URL being used (e.g., `"ws://localhost:9090"`) |
| `latencyMs` | `number` | Round-trip latency in milliseconds (only present when `connected` is `true`) |

#### Example: Connected

**Request:**
```json
{
  "tool": "system_bridge_status"
}
```

**Response:**
```json
{
  "connected": true,
  "url": "ws://localhost:9090",
  "latencyMs": 2.5
}
```

#### Example: Disconnected

**Response:**
```json
{
  "connected": false,
  "url": "ws://localhost:9090"
}
```

---

### system_node_list

List all active ROS2 nodes.

**Description:** Queries the ROS2 graph and returns the names of all currently active nodes. Useful for understanding what software is running on the robot system.

**Safety:** Not checked (read-only operation)

#### Parameters

*This tool takes no parameters.*

#### Returns

An array of node name strings.

#### Example

**Request:**
```json
{
  "tool": "system_node_list"
}
```

**Response:**
```json
[
  "/turtlebot3_node",
  "/gazebo",
  "/robot_state_publisher",
  "/physical_mcp_bridge",
  "/rviz2"
]
```

---

## Quick Reference

### All Tools at a Glance

| # | Tool | Category | Safety Checked | Parameters |
|:-:|------|----------|:--------------:|------------|
| 1 | `ros2_topic_list` | Topic | No | *(none)* |
| 2 | `ros2_topic_info` | Topic | No | `topic` |
| 3 | `ros2_topic_subscribe` | Topic | No | `topic`, `message_count?`, `timeout_ms?` |
| 4 | `ros2_topic_publish` | Topic | **Yes** | `topic`, `message_type`, `message` |
| 5 | `ros2_topic_echo` | Topic | No | `topic`, `timeout_ms?` |
| 6 | `ros2_service_list` | Service | No | *(none)* |
| 7 | `ros2_service_info` | Service | No | `service` |
| 8 | `ros2_service_call` | Service | **Yes** | `service`, `service_type`, `request?` |
| 9 | `ros2_action_list` | Action | No | *(none)* |
| 10 | `ros2_action_send_goal` | Action | **Yes** | `action`, `action_type`, `goal` |
| 11 | `ros2_action_cancel` | Action | No | `action`, `goal_id?` |
| 12 | `ros2_action_status` | Action | No | `action` |
| 13 | `safety_status` | Safety | N/A | *(none)* |
| 14 | `safety_emergency_stop` | Safety | N/A | `reason?` |
| 15 | `safety_emergency_stop_release` | Safety | N/A | `confirmation` |
| 16 | `safety_get_policy` | Safety | N/A | *(none)* |
| 17 | `safety_update_velocity_limits` | Safety | N/A | `linearMax?`, `angularMax?` |
| 18 | `safety_update_geofence` | Safety | N/A | `xMin?`, `xMax?`, `yMin?`, `yMax?`, `zMin?`, `zMax?` |
| 19 | `safety_audit_log` | Safety | N/A | `limit?`, `violations_only?` |
| 20 | `system_bridge_status` | System | No | *(none)* |
| 21 | `system_node_list` | System | No | *(none)* |

### Safety Check Summary

| Check Type | Applied To | Default Limit |
|------------|-----------|---------------|
| Emergency stop | `ros2_topic_publish`, `ros2_service_call`, `ros2_action_send_goal` | N/A (binary on/off) |
| Velocity limits | `ros2_topic_publish` (Twist messages) | 0.5 m/s linear, 1.5 rad/s angular |
| Blocked topics | `ros2_topic_publish` | `/rosout`, `/parameter_events` |
| Blocked services | `ros2_service_call` | `/kill`, `/shutdown` |
| Publish rate limit | `ros2_topic_publish` | 10 Hz |
| Service rate limit | `ros2_service_call` | 60 per minute |
| Action rate limit | `ros2_action_send_goal` | 30 per minute |

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `PHYSICAL_MCP_BRIDGE_URL` | `ws://localhost:9090` | WebSocket URL for the ROS2 bridge |
| `PHYSICAL_MCP_POLICY` | *(built-in defaults)* | Path to a YAML safety policy file |

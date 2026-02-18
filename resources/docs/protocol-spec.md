# PhysicalMCP WebSocket Protocol Specification

**Version:** 1.0.0
**Date:** 2026-02-18
**Status:** Normative
**Authors:** PhysicalMCP Contributors

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Transport Layer](#2-transport-layer)
3. [Message Framing](#3-message-framing)
4. [Command Format](#4-command-format)
5. [Response Format](#5-response-format)
6. [Command Reference](#6-command-reference)
   - 6.1 [System Commands](#61-system-commands)
   - 6.2 [Topic Commands](#62-topic-commands)
   - 6.3 [Service Commands](#63-service-commands)
   - 6.4 [Action Commands](#64-action-commands)
   - 6.5 [Node Commands](#65-node-commands)
   - 6.6 [Emergency Commands](#66-emergency-commands)
7. [Connection Lifecycle](#7-connection-lifecycle)
8. [Error Handling](#8-error-handling)
9. [Validation](#9-validation)
10. [Security Considerations](#10-security-considerations)
11. [Appendix A: Full Message Schema](#appendix-a-full-message-schema)
12. [Appendix B: Status Codes Summary](#appendix-b-status-codes-summary)

---

## 1. Introduction

This document specifies the WebSocket protocol used for communication between the PhysicalMCP TypeScript MCP server (the **client**) and the PhysicalMCP Python ROS2 bridge (the **server**). The protocol defines a request-response messaging pattern over a persistent WebSocket connection, enabling AI agents to interact with ROS2 robotic systems through a safety-enforced intermediary layer.

### 1.1 Terminology

| Term | Definition |
|------|------------|
| **Client** | The TypeScript MCP server that initiates WebSocket connections and sends commands. |
| **Server** | The Python ROS2 bridge that receives commands, executes them against the ROS2 graph, and returns responses. |
| **Command** | A JSON message sent from the client to the server requesting an operation. |
| **Response** | A JSON message sent from the server to the client containing the result of a command. |
| **Bridge** | Synonym for the server; the component that bridges WebSocket messages to ROS2. |

### 1.2 Conventions

The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHOULD", "SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this document are to be interpreted as described in [RFC 2119](https://www.rfc-editor.org/rfc/rfc2119).

---

## 2. Transport Layer

### 2.1 Protocol

Communication uses the WebSocket protocol as defined in [RFC 6455](https://www.rfc-editor.org/rfc/rfc6455).

### 2.2 Default Endpoint

```
ws://<host>:9090
```

- **Default host:** `localhost` (configurable via `PHYSICAL_MCP_BRIDGE_URL` environment variable)
- **Default port:** `9090`
- **Path:** `/` (root)

### 2.3 Sub-protocol

No WebSocket sub-protocol is negotiated. The `Sec-WebSocket-Protocol` header SHOULD NOT be sent.

### 2.4 Encoding

All messages are UTF-8 encoded text frames. Binary frames MUST NOT be used for protocol messages.

---

## 3. Message Framing

Each WebSocket text frame contains exactly one JSON message. Messages MUST NOT be split across multiple frames. Each frame MUST contain a complete, valid JSON object.

- The client MUST send one command per text frame.
- The server MUST send one response per text frame.
- Messages MUST be serialized as compact JSON (no unnecessary whitespace is required, though whitespace is permitted).

---

## 4. Command Format

Commands are sent from the client to the server. Every command MUST conform to the following structure:

```json
{
  "id": "<uuid-string>",
  "type": "<command_type>",
  "params": { ... }
}
```

### 4.1 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | `string` | Yes | A unique identifier for the request. MUST be a UUID v4 string. Used to correlate responses to their originating commands. |
| `type` | `string` | Yes | The command type identifier. MUST be one of the 16 defined command types (see [Section 6](#6-command-reference)). |
| `params` | `object` | No | An object containing command-specific parameters. Defaults to `{}` if omitted. |

### 4.2 Command ID Generation

The client MUST generate a unique UUID v4 for each command. The server uses this ID to tag the corresponding response. The client MUST NOT reuse IDs within the same connection session.

---

## 5. Response Format

Responses are sent from the server to the client. Every response MUST conform to the following structure:

```json
{
  "id": "<uuid-string-matching-request>",
  "status": "ok" | "error",
  "data": { ... },
  "timestamp": 1234567890.123
}
```

### 5.1 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | `string \| null` | Yes | The UUID from the originating command. Set to `null` if the command could not be parsed (and thus the ID could not be extracted). |
| `status` | `string` | Yes | Either `"ok"` for successful execution or `"error"` for failures. |
| `data` | `any` | Yes | The response payload. Structure depends on the command type. For errors, contains an object with an `error` field. |
| `timestamp` | `number` | Yes | Unix epoch timestamp (seconds with fractional milliseconds) indicating when the response was generated on the server. |

### 5.2 Response Correlation

The client matches responses to pending requests using the `id` field. If a response arrives with an `id` that does not match any pending request, the client SHOULD discard it and MAY log a warning.

---

## 6. Command Reference

This section defines all 16 command types, their parameters, response data, and usage examples.

---

### 6.1 System Commands

#### 6.1.1 `ping`

Health check to verify the bridge is reachable and operational.

**Parameters:** None.

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `bridge` | `string` | Always `"ok"` when the bridge is healthy. |

**Example:**

```
Client:
{
  "id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "type": "ping",
  "params": {}
}

Server:
{
  "id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "status": "ok",
  "data": {
    "bridge": "ok"
  },
  "timestamp": 1739913600.456
}
```

---

### 6.2 Topic Commands

#### 6.2.1 `topic_list`

List all topics currently available on the ROS2 graph.

**Parameters:** None.

**Response Data:** An array of topic descriptor objects.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The fully qualified topic name (e.g., `/cmd_vel`). |
| `type` | `string` | The ROS2 message type (e.g., `geometry_msgs/msg/Twist`). |

**Example:**

```
Client:
{
  "id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
  "type": "topic_list",
  "params": {}
}

Server:
{
  "id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
  "status": "ok",
  "data": [
    { "name": "/cmd_vel", "type": "geometry_msgs/msg/Twist" },
    { "name": "/odom", "type": "nav_msgs/msg/Odometry" },
    { "name": "/scan", "type": "sensor_msgs/msg/LaserScan" },
    { "name": "/tf", "type": "tf2_msgs/msg/TFMessage" }
  ],
  "timestamp": 1739913601.123
}
```

---

#### 6.2.2 `topic_info`

Get detailed information about a specific topic, including publisher and subscriber counts.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `topic` | `string` | Yes | The fully qualified topic name. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The topic name. |
| `type` | `string` | The ROS2 message type. |
| `publisher_count` | `number` | Number of active publishers on this topic. |
| `subscriber_count` | `number` | Number of active subscribers on this topic. |

**Example:**

```
Client:
{
  "id": "c3d4e5f6-a7b8-9012-cdef-123456789012",
  "type": "topic_info",
  "params": {
    "topic": "/cmd_vel"
  }
}

Server:
{
  "id": "c3d4e5f6-a7b8-9012-cdef-123456789012",
  "status": "ok",
  "data": {
    "name": "/cmd_vel",
    "type": "geometry_msgs/msg/Twist",
    "publisher_count": 1,
    "subscriber_count": 2
  },
  "timestamp": 1739913602.789
}
```

---

#### 6.2.3 `topic_subscribe`

Subscribe to a topic and collect a specified number of messages. The server creates a temporary subscription, waits for messages (up to the given count or timeout), and returns the collected messages in a single response.

**Parameters:**

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `topic` | `string` | Yes | -- | The fully qualified topic name. |
| `count` | `number` | No | `1` | Number of messages to collect before returning. |
| `timeout_ms` | `number` | No | `5000` | Maximum time in milliseconds to wait for messages. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `messages` | `array` | Array of collected messages. Each element is a deserialized ROS2 message object. May contain fewer than `count` messages if the timeout is reached. |

**Example:**

```
Client:
{
  "id": "d4e5f6a7-b8c9-0123-defa-234567890123",
  "type": "topic_subscribe",
  "params": {
    "topic": "/odom",
    "count": 3,
    "timeout_ms": 5000
  }
}

Server:
{
  "id": "d4e5f6a7-b8c9-0123-defa-234567890123",
  "status": "ok",
  "data": {
    "messages": [
      {
        "header": { "stamp": { "sec": 100, "nanosec": 500000000 }, "frame_id": "odom" },
        "pose": {
          "pose": {
            "position": { "x": 1.05, "y": 0.23, "z": 0.0 },
            "orientation": { "x": 0.0, "y": 0.0, "z": 0.12, "w": 0.99 }
          }
        }
      },
      {
        "header": { "stamp": { "sec": 100, "nanosec": 600000000 }, "frame_id": "odom" },
        "pose": {
          "pose": {
            "position": { "x": 1.06, "y": 0.24, "z": 0.0 },
            "orientation": { "x": 0.0, "y": 0.0, "z": 0.12, "w": 0.99 }
          }
        }
      },
      {
        "header": { "stamp": { "sec": 100, "nanosec": 700000000 }, "frame_id": "odom" },
        "pose": {
          "pose": {
            "position": { "x": 1.07, "y": 0.25, "z": 0.0 },
            "orientation": { "x": 0.0, "y": 0.0, "z": 0.12, "w": 0.99 }
          }
        }
      }
    ]
  },
  "timestamp": 1739913605.234
}
```

**Timeout Behavior:** If fewer than `count` messages arrive before `timeout_ms` elapses, the server MUST return whatever messages were collected (which may be an empty array).

---

#### 6.2.4 `topic_publish`

Publish a single message to a ROS2 topic.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `topic` | `string` | Yes | The fully qualified topic name. |
| `message_type` | `string` | Yes | The ROS2 message type (e.g., `geometry_msgs/msg/Twist`). |
| `message` | `object` | Yes | The message payload matching the structure of the specified message type. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `published` | `boolean` | `true` if the message was published successfully. |

**Example:**

```
Client:
{
  "id": "e5f6a7b8-c9d0-1234-efab-345678901234",
  "type": "topic_publish",
  "params": {
    "topic": "/cmd_vel",
    "message_type": "geometry_msgs/msg/Twist",
    "message": {
      "linear": { "x": 0.2, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
    }
  }
}

Server:
{
  "id": "e5f6a7b8-c9d0-1234-efab-345678901234",
  "status": "ok",
  "data": {
    "published": true
  },
  "timestamp": 1739913606.891
}
```

**Safety Note:** On the client side (MCP server), publish commands are subject to safety policy evaluation (velocity limits, geofence, rate limits, blocked topics, emergency stop) BEFORE being sent over the WebSocket. Additionally, the bridge enforces a secondary emergency stop check.

---

#### 6.2.5 `topic_echo`

Retrieve the latest (most recent) message from a topic. Functionally equivalent to subscribing for a single message with a short timeout.

**Parameters:**

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `topic` | `string` | Yes | -- | The fully qualified topic name. |
| `timeout_ms` | `number` | No | `3000` | Maximum time in milliseconds to wait for a message. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `message` | `object \| null` | The latest message from the topic, or `null` if no message was received within the timeout. |

**Example:**

```
Client:
{
  "id": "f6a7b8c9-d0e1-2345-fabc-456789012345",
  "type": "topic_echo",
  "params": {
    "topic": "/scan",
    "timeout_ms": 2000
  }
}

Server:
{
  "id": "f6a7b8c9-d0e1-2345-fabc-456789012345",
  "status": "ok",
  "data": {
    "message": {
      "header": { "stamp": { "sec": 200, "nanosec": 100000000 }, "frame_id": "base_scan" },
      "angle_min": -3.14159,
      "angle_max": 3.14159,
      "range_min": 0.12,
      "range_max": 3.5,
      "ranges": [1.2, 1.3, 1.5, 0.8, 3.5, 3.5]
    }
  },
  "timestamp": 1739913607.456
}
```

**No Message Available:**

```
Server:
{
  "id": "f6a7b8c9-d0e1-2345-fabc-456789012345",
  "status": "ok",
  "data": {
    "message": null
  },
  "timestamp": 1739913609.456
}
```

---

### 6.3 Service Commands

#### 6.3.1 `service_list`

List all ROS2 services currently available on the graph.

**Parameters:** None.

**Response Data:** An array of service descriptor objects.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The fully qualified service name (e.g., `/spawn_entity`). |
| `type` | `string` | The ROS2 service type (e.g., `gazebo_msgs/srv/SpawnEntity`). |

**Example:**

```
Client:
{
  "id": "a7b8c9d0-e1f2-3456-abcd-567890123456",
  "type": "service_list",
  "params": {}
}

Server:
{
  "id": "a7b8c9d0-e1f2-3456-abcd-567890123456",
  "status": "ok",
  "data": [
    { "name": "/reset_simulation", "type": "std_srvs/srv/Empty" },
    { "name": "/spawn_entity", "type": "gazebo_msgs/srv/SpawnEntity" },
    { "name": "/get_model_list", "type": "gazebo_msgs/srv/GetModelList" }
  ],
  "timestamp": 1739913608.321
}
```

---

#### 6.3.2 `service_info`

Get type information for a specific service.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `service` | `string` | Yes | The fully qualified service name. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The service name. |
| `type` | `string` | The ROS2 service type. |

**Example:**

```
Client:
{
  "id": "b8c9d0e1-f2a3-4567-bcde-678901234567",
  "type": "service_info",
  "params": {
    "service": "/reset_simulation"
  }
}

Server:
{
  "id": "b8c9d0e1-f2a3-4567-bcde-678901234567",
  "status": "ok",
  "data": {
    "name": "/reset_simulation",
    "type": "std_srvs/srv/Empty"
  },
  "timestamp": 1739913609.654
}
```

---

#### 6.3.3 `service_call`

Invoke a ROS2 service and return the result.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `service` | `string` | Yes | The fully qualified service name. |
| `service_type` | `string` | Yes | The ROS2 service type (e.g., `std_srvs/srv/Empty`). |
| `request` | `object` | No | The service request payload. Defaults to `{}` for services with no request fields. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `result` | `object` | The deserialized service response. |

**Example (parameterless service):**

```
Client:
{
  "id": "c9d0e1f2-a3b4-5678-cdef-789012345678",
  "type": "service_call",
  "params": {
    "service": "/reset_simulation",
    "service_type": "std_srvs/srv/Empty"
  }
}

Server:
{
  "id": "c9d0e1f2-a3b4-5678-cdef-789012345678",
  "status": "ok",
  "data": {
    "result": {}
  },
  "timestamp": 1739913610.987
}
```

**Example (service with request parameters):**

```
Client:
{
  "id": "d0e1f2a3-b4c5-6789-defa-890123456789",
  "type": "service_call",
  "params": {
    "service": "/get_model_list",
    "service_type": "gazebo_msgs/srv/GetModelList",
    "request": {}
  }
}

Server:
{
  "id": "d0e1f2a3-b4c5-6789-defa-890123456789",
  "status": "ok",
  "data": {
    "result": {
      "model_names": ["ground_plane", "turtlebot3_burger"],
      "success": true
    }
  },
  "timestamp": 1739913611.456
}
```

---

### 6.4 Action Commands

#### 6.4.1 `action_list`

List all action servers currently available on the ROS2 graph.

**Parameters:** None.

**Response Data:** An array of action descriptor objects.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | The fully qualified action server name (e.g., `/navigate_to_pose`). |
| `type` | `string` | The ROS2 action type (e.g., `nav2_msgs/action/NavigateToPose`). |

**Example:**

```
Client:
{
  "id": "e1f2a3b4-c5d6-7890-efab-901234567890",
  "type": "action_list",
  "params": {}
}

Server:
{
  "id": "e1f2a3b4-c5d6-7890-efab-901234567890",
  "status": "ok",
  "data": [
    { "name": "/navigate_to_pose", "type": "nav2_msgs/action/NavigateToPose" },
    { "name": "/follow_path", "type": "nav2_msgs/action/FollowPath" }
  ],
  "timestamp": 1739913612.321
}
```

---

#### 6.4.2 `action_send_goal`

Send a goal to a ROS2 action server.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `action` | `string` | Yes | The fully qualified action server name. |
| `action_type` | `string` | Yes | The ROS2 action type (e.g., `nav2_msgs/action/NavigateToPose`). |
| `goal` | `object` | Yes | The goal payload matching the action's goal structure. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `accepted` | `boolean` | Whether the action server accepted the goal. |
| `goal_id` | `string` | The unique identifier assigned to this goal by the action server. |

**Example:**

```
Client:
{
  "id": "f2a3b4c5-d6e7-8901-fabc-012345678901",
  "type": "action_send_goal",
  "params": {
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

Server:
{
  "id": "f2a3b4c5-d6e7-8901-fabc-012345678901",
  "status": "ok",
  "data": {
    "accepted": true,
    "goal_id": "abc12345-def6-7890-ghij-klmnopqrstuv"
  },
  "timestamp": 1739913613.654
}
```

**Goal Rejected:**

```
Server:
{
  "id": "f2a3b4c5-d6e7-8901-fabc-012345678901",
  "status": "ok",
  "data": {
    "accepted": false,
    "goal_id": ""
  },
  "timestamp": 1739913613.654
}
```

---

#### 6.4.3 `action_cancel`

Cancel an active goal on an action server. If no `goal_id` is specified, all active goals on the given action server are cancelled.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `action` | `string` | Yes | The fully qualified action server name. |
| `goal_id` | `string` | No | The ID of a specific goal to cancel. If omitted, all goals are cancelled. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `cancelled` | `boolean` | `true` if the cancellation request was accepted. |

**Example (cancel specific goal):**

```
Client:
{
  "id": "a3b4c5d6-e7f8-9012-abcd-123456789012",
  "type": "action_cancel",
  "params": {
    "action": "/navigate_to_pose",
    "goal_id": "abc12345-def6-7890-ghij-klmnopqrstuv"
  }
}

Server:
{
  "id": "a3b4c5d6-e7f8-9012-abcd-123456789012",
  "status": "ok",
  "data": {
    "cancelled": true
  },
  "timestamp": 1739913614.987
}
```

**Example (cancel all goals):**

```
Client:
{
  "id": "b4c5d6e7-f8a9-0123-bcde-234567890123",
  "type": "action_cancel",
  "params": {
    "action": "/navigate_to_pose"
  }
}

Server:
{
  "id": "b4c5d6e7-f8a9-0123-bcde-234567890123",
  "status": "ok",
  "data": {
    "cancelled": true
  },
  "timestamp": 1739913615.321
}
```

---

#### 6.4.4 `action_status`

Get the status of all goals on an action server.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `action` | `string` | Yes | The fully qualified action server name. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `statuses` | `array` | Array of goal status objects, each containing goal metadata and current state. |

**Example:**

```
Client:
{
  "id": "c5d6e7f8-a9b0-1234-cdef-345678901234",
  "type": "action_status",
  "params": {
    "action": "/navigate_to_pose"
  }
}

Server:
{
  "id": "c5d6e7f8-a9b0-1234-cdef-345678901234",
  "status": "ok",
  "data": {
    "statuses": [
      {
        "goal_id": "abc12345-def6-7890-ghij-klmnopqrstuv",
        "status": "EXECUTING"
      },
      {
        "goal_id": "xyz98765-uvw4-3210-pqrs-lkjihgfedcba",
        "status": "SUCCEEDED"
      }
    ]
  },
  "timestamp": 1739913616.654
}
```

---

### 6.5 Node Commands

#### 6.5.1 `node_list`

List all active ROS2 nodes on the graph.

**Parameters:** None.

**Response Data:** An array of node name strings.

**Example:**

```
Client:
{
  "id": "d6e7f8a9-b0c1-2345-defa-456789012345",
  "type": "node_list",
  "params": {}
}

Server:
{
  "id": "d6e7f8a9-b0c1-2345-defa-456789012345",
  "status": "ok",
  "data": [
    "/turtlebot3_burger/robot_state_publisher",
    "/gazebo",
    "/physical_mcp_bridge",
    "/rviz2"
  ],
  "timestamp": 1739913617.987
}
```

---

### 6.6 Emergency Commands

#### 6.6.1 `emergency_stop`

Activate the emergency stop. This immediately halts all robot motion and blocks subsequent commands that could affect the physical system.

**Behavior on the server (bridge) side:**
1. Sets the internal emergency stop flag to `true`.
2. Cancels all active action goals.
3. Publishes a zero-velocity `Twist` message to `/cmd_vel`.
4. All subsequent `topic_publish`, `service_call`, and `action_send_goal` commands are rejected while the e-stop is active.

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `reason` | `string` | No | A human-readable reason for activating the emergency stop. Logged for audit purposes. |

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `stopped` | `boolean` | `true` if the emergency stop was successfully activated. |

**Example:**

```
Client:
{
  "id": "e7f8a9b0-c1d2-3456-efab-567890123456",
  "type": "emergency_stop",
  "params": {
    "reason": "Obstacle detected within safety perimeter"
  }
}

Server:
{
  "id": "e7f8a9b0-c1d2-3456-efab-567890123456",
  "status": "ok",
  "data": {
    "stopped": true
  },
  "timestamp": 1739913618.123
}
```

---

#### 6.6.2 `emergency_stop_release`

Release the emergency stop, allowing commands to resume.

**Parameters:** None.

**Response Data:**

| Field | Type | Description |
|-------|------|-------------|
| `released` | `boolean` | `true` if the emergency stop was successfully released. |

**Example:**

```
Client:
{
  "id": "f8a9b0c1-d2e3-4567-fabc-678901234567",
  "type": "emergency_stop_release",
  "params": {}
}

Server:
{
  "id": "f8a9b0c1-d2e3-4567-fabc-678901234567",
  "status": "ok",
  "data": {
    "released": true
  },
  "timestamp": 1739913619.456
}
```

**Note:** The MCP server (client-side) enforces an additional confirmation requirement (`CONFIRM_RELEASE`) before issuing this command. This is a safety measure handled at the MCP tool layer, not at the WebSocket protocol layer.

---

## 7. Connection Lifecycle

### 7.1 Overview

```
  Client (MCP Server)                         Server (ROS2 Bridge)
  ====================                         ====================

  1. TCP Connect
     ──────────────────────────────────────────>
                                                Accept

  2. WebSocket Handshake (HTTP Upgrade)
     ──────────────────────────────────────────>
     <──────────────────────────────────────────
                                                101 Switching Protocols

  3. Connection Verification (ping)
     {"id":"...","type":"ping","params":{}}
     ──────────────────────────────────────────>
     <──────────────────────────────────────────
     {"id":"...","status":"ok","data":{"bridge":"ok"},...}

  4. Normal Operation (commands + responses)
     {"id":"...","type":"topic_list",...}
     ──────────────────────────────────────────>
     <──────────────────────────────────────────
     {"id":"...","status":"ok","data":[...],...}

     {"id":"...","type":"topic_publish",...}
     ──────────────────────────────────────────>
     <──────────────────────────────────────────
     {"id":"...","status":"ok","data":{"published":true},...}

       ...repeated for duration of session...

  5. Heartbeat (WebSocket ping/pong frames)
     [WS PING frame]
     ──────────────────────────────────────────>
     <──────────────────────────────────────────
     [WS PONG frame]

       ...every 15 seconds...

  6. Disconnect
     [WS Close frame]
     ──────────────────────────────────────────>
     <──────────────────────────────────────────
     [WS Close frame]
```

### 7.2 Connection Establishment

1. The client initiates a WebSocket connection to `ws://<host>:9090`.
2. Upon successful WebSocket handshake, the client SHOULD send a `ping` command to verify the bridge is operational.
3. If the `ping` response indicates the bridge is healthy, the connection is considered established.
4. If the `ping` fails or times out, the client SHOULD close the connection and retry after a delay.

### 7.3 Normal Operation

During normal operation, the client sends commands and receives responses. Commands and responses are correlated by the `id` field.

- The client MAY have multiple commands in-flight concurrently.
- The server MUST respond to each command exactly once.
- Responses MAY arrive in a different order than commands were sent.
- The client MUST track pending requests and match responses by `id`.

### 7.4 Heartbeat Mechanism

The client maintains connection health using WebSocket-level ping/pong frames (not to be confused with the application-level `ping` command).

```
  Heartbeat Timeline
  ==================

  t=0s     Client sends WS PING frame
  t=0s     Server responds with WS PONG frame (automatic per RFC 6455)
  ...
  t=15s    Client sends WS PING frame
  t=15s    Server responds with WS PONG frame
  ...
  t=30s    Client sends WS PING frame
  t=30s    Server responds with WS PONG frame
  ...

  Stale Connection Detection
  ==========================

  t=0s     Client sends WS PING frame
  t=0s     Server responds with WS PONG frame
  ...
  t=15s    Client sends WS PING frame
           [no PONG received -- network issue / server crash]
  ...
  t=30s    Client sends WS PING frame
           [no PONG received]
  ...
  t=45s    Client checks: last PONG was at t=0s (45s ago > 30s threshold)
           Client TERMINATES the connection
           Client begins reconnection procedure
```

**Rules:**

- The client MUST send a WebSocket ping frame every **15 seconds**.
- The client MUST track the timestamp of the most recent pong frame received.
- If no pong frame has been received within **30 seconds** of the last pong, the client MUST consider the connection stale.
- On detecting a stale connection, the client MUST terminate the WebSocket connection (not a clean close -- use `terminate()`).
- The client SHOULD then initiate the reconnection procedure (see Section 7.5).

### 7.5 Reconnection

When a connection is lost (either through stale detection, server-initiated close, or network error):

```
  Reconnection Procedure
  ======================

  Connection Lost
       |
       v
  Reject all pending requests
  with "Connection closed" error
       |
       v
  Wait 5 seconds
       |
       v
  Attempt reconnection -------> Success? ---> Resume normal operation
       |                                       (send ping to verify)
       | Failure
       v
  Wait 5 seconds
       |
       v
  Attempt reconnection -------> Success? ---> Resume normal operation
       |
       | Failure
       v
    (repeat indefinitely)
```

**Rules:**

- On disconnection, ALL pending requests MUST be rejected with an error.
- The client MUST attempt reconnection every **5 seconds**.
- The reconnection loop continues until a connection is established or the client is shut down.
- A circuit breaker SHOULD be employed to prevent excessive reconnection attempts. After **5 consecutive failures**, the circuit breaker opens for **30 seconds** before allowing further attempts.
- When the circuit breaker is open, `send()` calls fail immediately with an error rather than waiting for reconnection.

### 7.6 Disconnection

On intentional disconnection:

1. The client stops the heartbeat timer.
2. All pending requests are rejected with a "Disconnecting" error.
3. The client sends a WebSocket close frame.
4. The WebSocket connection is terminated.

---

## 8. Error Handling

### 8.1 Error Response Format

All errors are returned as standard responses with `status` set to `"error"`:

```json
{
  "id": "<request-id-or-null>",
  "status": "error",
  "data": {
    "error": "<human-readable error message>"
  },
  "timestamp": 1234567890.123
}
```

### 8.2 Error Categories

#### 8.2.1 Parse Errors

Occur when the server cannot parse an incoming message as valid JSON or extract the required command fields.

- The `id` field in the response is set to `null` (since the ID could not be extracted).
- The client SHOULD log parse errors but cannot correlate them to a specific pending request.

**Example:**

```
Client sends:
  { this is not valid JSON }

Server responds:
{
  "id": null,
  "status": "error",
  "data": {
    "error": "Parse error: Expecting property name enclosed in double quotes: line 1 column 3 (char 2)"
  },
  "timestamp": 1739913620.123
}
```

#### 8.2.2 Unknown Command Type

Occurs when the `type` field does not match any defined command type.

**Example:**

```
Client:
{
  "id": "a1b2c3d4-0000-0000-0000-000000000000",
  "type": "robot_dance",
  "params": {}
}

Server:
{
  "id": "a1b2c3d4-0000-0000-0000-000000000000",
  "status": "error",
  "data": {
    "error": "Unknown command: robot_dance"
  },
  "timestamp": 1739913621.456
}
```

#### 8.2.3 Missing Required Parameters

Occurs when a command is missing one or more required parameters.

**Example:**

```
Client:
{
  "id": "b2c3d4e5-0000-0000-0000-000000000000",
  "type": "topic_info",
  "params": {}
}

Server:
{
  "id": "b2c3d4e5-0000-0000-0000-000000000000",
  "status": "error",
  "data": {
    "error": "'topic'"
  },
  "timestamp": 1739913622.789
}
```

#### 8.2.4 ROS2 Execution Errors

Occur when a command is valid but the underlying ROS2 operation fails (e.g., topic does not exist, service is unavailable, message type mismatch).

**Example:**

```
Client:
{
  "id": "c3d4e5f6-0000-0000-0000-000000000000",
  "type": "topic_publish",
  "params": {
    "topic": "/nonexistent_topic",
    "message_type": "std_msgs/msg/String",
    "message": { "data": "hello" }
  }
}

Server:
{
  "id": "c3d4e5f6-0000-0000-0000-000000000000",
  "status": "error",
  "data": {
    "error": "Failed to create publisher for /nonexistent_topic"
  },
  "timestamp": 1739913623.321
}
```

#### 8.2.5 Emergency Stop Rejection

Occurs when a mutating command (publish, service call, action goal) is sent while the emergency stop is active on the bridge.

**Example:**

```
Client:
{
  "id": "d4e5f6a7-0000-0000-0000-000000000000",
  "type": "topic_publish",
  "params": {
    "topic": "/cmd_vel",
    "message_type": "geometry_msgs/msg/Twist",
    "message": {
      "linear": { "x": 0.1, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
    }
  }
}

Server:
{
  "id": "d4e5f6a7-0000-0000-0000-000000000000",
  "status": "ok",
  "data": {
    "error": "Emergency stop active on bridge"
  },
  "timestamp": 1739913624.654
}
```

**Note:** The bridge returns this with `status: "ok"` but includes an `error` field in `data`. This is because the bridge-level e-stop is a secondary safety mechanism; the primary e-stop in the MCP server blocks the command before it reaches the bridge.

#### 8.2.6 Client-Side Timeout

The client enforces a default timeout of **10 seconds** per request. If no response is received within this window, the client rejects the pending request locally.

- This is a client-side mechanism; no message is sent to the server.
- The server MAY still send a response later, which the client MUST discard (no matching pending request).
- Timed-out requests produce an error of the form: `Request <id> timed out after 10000ms`.

---

## 9. Validation

### 9.1 Client-Side Validation (TypeScript)

The client validates all incoming responses using [Zod](https://github.com/colinhacks/zod) schemas:

```typescript
const BridgeCommandSchema = z.object({
  id: z.string(),
  type: z.string(),
  params: z.record(z.unknown()).optional().default({}),
});

const BridgeResponseSchema = z.object({
  id: z.string().nullable(),
  status: z.enum(['ok', 'error']),
  data: z.unknown(),
  timestamp: z.number(),
});
```

- Outgoing commands are constructed programmatically and are guaranteed to conform.
- Incoming responses are parsed through `BridgeResponseSchema.parse()`.
- Responses that fail validation are discarded and a warning is logged.

### 9.2 Server-Side Validation (Python)

The server validates incoming commands manually:

- The raw message is parsed as JSON. A `json.JSONDecodeError` produces a parse error response with `id: null`.
- The `id` field is extracted; a missing `id` raises a `KeyError`.
- The `type` field is matched against the `CommandType` enum. An invalid type produces a `ValueError`.
- The `params` field defaults to `{}` if absent.
- Required parameters for each command type are accessed directly; missing keys raise `KeyError`, which is caught and returned as an error response.

### 9.3 Validation Rules

| Rule | Enforcement Point |
|------|-------------------|
| Command `id` must be a non-empty string | Client (generation), Server (extraction) |
| Command `type` must be a known command type | Server |
| Required params must be present | Server |
| Response `id` must match a pending request | Client |
| Response `status` must be `"ok"` or `"error"` | Client (Zod) |
| Response `timestamp` must be a number | Client (Zod) |
| Messages must be valid JSON | Both sides |

---

## 10. Security Considerations

### 10.1 Transport Security

The default transport (`ws://`) is unencrypted. For deployments outside of `localhost`:

- The connection SHOULD use TLS (`wss://`).
- The server SHOULD be configured with appropriate TLS certificates.
- The client SHOULD verify the server certificate.

### 10.2 Authentication

The current protocol does not include authentication. Access control MUST be enforced through network-level mechanisms:

- Bind the server to `localhost` or a private network interface.
- Use firewall rules to restrict access to port 9090.
- Use SSH tunnels or VPNs for remote access.

### 10.3 Input Sanitization

- The server MUST NOT execute arbitrary code from message payloads.
- ROS2 message construction uses the `rosidl_runtime_py` library, which sets fields through typed accessors. Arbitrary fields are silently ignored.
- Service and action type strings are validated against the ROS2 type system before use.

### 10.4 Denial of Service

- The client-side safety layer enforces rate limits before commands reach the bridge.
- The server SHOULD limit the number of concurrent WebSocket connections.
- The server SHOULD implement message size limits to prevent memory exhaustion.

---

## Appendix A: Full Message Schema

### A.1 Command Types Summary

| Command Type | Category | Parameters | Response Data |
|-------------|----------|------------|---------------|
| `ping` | System | _(none)_ | `{ bridge: "ok" }` |
| `topic_list` | Topic | _(none)_ | `[{ name, type }, ...]` |
| `topic_info` | Topic | `{ topic }` | `{ name, type, publisher_count, subscriber_count }` |
| `topic_subscribe` | Topic | `{ topic, count?, timeout_ms? }` | `{ messages: [...] }` |
| `topic_publish` | Topic | `{ topic, message_type, message }` | `{ published: true }` |
| `topic_echo` | Topic | `{ topic, timeout_ms? }` | `{ message: ... }` |
| `service_list` | Service | _(none)_ | `[{ name, type }, ...]` |
| `service_info` | Service | `{ service }` | `{ name, type }` |
| `service_call` | Service | `{ service, service_type, request? }` | `{ result: ... }` |
| `action_list` | Action | _(none)_ | `[{ name, type }, ...]` |
| `action_send_goal` | Action | `{ action, action_type, goal }` | `{ accepted, goal_id }` |
| `action_cancel` | Action | `{ action, goal_id? }` | `{ cancelled: true }` |
| `action_status` | Action | `{ action }` | `{ statuses: [...] }` |
| `node_list` | Node | _(none)_ | `["node_name", ...]` |
| `emergency_stop` | Emergency | `{ reason? }` | `{ stopped: true }` |
| `emergency_stop_release` | Emergency | _(none)_ | `{ released: true }` |

### A.2 Type Definitions (TypeScript)

```typescript
// Command (Client -> Server)
interface BridgeCommand {
  id: string;           // UUID v4
  type: string;         // One of the 16 command types
  params: Record<string, unknown>;  // Command-specific parameters
}

// Response (Server -> Client)
interface BridgeResponse {
  id: string | null;    // Matches the command ID, or null for parse errors
  status: 'ok' | 'error';
  data: unknown;        // Command-specific response data
  timestamp: number;    // Unix epoch seconds with fractional milliseconds
}
```

### A.3 Type Definitions (Python)

```python
@dataclass
class Command:
    id: str               # UUID v4
    type: CommandType      # Enum of 16 command types
    params: dict           # Command-specific parameters

def build_response(
    cmd_id: Optional[str],  # Matches the command ID, or None for parse errors
    status: str,            # "ok" or "error"
    data: Any,              # Command-specific response data
) -> dict:
    return {
        'id': cmd_id,
        'status': status,
        'data': data,
        'timestamp': time.time(),  # Unix epoch seconds
    }
```

---

## Appendix B: Status Codes Summary

| Status | Meaning | When Used |
|--------|---------|-----------|
| `"ok"` | Command executed successfully | Normal successful responses |
| `"error"` | Command execution failed | Parse errors, unknown commands, missing params, ROS2 errors |

---

*End of specification.*

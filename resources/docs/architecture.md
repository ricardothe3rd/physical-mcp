# PhysicalMCP Architecture Deep-Dive

**Version:** 1.0
**Last Updated:** 2026-02-18
**Package:** `@ricardothe3rd/physical-mcp`

---

## Table of Contents

1. [Overview](#overview)
2. [Design Philosophy](#design-philosophy)
3. [System Architecture](#system-architecture)
4. [Component Breakdown](#component-breakdown)
   - [TypeScript MCP Server](#typescript-mcp-server)
   - [Python ROS2 Bridge](#python-ros2-bridge)
   - [Docker Infrastructure](#docker-infrastructure)
5. [Communication Architecture](#communication-architecture)
6. [Safety Architecture](#safety-architecture)
7. [Connection Resilience](#connection-resilience)
8. [Protocol Specification](#protocol-specification)
9. [Tool Taxonomy](#tool-taxonomy)
10. [Data Flow Walkthrough](#data-flow-walkthrough)
11. [Deployment Topology](#deployment-topology)
12. [Key Design Decisions and Rationale](#key-design-decisions-and-rationale)

---

## Overview

PhysicalMCP is a safety-first Model Context Protocol (MCP) server that enables AI agents to interact with physical robots running ROS2. It bridges the gap between the AI tool-calling world (MCP over stdio) and the robotics middleware world (ROS2/DDS), while enforcing configurable safety policies that prevent an AI agent from issuing dangerous commands to real hardware.

The system is designed around one core invariant: **no command reaches the robot without first passing through the safety layer**. This invariant holds regardless of bridge state, network conditions, or agent behavior.

---

## Design Philosophy

PhysicalMCP is built on five principles:

1. **Safety is non-negotiable.** The safety layer lives in the MCP server process, not in the bridge. A misbehaving or disconnected bridge cannot bypass safety checks. Commands are evaluated and potentially blocked before they ever leave the TypeScript process.

2. **Defense in depth.** Critical safety mechanisms (e-stop, rate limiting) are duplicated across layers. A single point of failure in any one layer does not compromise overall safety.

3. **Fail closed.** When the system cannot determine whether an operation is safe --- due to connection loss, policy evaluation errors, or ambiguous configuration --- it denies the operation. The default posture is always denial.

4. **Configuration over code.** Safety policies are defined in YAML, not hardcoded. Different robots, environments, and operational contexts can have completely different safety envelopes without modifying source code.

5. **Resilience by design.** Network partitions, bridge restarts, and transient failures are expected, not exceptional. Circuit breakers, automatic reconnection, and heartbeat monitoring ensure the system degrades gracefully.

---

## System Architecture

```
+----------------------------------------------------------------------+
|                          AI AGENT (e.g., Claude)                     |
|                                                                      |
|  The AI agent calls MCP tools like ros2_topic_publish,               |
|  ros2_service_call, etc. It receives structured responses.           |
+----------------------------------+-----------------------------------+
                                   |
                            MCP Protocol (stdio)
                            JSON-RPC over stdin/stdout
                                   |
+----------------------------------v-----------------------------------+
|                    TYPESCRIPT MCP SERVER                              |
|                    packages/mcp-server/                               |
|                                                                      |
|  +------------------+    +-------------------+    +----------------+ |
|  |   TOOL LAYER     |    |   SAFETY LAYER    |    |  BRIDGE LAYER  | |
|  |                  |    |                   |    |                | |
|  | 21 MCP tools     |--->| PolicyEngine      |--->| WSClient       | |
|  | across 5         |    | PolicyLoader      |    | ConnectionMgr  | |
|  | categories       |    | Geofence          |    | Protocol       | |
|  |                  |    | RateLimiter       |    |                | |
|  | (topic, service, |    | AuditLogger       |    | WebSocket on   | |
|  |  action, safety, |    |                   |    | port 9090      | |
|  |  system)         |    | YAML policies     |    |                | |
|  +------------------+    +-------------------+    +-------+--------+ |
|                                                           |          |
+-----------------------------------------------------------+----------+
                                                            |
                                            WebSocket (JSON, port 9090)
                                            16 command types
                                            UUID correlation IDs
                                                            |
+-----------------------------------------------------------v----------+
|                     PYTHON ROS2 BRIDGE                                |
|                     packages/ros2-bridge/                             |
|                                                                       |
|  +----------------+  +---------------+  +---------------------------+ |
|  | bridge_node.py |  | discovery.py  |  | topic_handler.py          | |
|  | (Main node,    |  | (ROS2 graph   |  | service_handler.py        | |
|  |  WS server,    |  |  introspect)  |  | action_handler.py         | |
|  |  cmd routing)  |  |               |  | telemetry.py              | |
|  +----------------+  +---------------+  +---------------------------+ |
|                                                                       |
|                         Native rclpy (DDS)                            |
+----------------------------+------------------------------------------+
                             |
                    ROS2 Middleware (DDS)
                             |
+----------------------------v------------------------------------------+
|                        ROS2 ECOSYSTEM                                 |
|                                                                       |
|   /cmd_vel   /odom   /scan   /joint_states   /tf   /diagnostics      |
|                                                                       |
|   Navigation2, MoveIt2, sensor drivers, actuator controllers, etc.    |
+-----------------------------------------------------------------------+
```

---

## Component Breakdown

### TypeScript MCP Server

**Location:** `packages/mcp-server/`

The MCP server is the primary process that an AI agent connects to. It is a Node.js/TypeScript application that implements the MCP specification, exposing robot capabilities as callable tools while enforcing safety constraints on every operation.

#### Entry Point: `src/index.ts`

The entry point is responsible for:

- Parsing CLI arguments (WebSocket URL, policy file path, log level)
- Initializing the MCP server instance
- Registering all 21 tools with their schemas and handlers
- Establishing the WebSocket connection to the Python bridge
- Starting the stdio transport for MCP communication

The server starts in a well-defined sequence: safety layer initialization (policy loading, geofence configuration) completes before tool registration, which completes before the connection to the bridge is attempted. This ordering guarantees that the safety layer is always fully operational before any tool can be invoked.

#### Bridge Layer: `src/bridge/`

The bridge layer manages the WebSocket connection between the TypeScript server and the Python ROS2 bridge.

```
src/bridge/
  +-- WSClient.ts          WebSocket client with heartbeat and request/response matching
  +-- ConnectionManager.ts Auto-reconnect logic with circuit breaker
  +-- Protocol.ts          16 command types, Zod validation schemas
```

**WSClient** maintains a persistent WebSocket connection with the following capabilities:

- **Heartbeat monitoring:** Sends a ping every 15 seconds. If no pong is received within 30 seconds, the connection is considered stale and is torn down for reconnection.
- **Request/response matching:** Every outgoing command is tagged with a UUID correlation ID. The client maintains a map of pending requests and resolves the correct Promise when the corresponding response arrives. This enables concurrent in-flight commands without ambiguity.
- **Serialization:** All messages are JSON-serialized and validated against Zod schemas before transmission.

**ConnectionManager** wraps `WSClient` with resilience logic:

- **Automatic reconnection:** When a connection drops, the manager immediately begins reconnection attempts with exponential backoff.
- **Circuit breaker:** After a configurable number of consecutive connection failures, the circuit breaker opens. While open, all tool invocations that require bridge communication receive an immediate error response without attempting a connection. The circuit breaker transitions to half-open after a cooldown period, allowing a single probe request through.

**Protocol** defines the contract between the TypeScript server and the Python bridge:

- 16 command types covering topic, service, action, discovery, and system operations
- Zod schemas for every command request and response, providing runtime type safety
- Serialization/deserialization helpers

#### Safety Layer: `src/safety/`

The safety layer is the most critical subsystem. It sits between tool handlers and bridge communication, evaluating every operation against a set of configurable policies before allowing it to proceed.

```
src/safety/
  +-- PolicyEngine.ts   Core evaluator; the single checkpoint for all operations
  +-- PolicyLoader.ts   YAML parser that reads and validates policy files
  +-- Geofence.ts       Rectangular geographic bounds checking
  +-- RateLimiter.ts    Sliding-window rate limiting per topic/service
  +-- AuditLogger.ts    Immutable command audit trail
```

**PolicyEngine** is the central evaluator. It exposes methods like `checkPublish()`, `checkServiceCall()`, and `checkActionGoal()`. Each method runs the operation through an ordered evaluation pipeline:

```
Operation arrives
       |
       v
  [E-Stop Active?] ---YES---> BLOCK (all operations denied)
       |
       NO
       v
  [Topic/Service Blocked?] ---YES---> BLOCK (explicit deny list)
       |
       NO
       v
  [Velocity Limits?] ---EXCEEDS---> BLOCK (for /cmd_vel and similar)
       |
       WITHIN BOUNDS
       v
  [Rate Limit?] ---EXCEEDED---> BLOCK (too many commands in window)
       |
       WITHIN LIMIT
       v
  [Geofence?] ---OUTSIDE---> BLOCK (position-based commands only)
       |
       INSIDE or N/A
       v
  ALLOW --> forward to bridge
```

**PolicyLoader** reads YAML policy files and produces a structured policy object. Policies can define:

- Blocked topic and service names (glob patterns supported)
- Velocity limits (linear and angular, per-axis)
- Rate limits (max commands per time window, per topic)
- Geofence boundaries (rectangular, defined by min/max x/y coordinates)
- E-stop behavior and override rules

**Geofence** implements rectangular bounds checking. When a position-aware command (such as a navigation goal) is evaluated, the geofence module checks whether the target position falls within the configured operational envelope. Operations targeting positions outside the geofence are blocked.

**RateLimiter** implements a sliding-window algorithm. For each topic or service, it maintains a timestamped log of recent operations. When a new operation arrives, it counts how many operations occurred within the configured window and compares against the configured maximum. This prevents an AI agent from flooding a topic with high-frequency commands, which could cause physical damage through rapid oscillation or excessive actuator wear.

**AuditLogger** records every operation --- allowed and blocked --- to an append-only audit trail. Each log entry includes:

- Timestamp
- Operation type and target (topic, service, action)
- Parameters (with sensitive fields redacted if configured)
- Decision (allowed/blocked) and reason
- Correlation ID for tracing

#### Tools: `src/tools/`

The server exposes 21 MCP tools organized into five categories:

| Category | Tools | Purpose |
|----------|-------|---------|
| **Topic** | subscribe, unsubscribe, echo, publish, list | Interact with ROS2 pub/sub topics |
| **Service** | call, list, type | Call ROS2 services and discover available services |
| **Action** | send_goal, cancel, status, list | Manage ROS2 action goals (long-running tasks) |
| **Safety** | e_stop, get_policy, set_policy, get_audit_log | Control and inspect safety mechanisms |
| **System** | get_nodes, get_status, ping, diagnostics | Inspect the ROS2 graph and bridge health |

Each tool handler follows the same pattern:

1. Parse and validate input parameters (Zod schemas)
2. Call the appropriate `PolicyEngine.check*()` method
3. If denied: return the violation details to the AI agent
4. If allowed: construct a protocol command, send via `WSClient`, await response
5. Transform the bridge response into the MCP tool result format

#### Utils

- **`error-recovery.ts`**: Provides a `withRetry()` wrapper implementing exponential backoff with jitter, and a standalone circuit breaker class for wrapping arbitrary async operations.
- **`errors.ts`**: Defines a hierarchy of custom error types (`ConnectionError`, `TimeoutError`, `PolicyViolationError`, `ProtocolError`, etc.) for precise error handling throughout the stack.

---

### Python ROS2 Bridge

**Location:** `packages/ros2-bridge/`

The Python bridge is a ROS2 node that translates WebSocket commands from the TypeScript server into native ROS2 operations via `rclpy`. It runs inside a ROS2 environment (typically a Docker container with ROS2 Humble installed).

```
packages/ros2-bridge/
  +-- bridge_node.py      Main rclpy node, WebSocket server, command routing
  +-- discovery.py        ROS2 graph introspection (topics, services, actions, nodes)
  +-- topic_handler.py    Topic subscribe, echo, publish operations
  +-- service_handler.py  Service calls with configurable timeout
  +-- action_handler.py   Action goal management (send, cancel, status)
  +-- telemetry.py        Bridge health metrics (uptime, command counts, latencies)
  +-- protocol.py         Python mirror of the TypeScript protocol definitions
```

**bridge_node.py** is the main entry point. It:

1. Initializes an `rclpy` node
2. Starts a WebSocket server on port 9090
3. Accepts connections from the TypeScript MCP server
4. Routes incoming commands to the appropriate handler based on command type
5. Sends responses back over the WebSocket with matching correlation IDs
6. Maintains a secondary e-stop state (defense-in-depth)

**discovery.py** uses the ROS2 graph API to discover:

- All active nodes and their namespaces
- Published and subscribed topics with message types
- Available services with service types
- Available actions with action types

This information is used by the system tools (node listing, topic listing) and helps the AI agent understand what capabilities are available on the robot.

**topic_handler.py** manages ROS2 topic operations:

- Creates dynamic subscribers for topic echo/subscribe
- Creates dynamic publishers for topic publish
- Handles message type resolution and deserialization
- Manages subscription lifecycles

**service_handler.py** manages synchronous ROS2 service calls:

- Creates dynamic service clients
- Handles request serialization and response deserialization
- Enforces a configurable timeout to prevent indefinite blocking

**action_handler.py** manages ROS2 action operations:

- Creates dynamic action clients
- Sends goals with feedback callbacks
- Supports goal cancellation
- Tracks goal status (pending, active, succeeded, failed, canceled)

**telemetry.py** collects bridge health metrics:

- Uptime and connection count
- Command counts by type (total, success, failure)
- Latency statistics (mean, p95, p99)
- ROS2 graph statistics (node count, topic count)

**protocol.py** mirrors the TypeScript protocol definitions in Python, ensuring both sides of the WebSocket agree on message formats. This is a manual synchronization point --- changes to the protocol must be reflected in both files.

---

### Docker Infrastructure

**Location:** `docker/`

```
docker/
  +-- Dockerfile.bridge    ROS2 Humble base image + Python bridge dependencies
  +-- Dockerfile.sim       Gazebo simulation with TurtleBot3
  +-- docker-compose.yml   Orchestrates bridge + simulation containers
```

**Dockerfile.bridge** builds a container with:

- ROS2 Humble (Ubuntu 22.04 base)
- Python 3.10+ with `rclpy`, `websockets`, and bridge dependencies
- The Python bridge source code

**Dockerfile.sim** builds a simulation container with:

- ROS2 Humble
- Gazebo (physics simulator)
- TurtleBot3 simulation packages
- A pre-configured simulation world

**docker-compose.yml** orchestrates both containers with:

- Shared ROS2 DDS network (containers on the same ROS_DOMAIN_ID)
- Port 9090 exposed from the bridge container for WebSocket connections
- Volume mounts for policy files and logs
- Health checks for both containers

---

## Communication Architecture

PhysicalMCP uses three distinct communication channels, each optimized for its specific role:

```
+-------------+          +------------------+          +---------------+
|  AI Agent   |  stdio   | TypeScript MCP   |    WS    | Python ROS2   |
|  (Claude,   |<-------->| Server           |<-------->| Bridge        |
|   etc.)     | MCP/     | (Node.js)        |  JSON    | (rclpy node)  |
|             | JSON-RPC |                  | :9090    |               |
+-------------+          +------------------+          +-------+-------+
                                                               |
                                                          rclpy (DDS)
                                                               |
                                                        +------v------+
                                                        | ROS2 Graph  |
                                                        | (Topics,    |
                                                        |  Services,  |
                                                        |  Actions)   |
                                                        +-------------+
```

### Channel 1: AI Agent to MCP Server (stdio / MCP Protocol)

- **Transport:** Standard I/O (stdin/stdout)
- **Protocol:** MCP (Model Context Protocol), which is JSON-RPC 2.0 based
- **Direction:** Bidirectional. The agent sends tool calls; the server sends results.
- **Characteristics:** Synchronous from the agent's perspective. The agent sends a tool call and waits for the result.

### Channel 2: MCP Server to Python Bridge (WebSocket)

- **Transport:** WebSocket on port 9090
- **Protocol:** Custom JSON protocol with 16 command types
- **Direction:** Bidirectional. The server sends commands; the bridge sends responses. The bridge may also push asynchronous events (topic messages, action feedback).
- **Characteristics:**
  - Persistent connection with heartbeat monitoring
  - Concurrent in-flight commands via UUID correlation IDs
  - Automatic reconnection with circuit breaker

### Channel 3: Python Bridge to ROS2 (Native rclpy / DDS)

- **Transport:** DDS (Data Distribution Service) via rclpy
- **Protocol:** Native ROS2 message serialization (CDR)
- **Direction:** Bidirectional. The bridge publishes to topics, calls services, sends action goals, and subscribes to topics, receives service responses, and receives action feedback.
- **Characteristics:** Zero-copy in shared-memory configurations. Fully decoupled pub/sub semantics. QoS profiles for reliability tuning.

---

## Safety Architecture

Safety is the defining characteristic of PhysicalMCP. The safety architecture is designed to prevent an AI agent from causing physical harm through a robot, even if the agent is operating with flawed reasoning or adversarial intent.

### Safety Invariant

**No ROS2 command originating from an AI agent reaches the robot without passing through the PolicyEngine.**

This invariant is enforced architecturally: the PolicyEngine sits in the only code path between tool handlers and the WebSocket client. There is no alternative path.

### Defense-in-Depth Model

```
+-----------------------------------------------------------------------+
|                        LAYER 1: MCP Server                            |
|                                                                       |
|  +--------------------+                                               |
|  | PolicyEngine       |  Evaluates EVERY operation before it can      |
|  | - E-Stop           |  reach the bridge. This is the primary        |
|  | - Blocked Topics   |  safety checkpoint.                          |
|  | - Velocity Limits  |                                               |
|  | - Rate Limits      |  If this layer blocks a command, the command  |
|  | - Geofence         |  NEVER leaves the TypeScript process.         |
|  +--------------------+                                               |
|                                                                       |
|  +--------------------+                                               |
|  | AuditLogger        |  Records all decisions for post-incident      |
|  |                    |  analysis and compliance.                     |
|  +--------------------+                                               |
+-----------------------------------------------------------------------+

+-----------------------------------------------------------------------+
|                        LAYER 2: Python Bridge                         |
|                                                                       |
|  +--------------------+                                               |
|  | Secondary E-Stop   |  Independent e-stop state in the bridge.     |
|  |                    |  If the bridge's e-stop is active, no         |
|  |                    |  commands are executed even if they passed    |
|  |                    |  Layer 1.                                     |
|  +--------------------+                                               |
|                                                                       |
|  This layer provides redundancy. If the TypeScript server's safety    |
|  layer has a defect, the bridge provides a second line of defense.    |
+-----------------------------------------------------------------------+

+-----------------------------------------------------------------------+
|                        LAYER 3: ROS2 / Robot                          |
|                                                                       |
|  Hardware safety systems (limit switches, hardware e-stop, motor      |
|  controllers with built-in current limiting) provide the final        |
|  safety net. PhysicalMCP does not replace these; it complements them. |
+-----------------------------------------------------------------------+
```

### Safety Evaluation Pipeline (Detailed)

When an AI agent calls a tool like `ros2_topic_publish` with a velocity command, the following evaluation occurs:

```
ros2_topic_publish("/cmd_vel", { linear: { x: 5.0 }, angular: { z: 2.0 } })
                        |
                        v
            +------------------------+
            | PolicyEngine           |
            | .checkPublish()        |
            +------------------------+
                        |
         +--------------v--------------+
         | 1. E-STOP CHECK             |
         |                             |
         | Is the global e-stop        |
         | active?                     |
         |                             |
         | YES --> BLOCK               |
         |   reason: "E-stop active"   |
         |   severity: CRITICAL        |
         |                             |
         | NO --> continue             |
         +-----------------------------+
                        |
         +--------------v--------------+
         | 2. BLOCKED TOPIC CHECK      |
         |                             |
         | Is "/cmd_vel" in the        |
         | blocked topics list?        |
         |                             |
         | YES --> BLOCK               |
         |   reason: "Topic blocked    |
         |   by policy"               |
         |                             |
         | NO --> continue             |
         +-----------------------------+
                        |
         +--------------v--------------+
         | 3. VELOCITY LIMITS CHECK    |
         |                             |
         | For velocity topics:        |
         | linear.x = 5.0             |
         |   max allowed = 1.0        |
         |   5.0 > 1.0 --> BLOCK      |
         |                             |
         | angular.z = 2.0            |
         |   max allowed = 1.5        |
         |   2.0 > 1.5 --> BLOCK      |
         |                             |
         | reason: "Velocity exceeds   |
         |   policy limit: linear.x    |
         |   5.0 > max 1.0"           |
         +-----------------------------+
                        |
         +--------------v--------------+
         | 4. RATE LIMIT CHECK         |
         |                             |
         | Sliding window for          |
         | "/cmd_vel":                 |
         |   Window: 1 second          |
         |   Max: 10 commands          |
         |   Current: 8               |
         |   8 < 10 --> ALLOW          |
         |                             |
         | If 11 --> BLOCK             |
         |   reason: "Rate limit       |
         |   exceeded for /cmd_vel"    |
         +-----------------------------+
                        |
         +--------------v--------------+
         | 5. GEOFENCE CHECK           |
         |                             |
         | (For navigation goals only) |
         | Target position within      |
         | configured rectangular      |
         | boundary?                   |
         |                             |
         | OUTSIDE --> BLOCK           |
         | INSIDE / N/A --> ALLOW      |
         +-----------------------------+
                        |
                        v
              +------------------+
              | DECISION: ALLOW  |
              | or BLOCK         |
              +------------------+
                   |         |
            ALLOW  |         | BLOCK
                   v         v
            Forward to    Return violation
            WSClient      to AI agent
            for bridge    (command never
            execution     leaves process)
```

### E-Stop Mechanism

The e-stop is the highest-priority safety control. When activated:

- **MCP Server e-stop:** All `PolicyEngine.check*()` methods immediately return BLOCK for every operation. No commands reach the bridge. The e-stop can be activated and deactivated via the `ros2_e_stop` tool.
- **Bridge e-stop:** The bridge maintains its own independent e-stop state. Even if a command somehow bypasses the server e-stop (which should be impossible architecturally), the bridge e-stop provides a redundant check.

The dual e-stop design ensures that a defect in either layer does not compromise overall safety.

### Audit Trail

Every operation --- both allowed and blocked --- is recorded by the AuditLogger. The audit trail provides:

- **Operational visibility:** Operators can review what the AI agent did during a session.
- **Incident investigation:** When something goes wrong, the audit trail provides a complete timeline of commands and safety decisions.
- **Compliance:** The immutable log can be used to demonstrate that safety policies were enforced.

The audit log is queryable via the `ros2_get_audit_log` MCP tool, allowing the AI agent itself to review its own command history.

---

## Connection Resilience

The connection between the MCP server and the Python bridge operates over a network (WebSocket), which means it is subject to latency, packet loss, and disconnections. PhysicalMCP handles these conditions explicitly.

### Heartbeat Monitoring

```
MCP Server                              Python Bridge
     |                                        |
     |--- WebSocket ping ------------------>  |
     |                              (every 15s)|
     |  <--- WebSocket pong ----------------  |
     |                                        |
     |          ... 15 seconds pass ...       |
     |                                        |
     |--- WebSocket ping ------------------>  |
     |                                        |
     |          ... 30 seconds pass ...       |
     |          (no pong received)            |
     |                                        |
     | CONNECTION STALE - tear down           |
     | and reconnect                          |
     |                                        |
```

- **Ping interval:** 15 seconds
- **Stale threshold:** 30 seconds (2 missed pongs)
- **Action on stale:** Connection is torn down and the ConnectionManager initiates reconnection

### Reconnection with Circuit Breaker

```
Connection Lost
       |
       v
  [Attempt Reconnect]
       |
  +----+----+
  |         |
SUCCESS   FAILURE
  |         |
  v         v
Resume   [Increment failure count]
normal        |
operation     v
         [Failures >= threshold?]
              |
         +----+----+
         |         |
         NO       YES
         |         |
         v         v
    Wait with    CIRCUIT OPEN
    exponential   |
    backoff,     All tool calls get
    then retry   immediate error
                  |
                  v
             [Wait cooldown period]
                  |
                  v
             CIRCUIT HALF-OPEN
                  |
                  v
             [Allow one probe request]
                  |
             +----+----+
             |         |
           SUCCESS   FAILURE
             |         |
             v         v
          CIRCUIT    CIRCUIT OPEN
          CLOSED     (restart cooldown)
             |
             v
          Resume normal
          operation
```

### Request/Response Matching

Multiple commands can be in flight simultaneously over a single WebSocket connection. Each command is assigned a UUID correlation ID, enabling the client to match responses to their originating requests:

```
MCP Server                                    Python Bridge
     |                                              |
     |-- { id: "abc-123", type: "topic_publish",    |
     |     topic: "/cmd_vel", ... } ------------->  |
     |                                              |
     |-- { id: "def-456", type: "service_call",     |
     |     service: "/get_map", ... } ----------->  |
     |                                              |
     |  <-- { id: "def-456", status: "success",     |
     |        result: { ... } } ------------------  |
     |                                              |
     |  Response "def-456" matched to pending       |
     |  request "def-456", Promise resolved.        |
     |                                              |
     |  <-- { id: "abc-123", status: "success" } -  |
     |                                              |
     |  Response "abc-123" matched, Promise resolved.|
     |                                              |
```

This design allows the MCP server to handle multiple concurrent tool calls without serializing them, which is important for responsiveness when an AI agent issues rapid sequences of commands.

---

## Protocol Specification

The WebSocket protocol between the MCP server and Python bridge defines 16 command types:

### Command Types by Category

| # | Command Type | Category | Direction | Description |
|---|-------------|----------|-----------|-------------|
| 1 | `topic_subscribe` | Topic | Server -> Bridge | Subscribe to a ROS2 topic |
| 2 | `topic_unsubscribe` | Topic | Server -> Bridge | Unsubscribe from a ROS2 topic |
| 3 | `topic_echo` | Topic | Server -> Bridge | Get current messages on a topic |
| 4 | `topic_publish` | Topic | Server -> Bridge | Publish a message to a topic |
| 5 | `topic_list` | Topic | Server -> Bridge | List available topics |
| 6 | `service_call` | Service | Server -> Bridge | Call a ROS2 service |
| 7 | `service_list` | Service | Server -> Bridge | List available services |
| 8 | `service_type` | Service | Server -> Bridge | Get a service's type |
| 9 | `action_send_goal` | Action | Server -> Bridge | Send a goal to an action server |
| 10 | `action_cancel` | Action | Server -> Bridge | Cancel an active action goal |
| 11 | `action_status` | Action | Server -> Bridge | Get status of an action goal |
| 12 | `action_list` | Action | Server -> Bridge | List available actions |
| 13 | `node_list` | System | Server -> Bridge | List active ROS2 nodes |
| 14 | `ping` | System | Server -> Bridge | Connectivity check |
| 15 | `e_stop` | Safety | Server -> Bridge | Activate/deactivate bridge e-stop |
| 16 | `telemetry` | System | Server -> Bridge | Get bridge health metrics |

### Message Format

Every message follows a common envelope:

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "type": "topic_publish",
  "timestamp": "2026-02-18T12:00:00.000Z",
  "payload": {
    "topic": "/cmd_vel",
    "message_type": "geometry_msgs/msg/Twist",
    "data": {
      "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.3 }
    }
  }
}
```

Response envelope:

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "type": "topic_publish",
  "status": "success",
  "timestamp": "2026-02-18T12:00:00.050Z",
  "payload": {
    "published": true
  }
}
```

### Runtime Validation

Both sides validate messages using schema definitions:

- **TypeScript (MCP Server):** Zod schemas in `Protocol.ts` validate every outgoing command and every incoming response at runtime. Malformed responses from the bridge are caught immediately and surfaced as `ProtocolError` exceptions.
- **Python (Bridge):** `protocol.py` mirrors these definitions and validates incoming commands. Malformed commands from the server are rejected with an error response.

This bilateral validation ensures that protocol drift between the TypeScript and Python codebases is caught at runtime rather than silently causing undefined behavior.

---

## Tool Taxonomy

### Topic Tools

| Tool | Description | Safety Checks |
|------|-------------|---------------|
| `ros2_topic_list` | List all active topics and their message types | None (read-only) |
| `ros2_topic_echo` | Get the latest message(s) on a topic | None (read-only) |
| `ros2_topic_subscribe` | Start a persistent subscription to a topic | None (read-only) |
| `ros2_topic_unsubscribe` | End a topic subscription | None (read-only) |
| `ros2_topic_publish` | Publish a message to a topic | E-stop, blocked topics, velocity limits, rate limits |

### Service Tools

| Tool | Description | Safety Checks |
|------|-------------|---------------|
| `ros2_service_list` | List all available services and their types | None (read-only) |
| `ros2_service_type` | Get the type definition for a service | None (read-only) |
| `ros2_service_call` | Call a ROS2 service with arguments | E-stop, blocked services, rate limits |

### Action Tools

| Tool | Description | Safety Checks |
|------|-------------|---------------|
| `ros2_action_list` | List all available action servers | None (read-only) |
| `ros2_action_send_goal` | Send a goal to an action server | E-stop, blocked actions, geofence, rate limits |
| `ros2_action_cancel` | Cancel an active goal | E-stop (cancel is generally safe but gated by e-stop) |
| `ros2_action_status` | Get the status of a goal | None (read-only) |

### Safety Tools

| Tool | Description | Safety Checks |
|------|-------------|---------------|
| `ros2_e_stop` | Activate or deactivate the emergency stop | None (this IS the safety control) |
| `ros2_get_policy` | View current safety policy | None (read-only) |
| `ros2_set_policy` | Update safety policy at runtime | Validation only (policy must be well-formed) |
| `ros2_get_audit_log` | Query the command audit trail | None (read-only) |

### System Tools

| Tool | Description | Safety Checks |
|------|-------------|---------------|
| `ros2_get_nodes` | List all active ROS2 nodes | None (read-only) |
| `ros2_get_status` | Get bridge connection status and health | None (read-only) |
| `ros2_ping` | Test bridge connectivity | None (diagnostic) |
| `ros2_diagnostics` | Get detailed bridge telemetry | None (read-only) |

---

## Data Flow Walkthrough

This section traces a complete command through the system, from the AI agent's tool call to the ROS2 operation and back.

### Scenario: AI Agent Publishes a Velocity Command

The AI agent wants the robot to move forward at 0.5 m/s.

```
Step 1: AI Agent -> MCP Server (stdio)
==========================================
The AI agent issues a tool call:

  Tool: ros2_topic_publish
  Args: {
    topic: "/cmd_vel",
    message_type: "geometry_msgs/msg/Twist",
    data: {
      linear: { x: 0.5, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 }
    }
  }


Step 2: Tool Handler (MCP Server)
==========================================
The ros2_topic_publish handler:
  a) Validates input parameters with Zod
  b) Calls PolicyEngine.checkPublish("/cmd_vel", data)


Step 3: PolicyEngine Evaluation (MCP Server)
==========================================
  a) E-stop check:       e-stop is NOT active         -> PASS
  b) Blocked topic:      "/cmd_vel" is NOT blocked     -> PASS
  c) Velocity limit:     linear.x = 0.5, max = 1.0    -> PASS
                          angular.z = 0.0, max = 1.5   -> PASS
  d) Rate limit:         3 commands in last second,
                          max = 10                      -> PASS
  e) Geofence:           N/A for velocity commands     -> SKIP

  Decision: ALLOW
  AuditLogger: record ALLOW for /cmd_vel publish


Step 4: WSClient Sends Command (MCP Server -> Bridge)
==========================================
  {
    "id": "a1b2c3d4-...",
    "type": "topic_publish",
    "payload": {
      "topic": "/cmd_vel",
      "message_type": "geometry_msgs/msg/Twist",
      "data": { "linear": { "x": 0.5 }, "angular": { "z": 0.0 } }
    }
  }

  WSClient stores pending request "a1b2c3d4-..." with a Promise.


Step 5: Bridge Receives Command (Python Bridge)
==========================================
  bridge_node.py receives the WebSocket message.
  Routes to topic_handler.py based on type "topic_publish".
  topic_handler creates/reuses a publisher for "/cmd_vel".
  Publishes the Twist message to the ROS2 topic via rclpy.


Step 6: ROS2 Execution
==========================================
  The Twist message propagates via DDS to the robot's
  motor controller node, which applies the velocity.
  The robot begins moving forward at 0.5 m/s.


Step 7: Bridge Sends Response (Bridge -> MCP Server)
==========================================
  {
    "id": "a1b2c3d4-...",
    "type": "topic_publish",
    "status": "success",
    "payload": { "published": true }
  }


Step 8: WSClient Matches Response (MCP Server)
==========================================
  WSClient receives the response.
  Matches "a1b2c3d4-..." to the pending request.
  Resolves the Promise with the response payload.


Step 9: Tool Handler Returns Result (MCP Server -> AI Agent)
==========================================
  The tool handler formats the MCP tool result:

  {
    "content": [{
      "type": "text",
      "text": "Published to /cmd_vel successfully."
    }]
  }

  This is sent to the AI agent over stdio.
```

---

## Deployment Topology

### Development / Simulation

```
+---------------------------+     +-------------------------------+
|  Host Machine             |     |  Docker Network               |
|                           |     |                               |
|  +---------------------+ |     |  +-------------------------+  |
|  | AI Agent (Claude)   | |     |  | ros2-bridge container   |  |
|  |         |           | |     |  |                         |  |
|  |    MCP (stdio)      | |     |  |  bridge_node.py         |  |
|  |         |           | |     |  |       |                 |  |
|  |         v           | |     |  |    rclpy (DDS)          |  |
|  | +------------------+| | WS  |  |       |                 |  |
|  | | MCP Server       ||<-------->|  Port 9090              |  |
|  | | (Node.js)        || |:9090|  +-------------------------+  |
|  | +------------------+| |     |                               |
|  +---------------------+ |     |  +-------------------------+  |
|                           |     |  | sim container           |  |
|                           |     |  |                         |  |
|                           |     |  | Gazebo + TurtleBot3     |  |
|                           |     |  | (shared DDS network)    |  |
|                           |     |  +-------------------------+  |
+---------------------------+     +-------------------------------+
```

### Production / Physical Robot

```
+---------------------------+     +-------------------------------+
|  Operator Workstation     |     |  Robot Computer (e.g., Jetson)|
|                           |     |                               |
|  +---------------------+ |     |  +-------------------------+  |
|  | AI Agent            | |     |  | bridge_node.py          |  |
|  |         |           | |     |  |       |                 |  |
|  |    MCP (stdio)      | |     |  |    rclpy (DDS)          |  |
|  |         v           | | WS  |  |       |                 |  |
|  | +------------------+|<-------->|  Port 9090              |  |
|  | | MCP Server       || |:9090|  +-------------------------+  |
|  | | (Node.js)        || |     |                               |
|  | +------------------+| |     |  +-------------------------+  |
|  +---------------------+ |     |  | ROS2 Nodes              |  |
|                           |     |  | Nav2, MoveIt2, drivers  |  |
|  YAML policy file         |     |  +-------------------------+  |
|  tuned for this robot     |     |                               |
+---------------------------+     +-------------------------------+
```

In production deployments, the YAML policy file is tuned for the specific robot's capabilities --- its maximum safe velocities, operational geofence, permitted topics and services, and rate limits.

---

## Key Design Decisions and Rationale

### 1. Safety Layer in the MCP Server, Not the Bridge

**Decision:** All safety evaluation happens in the TypeScript MCP server process, not in the Python bridge.

**Rationale:** The bridge is a network hop away and may be unavailable. If safety checks lived in the bridge, a bridge crash or network partition would leave the system with no safety enforcement. By placing safety checks in the same process as the tool handlers, the system guarantees that no command can bypass safety --- even if the bridge is down, unreachable, or compromised. A command that fails safety evaluation never reaches the network at all.

### 2. Dual E-Stop (Server + Bridge)

**Decision:** Both the MCP server and the Python bridge maintain independent e-stop states.

**Rationale:** Defense in depth. The server e-stop is the primary control, but the bridge e-stop provides a second line of defense. If a software defect in the MCP server somehow allows a command through despite the server e-stop being active, the bridge e-stop catches it. The two e-stops are independent --- activating one does not require the other to be active, and deactivating one does not deactivate the other.

### 3. YAML Policies for Safety Configuration

**Decision:** Safety policies are defined in YAML files, not in code.

**Rationale:** Different robots have fundamentally different safety requirements. A warehouse AGV might have a max linear velocity of 2.0 m/s, while a surgical robot might be limited to 0.01 m/s. A research drone might need a 3D geofence, while a mobile base needs a 2D one. Hardcoding these values would require code changes and redeployment for every robot. YAML policies allow operators to configure safety parameters without touching source code, and to version-control policies alongside robot-specific configuration.

### 4. Circuit Breaker on the WebSocket Connection

**Decision:** The ConnectionManager uses a circuit breaker pattern to manage connection failures.

**Rationale:** Without a circuit breaker, repeated connection failures cause each tool call to wait for a connection timeout before failing. This degrades the AI agent's experience and wastes resources. The circuit breaker short-circuits these attempts: after a threshold of failures, tool calls fail immediately with a clear error message ("bridge unavailable, circuit open"). The cooldown period prevents the system from hammering a dead endpoint. The half-open state allows the system to automatically recover when the bridge comes back online.

### 5. WebSocket Heartbeat (15s Ping, 30s Stale)

**Decision:** The WSClient sends pings every 15 seconds and considers the connection stale after 30 seconds without a pong.

**Rationale:** TCP keepalive is insufficient for detecting application-level failures (e.g., the bridge process is frozen but the OS is still ACKing TCP packets). Application-level heartbeats ensure that the bridge is not just network-reachable but actually processing messages. The 15/30 second timing balances responsiveness (detecting failures within 30 seconds) against overhead (one ping every 15 seconds is negligible).

### 6. UUID Correlation IDs for Request/Response Matching

**Decision:** Every command is tagged with a UUID, and responses echo that UUID for matching.

**Rationale:** The WebSocket connection carries multiple concurrent commands. Without correlation IDs, the client cannot determine which response corresponds to which request. UUIDs provide globally unique identifiers with no coordination required. The alternative --- sequential request numbering --- would require additional state management and is fragile across reconnections.

### 7. Zod Schemas for Runtime Protocol Validation

**Decision:** All protocol messages are validated at runtime using Zod schemas, not just at compile time via TypeScript types.

**Rationale:** TypeScript's type system is erased at runtime. A message received over the WebSocket is `unknown` at runtime regardless of what the TypeScript type says. Zod schemas provide runtime validation that catches malformed messages, type mismatches, and missing fields before they can cause undefined behavior deeper in the stack. This is particularly important for the protocol boundary between TypeScript and Python, where there is no shared type system.

### 8. Separate TypeScript and Python Codebases

**Decision:** The MCP server is in TypeScript and the ROS2 bridge is in Python, rather than having a single language for both.

**Rationale:** This decision is driven by ecosystem constraints. The MCP ecosystem is TypeScript-native --- the MCP SDK, tooling, and reference implementations are all TypeScript. ROS2's primary client library is `rclpy` (Python), and the vast majority of ROS2 tooling, tutorials, and community packages use Python. Using each ecosystem's native language minimizes friction and maximizes compatibility. The WebSocket protocol boundary is a clean separation point that makes the language difference manageable.

---

*This document describes PhysicalMCP v1.0 architecture. For implementation-level details, refer to the source code in `packages/mcp-server/` and `packages/ros2-bridge/`.*

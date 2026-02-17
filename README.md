# PhysicalMCP

**Safety-first MCP server for ROS2 robots.** Bridge AI agents to physical systems with built-in velocity limits, geofence boundaries, emergency stop, rate limiting, and full audit logging.

```
npm install -g @chinchillaenterprises/physical-mcp
```

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
| ROS2 full coverage (topics/services/actions) | Yes | Yes | Partial |

## Architecture

```
┌─────────────┐     stdio      ┌──────────────────────────────────────┐
│   AI Agent   │◄──────────────►│          MCP Server (TypeScript)     │
│  (Claude,    │    MCP         │                                      │
│   GPT, etc.) │   protocol     │  ┌────────────────────────────────┐  │
└─────────────┘                │  │        Safety Layer             │  │
                               │  │  ┌─────────┐ ┌──────────────┐  │  │
                               │  │  │Velocity │ │  Geofence    │  │  │
                               │  │  │ Limits  │ │  Boundaries  │  │  │
                               │  │  └─────────┘ └──────────────┘  │  │
                               │  │  ┌─────────┐ ┌──────────────┐  │  │
                               │  │  │  Rate   │ │  E-Stop +    │  │  │
                               │  │  │ Limiter │ │  Audit Log   │  │  │
                               │  │  └─────────┘ └──────────────┘  │  │
                               │  └────────────────────────────────┘  │
                               └──────────────┬───────────────────────┘
                                              │ WebSocket (port 9090)
                               ┌──────────────▼───────────────────────┐
                               │       ROS2 Bridge (Python)           │
                               │  rclpy + websockets                  │
                               │  Secondary e-stop as fail-safe       │
                               └──────────────┬───────────────────────┘
                                              │ ROS2 DDS
                               ┌──────────────▼───────────────────────┐
                               │         ROS2 Robot                   │
                               │  Topics, Services, Actions           │
                               └──────────────────────────────────────┘
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
claude mcp add physical-mcp -- npx @chinchillaenterprises/physical-mcp
```

### 3. Talk to your robot

Ask Claude things like:
- "List all ROS2 topics"
- "What's the robot's current pose?"
- "Move the robot forward at 0.1 m/s"
- "Show me the safety status"
- "Activate emergency stop"

## MCP Tools (21)

### Topic Tools

| Tool | Description |
|------|-------------|
| `ros2_topic_list` | List all available topics with their message types |
| `ros2_topic_info` | Get details for a topic (publishers, subscribers) |
| `ros2_topic_subscribe` | Collect N messages from a topic |
| `ros2_topic_publish` | Publish a message to a topic *(safety checked)* |
| `ros2_topic_echo` | Get the latest message from a topic |

### Service Tools

| Tool | Description |
|------|-------------|
| `ros2_service_list` | List all available services |
| `ros2_service_info` | Get service type information |
| `ros2_service_call` | Call a ROS2 service *(safety checked)* |

### Action Tools

| Tool | Description |
|------|-------------|
| `ros2_action_list` | List all action servers |
| `ros2_action_send_goal` | Send a goal to an action server *(safety checked)* |
| `ros2_action_cancel` | Cancel a specific or all active goals |
| `ros2_action_status` | Get status of active goals |

### Safety Tools

| Tool | Description |
|------|-------------|
| `safety_status` | Get full safety system status |
| `safety_emergency_stop` | Activate emergency stop — blocks all commands |
| `safety_emergency_stop_release` | Release e-stop (requires `CONFIRM_RELEASE`) |
| `safety_get_policy` | Get current safety policy configuration |
| `safety_update_velocity_limits` | Adjust velocity limits at runtime |
| `safety_update_geofence` | Adjust geofence boundaries at runtime |
| `safety_audit_log` | Query the command audit trail with filtering |

### System Tools

| Tool | Description |
|------|-------------|
| `system_bridge_status` | Check bridge connection health + latency |
| `system_node_list` | List all ROS2 nodes |

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

### Emergency Stop

Dual-layer emergency stop:

1. **Primary (TypeScript):** Blocks all publish/service/action commands at the MCP server
2. **Secondary (Python):** Bridge-level e-stop publishes zero velocity to `/cmd_vel` and cancels all active action goals

Releasing requires explicit confirmation (`CONFIRM_RELEASE`) to prevent accidental resumption.

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
      "args": ["@chinchillaenterprises/physical-mcp"],
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

## License

MIT - see [LICENSE](LICENSE) for details.

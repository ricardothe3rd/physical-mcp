# MCP-ROS2 Bridges Compared: PhysicalMCP vs robotmcp vs wise-vision-mcp

*When AI meets real hardware, what stands between a helpful command and a costly mistake?*

---

The Model Context Protocol (MCP) has opened a compelling new frontier: letting AI agents interact with physical robots through ROS2. Several open-source projects now bridge MCP to ROS2, and each makes different tradeoffs. This article compares the three most notable options -- PhysicalMCP, robotmcp, and wise-vision-mcp -- to help you choose the right tool for your use case.

## TL;DR

- **robotmcp** is the most popular MCP-ROS2 bridge with strong community adoption. It covers topics, services, and actions, but provides no safety layer. Best for simulation and hobby use.
- **wise-vision-mcp** is a lighter alternative with partial ROS2 coverage. Good for quick experiments with topics and basic services.
- **PhysicalMCP** adds a full safety layer (velocity limits, geofence, e-stop, rate limiting, audit logging) on top of complete ROS2 coverage. Designed for scenarios where commands reach real hardware.

## Feature Comparison

| Feature | PhysicalMCP | robotmcp | wise-vision-mcp |
|---|:---:|:---:|:---:|
| **ROS2 Topics** (list, subscribe, publish, echo) | Yes | Yes | Yes |
| **ROS2 Services** (list, info, call) | Yes | Yes | Partial |
| **ROS2 Actions** (send goal, cancel, status) | Yes | Yes | Limited |
| **Total MCP Tools** | 21 | -- | -- |
| | | | |
| **Velocity Limits** | Yes | No | No |
| **Geofence Boundaries** | Yes | No | No |
| **Emergency Stop (E-Stop)** | Yes (dual-layer) | No | No |
| **Rate Limiting** | Yes | No | No |
| **Blocked Topics/Services** | Yes | No | No |
| **Audit Logging** | Yes | No | No |
| **YAML Safety Policies** | Yes | No | No |
| **Runtime Policy Updates** | Yes | No | No |
| | | | |
| **Architecture** | TypeScript MCP + Python ROS2 bridge | Direct ROS2 integration | Direct ROS2 integration |
| **Docker + Simulation** | Yes (Gazebo + TurtleBot3) | Varies | Varies |
| **Unit Tests** | 75 | -- | -- |
| **License** | MIT | -- | -- |
| **Community Size** | Newer | ~1K GitHub stars | ~65 GitHub stars |

*Note: Cells marked "--" indicate information that was not verified at the time of writing. Absence of a listed feature does not necessarily mean it cannot be added by users.*

## Why Safety Matters

In simulation, a bad command is a minor inconvenience. You reset the environment and try again. With real hardware, the stakes change fundamentally.

### The cost of an unchecked command

Consider what happens when an AI agent sends a velocity command to a real robot without safety enforcement:

**Scenario 1: Velocity overshoot.** An LLM decides to set linear velocity to 5.0 m/s on a TurtleBot3 whose maximum safe speed is 0.22 m/s. The command is technically valid ROS2 -- the topic accepts any float -- but the robot lurches forward at an unsafe speed, potentially falling off a table, crashing into equipment, or injuring a person nearby.

**Scenario 2: Runaway loop.** An AI agent enters a feedback loop, publishing velocity commands at hundreds of Hz. The ROS2 graph becomes saturated, other nodes starve for bandwidth, and the robot's behavior becomes unpredictable.

**Scenario 3: Dangerous service call.** The agent, exploring available services, calls `/shutdown` or `/kill` on a navigation stack mid-operation. The robot loses localization while moving and collides with an obstacle.

**Scenario 4: No way to stop.** Something goes wrong and you need the robot to stop immediately. Without a software e-stop that blocks all commands at the MCP layer, you are relying on physical access to the robot's power switch -- which may not be immediately reachable.

These are not hypothetical edge cases. They are routine failure modes when bridging language models to physical systems. LLMs hallucinate, misinterpret units, retry failed commands aggressively, and explore APIs by calling whatever is available.

### What real hardware costs

A TurtleBot3 runs about $600. A TurtleBot4 is around $1,800. Industrial mobile robots start at $20,000 and scale well into six figures. A robotic arm in a research lab can easily cost $50,000+. The equipment around the robot -- sensors, workstations, other robots -- adds to the potential damage.

Beyond hardware cost, there is the question of liability. If an AI agent commands a robot to move in an unsafe way and someone is injured, the question "what safety measures were in place?" will be asked. "We used a passthrough bridge with no safety checks" is not a strong answer.

### The gap in the ecosystem

Every industrial robot controller ships with safety features: velocity limits, workspace boundaries, emergency stop, and logging. These are not optional extras -- they are requirements for operating around people. The MCP-ROS2 ecosystem currently lacks this layer. PhysicalMCP was built specifically to fill that gap.

## Architecture Comparison

### robotmcp

robotmcp connects MCP clients to ROS2 directly. Its strength is simplicity: fewer moving parts, straightforward setup, and a large community that has validated its ROS2 coverage. Commands from the AI agent flow through to ROS2 with minimal transformation.

```
AI Agent  -->  MCP  -->  robotmcp  -->  ROS2
```

This architecture is excellent for getting started quickly, for simulation-only workflows, and for environments where safety is handled at another layer (e.g., hardware safety controllers on the robot itself).

### wise-vision-mcp

wise-vision-mcp follows a similar direct-bridge pattern with a lighter footprint. It covers topics and some services, making it a good fit for monitoring and simple interactions.

```
AI Agent  -->  MCP  -->  wise-vision-mcp  -->  ROS2
```

### PhysicalMCP

PhysicalMCP introduces a policy engine between the MCP interface and the ROS2 bridge. Every publish, service call, and action goal passes through the safety layer before reaching the robot.

```
AI Agent  -->  MCP  -->  PhysicalMCP (TypeScript)
                              |
                         Safety Layer
                         - Velocity check
                         - Geofence check
                         - Rate limit check
                         - Blocked topic/service check
                         - E-stop check
                         - Audit log
                              |
                         WebSocket (port 9090)
                              |
                         ROS2 Bridge (Python)
                         - Secondary e-stop
                              |
                         ROS2 DDS  -->  Robot
```

The two-process design (TypeScript MCP server + Python ROS2 bridge) means the safety layer runs independently of the ROS2 runtime. Even if the bridge process has issues, the MCP server continues to enforce policy. The bridge also maintains its own secondary emergency stop as an additional fail-safe.

## Safety Features in Detail

### Velocity Limits

PhysicalMCP checks every `geometry_msgs/msg/Twist` message against configurable velocity limits before it reaches the robot. The check computes the actual speed magnitude (not just individual axis values) and blocks the entire command if any limit is exceeded.

```yaml
# turtlebot3.yaml - tuned to the robot's actual hardware limits
velocity:
  linearMax: 0.22    # TurtleBot3 Burger max is 0.22 m/s
  angularMax: 2.84   # TurtleBot3 Burger max is 2.84 rad/s
```

When a velocity violation is detected:

```
You: "Move forward at 2 m/s"

> SAFETY BLOCKED: Publish to /cmd_vel denied.
>
> Violations:
> - [velocity_exceeded] Linear velocity 2.00 m/s exceeds limit of 0.22 m/s
```

The command never reaches the robot. With robotmcp or wise-vision-mcp, that 2 m/s command would be sent directly to `/cmd_vel`.

### Geofence Boundaries

Define the 3D workspace the robot is allowed to operate within:

```yaml
geofence:
  xMin: -10.0
  xMax: 10.0
  yMin: -10.0
  yMax: 10.0
  zMin: 0.0
  zMax: 0.5
```

Navigation goals or position commands outside these boundaries are blocked. This is especially important for robots operating in shared spaces where certain areas are off-limits.

### Rate Limiting

Prevents an AI agent from flooding the ROS2 graph:

```yaml
rateLimits:
  publishHz: 10          # max publishes per second
  servicePerMinute: 60   # max service calls per minute
  actionPerMinute: 30    # max action goals per minute
```

LLMs can be aggressive retriers. If a command fails, some agents will retry rapidly. Rate limiting ensures that even pathological retry behavior cannot overwhelm the robot's communication bus.

### Blocked Topics and Services

Certain ROS2 interfaces should never be accessible to an AI agent:

```yaml
blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
  - /delete_entity
```

This is a simple allowlist/blocklist mechanism, but it prevents the most dangerous category of mistakes: an AI agent exploring the ROS2 graph and calling destructive services out of curiosity.

### Emergency Stop (Dual-Layer)

PhysicalMCP implements e-stop at two independent layers:

1. **Primary (TypeScript MCP server):** When activated, the policy engine blocks ALL publish, service, and action commands. No exceptions. Even a perfectly safe 0.01 m/s velocity command is rejected while e-stop is active.

2. **Secondary (Python ROS2 bridge):** The bridge publishes zero velocity to `/cmd_vel` and cancels all active action goals. This acts as an independent fail-safe -- if the MCP server has issues, the bridge can still halt the robot.

Releasing e-stop requires the explicit confirmation string `CONFIRM_RELEASE`:

```
You: "Activate emergency stop"
> EMERGENCY STOP ACTIVATED. All commands blocked.

You: "Move forward slowly"
> SAFETY BLOCKED: Emergency stop is active.

You: "Release emergency stop"
> Confirmation required. Call with confirmation: "CONFIRM_RELEASE"

You: "Release emergency stop with confirmation CONFIRM_RELEASE"
> Emergency stop released. Commands allowed.
```

This deliberate friction prevents accidental resumption. Neither robotmcp nor wise-vision-mcp offer any form of software e-stop through MCP.

### Audit Logging

Every command that passes through PhysicalMCP is recorded:

```json
{
  "id": "a1b2c3",
  "timestamp": 1708300000000,
  "command": "publish",
  "target": "/cmd_vel",
  "params": { "linear": { "x": 5.0 }, "angular": { "z": 0.0 } },
  "safetyResult": {
    "allowed": false,
    "violations": [
      {
        "type": "velocity_exceeded",
        "message": "Linear velocity 5.00 m/s exceeds limit of 0.22 m/s"
      }
    ]
  }
}
```

The audit log captures both allowed and blocked commands, making it possible to:
- Review what an AI agent actually did during a session
- Debug why a command was blocked
- Build a compliance record for safety reviews
- Identify patterns of misuse or misconfiguration

Query the log with filtering:

```
You: "Show me only blocked commands from the audit log"
> Audit Log (12 total, 4 blocked, 0 errors)
> [filtered to violations only...]
```

## YAML Safety Policies

PhysicalMCP ships with two built-in policies and supports custom policies via the `PHYSICAL_MCP_POLICY` environment variable.

### Default policy (conservative)

```yaml
name: default
description: Conservative default safety policy for any ROS2 robot

velocity:
  linearMax: 0.5    # m/s
  angularMax: 1.5   # rad/s

geofence:
  xMin: -5.0
  xMax: 5.0
  yMin: -5.0
  yMax: 5.0
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

### TurtleBot3 policy (hardware-matched)

```yaml
name: turtlebot3
description: Safety policy tuned for TurtleBot3 in Gazebo simulation

velocity:
  linearMax: 0.22    # TurtleBot3 Burger max is 0.22 m/s
  angularMax: 2.84   # TurtleBot3 Burger max is 2.84 rad/s

geofence:
  xMin: -10.0
  xMax: 10.0
  yMin: -10.0
  yMax: 10.0
  zMin: 0.0
  zMax: 0.5

rateLimits:
  publishHz: 30
  servicePerMinute: 120
  actionPerMinute: 60

blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
  - /delete_entity
```

Partial policies are supported -- omit any field and it falls back to defaults. Policies can also be updated at runtime via the `safety_update_velocity_limits` and `safety_update_geofence` MCP tools, allowing dynamic adjustment without restarting the server.

## When to Use Each Tool

### Use robotmcp when:

- You are working **exclusively in simulation** and safety is not a concern
- You want the **largest community** and the most battle-tested ROS2 coverage
- You need a **simple, minimal setup** with no additional configuration
- Safety is handled at another layer (e.g., hardware safety controllers on the physical robot, or a separate safety node in your ROS2 graph)
- You are building a **demo or proof of concept** and speed of integration matters most

robotmcp has earned its popularity. It works, it covers the ROS2 API well, and its community means you are more likely to find answers to questions. For pure simulation workflows, it is a solid choice.

### Use wise-vision-mcp when:

- You need a **lightweight bridge** for monitoring or simple topic interactions
- Your use case is limited to **reading sensor data** and **publishing to a few topics**
- You want a **smaller dependency footprint**
- You are getting started with MCP-ROS2 integration and want something straightforward

wise-vision-mcp fills a useful niche for lighter workloads where full action support is not required.

### Use PhysicalMCP when:

- Your AI agent's commands will reach **real physical hardware**
- You need **enforceable velocity limits** matched to your robot's specifications
- You need **geofence boundaries** to constrain the robot's workspace
- You want a **software emergency stop** accessible through the AI interface
- You need an **audit trail** of every command for debugging or compliance
- You are building a system that will be **demonstrated to stakeholders** who will ask about safety
- You are in a **research lab, warehouse, or any shared space** where a runaway robot is unacceptable
- You want to **configure different safety policies** for different robots or environments

## Getting Started with PhysicalMCP

### Quick start (simulation)

```bash
# 1. Start the simulation + ROS2 bridge
cd docker
docker compose up

# 2. Add PhysicalMCP to Claude Code
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp

# 3. Start talking to your robot
# "List all ROS2 topics"
# "Move the robot forward at 0.1 m/s"
# "Show me the safety status"
```

### Custom safety policy

```bash
# Point to your own policy file
PHYSICAL_MCP_POLICY=/path/to/my-robot.yaml npx @ricardothe3rd/physical-mcp
```

### Claude Desktop configuration

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

## Conclusion

The MCP-ROS2 ecosystem is young and growing fast. robotmcp deserves credit for pioneering the space and building the largest community. wise-vision-mcp provides a useful lightweight alternative. Both are good tools for their intended use cases.

But the ecosystem has a gap: none of the existing bridges enforce safety at the MCP layer. When you move from simulation to real hardware -- when an AI agent's commands translate to physical motion in a space shared with people and expensive equipment -- that gap becomes the most important problem to solve.

PhysicalMCP exists to fill that gap. It does not replace robotmcp's community or wise-vision-mcp's simplicity. It adds the safety layer that the ecosystem needs before MCP-controlled robots can move from demos to production.

Safety is not a feature you add later. It is the foundation you build on.

---

**Links:**
- [PhysicalMCP](https://github.com/ricardothe3rd/physical-mcp) -- Safety-first MCP server for ROS2 robots
- [npm: @ricardothe3rd/physical-mcp](https://www.npmjs.com/package/@ricardothe3rd/physical-mcp)

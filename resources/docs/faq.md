# PhysicalMCP FAQ

Frequently asked questions about PhysicalMCP -- the safety-first MCP server for ROS2 robots.

---

## Table of Contents

- [General](#general)
- [Installation](#installation)
- [Safety](#safety)
- [Docker and Simulation](#docker-and-simulation)
- [Bridge Connection](#bridge-connection)
- [Supported Robots](#supported-robots)
- [Claude Desktop and MCP Client Integration](#claude-desktop-and-mcp-client-integration)
- [Troubleshooting](#troubleshooting)
- [Security](#security)
- [Contributing](#contributing)
- [Roadmap](#roadmap)

---

## General

### 1. What is PhysicalMCP?

PhysicalMCP is an open-source MCP (Model Context Protocol) server that bridges AI agents to physical robots running ROS2. It is built around a safety-first philosophy: every command an AI agent sends -- whether a topic publish, service call, or action goal -- passes through a configurable safety layer before it can reach the robot. The safety layer enforces velocity limits, geofence boundaries, rate limiting, blocked topics/services, emergency stop, deadman switch, acceleration limits, and full audit logging.

The npm package is [`@ricardothe3rd/physical-mcp`](https://www.npmjs.com/package/@ricardothe3rd/physical-mcp). The source code is on GitHub at [github.com/ricardothe3rd/physical-mcp](https://github.com/ricardothe3rd/physical-mcp).

### 2. How is PhysicalMCP different from robotmcp and other MCP-ROS2 bridges?

Other MCP-ROS2 bridges such as robotmcp and wise-vision-mcp pass commands directly from the AI agent to ROS2 with no safety checks. This works fine for simulation, but it is dangerous for real hardware. An AI agent can hallucinate velocity values, retry failed commands aggressively, or call destructive services out of curiosity.

PhysicalMCP adds a full safety layer that evaluates every command before it reaches the robot:

| Feature | PhysicalMCP | robotmcp | wise-vision-mcp |
|---------|:-----------:|:--------:|:----------------:|
| Velocity limits | Yes | No | No |
| Geofence boundaries | Yes | No | No |
| Emergency stop | Yes (dual-layer) | No | No |
| Rate limiting | Yes | No | No |
| Blocked topics/services | Yes | No | No |
| Audit logging | Yes | No | No |
| YAML safety policies | Yes | No | No |
| Deadman switch | Yes | No | No |

For a full comparison, see [resources/docs/comparison.md](comparison.md).

### 3. What are the 27 tools PhysicalMCP provides?

The tools are organized into five categories:

**Topic Tools (5):** `ros2_topic_list`, `ros2_topic_info`, `ros2_topic_subscribe`, `ros2_topic_publish`, `ros2_topic_echo`

**Service Tools (3):** `ros2_service_list`, `ros2_service_info`, `ros2_service_call`

**Action Tools (4):** `ros2_action_list`, `ros2_action_send_goal`, `ros2_action_cancel`, `ros2_action_status`

**Safety Tools (9):** `safety_status`, `safety_emergency_stop`, `safety_emergency_stop_release`, `safety_get_policy`, `safety_update_velocity_limits`, `safety_update_geofence`, `safety_audit_log`, `safety_set_clamp_mode`, `safety_deadman_switch`, `safety_heartbeat`

**System Tools (2):** `system_bridge_status`, `system_node_list`

Tools that can affect the robot (publish, service call, action goal) are safety-checked. Read-only tools (list, echo, status) are not gated because they cannot cause physical motion.

---

## Installation

### 4. How do I install PhysicalMCP?

The fastest way is via npx -- no global install needed:

```bash
npx @ricardothe3rd/physical-mcp
```

To add it to Claude Code:

```bash
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp
```

To install globally:

```bash
npm install -g @ricardothe3rd/physical-mcp
```

**Requirements:**
- Node.js >= 18 (for the MCP server)
- Python >= 3.10 and ROS2 Humble (for the bridge, which runs on the robot or a machine with ROS2 access)
- Docker + Docker Compose (optional, for the included Gazebo simulation)

### 5. Do I need ROS2 installed on my development machine?

No. ROS2 is only required on the machine running the Python bridge, which connects to the robot's ROS2 network. The TypeScript MCP server (the part your AI client talks to) does not require ROS2. In the typical Docker-based setup, ROS2 runs inside the containers and you only need Node.js on your host machine.

### 6. What versions of ROS2 are supported?

PhysicalMCP's Python bridge is tested against **ROS2 Humble** (the current LTS release, supported through 2027). Other ROS2 distributions (Iron, Jazzy) may work but have not been officially validated. If you test with another distribution and encounter issues, please open a GitHub issue.

---

## Safety

### 7. Can the safety layer be bypassed?

Not through the MCP interface. All publish, service call, and action goal commands pass through the PolicyEngine in the TypeScript MCP server. There is no `--skip-safety` flag, no backdoor tool, and no way to send a command to the bridge without it being evaluated by the safety layer first.

The safety architecture is enforced structurally: the PolicyEngine sits in the only code path between tool handlers and the WebSocket client. The Python bridge has a secondary emergency stop as an additional fail-safe.

That said, PhysicalMCP's safety layer is a guardrail, not a security boundary. If someone has direct access to the ROS2 network (e.g., via `ros2 topic pub` on the robot), they can bypass the MCP server entirely. The safety layer protects against accidental damage from AI agents, not from adversarial users with system access. See [SECURITY.md](../../SECURITY.md) for the full security scope.

### 8. What happens when emergency stop is activated?

When e-stop is activated:

1. **Primary e-stop (TypeScript MCP server):** The PolicyEngine immediately blocks ALL publish, service, and action commands. No exceptions. Even a perfectly safe 0.01 m/s velocity command is rejected.
2. **Secondary e-stop (Python bridge):** The bridge publishes zero velocity to `/cmd_vel` and cancels all active action goals.

Every command attempted while e-stop is active returns a clear error: "Emergency stop is active. Release e-stop before publishing."

Releasing e-stop requires the explicit confirmation string `CONFIRM_RELEASE` to prevent accidental resumption:

```
"Release emergency stop with confirmation CONFIRM_RELEASE"
```

### 9. What is the deadman switch and when should I use it?

The deadman switch auto-activates the emergency stop if no heartbeat is received within a configurable timeout. It prevents a robot from continuing to execute commands if the AI agent disconnects, the network drops, or the MCP server crashes.

**When to use it:**
- Any time your commands reach real hardware
- Autonomous operations without continuous human oversight
- Network-connected setups where connectivity might be interrupted

**Configuration:**

```yaml
deadmanSwitch:
  enabled: true
  timeoutMs: 30000   # 30 seconds
```

The AI agent (or MCP client) must call `safety_heartbeat` periodically to prevent the deadman switch from triggering.

### 10. What is the difference between block mode and clamp mode for velocity limits?

**Block mode** (`clampMode: false`, the default): Commands exceeding velocity limits are rejected outright. The command never reaches the robot. The AI agent receives an error explaining the violation.

**Clamp mode** (`clampMode: true`): Commands exceeding velocity limits are scaled down to the maximum while preserving direction. The command is sent to the robot at the reduced speed, and a warning is included in the response.

**Recommendation:** Use block mode for real hardware (you want the AI to know when it is trying to go too fast). Use clamp mode in simulation or when the limits are set to the robot's hardware maximums (where clamping cannot cause damage).

---

## Docker and Simulation

### 11. How do I run the included simulation?

The Docker setup launches a Gazebo simulation with a TurtleBot3 Burger robot and the ROS2 bridge:

```bash
cd docker
docker compose up
```

The first build takes 10-15 minutes (downloading ROS2 Humble, Gazebo, and TurtleBot3 packages). Subsequent starts use cached layers and are fast.

Once running, the bridge listens on `ws://localhost:9090`. Point your MCP server at it:

```bash
npx @ricardothe3rd/physical-mcp
```

The default bridge URL is already `ws://localhost:9090`, so no extra configuration is needed.

**Requirements:** Docker Engine 20.10+ and Docker Compose v2. Approximately 5 GB of disk space for images.

For Gazebo GUI display, see the [Docker guide](docker-guide.md) for X11 forwarding instructions. The simulation works headlessly without X11 -- all ROS2 topics function normally even without the visual window.

### 12. Can I run the simulation headless (no GUI)?

Yes. Set the `DISPLAY` variable to empty:

```bash
DISPLAY= docker compose up
```

Gazebo runs in server-only mode. The physics simulation still operates and all ROS2 topics (`/cmd_vel`, `/odom`, `/scan`, etc.) are published normally. You simply do not see the 3D rendering. This is useful for CI/CD environments, remote servers, or machines without a display.

---

## Bridge Connection

### 13. What happens if the WebSocket connection between the MCP server and bridge drops?

The connection manager handles disconnections automatically:

1. **Immediate impact:** All pending requests are rejected with a clear error message. Any tool call that requires bridge communication returns "Bridge not connected."
2. **Reconnection:** The connection manager attempts reconnection every 5 seconds with exponential backoff.
3. **Circuit breaker:** After 5 consecutive connection failures, the circuit breaker opens. While open, tool calls fail immediately without waiting for a connection attempt. After a 30-second cooldown, the circuit breaker allows a single probe request to check if the bridge is back.
4. **Safety impact:** If the deadman switch is enabled, loss of connection means the heartbeat stops, which triggers the deadman switch timeout and activates the emergency stop on the bridge side.

The bridge-side secondary e-stop operates independently. If the bridge was told to activate e-stop before the connection dropped, it remains active regardless of the MCP server's state.

### 14. Can I connect to a bridge running on a remote machine?

Yes. Set the `PHYSICAL_MCP_BRIDGE_URL` environment variable to the remote bridge's address:

```bash
PHYSICAL_MCP_BRIDGE_URL=ws://192.168.1.105:9090 npx @ricardothe3rd/physical-mcp
```

Ensure port 9090 (TCP) is open between the two machines. For remote deployments outside a local network, use SSH tunneling or a VPN. The default WebSocket transport is unencrypted (`ws://`), so avoid exposing port 9090 over the public internet without additional security measures.

---

## Supported Robots

### 15. What robots does PhysicalMCP support?

PhysicalMCP works with **any robot that runs ROS2**. It connects to the robot through the standard ROS2 topic, service, and action interfaces. If your robot has a ROS2 driver that publishes topics and exposes services, PhysicalMCP can interact with it.

The included simulation uses TurtleBot3 Burger, and a pre-tuned safety policy is provided for it. Community-contributed safety policies for other robots are welcome.

**Robots tested or with included policy profiles:**

- TurtleBot3 Burger / Waffle / Waffle Pi
- TurtleBot4

**Robots on the roadmap for future policy profiles:**

- Universal Robots (UR3/UR5/UR10)
- Boston Dynamics Spot
- Franka Emika Panda
- Clearpath Jackal / Husky
- DJI drones (via ROS2 bridge)

**Custom robots:** Any ROS2 robot works. Create a YAML safety policy tailored to your robot's velocity limits, workspace, and available topics/services. See the [safety policy authoring guide](safety-policy-guide.md) for instructions.

### 16. Does PhysicalMCP support robot arms and manipulators?

Yes, with some limitations. PhysicalMCP's velocity limits apply to end-effector velocity commands published via `geometry_msgs/msg/Twist` messages. Joint-level velocity limits (per-joint speed constraints) are not yet natively supported; for those, you should rely on your robot arm's own joint limit enforcement in its ROS2 driver or controller.

The geofence feature is particularly valuable for robot arms, as it constrains the workspace to prevent collisions with the robot's base, the table, or nearby objects.

**Recommended settings for robot arms:**
- Very low velocity limits (0.01-0.05 m/s linear)
- Tight geofence matching the arm's safe workspace
- Deadman switch enabled with a short timeout (5-10 seconds)
- Block mode (not clamp mode)

---

## Claude Desktop and MCP Client Integration

### 17. How do I add PhysicalMCP to Claude Desktop?

Add the following to your `claude_desktop_config.json`:

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

Both environment variables are optional:
- `PHYSICAL_MCP_BRIDGE_URL` defaults to `ws://localhost:9090`.
- `PHYSICAL_MCP_POLICY` defaults to the built-in conservative policy. Set it to a file path if you have a custom policy.

### 18. How do I add PhysicalMCP to Claude Code?

```bash
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp
```

Set environment variables in your shell before starting Claude Code, or use the `--env` flag:

```bash
export PHYSICAL_MCP_BRIDGE_URL=ws://localhost:9090
export PHYSICAL_MCP_POLICY=/path/to/my-policy.yaml
```

### 19. Does PhysicalMCP work with AI clients other than Claude?

Yes. PhysicalMCP is a standard MCP server. It works with **any MCP-compatible client**, including Claude Desktop, Claude Code, GPT-based agents using MCP adapters, and local LLMs running through MCP-compatible frameworks. The safety layer enforces the same policies regardless of which AI sends the commands.

See the [Local LLM Guide](local-llm-guide.md) for detailed instructions on using PhysicalMCP with Ollama, llama.cpp, vLLM, and other local LLM setups.

---

## Troubleshooting

### 20. The bridge is not connecting. What should I check?

If you see "Bridge not connected. Start the ROS2 bridge and try again":

1. **Is the bridge process running?**
   - Docker: `docker compose ps` in the `docker/` directory. Both `sim` and `bridge` should show "Up."
   - Local: Verify `physical-mcp-bridge` is running in a terminal with ROS2 sourced.

2. **Is port 9090 reachable?**
   - From the MCP server machine: `curl -s http://localhost:9090` (it will fail with a WebSocket error, but if it connects at all, the port is reachable). If the connection is refused, the bridge is not listening.
   - Check for firewall rules blocking port 9090.

3. **Is the bridge URL correct?**
   - Verify `PHYSICAL_MCP_BRIDGE_URL` matches where the bridge is actually running. If the bridge is in Docker with `network_mode: host`, it is on `localhost:9090`. If on a remote machine, use that machine's IP.

4. **Is another process using port 9090?**
   - Run `lsof -i :9090` to check. A previous bridge instance or `rosbridge_suite` might be occupying the port.

### 21. `ros2_topic_list` returns empty or is missing expected topics.

This usually means the bridge started before the simulation was fully loaded. In Docker, the bridge container starts as soon as the simulation container's process launches, but Gazebo takes additional time to load the world and spawn the robot model.

**Fix:** Restart the bridge after the simulation is fully loaded:

```bash
docker compose restart bridge
```

If running against real hardware, verify that the robot's ROS2 driver is running by checking `ros2 topic list` directly on the robot.

### 22. I am getting rate limit errors during normal use.

The default policy limits publishes to 10 Hz. If you are sending commands faster than that, either:

1. Increase the limit in your YAML policy:

   ```yaml
   rateLimits:
     publishHz: 30
   ```

2. Or update the limit at runtime using the `safety_update_velocity_limits` tool (note: rate limit updates may require a safety policy change via the YAML file).

For AI agent conversations, 10 Hz is usually more than sufficient. Rate limit errors during normal conversational use may indicate the AI agent is retrying commands too aggressively.

---

## Security

### 23. Is PhysicalMCP secure for production use?

PhysicalMCP's safety layer is designed to prevent **accidental damage from AI agents**, not to resist adversarial attacks from users with system access. Key security considerations:

- **The WebSocket transport is unencrypted by default** (`ws://`). For deployments beyond localhost, use TLS (`wss://`), SSH tunnels, or a VPN.
- **There is no authentication** on the WebSocket connection. Access control must be enforced at the network level (firewall rules, binding to localhost, VPN).
- **Anyone with access to the ROS2 network can bypass the MCP server** entirely by publishing to ROS2 topics directly.

For production deployments, we recommend:
- Running the bridge bound to localhost or a private interface
- Using firewall rules to restrict access to port 9090
- Enabling the deadman switch
- Using allowlists instead of blocklists for topics and services
- Reviewing audit logs regularly

Authentication, TLS, and role-based access control are planned for future versions. See the [roadmap](roadmap.md) for details.

---

## Contributing

### 24. How can I contribute to PhysicalMCP?

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for full details. The most impactful contributions right now:

- **Robot-specific safety policies:** YAML configs for popular robots (UR5, Franka, Spot, Jackal, Husky). If you have access to a robot and can determine its safe velocity limits and workspace, a contributed policy file is extremely valuable.
- **Testing:** Edge cases, integration tests, and performance benchmarks. Safety-related changes must include tests for valid commands, blocked commands, and boundary conditions.
- **Documentation:** Tutorials, guides, and translations.
- **Bridge features:** TF2 support, parameter handling, diagnostics.
- **Additional safety checks:** Collision avoidance, joint limits, workspace monitoring.

**Process:**
1. Fork the repo and create a branch from `main`.
2. Make your changes.
3. Ensure `npm run build` and `npm test` pass.
4. Submit a PR with a clear description.

### 25. Where do I report bugs or request features?

- **Bugs:** Open a GitHub issue at [github.com/ricardothe3rd/physical-mcp/issues](https://github.com/ricardothe3rd/physical-mcp/issues). Include steps to reproduce, your environment (OS, Node.js version, ROS2 distribution), and any error logs.
- **Feature requests:** Open a GitHub issue describing the use case and proposed solution.
- **Security vulnerabilities:** Do NOT open a public issue. See [SECURITY.md](../../SECURITY.md) for responsible disclosure instructions.

---

## Roadmap

### 26. What is planned for future releases?

**v0.2.0 -- Hardening** (near-term):
- Workspace zones with different speed limits per zone
- Command queue introspection
- Safety event webhooks
- 90%+ code coverage
- Integration test suite

**v0.3.0 -- Multi-Robot and Observability:**
- Named robot connections (multiple bridges simultaneously)
- Per-robot policies
- Structured logging, Prometheus metrics, OpenTelemetry traces
- Streaming topic subscriptions via MCP resources

**v0.4.0 -- Cloud Relay (SaaS):**
- Hosted WebSocket relay for connecting to robots from anywhere
- Authentication (API key / OAuth2), TLS encryption
- Web dashboard with real-time robot status and safety event timeline

**v0.5.0 -- Enterprise and Safety Certification:**
- SSO/SAML, role-based access control
- Compliance export for regulatory submissions
- IEC 61508 / ISO 13849 alignment documentation
- Collision avoidance integration, force/torque limits

**v1.0.0 -- Production Ready:**
- Policy marketplace for community-contributed safety policies
- Plugin system for custom safety checks
- Formal support for TurtleBot4, UR robots, Spot, Franka, Clearpath, DJI drones

See the full [roadmap](roadmap.md) for details and timelines.

### 27. How can I influence the roadmap?

1. **Star the repo** on GitHub -- shows demand and helps prioritize.
2. **Open an issue** -- feature requests, bug reports, and questions all inform priorities.
3. **Comment on the roadmap** -- tell us what you need most.
4. **Contribute** -- PRs for safety policies, tests, and documentation are especially welcome.
5. **Share your use case** -- real-world deployment stories help prioritize the right features.

---

*Have a question not covered here? Open an issue on [GitHub](https://github.com/ricardothe3rd/physical-mcp/issues) or check the other documentation in `resources/docs/`.*

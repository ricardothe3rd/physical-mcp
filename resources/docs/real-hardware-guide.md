# Connecting PhysicalMCP to Real Hardware

This guide walks you through connecting PhysicalMCP to a physical robot running ROS2. It covers installation, configuration, safety setup, and operational best practices.

---

> **WARNING: Always test in simulation first.**
>
> Before connecting to real hardware, run your exact workflow in the Gazebo simulation (`docker compose up`) and verify that:
>
> 1. Your safety policy blocks dangerous commands as expected
> 2. Velocity limits are correctly enforced
> 3. Emergency stop activates and releases properly
> 4. The AI agent's behavior is predictable and safe
>
> Real hardware can cause property damage, injury, or worse. PhysicalMCP's safety layer reduces risk significantly, but it is not a substitute for careful testing and human oversight.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Step 1: Install the Python Bridge](#step-1-install-the-python-bridge)
- [Step 2: Start the Bridge](#step-2-start-the-bridge)
- [Step 3: Configure a Safety Policy](#step-3-configure-a-safety-policy)
- [Step 4: Point the MCP Server at the Bridge](#step-4-point-the-mcp-server-at-the-bridge)
- [Step 5: Test with Read-Only Tools First](#step-5-test-with-read-only-tools-first)
- [Safety Checklist Before Real Hardware](#safety-checklist-before-real-hardware)
- [Network Considerations](#network-considerations)
- [Known Robot Platform Tips](#known-robot-platform-tips)
- [Operational Procedures](#operational-procedures)

---

## Prerequisites

Before connecting to real hardware, you need:

1. **ROS2 Humble installed** on the robot's computer (or on a machine with network access to the robot's ROS2 DDS network). PhysicalMCP's bridge is tested against ROS2 Humble. Other ROS2 distributions (Iron, Jazzy) may work but are not officially supported.

2. **Python 3.10 or later** on the machine where the bridge will run.

3. **A robot with a working ROS2 driver.** The robot must already publish topics, offer services, or expose action servers through ROS2. PhysicalMCP does not replace robot drivers; it provides an AI-accessible interface to an existing ROS2 system.

4. **Node.js 18 or later** on the machine where the MCP server will run (this can be a different machine from the robot).

5. **Network connectivity** between the MCP server machine and the bridge machine (see [Network Considerations](#network-considerations)).

---

## Step 1: Install the Python Bridge

The bridge must run on a machine that has both ROS2 and network access to the robot. This is typically one of:

- **The robot's onboard computer** (Jetson, Raspberry Pi, Intel NUC, etc.)
- **A workstation on the same LAN** that is part of the robot's ROS2 DDS network

Install the bridge package:

```bash
cd packages/ros2-bridge
pip install -e .
```

This installs the `physical-mcp-bridge` command-line tool. The only Python dependency is `websockets>=12.0`. The bridge also requires `rclpy`, which is provided by your ROS2 installation.

Verify the installation:

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash

# Check that the bridge command is available
physical-mcp-bridge --help
```

If `physical-mcp-bridge` is not found after installation, ensure that the pip install location is on your `PATH`. On some systems, `pip install --user` places binaries in `~/.local/bin/`.

---

## Step 2: Start the Bridge

Source your ROS2 environment and start the bridge:

```bash
source /opt/ros/humble/setup.bash
physical-mcp-bridge
```

The bridge starts a WebSocket server on **port 9090** by default. You should see output indicating it is listening for connections.

To verify the bridge is working, check that it can see ROS2 topics:

```bash
# In a separate terminal
source /opt/ros/humble/setup.bash
ros2 topic list
```

If `ros2 topic list` shows your robot's topics, the bridge will be able to see them too.

### Running the bridge as a systemd service

For persistent operation on a robot computer, create a systemd service:

```ini
# /etc/systemd/system/physical-mcp-bridge.service
[Unit]
Description=PhysicalMCP ROS2 Bridge
After=network.target

[Service]
Type=simple
User=robot
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && physical-mcp-bridge"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl enable physical-mcp-bridge
sudo systemctl start physical-mcp-bridge
```

---

## Step 3: Configure a Safety Policy

Create a YAML safety policy tailored to your specific robot. This is the most important step for real hardware safety.

### Start from the default policy

```bash
cp packages/mcp-server/policies/default.yaml my-robot-policy.yaml
```

### Set velocity limits matching your hardware

Look up your robot's maximum safe velocities from the manufacturer's specifications or URDF. Set `linearMax` and `angularMax` to values **at or below** the hardware limits:

```yaml
name: my-robot
description: Safety policy for [your robot name/model]

velocity:
  linearMax: 0.3      # m/s - set to your robot's safe operating speed
  angularMax: 1.0      # rad/s - set to your robot's safe turning rate
  clampMode: false     # BLOCK mode recommended for real hardware
```

**Guidance on choosing limits:**

- For initial testing, set limits to **25-50% of the robot's maximum speed**. You can increase them later after gaining confidence.
- Use **block mode** (`clampMode: false`) for real hardware. This ensures that commands exceeding the limit are rejected outright rather than silently reduced. You want the AI agent to know when it is trying to go too fast.
- If your robot has different speed limits for different conditions (loaded vs. unloaded, indoor vs. outdoor), use the most conservative values.

### Define a geofence matching your workspace

Measure your workspace and set boundaries with a safety margin:

```yaml
geofence:
  xMin: -3.0     # meters - measure from the robot's starting position
  xMax: 3.0
  yMin: -2.0
  yMax: 2.0
  zMin: 0.0       # ground level
  zMax: 1.0       # ceiling (relevant for drones and arms)
```

**Start small.** A geofence of 2-3 meters in each direction is sufficient for initial testing. Expand as you verify the system behaves correctly.

### Set conservative rate limits

```yaml
rateLimits:
  publishHz: 5           # lower than simulation defaults
  servicePerMinute: 30   # conservative for real hardware
  actionPerMinute: 15
```

### Block dangerous topics and services

Identify any topics or services that should never be accessed by an AI agent on your robot:

```yaml
blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
  - /set_parameters       # prevents reconfiguring nodes at runtime
```

For maximum safety, consider using an allowlist instead of a blocklist:

```yaml
# Only allow these specific topics (everything else is blocked)
allowedTopics:
  - /cmd_vel
  - /odom
  - /scan

allowedServices:
  - /navigate_to_pose
```

### Enable the deadman switch

For real hardware, enabling the deadman switch is strongly recommended. It auto-activates the emergency stop if the MCP server disconnects:

```yaml
deadmanSwitch:
  enabled: true
  timeoutMs: 30000    # 30 seconds - stop robot if no heartbeat for 30s
```

The AI agent (or the MCP client) must call `safety_heartbeat` periodically to keep the robot active. If communication is lost, the robot stops automatically.

### Full example policy for real hardware

```yaml
name: my-ground-robot
description: Conservative policy for initial real hardware testing

velocity:
  linearMax: 0.2
  angularMax: 0.8
  clampMode: false

geofence:
  xMin: -2.0
  xMax: 2.0
  yMin: -2.0
  yMax: 2.0
  zMin: 0.0
  zMax: 0.5

rateLimits:
  publishHz: 5
  servicePerMinute: 30
  actionPerMinute: 15

deadmanSwitch:
  enabled: true
  timeoutMs: 30000

blockedTopics:
  - /rosout
  - /parameter_events
  - /diagnostics

blockedServices:
  - /kill
  - /shutdown
  - /set_parameters
```

---

## Step 4: Point the MCP Server at the Bridge

The MCP server connects to the Python bridge over WebSocket. Set the bridge URL to point at the machine running the bridge:

```bash
export PHYSICAL_MCP_BRIDGE_URL=ws://ROBOT_IP:9090
export PHYSICAL_MCP_POLICY=/path/to/my-robot-policy.yaml
npx @ricardothe3rd/physical-mcp
```

Replace `ROBOT_IP` with:
- `localhost` or `127.0.0.1` if the MCP server and bridge run on the same machine
- The robot computer's IP address if they are on different machines (e.g., `ws://192.168.1.105:9090`)

### Claude Desktop configuration

Add to your `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "physical-mcp": {
      "command": "npx",
      "args": ["@ricardothe3rd/physical-mcp"],
      "env": {
        "PHYSICAL_MCP_BRIDGE_URL": "ws://192.168.1.105:9090",
        "PHYSICAL_MCP_POLICY": "/path/to/my-robot-policy.yaml"
      }
    }
  }
}
```

### Claude Code configuration

```bash
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp
```

Then set environment variables in your shell or in a `.env` file before starting Claude Code.

---

## Step 5: Test with Read-Only Tools First

Before sending any commands that could move the robot, verify that the system is working correctly using read-only tools. These tools only observe the ROS2 graph and cannot cause any physical action.

### Test 1: Check bridge connectivity

Ask the AI agent:

> "Check the bridge connection status."

This calls `system_bridge_status`. You should see a successful connection with low latency. If the bridge is not connected, revisit Steps 2 and 4.

### Test 2: List ROS2 topics

> "List all available ROS2 topics."

This calls `ros2_topic_list`. Verify that your robot's expected topics appear (e.g., `/cmd_vel`, `/odom`, `/scan`, `/joint_states`). If topics are missing, the robot's ROS2 driver may not be running.

### Test 3: List ROS2 nodes

> "List all ROS2 nodes."

This calls `system_node_list`. Verify that your robot's nodes are present.

### Test 4: Subscribe to sensor data

> "Show me the latest message from /odom."

This calls `ros2_topic_echo` on a read-only topic. Verify that the data looks reasonable (positions, velocities, timestamps).

### Test 5: Verify the safety policy

> "Show me the current safety status."

This calls `safety_status`. Verify that your custom policy is loaded with the correct velocity limits, geofence, and rate limits.

### Test 6: Test emergency stop

> "Activate emergency stop."

Verify that the e-stop activates. Then release it:

> "Release emergency stop with confirmation CONFIRM_RELEASE."

This verifies that the safety controls work before you send any movement commands.

### Only after all tests pass: try a movement command

Start with the smallest possible movement:

> "Move the robot forward at 0.05 m/s for one second."

Stand near the robot's e-stop button. Watch the robot carefully. Verify that the movement matches what you expect.

---

## Safety Checklist Before Real Hardware

Complete this checklist before every session with real hardware. Do not skip items.

### Physical safety

- [ ] **Hardware emergency stop is accessible.** You (or someone) can reach the robot's physical e-stop button within 2 seconds from your operating position. Never operate a robot where you cannot quickly reach the e-stop.
- [ ] **Workspace is clear.** Remove people, pets, fragile objects, and obstacles from the robot's operational area and geofence zone.
- [ ] **Robot is on a flat, stable surface.** Uneven terrain can cause unexpected behavior even at low speeds.
- [ ] **Battery is adequately charged.** Low battery can cause unpredictable motor behavior on some platforms.

### Software safety

- [ ] **Conservative velocity limits are set.** Your policy's `linearMax` and `angularMax` are at or below 50% of the robot's hardware maximum for initial testing.
- [ ] **Small geofence is defined.** The geofence is tight enough that the robot cannot reach walls, furniture, or people even if the geofence check has a momentary delay.
- [ ] **Deadman switch is enabled.** `deadmanSwitch.enabled: true` ensures the robot stops if communication is lost.
- [ ] **Block mode is active.** `clampMode: false` ensures over-limit commands are rejected, not silently reduced.
- [ ] **Dangerous topics and services are blocked.** System topics (`/rosout`, `/parameter_events`) and dangerous services (`/kill`, `/shutdown`) are in the blocked list.
- [ ] **Read-only tools have been tested first.** You have completed all tests in [Step 5](#step-5-test-with-read-only-tools-first) before sending movement commands.

### Operational safety

- [ ] **You have tested this workflow in simulation.** The exact sequence of commands you plan to use has been run in Gazebo first.
- [ ] **Someone is watching the robot.** Never operate real hardware with an AI agent without visual line-of-sight to the robot.
- [ ] **You know how to activate the software e-stop.** You can quickly tell the AI agent to activate the emergency stop, or you can kill the bridge process to halt all commands.

---

## Network Considerations

### Same LAN requirement

The MCP server and the bridge must be able to communicate over WebSocket (TCP, port 9090). In practice, this means:

- **Same local network (LAN):** The simplest and most reliable setup. Both machines are on the same WiFi network or Ethernet switch.
- **Direct Ethernet connection:** For low-latency operation, connect the operator workstation directly to the robot computer via Ethernet cable.
- **VPN:** If the machines are on different networks, a VPN can bridge them. Expect higher latency.

### Firewall configuration

Ensure port 9090 (TCP) is open between the MCP server machine and the bridge machine:

```bash
# On the bridge machine (Ubuntu/Debian)
sudo ufw allow 9090/tcp

# Verify
sudo ufw status
```

On the MCP server machine, outbound connections to port 9090 must not be blocked.

### ROS2 DDS discovery

The Python bridge needs to discover ROS2 nodes via DDS. If the bridge runs on the robot's computer, this works automatically. If the bridge runs on a separate machine:

- Both machines must be on the same LAN
- Both must use the same `ROS_DOMAIN_ID` (default: 0)
- `ROS_LOCALHOST_ONLY` must **not** be set to `1` on either machine
- Some network configurations (corporate VPNs, complex subnets) can block DDS multicast. If topic discovery fails, check your network's multicast settings.

### Latency expectations

| Setup | Typical Latency |
|-------|----------------|
| Bridge on robot, MCP server on same machine | < 1 ms |
| Bridge on robot, MCP server on same LAN (Ethernet) | 1-5 ms |
| Bridge on robot, MCP server on same LAN (WiFi) | 5-50 ms |
| Bridge on robot, MCP server over VPN | 50-200 ms |

For real-time control applications, Ethernet is strongly recommended over WiFi. WiFi latency spikes can cause jerky motion or missed deadman switch heartbeats.

---

## Known Robot Platform Tips

### Ground robots (TurtleBot3, Clearpath, etc.)

- **Velocity topics:** Most ground robots use `/cmd_vel` with `geometry_msgs/msg/Twist`. PhysicalMCP's velocity limiting is designed for this.
- **Odometry:** Subscribe to `/odom` to track the robot's position. The geofence check uses this data when available.
- **LIDAR:** Subscribe to `/scan` for obstacle awareness. This is read-only and safe to access freely.
- **Navigation:** If your robot runs Nav2 (`ros-humble-nav2-bringup`), you can send navigation goals via the `ros2_action_send_goal` tool. These are subject to geofence checks.
- **Start slow:** Even if the robot can move at 2 m/s, start testing at 0.1 m/s. Increase speed only after confirming correct behavior at lower speeds.

**Recommended initial policy values for ground robots:**

```yaml
velocity:
  linearMax: 0.2      # very conservative start
  angularMax: 0.5
  clampMode: false
geofence:
  xMin: -2.0
  xMax: 2.0
  yMin: -2.0
  yMax: 2.0
  zMin: 0.0
  zMax: 0.5
deadmanSwitch:
  enabled: true
  timeoutMs: 30000
```

### Robot arms and manipulators

- **Velocity limits apply to end-effector commands** published via Twist messages. Joint-level velocity limits are not yet supported by PhysicalMCP; rely on your robot's own joint limit enforcement.
- **Geofence is critical.** A robot arm can collide with its own base, the table, or nearby objects. Set a tight geofence matching the arm's safe workspace.
- **Use the lowest possible speeds.** Robot arms can exert significant force. Start at 0.01-0.05 m/s for end-effector velocity.
- **Block joint-level control topics** if you only want the AI agent to use task-space (Cartesian) commands.
- **Enable the deadman switch** with a short timeout (5-10 seconds). An unattended arm is dangerous.

**Recommended initial policy values for robot arms:**

```yaml
velocity:
  linearMax: 0.05
  angularMax: 0.3
  clampMode: false
geofence:
  xMin: -0.8
  xMax: 0.8
  yMin: -0.8
  yMax: 0.8
  zMin: 0.0
  zMax: 1.2
deadmanSwitch:
  enabled: true
  timeoutMs: 5000
```

### Drones and aerial robots

- **PhysicalMCP is not a flight controller.** It can send commands to a ROS2-based flight stack, but it does not replace the flight controller's own safety systems (return-to-home, geofence, battery failsafe).
- **3D geofence is essential.** Set `zMax` to a safe altitude for your environment. Indoor drones might use `zMax: 2.0`; outdoor drones need larger values.
- **Enable the deadman switch** with a short timeout (10 seconds). Loss of communication with a drone should trigger a land or hold command.
- **Block arming and disarming services** if you do not want the AI agent to control the drone's armed state.
- **Always have a manual override** (RC transmitter) ready to take control immediately.

**Recommended initial policy values for drones:**

```yaml
velocity:
  linearMax: 0.5
  angularMax: 1.0
  clampMode: false
geofence:
  xMin: -5.0
  xMax: 5.0
  yMin: -5.0
  yMax: 5.0
  zMin: 0.0
  zMax: 3.0
deadmanSwitch:
  enabled: true
  timeoutMs: 10000
```

### General advice for any platform

1. **Read the robot's documentation.** Understand its ROS2 topic and service layout before connecting PhysicalMCP.
2. **Use `ros2 topic list` and `ros2 service list`** on the robot before involving the AI agent. Know what is available.
3. **Monitor the robot visually at all times.** PhysicalMCP's safety layer is a significant mitigation, but it is one layer in a defense-in-depth approach. Your eyes and the hardware e-stop are the final safety net.
4. **Log everything.** Use `safety_audit_log` to review all commands the AI agent sent after each session.
5. **Start each session by verifying the safety policy** with `safety_status` before sending any commands.

---

## Operational Procedures

### Starting a session

1. Power on the robot and verify its ROS2 driver is running.
2. Start the bridge on the robot computer: `physical-mcp-bridge`.
3. Start the MCP server on your workstation with the correct bridge URL and policy file.
4. Run the read-only test sequence (Step 5) to verify connectivity and safety policy.
5. Begin your work, starting with the smallest possible commands.

### Ending a session

1. Stop any active robot motion by publishing zero velocity or activating the e-stop.
2. Stop the MCP server.
3. Stop the bridge: `Ctrl+C` or `systemctl stop physical-mcp-bridge`.
4. Review the audit log for the session if needed.
5. Power down the robot.

### Emergency procedures

If the robot behaves unexpectedly:

1. **Press the hardware e-stop button** on the robot. This is always the fastest and most reliable stop.
2. If you cannot reach the button, tell the AI agent: "Activate emergency stop."
3. If the AI agent is unresponsive, kill the bridge process (`Ctrl+C` or `kill`). Without the bridge, no commands can reach the robot.
4. If the bridge is unresponsive, power off the robot.

Never assume software will stop the robot. The hardware e-stop is your last line of defense.

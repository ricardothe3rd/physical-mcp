# PhysicalMCP Launch Playbook

## The Strategy

**Core angle:** "The only AI-to-robot bridge with a safety layer."

Every competitor (robotmcp 1K stars, wise-vision 65 stars) lets AI send raw commands to robots with zero guardrails. PhysicalMCP is the first to say "no, you can't drive that robot at 10 m/s" and block it with a clear message. That's the hook.

**Launch day:** Tuesday (60% higher avg HN score than other days)

**Sequence:**
1. Record demo GIFs/video FIRST (nothing launches without visuals)
2. Add GIFs to README
3. npm publish (so people can actually install it)
4. Post simultaneously: Hacker News → Reddit → Twitter → ROS Discourse
5. Spend the entire day replying to every comment

---

## Step 1: Demo Recording Plan

### GIF 1: "The Magic Moment" (10 seconds)
**What to show:** Terminal split-screen. Left = Claude conversation. Right = Gazebo sim.
1. Type: "Move the robot forward at 0.1 m/s"
2. Robot moves in Gazebo
3. Type: "Move the robot forward at 5 m/s"
4. Red "SAFETY BLOCKED" message appears
5. Robot doesn't move

**Why it works:** Single GIF shows both the capability AND the differentiator.

### GIF 2: "Emergency Stop" (8 seconds)
1. Robot is moving
2. "Activate emergency stop"
3. Robot stops immediately
4. "Move forward" → BLOCKED
5. Brief flash of the audit log

### GIF 3: "Full Discovery" (6 seconds)
1. "List all ROS2 topics" → clean JSON output
2. "What's the robot's position?" → odometry data
3. Shows the AI can see everything about the robot

### How to Record
```bash
# Option A: asciinema (terminal recording → GIF)
brew install asciinema
pip install asciinema-agg  # converts to GIF
asciinema rec demo.cast
# ... do the demo ...
agg demo.cast demo.gif --cols 120 --rows 30

# Option B: Screen record with OBS/QuickTime
# Capture both terminal + Gazebo side by side
# Convert to GIF with: ffmpeg -i demo.mp4 -vf "fps=10,scale=800:-1" demo.gif

# Option C: For YouTube Short
# Record screen + voiceover explaining what's happening
# 15-30 seconds, vertical format (1080x1920)
```

### Where GIFs Go in README
```markdown
## Why PhysicalMCP?

![Safety demo](docs/demo-safety-block.gif)

*Claude tries to drive a TurtleBot3 at 5 m/s. PhysicalMCP blocks it — the limit is 0.5 m/s.*
```

---

## Step 2: Post Drafts

### Hacker News — Show HN

**Title:** `Show HN: PhysicalMCP – Safety-first MCP server for controlling ROS2 robots with AI`

**Comment (post as first comment immediately):**

```
Hey HN — I built PhysicalMCP because every existing MCP-to-ROS2 bridge (robotmcp, wise-vision, etc.) lets AI agents send raw commands to robots with zero guardrails. That's fine for simulation, but terrifying for real hardware.

PhysicalMCP adds a safety layer that sits between the AI and the robot:

- Velocity limits: caps linear/angular velocity (blocks "drive at 10 m/s")
- Geofence: defines workspace boundaries the robot must stay in
- Rate limiting: prevents flooding the ROS2 graph
- Emergency stop: dual-layer e-stop with confirmation-to-release
- Full audit log: every command recorded with safety check result

The safety layer is enforced in the MCP server (TypeScript) BEFORE commands reach the ROS2 bridge (Python). The bridge has a secondary e-stop as a fail-safe.

21 MCP tools covering topics, services, actions, and 7 safety-specific tools. Safety policies are YAML configs — swap a file and the limits change instantly.

Stack: TypeScript MCP server (@modelcontextprotocol/sdk) + Python ROS2 bridge (rclpy + websockets), connected via WebSocket. Docker setup included for Gazebo TurtleBot3 simulation.

GitHub: https://github.com/ricardothe3rd/physical-mcp

Would love feedback on the safety model — especially from anyone working with real robots and AI agents.
```

**Why this works:**
- States the problem immediately (raw commands = dangerous)
- Lists concrete features with examples
- Technical depth (HN loves architecture details)
- Ends with specific feedback request
- Links to GitHub (HN favors open source)

---

### Reddit r/ROS

**Title:** `Open-source MCP server for ROS2 with built-in safety layer — control your robot with Claude/GPT, velocity limits, geofence, e-stop, and audit logging`

**Body:**

```
I built PhysicalMCP — a safety-first MCP server that bridges AI agents (Claude, GPT, any LLM) to ROS2 robots.

Every existing MCP-ROS2 bridge lets AI send raw commands with no guardrails. PhysicalMCP adds:

✅ Velocity limits (configurable per robot)
✅ Geofence boundaries
✅ Rate limiting (per topic/service/action)
✅ Dual-layer emergency stop (MCP server + bridge)
✅ Full audit logging of every command
✅ YAML safety policies (swap a file, change limits)
✅ 21 MCP tools: topics, services, actions, safety, system

Architecture: TypeScript MCP server → WebSocket → Python ROS2 bridge (rclpy)

Docker setup included with Gazebo TurtleBot3 for testing.

The safety layer evaluates EVERY publish, service call, and action goal before it reaches the bridge. Try to publish cmd_vel at 10 m/s and you get:

    SAFETY BLOCKED: Publish to /cmd_vel denied.
    Violations:
    - [velocity_exceeded] Linear velocity 10.00 m/s exceeds limit of 0.5 m/s

GitHub: https://github.com/ricardothe3rd/physical-mcp

Interested in feedback from anyone running AI agents on real hardware. What safety features would you need?
```

---

### Reddit r/ClaudeAI

**Title:** `I built an MCP server that lets Claude control real robots — with a safety layer that blocks dangerous commands [open source]`

**Body:**

```
PhysicalMCP bridges Claude (or any MCP-compatible LLM) to ROS2 robots with built-in safety:

- "Move forward at 0.1 m/s" → ✅ Published
- "Move forward at 5 m/s" → ❌ BLOCKED (exceeds 0.5 m/s limit)
- "Activate emergency stop" → 🛑 All commands blocked until explicit release

21 tools for full ROS2 control: topics, services, actions, plus 7 safety-specific tools (e-stop, audit log, policy management, geofence, velocity limits).

Install: `claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp`

Every command is safety-checked and audit-logged. The AI can read sensors and control movement, but can't exceed velocity limits, access blocked topics, or bypass the e-stop.

GitHub: https://github.com/ricardothe3rd/physical-mcp

This is the first MCP server I've seen with a real safety layer — existing ones just pass commands straight through.
```

---

### Reddit r/LocalLLaMA

**Title:** `MCP server that connects any LLM to ROS2 robots with safety guardrails — works with local models via any MCP client`

**Body:**

```
Built PhysicalMCP — an MCP server for controlling ROS2 robots from any LLM. Since it's MCP-based, it works with Claude, GPT, or any local model that supports MCP tool calling.

The key difference from existing ROS-MCP bridges: a safety layer that blocks dangerous commands before they reach the robot.

- Velocity limits, geofence, rate limiting, e-stop, audit logging
- 21 tools covering the full ROS2 stack
- YAML safety policies — configure limits per robot
- TypeScript server + Python bridge, Docker setup included

GitHub: https://github.com/ricardothe3rd/physical-mcp

Anyone using local models for robotics? Curious what safety features you'd need beyond velocity limits and geofence.
```

---

### Reddit r/robotics

**Title:** `PhysicalMCP: open-source bridge between AI agents and ROS2 with velocity limits, geofence, e-stop, and audit logging`

**Body:**

```
I've been working on PhysicalMCP — a safety-first MCP server that lets AI agents control ROS2 robots.

The problem: every existing AI-to-ROS2 bridge passes commands straight through. That's fine for simulation, but for real hardware, you need guardrails.

PhysicalMCP enforces safety BEFORE commands reach the robot:
- Velocity limits (configurable linear + angular max)
- Geofence workspace boundaries
- Rate limiting per topic/service/action
- Dual-layer emergency stop (server-side + bridge-side)
- Complete audit trail of every command

21 MCP tools for topics, services, actions + 7 safety management tools. Includes Docker setup with Gazebo TurtleBot3 and YAML policy configs.

The safety policy is swappable — I've included a conservative default and a TurtleBot3-specific one. You can write your own for any robot.

GitHub: https://github.com/ricardothe3rd/physical-mcp

Would appreciate feedback from anyone doing real-world robot operations with AI in the loop.
```

---

### Twitter/X Thread

```
🤖 I built PhysicalMCP — the first MCP server for ROS2 robots with a real safety layer.

Every other AI-to-robot bridge lets the AI send raw commands. No limits. No guardrails.

PhysicalMCP says: "No, you can't drive that robot at 10 m/s."

🧵👇

---

The safety layer checks EVERY command before it reaches the robot:

✅ 0.1 m/s → Published
❌ 5.0 m/s → BLOCKED (limit: 0.5 m/s)
🛑 E-stop → All commands blocked until explicit release

[attach GIF 1]

---

7 safety features no other MCP-ROS2 bridge has:

• Velocity limits (linear + angular)
• Geofence boundaries
• Rate limiting
• Emergency stop (dual-layer)
• Blocked topics/services
• YAML policy configs
• Full audit logging

---

21 MCP tools for full ROS2 control:
• Topics: list, subscribe, publish, echo
• Services: list, call
• Actions: list, send goal, cancel
• Safety: e-stop, status, policy, audit
• System: bridge health, node list

---

Stack:
TypeScript MCP server + Python ROS2 bridge
Connected via WebSocket
Safety enforced in TS BEFORE reaching ROS2
Bridge has secondary e-stop as fail-safe

---

Open source. MIT license.

GitHub: github.com/ricardothe3rd/physical-mcp

Install: npx @ricardothe3rd/physical-mcp

If you're using AI to control robots, you need a safety layer.

This is it. 🔒🤖
```

---

### ROS Discourse

**Title:** `PhysicalMCP: Safety-first MCP server for bridging AI agents to ROS2`

**Body:**

```
Hi everyone,

I'm releasing PhysicalMCP, an open-source MCP server that bridges AI agents (Claude, GPT, any LLM with MCP support) to ROS2 robots — with a built-in safety layer.

**Why another MCP-ROS2 bridge?**

Existing bridges (robotmcp, wise-vision, lpigeon) let AI agents send raw commands with no safety checks. For simulation and research, that's fine. For real hardware or production deployments, you need enforced guardrails.

**Safety features (the differentiator):**
- Velocity limits: configurable linear + angular max, blocks any Twist message exceeding limits
- Geofence: workspace boundaries checked before position-affecting commands
- Rate limiting: per-topic publish rate, per-service call rate, per-action goal rate
- Emergency stop: dual-layer (MCP server blocks all commands + bridge publishes zero velocity and cancels all goals)
- Blocked topics/services: prevent access to system topics (/rosout, /parameter_events) and dangerous services (/kill, /shutdown)
- Audit logging: every command recorded with timestamp, safety result, and execution outcome
- YAML policy configs: swap a policy file to change all limits per robot

**Architecture:**
TypeScript MCP server (21 tools) → WebSocket → Python ROS2 bridge (rclpy)

Safety is enforced in the TypeScript server BEFORE commands reach the bridge. The bridge has a secondary e-stop as an additional fail-safe.

**Included:**
- Docker Compose setup with Gazebo + TurtleBot3 for testing
- Default + TurtleBot3-specific safety policies
- 75 unit tests for the safety layer (velocity, geofence, rate limiter, audit logger, policy loader, protocol)

**Install:**
```
claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp
```

**GitHub:** https://github.com/ricardothe3rd/physical-mcp

I'd love feedback, especially on:
1. What additional safety constraints would you want for production robot deployments?
2. Interest in a cloud relay for remote robot access?
3. Any robot-specific safety policies you'd want included by default?

Thanks!
```

---

## Step 3: Launch Day Checklist

### Before Launch
- [ ] Record 3 demo GIFs (magic moment, e-stop, discovery)
- [ ] Add GIFs to README between "Why PhysicalMCP?" and "Architecture"
- [ ] npm publish (`npm publish --access public`)
- [ ] Verify `npx @ricardothe3rd/physical-mcp` works
- [ ] Record YouTube Short (optional but high-impact)
- [ ] Prepare all post drafts in browser tabs

### Launch Day (Tuesday)
- [ ] **10:00 AM EST** — Post on Hacker News (Show HN)
- [ ] **10:05 AM** — Post first comment on HN immediately
- [ ] **10:10 AM** — Post on r/ROS
- [ ] **10:15 AM** — Post on r/ClaudeAI
- [ ] **10:20 AM** — Post on r/robotics
- [ ] **10:25 AM** — Post on r/LocalLLaMA
- [ ] **10:30 AM** — Post Twitter thread
- [ ] **11:00 AM** — Post on ROS Discourse
- [ ] **All day** — Reply to EVERY comment within 30 minutes
- [ ] **Evening** — Cross-post any traction ("We hit #X on HN!")

### Post-Launch (Days 2-7)
- [ ] Share any traction on Twitter (star count milestones, interesting comments)
- [ ] If HN post doesn't gain traction, resubmit on Saturday (2nd best day)
- [ ] Write a blog post / dev.to article expanding on the safety approach
- [ ] Engage with anyone who opens issues or PRs on GitHub

---

## Key Principles

1. **Demo GIFs are non-negotiable.** The robotmcp README has 3 GIFs and 1K stars. Posts without visuals die.
2. **Reply to every comment.** HN and Reddit reward active OP engagement. Budget the entire day for this.
3. **Lead with the safety angle.** "The only one with safety" is a stronger hook than "another ROS-MCP bridge."
4. **Be specific.** "Blocks velocity at 0.5 m/s" > "has safety features."
5. **Ask for feedback.** Every post ends with a question. This drives comments, which drives visibility.
6. **No superlatives.** Don't say "best" or "fastest." Let the safety comparison table speak for itself.

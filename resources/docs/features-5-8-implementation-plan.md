# Implementation Plan: Features 5-8

> Concrete implementation plans for the web dashboard, CI badge, setup wizard, and Gazebo demo world.

---

## Feature 5: Simple Web Dashboard (Audit Event Viewer)

**Goal:** A localhost web UI that displays safety audit events in real-time from PhysicalMCP.

### Architecture

```
Browser (localhost:3000)
    │
    ▼
Express server (serves static files + REST API)
    │
    ├── GET /api/audit         → Returns audit entries (JSON)
    ├── GET /api/audit/stats   → Returns {total, allowed, blocked, errors}
    ├── GET /api/audit/stream  → SSE stream of new audit events
    └── GET /                  → Serves the single-page dashboard
    │
    ▼
AuditLogger (already exists in safety/audit-logger.ts)
```

### What Already Exists
- `AuditLogger.getEntries(options?)` — returns entries with filtering (limit, violationsOnly, commandType)
- `AuditLogger.getStats()` — returns `{total, allowed, blocked, errors}`
- `AuditExporter.export()` — exports to JSON/CSV/JSONL with time range + type filtering
- `SafetyViolationType` enum — 11 violation types for color-coding

### Implementation Steps

**Step 1: Add Express dependency**
```bash
cd packages/mcp-server
npm install express
npm install -D @types/express
```

**Step 2: Create dashboard server** (`src/dashboard/server.ts`)
- Embedded Express server that starts alongside the MCP server
- Serves static HTML/CSS/JS from `src/dashboard/public/`
- REST endpoints wrapping AuditLogger methods
- SSE endpoint for real-time push using the existing `SafetyEventEmitter`
- Enabled via `--dashboard` flag or `PHYSICAL_MCP_DASHBOARD=true` env var
- Default port: 3000 (configurable via `--dashboard-port`)

**Step 3: Create dashboard UI** (`src/dashboard/public/index.html`)

Single HTML file with embedded CSS/JS (no build step needed):

```
┌─────────────────────────────────────────────────────────┐
│  PhysicalMCP Dashboard                    [Live ●]      │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Stats Bar:                                             │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐   │
│  │ Total    │ │ Allowed  │ │ Blocked  │ │ Errors   │   │
│  │   150    │ │   140    │ │    8     │ │    2     │   │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘   │
│                                                         │
│  Filter: [All ▼] [Blocked Only ☐] [Search...]          │
│                                                         │
│  Audit Event Log:                                       │
│  ┌─────────┬────────┬──────────┬────────┬──────────┐   │
│  │  Time   │  Type  │  Target  │ Result │ Details  │   │
│  ├─────────┼────────┼──────────┼────────┼──────────┤   │
│  │ 14:23   │ pub    │ /cmd_vel │ ✅     │          │   │
│  │ 14:22   │ pub    │ /cmd_vel │ 🛑     │ vel 10.0 │   │
│  │ 14:21   │ safety │ e-stop   │ ✅     │ manual   │   │
│  └─────────┴────────┴──────────┴────────┴──────────┘   │
│                                                         │
│  Safety Score: ████████████░░░░ 82/100                  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

**Key UI features:**
- Auto-refresh via SSE (new events appear at top without page reload)
- Color-coded rows: green (allowed), red (blocked), yellow (clamped/warning)
- Expandable rows to see full violation details and params
- Filter by command type, result, time range
- Stats bar updates live
- Safety score gauge
- Export button (calls existing audit-export.ts)
- No npm frontend framework needed — vanilla HTML/CSS/JS with fetch()

**Step 4: Wire into index.ts**
```typescript
// In parseArgs(), add:
// --dashboard       Enable web dashboard (default: false)
// --dashboard-port  Dashboard port (default: 3000)

// After MCP server starts:
if (args.dashboard) {
  const { startDashboard } = await import('./dashboard/server.js');
  startDashboard({ port: args.dashboardPort, auditLogger, policyEngine });
}
```

### Files to Create
```
packages/mcp-server/src/dashboard/
├── server.ts           # Express server with REST + SSE endpoints
└── public/
    └── index.html      # Single-file dashboard (HTML + CSS + JS embedded)
```

### Files to Modify
```
packages/mcp-server/src/index.ts    # Add --dashboard flag and startup
packages/mcp-server/package.json    # Add express dependency
```

### Testing
- Unit test: dashboard REST endpoints return correct audit data
- Manual test: start with `--dashboard`, open browser, verify events appear
- SSE test: publish a command, verify it appears in dashboard without refresh

---

## Feature 6: GitHub Actions Badge in README

**Goal:** Show real CI status badge on GitHub and npm.

### Current State
The badge already exists in README.md:
```markdown
[![CI](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml)
```

### What Needs to Happen
1. **Verify CI passes on GitHub** — push triggers the workflow, badge auto-updates
2. **Add more badges** that update dynamically:

```markdown
[![CI](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/ricardothe3rd/physical-mcp/actions/workflows/ci.yml)
[![npm version](https://img.shields.io/npm/v/@ricardothe3rd/physical-mcp)](https://www.npmjs.com/package/@ricardothe3rd/physical-mcp)
[![npm downloads](https://img.shields.io/npm/dm/@ricardothe3rd/physical-mcp)](https://www.npmjs.com/package/@ricardothe3rd/physical-mcp)
[![GitHub stars](https://img.shields.io/github/stars/ricardothe3rd/physical-mcp)](https://github.com/ricardothe3rd/physical-mcp/stargazers)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
```

### Implementation
- The CI badge will work automatically once CI runs on GitHub (just needs a push)
- npm download badge needs npm publish to work (currently blocked by expired token)
- GitHub stars badge works immediately
- Add `npm downloads` and `GitHub stars` badges to README

### Effort: 15 minutes (mostly just adding badge URLs)

---

## Feature 7: Interactive Setup Wizard (`npx physical-mcp init`)

**Goal:** Run `npx @ricardothe3rd/physical-mcp init` to interactively generate a safety policy YAML and configure the bridge connection.

### Architecture

```
$ npx @ricardothe3rd/physical-mcp init

Welcome to PhysicalMCP Setup!

? What type of robot are you using?
  ❯ TurtleBot3 (Burger/Waffle)
    Universal Robots (UR3/UR5/UR10)
    Custom robot
    Simulation only (Gazebo)

? Bridge URL: (ws://localhost:9090)

? Maximum linear velocity (m/s): (0.5)

? Maximum angular velocity (rad/s): (1.5)

? Enable geofence? (Y/n)
? Geofence X range: (-5.0 to 5.0)
? Geofence Y range: (-5.0 to 5.0)

? Enable deadman switch? (y/N)

? Tool profile:
  ❯ minimal (read-only: topics, safety, system)
    standard (+ publish, services, actions)
    full (all 100 tools)

✅ Generated policy: ./physical-mcp-policy.yaml
✅ Generated config: ./physical-mcp.json

To start: npx @ricardothe3rd/physical-mcp --policy ./physical-mcp-policy.yaml

To add to Claude:
  claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp --policy ./physical-mcp-policy.yaml
```

### Implementation Steps

**Step 1: Add prompts dependency**
```bash
npm install prompts  # Lightweight interactive prompts (15KB, zero deps)
npm install -D @types/prompts
```

**Step 2: Create wizard** (`src/cli/init.ts`)

```typescript
import prompts from 'prompts';
import { writeFileSync } from 'fs';
import yaml from 'yaml';

export async function runInitWizard(): Promise<void> {
  // Robot type selection (presets)
  // Bridge URL
  // Velocity limits
  // Geofence config
  // Deadman switch
  // Tool profile
  // Rate limits
  // Blocked topics
  // Generate YAML policy file
  // Generate JSON config for Claude Desktop / Cursor / Gemini CLI
  // Print next steps
}
```

**Step 3: Add subcommand routing to index.ts**

```typescript
// At top of main(), before MCP server starts:
const subcommand = process.argv[2];
if (subcommand === 'init') {
  const { runInitWizard } = await import('./cli/init.js');
  await runInitWizard();
  process.exit(0);
}
```

**Step 4: Robot presets**

Built-in presets that auto-fill limits:

```typescript
const ROBOT_PRESETS = {
  turtlebot3: {
    linearMax: 0.22, angularMax: 2.84, clampMode: true,
    geofence: { xMin: -5, xMax: 5, yMin: -5, yMax: 5, zMin: 0, zMax: 1 }
  },
  ur5: {
    linearMax: 0.1, angularMax: 0.5, clampMode: false,
    // Robot arm — tighter limits
  },
  drone: {
    linearMax: 2.0, angularMax: 1.0, clampMode: true,
    geofence: { xMin: -50, xMax: 50, yMin: -50, yMax: 50, zMin: 0, zMax: 30 }
  },
  simulation: {
    linearMax: 0.5, angularMax: 1.5, clampMode: false,
    geofence: { xMin: -10, xMax: 10, yMin: -10, yMax: 10, zMin: 0, zMax: 5 }
  },
  custom: null // user fills in everything
};
```

**Step 5: Output generation**

Generate two files:
1. `physical-mcp-policy.yaml` — safety policy
2. `physical-mcp.json` — client config (for Claude Desktop, Cursor, Gemini CLI)

The JSON config auto-detects which client to target:
```json
{
  "mcpServers": {
    "physical-mcp": {
      "command": "npx",
      "args": ["@ricardothe3rd/physical-mcp", "--policy", "./physical-mcp-policy.yaml"]
    }
  }
}
```

### Files to Create
```
packages/mcp-server/src/cli/
├── init.ts             # Interactive wizard
└── presets.ts          # Robot preset configurations
```

### Files to Modify
```
packages/mcp-server/src/index.ts    # Add subcommand routing
packages/mcp-server/package.json    # Add prompts dependency
```

### Testing
- Unit test: preset generation produces valid YAML
- Unit test: generated YAML passes safety_validate_policy
- Manual test: run `npx physical-mcp init`, verify output files

---

## Feature 8: Custom Gazebo Demo World

**Goal:** A Gazebo world with walls, obstacles, and geofence-visible boundaries to demonstrate safety features visually.

### Architecture

```
┌─────────────────────────────────────────────────┐
│                 Gazebo World                     │
│                                                  │
│  ┌─── Geofence boundary (visible walls) ────┐   │
│  │                                          │   │
│  │    ┌─────┐        ┌─────┐                │   │
│  │    │ Box │        │ Box │                │   │
│  │    └─────┘        └─────┘                │   │
│  │                                          │   │
│  │         🤖 TurtleBot3                    │   │
│  │                                          │   │
│  │    ┌────────────┐     ┌──────┐           │   │
│  │    │   Table    │     │ Wall │           │   │
│  │    └────────────┘     └──────┘           │   │
│  │                                          │   │
│  │    ⚠️ Warning zone (geofence margin)     │   │
│  └──────────────────────────────────────────┘   │
│                                                  │
│  ⛔ Outside geofence (blocked area)              │
└─────────────────────────────────────────────────┘
```

### Implementation Steps

**Step 1: Create SDF world file** (`docker/worlds/physical_mcp_demo.world`)

Gazebo world with:
- 10m x 10m walled area (matches default geofence: -5 to +5)
- Visual markers at geofence warning margin (colored floor strips at 1m from walls)
- 4-6 obstacles inside (boxes, cylinders, walls of varying sizes)
- A narrow corridor to test collision zone warnings
- Open area for free driving
- Good lighting for camera tool demos
- Textured ground plane for odometry visual feedback

**Step 2: Create matching policy** (`packages/mcp-server/policies/demo.yaml`)

```yaml
name: demo-world
description: Matches the PhysicalMCP demo Gazebo world

velocity:
  linearMax: 0.5
  angularMax: 1.5
  clampMode: true

acceleration:
  enabled: true
  linearMaxAccel: 1.0
  angularMaxAccel: 3.0

geofence:
  xMin: -4.5
  xMax: 4.5
  yMin: -4.5
  yMax: 4.5
  zMin: 0.0
  zMax: 2.0

geofenceWarningMargin: 1.0

rateLimits:
  publishHz: 10
  servicePerMinute: 60
  actionPerMinute: 30

deadmanSwitch:
  enabled: false

blockedTopics:
  - /rosout
  - /parameter_events

blockedServices:
  - /kill
  - /shutdown
```

**Step 3: Create launch file** (`docker/launch/demo_world.launch.py`)

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Gazebo with custom world
        # TurtleBot3 spawn at (0, 0, 0)
        # Nav2 stack (optional, for path planning demos)
    ])
```

**Step 4: Update Docker setup**

```yaml
# docker-compose.yml - add demo profile
services:
  sim-demo:
    build:
      context: .
      dockerfile: docker/Dockerfile.sim
    environment:
      - TURTLEBOT3_MODEL=burger
      - GAZEBO_WORLD=/worlds/physical_mcp_demo.world
    volumes:
      - ./docker/worlds:/worlds:ro
    network_mode: host
```

**Step 5: Update Dockerfile.sim**

Add ability to specify custom world via environment variable:
```dockerfile
ENV GAZEBO_WORLD=""
CMD if [ -n "$GAZEBO_WORLD" ]; then \
      ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=$GAZEBO_WORLD; \
    else \
      ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py; \
    fi
```

### Files to Create
```
docker/worlds/
└── physical_mcp_demo.world     # SDF world file with obstacles + geofence walls

docker/launch/
└── demo_world.launch.py        # ROS2 launch file for demo

packages/mcp-server/policies/
└── demo.yaml                   # Safety policy matching the demo world
```

### Files to Modify
```
docker/docker-compose.yml       # Add demo service variant
docker/Dockerfile.sim           # Support custom world via env var
```

### Demo Script (For Recording Video)

```
1. Start: docker compose up sim-demo bridge
2. Connect: claude mcp add physical-mcp -- npx @ricardothe3rd/physical-mcp --policy policies/demo.yaml
3. Show: "List all topics" → topics appear
4. Show: "Move forward at 0.2 m/s" → robot moves, safety passes
5. Show: "Move forward at 5 m/s" → BLOCKED, velocity exceeded
6. Show: "Move to position (4.5, 0)" → geofence warning at margin
7. Show: "Move to position (6, 0)" → BLOCKED, geofence violation
8. Show: "Emergency stop" → robot halts
9. Show: "Show audit log" → all events visible
10. Show: Dashboard (localhost:3000) → visual audit trail
```

---

## Priority Order & Dependencies

```
Feature 6 (CI Badge)         → 15 min, no dependencies, do first
    │
Feature 7 (Init Wizard)      → 1 day, no dependencies, high user value
    │
Feature 8 (Gazebo Demo)      → 1 day, needs demo policy
    │
Feature 5 (Web Dashboard)    → 2-3 days, benefits from demo world for testing
```

### Recommended Build Order:
1. **Day 1 morning:** Feature 6 (badges) — ship immediately
2. **Day 1 afternoon:** Feature 7 (init wizard) — high UX impact
3. **Day 2:** Feature 8 (Gazebo world + demo policy)
4. **Day 3-4:** Feature 5 (dashboard) — uses demo world for live testing
5. **Day 5:** Record demo video using all 4 features together

---

## Success Metrics

| Feature | Success Looks Like |
|---------|-------------------|
| Dashboard | Open browser, see audit events appear in real-time as Claude sends commands |
| CI Badge | Green "passing" badge visible on GitHub repo page and npm |
| Init Wizard | New user runs `npx @ricardothe3rd/physical-mcp init`, gets working config in 30 seconds |
| Gazebo Demo | Clear visual demo of geofence walls, safety blocking, e-stop — ready for recording |

---

*Last updated: 2026-02-23*

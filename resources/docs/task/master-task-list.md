# PhysicalMCP: Master Task List

> All pending tasks organized by feature area with dependencies and build order.

---

## Build Order Overview

```
Week 1:
  Day 1 ──► #7  npm publish (BLOCKER — unblocks 8 other tasks)
         ──► #8  GitHub release
         ──► #9  Dynamic badges in README
         ──► #10-14 Init wizard (Feature 7)

  Day 2 ──► #15-19 Gazebo demo world (Feature 8)
         ──► #27 Robot safety policies (UR5, Spot, drone)
         ──► #28 OpenAI/Gemini examples in README

  Day 3-4 ► #20-24 Web dashboard (Feature 5)
           ► #26 "Good first issue" tickets

  Day 5 ──► #25 Record demo GIF/video
         ──► #29 Dev.to blog post
         ──► #33 GitHub Sponsors
         ──► #34 Email waitlist

  Day 6-7 ► #30 Hacker News launch
           ► #31 Reddit posts
           ► #32 ROS Discourse post
```

---

## BLOCKER — Do First

### #7 — Fix npm token and publish v1.0.0 to npm
- **Status:** Pending
- **Blocks:** #8, #9, #25, #29, #30, #31, #32
- **What:** `npm login` to re-authenticate (token expired), then `cd packages/mcp-server && npm publish --access public`
- **Why:** Without this, `npx @ricardothe3rd/physical-mcp` doesn't work and the README install command is broken. Everything downstream depends on this.

### #8 — Create GitHub release for v1.0.0
- **Status:** Pending
- **Blocked by:** #7
- **What:** `gh release create v1.0.0 --title "v1.0.0 — 100 MCP Tools, Safety-First" --generate-notes`

### #9 — Add npm downloads and GitHub stars badges to README
- **Status:** Pending
- **Blocked by:** #7
- **What:** Add dynamic shields.io badges for npm weekly downloads and GitHub stars count to both root README.md and packages/mcp-server/README.md

---

## Feature 6: CI Badges (15 minutes)

Covered by #9 above. The CI badge already exists; just need npm downloads and stars badges after publish.

---

## Feature 7: Init Wizard (`npx physical-mcp init`) — 1 Day

### #10 — Install prompts dependency
- **Status:** Pending
- **Blocks:** #11, #12
- **What:** `cd packages/mcp-server && npm install prompts && npm install -D @types/prompts`
- **Why:** Lightweight interactive prompts library (15KB, zero deps) for the CLI wizard

### #11 — Create robot safety presets (presets.ts)
- **Status:** Pending
- **Blocked by:** #10
- **Blocks:** #12, #14
- **What:** Create `packages/mcp-server/src/cli/presets.ts` with built-in robot configurations:
  - **turtlebot3:** 0.22 m/s linear, 2.84 rad/s angular, clampMode true, 5m geofence
  - **ur5:** 0.1 m/s linear, 0.5 rad/s angular, tight limits for robot arm
  - **drone:** 2.0 m/s linear, 1.0 rad/s angular, 50m geofence, 30m height limit
  - **simulation:** 0.5 m/s, 1.5 rad/s, 10m geofence
  - **custom:** null (user fills in everything)
- **Output:** Each preset is a full policy object that serializes to valid YAML

### #12 — Create interactive init wizard (init.ts)
- **Status:** Pending
- **Blocked by:** #10, #11
- **Blocks:** #13, #14
- **What:** Create `packages/mcp-server/src/cli/init.ts`. Interactive prompts flow:
  1. Robot type selection (from presets)
  2. Bridge URL (default ws://localhost:9090)
  3. Velocity limits (pre-filled from preset)
  4. Geofence enable + dimensions
  5. Deadman switch enable
  6. Tool profile (minimal/standard/full)
  7. Rate limits
- **Output files generated:**
  - `physical-mcp-policy.yaml` — safety policy
  - `physical-mcp.json` — client config for Claude Desktop / Cursor / Gemini CLI
- **Prints:** Next steps with copy-paste commands

### #13 — Wire init subcommand into index.ts
- **Status:** Pending
- **Blocked by:** #12
- **Blocks:** #14
- **What:** Add subcommand routing to `packages/mcp-server/src/index.ts`. Check `process.argv[2]` for `'init'` before starting MCP server. If init, import and run wizard, then exit.

### #14 — Write tests for init wizard and presets
- **Status:** Pending
- **Blocked by:** #11, #12, #13
- **What:** Tests for:
  - Each preset generates valid policy objects with all required fields
  - Generated YAML is valid and parseable
  - Generated JSON config has correct structure
  - Preset selection produces expected values
  - Generated YAML passes safety_validate_policy logic

---

## Feature 8: Gazebo Demo World — 1 Day

### #15 — Create Gazebo demo world SDF file
- **Status:** Pending
- **Blocks:** #17, #18, #19
- **What:** Create `docker/worlds/physical_mcp_demo.world` (SDF format)
  - 10m x 10m walled area matching default geofence (-5 to +5)
  - Visual floor markers at 1m from walls (geofence warning margin)
  - 4-6 obstacles inside (boxes, cylinders, walls)
  - Narrow corridor for collision zone demos
  - Open area for free driving
  - Good lighting for camera tool demos
  - TurtleBot3 spawn at (0, 0, 0)

### #16 — Create demo.yaml safety policy matching demo world
- **Status:** Pending
- **What:** Create `packages/mcp-server/policies/demo.yaml`
  - velocity: 0.5 m/s linear, 1.5 rad/s angular, clampMode true
  - acceleration: enabled, 1.0 m/s², 3.0 rad/s²
  - geofence: -4.5 to 4.5 (inside the walls), z 0-2
  - geofenceWarningMargin: 1.0
  - rateLimits: 10Hz publish, 60 service/min, 30 action/min
  - deadmanSwitch: disabled (for demo simplicity)

### #17 — Create ROS2 launch file for demo world
- **Status:** Pending
- **Blocked by:** #15
- **What:** Create `docker/launch/demo_world.launch.py`. Python ROS2 launch file that launches Gazebo with the custom world, spawns TurtleBot3 Burger at origin, optionally includes Nav2 stack.

### #18 — Update Dockerfile.sim to support custom Gazebo worlds
- **Status:** Pending
- **Blocked by:** #15
- **Blocks:** #19
- **What:** Modify `docker/Dockerfile.sim` to accept `GAZEBO_WORLD` env var. If set, use that world file instead of default.

### #19 — Add demo service to docker-compose.yml
- **Status:** Pending
- **Blocked by:** #15, #18
- **What:** Add `sim-demo` service variant to `docker/docker-compose.yml`. Mount `./docker/worlds:/worlds:ro`, set `GAZEBO_WORLD=/worlds/physical_mcp_demo.world`. Users run: `docker compose up sim-demo bridge`

---

## Feature 5: Web Dashboard — 2-3 Days

### #20 — Install Express dependency for dashboard
- **Status:** Pending
- **Blocks:** #21, #22
- **What:** `cd packages/mcp-server && npm install express && npm install -D @types/express`

### #21 — Create dashboard REST + SSE server (server.ts)
- **Status:** Pending
- **Blocked by:** #20
- **Blocks:** #23, #24
- **What:** Create `packages/mcp-server/src/dashboard/server.ts`. Express server with:
  - `GET /api/audit` — returns audit entries JSON (supports ?limit, ?violationsOnly, ?commandType)
  - `GET /api/audit/stats` — returns {total, allowed, blocked, errors}
  - `GET /api/audit/stream` — SSE endpoint pushing new events via SafetyEventEmitter
  - `GET /api/policy` — returns current safety policy config
  - `GET /` — serves static index.html
  - Export `startDashboard({ port, auditLogger, policyEngine })` function

### #22 — Create dashboard single-page UI (index.html)
- **Status:** Pending
- **Blocked by:** #20
- **Blocks:** #24
- **What:** Create `packages/mcp-server/src/dashboard/public/index.html`. Single HTML file with embedded CSS/JS (no build step). UI includes:
  - Header with "PhysicalMCP Dashboard" and live connection indicator
  - Stats bar: 4 cards (total, allowed, blocked, errors) updating live
  - Filter row: command type dropdown, blocked-only checkbox, search
  - Audit table: time, type, target, result (color-coded), expandable details
  - Safety score gauge (0-100)
  - Export button (download audit JSON)
  - SSE connection for real-time updates (new rows at top, no refresh)
  - Colors: green=allowed, red=blocked, yellow=clamped/warning

### #23 — Wire dashboard into index.ts with --dashboard flag
- **Status:** Pending
- **Blocked by:** #21
- **Blocks:** #24
- **What:** Add to `packages/mcp-server/src/index.ts`:
  - `--dashboard` flag (boolean, default false, env: PHYSICAL_MCP_DASHBOARD)
  - `--dashboard-port` flag (number, default 3000, env: PHYSICAL_MCP_DASHBOARD_PORT)
  - After MCP server starts, if dashboard enabled, import and call startDashboard()
  - Pass auditLogger and policyEngine instances
  - Log dashboard URL to stderr on startup

### #24 — Write tests for dashboard endpoints
- **Status:** Pending
- **Blocked by:** #21, #22, #23
- **What:** Create `packages/mcp-server/src/dashboard/server.test.ts`:
  - GET /api/audit returns valid JSON array
  - GET /api/audit?limit=5 respects limit
  - GET /api/audit?violationsOnly=true filters correctly
  - GET /api/audit/stats returns {total, allowed, blocked, errors}
  - GET /api/policy returns current policy
  - GET / returns HTML content
  - SSE stream sends events when new audit entries are logged

---

## Visibility & Growth — Week 1-2

### #25 — Record demo GIF/video for README
- **Status:** Pending
- **Blocked by:** #7
- **What:** Record 2-5 min demo showing:
  1. `docker compose up` (Gazebo + TurtleBot3)
  2. Connect Claude via MCP
  3. "List all topics" — topics appear
  4. "Move forward at 0.2 m/s" — robot moves, safety passes
  5. "Move at 5 m/s" — BLOCKED, velocity exceeded
  6. "Emergency stop" — robot halts
  7. "Show audit log" — all events visible
- Create GIF version (15-30 sec) for README hero section
- Full video for YouTube/blog
- Tools: asciinema (terminal), OBS (Gazebo window), or screen record

### #26 — Create 10-15 "good first issue" tickets on GitHub
- **Status:** Pending
- **What:** Well-scoped issues tagged "good first issue":
  1. Add UR5 robot arm safety policy (policies/ur5.yaml)
  2. Add Spot quadruped safety policy
  3. Add DJI Tello drone safety policy
  4. Add TurtleBot4 safety policy
  5. Add safety_audit_log CSV export format option
  6. Add --json flag for machine-readable CLI output
  7. Add ros2_topic_hz tool (measure topic publish rate)
  8. Add ros2_bag_info tool (read rosbag metadata)
  9. Improve error message when bridge is unreachable
  10. Add unit tests for geofence circular boundary edge cases
  11. Add Spanish translation for README
  12. Add Docker healthcheck to bridge container
  13. Add example policy for warehouse AGV use case
  14. Document how to use with Ollama (local LLM)
  15. Add ros2_lifecycle_* tools for node lifecycle management
- Each issue needs: description, expected behavior, files to modify, "good first issue" label

### #27 — Write more robot safety policies (UR5, Spot, drone)
- **Status:** Pending
- **What:** Create in `packages/mcp-server/policies/`:
  - **ur5.yaml:** Very tight velocity (0.1 m/s), small geofence (1m radius), high rate limits for control loops
  - **spot.yaml:** Moderate velocity (1.5 m/s), large geofence, acceleration limits for stairs
  - **drone.yaml:** Higher velocity (2.0 m/s), large 3D geofence (height limit critical), mandatory deadman switch
  - **turtlebot4.yaml:** Updated TurtleBot4 specs (faster than TurtleBot3)
- Each with name, description, and comments explaining the choices

### #28 — Add OpenAI/Gemini integration examples to README
- **Status:** Pending
- **What:** Add "Works With Any AI" section to README showing:
  - Claude (already there)
  - OpenAI Agents SDK (5 lines Python)
  - Gemini CLI (JSON config)
  - Cursor (mcp.json)
  - Any LLM via mcp-use (5 lines Python)
- Link to full guide at `resources/docs/openai-gemini-integration.md`

### #29 — Publish blog post on Dev.to
- **Status:** Pending
- **Blocked by:** #7
- **What:** Polish existing draft at `resources/docs/blog-post-devto.md`. Update with current stats (100 tools, 2167 tests, v1.0.0), npm install command, comparison table, safety demo. Tags: #robotics #ai #opensource #typescript

### #30 — Post "Show HN" on Hacker News
- **Status:** Pending
- **Blocked by:** #7, #25, #29
- **What:** Title: "Show HN: PhysicalMCP – Safety-first MCP server bridging AI agents to ROS2 robots (100 tools)"
- URL: https://github.com/ricardothe3rd/physical-mcp
- Be ready to answer comments for 2-3 hours (critical for HN ranking)
- Key points: safety layer differentiator, any AI model, 2167 tests, open source MIT

### #31 — Post on Reddit (r/ROS, r/robotics, r/LocalLLaMA)
- **Status:** Pending
- **Blocked by:** #7, #30
- **What:** Tailored posts to 3 subreddits:
  - **r/ROS** (85K): ROS2 integration, safety layer, policy system
  - **r/robotics** (270K): AI-to-robot bridge, open standard potential
  - **r/LocalLLaMA** (500K+): MCP compatibility, works with any LLM including local

### #32 — Post on ROS Discourse
- **Status:** Pending
- **Blocked by:** #7, #30
- **What:** Post on discourse.ros.org, General category. Focus on ROS2 integration technical depth, safety policy system, how it complements existing ROS2 tooling.

---

## Revenue Foundation

### #33 — Set up GitHub Sponsors tiers
- **Status:** Pending
- **What:** Configure at github.com/sponsors/ricardothe3rd:
  - $9/mo "Supporter" — name in README sponsors section, Discord role
  - $49/mo "Startup" — priority issue responses, early feature access
  - $149/mo "Production" — 1 hour/month architecture call, private Discord channel
  - $499/mo "Enterprise Sponsor" — logo on website/README, roadmap influence
- FUNDING.yml already has `github: [ricardothe3rd]`

### #34 — Add "PhysicalMCP Cloud" email waitlist to README
- **Status:** Pending
- **What:** Add section to README footer: "PhysicalMCP Cloud (coming soon) — managed dashboard, fleet monitoring, cloud tunnel." Link to email collection form (Google Forms, Typeform, or Mailchimp). Validates demand before building.

---

## Dependency Graph

```
#7 (npm publish) ─────────────────────────────────────────────┐
 ├── #8  (GitHub release)                                      │
 ├── #9  (dynamic badges)                                      │
 ├── #25 (demo video)                                          │
 ├── #29 (Dev.to blog)                                         │
 │    └── #30 (Hacker News) ────────────┐                     │
 │         ├── #31 (Reddit)              │                     │
 │         └── #32 (ROS Discourse)       │                     │
 └───────────────────────────────────────┘                     │
                                                                │
#10 (install prompts) ──────────────────────────────────────┐  │
 └── #11 (presets.ts)                                        │  │
      └── #12 (init.ts)                                      │  │
           └── #13 (wire into index.ts)                      │  │
                └── #14 (tests)                              │  │
                                                              │  │
#15 (Gazebo world) ─────────────────────────────────────┐    │  │
 ├── #17 (launch file)                                   │    │  │
 ├── #18 (Dockerfile update)                             │    │  │
 │    └── #19 (docker-compose)                           │    │  │
 └───────────────────────────────────────────────────────┘    │  │
                                                              │  │
#16 (demo.yaml) ── no dependencies                           │  │
                                                              │  │
#20 (install Express) ──────────────────────────────────┐    │  │
 ├── #21 (server.ts)                                     │    │  │
 │    └── #23 (wire --dashboard flag)                    │    │  │
 ├── #22 (index.html)                                    │    │  │
 │        └── #24 (tests)                                │    │  │
 └───────────────────────────────────────────────────────┘    │  │
                                                              │  │
NO DEPENDENCIES (start anytime):                              │  │
 #16 (demo.yaml)                                              │  │
 #26 (good first issues)                                      │  │
 #27 (robot policies)                                         │  │
 #28 (OpenAI/Gemini README)                                   │  │
 #33 (GitHub Sponsors)                                        │  │
 #34 (email waitlist)                                         │  │
```

---

## Summary

| Area | Tasks | Total Effort |
|------|-------|-------------|
| Blocker (npm publish) | #7, #8, #9 | 30 min |
| Feature 7: Init Wizard | #10, #11, #12, #13, #14 | 1 day |
| Feature 8: Gazebo Demo | #15, #16, #17, #18, #19 | 1 day |
| Feature 5: Dashboard | #20, #21, #22, #23, #24 | 2-3 days |
| Visibility & Growth | #25, #26, #27, #28, #29, #30, #31, #32 | 2-3 days |
| Revenue Foundation | #33, #34 | 1 hour |
| **Total** | **28 tasks** | **~7-8 days** |

---

*Last updated: 2026-02-23*

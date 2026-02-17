# PhysicalMCP - Remaining Tasks

## Status Overview

| Task | Status | Details |
|------|--------|---------|
| #1 Monorepo scaffold | DONE | GitHub repo, all dirs, package.json, tsconfig, .gitignore, CLAUDE.md |
| #2 WebSocket protocol | DONE | protocol.ts, protocol.py, ws-client.ts, connection-manager.ts |
| #3 MCP server entry point | DONE | index.ts + 5 tool files (topic, service, action, safety, system) |
| #4 Python ROS2 bridge | DONE | bridge_node.py, discovery.py, topic/service/action handlers, telemetry |
| #5 Safety layer | DONE | policy-engine.ts, geofence.ts, rate-limiter.ts, audit-logger.ts, policy-loader.ts, types.ts, default.yaml, turtlebot3.yaml |
| #6 Docker setup | DONE | Dockerfile.bridge, Dockerfile.sim, docker-compose.yml |
| #7 README + Publish | NOT STARTED | README.md, demo script, npm publish |
| TypeScript build | BLOCKED | tsc compilation hangs/fails - needs debugging |

---

## Files Created (38 total)

### TypeScript MCP Server (packages/mcp-server/)
- `package.json` - npm package config (@chinchillaenterprises/physical-mcp)
- `tsconfig.json` - TypeScript config (ES2022, strict)
- `src/index.ts` - Main MCP server entry with 21 tool registrations
- `src/bridge/protocol.ts` - WebSocket protocol types + zod schemas
- `src/bridge/ws-client.ts` - WebSocket client with request/response matching
- `src/bridge/connection-manager.ts` - Auto-reconnect + circuit breaker
- `src/tools/topic-tools.ts` - 5 topic tools (list, info, subscribe, publish, echo)
- `src/tools/service-tools.ts` - 3 service tools (list, info, call)
- `src/tools/action-tools.ts` - 4 action tools (list, send_goal, cancel, status)
- `src/tools/safety-tools.ts` - 7 safety tools (e-stop, status, policy, audit, etc.)
- `src/tools/system-tools.ts` - 2 system tools (bridge_status, node_list)
- `src/safety/types.ts` - Safety type definitions
- `src/safety/policy-engine.ts` - Core safety evaluation (velocity, geofence, rate, e-stop)
- `src/safety/policy-loader.ts` - YAML policy parser with defaults
- `src/safety/geofence.ts` - Workspace boundary checking
- `src/safety/rate-limiter.ts` - Per-topic/service rate limiting
- `src/safety/audit-logger.ts` - Command audit trail
- `src/types/ros.ts` - ROS2 type definitions
- `src/utils/error-recovery.ts` - Retry + circuit breaker utilities
- `policies/default.yaml` - Conservative default safety policy
- `policies/turtlebot3.yaml` - TurtleBot3 Gazebo demo policy

### Python ROS2 Bridge (packages/ros2-bridge/)
- `physical_mcp_bridge/__init__.py` - Package init
- `physical_mcp_bridge/protocol.py` - Shared protocol (mirrors TS)
- `physical_mcp_bridge/bridge_node.py` - Main rclpy node + WebSocket server
- `physical_mcp_bridge/discovery.py` - ROS2 graph discovery
- `physical_mcp_bridge/topic_handler.py` - Topic subscribe/publish/echo
- `physical_mcp_bridge/service_handler.py` - Service call handler
- `physical_mcp_bridge/action_handler.py` - Action goal handler
- `physical_mcp_bridge/telemetry.py` - Bridge health metrics
- `requirements.txt` - Python dependencies
- `setup.py` - Package setup

### Docker
- `docker/Dockerfile.bridge` - ROS2 Humble + bridge
- `docker/Dockerfile.sim` - Gazebo + TurtleBot3
- `docker/docker-compose.yml` - Full stack compose

### Root
- `.gitignore`
- `CLAUDE.md` - Project context for Claude
- `LICENSE` - MIT
- `resources/docs/week1-build-plan.md` - Original build plan

---

## Remaining Tasks

### 1. Fix TypeScript Build (CRITICAL - BLOCKER)
**Priority:** P0 - Everything else depends on this

The `tsc` compiler hangs when running. Need to:
- [ ] Debug why `tsc` hangs (likely a type inference loop or circular dependency)
- [ ] Run `tsc --listFiles` to check what files are being processed
- [ ] Check for circular imports between tool files and safety modules
- [ ] Fix `zodToJsonSchema` return type casting issues
- [ ] Verify `@modelcontextprotocol/sdk` type imports resolve correctly
- [ ] Get clean `npm run build` with no errors
- [ ] Verify `dist/index.js` is generated and executable

### 2. Write Unit Tests
**Priority:** P1 - Validates safety layer works

Tests that run WITHOUT ROS2 (on macOS):
- [ ] `safety/policy-engine.test.ts` - Test all violation types:
  - Velocity exceeded (linear + angular)
  - Blocked topic/service
  - Rate limit exceeded
  - Emergency stop active
  - Allowed commands pass through
- [ ] `safety/geofence.test.ts` - Boundary checks:
  - Inside bounds = ok
  - Outside X/Y/Z = violation
  - Edge cases (exact boundary)
- [ ] `safety/rate-limiter.test.ts` - Rate limiting:
  - Under limit = ok
  - Over limit = violation
  - Window expiry resets count
- [ ] `safety/policy-loader.test.ts` - YAML parsing:
  - Load default.yaml
  - Load turtlebot3.yaml
  - Missing file = use defaults
  - Partial YAML merges with defaults
- [ ] `bridge/protocol.test.ts` - Serialization:
  - Valid command parse
  - Invalid JSON rejection
  - Response building
- [ ] `safety/audit-logger.test.ts` - Audit trail:
  - Entries logged correctly
  - Filter by violations only
  - Stats calculation
  - Max entries cap

### 3. Write README.md
**Priority:** P1 - First thing visitors see

- [ ] Project title + one-liner description
- [ ] Architecture diagram (ASCII or mermaid)
  - Show: Claude -> MCP Server -> Safety Layer -> WebSocket -> Bridge -> ROS2
- [ ] Quick start guide:
  1. `npm install -g @chinchillaenterprises/physical-mcp`
  2. `docker compose up` (sim + bridge)
  3. `claude mcp add physical-mcp npx @chinchillaenterprises/physical-mcp`
  4. Ask Claude to list topics
- [ ] Tool reference table (21 tools with descriptions)
- [ ] Safety features section (THE differentiator):
  - Velocity limits
  - Geofence
  - Rate limiting
  - Emergency stop
  - Audit logging
- [ ] Configuration section (env vars, policy YAML)
- [ ] Competitor comparison table:
  | Feature | PhysicalMCP | robotmcp | wise-vision |
  |---------|-------------|----------|-------------|
  | Safety Layer | YES | No | No |
  | E-Stop | YES | No | No |
  | Audit Log | YES | No | No |
  | Geofence | YES | No | No |
  | Rate Limiting | YES | No | No |
- [ ] License (MIT)
- [ ] Contributing guide

### 4. Create Demo Script
**Priority:** P2 - Shows the product in action

File: `demo/safety-demo.md`
- [ ] Demo scenario walkthrough:
  1. Start sim + bridge: `docker compose up`
  2. Connect Claude to PhysicalMCP
  3. **Safe operation**: List topics, read /odom, publish safe velocity to /cmd_vel
  4. **Safety block**: Try to publish velocity > limit (BLOCKED with clear message)
  5. **E-stop**: Activate emergency stop, try to publish (BLOCKED)
  6. **E-stop release**: Release with confirmation, publish works again
  7. **Audit trail**: View all commands and violations
- [ ] Record terminal demo / GIF showing the above

### 5. Publish to npm
**Priority:** P2 - Makes it installable

- [ ] Verify `npm run build` produces clean output
- [ ] Test `npx @chinchillaenterprises/physical-mcp` works
- [ ] `npm publish --access public`
- [ ] Verify install: `claude mcp add physical-mcp npx @chinchillaenterprises/physical-mcp`

### 6. Initial Git Commit + Push
**Priority:** P1 - Get code on GitHub

- [ ] Review all files for secrets/credentials (none expected)
- [ ] `git add` all project files
- [ ] Commit with descriptive message
- [ ] Push to `ricardothe3rd/physical-mcp`

### 7. Docker Testing (Requires Linux/Docker)
**Priority:** P3 - Integration validation

- [ ] `docker build` both images
- [ ] `docker compose up` full stack
- [ ] Verify bridge connects to ROS2
- [ ] Verify MCP server connects to bridge
- [ ] Run through demo scenario end-to-end

---

## Execution Order

```
1. Fix TypeScript Build     [P0] ← everything depends on this
2. Initial Git Commit       [P1] ← get code saved
3. Write Unit Tests         [P1] ← validate safety layer
4. Write README.md          [P1] ← first impressions
5. Create Demo Script       [P2] ← show it works
6. Publish to npm           [P2] ← make it installable
7. Docker Testing           [P3] ← integration validation
```

---

## Known Issues

1. **tsc hangs** - TypeScript compiler doesn't complete. Likely causes:
   - `zodToJsonSchema` return type inference is expensive
   - Circular imports between tools and safety modules
   - `@modelcontextprotocol/sdk` type resolution

2. **psutil dependency** - `telemetry.py` imports `psutil` but it's not in requirements.txt. Either add it or remove the import.

3. **Python type hints** - Some Python files use `list[dict]` syntax which requires Python 3.9+. The setup.py requires 3.10+ which covers this.

---

## Revenue Milestones (Post Launch)

| Milestone | Target | Revenue |
|-----------|--------|---------|
| 100 GitHub stars | Week 2-3 | $0 (credibility) |
| First npm install by stranger | Week 1-2 | $0 (validation) |
| Cloud relay beta | Month 2 | First paid user |
| 10 paying teams | Month 3-4 | ~$500/mo |
| Enterprise pilot | Month 5-6 | ~$2K+/mo |

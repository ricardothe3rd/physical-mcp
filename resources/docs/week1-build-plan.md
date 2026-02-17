# PhysicalMCP - Week 1 Build Plan

## Context

**Problem:** AI agents (Claude, GPT, etc.) can call APIs and write code but cannot control physical devices. The infrastructure connecting AI agents to robots does not exist as a product. Existing open-source ROS2-MCP bridges (robotmcp ~1K stars, wise-vision ~65 stars) are hobby projects with NO safety features, NO cloud/remote access, NO monetization, and NO company behind them.

**Solution:** PhysicalMCP - a safety-first MCP server that bridges AI agents to ROS2 robots. The killer differentiator is the **safety layer** (velocity limits, geofence, rate limiting, e-stop, audit logging) that no competitor has.

**Architecture:** Hybrid TypeScript MCP server + Python ROS2 bridge agent, communicating via WebSocket. Safety enforcement happens in the TS server before commands reach the robot.

**Week 1 Goal:** Ship an open-source MVP on GitHub + npm with a Gazebo demo showing Claude controlling a TurtleBot3 AND being blocked by safety constraints.

---

## Monorepo Structure

```
physical-mcp/
├── packages/
│   ├── mcp-server/                   # TypeScript MCP server (npm package)
│   │   ├── src/
│   │   │   ├── index.ts              # Server entry, tool registration
│   │   │   ├── tools/
│   │   │   │   ├── topic-tools.ts    # list, subscribe, publish, echo
│   │   │   │   ├── service-tools.ts  # list, call
│   │   │   │   ├── action-tools.ts   # list, send_goal, cancel, status
│   │   │   │   ├── safety-tools.ts   # e-stop, status, policy, audit
│   │   │   │   └── system-tools.ts   # bridge status, node list, info
│   │   │   ├── safety/
│   │   │   │   ├── policy-engine.ts  # Core safety evaluation
│   │   │   │   ├── policy-loader.ts  # YAML config parser
│   │   │   │   ├── geofence.ts       # Workspace boundary checks
│   │   │   │   ├── rate-limiter.ts   # Command rate limiting
│   │   │   │   ├── audit-logger.ts   # Command audit trail
│   │   │   │   └── types.ts          # Safety type definitions
│   │   │   ├── bridge/
│   │   │   │   ├── ws-client.ts      # WebSocket client to Python bridge
│   │   │   │   ├── protocol.ts       # Message protocol types/schemas
│   │   │   │   └── connection-manager.ts
│   │   │   ├── types/
│   │   │   │   └── ros.ts            # ROS2 type definitions
│   │   │   └── utils/
│   │   │       └── error-recovery.ts # Circuit breaker (from mcp-ai-assistant)
│   │   ├── policies/
│   │   │   ├── default.yaml          # Conservative default policy
│   │   │   └── turtlebot3.yaml       # TurtleBot3 demo policy
│   │   ├── package.json
│   │   └── tsconfig.json
│   │
│   └── ros2-bridge/                  # Python ROS2 bridge agent
│       ├── physical_mcp_bridge/
│       │   ├── __init__.py
│       │   ├── bridge_node.py        # rclpy node + WebSocket server
│       │   ├── topic_handler.py      # Topic pub/sub
│       │   ├── service_handler.py    # Service calls
│       │   ├── action_handler.py     # Action goals
│       │   ├── discovery.py          # Node/topic/service discovery
│       │   ├── protocol.py           # Shared protocol (mirrors TS)
│       │   └── telemetry.py          # Periodic status reporting
│       ├── setup.py
│       └── requirements.txt
│
├── docker/
│   ├── Dockerfile.bridge             # ROS2 Humble + bridge
│   ├── Dockerfile.sim                # Gazebo + TurtleBot3
│   └── docker-compose.yml            # Full stack
│
├── resources/
│   └── docs/
│       └── week1-build-plan.md       # This file
│
├── demo/
│   └── safety-demo.md                # Demo walkthrough script
├── README.md
├── LICENSE                           # MIT
└── CLAUDE.md
```

---

## Key Files to Reuse from Existing Codebase

| What | Source | How to Reuse |
|------|--------|-------------|
| MCP server pattern | `mcp-ai-assistant/src/index.ts` | Copy server setup, tool registration, and switch-based dispatch pattern |
| Error recovery | `mcp-ai-assistant/src/utils/error-recovery.ts` | Copy circuit breaker + retry logic for bridge connection |
| tsconfig.json | `mcp-ai-assistant/tsconfig.json` | Copy directly (ES2022, strict, declarations) |
| package.json structure | `mcp-ai-assistant/package.json` | Adapt: swap elevenlabs/express deps for ws/yaml |
| Safety YAML patterns | `skating-drone/config/config.yaml` | Inspire safety policy YAML schema (geofence, velocity limits) |
| Python patterns | `ios-keymapper/mapper.py` | State machine, threading, type hints, error handling patterns |

---

## MCP Tools (21 total)

### Topic Tools (5)
- `ros2_topic_list` - List all topics with types
- `ros2_topic_info` - Get topic details
- `ros2_topic_subscribe` - Collect N messages
- `ros2_topic_publish` - Publish message (**SAFETY CHECKED**)
- `ros2_topic_echo` - Get latest message

### Service Tools (3)
- `ros2_service_list` - List all services
- `ros2_service_info` - Get service details
- `ros2_service_call` - Call a service (**SAFETY CHECKED**)

### Action Tools (4)
- `ros2_action_list` - List action servers
- `ros2_action_send_goal` - Send goal (**SAFETY CHECKED**)
- `ros2_action_cancel` - Cancel goal
- `ros2_action_status` - Get goal status

### Safety Tools (7) - THE DIFFERENTIATOR
- `safety_status` - Current safety system status
- `safety_emergency_stop` - Activate e-stop
- `safety_emergency_stop_release` - Release e-stop
- `safety_get_policy` - Get current policy config
- `safety_update_velocity_limits` - Adjust velocity limits
- `safety_update_geofence` - Adjust workspace bounds
- `safety_audit_log` - View command audit trail

### System Tools (2)
- `system_bridge_status` - Bridge connection health
- `system_node_list` - List ROS2 nodes

---

## Safety Policy (YAML)

The default policy enforces:
- **Velocity limits**: linear max 0.5 m/s, angular max 1.5 rad/s
- **Geofence**: 10m x 10m rectangular boundary
- **Rate limiting**: 10 Hz publish, 60 service calls/min
- **Blocked topics**: /rosout, /parameter_events
- **Blocked services**: /kill, /shutdown
- **Emergency stop**: publishes zero-velocity to /cmd_vel + cancels all actions
- **Audit logging**: all commands + all violations

Safety is enforced in the TypeScript server BEFORE commands reach the Python bridge. The bridge has a secondary e-stop flag as a fail-safe.

---

## Day-by-Day Schedule

### Day 1: Project Scaffold + WebSocket Ping
- [ ] Create GitHub repo `ricardothe3rd/physical-mcp`
- [ ] Set up monorepo: TS server scaffold (package.json, tsconfig.json, src/index.ts)
- [ ] Set up Python bridge scaffold (bridge_node.py with websockets)
- [ ] Define WebSocket protocol (protocol.ts + protocol.py)
- [ ] Get ping/pong working: MCP server -> WebSocket -> bridge -> pong

### Day 2: Docker + Bridge Connection
- [ ] Dockerfile.bridge (ROS2 Humble + Python bridge)
- [ ] Dockerfile.sim (Gazebo + TurtleBot3)
- [ ] docker-compose.yml (sim + bridge, network_mode: host)
- [ ] Connection manager with reconnection + circuit breaker
- [ ] Verify: bridge in Docker, TS server on Mac, ping works

### Day 3: Discovery + Subscribe Tools
- [ ] Python: discovery.py (list topics, services, nodes via rclpy)
- [ ] Python: topic_handler.py (subscribe, echo)
- [ ] TS: system_bridge_status, system_node_list tools
- [ ] TS: ros2_topic_list, ros2_topic_subscribe, ros2_topic_echo
- [ ] Test: Claude lists topics and reads /odom from TurtleBot3

### Day 4: Publish + Service + Action Tools
- [ ] Python: topic publish, service_handler.py, action_handler.py
- [ ] TS: ros2_topic_publish, ros2_service_list, ros2_service_call
- [ ] TS: ros2_action_list, ros2_action_send_goal, ros2_action_cancel
- [ ] Test: Claude moves TurtleBot3 via /cmd_vel publish

### Day 5: Safety Layer Core
- [ ] TS: safety types, policy-loader (YAML), policy-engine
- [ ] TS: velocity limit checking, rate limiter, geofence
- [ ] TS: audit-logger
- [ ] Wire safety into ros2_topic_publish handler
- [ ] Write default.yaml + turtlebot3.yaml policies
- [ ] Test: publish at safe speed = ALLOWED, publish at 10 m/s = BLOCKED

### Day 6: Safety Tools + Demo
- [ ] TS: All 7 safety_ tools
- [ ] Emergency stop: zero-velocity publish + action cancellation
- [ ] Create demo scenario script (safe move -> blocked move -> e-stop -> release)
- [ ] Record demo video showing safety in action
- [ ] Polish safety violation response messages

### Day 7: README + Publish
- [ ] Write comprehensive README (architecture diagram, quickstart, tool reference)
- [ ] Comparison table vs competitors (safety column highlighted)
- [ ] `npm publish` as @chinchillaenterprises/physical-mcp
- [ ] Push Docker images
- [ ] Post to: Reddit r/ROS, ROS Discourse, Twitter/X, Hacker News

---

## Testing Strategy

### Unit Tests (no ROS needed, run on Mac)
- Safety policy engine: test all violation types
- Policy loader: YAML parsing
- Geofence: boundary checks
- Rate limiter: burst + sustained
- Protocol: serialization/deserialization
- Connection manager: mock WebSocket

### Integration Tests (Docker)
- docker-compose up sim + bridge
- TS server connects to bridge
- Each MCP tool called and verified
- Safety violations correctly generated

---

## Revenue Path (Post Week 1)

| Phase | Timeline | What | Revenue |
|-------|----------|------|---------|
| Open Source Launch | Week 1-4 | Free: local server + safety + bridge | Stars, community, credibility |
| Cloud Relay | Month 2-3 | Remote robot access via hosted WebSocket relay | Free (1 robot) / $29/mo (5 robots) / $99/mo (20 robots) |
| Enterprise | Month 4-6 | On-prem relay, SSO, compliance, SLA | $500+/mo per fleet |
| Ecosystem | Month 6+ | Safety policy marketplace, dashboard, certifications | Marketplace fees + SaaS |

The safety layer IS the moat. Once companies embed PhysicalMCP safety policies into their workflow, switching costs are high (configs, audit history, compliance docs).

---

## Verification

After Week 1, success looks like:

1. **`npx @chinchillaenterprises/physical-mcp`** starts the MCP server
2. **`docker compose up`** starts Gazebo TurtleBot3 + Python bridge
3. **Claude can**: list topics, subscribe to /odom, publish to /cmd_vel, call services, send action goals
4. **Claude CANNOT**: exceed velocity limits, publish to blocked topics, bypass e-stop, exceed rate limits
5. **Every command is audit-logged** with timestamp, type, target, safety result
6. **Demo video** shows the safety layer blocking dangerous commands
7. **README** with architecture diagram, quickstart, comparison table
8. **Published on npm** and installable via `claude mcp add`

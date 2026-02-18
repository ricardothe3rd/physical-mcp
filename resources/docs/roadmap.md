# PhysicalMCP Roadmap

> Last updated: 2026-02-18

This document outlines the planned development of PhysicalMCP. Priorities are based on user feedback — open an issue or discussion to influence what gets built next.

---

## v0.1.0 — MVP (Current Release)

- [x] 21 MCP tools (topics, services, actions, safety, system)
- [x] Safety layer: velocity limits, geofence, rate limiting, blocked topics/services
- [x] Emergency stop (dual-layer: server + bridge)
- [x] Full audit logging
- [x] YAML safety policies
- [x] WebSocket protocol with heartbeat and auto-reconnect
- [x] Python ROS2 bridge (rclpy + websockets)
- [x] Docker setup (Gazebo TurtleBot3 + bridge)
- [x] 75 unit tests
- [x] CLI flags (--help, --version, --policy, --bridge-url, --verbose)

---

## v0.2.0 — Hardening

Target: 2-3 weeks after v0.1.0

### Safety Enhancements
- [ ] **Deadman switch** — Require periodic heartbeat from AI agent; auto e-stop if silent for configurable duration
- [ ] **Velocity clamping mode** — Option to reduce velocities to max instead of blocking entirely
- [ ] **Workspace zones** — Named zones with different speed limits (e.g., "loading_dock" = 0.1 m/s max)
- [ ] **Command queue introspection** — See what commands are pending in the bridge
- [ ] **Safety event webhooks** — HTTP callbacks on violations, e-stop events

### Testing & Quality
- [ ] 90%+ code coverage with vitest
- [ ] Connection manager tests with mock WebSocket
- [ ] WebSocket client tests with mock server
- [ ] Python bridge unit tests (pytest)
- [ ] Integration test suite (Docker-based, end-to-end)
- [ ] Fuzz testing for protocol parser

### Developer Experience
- [ ] JSDoc comments on all public APIs
- [ ] API reference documentation (auto-generated)
- [ ] Architecture deep-dive document
- [ ] Safety policy authoring guide
- [ ] WebSocket protocol specification
- [ ] "Good first issue" labels for new contributors

---

## v0.3.0 — Multi-Robot & Observability

Target: 4-6 weeks after v0.1.0

### Multi-Robot Support
- [ ] **Named robot connections** — Connect to multiple bridges simultaneously
- [ ] **Per-robot policies** — Different safety limits per robot
- [ ] **Robot selector tool** — Switch active robot context within a conversation
- [ ] **Fleet status overview** — See all connected robots and their states

### Observability
- [ ] **Structured logging** — JSON log output with configurable levels
- [ ] **Metrics export** — Prometheus-compatible metrics endpoint
- [ ] **OpenTelemetry traces** — Distributed tracing across MCP server → bridge → ROS2
- [ ] **Health check endpoint** — HTTP endpoint for monitoring tools

### Protocol Improvements
- [ ] **Streaming subscriptions** — Long-lived topic subscriptions via MCP resources
- [ ] **Binary message support** — Efficient transfer of sensor data (images, point clouds)
- [ ] **Protocol versioning** — Backward-compatible protocol negotiation

---

## v0.4.0 — Cloud Relay (SaaS)

Target: 2-3 months after v0.1.0

### Core Cloud Features
- [ ] **Hosted WebSocket relay** — Connect to robots from anywhere without VPN
- [ ] **Authentication** — API key or OAuth2 for relay access
- [ ] **TLS encryption** — End-to-end encrypted connections
- [ ] **Connection multiplexing** — Multiple AI agents per robot (with conflict resolution)

### Dashboard (Web UI)
- [ ] **Real-time robot status** — Live view of connected robots, topics, and sensor data
- [ ] **Safety event timeline** — Visual audit trail with filtering
- [ ] **Policy editor** — Edit safety policies in the browser with validation
- [ ] **Session replay** — Replay past AI-robot interactions

### Pricing Tiers
- **Free**: 1 robot, local connections only
- **Pro ($29/mo)**: 5 robots, cloud relay, dashboard
- **Team ($99/mo)**: 20 robots, SSO, priority support
- **Enterprise**: Custom limits, on-prem relay, SLA, compliance

---

## v0.5.0 — Enterprise & Safety Certification

Target: 4-6 months after v0.1.0

### Enterprise Features
- [ ] **SSO / SAML integration** — Corporate identity provider support
- [ ] **Role-based access control (RBAC)** — Define who can do what per robot
- [ ] **Compliance export** — Generate safety reports for regulatory submissions
- [ ] **Audit trail retention** — Configurable retention policies with backup
- [ ] **On-premise relay** — Self-hosted cloud relay option

### Safety Certification
- [ ] **Formal safety analysis** — FMEA (Failure Mode and Effects Analysis) document
- [ ] **IEC 61508 / ISO 13849 alignment** — Document how PhysicalMCP maps to industrial safety standards
- [ ] **Safety integrity level (SIL) assessment** — Target SIL 1 for the software safety layer
- [ ] **Third-party audit** — Independent review of the safety architecture

### Advanced Safety Features
- [ ] **Collision avoidance integration** — Subscribe to proximity sensors and auto e-stop
- [ ] **Path planning constraints** — Validate navigation goals against known obstacles
- [ ] **Force/torque limits** — Safety limits for manipulation tasks
- [ ] **Operator override** — Human-in-the-loop approval for high-risk commands
- [ ] **Safety interlock protocol** — Formal handshake before entering autonomous mode

---

## v1.0.0 — Production Ready

Target: 6-12 months after v0.1.0

### Stability & Performance
- [ ] **Battle-tested** with 100+ community deployments
- [ ] **Performance benchmarks** — Documented latency and throughput characteristics
- [ ] **Graceful degradation** — Defined behavior for every failure mode
- [ ] **Backward compatibility guarantee** — Semantic versioning contract

### Ecosystem
- [ ] **Policy marketplace** — Community-contributed safety policies for popular robots
- [ ] **Plugin system** — Custom safety checks written in TypeScript or Python
- [ ] **CI/CD integration** — Safety policy testing in deployment pipelines
- [ ] **Simulation-first workflow** — Test in sim, promote to hardware with policy swap

### Robot Platform Support
- [ ] TurtleBot3 (included from v0.1.0)
- [ ] TurtleBot4
- [ ] Universal Robots (UR3/UR5/UR10)
- [ ] Boston Dynamics Spot
- [ ] Franka Emika Panda
- [ ] Clearpath Jackal / Husky
- [ ] DJI drones (via ROS2 bridge)
- [ ] Custom robots (via policy authoring guide)

---

## Long-Term Vision

PhysicalMCP aims to become the **standard safety layer** for AI-to-robot communication. As AI agents become more capable and autonomous, the need for enforceable safety constraints grows exponentially.

The end state:
- Every robot fleet using AI agents runs PhysicalMCP (or something built on it)
- Safety policies are as standard as SSL certificates — you don't deploy without them
- The audit trail is a compliance requirement, not an optional feature
- PhysicalMCP is to robot safety what Let's Encrypt is to web security: free, ubiquitous, and non-negotiable

---

## How to Influence This Roadmap

1. **Star the repo** — Shows demand and helps prioritize
2. **Open an issue** — Feature requests, bug reports, and questions all help
3. **Comment on this roadmap** — Tell us what you need most
4. **Contribute** — PRs welcome, especially for robot-specific safety policies
5. **Share your use case** — Real-world stories help us prioritize the right features

GitHub: [github.com/ricardothe3rd/physical-mcp](https://github.com/ricardothe3rd/physical-mcp)

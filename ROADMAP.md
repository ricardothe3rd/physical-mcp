# PhysicalMCP Roadmap

## Current: v0.1.x (MVP)

- [x] 40 MCP tools (topic/service/action/safety/system/batch/recording)
- [x] Safety layer: velocity limits, geofence, rate limiting, e-stop
- [x] Acceleration limits and geofence proximity warnings
- [x] Per-topic velocity overrides
- [x] Command approval (human-in-the-loop)
- [x] Safety score tracking
- [x] Safety event pub/sub
- [x] Deadman switch
- [x] Batch command execution
- [x] Topic recording
- [x] Robot state machine
- [x] Policy hot-reload
- [x] Policy validation
- [x] Configurable logger
- [x] CLI flags (--bridge-url, --policy, --verbose, --version)
- [x] Startup self-test
- [x] 400+ tests
- [x] Docker compose (Gazebo + TurtleBot3)
- [x] GitHub CI/CD

## v0.2.0 — Quality & Polish

- [ ] Code coverage > 90%
- [ ] ESLint strict mode passing
- [ ] Python ruff linting
- [ ] Snapshot tests for all tool responses
- [ ] Input validation for all tool parameters
- [ ] Safety policy inheritance (base + override)
- [ ] Time-based policies (different limits day vs night)
- [ ] Configurable violation mode: "warn" vs "block"
- [ ] TF2 tree visualization tool
- [ ] Diagnostics aggregator tool
- [ ] URDF viewer tool
- [ ] Launch file management tools
- [ ] MessagePack binary protocol option
- [ ] WebSocket compression

## v0.3.0 — Multi-Robot & Fleet

- [ ] Multi-robot safety isolation (per-robot policies)
- [ ] Fleet status dashboard tool
- [ ] Namespace-aware tool routing
- [ ] Tool grouping profiles (minimal, standard, full)
- [ ] Cross-robot command coordination
- [ ] Position tracking via /odom for real-time geofence

## v1.0.0 — Cloud Relay

- [ ] Hosted WebSocket relay server
- [ ] API key authentication
- [ ] Multi-tenant isolation
- [ ] Usage metering
- [ ] Web dashboard for fleet management
- [ ] Stripe billing integration

### Pricing (Planned)

| Tier | Robots | Price |
|------|--------|-------|
| Free | 1 | $0/mo |
| Pro | 5 | $29/mo |
| Team | 20 | $99/mo |
| Enterprise | Unlimited | Custom |

## v1.x — Enterprise

- [ ] SSO/SAML integration
- [ ] Compliance reporting (audit log exports, retention)
- [ ] Safety policy marketplace
- [ ] On-premise relay deployment
- [ ] Custom safety certifications
- [ ] SLA guarantees

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for how to get involved. All contributions welcome.

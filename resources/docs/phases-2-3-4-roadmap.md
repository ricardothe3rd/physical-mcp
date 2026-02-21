# PhysicalMCP: Phases 2-4 Roadmap

> Phase 1 (Protocol + SDK) is complete. This document captures the plan for Phases 2-4 for future reference.

---

## Phase 2: The Cloud Bridge (Target: Months 3-5)

### Secure Tunnel: AI Agent <-> Physical Device Over Internet
- Cloud relay server (WebSocket proxy with TLS)
- Device registration + unique device IDs
- Tunnel authentication (API keys + JWT)
- NAT traversal so robots behind firewalls can connect out
- **Why it matters:** Currently local-only (ws://localhost:9090). Cloud tunnel lets an AI agent in the cloud control a robot anywhere.

### Real-time Streaming: Sensor Data, Camera Feeds, Telemetry
- Cloud-accessible sensor data relay (build on existing `sensor-tools.ts`)
- Video/camera feed relay (build on existing `camera-tools.ts`)
- Telemetry aggregation service (build on existing `telemetry.py`)
- Data compression for bandwidth efficiency
- **What exists locally:** sensor-tools, camera-tools, telemetry module - all work over local WebSocket

### Auth + Permissions: Who Can Control What
- API key management (per-device, per-user)
- User accounts + team management
- Role-based access control (RBAC) - viewer/operator/admin
- Multi-tenant isolation
- **What exists locally:** command-approval.ts (human-in-loop), tool-profiles.ts (minimal/standard/full)

### Dashboard: Monitor All Connected Devices + Agent Actions
- Web UI (Next.js) for fleet overview
- Real-time audit event display (build on `audit-export.ts`)
- Device health monitoring
- Policy configuration UI (visual YAML editor)
- Alert routing (Slack, PagerDuty, email on safety violations)
- **What exists locally:** audit-export.ts, fleet-tools.ts, multi-robot.ts

---

## Phase 3: Device Library + Marketplace (Target: Months 5-8)

### Pre-built Integrations
- DJI drone SDK integration (Tello, Mini series)
- Common robot arms (UR5, Franka Emika, xArm)
- IoT hub bridges (Home Assistant, MQTT)
- Arduino/ESP32 microcontroller bridge
- AGV/AMR platforms (Clearpath, MiR)

### Skills Marketplace
- Reusable physical capabilities as installable packages
- Examples: "pick-and-place", "navigate-to-waypoint", "visual-scan", "follow-person"
- Community-contributed skills with review/rating
- Versioned skill definitions with safety metadata
- **Foundation exists:** tool-profiles.ts has concept of tool grouping

### Community Contributions
- Plugin architecture for third-party device drivers
- Skill template generator CLI
- Documentation for adding new devices
- **Foundation exists:** CONTRIBUTING.md, GOVERNANCE.md, issue templates

### Enterprise Features
- Fleet management dashboard (build on fleet-tools.ts)
- Compliance reporting (ISO 26262, IEC 61508 readiness)
- SSO/SAML authentication
- On-premise deployment option
- SLA-backed support tiers
- **Foundation exists:** audit-export.ts, policy-versioning concept

---

## Phase 4: Revenue (Target: Month 6+)

### Pricing Model

| Tier | Price | What's Included |
|------|-------|-----------------|
| **Open Source** | Free | npm package, local safety layer, unlimited robots, all 70 tools |
| **Cloud Starter** | $29/robot/month | Cloud tunnel, dashboard, 30-day audit retention, email alerts |
| **Cloud Pro** | $149/robot/month | Fleet management, RBAC, 1-year audit retention, Slack/PD alerts |
| **Enterprise** | Custom ($500+/robot) | On-prem, SSO, compliance reports, SLA, dedicated support |

### Revenue Streams
1. **Cloud Bridge SaaS** ($29-499/robot/month) - Primary revenue
2. **GitHub Sponsors** ($9-499/month tiers) - Community support + early access
3. **Enterprise contracts** ($1K+/month) - Custom deployments
4. **Marketplace cut** (20%) - On skill/integration sales (Phase 3+)

### Comparable Company Benchmarks
- **Foxglove** (robotics data platform): $18-90/user/month, raised $58.7M
- **Balena** (IoT fleet): $99-2999/month, per-device pricing
- **ThingsBoard** (IoT platform): Free core + cloud SaaS tiers
- **PickNik Robotics** (MoveIt): ~$12.1M ARR from open-core model

### GitHub Sponsors Tiers
| Tier | Price | Perks |
|------|-------|-------|
| Supporter | $9/mo | Name in README, Discord role |
| Startup | $49/mo | Priority issues, early feature access |
| Production | $149/mo | 1hr/month architecture call, private channel |
| Enterprise Sponsor | $499/mo | Logo on website, roadmap influence |

### Sponsorware Strategy
Build one premium feature in private (e.g., Fleet Safety Dashboard), announce publicly: "Goes open-source when we hit 50 sponsors." This simultaneously drives revenue, community engagement, and marketing.

### Investment Path (If Desired)
- **Seed ($1-3M):** At 500+ stars, 1K+/week npm downloads, 2-3 design partners
- **Series A ($5-15M):** At $500K-1M ARR, 3+ enterprise customers, NRR >110%
- **Key investors for this space:** Lemnos, Eclipse Ventures, Amplify Partners, Bloomberg Beta, boldstart ventures

### Realistic Revenue Timeline
| Month | Revenue | Focus |
|-------|---------|-------|
| 1-3 | $0-500/mo | Stars, npm downloads, community |
| 3-6 | $500-5K/mo | Sponsors + first cloud customers |
| 6-12 | $5K-20K/mo | Cloud SaaS + enterprise outreach |
| 12+ | $20K+/mo | Raise seed or ramen profitable |

---

## Current Phase 1 Status (Complete)

- 70 MCP tools across 17 categories
- 25 safety modules (velocity, geofence, e-stop, deadman, audit, collision, approval)
- 2167 passing tests across 83 test files
- Docker + Gazebo simulation
- Python ROS2 bridge with secondary e-stop
- Full documentation suite
- CI/CD pipelines (GitHub Actions)
- Published to npm as `@ricardothe3rd/physical-mcp`

---

*Last updated: 2026-02-21*

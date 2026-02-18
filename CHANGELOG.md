# Changelog

All notable changes to PhysicalMCP will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-02-18

### Added
- Initial release of PhysicalMCP
- TypeScript MCP server with 21 tools
  - 5 topic tools (list, info, subscribe, publish, echo)
  - 3 service tools (list, info, call)
  - 4 action tools (list, send_goal, cancel, status)
  - 7 safety tools (status, e-stop, e-stop release, get policy, update velocity, update geofence, audit log)
  - 2 system tools (bridge status, node list)
- Safety layer with policy engine
  - Velocity limits (linear + angular)
  - Geofence workspace boundaries
  - Rate limiting (per topic/service/action)
  - Blocked topics and services
  - Emergency stop (dual-layer: server + bridge)
  - Full audit logging
- YAML safety policy configuration
  - Default conservative policy
  - TurtleBot3-specific policy
  - Runtime policy updates
- Python ROS2 bridge
  - WebSocket server (port 9090)
  - Topic publish/subscribe/echo
  - Service call handler
  - Action goal/cancel/status handler
  - Node/topic/service discovery
  - Bridge-level emergency stop
- Docker setup
  - Gazebo + TurtleBot3 simulation
  - ROS2 Humble bridge container
  - Docker Compose full stack
- 75 unit tests for safety layer
- CI pipeline (GitHub Actions)

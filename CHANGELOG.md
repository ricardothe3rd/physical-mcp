# Changelog

All notable changes to PhysicalMCP will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Acceleration limits — prevent sudden velocity changes (configurable m/s² and rad/s²)
- Geofence proximity warnings — warn when approaching boundary (configurable margin)
- Position check tool (`safety_check_position`) — check geofence + proximity in one call
- Acceleration limits tool (`safety_update_acceleration_limits`) — configure at runtime
- Audit log export tool (`safety_export_audit_log`) — export to JSON file
- Rate limiter stats in safety status output
- Rate limiter cleanup method to prevent memory leaks
- Shared constants file (`utils/constants.ts`)
- Custom error classes (`utils/errors.ts`)
- 54 new tests (196 total)
- Tool dispatch tests with safety handler verification
- ws-client mock tests
- `.prettierrc` configuration

### Changed
- Safety status now includes acceleration config, geofence warning margin, and rate limiter stats
- Total tool count increased from 24 to 27

## [0.1.0] - 2026-02-18

### Added
- Initial release of PhysicalMCP
- TypeScript MCP server with 24 tools
  - 5 topic tools (list, info, subscribe, publish, echo)
  - 3 service tools (list, info, call)
  - 4 action tools (list, send_goal, cancel, status)
  - 10 safety tools (status, e-stop, e-stop release, get policy, update velocity, update geofence, audit log, clamp mode, deadman switch, heartbeat)
  - 2 system tools (bridge status, node list)
- Safety layer with policy engine
  - Velocity limits (linear + angular)
  - Velocity clamping mode (reduce to max instead of blocking)
  - Geofence workspace boundaries (rectangular + circular)
  - Rate limiting (per topic/service/action)
  - Blocked topics and services
  - Emergency stop (dual-layer: server + bridge)
  - Deadman switch (auto e-stop on heartbeat timeout)
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
- CLI with --bridge-url, --policy, --verbose, --version, --help flags
- Startup self-test (validates URL, policy, Node.js version)
- Circuit breaker for bridge connection resilience
- 142 unit tests for safety layer
- CI pipeline (GitHub Actions) with Node 18/20/22 matrix
- Comprehensive documentation suite

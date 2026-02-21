# Changelog

All notable changes to PhysicalMCP will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-21

### Added

#### New Tools (76 new tools, 100 total)
- **Introspection tools** (3): ros2_msg_type_info, ros2_srv_type_info, ros2_action_type_info
- **Namespace tools** (3): ros2_namespace_list, ros2_namespace_remap, ros2_namespace_clear_remaps
- **Sensor tools** (2): ros2_sensor_summary, ros2_sensor_read
- **Launch tools** (3): ros2_launch_start, ros2_launch_stop, ros2_launch_status
- **Waypoint tools** (4): ros2_waypoint_save, ros2_waypoint_list, ros2_waypoint_delete, ros2_waypoint_navigate
- **Batch tool** (1): ros2_batch_execute
- **Conditional tools** (2): ros2_conditional_execute, ros2_wait_for_condition
- **Scheduled tools** (3): ros2_schedule_command, ros2_schedule_cancel, ros2_schedule_list
- **TF2 tools** (2): ros2_tf_tree, ros2_tf_lookup
- **Diagnostic tools** (2): ros2_diagnostics_summary, ros2_diagnostics_detail
- **Fleet tools** (3): ros2_fleet_status, ros2_fleet_add, ros2_fleet_remove
- **Recording tools** (3): ros2_topic_record_start, ros2_topic_record_stop, ros2_topic_record_status
- **Camera tools** (3): ros2_camera_info, ros2_image_preview, ros2_image_topics
- **Map tools** (3): ros2_map_info, ros2_costmap_info, ros2_map_topics
- **History tools** (3): ros2_command_history, ros2_command_stats, ros2_command_replay
- **Path tools** (4): ros2_plan_path, ros2_path_info, ros2_costmap_update, ros2_navigation_status
- **Network tools** (3): ros2_network_stats, ros2_network_bandwidth, ros2_network_latency_test
- **Power tools** (2): ros2_battery_status, ros2_power_supply_status
- **Description tools** (2): ros2_robot_description, ros2_robot_joints

#### New Safety Features
- Acceleration limits — prevent sudden velocity changes (configurable m/s^2 and rad/s^2)
- Geofence proximity warnings — warn when approaching boundary (configurable margin)
- Position check tool (safety_check_position) — check geofence + proximity in one call
- Acceleration limits tool (safety_update_acceleration_limits) — configure at runtime
- Audit log export tool (safety_export_audit_log) — export to JSON file
- Safety policy validation tool (safety_validate_policy)
- Command approval mode (safety_approval_list, safety_approval_approve, safety_approval_deny, safety_approval_config)
- Configurable violation mode (safety_violation_mode) — warn vs block
- Time-based policies (safety_time_policy, safety_time_policy_status)
- Rate limiter stats in safety status output
- Rate limiter cleanup to prevent memory leaks
- Collision zone proximity warning system
- Safety event pub/sub system
- Velocity clamping auto-reduce module
- Multi-robot safety isolation manager
- Position tracking via /odom for live geofence enforcement

#### New Infrastructure
- Shared constants module (constants.ts)
- Custom error classes (errors.ts) — 10 typed error classes
- Structured logging module (logger.ts) — text and JSON formats
- Tool profiles for minimal/standard/full tool grouping
- Message type registry with validation
- Topic recorder utility
- Python WebSocket manager with backpressure and rate limiting
- Python message type validation for ROS2 bridge
- Python graceful dependency checking module
- WebSocket heartbeat manager with latency tracking

#### Testing & CI
- 2167 tests across 83 test files (up from 142 tests in v0.1.0)
- Performance benchmark tests for safety subsystems
- Handler coverage tests (40 tests for all handler types)
- Tool dispatch routing tests
- MCP response format consistency tests
- Concurrent safety and rate limiter edge case tests
- ESLint 9 flat config with typescript-eslint
- Prettier configuration
- Vitest coverage configuration (80% thresholds)
- Docker workflow for GHCR image builds
- Release workflow with npm provenance and GitHub Releases

### Changed
- Total tool count: 24 → 100 across 25 categories
- Total test count: 142 → 2167 across 83 files
- Safety status now includes acceleration config, geofence warning margin, and rate limiter stats
- ESLint upgraded from v8 to v9 with flat config format
- System tools expanded with system_health_status and param tools separated

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

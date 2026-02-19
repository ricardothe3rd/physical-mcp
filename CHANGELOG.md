# Changelog

All notable changes to PhysicalMCP will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Introspection tools (ros2_msg_type_info, ros2_srv_type_info, ros2_action_type_info)
- Namespace management tools (ros2_namespace_list, ros2_namespace_remap, ros2_namespace_clear_remaps)
- Sensor data tools (ros2_sensor_summary, ros2_sensor_read)
- Launch file management tools (ros2_launch_start, ros2_launch_stop, ros2_launch_status)
- Waypoint management tools (ros2_waypoint_save, ros2_waypoint_list, ros2_waypoint_delete, ros2_waypoint_navigate)
- Batch command execution tool (ros2_batch_execute)
- Conditional execution tools (ros2_conditional_execute, ros2_wait_for_condition)
- Scheduled command tools (ros2_schedule_command, ros2_schedule_cancel, ros2_schedule_list)
- TF2 tools (ros2_tf_tree, ros2_tf_lookup)
- Diagnostic tools (ros2_diagnostics_summary, ros2_diagnostics_detail)
- Fleet management tools (ros2_fleet_status, ros2_fleet_add, ros2_fleet_remove)
- Recording tools (ros2_topic_record_start, ros2_topic_record_stop, ros2_topic_record_status)
- Acceleration limits -- prevent sudden velocity changes (configurable m/s^2 and rad/s^2)
- Geofence proximity warnings -- warn when approaching boundary (configurable margin)
- Position check tool (safety_check_position) -- check geofence + proximity in one call
- Acceleration limits tool (safety_update_acceleration_limits) -- configure at runtime
- Audit log export tool (safety_export_audit_log) -- export to JSON file
- Safety policy validation tool (safety_validate_policy)
- Command approval mode (safety_approval_list, safety_approval_approve, safety_approval_deny, safety_approval_config)
- Configurable violation mode (safety_violation_mode) -- warn vs block
- Time-based policies (safety_time_policy, safety_time_policy_status)
- Rate limiter stats in safety status output
- Rate limiter cleanup method to prevent memory leaks
- Shared constants module (constants.ts)
- Custom error classes (errors.ts) -- 10 typed error classes
- Structured logging module (logger.ts) -- text and JSON formats
- Performance benchmark tests for safety subsystems
- Handler coverage tests (40 tests for topic, service, action, system, batch handlers)
- Tool dispatch routing tests
- MCP response format consistency tests
- Concurrent safety and rate limiter edge case tests
- Audit logger edge case tests
- Protocol validation tests
- WebSocket client tests
- ESLint 9 flat config with typescript-eslint
- Prettier configuration
- Vitest coverage configuration (80% thresholds)
- Docker workflow for GHCR image builds
- Release workflow with npm provenance and GitHub Releases
- Release checklist document

### Changed
- Safety status now includes acceleration config, geofence warning margin, and rate limiter stats
- Total tool count increased from 24 to 70 across 17 categories
- Total test count increased from 142 to 1369 across 61 files
- ESLint upgraded from v8 to v9 with flat config format
- Package.json updated with format, format:check scripts

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

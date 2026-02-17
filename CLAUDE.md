# PhysicalMCP

Safety-first MCP server bridging AI agents to ROS2 robots.

## Architecture
- `packages/mcp-server/` - TypeScript MCP server (npm package)
- `packages/ros2-bridge/` - Python ROS2 bridge (rclpy + websockets)
- `docker/` - Dockerfiles for ROS2 bridge and Gazebo sim

## Key Patterns
- MCP server uses @modelcontextprotocol/sdk with zod validation
- WebSocket protocol defined in both TS (protocol.ts) and Python (protocol.py)
- Safety enforcement happens in TS server BEFORE commands reach bridge
- Bridge has secondary e-stop as fail-safe

## Commands
- `npm run build` - Build TS server (in packages/mcp-server)
- `npm run dev` - Dev mode with tsx watch
- `npm test` - Run vitest tests
- `docker compose up` - Start full stack (sim + bridge)

## Safety Layer
All publish/service/action commands pass through policy engine:
- Velocity limits (linear + angular)
- Geofence boundary checks
- Rate limiting (per-topic, per-service)
- Blocked topics/services list
- Emergency stop (zero-velocity + cancel all actions)
- Full audit logging

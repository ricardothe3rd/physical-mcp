# Contributing to PhysicalMCP

Thanks for your interest in contributing to PhysicalMCP. This guide covers everything you need to get started.

## Prerequisites

- **Node.js 20+** (LTS recommended)
- **Python 3.10+** (for ROS2 bridge work)
- **Docker + Docker Compose** (optional, for full stack integration testing)

## Development Setup

```bash
# Clone the repository
git clone https://github.com/ricardothe3rd/physical-mcp.git
cd physical-mcp

# Set up the TypeScript MCP server
cd packages/mcp-server
npm install
npm run build
npm test
```

### TypeScript MCP Server

```bash
cd packages/mcp-server
npm run build       # compile TypeScript
npm test            # run tests
npm run dev         # watch mode with tsx
npm run typecheck   # type check without emitting
```

### Python ROS2 Bridge

The bridge requires ROS2. For development without ROS2, you can run the Python unit tests:

```bash
cd packages/ros2-bridge
pip install -e .
pip install pytest
pytest
```

For full integration testing, use Docker:

```bash
cd docker
docker compose up --build
```

## Project Structure

```
packages/
  mcp-server/              # TypeScript MCP server (npm: @ricardothe3rd/physical-mcp)
    src/
      index.ts             # Entry point, tool registration
      bridge/              # WebSocket client + connection manager
      tools/               # MCP tool definitions
      safety/              # Policy engine, geofence, rate limiter, audit
      types/               # Type definitions
      utils/               # Error recovery utilities, constants, custom errors
  ros2-bridge/             # Python ROS2 bridge (rclpy + websockets)
    src/
      physical_mcp_bridge/ # Bridge implementation
docker/                    # Dockerfiles for ROS2 bridge and Gazebo sim
```

## How to Add a New MCP Tool

1. **Create a tool file** in `packages/mcp-server/src/tools/` (e.g., `my-feature-tools.ts`)
2. **Export two functions** from your tool file:
   - `getMyFeatureTools()` -- returns an array of tool definitions (name, description, inputSchema)
   - `handleMyFeatureTool(name, args, bridge, safetyManager)` -- handles tool invocations and returns results
3. **Register in `index.ts`** -- import your functions and add them to the tool listing and request handler
4. **Add tests** -- create `my-feature-tools.test.ts` alongside your tool file with coverage for valid inputs, invalid inputs, and edge cases

Example tool definition pattern:

```typescript
import { z } from "zod";
import { toInputSchema } from "../utils/schema-helper.js";

const MyToolInputSchema = z.object({
  param: z.string().describe("Description of the parameter"),
});

export function getMyFeatureTools() {
  return [
    {
      name: "my_tool_name",
      description: "What this tool does",
      inputSchema: toInputSchema(MyToolInputSchema),
    },
  ];
}

export async function handleMyFeatureTool(
  name: string,
  args: Record<string, unknown>,
  bridge: BridgeClient,
  safetyManager: SafetyManager
) {
  switch (name) {
    case "my_tool_name": {
      const parsed = MyToolInputSchema.parse(args);
      // Implementation here
    }
    default:
      return null;
  }
}
```

## How to Add a New Safety Check

1. **Modify `packages/mcp-server/src/safety/policy-engine.ts`** to add your new check to the policy evaluation pipeline
2. **Define configuration** for the check in the safety policy types (if it needs to be configurable)
3. **Add the check** to the appropriate stage of command validation (pre-publish, pre-service-call, etc.)
4. **Add tests** that cover:
   - Valid commands that should pass the new check
   - Invalid commands that should be blocked
   - Edge cases and boundary values
   - Interaction with existing safety checks (ensure no regressions)
5. **Update the safety status output** if the check has runtime-queryable state

## Code Style

### TypeScript

- ES2022 target, ESM modules
- **Strict mode enabled** -- all TypeScript strict checks are on
- Use `zod` for runtime validation of tool inputs
- All tool input schemas use the `toInputSchema()` helper
- Use custom error classes from `utils/errors.ts` where possible
- Tests use **vitest** as the test framework

### Python

- Python 3.10+ (use modern type hints: `list[dict]` not `List[Dict]`)
- Follow PEP 8
- Use docstrings for all public methods
- Type hints on all function signatures

### Commit Messages

This project follows **Conventional Commits**. Prefix your commit messages with one of:

- `feat:` -- a new feature
- `fix:` -- a bug fix
- `test:` -- adding or updating tests
- `docs:` -- documentation changes
- `chore:` -- maintenance tasks (dependencies, configs, etc.)
- `ci:` -- CI/CD pipeline changes

Examples:

```
feat: add joint limit safety check
fix: geofence boundary calculation off-by-one
test: add edge case tests for rate limiter cleanup
docs: update tool registration guide
chore: bump vitest to 3.x
ci: add Node 22 to test matrix
```

## Testing

### Writing Tests

Tests live next to the source files (e.g., `policy-engine.test.ts` alongside `policy-engine.ts`).

```bash
# Run all tests
npm test

# Run specific test file
npx vitest run src/safety/geofence.test.ts

# Watch mode
npm run test:watch
```

### Safety Tests

Safety-related changes **must** include test cases for:

- Valid commands that should pass
- Invalid commands that should be blocked
- Edge cases (exact limits, boundary values)

## Pull Request Process

1. **Fork** the repository on GitHub
2. **Create a branch** from `main` (e.g., `feat/joint-limits` or `fix/geofence-edge-case`)
3. **Implement** your changes following the code style guidelines above
4. **Ensure all checks pass**: `npm run build` and `npm test`
5. **Submit a PR** with a clear description of what you changed and why
   - Reference any related issues
   - Describe how you tested the changes
   - Note any breaking changes

## Areas Where Help Is Wanted

- **Robot-specific safety policies** -- YAML configs for popular robots (UR5, Franka, Spot, etc.)
- **Additional safety checks** -- collision avoidance, joint limits, workspace monitoring
- **Documentation** -- tutorials, guides, translations
- **Testing** -- edge cases, integration tests, performance benchmarks
- **Bridge features** -- TF2 support, parameter handling, diagnostics

## Reporting Issues

Use the GitHub issue templates:

- **Bug reports** -- include steps to reproduce, environment details, and logs
- **Feature requests** -- describe the use case and proposed solution

## Security

See [SECURITY.md](SECURITY.md) for reporting security vulnerabilities. Do **not** open public issues for security problems.

# Contributing to PhysicalMCP

Thanks for your interest in contributing. This guide covers everything you need to get started.

## Development Setup

### Prerequisites

- Node.js >= 18
- Python >= 3.10 (for bridge work)
- Docker + Docker Compose (for integration testing)

### TypeScript MCP Server

```bash
cd packages/mcp-server
npm install
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
packages/mcp-server/src/
├── index.ts              # Entry point, tool registration
├── bridge/               # WebSocket client + connection manager
├── tools/                # MCP tool definitions (5 files)
├── safety/               # Policy engine, geofence, rate limiter, audit
├── types/                # Type definitions
└── utils/                # Error recovery utilities
```

## Code Style

### TypeScript
- ES2022 target, ESM modules
- Strict mode enabled
- Use `zod` for runtime validation
- All tool input schemas use `toInputSchema()` helper
- Errors should use custom error classes where possible

### Python
- Python 3.10+ (use modern type hints: `list[dict]` not `List[Dict]`)
- Follow PEP 8
- Use docstrings for all public methods
- Type hints on all function signatures

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

1. Fork the repo and create a branch from `main`
2. Make your changes
3. Ensure `npm run build` and `npm test` pass
4. Write a clear PR description using the template
5. Submit the PR

## Areas Where Help Is Wanted

- **Robot-specific safety policies** — YAML configs for popular robots (UR5, Franka, Spot, etc.)
- **Additional safety checks** — collision avoidance, joint limits, workspace monitoring
- **Documentation** — tutorials, guides, translations
- **Testing** — edge cases, integration tests, performance benchmarks
- **Bridge features** — TF2 support, parameter handling, diagnostics

## Reporting Issues

Use the GitHub issue templates:
- **Bug reports** — include steps to reproduce, environment details, and logs
- **Feature requests** — describe the use case and proposed solution

## Security

See [SECURITY.md](SECURITY.md) for reporting security vulnerabilities.

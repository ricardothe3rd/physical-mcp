# PhysicalMCP Master Task List

Total: 200+ tasks across 12 categories. All autonomous — no permission needed.

---

## 1. README & First Impressions (15 tasks)

- [ ] Add GitHub badges (CI status, npm version, license, tests, node version)
- [ ] Add GIF placeholder sections with alt-text descriptions
- [ ] Add "What it looks like" section with terminal output examples
- [ ] Add troubleshooting section (common errors + fixes)
- [ ] Add FAQ section (5+ questions)
- [ ] Add "Migrating from robotmcp" section
- [ ] Improve quick start with prerequisites checklist
- [ ] Add Claude Code AND Claude Desktop setup instructions
- [ ] Add "Star History" badge/chart placeholder
- [ ] Add social preview image description (for og:image)
- [ ] Add "Sponsors" section placeholder
- [ ] Add "Used By" section placeholder
- [ ] Add shields.io dynamic badges for test count
- [ ] Proofread entire README for typos and clarity
- [ ] Add anchor links table of contents at top

## 2. Documentation (25 tasks)

- [ ] Create CONTRIBUTING.md (dev setup, testing, PR process, code style)
- [ ] Create CODE_OF_CONDUCT.md (Contributor Covenant)
- [ ] Create SECURITY.md (vulnerability reporting process)
- [ ] Create CHANGELOG.md (initial release notes)
- [ ] Create .github/ISSUE_TEMPLATE/bug_report.md
- [ ] Create .github/ISSUE_TEMPLATE/feature_request.md
- [ ] Create .github/PULL_REQUEST_TEMPLATE.md
- [ ] Write API reference doc (all 21 tools with params and examples)
- [ ] Write safety policy authoring guide
- [ ] Write architecture deep-dive doc (design decisions explained)
- [ ] Write WebSocket protocol specification doc
- [ ] Write Docker troubleshooting guide
- [ ] Write "Building custom safety policies" tutorial
- [ ] Write "Connecting to a real robot" guide
- [ ] Write "Using with local LLMs" guide
- [ ] Add inline JSDoc comments to all public TypeScript APIs
- [ ] Add Python docstrings to all public methods
- [ ] Create resources/docs/architecture.md with detailed diagrams
- [ ] Create resources/docs/protocol-spec.md
- [ ] Create resources/docs/safety-deep-dive.md
- [ ] Create resources/docs/policy-authoring.md
- [ ] Create resources/docs/docker-guide.md
- [ ] Create resources/docs/real-hardware-guide.md
- [ ] Create resources/docs/local-llm-guide.md
- [ ] Create resources/docs/faq.md

## 3. CI/CD & DevOps (20 tasks)

- [ ] Create .github/workflows/ci.yml (build + test on push/PR)
- [ ] Add TypeScript type-check step to CI
- [ ] Add Python lint step to CI (ruff)
- [ ] Add Docker build test step to CI
- [ ] Create .github/workflows/release.yml (npm publish on tag)
- [ ] Create .github/dependabot.yml (dependency updates)
- [ ] Add code coverage reporting (vitest coverage + badge)
- [ ] Create .github/workflows/docker.yml (build + push images)
- [ ] Add pre-commit hook config (.husky or lefthook)
- [ ] Create .editorconfig for consistent formatting
- [ ] Create .prettierrc for TypeScript formatting
- [ ] Create .eslintrc.json with project rules
- [ ] Create ruff.toml for Python linting
- [ ] Add auto-labeler config for issues (.github/labeler.yml)
- [ ] Add stale issue bot config (.github/stale.yml)
- [ ] Create release checklist template
- [ ] Add npm provenance (--provenance flag in publish)
- [ ] Create Makefile with common commands (build, test, docker, lint)
- [ ] Add git hooks for pre-commit lint
- [ ] Create .nvmrc with Node.js version

## 4. Testing — TypeScript (30 tasks)

- [ ] Add connection-manager tests with mock WebSocket
- [ ] Add ws-client tests with mock server
- [ ] Test emergency stop publishes zero velocity to bridge
- [ ] Test concurrent safety checks (race conditions)
- [ ] Test rate limiter under high load (100+ rapid calls)
- [ ] Test rate limiter window expiry edge case
- [ ] Test policy hot-reload scenario
- [ ] Test geofence exact boundary (on the line)
- [ ] Test geofence negative coordinates
- [ ] Test audit logger memory cap (>10000 entries)
- [ ] Test audit logger concurrent writes
- [ ] Test protocol with malformed JSON
- [ ] Test protocol with missing required fields
- [ ] Test protocol with extra unknown fields
- [ ] Test protocol with oversized messages
- [ ] Test topic tools with bridge error responses
- [ ] Test service tools with timeout scenarios
- [ ] Test action tools with goal rejection
- [ ] Test action tools with goal cancellation
- [ ] Test safety tools e-stop + release flow
- [ ] Test system tools with disconnected bridge
- [ ] Test index.ts tool dispatch routing
- [ ] Test index.ts bridge connection failure graceful handling
- [ ] Test velocity check with only angular component
- [ ] Test velocity check with diagonal movement
- [ ] Test blocked topic with partial match (prefix)
- [ ] Test allowed topics whitelist mode
- [ ] Test allowed services whitelist mode
- [ ] Add vitest coverage config and aim for >90%
- [ ] Add snapshot tests for tool response formats

## 5. Testing — Python (20 tasks)

- [ ] Create tests/ directory in ros2-bridge
- [ ] Unit test discovery.py list_topics
- [ ] Unit test discovery.py get_topic_info
- [ ] Unit test discovery.py list_services
- [ ] Unit test discovery.py list_actions
- [ ] Unit test discovery.py list_nodes
- [ ] Unit test topic_handler.py subscribe with timeout
- [ ] Unit test topic_handler.py publish
- [ ] Unit test topic_handler.py _set_message_fields
- [ ] Unit test service_handler.py call with timeout
- [ ] Unit test service_handler.py call with unavailable service
- [ ] Unit test action_handler.py send_goal
- [ ] Unit test action_handler.py cancel_goal
- [ ] Unit test action_handler.py cancel_all
- [ ] Unit test action_handler.py get_status
- [ ] Unit test telemetry.py get_status
- [ ] Unit test protocol.py parse_command
- [ ] Unit test protocol.py build_response
- [ ] Unit test bridge_node.py command dispatch
- [ ] Add pytest config and requirements-dev.txt

## 6. Code Quality & Robustness (35 tasks)

- [ ] Add input validation to every MCP tool (validate required fields)
- [ ] Add timeout handling for all bridge commands (configurable)
- [ ] Add graceful degradation when bridge disconnects mid-command
- [ ] Add WebSocket heartbeat/keepalive ping
- [ ] Handle malformed ROS2 messages gracefully in Python handlers
- [ ] Add message type validation before publish
- [ ] Handle concurrent e-stop from multiple sources
- [ ] Handle bridge reconnection during active subscription
- [ ] Add WebSocket connection backpressure handling
- [ ] Rate limiter cleanup — prevent memory leak from abandoned windows
- [ ] Audit logger — add export to file capability
- [ ] Audit logger — add rotation (new file per day/size)
- [ ] Add configurable log levels (debug/info/warn/error)
- [ ] Add structured logging (JSON format option)
- [ ] Create shared constants file (ports, timeouts, defaults)
- [ ] Add TypeScript strict null checks verification
- [ ] Add proper error types (custom error classes)
- [ ] Handle SIGTERM/SIGINT gracefully in Python bridge
- [ ] Add startup self-test (verify bridge reachable)
- [ ] Add config validation on startup (check YAML is valid)
- [ ] Add environment variable validation on startup
- [ ] Handle node_modules resolution for policies path
- [ ] Make policy path resolve relative to package root
- [ ] Add --version flag to CLI
- [ ] Add --help flag to CLI
- [ ] Add --policy flag to CLI (alternative to env var)
- [ ] Add --bridge-url flag to CLI
- [ ] Add --verbose flag to CLI
- [ ] Python: handle ROS2 not installed gracefully
- [ ] Python: handle websockets import failure gracefully
- [ ] Python: add proper async exception handling
- [ ] Python: add signal handlers for clean shutdown
- [ ] Python: handle multiple concurrent WebSocket clients
- [ ] Python: add configurable port via env var
- [ ] Python: add configurable host via env var

## 7. New Features — Safety (20 tasks)

- [ ] Policy hot-reload (watch YAML file, apply changes without restart)
- [ ] Configurable violation response: "warn" vs "block" mode
- [ ] Safety override with API key authentication
- [ ] Position tracking via /odom for geofence enforcement
- [ ] Acceleration limits (rate of velocity change)
- [ ] Collision zone warnings (near geofence boundary)
- [ ] Safety violation notifications (webhook/callback)
- [ ] Per-topic velocity limits (different limits for different topics)
- [ ] Time-based policies (different limits day vs night)
- [ ] Safety policy inheritance (base + override)
- [ ] Safety policy versioning and rollback
- [ ] Command approval mode (human-in-the-loop for dangerous commands)
- [ ] Safety score per session (% of commands blocked)
- [ ] Automatic velocity clamping option (reduce to max instead of block)
- [ ] Geofence shapes: circle, polygon (not just rectangle)
- [ ] 3D geofence visualization data export
- [ ] Safety event pub/sub (subscribe to violation events)
- [ ] Robot state machine (idle, moving, e-stopped, error)
- [ ] Deadman switch (auto-stop if no commands for N seconds)
- [ ] Multi-robot safety isolation (per-robot policies)

## 8. New Features — Tools & Capabilities (25 tasks)

- [ ] TF2 tree visualization tool (show transform tree)
- [ ] Parameter get/set tools (read/write ROS2 params)
- [ ] Bag recording tool (start/stop rosbag2 recording)
- [ ] Launch file management tools (list/start/stop launches)
- [ ] Robot description tool (fetch and display URDF)
- [ ] Diagnostics aggregator tool (system health overview)
- [ ] Battery/power monitoring tool
- [ ] Network quality monitoring tool
- [ ] Camera/image topic preview tool
- [ ] Map visualization data tool
- [ ] Path planning integration tool
- [ ] Sensor data summary tool (all sensors at once)
- [ ] Multi-robot fleet status tool
- [ ] Command history/replay tool
- [ ] Waypoint management tool (save/load/navigate)
- [ ] Custom message type registration
- [ ] Topic recording (subscribe and save to file)
- [ ] Service introspection (show request/response types)
- [ ] Action introspection (show goal/feedback/result types)
- [ ] Namespace-aware tool routing
- [ ] Tool grouping and profiles (minimal, standard, full)
- [ ] Interactive mode (stream topic data continuously)
- [ ] Batch command execution tool
- [ ] Conditional command tool (if sensor X then do Y)
- [ ] Scheduled command tool (publish at specific time)

## 9. Launch & Marketing (25 tasks)

- [ ] Polish and finalize Hacker News Show HN post
- [ ] Polish and finalize Reddit r/ROS post
- [ ] Polish and finalize Reddit r/ClaudeAI post
- [ ] Polish and finalize Reddit r/robotics post
- [ ] Polish and finalize Reddit r/LocalLLaMA post
- [ ] Polish and finalize Twitter/X thread
- [ ] Polish and finalize ROS Discourse post
- [ ] Write dev.to blog post: "Why AI Agents Need Safety Guardrails for Robots"
- [ ] Write dev.to blog post: "Building a Safety-First MCP Server"
- [ ] Write Medium article on the safety layer architecture
- [ ] Create Product Hunt listing draft
- [ ] Create demo recording script (step-by-step terminal commands)
- [ ] Create ASCII art demo output for README (safety block example)
- [ ] Create terminal screenshot mockups showing safety blocks
- [ ] Write YouTube video script (2-3 min walkthrough)
- [ ] Write YouTube Short script (15-30 sec hook)
- [ ] Create launch day minute-by-minute timeline
- [ ] Create post-launch week 1 engagement plan
- [ ] Create star milestone celebration post templates (100, 500, 1K)
- [ ] Draft weekly project update template
- [ ] Create "awesome-physical-mcp" list of use cases
- [ ] Write comparison article: PhysicalMCP vs robotmcp vs wise-vision
- [ ] Create infographic: "AI Robot Safety Checklist"
- [ ] Draft email for robotics newsletters
- [ ] Draft email for AI/ML newsletters

## 10. Community & Open Source (15 tasks)

- [ ] Create GitHub issue labels (bug, feature, safety, docs, good-first-issue)
- [ ] Write 10 "good first issue" issues for contributors
- [ ] Create GitHub project board with columns
- [ ] Write public roadmap document
- [ ] Create GOVERNANCE.md
- [ ] Set up GitHub Discussions (categories: general, ideas, show-and-tell, Q&A)
- [ ] Create Discord/community invite (or GitHub Discussions)
- [ ] Write "How to add a new MCP tool" contributor tutorial
- [ ] Write "How to add a new safety check" contributor tutorial
- [ ] Write "How to write a robot-specific policy" guide
- [ ] Create logo concepts document (design brief)
- [ ] Set up GitHub Sponsors / funding.yml
- [ ] Create social media account naming plan
- [ ] Write maintainer guide (release process, review guidelines)
- [ ] Create RFC process for major changes

## 11. Cloud Relay Prep (15 tasks)

- [ ] Write cloud relay architecture design doc
- [ ] Design auth system (API keys + JWT)
- [ ] Design WebSocket relay protocol (client ↔ relay ↔ bridge)
- [ ] Design usage metering schema
- [ ] Design multi-tenant isolation model
- [ ] Design pricing tiers (free / $29 / $99 / enterprise)
- [ ] Create relay server project scaffold
- [ ] Create landing page wireframe/copy
- [ ] Write pricing page copy
- [ ] Draft terms of service
- [ ] Draft privacy policy
- [ ] Design relay dashboard mockup
- [ ] Plan Stripe integration approach
- [ ] Design robot registration and pairing flow
- [ ] Write cloud relay SDK design doc

## 12. Performance & Optimization (10 tasks)

- [ ] Benchmark: measure command round-trip latency
- [ ] Benchmark: measure safety check overhead per command
- [ ] Benchmark: max sustained commands per second
- [ ] Optimize rate limiter timestamp cleanup
- [ ] Optimize audit logger memory usage
- [ ] Add MessagePack option for binary WebSocket protocol
- [ ] Add WebSocket compression (permessage-deflate)
- [ ] Profile TypeScript startup time and optimize
- [ ] Profile Python bridge startup time
- [ ] Add lazy loading for tool modules

---

## Execution Priority

### Phase 1: Launch-Ready (do NOW)
Categories: 1, 2 (critical docs), 3 (CI), 6 (robustness), 9 (marketing)

### Phase 2: Credibility (week 1-2)
Categories: 4, 5 (testing), 2 (remaining docs), 10 (community)

### Phase 3: Growth (month 1-2)
Categories: 7, 8 (features), 12 (performance)

### Phase 4: Revenue (month 2-3)
Category: 11 (cloud relay)

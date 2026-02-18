# Security Policy

## Reporting a Vulnerability

If you discover a security vulnerability in PhysicalMCP, please report it responsibly:

1. **Do NOT** open a public GitHub issue
2. Email the maintainer or use GitHub's private vulnerability reporting feature
3. Include a description of the vulnerability and steps to reproduce
4. Allow reasonable time for a fix before public disclosure

## Scope

Security issues that are in scope:

- **Safety layer bypasses** — any way to send commands to the robot without safety checks
- **Emergency stop bypass** — any way to move the robot while e-stop is active
- **Audit log tampering** — any way to modify or delete audit entries
- **WebSocket injection** — unauthorized command injection via the bridge connection
- **Policy loading vulnerabilities** — YAML parsing exploits or policy bypass

## Safety vs Security

PhysicalMCP's safety layer is designed to prevent accidental damage from AI agents, not to resist adversarial attacks from users with system access. If someone has access to the ROS2 network directly, they can bypass the MCP server entirely. The safety layer is a guardrail, not a security boundary.

That said, we take safety bypass bugs seriously — if the safety layer can be circumvented through the MCP interface, that's a bug we want to fix.

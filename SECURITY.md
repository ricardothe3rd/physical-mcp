# Security Policy

## Supported Versions

| Version | Supported |
| ------- | --------- |
| 0.1.x   | Yes       |

## Reporting a Vulnerability

If you discover a security vulnerability in PhysicalMCP, please report it responsibly.

**Do NOT open a public GitHub issue for security vulnerabilities.**

Email: **security@chinchillaenterprises.com**

Please include:

- A description of the vulnerability
- Steps to reproduce the issue
- Potential impact assessment
- Any suggested mitigations

### Response Timeline

- **48 hours** -- acknowledgment of your report
- **7 days** -- plan for a fix communicated to you
- **Coordinated disclosure** -- we will coordinate public disclosure after a fix is released

## Scope

Security issues that are **in scope**:

- **MCP server** -- tool dispatch, input validation, command processing
- **Safety layer** -- policy engine bypasses, e-stop circumvention, audit log tampering
- **Bridge protocol** -- WebSocket injection, unauthorized command execution
- **Docker images** -- container escape, privilege escalation

Security issues that are **out of scope**:

- **ROS2 itself** -- report upstream to Open Robotics
- **Third-party dependencies** -- report upstream to the respective maintainers
- **Physical access attacks** -- users with direct access to the ROS2 network

## Safety vs Security

PhysicalMCP's safety layer prevents accidental damage from AI agents, not adversarial attacks from users with system access. If someone has direct access to the ROS2 network, they can bypass the MCP server entirely. The safety layer is a guardrail, not a security boundary.

That said, we take safety bypass bugs seriously. If the safety layer can be circumvented through the MCP interface, that is a bug we want to fix.

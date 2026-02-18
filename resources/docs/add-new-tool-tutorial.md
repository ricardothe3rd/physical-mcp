# Tutorial: How to Add a New MCP Tool to PhysicalMCP

This guide walks through the complete process of adding a new MCP tool to the PhysicalMCP server, from defining the tool schema to writing tests. We use a concrete worked example throughout: adding a `ros2_param_get` tool that retrieves ROS2 node parameters.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Step 1: Choose the Right Tools File](#step-1-choose-the-right-tools-file)
3. [Step 2: Define the Tool with a Zod Schema](#step-2-define-the-tool-with-a-zod-schema)
4. [Step 3: Add the Handler Case](#step-3-add-the-handler-case)
5. [Step 4: Add the Protocol Command Type (If Needed)](#step-4-add-the-protocol-command-type-if-needed)
6. [Step 5: Register the Tools File (If New)](#step-5-register-the-tools-file-if-new)
7. [Step 6: Add Safety Checks (If the Tool Modifies Robot State)](#step-6-add-safety-checks-if-the-tool-modifies-robot-state)
8. [Step 7: Write Tests](#step-7-write-tests)
9. [Complete Worked Example: ros2_param_get](#complete-worked-example-ros2_param_get)
10. [Checklist](#checklist)

---

## Architecture Overview

Every MCP tool in PhysicalMCP follows a three-layer pattern:

```
MCP Client (Claude, etc.)
  |
  v
Tool Definition (zod schema + metadata)     <-- packages/mcp-server/src/tools/
  |
  v
Handler Function (safety check + bridge call) <-- same file
  |
  v
WebSocket Protocol (command type + params)   <-- packages/mcp-server/src/bridge/protocol.ts
  |
  v
Python ROS2 Bridge                           <-- packages/ros2-bridge/
```

Key files involved:

| File | Purpose |
|------|---------|
| `src/tools/<category>-tools.ts` | Tool definitions (schema) and handler functions |
| `src/bridge/protocol.ts` | `CommandType` enum and WebSocket message schemas |
| `src/index.ts` | Tool registration and dispatch routing |
| `src/safety/policy-engine.ts` | Safety checks (velocity, geofence, rate limiting, etc.) |

---

## Step 1: Choose the Right Tools File

Tools are organized by category in `packages/mcp-server/src/tools/`:

| File | Category | Examples |
|------|----------|---------|
| `topic-tools.ts` | Topic operations | `ros2_topic_list`, `ros2_topic_publish` |
| `service-tools.ts` | Service operations | `ros2_service_list`, `ros2_service_call` |
| `action-tools.ts` | Action operations | `ros2_action_list`, `ros2_action_send_goal` |
| `safety-tools.ts` | Safety system | `safety_emergency_stop`, `safety_status` |
| `system-tools.ts` | System/diagnostics | `system_bridge_status`, `system_node_list` |

For our `ros2_param_get` example, this is a system-level operation that reads node parameters. It fits best in `system-tools.ts`, or you could create a new `param-tools.ts` if you plan to add multiple parameter-related tools.

**Rule of thumb:** If the tool fits an existing category, add it there. Only create a new file when you are adding a genuinely new category of tools (e.g., a set of parameter tools, TF-related tools, etc.).

---

## Step 2: Define the Tool with a Zod Schema

Every tool needs two things:
1. A **tool definition** returned by the `get*Tools()` function
2. A **handler case** in the `handle*Tool()` function

The tool definition uses the MCP `Tool` type and a Zod schema for input validation.

### The `toInputSchema` helper

Every tools file includes this helper that converts a Zod schema into the JSON Schema format required by the MCP SDK:

```typescript
import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}
```

### Adding the tool definition

Add a new entry to the array returned by the `get*Tools()` function:

```typescript
export function getSystemTools(): Tool[] {
  return [
    // ... existing tools ...
    {
      name: 'ros2_param_get',
      description: 'Get parameter values from a ROS2 node',
      inputSchema: toInputSchema(z.object({
        node: z.string().describe('Full node name (e.g. /turtlesim)'),
        parameters: z.array(z.string()).describe('List of parameter names to retrieve'),
      })),
    },
  ];
}
```

### Tool naming conventions

- ROS2 tools: `ros2_<category>_<action>` (e.g., `ros2_topic_publish`, `ros2_param_get`)
- Safety tools: `safety_<action>` (e.g., `safety_emergency_stop`)
- System tools: `system_<action>` (e.g., `system_bridge_status`)

### Schema guidelines

- Use `.describe()` on every field to provide clear documentation for the AI agent.
- Use `.default()` for optional parameters that have sensible defaults.
- Use `.optional()` for truly optional parameters.
- Use `z.record(z.unknown())` for freeform JSON objects (like ROS2 message data).
- Use `z.literal()` for confirmation parameters (see `safety_emergency_stop_release`).

---

## Step 3: Add the Handler Case

The handler is an `async` function that takes the tool name, arguments, a `ConnectionManager`, and (optionally) a `PolicyEngine`. It returns an MCP-formatted response.

Add a new `case` to the `switch` statement in the handler function:

```typescript
export async function handleSystemTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    // ... existing cases ...

    case 'ros2_param_get': {
      const response = await connection.send(CommandType.GET_PARAMS, {
        node: args.node,
        parameters: args.parameters,
      });
      if (response.status === 'error') {
        return {
          content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }],
          isError: true,
        };
      }
      return {
        content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }],
      };
    }

    default:
      return {
        content: [{ type: 'text', text: `Unknown system tool: ${name}` }],
        isError: true,
      };
  }
}
```

### Handler patterns

There are two common handler patterns:

**1. Read-only (no safety check needed):**

```typescript
case 'ros2_topic_list': {
  const response = await connection.send(CommandType.TOPIC_LIST);
  if (response.status === 'error') {
    return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
  }
  return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
}
```

**2. State-modifying (safety check required):**

```typescript
case 'ros2_topic_publish': {
  const topic = args.topic as string;
  const message = args.message as Record<string, unknown>;

  // SAFETY CHECK
  const check = safety.checkPublish(topic, message);
  if (!check.allowed) {
    const violationText = check.violations
      .map(v => `- [${v.type}] ${v.message}`)
      .join('\n');
    return {
      content: [{ type: 'text', text: `SAFETY BLOCKED: Publish to ${topic} denied.\n\nViolations:\n${violationText}` }],
      isError: true,
    };
  }

  const response = await connection.send(CommandType.TOPIC_PUBLISH, { topic, message });
  // ... handle response ...
}
```

### Response format

All handlers return the same structure:

```typescript
{
  content: [{ type: 'text', text: '...' }],
  isError?: boolean  // set to true for errors/blocked commands
}
```

- Return `isError: true` when the command fails or is blocked by safety.
- Use `JSON.stringify(data, null, 2)` for structured data so the AI can parse it.
- Use human-readable strings for status messages.

---

## Step 4: Add the Protocol Command Type (If Needed)

If your tool requires a new command type that the Python bridge must handle, add it to `src/bridge/protocol.ts`:

```typescript
export const CommandType = {
  // ... existing types ...
  TOPIC_LIST: 'topic.list',
  TOPIC_PUBLISH: 'topic.publish',
  // etc.
  GET_PARAMS: 'params.get',       // <-- already exists for our example
  SET_PARAMS: 'params.set',       // <-- you'd add this for a new param set tool
} as const;
```

The string value (e.g., `'params.get'`) is what gets sent over WebSocket to the Python bridge, which has a matching handler. The convention is `<category>.<action>`.

**Important:** If you add a new command type, you must also add the corresponding handler in the Python bridge (`packages/ros2-bridge/`). That is outside the scope of this tutorial but follows a similar pattern on the Python side.

For our `ros2_param_get` example, the `GET_PARAMS` command type already exists in `protocol.ts`, so no change is needed.

---

## Step 5: Register the Tools File (If New)

If you created a **new** tools file (e.g., `param-tools.ts`), you need to register it in `src/index.ts`:

### 5a. Import the getter and handler

```typescript
import { getParamTools, handleParamTool } from './tools/param-tools.js';
```

### 5b. Add tools to the `allTools` array

```typescript
const allTools = [
  ...getTopicTools(),
  ...getServiceTools(),
  ...getActionTools(),
  ...getSafetyTools(),
  ...getSystemTools(),
  ...getParamTools(),  // <-- add this
];
```

### 5c. Create the name set for dispatch

```typescript
const paramToolNames = new Set(getParamTools().map(t => t.name));
```

### 5d. Add dispatch routing in the `CallToolRequestSchema` handler

```typescript
if (paramToolNames.has(name)) {
  return await handleParamTool(name, toolArgs, connection, safety);
}
```

If you added the tool to an **existing** file (like `system-tools.ts`), none of this is needed -- the tool is automatically picked up by the existing `getSystemTools()` call.

---

## Step 6: Add Safety Checks (If the Tool Modifies Robot State)

**Read-only tools** (listing, echoing, getting parameters, checking status) do **not** need safety checks.

**State-modifying tools** (publishing, calling services, sending action goals, setting parameters) **must** go through the safety policy engine before executing.

### When to add safety checks

Ask yourself: "Could this tool cause the robot to move, change its configuration, or affect its physical environment?" If yes, add a safety check.

Examples:
- `ros2_topic_publish` -- YES, could send velocity commands
- `ros2_service_call` -- YES, could trigger physical actions
- `ros2_action_send_goal` -- YES, sends navigation goals
- `ros2_param_get` -- NO, read-only
- `ros2_topic_list` -- NO, read-only

### How to add the check

1. Accept `PolicyEngine` as a parameter in your handler function.
2. Call the appropriate safety method before sending the command:
   - `safety.checkPublish(topic, message)` for topic publishes
   - `safety.checkServiceCall(service, args)` for service calls
   - `safety.checkActionGoal(action, goal)` for action goals
3. If `!check.allowed`, format the violations and return an error response.

If you are adding a completely new category of state-modifying commands (e.g., parameter setting), you may need to add a new check method to `PolicyEngine`. See the [How to Add a New Safety Check](./add-safety-check-tutorial.md) tutorial for details.

---

## Step 7: Write Tests

Tests are colocated with source files using the `*.test.ts` naming convention and use the [Vitest](https://vitest.dev/) framework.

### Testing a tool handler

Since tool handlers depend on `ConnectionManager` (WebSocket) and `PolicyEngine`, tests typically fall into two categories:

**1. Unit tests for safety integration (mock the connection):**

```typescript
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { handleSystemTool } from './system-tools.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';

// Mock the connection manager
const mockConnection = {
  send: vi.fn(),
  isConnected: true,
  isBridgeAvailable: true,
} as unknown as ConnectionManager;

describe('ros2_param_get handler', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it('sends GET_PARAMS command with correct parameters', async () => {
    (mockConnection.send as ReturnType<typeof vi.fn>).mockResolvedValue({
      status: 'ok',
      data: { 'use_sim_time': false, 'robot_description': '...' },
    });

    const result = await handleSystemTool(
      'ros2_param_get',
      { node: '/turtlesim', parameters: ['use_sim_time'] },
      mockConnection
    );

    expect(mockConnection.send).toHaveBeenCalledWith('params.get', {
      node: '/turtlesim',
      parameters: ['use_sim_time'],
    });
    expect(result.isError).toBeUndefined();
    expect(result.content[0].text).toContain('use_sim_time');
  });

  it('returns error when bridge returns error', async () => {
    (mockConnection.send as ReturnType<typeof vi.fn>).mockResolvedValue({
      status: 'error',
      data: { message: 'Node not found' },
    });

    const result = await handleSystemTool(
      'ros2_param_get',
      { node: '/nonexistent', parameters: ['foo'] },
      mockConnection
    );

    expect(result.isError).toBe(true);
    expect(result.content[0].text).toContain('Error');
  });
});
```

**2. Safety policy tests (for state-modifying tools):**

```typescript
import { describe, it, expect, beforeEach } from 'vitest';
import { PolicyEngine } from '../safety/policy-engine.js';

describe('safety checks for param_set tool', () => {
  let engine: PolicyEngine;

  beforeEach(() => {
    engine = new PolicyEngine();
  });

  it('blocks parameter changes during e-stop', () => {
    engine.activateEmergencyStop();
    const result = engine.checkServiceCall('/set_parameters', {
      parameters: [{ name: 'max_speed', value: 1.0 }],
    });
    expect(result.allowed).toBe(false);
    expect(result.violations[0].type).toBe('emergency_stop_active');
  });
});
```

### Running tests

From `packages/mcp-server/`:

```bash
# Run all tests
npm test

# Run a specific test file
npx vitest run src/tools/system-tools.test.ts

# Run tests in watch mode
npx vitest src/tools/system-tools.test.ts
```

---

## Complete Worked Example: ros2_param_get

Here is every file change needed to add `ros2_param_get` to the existing `system-tools.ts`. Since `CommandType.GET_PARAMS` already exists in `protocol.ts`, and this is a read-only tool being added to an existing tools file, the changes are minimal.

### File 1: `src/tools/system-tools.ts`

**Add the tool definition** to `getSystemTools()`:

```typescript
export function getSystemTools(): Tool[] {
  return [
    {
      name: 'system_bridge_status',
      description: 'Get the connection status of the ROS2 bridge (WebSocket health, latency)',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'system_node_list',
      description: 'List all active ROS2 nodes',
      inputSchema: toInputSchema(z.object({})),
    },
    // NEW TOOL
    {
      name: 'ros2_param_get',
      description: 'Get parameter values from a ROS2 node',
      inputSchema: toInputSchema(z.object({
        node: z.string().describe('Full node name (e.g. /turtlesim)'),
        parameters: z.array(z.string()).describe('List of parameter names to retrieve'),
      })),
    },
  ];
}
```

**Add the handler case** to `handleSystemTool()`:

```typescript
case 'ros2_param_get': {
  const response = await connection.send(CommandType.GET_PARAMS, {
    node: args.node,
    parameters: args.parameters,
  });
  if (response.status === 'error') {
    return {
      content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }],
      isError: true,
    };
  }
  return {
    content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }],
  };
}
```

**Add the import** (if not already present):

```typescript
import { CommandType } from '../bridge/protocol.js';
```

### File 2: `src/bridge/protocol.ts` (no change needed)

The `GET_PARAMS` command type already exists:

```typescript
export const CommandType = {
  // ...
  GET_PARAMS: 'params.get',
  // ...
} as const;
```

### File 3: `src/index.ts` (no change needed)

Since we added the tool to `system-tools.ts`, the existing `getSystemTools()` and `handleSystemTool()` calls automatically include it.

### File 4: `src/tools/system-tools.test.ts` (new test file)

```typescript
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getSystemTools, handleSystemTool } from './system-tools.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

const mockConnection = {
  send: vi.fn(),
  isConnected: true,
  isBridgeAvailable: true,
} as unknown as ConnectionManager;

describe('system tools', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  describe('getSystemTools', () => {
    it('includes ros2_param_get', () => {
      const tools = getSystemTools();
      const paramTool = tools.find(t => t.name === 'ros2_param_get');
      expect(paramTool).toBeDefined();
      expect(paramTool!.description).toContain('parameter');
    });
  });

  describe('handleSystemTool - ros2_param_get', () => {
    it('sends params.get command to bridge', async () => {
      (mockConnection.send as ReturnType<typeof vi.fn>).mockResolvedValue({
        status: 'ok',
        data: { use_sim_time: false },
      });

      const result = await handleSystemTool(
        'ros2_param_get',
        { node: '/turtlesim', parameters: ['use_sim_time'] },
        mockConnection
      );

      expect(mockConnection.send).toHaveBeenCalledWith('params.get', {
        node: '/turtlesim',
        parameters: ['use_sim_time'],
      });
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('use_sim_time');
    });

    it('returns error when bridge responds with error', async () => {
      (mockConnection.send as ReturnType<typeof vi.fn>).mockResolvedValue({
        status: 'error',
        data: { message: 'Node /missing not found' },
      });

      const result = await handleSystemTool(
        'ros2_param_get',
        { node: '/missing', parameters: ['foo'] },
        mockConnection
      );

      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Error');
    });

    it('returns unknown tool error for invalid name', async () => {
      const result = await handleSystemTool(
        'system_nonexistent',
        {},
        mockConnection
      );
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Unknown system tool');
    });
  });
});
```

---

## Checklist

Use this checklist when adding a new tool:

- [ ] **Chose the right tools file** (or created a new one with the standard pattern)
- [ ] **Defined the tool** in the `get*Tools()` function with a Zod schema
- [ ] **Added `.describe()`** to every schema field
- [ ] **Added the handler case** in the `handle*Tool()` switch statement
- [ ] **Added `CommandType`** in `protocol.ts` (if a new command type is needed)
- [ ] **Registered in `index.ts`** (only if you created a new tools file)
- [ ] **Added safety checks** (if the tool modifies robot state)
- [ ] **Wrote tests** covering:
  - [ ] Tool appears in the tools list
  - [ ] Handler sends the correct command type and parameters
  - [ ] Handler returns proper error on bridge error
  - [ ] Safety violations are reported correctly (if applicable)
  - [ ] Handler returns unknown tool error for the default case
- [ ] **Ran `npm test`** and all tests pass
- [ ] **Ran `npm run build`** and TypeScript compiles without errors

# Using PhysicalMCP with Local LLMs

PhysicalMCP is a standard MCP (Model Context Protocol) server. While the quick-start examples use Claude, **PhysicalMCP works with any MCP-compatible client** -- including local LLMs running on your own hardware. This guide covers how to connect PhysicalMCP to local LLMs using Ollama, llama.cpp, vLLM, and other frameworks.

---

## Table of Contents

- [How It Works](#how-it-works)
- [Requirements](#requirements)
- [Option 1: Ollama with MCP Support](#option-1-ollama-with-mcp-support)
- [Option 2: llama.cpp with Function Calling](#option-2-llamacpp-with-function-calling)
- [Option 3: vLLM with MCP Adapter](#option-3-vllm-with-mcp-adapter)
- [Option 4: Any LLM with an MCP Client Library](#option-4-any-llm-with-an-mcp-client-library)
- [Option 5: LangChain or LlamaIndex with MCP](#option-5-langchain-or-llamaindex-with-mcp)
- [Writing Your Own MCP Client](#writing-your-own-mcp-client)
- [Model Recommendations](#model-recommendations)
- [Safety Considerations for Local LLMs](#safety-considerations-for-local-llms)
- [Troubleshooting](#troubleshooting)

---

## How It Works

PhysicalMCP communicates over the MCP protocol, which uses JSON-RPC 2.0 over stdio (stdin/stdout). The architecture is:

```
+---------------------+          +----------------------------+          +------------------+
|   Local LLM         |          |   PhysicalMCP              |          |   ROS2 Bridge    |
|   (Ollama, llama.   |  stdio   |   MCP Server               |    WS    |   (Python)       |
|    cpp, vLLM, etc.) | <------> |   (TypeScript/Node.js)     | <------> |                  |
|                     | JSON-RPC |                            | :9090    |   ROS2 DDS       |
|   + MCP client      |          |   Safety layer enforced    |          |       |          |
|     adapter         |          |   on every command         |          |     Robot        |
+---------------------+          +----------------------------+          +------------------+
```

The LLM does not communicate with the robot directly. It communicates with PhysicalMCP through an MCP client, which spawns the PhysicalMCP server process and sends tool calls over stdio. The safety layer sits inside PhysicalMCP and evaluates every command identically, regardless of whether the calling LLM is Claude, GPT-4, Llama, Mistral, or any other model.

**Key point:** PhysicalMCP does not care what LLM is driving it. The safety layer is the same for all clients.

---

## Requirements

For all options below, you need:

1. **PhysicalMCP installed:**
   ```bash
   npm install -g @ricardothe3rd/physical-mcp
   ```
   Or use it via `npx @ricardothe3rd/physical-mcp` (no global install needed).

2. **The ROS2 bridge running** (either via Docker or on a machine with ROS2):
   ```bash
   cd docker && docker compose up
   ```

3. **A local LLM** capable of function calling / tool use. Not all models support this. See [Model Recommendations](#model-recommendations) for guidance.

4. **An MCP client** that can spawn an MCP server process and relay tool calls from the LLM. This is the adapter layer between your LLM and PhysicalMCP.

---

## Option 1: Ollama with MCP Support

[Ollama](https://ollama.ai) is the most popular way to run LLMs locally. When paired with an MCP client framework, it can drive PhysicalMCP.

### Step 1: Install Ollama and pull a model

```bash
# Install Ollama (macOS / Linux)
curl -fsSL https://ollama.ai/install.sh | sh

# Pull a model with tool-calling support
ollama pull llama3.1:8b
# Or for better tool-calling performance:
ollama pull qwen2.5:14b
```

### Step 2: Use an MCP client that supports Ollama

Several MCP client libraries support Ollama as a backend. The pattern is: the MCP client connects to PhysicalMCP over stdio, discovers its tools, and translates those tools into function-calling format for the Ollama model.

**Using the MCP TypeScript SDK with Ollama:**

Create a file called `ollama-robot.ts`:

```typescript
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { StdioClientTransport } from "@modelcontextprotocol/sdk/client/stdio.js";
import Ollama from "ollama";

async function main() {
  // 1. Connect to PhysicalMCP via stdio
  const transport = new StdioClientTransport({
    command: "npx",
    args: ["@ricardothe3rd/physical-mcp"],
    env: {
      ...process.env,
      PHYSICAL_MCP_BRIDGE_URL: "ws://localhost:9090",
    },
  });

  const mcpClient = new Client({
    name: "ollama-robot-client",
    version: "1.0.0",
  });

  await mcpClient.connect(transport);

  // 2. Discover PhysicalMCP's tools
  const { tools } = await mcpClient.listTools();

  // 3. Convert MCP tools to Ollama's tool format
  const ollamaTools = tools.map((tool) => ({
    type: "function" as const,
    function: {
      name: tool.name,
      description: tool.description,
      parameters: tool.inputSchema,
    },
  }));

  // 4. Chat loop with Ollama
  const ollama = new Ollama.Ollama();
  const messages: any[] = [
    {
      role: "system",
      content:
        "You are a helpful robot assistant. Use the available tools to " +
        "interact with the ROS2 robot. Always check the safety status " +
        "before sending movement commands.",
    },
    {
      role: "user",
      content: "List all available ROS2 topics",
    },
  ];

  const response = await ollama.chat({
    model: "qwen2.5:14b",
    messages,
    tools: ollamaTools,
  });

  // 5. If the model made a tool call, execute it via MCP
  if (response.message.tool_calls) {
    for (const toolCall of response.message.tool_calls) {
      console.log(`Calling tool: ${toolCall.function.name}`);

      const result = await mcpClient.callTool({
        name: toolCall.function.name,
        arguments: toolCall.function.arguments,
      });

      console.log("Result:", JSON.stringify(result, null, 2));
    }
  } else {
    console.log("Response:", response.message.content);
  }

  await mcpClient.close();
}

main().catch(console.error);
```

Run it:

```bash
npx tsx ollama-robot.ts
```

### Step 3: Verify safety enforcement

Test that the safety layer works identically with your local LLM:

```typescript
// This should be BLOCKED by the safety layer, regardless of which LLM sent it
const dangerousResult = await mcpClient.callTool({
  name: "ros2_topic_publish",
  arguments: {
    topic: "/cmd_vel",
    message_type: "geometry_msgs/msg/Twist",
    data: {
      linear: { x: 5.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 },
    },
  },
});

// Expected: SAFETY BLOCKED: velocity_exceeded
console.log(dangerousResult);
```

---

## Option 2: llama.cpp with Function Calling

[llama.cpp](https://github.com/ggerganov/llama.cpp) can serve models with function-calling support via its built-in HTTP server.

### Step 1: Start llama.cpp server with function calling

```bash
# Build llama.cpp (if not already built)
cd llama.cpp && make -j

# Start the server with a tool-calling model
./llama-server \
  -m models/qwen2.5-14b-instruct-q4_k_m.gguf \
  --port 8080 \
  -ngl 99 \
  --jinja
```

The `--jinja` flag enables chat template processing, which is needed for tool-calling format.

### Step 2: Connect via an MCP client

Use the same MCP client pattern as the Ollama example, but point the LLM calls at the llama.cpp server's OpenAI-compatible endpoint:

```typescript
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { StdioClientTransport } from "@modelcontextprotocol/sdk/client/stdio.js";
import OpenAI from "openai";

async function main() {
  // Connect to PhysicalMCP
  const transport = new StdioClientTransport({
    command: "npx",
    args: ["@ricardothe3rd/physical-mcp"],
    env: {
      ...process.env,
      PHYSICAL_MCP_BRIDGE_URL: "ws://localhost:9090",
    },
  });

  const mcpClient = new Client({
    name: "llamacpp-robot-client",
    version: "1.0.0",
  });
  await mcpClient.connect(transport);

  // Discover tools
  const { tools } = await mcpClient.listTools();
  const openaiTools = tools.map((tool) => ({
    type: "function" as const,
    function: {
      name: tool.name,
      description: tool.description || "",
      parameters: tool.inputSchema,
    },
  }));

  // Connect to llama.cpp's OpenAI-compatible API
  const llm = new OpenAI({
    baseURL: "http://localhost:8080/v1",
    apiKey: "not-needed",
  });

  const response = await llm.chat.completions.create({
    model: "local-model",
    messages: [
      {
        role: "system",
        content:
          "You are a robot assistant. Use tools to interact with the ROS2 robot safely.",
      },
      { role: "user", content: "What topics are available on the robot?" },
    ],
    tools: openaiTools,
  });

  // Execute any tool calls through MCP
  const choice = response.choices[0];
  if (choice.message.tool_calls) {
    for (const toolCall of choice.message.tool_calls) {
      const args = JSON.parse(toolCall.function.arguments);
      const result = await mcpClient.callTool({
        name: toolCall.function.name,
        arguments: args,
      });
      console.log(`${toolCall.function.name}:`, JSON.stringify(result, null, 2));
    }
  }

  await mcpClient.close();
}

main().catch(console.error);
```

---

## Option 3: vLLM with MCP Adapter

[vLLM](https://github.com/vllm-project/vllm) provides high-throughput LLM inference with an OpenAI-compatible API. It supports tool calling with compatible models.

### Step 1: Start vLLM

```bash
# Install vLLM
pip install vllm

# Start vLLM with a tool-calling model
vllm serve Qwen/Qwen2.5-14B-Instruct \
  --port 8000 \
  --enable-auto-tool-choice \
  --tool-call-parser hermes
```

The `--enable-auto-tool-choice` and `--tool-call-parser` flags enable function-calling support.

### Step 2: Connect via MCP client

Use the same OpenAI-compatible client pattern shown in the llama.cpp example, but change the `baseURL` to `http://localhost:8000/v1`. The MCP client code is identical -- only the LLM endpoint changes.

```typescript
const llm = new OpenAI({
  baseURL: "http://localhost:8000/v1",
  apiKey: "not-needed",
});
```

Everything else -- MCP tool discovery, tool call execution, safety enforcement -- works the same way.

---

## Option 4: Any LLM with an MCP Client Library

The pattern for using PhysicalMCP with any LLM is:

1. **Spawn PhysicalMCP** as a subprocess using an MCP client library.
2. **Discover tools** by calling `listTools()` on the MCP client.
3. **Convert MCP tools** to your LLM's function-calling format.
4. **Send the user's message** to the LLM with the tools attached.
5. **Execute tool calls** returned by the LLM through the MCP client.
6. **Feed results back** to the LLM and repeat.

### MCP Client Libraries

MCP client libraries are available in multiple languages:

| Language | Library | Notes |
|----------|---------|-------|
| TypeScript/JavaScript | `@modelcontextprotocol/sdk` | Official SDK. Most mature. |
| Python | `mcp` | Official Python SDK. |
| Rust | `mcp-rust-sdk` | Community SDK. |
| Go | `mcp-go` | Community SDK. |

### Python Example

```python
import asyncio
import json
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

async def main():
    # 1. Connect to PhysicalMCP
    server_params = StdioServerParameters(
        command="npx",
        args=["@ricardothe3rd/physical-mcp"],
        env={
            "PHYSICAL_MCP_BRIDGE_URL": "ws://localhost:9090",
        },
    )

    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            # 2. Discover tools
            tools_response = await session.list_tools()
            print(f"Discovered {len(tools_response.tools)} tools:")
            for tool in tools_response.tools:
                print(f"  - {tool.name}: {tool.description}")

            # 3. Call a tool directly
            result = await session.call_tool(
                "ros2_topic_list",
                arguments={},
            )
            print(f"\nROS2 Topics: {result}")

            # 4. Test safety enforcement
            result = await session.call_tool(
                "safety_status",
                arguments={},
            )
            print(f"\nSafety Status: {result}")

            # 5. Integrate with your LLM of choice
            # Convert tools_response.tools to your LLM's format
            # Send user message + tools to LLM
            # Execute tool calls from LLM response
            # Feed results back to LLM


asyncio.run(main())
```

Install dependencies:

```bash
pip install mcp
```

Run:

```bash
python ollama_robot.py
```

### Connecting the Python MCP Client to a Local LLM

Here is a more complete example that connects the MCP client to an Ollama model:

```python
import asyncio
import json
import httpx
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


async def chat_with_robot(user_message: str):
    server_params = StdioServerParameters(
        command="npx",
        args=["@ricardothe3rd/physical-mcp"],
        env={"PHYSICAL_MCP_BRIDGE_URL": "ws://localhost:9090"},
    )

    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            # Discover and convert tools
            tools_response = await session.list_tools()
            ollama_tools = []
            for tool in tools_response.tools:
                ollama_tools.append({
                    "type": "function",
                    "function": {
                        "name": tool.name,
                        "description": tool.description or "",
                        "parameters": tool.inputSchema,
                    },
                })

            # Send to Ollama
            async with httpx.AsyncClient() as http:
                response = await http.post(
                    "http://localhost:11434/api/chat",
                    json={
                        "model": "qwen2.5:14b",
                        "messages": [
                            {
                                "role": "system",
                                "content": (
                                    "You control a ROS2 robot through tools. "
                                    "Always check safety status before moving."
                                ),
                            },
                            {"role": "user", "content": user_message},
                        ],
                        "tools": ollama_tools,
                        "stream": False,
                    },
                    timeout=120.0,
                )
                data = response.json()

            # Execute tool calls
            message = data.get("message", {})
            tool_calls = message.get("tool_calls", [])

            if tool_calls:
                for tc in tool_calls:
                    name = tc["function"]["name"]
                    args = tc["function"]["arguments"]
                    print(f"Calling {name} with {json.dumps(args)}")

                    result = await session.call_tool(name, arguments=args)
                    print(f"Result: {result}\n")
            else:
                print(f"Response: {message.get('content', '')}")


asyncio.run(chat_with_robot("List all ROS2 topics on the robot"))
```

---

## Option 5: LangChain or LlamaIndex with MCP

Both LangChain and LlamaIndex have MCP integration support, making it straightforward to use PhysicalMCP as a tool provider for agents built with these frameworks.

### LangChain Example

```python
from langchain_mcp_adapters.client import MultiServerMCPClient
from langchain_ollama import ChatOllama
from langgraph.prebuilt import create_react_agent

async def main():
    # Connect to PhysicalMCP as an MCP server
    async with MultiServerMCPClient(
        {
            "physical-mcp": {
                "command": "npx",
                "args": ["@ricardothe3rd/physical-mcp"],
                "env": {
                    "PHYSICAL_MCP_BRIDGE_URL": "ws://localhost:9090",
                },
            }
        }
    ) as mcp_client:
        # Get tools from PhysicalMCP
        tools = mcp_client.get_tools()

        # Create a LangChain agent with a local LLM
        llm = ChatOllama(model="qwen2.5:14b")
        agent = create_react_agent(llm, tools)

        # Run the agent
        result = await agent.ainvoke(
            {"messages": [{"role": "user", "content": "List all ROS2 topics"}]}
        )
        print(result)
```

Install dependencies:

```bash
pip install langchain-mcp-adapters langchain-ollama langgraph
```

### LlamaIndex Example

```python
from llama_index.tools.mcp import BasicMCPClient, McpToolSpec

async def main():
    # Connect to PhysicalMCP
    mcp_client = BasicMCPClient(
        command="npx",
        args=["@ricardothe3rd/physical-mcp"],
        env={"PHYSICAL_MCP_BRIDGE_URL": "ws://localhost:9090"},
    )
    mcp_tool_spec = McpToolSpec(client=mcp_client)

    # Get tools
    tools = await mcp_tool_spec.to_tool_list_async()

    # Use with any LlamaIndex agent and LLM
    # ...
```

Install dependencies:

```bash
pip install llama-index-tools-mcp
```

---

## Writing Your Own MCP Client

If none of the above options fit your setup, you can write a minimal MCP client. The MCP protocol is JSON-RPC 2.0 over stdio. Here is the minimal interaction flow:

### 1. Spawn the PhysicalMCP process

```bash
npx @ricardothe3rd/physical-mcp
```

The process reads JSON-RPC messages from stdin and writes responses to stdout.

### 2. Initialize the session

Send:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "initialize",
  "params": {
    "protocolVersion": "2024-11-05",
    "capabilities": {},
    "clientInfo": { "name": "my-client", "version": "1.0.0" }
  }
}
```

### 3. Discover tools

Send:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/list",
  "params": {}
}
```

The response contains all 27 tools with their names, descriptions, and JSON Schema input definitions.

### 4. Call a tool

Send:

```json
{
  "jsonrpc": "2.0",
  "id": 3,
  "method": "tools/call",
  "params": {
    "name": "ros2_topic_list",
    "arguments": {}
  }
}
```

### 5. Feed the result back to your LLM

Take the tool result, format it as a function call response in your LLM's format, and send it back to the model for the next turn.

The full MCP specification is available at [modelcontextprotocol.io](https://modelcontextprotocol.io). PhysicalMCP implements the standard tools capability.

---

## Model Recommendations

Not all LLMs handle function calling well. For best results with PhysicalMCP, use models that have been specifically trained or fine-tuned for tool use.

### Recommended Models (Local)

| Model | Size | Tool Calling | Notes |
|-------|------|:------------:|-------|
| Qwen 2.5 Instruct | 7B / 14B / 32B / 72B | Excellent | Best tool-calling performance in its size class. 14B is a good balance of quality and speed. |
| Llama 3.1 Instruct | 8B / 70B | Good | Solid tool calling. 8B is fast but less reliable for complex multi-tool sequences. |
| Mistral Nemo | 12B | Good | Strong tool calling for its size. |
| DeepSeek-V3 | 671B (MoE) | Excellent | Outstanding tool calling, but requires significant hardware. |
| Hermes 3 | Various | Good | Specifically tuned for function calling via the Hermes format. |

### Minimum Hardware Requirements

| Model Size | RAM Required | GPU VRAM (if offloading) |
|-----------|-------------|-------------------------|
| 7-8B (Q4) | 6 GB | 6 GB |
| 14B (Q4) | 10 GB | 10 GB |
| 32B (Q4) | 20 GB | 20 GB |
| 70B (Q4) | 40 GB | 40 GB |

For robotics applications, we recommend at least a 14B parameter model. Smaller models (7-8B) can make tool calls but are more likely to hallucinate parameter values (e.g., velocity values that are unreasonably high). PhysicalMCP's safety layer catches these errors, but a better model produces a better experience.

### Models to Avoid for Tool Calling

- Base models (non-instruct variants) -- they do not follow tool-calling formats
- Very small models (< 3B parameters) -- unreliable tool-calling behavior
- Models without explicit function-calling training -- they may generate tool calls in the wrong format

---

## Safety Considerations for Local LLMs

PhysicalMCP's safety layer works identically regardless of which LLM drives it. However, local LLMs deserve extra attention for several reasons:

### 1. Local LLMs hallucinate more aggressively

Smaller models are more likely to hallucinate parameter values. For example, when asked to "move the robot forward slowly," a small model might generate a velocity of 2.0 m/s instead of 0.1 m/s. PhysicalMCP's velocity limits catch this, but you should be aware of it.

**Mitigation:** Set conservative velocity limits. Use block mode (not clamp mode). Start with limits well below the robot's maximum.

### 2. Local LLMs may not respect safety instructions

A smaller model may ignore system prompt instructions about safety. It might call `safety_emergency_stop_release` without being asked, or attempt to publish to blocked topics repeatedly.

**Mitigation:** PhysicalMCP enforces safety at the server level, not through prompt engineering. Blocked topics are blocked regardless of what the LLM tries. E-stop release requires the explicit `CONFIRM_RELEASE` string. The safety layer does not trust the LLM.

### 3. Tool-calling format errors are more common

Local LLMs sometimes generate malformed tool calls (wrong parameter names, missing required fields, incorrect types). PhysicalMCP validates all inputs with Zod schemas and returns clear error messages.

**Mitigation:** Use models known for reliable tool calling (see [Model Recommendations](#model-recommendations)). Implement retry logic in your MCP client for transient formatting errors.

### 4. Verify safety enforcement before deploying

Before using a local LLM with real hardware:

1. Run the model against PhysicalMCP in simulation first.
2. Deliberately test safety boundaries: ask the model to move at dangerous speeds, publish to blocked topics, and send commands while e-stop is active.
3. Verify that all dangerous commands are blocked by the safety layer.
4. Review the audit log (`safety_audit_log`) to see what the model attempted.

```
"Show me the safety audit log filtered to violations only"
```

If the model generates a high number of safety violations, consider using a larger model or adding more specific instructions to the system prompt.

---

## Troubleshooting

### PhysicalMCP tools are not showing up in my LLM's tool list

- Verify PhysicalMCP is installed: `npx @ricardothe3rd/physical-mcp --help`
- Verify the MCP client can spawn the process: run `npx @ricardothe3rd/physical-mcp` manually and check for errors.
- Check that you are calling `listTools()` after `initialize()` on the MCP client.

### The LLM generates tool calls but they fail with parameter errors

- The LLM is generating parameters in the wrong format. Check the error message from PhysicalMCP -- it will indicate which parameter is invalid.
- Try a larger or better model with stronger tool-calling capabilities.
- Add explicit examples in the system prompt showing correct tool call format.

### Tool calls succeed but the LLM does not process the results

- Ensure you are feeding tool results back to the LLM as function call responses.
- Some LLMs require a specific format for tool results (e.g., a `tool` role message). Check your LLM's documentation.

### The LLM keeps retrying blocked commands

- This is expected behavior from some models. PhysicalMCP blocks every attempt and logs it.
- Add a system prompt instruction: "If a command is blocked by the safety layer, do not retry it. Explain the safety violation to the user instead."
- Rate limiting in PhysicalMCP prevents the retries from causing harm.

### High latency between tool calls

- If using a large model locally, inference time can be significant. PhysicalMCP adds minimal overhead (< 5 ms per safety check).
- Consider using a smaller model for interactive use and a larger model for planning.
- For llama.cpp, increase `--n-gpu-layers` to offload more layers to GPU.
- For vLLM, ensure you have enough GPU memory for KV cache.

### Connection issues when spawning PhysicalMCP from an MCP client

- Ensure `npx` is on the system PATH visible to your MCP client process.
- If using Docker or a virtual environment, the MCP client may not inherit the correct PATH. Set it explicitly in the spawn options.
- Check that the `PHYSICAL_MCP_BRIDGE_URL` environment variable is correctly passed through to the spawned process.

---

## Summary

PhysicalMCP is LLM-agnostic by design. The safety layer enforces the same policies whether the commands come from Claude, GPT-4, a 7B Llama model running on a laptop, or a 70B model on a GPU cluster. The integration pattern is consistent across all setups:

1. Use an MCP client library to spawn and connect to PhysicalMCP.
2. Discover PhysicalMCP's tools via the standard `tools/list` method.
3. Convert those tools to your LLM's function-calling format.
4. Execute tool calls from the LLM through the MCP client.
5. Trust the safety layer -- it works regardless of the LLM.

The MCP protocol is an open standard. As more LLM frameworks add native MCP support, connecting local LLMs to PhysicalMCP will become even simpler. The safety guarantees remain the same.

---

**Links:**
- [PhysicalMCP GitHub](https://github.com/ricardothe3rd/physical-mcp)
- [npm: @ricardothe3rd/physical-mcp](https://www.npmjs.com/package/@ricardothe3rd/physical-mcp)
- [MCP Specification](https://modelcontextprotocol.io)
- [Ollama](https://ollama.ai)
- [llama.cpp](https://github.com/ggerganov/llama.cpp)
- [vLLM](https://github.com/vllm-project/vllm)
- [MCP TypeScript SDK](https://github.com/modelcontextprotocol/typescript-sdk)
- [MCP Python SDK](https://github.com/modelcontextprotocol/python-sdk)

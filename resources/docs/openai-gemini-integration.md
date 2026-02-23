# PhysicalMCP: OpenAI & Gemini Integration Guide

> How to connect PhysicalMCP to OpenAI (GPT-4o, o3) and Google Gemini models.

---

## Quick Summary

PhysicalMCP is a standard MCP server. It works with **any** MCP-compatible client out of the box — no code changes needed. Here's how each platform connects:

| Platform | Method | Complexity |
|----------|--------|------------|
| Claude Desktop/Code | `claude mcp add` | 1 command |
| OpenAI Agents SDK | `MCPServerStdio` | 10 lines Python |
| OpenAI Responses API | Remote MCP tool | API call |
| ChatGPT Desktop | Connectors UI | Needs public HTTPS endpoint |
| Google Gemini CLI | `settings.json` | 1 JSON file |
| Google ADK | `McpToolset` | 10 lines Python |
| Cursor / Continue / Cline | `mcp.json` config | 1 JSON file |
| Any LLM via mcp-use | `MCPAgent` | 5 lines Python |

---

## OpenAI Integration

### Option 1: OpenAI Agents SDK (Recommended)

```bash
pip install openai-agents
```

```python
from agents import Agent, Runner
from agents.mcp import MCPServerStdio

async with MCPServerStdio(
    name="PhysicalMCP",
    params={
        "command": "npx",
        "args": ["@ricardothe3rd/physical-mcp"],
    },
) as server:
    agent = Agent(
        name="Robot Operator",
        instructions="Control the robot safely using available tools.",
        mcp_servers=[server],
    )
    result = await Runner.run(agent, "Move the robot forward 0.5 meters.")
    print(result.final_output)
```

### Option 2: OpenAI Responses API (Direct)

```python
from openai import OpenAI

client = OpenAI()
response = client.responses.create(
    model="gpt-4o",
    tools=[{
        "type": "mcp",
        "server_label": "physical_mcp",
        "server_url": "https://your-physical-mcp-host.com/mcp",
        "require_approval": "always",  # Recommended for safety
    }],
    input="What is the robot's current position?",
)
```

> Note: Responses API requires a publicly reachable HTTPS endpoint.

### Option 3: ChatGPT Desktop (Connectors)

1. ChatGPT → Settings → Connectors → Create
2. Name: "PhysicalMCP Robot Control"
3. Paste your MCP server's HTTPS endpoint
4. Requires ChatGPT Plus/Pro/Team/Enterprise

---

## Google Gemini Integration

### Option 1: Gemini CLI (Easiest)

```bash
npm install -g @google/gemini-cli
```

Add to `~/.gemini/settings.json`:
```json
{
  "mcpServers": {
    "physical-mcp": {
      "command": "npx",
      "args": ["@ricardothe3rd/physical-mcp"]
    }
  }
}
```

### Option 2: Google ADK (Agent Framework)

```bash
pip install google-adk
```

```python
from google.adk.agents import LlmAgent
from google.adk.tools.mcp_tool import McpToolset
from google.adk.tools.mcp_tool.mcp_session_manager import StdioConnectionParams
from mcp import StdioServerParameters

root_agent = LlmAgent(
    model='gemini-2.0-flash',
    name='robot_controller',
    instruction='Control the robot safely.',
    tools=[
        McpToolset(
            connection_params=StdioConnectionParams(
                server_params=StdioServerParameters(
                    command='npx',
                    args=['@ricardothe3rd/physical-mcp'],
                ),
            ),
        )
    ],
)
```

### Option 3: Gemini Python SDK (Experimental)

```bash
pip install "google-genai>=1.55.0" mcp
```

```python
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
from google import genai

async with stdio_client(StdioServerParameters(
    command="npx", args=["@ricardothe3rd/physical-mcp"]
)) as (read, write):
    async with ClientSession(read, write) as session:
        await session.initialize()
        response = await genai.Client().aio.models.generate_content(
            model="gemini-2.0-flash",
            contents="List available robot tools",
            config=genai.types.GenerateContentConfig(tools=[session]),
        )
```

---

## Any LLM via mcp-use

Works with OpenAI, Gemini, Ollama, or any LangChain-compatible model:

```bash
pip install mcp-use langchain-openai  # or langchain-google-genai
```

```python
from langchain_openai import ChatOpenAI  # or ChatGoogleGenerativeAI
from mcp_use import MCPAgent, MCPClient

config = {
    "mcpServers": {
        "physical-mcp": {
            "command": "npx",
            "args": ["@ricardothe3rd/physical-mcp"]
        }
    }
}
client = MCPClient.from_dict(config)
llm = ChatOpenAI(model="gpt-4o")
agent = MCPAgent(llm=llm, client=client)
result = await agent.run("Move the robot to the home position.")
```

---

## Editor/IDE Integration

### Cursor
Add to `~/.cursor/mcp.json`:
```json
{
  "mcpServers": {
    "physical-mcp": {
      "command": "npx",
      "args": ["@ricardothe3rd/physical-mcp"]
    }
  }
}
```

### Continue (VS Code / JetBrains)
Add `mcpServers` block to Continue config with the same format as above.

### Cline (VS Code)
Configure via the Cline extension settings UI — add PhysicalMCP as an MCP server.

---

## Tool Filtering (Security Best Practice)

When giving AI agents access to PhysicalMCP, limit which tools they can see:

**OpenAI Agents SDK:**
```python
from agents.mcp import create_static_tool_filter

server = MCPServerStdio(
    params={...},
    tool_filter=create_static_tool_filter(
        allowed_tool_names=["ros2_topic_list", "ros2_topic_echo", "safety_status"]
    ),
)
```

**Google ADK:**
```python
McpToolset(
    connection_params=StdioConnectionParams(...),
    tool_filter=["ros2_topic_list", "ros2_topic_echo", "safety_status"]
)
```

**PhysicalMCP built-in profiles:**
```bash
PHYSICAL_MCP_TOOL_PROFILE=minimal npx @ricardothe3rd/physical-mcp  # Read-only tools
PHYSICAL_MCP_TOOL_PROFILE=standard npx @ricardothe3rd/physical-mcp # + publish/call
PHYSICAL_MCP_TOOL_PROFILE=full npx @ricardothe3rd/physical-mcp     # All 100 tools
```

---

*Last updated: 2026-02-23*

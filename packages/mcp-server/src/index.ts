#!/usr/bin/env node

/**
 * PhysicalMCP - Safety-first MCP server for ROS2 robots.
 *
 * Bridges AI agents (Claude, GPT, etc.) to physical robots via ROS2
 * with velocity limits, geofence, rate limiting, e-stop, and audit logging.
 */

import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
} from '@modelcontextprotocol/sdk/types.js';

import { ConnectionManager } from './bridge/connection-manager.js';
import { PolicyEngine } from './safety/policy-engine.js';

import { getTopicTools, handleTopicTool } from './tools/topic-tools.js';
import { getServiceTools, handleServiceTool } from './tools/service-tools.js';
import { getActionTools, handleActionTool } from './tools/action-tools.js';
import { getSafetyTools, handleSafetyTool } from './tools/safety-tools.js';
import { getSystemTools, handleSystemTool } from './tools/system-tools.js';

// Config from environment
const BRIDGE_URL = process.env.PHYSICAL_MCP_BRIDGE_URL || 'ws://localhost:9090';
const POLICY_PATH = process.env.PHYSICAL_MCP_POLICY || undefined;

// Core instances
const connection = new ConnectionManager(BRIDGE_URL);
const safety = new PolicyEngine(POLICY_PATH);

// MCP Server
const server = new Server(
  {
    name: '@ricardothe3rd/physical-mcp',
    version: '0.1.0',
  },
  {
    capabilities: {
      tools: {},
    },
  }
);

// Collect all tools
const allTools = [
  ...getTopicTools(),
  ...getServiceTools(),
  ...getActionTools(),
  ...getSafetyTools(),
  ...getSystemTools(),
];

// Tool name -> category mapping for dispatch
const topicToolNames = new Set(getTopicTools().map(t => t.name));
const serviceToolNames = new Set(getServiceTools().map(t => t.name));
const actionToolNames = new Set(getActionTools().map(t => t.name));
const safetyToolNames = new Set(getSafetyTools().map(t => t.name));
const systemToolNames = new Set(getSystemTools().map(t => t.name));

// List tools
server.setRequestHandler(ListToolsRequestSchema, async () => {
  return { tools: allTools };
});

// Handle tool calls
server.setRequestHandler(CallToolRequestSchema, async (request) => {
  const { name, arguments: args } = request.params;
  const toolArgs = (args || {}) as Record<string, unknown>;

  try {
    // Ensure bridge connection for non-safety tools
    if (!safetyToolNames.has(name) && !connection.isConnected) {
      try {
        await connection.connect();
      } catch {
        // Safety tools work without bridge
        if (!safetyToolNames.has(name)) {
          return {
            content: [{
              type: 'text',
              text: `Bridge not connected. Start the ROS2 bridge and try again.\nExpected bridge at: ${BRIDGE_URL}`,
            }],
            isError: true,
          };
        }
      }
    }

    // Dispatch to the right handler
    if (topicToolNames.has(name)) {
      return await handleTopicTool(name, toolArgs, connection, safety);
    }
    if (serviceToolNames.has(name)) {
      return await handleServiceTool(name, toolArgs, connection, safety);
    }
    if (actionToolNames.has(name)) {
      return await handleActionTool(name, toolArgs, connection, safety);
    }
    if (safetyToolNames.has(name)) {
      return await handleSafetyTool(name, toolArgs, connection, safety);
    }
    if (systemToolNames.has(name)) {
      return await handleSystemTool(name, toolArgs, connection);
    }

    return {
      content: [{ type: 'text', text: `Unknown tool: ${name}` }],
      isError: true,
    };
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    console.error(`[PhysicalMCP] Tool error (${name}): ${message}`);
    return {
      content: [{ type: 'text', text: `Error: ${message}` }],
      isError: true,
    };
  }
});

// Graceful shutdown
process.on('SIGINT', () => {
  console.error('[PhysicalMCP] Shutting down...');
  connection.disconnect();
  process.exit(0);
});

process.on('SIGTERM', () => {
  console.error('[PhysicalMCP] Terminated');
  connection.disconnect();
  process.exit(0);
});

// Start
async function main() {
  const transport = new StdioServerTransport();

  console.error('[PhysicalMCP] Starting safety-first MCP server for ROS2 v0.1.0');
  console.error(`[PhysicalMCP] Bridge URL: ${BRIDGE_URL}`);
  console.error(`[PhysicalMCP] Policy: ${POLICY_PATH || 'default'}`);
  console.error(`[PhysicalMCP] Tools registered: ${allTools.length}`);

  await server.connect(transport);
}

main().catch((error) => {
  console.error('[PhysicalMCP] Failed to start:', error);
  process.exit(1);
});

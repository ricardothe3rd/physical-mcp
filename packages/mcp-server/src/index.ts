#!/usr/bin/env node

/**
 * PhysicalMCP - Safety-first MCP server for ROS2 robots.
 *
 * Bridges AI agents (Claude, GPT, etc.) to physical robots via ROS2
 * with velocity limits, geofence, rate limiting, e-stop, and audit logging.
 */

import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';
import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
} from '@modelcontextprotocol/sdk/types.js';

import { ConnectionManager } from './bridge/connection-manager.js';
import { PolicyEngine } from './safety/policy-engine.js';
import { runStartupChecks, printStartupChecks } from './utils/startup-check.js';

import { getTopicTools, handleTopicTool } from './tools/topic-tools.js';
import { getServiceTools, handleServiceTool } from './tools/service-tools.js';
import { getActionTools, handleActionTool } from './tools/action-tools.js';
import { getSafetyTools, handleSafetyTool } from './tools/safety-tools.js';
import { getSystemTools, handleSystemTool } from './tools/system-tools.js';
import { getBatchTools, handleBatchTool } from './tools/batch-tools.js';
import { getRecordingTools, handleRecordingTool } from './tools/recording-tools.js';

// --- CLI argument parsing ---
function parseArgs(): { bridgeUrl: string; policyPath?: string; verbose: boolean } {
  const args = process.argv.slice(2);

  if (args.includes('--help') || args.includes('-h')) {
    console.error(`
PhysicalMCP - Safety-first MCP server for ROS2 robots

Usage: physical-mcp [options]

Options:
  --bridge-url <url>   WebSocket URL for the ROS2 bridge (default: ws://localhost:9090)
  --policy <path>      Path to a YAML safety policy file
  --verbose            Enable verbose logging
  --version            Show version number
  --help               Show this help message

Environment variables:
  PHYSICAL_MCP_BRIDGE_URL   Same as --bridge-url
  PHYSICAL_MCP_POLICY       Same as --policy
`);
    process.exit(0);
  }

  if (args.includes('--version') || args.includes('-v')) {
    try {
      const __dirname = dirname(fileURLToPath(import.meta.url));
      const pkg = JSON.parse(readFileSync(resolve(__dirname, '../package.json'), 'utf-8'));
      console.error(pkg.version);
    } catch {
      console.error('0.1.0');
    }
    process.exit(0);
  }

  let bridgeUrl = process.env.PHYSICAL_MCP_BRIDGE_URL || 'ws://localhost:9090';
  let policyPath = process.env.PHYSICAL_MCP_POLICY || undefined;
  let verbose = false;

  for (let i = 0; i < args.length; i++) {
    switch (args[i]) {
      case '--bridge-url':
        bridgeUrl = args[++i];
        break;
      case '--policy':
        policyPath = args[++i];
        break;
      case '--verbose':
        verbose = true;
        break;
    }
  }

  return { bridgeUrl, policyPath, verbose };
}

const config = parseArgs();

// Startup self-test
const startupResult = runStartupChecks(config);
printStartupChecks(startupResult);
if (!startupResult.passed) {
  process.exit(1);
}

// Core instances
const connection = new ConnectionManager(config.bridgeUrl);
const safety = new PolicyEngine(config.policyPath);

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
  ...getBatchTools(),
  ...getRecordingTools(),
];

// Tool name -> category mapping for dispatch
const topicToolNames = new Set(getTopicTools().map(t => t.name));
const serviceToolNames = new Set(getServiceTools().map(t => t.name));
const actionToolNames = new Set(getActionTools().map(t => t.name));
const safetyToolNames = new Set(getSafetyTools().map(t => t.name));
const systemToolNames = new Set(getSystemTools().map(t => t.name));
const batchToolNames = new Set(getBatchTools().map(t => t.name));
const recordingToolNames = new Set(getRecordingTools().map(t => t.name));

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
              text: `Bridge not connected. Start the ROS2 bridge and try again.\nExpected bridge at: ${config.bridgeUrl}\n\nTroubleshooting:\n- Docker: cd docker && docker compose up\n- Local: physical-mcp-bridge`,
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
    if (batchToolNames.has(name)) {
      return await handleBatchTool(name, toolArgs, connection, safety);
    }
    if (recordingToolNames.has(name)) {
      return await handleRecordingTool(name, toolArgs, connection);
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
  console.error(`[PhysicalMCP] Bridge URL: ${config.bridgeUrl}`);
  console.error(`[PhysicalMCP] Policy: ${config.policyPath || 'default'}`);
  console.error(`[PhysicalMCP] Tools registered: ${allTools.length}`);
  if (config.verbose) {
    console.error('[PhysicalMCP] Verbose logging enabled');
  }

  await server.connect(transport);
}

main().catch((error) => {
  console.error('[PhysicalMCP] Failed to start:', error);
  process.exit(1);
});

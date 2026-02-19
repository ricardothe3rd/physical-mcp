/**
 * ROS2 parameter MCP tools: list, get, set.
 *
 * Allows AI agents to read and write ROS2 node parameters.
 * The `ros2_param_set` tool is SAFETY CHECKED through the policy engine
 * (treated as a service call to the node's set_parameters service).
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';
import { validateNodeName, validateParamName } from '../utils/input-validation.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getParamTools(): Tool[] {
  return [
    {
      name: 'ros2_param_list',
      description: 'List all parameters for a ROS2 node',
      inputSchema: toInputSchema(z.object({
        nodeName: z.string().describe('Full node name (e.g. /turtlesim)'),
      })),
    },
    {
      name: 'ros2_param_get',
      description: 'Get the value of a ROS2 parameter',
      inputSchema: toInputSchema(z.object({
        nodeName: z.string().describe('Full node name (e.g. /turtlesim)'),
        paramName: z.string().describe('Parameter name (e.g. background_r)'),
      })),
    },
    {
      name: 'ros2_param_set',
      description: 'Set the value of a ROS2 parameter. SAFETY CHECKED — validated through the safety policy engine before execution.',
      inputSchema: toInputSchema(z.object({
        nodeName: z.string().describe('Full node name (e.g. /turtlesim)'),
        paramName: z.string().describe('Parameter name (e.g. background_r)'),
        value: z.unknown().describe('New parameter value (string, number, boolean, or array)'),
      })),
    },
  ];
}

export async function handleParamTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_param_list': {
      const nodeName = args.nodeName as string;
      const nameCheck = validateNodeName(nodeName);
      if (!nameCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid node name: ${nameCheck.error}` }], isError: true };
      }

      const response = await connection.send(CommandType.LIST_PARAMS, {
        node_name: nodeName,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_param_get': {
      const nodeName = args.nodeName as string;
      const paramName = args.paramName as string;

      const nameCheck = validateNodeName(nodeName);
      if (!nameCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid node name: ${nameCheck.error}` }], isError: true };
      }
      const paramCheck = validateParamName(paramName);
      if (!paramCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid parameter name: ${paramCheck.error}` }], isError: true };
      }

      const response = await connection.send(CommandType.GET_PARAMS, {
        node_name: nodeName,
        param_name: paramName,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `${nodeName}/${paramName} = ${JSON.stringify(response.data)}`,
        }],
      };
    }

    case 'ros2_param_set': {
      const nodeName = args.nodeName as string;
      const paramName = args.paramName as string;
      const value = args.value;

      // INPUT VALIDATION
      const nameCheck = validateNodeName(nodeName);
      if (!nameCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid node name: ${nameCheck.error}` }], isError: true };
      }
      const paramCheck = validateParamName(paramName);
      if (!paramCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid parameter name: ${paramCheck.error}` }], isError: true };
      }

      // SAFETY CHECK — treat param set as a service call to the node's set_parameters service
      const safetyService = `${nodeName}/set_parameters`;
      const safetyArgs = { param_name: paramName, value };
      const check = safety.checkServiceCall(safetyService, safetyArgs);
      if (!check.allowed) {
        const violationText = check.violations
          .map(v => `- [${v.type}] ${v.message}`)
          .join('\n');
        return {
          content: [{ type: 'text', text: `SAFETY BLOCKED: Parameter set on ${nodeName}/${paramName} denied.\n\nViolations:\n${violationText}` }],
          isError: true,
        };
      }

      const response = await connection.send(CommandType.SET_PARAMS, {
        node_name: nodeName,
        param_name: paramName,
        value,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return {
        content: [{
          type: 'text',
          text: `Set ${nodeName}/${paramName} = ${JSON.stringify(value)}`,
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown param tool: ${name}` }], isError: true };
  }
}

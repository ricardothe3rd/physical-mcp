/**
 * ROS2 service MCP tools: list, info, call.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';
import { validateServiceName } from '../utils/input-validation.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getServiceTools(): Tool[] {
  return [
    {
      name: 'ros2_service_list',
      description: 'List all available ROS2 services with their types',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_service_info',
      description: 'Get detailed info about a specific ROS2 service',
      inputSchema: toInputSchema(z.object({
        service: z.string().describe('Full service name (e.g. /spawn_entity)'),
      })),
    },
    {
      name: 'ros2_service_call',
      description: 'Call a ROS2 service with given arguments. Subject to safety checks.',
      inputSchema: toInputSchema(z.object({
        service: z.string().describe('Full service name'),
        serviceType: z.string().describe('Service type (e.g. std_srvs/srv/SetBool)'),
        args: z.record(z.unknown()).default({}).describe('Service call arguments as JSON'),
      })),
    },
  ];
}

export async function handleServiceTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_service_list': {
      const response = await connection.send(CommandType.SERVICE_LIST);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_service_info': {
      const svcName = args.service as string;
      const svcCheck = validateServiceName(svcName);
      if (!svcCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid service name: ${svcCheck.error}` }], isError: true };
      }
      const response = await connection.send(CommandType.SERVICE_INFO, { service: svcName });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_service_call': {
      const service = args.service as string;
      const callArgs = (args.args || {}) as Record<string, unknown>;

      // INPUT VALIDATION
      const callCheck = validateServiceName(service);
      if (!callCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid service name: ${callCheck.error}` }], isError: true };
      }

      // SAFETY CHECK
      const check = safety.checkServiceCall(service, callArgs);
      if (!check.allowed) {
        const violationText = check.violations
          .map(v => `- [${v.type}] ${v.message}`)
          .join('\n');
        return {
          content: [{ type: 'text', text: `SAFETY BLOCKED: Service call to ${service} denied.\n\nViolations:\n${violationText}` }],
          isError: true,
        };
      }

      const response = await connection.send(CommandType.SERVICE_CALL, {
        service,
        service_type: args.serviceType,
        args: callArgs,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown service tool: ${name}` }], isError: true };
  }
}

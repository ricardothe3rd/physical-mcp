/**
 * ROS2 topic MCP tools: list, info, subscribe, publish, echo.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';
import { validateTopicName, validateMessagePayload } from '../utils/input-validation.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getTopicTools(): Tool[] {
  return [
    {
      name: 'ros2_topic_list',
      description: 'List all active ROS2 topics with their message types',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_topic_info',
      description: 'Get detailed info about a specific ROS2 topic (type, publishers, subscribers)',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Full topic name (e.g. /cmd_vel)'),
      })),
    },
    {
      name: 'ros2_topic_subscribe',
      description: 'Subscribe to a topic and collect N messages',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Full topic name'),
        messageType: z.string().describe('Message type (e.g. geometry_msgs/msg/Twist)'),
        count: z.number().default(1).describe('Number of messages to collect'),
        timeoutSec: z.number().default(5).describe('Timeout in seconds'),
      })),
    },
    {
      name: 'ros2_topic_publish',
      description: 'Publish a message to a ROS2 topic. Subject to safety checks (velocity limits, geofence, rate limiting).',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Full topic name (e.g. /cmd_vel)'),
        messageType: z.string().describe('Message type (e.g. geometry_msgs/msg/Twist)'),
        message: z.record(z.unknown()).describe('Message data as JSON'),
      })),
    },
    {
      name: 'ros2_topic_echo',
      description: 'Get the latest message from a topic (one-shot)',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Full topic name'),
        messageType: z.string().describe('Message type'),
        timeoutSec: z.number().default(3).describe('Timeout in seconds'),
      })),
    },
  ];
}

export async function handleTopicTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_topic_list': {
      const response = await connection.send(CommandType.TOPIC_LIST);
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_topic_info': {
      const topicName = args.topic as string;
      const nameCheck = validateTopicName(topicName);
      if (!nameCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid topic name: ${nameCheck.error}` }], isError: true };
      }
      const response = await connection.send(CommandType.TOPIC_INFO, { topic: topicName });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_topic_subscribe': {
      const subTopic = args.topic as string;
      const subCheck = validateTopicName(subTopic);
      if (!subCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid topic name: ${subCheck.error}` }], isError: true };
      }
      const response = await connection.send(CommandType.TOPIC_SUBSCRIBE, {
        topic: subTopic,
        message_type: args.messageType,
        count: args.count || 1,
        timeout_sec: args.timeoutSec || 5,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    case 'ros2_topic_publish': {
      const topic = args.topic as string;
      const message = args.message as Record<string, unknown>;

      // INPUT VALIDATION
      const pubNameCheck = validateTopicName(topic);
      if (!pubNameCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid topic name: ${pubNameCheck.error}` }], isError: true };
      }
      const msgCheck = validateMessagePayload(message);
      if (!msgCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid message: ${msgCheck.error}` }], isError: true };
      }

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

      const response = await connection.send(CommandType.TOPIC_PUBLISH, {
        topic,
        message_type: args.messageType,
        message,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: `Published to ${topic} successfully` }] };
    }

    case 'ros2_topic_echo': {
      const echoTopic = args.topic as string;
      const echoCheck = validateTopicName(echoTopic);
      if (!echoCheck.valid) {
        return { content: [{ type: 'text', text: `Invalid topic name: ${echoCheck.error}` }], isError: true };
      }
      const response = await connection.send(CommandType.TOPIC_ECHO, {
        topic: echoTopic,
        message_type: args.messageType,
        timeout_sec: args.timeoutSec || 3,
      });
      if (response.status === 'error') {
        return { content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }], isError: true };
      }
      return { content: [{ type: 'text', text: JSON.stringify(response.data, null, 2) }] };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown topic tool: ${name}` }], isError: true };
  }
}

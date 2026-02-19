/**
 * ROS2 robot description MCP tools: fetch URDF, list joints.
 *
 * Primary strategy: echo the /robot_description topic (std_msgs/msg/String).
 * Fallback: read the robot_description parameter from /robot_state_publisher.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

/**
 * Minimal URDF joint parser.
 * Extracts joint name, type, parent, child, and optional limits from raw URDF XML.
 * No external XML library required -- uses simple regex matching which is
 * sufficient for well-formed URDF produced by standard ROS2 tooling.
 */
export interface UrdfJoint {
  name: string;
  type: string;
  parent: string;
  child: string;
  limits?: {
    lower?: number;
    upper?: number;
    effort?: number;
    velocity?: number;
  };
}

export function parseJointsFromUrdf(urdf: string): UrdfJoint[] {
  const joints: UrdfJoint[] = [];

  // Match each <joint ...> ... </joint> block (non-greedy, dotall)
  const jointRegex = /<joint\s+([^>]*)>([\s\S]*?)<\/joint>/g;
  let match: RegExpExecArray | null;

  while ((match = jointRegex.exec(urdf)) !== null) {
    const attrs = match[1];
    const body = match[2];

    const nameMatch = /name\s*=\s*"([^"]*)"/.exec(attrs);
    const typeMatch = /type\s*=\s*"([^"]*)"/.exec(attrs);
    if (!nameMatch || !typeMatch) continue;

    const parentMatch = /<parent\s+link\s*=\s*"([^"]*)"/.exec(body);
    const childMatch = /<child\s+link\s*=\s*"([^"]*)"/.exec(body);

    const joint: UrdfJoint = {
      name: nameMatch[1],
      type: typeMatch[1],
      parent: parentMatch ? parentMatch[1] : 'unknown',
      child: childMatch ? childMatch[1] : 'unknown',
    };

    // Optional <limit> element
    const limitMatch = /<limit\s+([^/>]*)\/?>/.exec(body);
    if (limitMatch) {
      const limitAttrs = limitMatch[1];
      const limits: UrdfJoint['limits'] = {};

      const lower = /lower\s*=\s*"([^"]*)"/.exec(limitAttrs);
      const upper = /upper\s*=\s*"([^"]*)"/.exec(limitAttrs);
      const effort = /effort\s*=\s*"([^"]*)"/.exec(limitAttrs);
      const velocity = /velocity\s*=\s*"([^"]*)"/.exec(limitAttrs);

      if (lower) limits.lower = parseFloat(lower[1]);
      if (upper) limits.upper = parseFloat(upper[1]);
      if (effort) limits.effort = parseFloat(effort[1]);
      if (velocity) limits.velocity = parseFloat(velocity[1]);

      if (Object.keys(limits).length > 0) {
        joint.limits = limits;
      }
    }

    joints.push(joint);
  }

  return joints;
}

export function getDescriptionTools(): Tool[] {
  return [
    {
      name: 'ros2_robot_description',
      description:
        'Fetch the robot URDF description. Reads from the /robot_description topic (std_msgs/msg/String). ' +
        'Falls back to the robot_description parameter on /robot_state_publisher if the topic is unavailable.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/robot_description').describe(
          'Topic to read the URDF from (default: /robot_description)',
        ),
        timeoutSec: z.number().default(5).describe('Timeout in seconds'),
      })),
    },
    {
      name: 'ros2_robot_joints',
      description:
        'Parse the robot URDF and return a structured list of joints with their names, types, parent/child links, and limits. ' +
        'Fetches the URDF from /robot_description then extracts joint information.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().default('/robot_description').describe(
          'Topic to read the URDF from (default: /robot_description)',
        ),
        timeoutSec: z.number().default(5).describe('Timeout in seconds'),
      })),
    },
  ];
}

/**
 * Attempt to fetch the URDF string.
 *
 * 1. Try topic echo on the given topic.
 * 2. If that fails, fall back to reading the parameter from /robot_state_publisher.
 */
async function fetchUrdf(
  connection: ConnectionManager,
  topic: string,
  timeoutSec: number,
): Promise<{ urdf?: string; error?: string }> {
  // Strategy 1: topic echo
  try {
    const echoResponse = await connection.send(CommandType.TOPIC_ECHO, {
      topic,
      message_type: 'std_msgs/msg/String',
      timeout_sec: timeoutSec,
    });

    if (echoResponse.status === 'ok' && echoResponse.data) {
      const data = echoResponse.data as Record<string, unknown>;
      // std_msgs/msg/String has a single `data` field
      const urdfString = typeof data.data === 'string' ? data.data : JSON.stringify(data);
      if (urdfString && urdfString.length > 0) {
        return { urdf: urdfString };
      }
    }
  } catch {
    // Fall through to parameter fallback
  }

  // Strategy 2: parameter fallback
  try {
    const paramResponse = await connection.send(CommandType.GET_PARAMS, {
      node_name: '/robot_state_publisher',
      param_name: 'robot_description',
    });

    if (paramResponse.status === 'ok' && paramResponse.data) {
      const urdfString = typeof paramResponse.data === 'string'
        ? paramResponse.data
        : JSON.stringify(paramResponse.data);
      if (urdfString && urdfString.length > 0) {
        return { urdf: urdfString };
      }
    }
  } catch {
    // Both strategies failed
  }

  return {
    error:
      `Could not fetch robot description from topic "${topic}" or parameter /robot_state_publisher/robot_description. ` +
      'Ensure the robot_state_publisher node is running and publishing the URDF.',
  };
}

export async function handleDescriptionTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_robot_description': {
      const topic = (args.topic as string) || '/robot_description';
      const timeoutSec = (args.timeoutSec as number) || 5;

      const result = await fetchUrdf(connection, topic, timeoutSec);
      if (result.error) {
        return { content: [{ type: 'text', text: result.error }], isError: true };
      }
      return { content: [{ type: 'text', text: result.urdf! }] };
    }

    case 'ros2_robot_joints': {
      const topic = (args.topic as string) || '/robot_description';
      const timeoutSec = (args.timeoutSec as number) || 5;

      const result = await fetchUrdf(connection, topic, timeoutSec);
      if (result.error) {
        return { content: [{ type: 'text', text: result.error }], isError: true };
      }

      const joints = parseJointsFromUrdf(result.urdf!);
      if (joints.length === 0) {
        return {
          content: [{
            type: 'text',
            text: 'No joints found in the URDF. The description may not contain <joint> elements or may use an unsupported format.',
          }],
        };
      }

      return { content: [{ type: 'text', text: JSON.stringify(joints, null, 2) }] };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown description tool: ${name}` }], isError: true };
  }
}

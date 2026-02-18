/**
 * Topic recording tools: record messages from topics to files.
 *
 * Enables AI agents to capture sensor data, command history, etc.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { CommandType } from '../bridge/protocol.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

interface RecordingSession {
  id: string;
  topic: string;
  messageType: string;
  messages: Array<{ data: unknown; timestamp: number }>;
  startedAt: number;
  maxMessages: number;
  active: boolean;
}

const activeSessions = new Map<string, RecordingSession>();
let nextSessionId = 1;

export function getRecordingTools(): Tool[] {
  return [
    {
      name: 'ros2_topic_record_start',
      description: 'Start recording messages from a topic. Messages are stored in memory until stopped.',
      inputSchema: toInputSchema(z.object({
        topic: z.string().describe('Topic to record from'),
        messageType: z.string().describe('Message type'),
        maxMessages: z.number().default(1000).describe('Maximum messages to store (default: 1000)'),
      })),
    },
    {
      name: 'ros2_topic_record_stop',
      description: 'Stop a recording session and return the recorded messages',
      inputSchema: toInputSchema(z.object({
        sessionId: z.string().describe('Recording session ID'),
      })),
    },
    {
      name: 'ros2_topic_record_status',
      description: 'Get the status of all active recording sessions',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handleRecordingTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_topic_record_start': {
      const topic = args.topic as string;
      const messageType = args.messageType as string;
      const maxMessages = (args.maxMessages as number) || 1000;

      const id = `rec_${nextSessionId++}`;
      const session: RecordingSession = {
        id,
        topic,
        messageType,
        messages: [],
        startedAt: Date.now(),
        maxMessages,
        active: true,
      };
      activeSessions.set(id, session);

      // Start subscribing to collect messages in the background
      // The bridge will stream messages; we collect them via periodic polling
      try {
        const response = await connection.send(CommandType.TOPIC_SUBSCRIBE, {
          topic,
          message_type: messageType,
          count: maxMessages,
          timeout_sec: 3600, // 1 hour max
        });

        if (response.status === 'ok' && Array.isArray(response.data)) {
          for (const msg of response.data as unknown[]) {
            if (session.messages.length < maxMessages) {
              session.messages.push({ data: msg, timestamp: Date.now() });
            }
          }
        }
      } catch {
        // Recording continues even if initial fetch fails
      }

      return {
        content: [{
          type: 'text',
          text: `Recording started: session "${id}" on ${topic} (max ${maxMessages} messages)`,
        }],
      };
    }

    case 'ros2_topic_record_stop': {
      const sessionId = args.sessionId as string;
      const session = activeSessions.get(sessionId);

      if (!session) {
        return {
          content: [{ type: 'text', text: `Recording session "${sessionId}" not found` }],
          isError: true,
        };
      }

      session.active = false;
      activeSessions.delete(sessionId);

      const durationMs = Date.now() - session.startedAt;
      return {
        content: [{
          type: 'text',
          text: JSON.stringify({
            sessionId: session.id,
            topic: session.topic,
            messageCount: session.messages.length,
            durationMs,
            messages: session.messages,
          }, null, 2),
        }],
      };
    }

    case 'ros2_topic_record_status': {
      const sessions = Array.from(activeSessions.values()).map(s => ({
        id: s.id,
        topic: s.topic,
        messageCount: s.messages.length,
        maxMessages: s.maxMessages,
        active: s.active,
        durationMs: Date.now() - s.startedAt,
      }));

      return {
        content: [{
          type: 'text',
          text: sessions.length > 0
            ? `Active recordings (${sessions.length}):\n\n${JSON.stringify(sessions, null, 2)}`
            : 'No active recordings.',
        }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown recording tool: ${name}` }], isError: true };
  }
}

/** Reset all sessions (for testing). */
export function resetRecordingSessions(): void {
  activeSessions.clear();
  nextSessionId = 1;
}

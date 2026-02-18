import { z } from 'zod';

export const CommandType = {
  TOPIC_LIST: 'topic.list',
  TOPIC_INFO: 'topic.info',
  TOPIC_SUBSCRIBE: 'topic.subscribe',
  TOPIC_PUBLISH: 'topic.publish',
  TOPIC_ECHO: 'topic.echo',
  SERVICE_LIST: 'service.list',
  SERVICE_INFO: 'service.info',
  SERVICE_CALL: 'service.call',
  ACTION_LIST: 'action.list',
  ACTION_SEND_GOAL: 'action.send_goal',
  ACTION_CANCEL: 'action.cancel',
  ACTION_STATUS: 'action.status',
  NODE_LIST: 'node.list',
  GET_PARAMS: 'params.get',
  SET_PARAMS: 'params.set',
  LIST_PARAMS: 'params.list',
  NODE_INFO: 'node.info',
  PING: 'ping',
  EMERGENCY_STOP: 'emergency_stop',
} as const;

export type CommandTypeValue = (typeof CommandType)[keyof typeof CommandType];

export const BridgeCommandSchema = z.object({
  id: z.string(),
  type: z.string(),
  params: z.record(z.unknown()).optional().default({}),
});

export type BridgeCommand = z.infer<typeof BridgeCommandSchema>;

export const BridgeResponseSchema = z.object({
  id: z.string().nullable(),
  status: z.enum(['ok', 'error']),
  data: z.unknown(),
  timestamp: z.number(),
});

export type BridgeResponse = z.infer<typeof BridgeResponseSchema>;

export function createCommand(type: CommandTypeValue, params: Record<string, unknown> = {}): Omit<BridgeCommand, 'id'> {
  return { type, params };
}

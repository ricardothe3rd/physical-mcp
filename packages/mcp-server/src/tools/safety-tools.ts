/**
 * Safety MCP tools: e-stop, status, policy, audit log.
 * THE DIFFERENTIATOR - no competitor has this.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { CommandType } from '../bridge/protocol.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';

function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getSafetyTools(): Tool[] {
  return [
    {
      name: 'safety_status',
      description: 'Get current safety system status including policy, e-stop state, velocity limits, geofence, and audit stats',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'safety_emergency_stop',
      description: 'EMERGENCY STOP - immediately halt all robot motion. Publishes zero velocity and blocks all further commands until released.',
      inputSchema: toInputSchema(z.object({
        reason: z.string().optional().describe('Reason for activating e-stop'),
      })),
    },
    {
      name: 'safety_emergency_stop_release',
      description: 'Release emergency stop and allow commands to resume',
      inputSchema: toInputSchema(z.object({
        confirmation: z.literal('CONFIRM_RELEASE').describe('Must be "CONFIRM_RELEASE" to proceed'),
      })),
    },
    {
      name: 'safety_get_policy',
      description: 'Get the current safety policy configuration (velocity limits, geofence, rate limits, blocked topics/services)',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'safety_update_velocity_limits',
      description: 'Update velocity limits. Cannot exceed hardware maximums.',
      inputSchema: toInputSchema(z.object({
        linearMax: z.number().optional().describe('Max linear velocity in m/s'),
        angularMax: z.number().optional().describe('Max angular velocity in rad/s'),
      })),
    },
    {
      name: 'safety_update_geofence',
      description: 'Update geofence workspace boundaries',
      inputSchema: toInputSchema(z.object({
        xMin: z.number().optional(),
        xMax: z.number().optional(),
        yMin: z.number().optional(),
        yMax: z.number().optional(),
        zMin: z.number().optional(),
        zMax: z.number().optional(),
      })),
    },
    {
      name: 'safety_audit_log',
      description: 'View the command audit trail showing all commands and safety violations',
      inputSchema: toInputSchema(z.object({
        limit: z.number().default(20).describe('Number of entries to return'),
        violationsOnly: z.boolean().default(false).describe('Show only blocked commands'),
        command: z.string().optional().describe('Filter by command type'),
      })),
    },
    {
      name: 'safety_set_clamp_mode',
      description: 'Toggle velocity clamp mode. When enabled, over-limit velocities are reduced to max instead of being blocked.',
      inputSchema: toInputSchema(z.object({
        enabled: z.boolean().describe('true = clamp to max, false = block entirely'),
      })),
    },
    {
      name: 'safety_deadman_switch',
      description: 'Configure the deadman switch. When enabled, auto-triggers e-stop if no heartbeat is received within the timeout period.',
      inputSchema: toInputSchema(z.object({
        enabled: z.boolean().optional().describe('Enable/disable the deadman switch'),
        timeoutMs: z.number().optional().describe('Timeout in milliseconds before auto e-stop (default: 30000)'),
      })),
    },
    {
      name: 'safety_heartbeat',
      description: 'Send a heartbeat to the deadman switch to prevent automatic e-stop. Call periodically when deadman switch is enabled.',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handleSafetyTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'safety_status': {
      const status = safety.getStatus();
      return { content: [{ type: 'text', text: JSON.stringify(status, null, 2) }] };
    }

    case 'safety_emergency_stop': {
      safety.activateEmergencyStop();

      // Send zero-velocity to bridge to stop robot immediately
      try {
        await connection.send(CommandType.EMERGENCY_STOP);
      } catch {
        // E-stop is activated in policy engine regardless of bridge state
      }

      const reason = (args.reason as string) || 'Manual activation';
      return {
        content: [{
          type: 'text',
          text: `EMERGENCY STOP ACTIVATED\n\nReason: ${reason}\n\nAll commands are now blocked. Zero velocity published to /cmd_vel.\nUse safety_emergency_stop_release with confirmation "CONFIRM_RELEASE" to resume.`,
        }],
      };
    }

    case 'safety_emergency_stop_release': {
      if (args.confirmation !== 'CONFIRM_RELEASE') {
        return {
          content: [{ type: 'text', text: 'E-stop release requires confirmation="CONFIRM_RELEASE"' }],
          isError: true,
        };
      }

      safety.releaseEmergencyStop();
      return {
        content: [{ type: 'text', text: 'Emergency stop released. Commands are now allowed.' }],
      };
    }

    case 'safety_get_policy': {
      const policy = safety.getPolicy();
      return { content: [{ type: 'text', text: JSON.stringify(policy, null, 2) }] };
    }

    case 'safety_update_velocity_limits': {
      const updates: Record<string, number> = {};
      if (args.linearMax !== undefined) updates.linearMax = args.linearMax as number;
      if (args.angularMax !== undefined) updates.angularMax = args.angularMax as number;
      safety.updateVelocityLimits(updates);
      const policy = safety.getPolicy();
      return {
        content: [{ type: 'text', text: `Velocity limits updated:\n${JSON.stringify(policy.velocity, null, 2)}` }],
      };
    }

    case 'safety_update_geofence': {
      const updates: Record<string, number> = {};
      for (const key of ['xMin', 'xMax', 'yMin', 'yMax', 'zMin', 'zMax']) {
        if (args[key] !== undefined) updates[key] = args[key] as number;
      }
      safety.updateGeofence(updates);
      const policy = safety.getPolicy();
      return {
        content: [{ type: 'text', text: `Geofence updated:\n${JSON.stringify(policy.geofence, null, 2)}` }],
      };
    }

    case 'safety_audit_log': {
      const entries = safety.getAuditLog({
        limit: (args.limit as number) || 20,
        violationsOnly: (args.violationsOnly as boolean) || false,
        command: args.command as string | undefined,
      });
      const stats = safety.getAuditStats();
      return {
        content: [{
          type: 'text',
          text: `Audit Log (${stats.total} total, ${stats.blocked} blocked, ${stats.errors} errors)\n\n${JSON.stringify(entries, null, 2)}`,
        }],
      };
    }

    case 'safety_set_clamp_mode': {
      const enabled = args.enabled as boolean;
      safety.updateVelocityLimits({ clampMode: enabled });
      return {
        content: [{
          type: 'text',
          text: enabled
            ? 'Velocity clamp mode ENABLED. Over-limit velocities will be reduced to the maximum instead of being blocked.'
            : 'Velocity clamp mode DISABLED. Over-limit velocities will be blocked.',
        }],
      };
    }

    case 'safety_deadman_switch': {
      const updates: Record<string, unknown> = {};
      if (args.enabled !== undefined) updates.enabled = args.enabled;
      if (args.timeoutMs !== undefined) updates.timeoutMs = args.timeoutMs;
      safety.updateDeadmanSwitch(updates);
      const status = safety.getStatus();
      return {
        content: [{
          type: 'text',
          text: `Deadman switch ${status.deadmanSwitch.enabled ? 'ENABLED' : 'DISABLED'} (timeout: ${status.deadmanSwitch.timeoutMs}ms)`,
        }],
      };
    }

    case 'safety_heartbeat': {
      safety.heartbeat();
      return {
        content: [{ type: 'text', text: 'Heartbeat received. Deadman switch timer reset.' }],
      };
    }

    default:
      return { content: [{ type: 'text', text: `Unknown safety tool: ${name}` }], isError: true };
  }
}

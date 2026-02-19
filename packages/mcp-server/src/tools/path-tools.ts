/**
 * Path planning integration tools for robot navigation.
 *
 * Enables AI agents to request path plans from nav2, inspect planned paths,
 * force costmap updates, and monitor navigation status.
 */

import { z } from 'zod';
import { zodToJsonSchema } from 'zod-to-json-schema';
import type { Tool } from '@modelcontextprotocol/sdk/types.js';
import { ConnectionManager } from '../bridge/connection-manager.js';
import { PolicyEngine } from '../safety/policy-engine.js';
import { CommandType } from '../bridge/protocol.js';

// Break zodToJsonSchema type inference chain to prevent tsc hang
function toInputSchema(schema: z.ZodType): Tool['inputSchema'] {
  return zodToJsonSchema(schema) as unknown as Tool['inputSchema'];
}

export function getPathTools(): Tool[] {
  return [
    {
      name: 'ros2_plan_path',
      description:
        'Request a path plan from the nav2 navigation stack using the ComputePathToPose action. ' +
        'Returns the planned path as a list of waypoints. Safety-checked against geofence for both start and goal positions.',
      inputSchema: toInputSchema(z.object({
        start_x: z.number().describe('Start X position in meters'),
        start_y: z.number().describe('Start Y position in meters'),
        start_theta: z.number().optional().describe('Start orientation in radians (optional, uses current pose if omitted)'),
        goal_x: z.number().describe('Goal X position in meters'),
        goal_y: z.number().describe('Goal Y position in meters'),
        goal_theta: z.number().default(0).describe('Goal orientation in radians (default: 0)'),
        planner_id: z.string().default('GridBased').describe('Planner plugin ID (default: "GridBased")'),
      })),
    },
    {
      name: 'ros2_path_info',
      description:
        'Get information about the current planned path by subscribing to the /plan topic (nav_msgs/Path). ' +
        'Returns the number of poses, total path distance, and bounding box of the path.',
      inputSchema: toInputSchema(z.object({})),
    },
    {
      name: 'ros2_costmap_update',
      description:
        'Force a costmap update by calling the costmap clear service. ' +
        'Useful after obstacle changes or map updates.',
      inputSchema: toInputSchema(z.object({
        costmap_name: z.string().default('local_costmap').describe('Costmap name to clear (default: "local_costmap")'),
      })),
    },
    {
      name: 'ros2_navigation_status',
      description:
        'Get the current navigation status by subscribing to the navigate_to_pose action status topic. ' +
        'Returns active goals and their current status.',
      inputSchema: toInputSchema(z.object({})),
    },
  ];
}

export async function handlePathTool(
  name: string,
  args: Record<string, unknown>,
  connection: ConnectionManager,
  safety: PolicyEngine
): Promise<{ content: { type: string; text: string }[]; isError?: boolean }> {
  switch (name) {
    case 'ros2_plan_path': {
      const startX = args.start_x as number;
      const startY = args.start_y as number;
      const startTheta = args.start_theta as number | undefined;
      const goalX = args.goal_x as number;
      const goalY = args.goal_y as number;
      const goalTheta = (args.goal_theta as number) ?? 0;
      const plannerId = (args.planner_id as string) || 'GridBased';

      // Safety check: verify start position is within geofence
      const startCheck = safety.checkPosition({ x: startX, y: startY, z: 0 });
      if (!startCheck.inside) {
        return {
          content: [{
            type: 'text',
            text: `Path planning blocked: start position (${startX}, ${startY}) is outside geofence boundary`,
          }],
          isError: true,
        };
      }

      // Safety check: verify goal position is within geofence
      const goalCheck = safety.checkPosition({ x: goalX, y: goalY, z: 0 });
      if (!goalCheck.inside) {
        return {
          content: [{
            type: 'text',
            text: `Path planning blocked: goal position (${goalX}, ${goalY}) is outside geofence boundary`,
          }],
          isError: true,
        };
      }

      // Build the ComputePathToPose goal
      const goal: Record<string, unknown> = {
        start: {
          header: { frame_id: 'map' },
          pose: {
            position: { x: startX, y: startY, z: 0 },
            orientation: startTheta !== undefined
              ? { x: 0, y: 0, z: Math.sin(startTheta / 2), w: Math.cos(startTheta / 2) }
              : { x: 0, y: 0, z: 0, w: 1 },
          },
        },
        goal: {
          header: { frame_id: 'map' },
          pose: {
            position: { x: goalX, y: goalY, z: 0 },
            orientation: { x: 0, y: 0, z: Math.sin(goalTheta / 2), w: Math.cos(goalTheta / 2) },
          },
        },
        planner_id: plannerId,
      };

      // If start_theta not provided, omit start pose (nav2 will use current pose)
      if (startTheta === undefined) {
        goal.use_start = false;
      }

      try {
        const response = await connection.send(CommandType.ACTION_SEND_GOAL, {
          action: '/compute_path_to_pose',
          action_type: 'nav2_msgs/action/ComputePathToPose',
          goal,
        });

        if (response.status === 'error') {
          return {
            content: [{ type: 'text', text: `Path planning error: ${JSON.stringify(response.data)}` }],
            isError: true,
          };
        }

        return {
          content: [{
            type: 'text',
            text: `Path planned from (${startX}, ${startY}) to (${goalX}, ${goalY}) using planner "${plannerId}":\n${JSON.stringify(response.data, null, 2)}`,
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Path planning failed: ${message}` }],
          isError: true,
        };
      }
    }

    case 'ros2_path_info': {
      try {
        const response = await connection.send(CommandType.TOPIC_SUBSCRIBE, {
          topic: '/plan',
          count: 1,
        });

        if (response.status === 'error') {
          return {
            content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }],
            isError: true,
          };
        }

        const data = response.data as Record<string, unknown> | undefined;
        const poses = (data?.poses as Array<Record<string, unknown>>) || [];

        // Calculate total distance
        let totalDistance = 0;
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        for (let i = 0; i < poses.length; i++) {
          const pose = poses[i].pose as Record<string, unknown> | undefined;
          const position = pose?.position as Record<string, number> | undefined;
          if (position) {
            const px = position.x ?? 0;
            const py = position.y ?? 0;
            if (px < minX) minX = px;
            if (px > maxX) maxX = px;
            if (py < minY) minY = py;
            if (py > maxY) maxY = py;

            if (i > 0) {
              const prevPose = poses[i - 1].pose as Record<string, unknown> | undefined;
              const prevPos = prevPose?.position as Record<string, number> | undefined;
              if (prevPos) {
                const dx = px - (prevPos.x ?? 0);
                const dy = py - (prevPos.y ?? 0);
                totalDistance += Math.sqrt(dx * dx + dy * dy);
              }
            }
          }
        }

        const info = {
          num_poses: poses.length,
          total_distance_m: Math.round(totalDistance * 1000) / 1000,
          bounding_box: poses.length > 0
            ? { min_x: minX, max_x: maxX, min_y: minY, max_y: maxY }
            : null,
        };

        return {
          content: [{
            type: 'text',
            text: `Path info from /plan:\n${JSON.stringify(info, null, 2)}`,
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Failed to get path info: ${message}` }],
          isError: true,
        };
      }
    }

    case 'ros2_costmap_update': {
      const costmapName = (args.costmap_name as string) || 'local_costmap';
      const service = `/${costmapName}/clear_entirely_${costmapName}`;

      // Safety check: costmap clearing is a service call
      const safetyCheck = safety.checkServiceCall(service, {});
      if (!safetyCheck.allowed) {
        return {
          content: [{
            type: 'text',
            text: `Costmap update blocked: ${safetyCheck.violations.map(v => v.message).join('; ')}`,
          }],
          isError: true,
        };
      }

      try {
        const response = await connection.send(CommandType.SERVICE_CALL, {
          service,
          service_type: 'nav2_msgs/srv/ClearEntireCostmap',
          args: {},
        });

        if (response.status === 'error') {
          return {
            content: [{ type: 'text', text: `Costmap update error: ${JSON.stringify(response.data)}` }],
            isError: true,
          };
        }

        return {
          content: [{
            type: 'text',
            text: `Costmap "${costmapName}" cleared successfully.\n${JSON.stringify(response.data, null, 2)}`,
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Costmap update failed: ${message}` }],
          isError: true,
        };
      }
    }

    case 'ros2_navigation_status': {
      try {
        const response = await connection.send(CommandType.TOPIC_SUBSCRIBE, {
          topic: '/navigate_to_pose/_action/status',
          count: 1,
        });

        if (response.status === 'error') {
          return {
            content: [{ type: 'text', text: `Error: ${JSON.stringify(response.data)}` }],
            isError: true,
          };
        }

        const data = response.data as Record<string, unknown> | undefined;
        const statusList = (data?.status_list as Array<Record<string, unknown>>) || [];

        const goals = statusList.map(entry => ({
          goal_id: entry.goal_info
            ? (entry.goal_info as Record<string, unknown>).goal_id
            : 'unknown',
          status: entry.status,
        }));

        const summary = {
          active_goals: goals.length,
          goals,
        };

        return {
          content: [{
            type: 'text',
            text: `Navigation status:\n${JSON.stringify(summary, null, 2)}`,
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: 'text', text: `Failed to get navigation status: ${message}` }],
          isError: true,
        };
      }
    }

    default:
      return { content: [{ type: 'text', text: `Unknown path tool: ${name}` }], isError: true };
  }
}

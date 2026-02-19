/**
 * Tool dispatch routing tests.
 *
 * Verifies that the tool name -> category mapping logic used in index.ts
 * is correct: every tool name maps to exactly one category, there are no
 * overlaps between categories, unknown names are unmatched, and the total
 * tool count is as expected.
 */

import { describe, it, expect } from 'vitest';
import { getTopicTools } from './topic-tools.js';
import { getServiceTools } from './service-tools.js';
import { getActionTools } from './action-tools.js';
import { getSafetyTools } from './safety-tools.js';
import { getSystemTools } from './system-tools.js';
import { getBatchTools } from './batch-tools.js';
import { getRecordingTools } from './recording-tools.js';
import { getConditionalTools } from './conditional-tools.js';
import { getScheduledTools } from './scheduled-tools.js';
import { getTfTools } from './tf-tools.js';
import { getDiagnosticTools } from './diagnostic-tools.js';
import { getFleetTools } from './fleet-tools.js';
import { getLaunchTools } from './launch-tools.js';
import { getWaypointTools } from './waypoint-tools.js';

// Mirrors the dispatch sets built in index.ts
const topicToolNames = new Set(getTopicTools().map(t => t.name));
const serviceToolNames = new Set(getServiceTools().map(t => t.name));
const actionToolNames = new Set(getActionTools().map(t => t.name));
const safetyToolNames = new Set(getSafetyTools().map(t => t.name));
const systemToolNames = new Set(getSystemTools().map(t => t.name));
const batchToolNames = new Set(getBatchTools().map(t => t.name));
const recordingToolNames = new Set(getRecordingTools().map(t => t.name));
const conditionalToolNames = new Set(getConditionalTools().map(t => t.name));
const scheduledToolNames = new Set(getScheduledTools().map(t => t.name));
const tfToolNames = new Set(getTfTools().map(t => t.name));
const diagnosticToolNames = new Set(getDiagnosticTools().map(t => t.name));
const fleetToolNames = new Set(getFleetTools().map(t => t.name));
const launchToolNames = new Set(getLaunchTools().map(t => t.name));
const waypointToolNames = new Set(getWaypointTools().map(t => t.name));

// All category sets in dispatch order (same order as index.ts)
const categorySets = [
  { name: 'topic', set: topicToolNames },
  { name: 'service', set: serviceToolNames },
  { name: 'action', set: actionToolNames },
  { name: 'safety', set: safetyToolNames },
  { name: 'system', set: systemToolNames },
  { name: 'batch', set: batchToolNames },
  { name: 'recording', set: recordingToolNames },
  { name: 'conditional', set: conditionalToolNames },
  { name: 'scheduled', set: scheduledToolNames },
  { name: 'tf', set: tfToolNames },
  { name: 'diagnostic', set: diagnosticToolNames },
  { name: 'fleet', set: fleetToolNames },
  { name: 'launch', set: launchToolNames },
  { name: 'waypoint', set: waypointToolNames },
];

/**
 * Simulate the dispatch logic from index.ts.
 * Returns the category name that would handle the tool, or null for unknown.
 */
function dispatchCategory(toolName: string): string | null {
  if (topicToolNames.has(toolName)) return 'topic';
  if (serviceToolNames.has(toolName)) return 'service';
  if (actionToolNames.has(toolName)) return 'action';
  if (safetyToolNames.has(toolName)) return 'safety';
  if (systemToolNames.has(toolName)) return 'system';
  if (batchToolNames.has(toolName)) return 'batch';
  if (recordingToolNames.has(toolName)) return 'recording';
  if (conditionalToolNames.has(toolName)) return 'conditional';
  if (scheduledToolNames.has(toolName)) return 'scheduled';
  if (tfToolNames.has(toolName)) return 'tf';
  if (diagnosticToolNames.has(toolName)) return 'diagnostic';
  if (fleetToolNames.has(toolName)) return 'fleet';
  if (launchToolNames.has(toolName)) return 'launch';
  if (waypointToolNames.has(toolName)) return 'waypoint';
  return null;
}

describe('Tool Dispatch Routing', () => {
  describe('topic tools map to topic handler', () => {
    const expected = [
      'ros2_topic_list',
      'ros2_topic_info',
      'ros2_topic_subscribe',
      'ros2_topic_publish',
      'ros2_topic_echo',
    ];

    it.each(expected)('%s dispatches to topic', (name) => {
      expect(dispatchCategory(name)).toBe('topic');
    });

    it('topic set contains exactly 5 tools', () => {
      expect(topicToolNames.size).toBe(5);
    });
  });

  describe('service tools map to service handler', () => {
    const expected = [
      'ros2_service_list',
      'ros2_service_info',
      'ros2_service_call',
    ];

    it.each(expected)('%s dispatches to service', (name) => {
      expect(dispatchCategory(name)).toBe('service');
    });

    it('service set contains exactly 3 tools', () => {
      expect(serviceToolNames.size).toBe(3);
    });
  });

  describe('action tools map to action handler', () => {
    const expected = [
      'ros2_action_list',
      'ros2_action_send_goal',
      'ros2_action_cancel',
      'ros2_action_status',
    ];

    it.each(expected)('%s dispatches to action', (name) => {
      expect(dispatchCategory(name)).toBe('action');
    });

    it('action set contains exactly 4 tools', () => {
      expect(actionToolNames.size).toBe(4);
    });
  });

  describe('safety tools map to safety handler', () => {
    const expected = [
      'safety_status',
      'safety_emergency_stop',
      'safety_emergency_stop_release',
      'safety_get_policy',
      'safety_update_velocity_limits',
      'safety_update_geofence',
      'safety_audit_log',
      'safety_set_clamp_mode',
      'safety_deadman_switch',
      'safety_heartbeat',
      'safety_update_acceleration_limits',
      'safety_export_audit_log',
      'safety_check_position',
      'safety_validate_policy',
      'safety_approval_list',
      'safety_approval_approve',
      'safety_approval_deny',
      'safety_approval_config',
      'safety_violation_mode',
      'safety_time_policy',
      'safety_time_policy_status',
    ];

    it.each(expected)('%s dispatches to safety', (name) => {
      expect(dispatchCategory(name)).toBe('safety');
    });

    it('safety set contains exactly 21 tools', () => {
      expect(safetyToolNames.size).toBe(21);
    });
  });

  describe('system tools map to system handler', () => {
    const expected = [
      'system_bridge_status',
      'system_node_list',
      'system_node_info',
      'ros2_param_list',
      'ros2_param_get',
      'ros2_param_set',
    ];

    it.each(expected)('%s dispatches to system', (name) => {
      expect(dispatchCategory(name)).toBe('system');
    });

    it('system set contains exactly 6 tools', () => {
      expect(systemToolNames.size).toBe(6);
    });
  });

  describe('batch tools map to batch handler', () => {
    const expected = ['ros2_batch_execute'];

    it.each(expected)('%s dispatches to batch', (name) => {
      expect(dispatchCategory(name)).toBe('batch');
    });

    it('batch set contains exactly 1 tool', () => {
      expect(batchToolNames.size).toBe(1);
    });
  });

  describe('recording tools map to recording handler', () => {
    const expected = [
      'ros2_topic_record_start',
      'ros2_topic_record_stop',
      'ros2_topic_record_status',
    ];

    it.each(expected)('%s dispatches to recording', (name) => {
      expect(dispatchCategory(name)).toBe('recording');
    });

    it('recording set contains exactly 3 tools', () => {
      expect(recordingToolNames.size).toBe(3);
    });
  });

  describe('conditional tools map to conditional handler', () => {
    const expected = [
      'ros2_conditional_execute',
      'ros2_wait_for_condition',
    ];

    it.each(expected)('%s dispatches to conditional', (name) => {
      expect(dispatchCategory(name)).toBe('conditional');
    });

    it('conditional set contains exactly 2 tools', () => {
      expect(conditionalToolNames.size).toBe(2);
    });
  });

  describe('scheduled tools map to scheduled handler', () => {
    const expected = [
      'ros2_schedule_command',
      'ros2_schedule_cancel',
      'ros2_schedule_list',
    ];

    it.each(expected)('%s dispatches to scheduled', (name) => {
      expect(dispatchCategory(name)).toBe('scheduled');
    });

    it('scheduled set contains exactly 3 tools', () => {
      expect(scheduledToolNames.size).toBe(3);
    });
  });

  describe('unknown tool names', () => {
    it('returns null for completely unknown tool', () => {
      expect(dispatchCategory('nonexistent_tool')).toBeNull();
    });

    it('returns null for empty string', () => {
      expect(dispatchCategory('')).toBeNull();
    });

    it('returns null for similar but wrong name', () => {
      expect(dispatchCategory('ros2_topic_delete')).toBeNull();
    });

    it('returns null for name with wrong prefix', () => {
      expect(dispatchCategory('ros3_topic_list')).toBeNull();
    });

    it('returns null for partial match', () => {
      expect(dispatchCategory('ros2_topic')).toBeNull();
    });

    it('returns null for name with extra suffix', () => {
      expect(dispatchCategory('safety_status_extended')).toBeNull();
    });
  });

  describe('disjoint tool name sets (no overlaps)', () => {
    it('no tool name appears in more than one category', () => {
      const overlaps: string[] = [];

      for (let i = 0; i < categorySets.length; i++) {
        for (let j = i + 1; j < categorySets.length; j++) {
          const setA = categorySets[i];
          const setB = categorySets[j];
          for (const name of setA.set) {
            if (setB.set.has(name)) {
              overlaps.push(`"${name}" found in both ${setA.name} and ${setB.name}`);
            }
          }
        }
      }

      expect(overlaps).toEqual([]);
    });

    it('each category set has no duplicate entries internally', () => {
      for (const { name, set } of categorySets) {
        // Sets cannot have duplicates by definition, but verify the source
        // arrays did not have duplicates that collapsed into a smaller set
        const getter = {
          topic: getTopicTools,
          service: getServiceTools,
          action: getActionTools,
          safety: getSafetyTools,
          system: getSystemTools,
          batch: getBatchTools,
          recording: getRecordingTools,
          conditional: getConditionalTools,
          scheduled: getScheduledTools,
          tf: getTfTools,
          diagnostic: getDiagnosticTools,
          fleet: getFleetTools,
          launch: getLaunchTools,
          waypoint: getWaypointTools,
        }[name]!;

        const tools = getter();
        expect(tools.length).toBe(set.size);
      }
    });
  });

  describe('total tool count', () => {
    it('all categories sum to 62 tools', () => {
      let total = 0;
      for (const { set } of categorySets) {
        total += set.size;
      }
      expect(total).toBe(62);
    });

    it('matches the count of all tools combined', () => {
      const allTools = [
        ...getTopicTools(),
        ...getServiceTools(),
        ...getActionTools(),
        ...getSafetyTools(),
        ...getSystemTools(),
        ...getBatchTools(),
        ...getRecordingTools(),
        ...getConditionalTools(),
        ...getScheduledTools(),
        ...getTfTools(),
        ...getDiagnosticTools(),
        ...getFleetTools(),
        ...getLaunchTools(),
        ...getWaypointTools(),
      ];
      expect(allTools.length).toBe(62);
    });

    it('14 categories exist', () => {
      expect(categorySets.length).toBe(14);
    });
  });

  describe('every registered tool dispatches to some category', () => {
    const allTools = [
      ...getTopicTools(),
      ...getServiceTools(),
      ...getActionTools(),
      ...getSafetyTools(),
      ...getSystemTools(),
      ...getBatchTools(),
      ...getRecordingTools(),
      ...getConditionalTools(),
      ...getScheduledTools(),
      ...getTfTools(),
      ...getDiagnosticTools(),
      ...getFleetTools(),
      ...getLaunchTools(),
      ...getWaypointTools(),
    ];

    it.each(allTools.map(t => t.name))('%s dispatches to a category', (name) => {
      expect(dispatchCategory(name)).not.toBeNull();
    });
  });
});

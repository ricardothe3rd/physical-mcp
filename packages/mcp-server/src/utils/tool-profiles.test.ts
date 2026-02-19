import { describe, it, expect, beforeEach } from 'vitest';
import { ToolProfileManager, ToolProfile } from './tool-profiles.js';

// ---------------------------------------------------------------------------
// Helpers – representative tool set
// ---------------------------------------------------------------------------

const ALL_TOOLS = [
  // topic
  { name: 'ros2_topic_list' },
  { name: 'ros2_topic_subscribe' },
  { name: 'ros2_topic_publish' },
  { name: 'ros2_topic_echo' },
  { name: 'ros2_topic_info' },
  // safety
  { name: 'safety_status' },
  { name: 'safety_emergency_stop' },
  { name: 'safety_emergency_stop_release' },
  { name: 'safety_get_policy' },
  { name: 'safety_update_velocity_limits' },
  { name: 'safety_update_geofence' },
  { name: 'safety_audit_log' },
  // system
  { name: 'system_bridge_status' },
  { name: 'system_node_list' },
  { name: 'system_node_info' },
  { name: 'system_health_status' },
  // service
  { name: 'ros2_service_list' },
  { name: 'ros2_service_call' },
  { name: 'ros2_service_info' },
  // action
  { name: 'ros2_action_list' },
  { name: 'ros2_action_send_goal' },
  { name: 'ros2_action_cancel' },
  { name: 'ros2_action_status' },
  // param
  { name: 'ros2_param_get' },
  { name: 'ros2_param_set' },
  { name: 'ros2_param_list' },
  // diagnostic
  { name: 'ros2_diagnostic_status' },
  { name: 'ros2_diagnostic_summary' },
  // sensor
  { name: 'ros2_sensor_summary' },
  // power
  { name: 'ros2_battery_status' },
  // tf (full only)
  { name: 'ros2_tf_lookup' },
  // fleet (full only)
  { name: 'ros2_fleet_status' },
  // launch (full only)
  { name: 'ros2_launch_list' },
  // waypoint (full only)
  { name: 'ros2_waypoint_save' },
  // camera (full only)
  { name: 'ros2_camera_info' },
  { name: 'ros2_image_preview' },
  // map (full only)
  { name: 'ros2_map_info' },
  // history (full only)
  { name: 'ros2_command_history' },
  // path (full only)
  { name: 'ros2_plan_path' },
  { name: 'ros2_navigation_status' },
  { name: 'ros2_costmap_update' },
  { name: 'ros2_path_info' },
  // network (full only)
  { name: 'ros2_network_stats' },
  // introspection (full only)
  { name: 'ros2_introspection_topic' },
  // namespace (full only)
  { name: 'ros2_namespace_list' },
  // description (full only)
  { name: 'ros2_description_fetch' },
  // batch (full only)
  { name: 'ros2_batch_publish' },
  // conditional (full only)
  { name: 'ros2_conditional_publish' },
  // scheduled (full only)
  { name: 'ros2_scheduled_publish' },
  // recording (full only)
  { name: 'ros2_recording_start' },
];

const MINIMAL_TOOL_NAMES = ALL_TOOLS.filter((t) => {
  const cat = ToolProfileManager.getCategoryForTool(t.name);
  return cat !== null && ['topic', 'safety', 'system'].includes(cat);
}).map((t) => t.name);

const STANDARD_ONLY_CATEGORIES = [
  'service',
  'action',
  'param',
  'diagnostic',
  'sensor',
  'power',
];

const STANDARD_TOOL_NAMES = ALL_TOOLS.filter((t) => {
  const cat = ToolProfileManager.getCategoryForTool(t.name);
  return (
    cat !== null &&
    [...['topic', 'safety', 'system'], ...STANDARD_ONLY_CATEGORIES].includes(cat)
  );
}).map((t) => t.name);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('ToolProfileManager', () => {
  // --- Constructor defaults ---

  describe('constructor', () => {
    it('defaults to full profile when no config supplied', () => {
      const mgr = new ToolProfileManager();
      expect(mgr.getProfile()).toBe('full');
    });

    it('accepts an explicit config', () => {
      const mgr = new ToolProfileManager({ profile: 'minimal' });
      expect(mgr.getProfile()).toBe('minimal');
    });
  });

  // --- getProfile / getConfig ---

  describe('getProfile / getConfig', () => {
    it('getProfile returns current profile', () => {
      const mgr = new ToolProfileManager({ profile: 'standard' });
      expect(mgr.getProfile()).toBe('standard');
    });

    it('getConfig returns a copy of the config', () => {
      const cfg = { profile: 'minimal' as ToolProfile, customExclude: ['safety_status'] };
      const mgr = new ToolProfileManager(cfg);
      const returned = mgr.getConfig();
      expect(returned).toEqual(cfg);
      // Must be a different reference
      expect(returned).not.toBe(cfg);
    });
  });

  // --- minimal profile ---

  describe('minimal profile', () => {
    let mgr: ToolProfileManager;

    beforeEach(() => {
      mgr = new ToolProfileManager({ profile: 'minimal' });
    });

    it('includes topic, safety, system tools', () => {
      for (const name of MINIMAL_TOOL_NAMES) {
        expect(mgr.isToolIncluded(name)).toBe(true);
      }
    });

    it('excludes service, action, param, and other tools', () => {
      expect(mgr.isToolIncluded('ros2_service_list')).toBe(false);
      expect(mgr.isToolIncluded('ros2_action_list')).toBe(false);
      expect(mgr.isToolIncluded('ros2_param_get')).toBe(false);
      expect(mgr.isToolIncluded('ros2_tf_lookup')).toBe(false);
      expect(mgr.isToolIncluded('ros2_fleet_status')).toBe(false);
    });

    it('filterTools returns only minimal tools', () => {
      const filtered = mgr.filterTools(ALL_TOOLS);
      const filteredNames = filtered.map((t) => t.name);
      expect(filteredNames.sort()).toEqual([...MINIMAL_TOOL_NAMES].sort());
    });
  });

  // --- standard profile ---

  describe('standard profile', () => {
    let mgr: ToolProfileManager;

    beforeEach(() => {
      mgr = new ToolProfileManager({ profile: 'standard' });
    });

    it('includes minimal + service, action, param, diagnostic, sensor, power', () => {
      for (const name of STANDARD_TOOL_NAMES) {
        expect(mgr.isToolIncluded(name)).toBe(true);
      }
    });

    it('excludes full-only tools like tf, fleet, launch', () => {
      expect(mgr.isToolIncluded('ros2_tf_lookup')).toBe(false);
      expect(mgr.isToolIncluded('ros2_fleet_status')).toBe(false);
      expect(mgr.isToolIncluded('ros2_launch_list')).toBe(false);
      expect(mgr.isToolIncluded('ros2_waypoint_save')).toBe(false);
    });

    it('filterTools returns only standard tools', () => {
      const filtered = mgr.filterTools(ALL_TOOLS);
      const filteredNames = filtered.map((t) => t.name);
      expect(filteredNames.sort()).toEqual([...STANDARD_TOOL_NAMES].sort());
    });
  });

  // --- full profile ---

  describe('full profile', () => {
    let mgr: ToolProfileManager;

    beforeEach(() => {
      mgr = new ToolProfileManager({ profile: 'full' });
    });

    it('includes all tools', () => {
      for (const tool of ALL_TOOLS) {
        expect(mgr.isToolIncluded(tool.name)).toBe(true);
      }
    });

    it('filterTools returns every tool', () => {
      const filtered = mgr.filterTools(ALL_TOOLS);
      expect(filtered.length).toBe(ALL_TOOLS.length);
    });
  });

  // --- custom profile ---

  describe('custom profile', () => {
    it('includes only tools in customInclude', () => {
      const mgr = new ToolProfileManager({
        profile: 'custom',
        customInclude: ['ros2_topic_list', 'safety_status'],
      });
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(true);
      expect(mgr.isToolIncluded('safety_status')).toBe(true);
      expect(mgr.isToolIncluded('ros2_service_list')).toBe(false);
    });

    it('empty customInclude includes nothing', () => {
      const mgr = new ToolProfileManager({
        profile: 'custom',
        customInclude: [],
      });
      for (const tool of ALL_TOOLS) {
        expect(mgr.isToolIncluded(tool.name)).toBe(false);
      }
    });

    it('missing customInclude on custom profile includes nothing', () => {
      const mgr = new ToolProfileManager({ profile: 'custom' });
      for (const tool of ALL_TOOLS) {
        expect(mgr.isToolIncluded(tool.name)).toBe(false);
      }
    });
  });

  // --- customExclude ---

  describe('customExclude', () => {
    it('removes tools from any profile', () => {
      const mgr = new ToolProfileManager({
        profile: 'full',
        customExclude: ['ros2_topic_publish', 'safety_emergency_stop'],
      });
      expect(mgr.isToolIncluded('ros2_topic_publish')).toBe(false);
      expect(mgr.isToolIncluded('safety_emergency_stop')).toBe(false);
      // Others still included
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(true);
    });

    it('overrides profile inclusion (minimal)', () => {
      const mgr = new ToolProfileManager({
        profile: 'minimal',
        customExclude: ['ros2_topic_list'],
      });
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(false);
    });

    it('overrides custom include', () => {
      const mgr = new ToolProfileManager({
        profile: 'custom',
        customInclude: ['ros2_topic_list', 'safety_status'],
        customExclude: ['ros2_topic_list'],
      });
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(false);
      expect(mgr.isToolIncluded('safety_status')).toBe(true);
    });
  });

  // --- isToolIncluded ---

  describe('isToolIncluded', () => {
    it('returns true for included tools', () => {
      const mgr = new ToolProfileManager({ profile: 'minimal' });
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(true);
      expect(mgr.isToolIncluded('safety_status')).toBe(true);
      expect(mgr.isToolIncluded('system_bridge_status')).toBe(true);
    });

    it('returns false for excluded tools', () => {
      const mgr = new ToolProfileManager({ profile: 'minimal' });
      expect(mgr.isToolIncluded('ros2_service_list')).toBe(false);
      expect(mgr.isToolIncluded('ros2_tf_lookup')).toBe(false);
    });

    it('returns false for completely unknown tools', () => {
      const mgr = new ToolProfileManager({ profile: 'minimal' });
      expect(mgr.isToolIncluded('unknown_tool')).toBe(false);
    });
  });

  // --- filterTools ---

  describe('filterTools', () => {
    it('returns only included tools', () => {
      const mgr = new ToolProfileManager({ profile: 'minimal' });
      const filtered = mgr.filterTools(ALL_TOOLS);
      for (const t of filtered) {
        expect(mgr.isToolIncluded(t.name)).toBe(true);
      }
    });

    it('preserves original tool objects', () => {
      const tools = [{ name: 'ros2_topic_list', extra: 42 }];
      const mgr = new ToolProfileManager({ profile: 'full' });
      const filtered = mgr.filterTools(tools);
      expect(filtered[0]).toBe(tools[0]);
      expect((filtered[0] as Record<string, unknown>).extra).toBe(42);
    });

    it('returns empty array when nothing matches', () => {
      const mgr = new ToolProfileManager({ profile: 'custom', customInclude: [] });
      expect(mgr.filterTools(ALL_TOOLS)).toEqual([]);
    });
  });

  // --- getCategoryForTool ---

  describe('getCategoryForTool', () => {
    it.each([
      ['ros2_topic_list', 'topic'],
      ['ros2_topic_subscribe', 'topic'],
      ['ros2_service_call', 'service'],
      ['ros2_action_send_goal', 'action'],
      ['ros2_param_get', 'param'],
      ['ros2_tf_lookup', 'tf'],
      ['ros2_diagnostic_status', 'diagnostic'],
      ['ros2_fleet_status', 'fleet'],
      ['ros2_launch_list', 'launch'],
      ['ros2_waypoint_save', 'waypoint'],
      ['ros2_sensor_summary', 'sensor'],
      ['ros2_camera_info', 'camera'],
      ['ros2_map_info', 'map'],
      ['ros2_command_history', 'history'],   // ros2_command_ -> first segment 'command' not in categories; special handling
      ['ros2_plan_path', 'path'],
      ['ros2_network_stats', 'network'],
      ['ros2_introspection_topic', 'introspection'],
      ['ros2_namespace_list', 'namespace'],
      ['ros2_description_fetch', 'description'],
      ['ros2_batch_publish', 'batch'],
      ['ros2_conditional_publish', 'conditional'],
      ['ros2_scheduled_publish', 'scheduled'],
      ['ros2_recording_start', 'recording'],
      ['safety_emergency_stop', 'safety'],
      ['safety_status', 'safety'],
      ['system_bridge_status', 'system'],
      ['system_node_list', 'system'],
    ])('%s -> %s', (toolName, expectedCategory) => {
      expect(ToolProfileManager.getCategoryForTool(toolName)).toBe(expectedCategory);
    });

    // Special-case mappings
    it.each([
      ['ros2_navigation_status', 'path'],
      ['ros2_costmap_update', 'path'],
      ['ros2_image_preview', 'camera'],
      ['ros2_battery_status', 'power'],
    ])('special prefix: %s -> %s', (toolName, expectedCategory) => {
      expect(ToolProfileManager.getCategoryForTool(toolName)).toBe(expectedCategory);
    });

    it('returns null for unknown tools', () => {
      expect(ToolProfileManager.getCategoryForTool('unknown_tool')).toBeNull();
      expect(ToolProfileManager.getCategoryForTool('ros2_zzz_foo')).toBeNull();
      expect(ToolProfileManager.getCategoryForTool('')).toBeNull();
    });
  });

  // --- getAllCategories ---

  describe('getAllCategories', () => {
    it('returns all known categories', () => {
      const cats = ToolProfileManager.getAllCategories();
      expect(cats).toContain('topic');
      expect(cats).toContain('safety');
      expect(cats).toContain('system');
      expect(cats).toContain('service');
      expect(cats).toContain('action');
      expect(cats).toContain('param');
      expect(cats).toContain('diagnostic');
      expect(cats).toContain('sensor');
      expect(cats).toContain('power');
      expect(cats).toContain('tf');
      expect(cats).toContain('fleet');
      expect(cats).toContain('launch');
      expect(cats).toContain('waypoint');
      expect(cats).toContain('camera');
      expect(cats).toContain('map');
      expect(cats).toContain('history');
      expect(cats).toContain('path');
      expect(cats).toContain('network');
      expect(cats).toContain('introspection');
      expect(cats).toContain('namespace');
      expect(cats).toContain('description');
      expect(cats).toContain('batch');
      expect(cats).toContain('conditional');
      expect(cats).toContain('scheduled');
      expect(cats).toContain('recording');
    });

    it('returns a fresh array each call', () => {
      const a = ToolProfileManager.getAllCategories();
      const b = ToolProfileManager.getAllCategories();
      expect(a).toEqual(b);
      expect(a).not.toBe(b);
    });
  });

  // --- setProfile ---

  describe('setProfile', () => {
    it('changes the active profile', () => {
      const mgr = new ToolProfileManager({ profile: 'full' });
      expect(mgr.getProfile()).toBe('full');

      mgr.setProfile('minimal');
      expect(mgr.getProfile()).toBe('minimal');
      expect(mgr.isToolIncluded('ros2_service_list')).toBe(false);
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(true);
    });

    it('preserves customExclude when changing profile', () => {
      const mgr = new ToolProfileManager({
        profile: 'full',
        customExclude: ['ros2_topic_list'],
      });
      mgr.setProfile('minimal');
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(false);
    });
  });

  // --- excludeTool / includeTool ---

  describe('excludeTool', () => {
    it('adds tool to exclude list', () => {
      const mgr = new ToolProfileManager({ profile: 'full' });
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(true);

      mgr.excludeTool('ros2_topic_list');
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(false);
    });

    it('does not add duplicates', () => {
      const mgr = new ToolProfileManager({ profile: 'full' });
      mgr.excludeTool('ros2_topic_list');
      mgr.excludeTool('ros2_topic_list');
      expect(mgr.getConfig().customExclude).toEqual(['ros2_topic_list']);
    });
  });

  describe('includeTool', () => {
    it('removes tool from exclude list', () => {
      const mgr = new ToolProfileManager({
        profile: 'full',
        customExclude: ['ros2_topic_list'],
      });
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(false);

      mgr.includeTool('ros2_topic_list');
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(true);
    });

    it('is a no-op if no exclude list exists', () => {
      const mgr = new ToolProfileManager({ profile: 'full' });
      mgr.includeTool('ros2_topic_list'); // should not throw
      expect(mgr.isToolIncluded('ros2_topic_list')).toBe(true);
    });
  });

  // --- getIncludedCategories ---

  describe('getIncludedCategories', () => {
    it('returns minimal categories for minimal profile', () => {
      const mgr = new ToolProfileManager({ profile: 'minimal' });
      expect(mgr.getIncludedCategories().sort()).toEqual(
        ['topic', 'safety', 'system'].sort(),
      );
    });

    it('returns standard categories for standard profile', () => {
      const mgr = new ToolProfileManager({ profile: 'standard' });
      const cats = mgr.getIncludedCategories();
      expect(cats).toContain('topic');
      expect(cats).toContain('service');
      expect(cats).toContain('power');
      expect(cats).not.toContain('tf');
    });

    it('returns all categories for full profile', () => {
      const mgr = new ToolProfileManager({ profile: 'full' });
      expect(mgr.getIncludedCategories()).toEqual(
        ToolProfileManager.getAllCategories(),
      );
    });

    it('returns empty array for custom profile', () => {
      const mgr = new ToolProfileManager({ profile: 'custom' });
      expect(mgr.getIncludedCategories()).toEqual([]);
    });
  });

  // --- getStats ---

  describe('getStats', () => {
    it('returns correct counts for full profile', () => {
      const mgr = new ToolProfileManager({ profile: 'full' });
      const stats = mgr.getStats(ALL_TOOLS);
      expect(stats.profile).toBe('full');
      expect(stats.totalTools).toBe(ALL_TOOLS.length);
      expect(stats.includedTools).toBe(ALL_TOOLS.length);
      expect(stats.excludedTools).toBe(0);
    });

    it('returns correct counts for minimal profile', () => {
      const mgr = new ToolProfileManager({ profile: 'minimal' });
      const stats = mgr.getStats(ALL_TOOLS);
      expect(stats.profile).toBe('minimal');
      expect(stats.totalTools).toBe(ALL_TOOLS.length);
      expect(stats.includedTools).toBe(MINIMAL_TOOL_NAMES.length);
      expect(stats.excludedTools).toBe(ALL_TOOLS.length - MINIMAL_TOOL_NAMES.length);
      expect(stats.categories.sort()).toEqual(['topic', 'safety', 'system'].sort());
    });

    it('accounts for customExclude', () => {
      const mgr = new ToolProfileManager({
        profile: 'full',
        customExclude: ['ros2_topic_list', 'safety_status'],
      });
      const stats = mgr.getStats(ALL_TOOLS);
      expect(stats.includedTools).toBe(ALL_TOOLS.length - 2);
      expect(stats.excludedTools).toBe(2);
    });
  });
});

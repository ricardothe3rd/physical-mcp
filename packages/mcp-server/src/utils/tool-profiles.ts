/**
 * Tool profile management for PhysicalMCP.
 *
 * Profiles control which MCP tools are exposed to the AI agent.
 *   - minimal: topic, safety, system
 *   - standard: minimal + service, action, param, diagnostic, sensor, power
 *   - full: every registered tool (default)
 *   - custom: user-supplied include list
 *
 * A customExclude list can further remove tools from any profile.
 */

export type ToolProfile = 'minimal' | 'standard' | 'full' | 'custom';

export interface ToolProfileConfig {
  profile: ToolProfile;
  customInclude?: string[];
  customExclude?: string[];
}

// ---------------------------------------------------------------------------
// Category definitions per profile
// ---------------------------------------------------------------------------

const MINIMAL_CATEGORIES: readonly string[] = [
  'topic',
  'safety',
  'system',
];

const STANDARD_CATEGORIES: readonly string[] = [
  ...MINIMAL_CATEGORIES,
  'service',
  'action',
  'param',
  'diagnostic',
  'sensor',
  'power',
];

// ---------------------------------------------------------------------------
// All known categories (order used by getAllCategories)
// ---------------------------------------------------------------------------

const ALL_CATEGORIES: readonly string[] = [
  'topic',
  'safety',
  'system',
  'service',
  'action',
  'param',
  'diagnostic',
  'sensor',
  'power',
  'tf',
  'fleet',
  'launch',
  'waypoint',
  'camera',
  'map',
  'history',
  'path',
  'network',
  'introspection',
  'namespace',
  'description',
  'batch',
  'conditional',
  'scheduled',
  'recording',
];

// ---------------------------------------------------------------------------
// Special sub-prefix -> category mapping for ros2_ tools
// ---------------------------------------------------------------------------

const ROS2_SPECIAL_PREFIXES: Record<string, string> = {
  navigation_: 'path',
  costmap_: 'path',
  plan_: 'path',
  image_: 'camera',
  battery_: 'power',
  command_: 'history',
};

// ---------------------------------------------------------------------------
// Helper: derive category from a tool name
// ---------------------------------------------------------------------------

function deriveCategory(toolName: string): string | null {
  // safety_ prefix
  if (toolName.startsWith('safety_')) {
    return 'safety';
  }

  // system_ prefix
  if (toolName.startsWith('system_')) {
    return 'system';
  }

  // ros2_ prefix – strip it, then determine category
  if (toolName.startsWith('ros2_')) {
    const rest = toolName.slice('ros2_'.length);

    // Check special prefixes first (order doesn't matter – they're unique)
    for (const [prefix, category] of Object.entries(ROS2_SPECIAL_PREFIXES)) {
      if (rest.startsWith(prefix)) {
        return category;
      }
    }

    // Standard: first segment before '_' is the category keyword
    const underscoreIdx = rest.indexOf('_');
    const keyword = underscoreIdx === -1 ? rest : rest.slice(0, underscoreIdx);

    if (ALL_CATEGORIES.includes(keyword)) {
      return keyword;
    }

    // Unknown ros2_ category
    return null;
  }

  return null;
}

// ---------------------------------------------------------------------------
// ToolProfileManager
// ---------------------------------------------------------------------------

export class ToolProfileManager {
  private config: ToolProfileConfig;

  constructor(config?: ToolProfileConfig) {
    this.config = config ?? { profile: 'full' };
  }

  /** Current profile name. */
  getProfile(): ToolProfile {
    return this.config.profile;
  }

  /** Full config object (shallow copy). */
  getConfig(): ToolProfileConfig {
    return { ...this.config };
  }

  /** Replace the active profile (resets customInclude). */
  setProfile(profile: ToolProfile): void {
    this.config = {
      ...this.config,
      profile,
    };
  }

  /** Add a tool name to the exclude list. */
  excludeTool(toolName: string): void {
    const list = this.config.customExclude ?? [];
    if (!list.includes(toolName)) {
      list.push(toolName);
    }
    this.config.customExclude = list;
  }

  /** Remove a tool name from the exclude list. */
  includeTool(toolName: string): void {
    if (!this.config.customExclude) return;
    this.config.customExclude = this.config.customExclude.filter(
      (n) => n !== toolName,
    );
  }

  // -----------------------------------------------------------------------
  // Tool inclusion logic
  // -----------------------------------------------------------------------

  /** Whether a single tool name passes the current profile + exclude list. */
  isToolIncluded(toolName: string): boolean {
    // 1. Check customExclude first – always wins
    if (this.config.customExclude?.includes(toolName)) {
      return false;
    }

    // 2. Profile-based inclusion
    switch (this.config.profile) {
      case 'full':
        return true;

      case 'custom':
        return this.config.customInclude?.includes(toolName) ?? false;

      case 'minimal':
      case 'standard': {
        const categories =
          this.config.profile === 'minimal'
            ? MINIMAL_CATEGORIES
            : STANDARD_CATEGORIES;
        const cat = deriveCategory(toolName);
        return cat !== null && categories.includes(cat);
      }

      default:
        return false;
    }
  }

  /** Filter an array of tool-like objects (must have a `name` property). */
  filterTools<T extends { name: string }>(tools: T[]): T[] {
    return tools.filter((t) => this.isToolIncluded(t.name));
  }

  // -----------------------------------------------------------------------
  // Introspection helpers
  // -----------------------------------------------------------------------

  /** Categories included by the current profile (not applicable for custom). */
  getIncludedCategories(): string[] {
    switch (this.config.profile) {
      case 'minimal':
        return [...MINIMAL_CATEGORIES];
      case 'standard':
        return [...STANDARD_CATEGORIES];
      case 'full':
        return [...ALL_CATEGORIES];
      case 'custom':
        return [];
      default:
        return [];
    }
  }

  /** All categories the system knows about. */
  static getAllCategories(): string[] {
    return [...ALL_CATEGORIES];
  }

  /** Derive the category for a tool name, or null if unknown. */
  static getCategoryForTool(toolName: string): string | null {
    return deriveCategory(toolName);
  }

  /** Summary statistics for a given set of tools. */
  getStats(allTools: { name: string }[]): {
    profile: ToolProfile;
    totalTools: number;
    includedTools: number;
    excludedTools: number;
    categories: string[];
  } {
    const included = this.filterTools(allTools);
    return {
      profile: this.config.profile,
      totalTools: allTools.length,
      includedTools: included.length,
      excludedTools: allTools.length - included.length,
      categories: this.getIncludedCategories(),
    };
  }
}

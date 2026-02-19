/**
 * Shared constants for PhysicalMCP.
 *
 * Centralizes magic numbers, defaults, and configuration values used
 * across the codebase.
 */

// --- Bridge Connection ---
export const DEFAULT_BRIDGE_URL = 'ws://localhost:9090';
export const BRIDGE_CONNECT_TIMEOUT_MS = 5000;
export const BRIDGE_COMMAND_TIMEOUT_MS = 10000;
export const BRIDGE_RECONNECT_INTERVAL_MS = 3000;
export const BRIDGE_MAX_RECONNECT_ATTEMPTS = 10;
export const BRIDGE_PING_INTERVAL_MS = 30000;

// --- Safety Defaults ---
export const DEFAULT_MAX_LINEAR_VELOCITY = 0.5;    // m/s
export const DEFAULT_MAX_ANGULAR_VELOCITY = 1.5;    // rad/s
export const DEFAULT_MAX_LINEAR_ACCEL = 1.0;        // m/s²
export const DEFAULT_MAX_ANGULAR_ACCEL = 2.0;       // rad/s²
export const DEFAULT_GEOFENCE_RADIUS = 5.0;         // meters
export const DEFAULT_GEOFENCE_CENTER = Object.freeze({ x: 0, y: 0, z: 0 });
export const DEFAULT_GEOFENCE_MIN = Object.freeze({ x: -5, y: -5, z: -1 });
export const DEFAULT_GEOFENCE_MAX = Object.freeze({ x: 5, y: 5, z: 5 });
export const DEFAULT_GEOFENCE_WARNING_MARGIN = 0.5; // meters
export const EMERGENCY_STOP_TOPIC = '/cmd_vel';

// --- Rate Limiting ---
export const DEFAULT_PUBLISH_RATE_LIMIT = 10;       // Hz (per second)
export const DEFAULT_PUBLISH_RATE_WINDOW_MS = 1000;
export const DEFAULT_SERVICE_RATE_LIMIT = 60;        // per minute
export const DEFAULT_SERVICE_RATE_WINDOW_MS = 60000;
export const DEFAULT_ACTION_RATE_LIMIT = 30;         // per minute
export const DEFAULT_ACTION_RATE_WINDOW_MS = 60000;

// --- Audit Logging ---
export const DEFAULT_AUDIT_MAX_ENTRIES = 10000;
export const AUDIT_CLEANUP_THRESHOLD = 100;

// --- Deadman Switch ---
export const DEFAULT_DEADMAN_TIMEOUT_MS = 5000;
export const DEADMAN_CHECK_INTERVAL_MS = 1000;

// --- Tool Categories ---
export const TOOL_CATEGORIES = Object.freeze([
  'topic',
  'service',
  'action',
  'safety',
  'system',
  'batch',
  'recording',
  'conditional',
  'scheduled',
  'tf',
  'diagnostic',
  'fleet',
  'launch',
  'waypoint',
  'introspection',
  'namespace',
  'sensor',
] as const);
export type ToolCategory = typeof TOOL_CATEGORIES[number];

// --- Blocked Defaults ---
export const DEFAULT_BLOCKED_TOPICS = Object.freeze(['/rosout', '/parameter_events'] as const);
export const DEFAULT_BLOCKED_SERVICES = Object.freeze(['/kill', '/shutdown'] as const);

// --- Protocol ---
export const PROTOCOL_VERSION = '1.0.0';
export const MAX_MESSAGE_SIZE_BYTES = 1_048_576; // 1 MB

// --- Server ---
export const SERVER_NAME = '@ricardothe3rd/physical-mcp';
export const SERVER_VERSION = '0.1.0';

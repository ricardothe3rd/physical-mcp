/**
 * Shared constants for PhysicalMCP.
 */

// --- Bridge ---
export const DEFAULT_BRIDGE_URL = 'ws://localhost:9090';
export const DEFAULT_BRIDGE_PORT = 9090;
export const BRIDGE_REQUEST_TIMEOUT_MS = 10000;
export const BRIDGE_HEARTBEAT_INTERVAL_MS = 15000;
export const BRIDGE_HEARTBEAT_STALE_MS = 30000;
export const BRIDGE_RECONNECT_INTERVAL_MS = 5000;

// --- Circuit Breaker ---
export const CIRCUIT_BREAKER_THRESHOLD = 5;
export const CIRCUIT_BREAKER_RESET_MS = 30000;

// --- Retry ---
export const RETRY_DELAYS_MS = [1000, 3000, 5000];
export const DEFAULT_MAX_RETRIES = 3;

// --- Safety ---
export const DEFAULT_LINEAR_MAX_VELOCITY = 0.5;    // m/s
export const DEFAULT_ANGULAR_MAX_VELOCITY = 1.5;    // rad/s
export const DEFAULT_LINEAR_MAX_ACCEL = 1.0;        // m/s²
export const DEFAULT_ANGULAR_MAX_ACCEL = 3.0;       // rad/s²
export const DEFAULT_PUBLISH_HZ = 10;
export const DEFAULT_SERVICE_PER_MINUTE = 60;
export const DEFAULT_ACTION_PER_MINUTE = 30;
export const DEFAULT_GEOFENCE_WARNING_MARGIN = 1.0; // meters
export const DEFAULT_DEADMAN_TIMEOUT_MS = 30000;
export const DEFAULT_AUDIT_MAX_ENTRIES = 10000;

// --- Acceleration ---
export const ACCEL_TRACKING_GAP_THRESHOLD_S = 2.0;  // reset tracking after this gap

// --- Node ---
export const MIN_NODE_VERSION = 18;

// --- Package ---
export const PACKAGE_NAME = '@ricardothe3rd/physical-mcp';
export const VERSION = '0.1.0';

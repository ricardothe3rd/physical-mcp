/**
 * Custom error types for PhysicalMCP.
 */

/** Base error for all PhysicalMCP errors */
export class PhysicalMcpError extends Error {
  constructor(message: string, public readonly code: string) {
    super(message);
    this.name = 'PhysicalMcpError';
  }
}

/** Bridge connection errors */
export class BridgeConnectionError extends PhysicalMcpError {
  constructor(message: string, public readonly bridgeUrl?: string) {
    super(message, 'BRIDGE_CONNECTION_ERROR');
    this.name = 'BridgeConnectionError';
  }
}

/** Bridge timeout errors */
export class BridgeTimeoutError extends PhysicalMcpError {
  constructor(message: string, public readonly timeoutMs?: number) {
    super(message, 'BRIDGE_TIMEOUT');
    this.name = 'BridgeTimeoutError';
  }
}

/** Safety violation errors (command blocked) */
export class SafetyViolationError extends PhysicalMcpError {
  constructor(
    message: string,
    public readonly violations: Array<{ type: string; message: string }>
  ) {
    super(message, 'SAFETY_VIOLATION');
    this.name = 'SafetyViolationError';
  }
}

/** Emergency stop active error */
export class EmergencyStopError extends PhysicalMcpError {
  constructor(message = 'Emergency stop is active. All commands are blocked.') {
    super(message, 'EMERGENCY_STOP_ACTIVE');
    this.name = 'EmergencyStopError';
  }
}

/** Input validation error */
export class ValidationError extends PhysicalMcpError {
  constructor(message: string, public readonly field?: string) {
    super(message, 'VALIDATION_ERROR');
    this.name = 'ValidationError';
  }
}

/** Tool not found error */
export class UnknownToolError extends PhysicalMcpError {
  constructor(toolName: string) {
    super(`Unknown tool: ${toolName}`, 'UNKNOWN_TOOL');
    this.name = 'UnknownToolError';
  }
}

/** Rate limit exceeded error */
export class RateLimitError extends PhysicalMcpError {
  constructor(message: string, public readonly key?: string) {
    super(message, 'RATE_LIMIT_EXCEEDED');
    this.name = 'RateLimitError';
  }
}

/** Geofence violation error */
export class GeofenceViolationError extends PhysicalMcpError {
  constructor(
    message: string,
    public readonly position?: { x: number; y: number; z: number }
  ) {
    super(message, 'GEOFENCE_VIOLATION');
    this.name = 'GeofenceViolationError';
  }
}

/** Policy configuration error */
export class PolicyError extends PhysicalMcpError {
  constructor(message: string) {
    super(message, 'POLICY_ERROR');
    this.name = 'PolicyError';
  }
}

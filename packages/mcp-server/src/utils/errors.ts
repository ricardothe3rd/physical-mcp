/**
 * Custom error types for PhysicalMCP.
 */

export class PhysicalMCPError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'PhysicalMCPError';
  }
}

export class BridgeConnectionError extends PhysicalMCPError {
  constructor(message: string, public readonly url?: string) {
    super(message);
    this.name = 'BridgeConnectionError';
  }
}

export class BridgeTimeoutError extends PhysicalMCPError {
  constructor(public readonly commandType: string, public readonly timeoutMs: number) {
    super(`Bridge command "${commandType}" timed out after ${timeoutMs}ms`);
    this.name = 'BridgeTimeoutError';
  }
}

export class SafetyViolationError extends PhysicalMCPError {
  constructor(
    message: string,
    public readonly violationType: string,
    public readonly details: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SafetyViolationError';
  }
}

export class PolicyLoadError extends PhysicalMCPError {
  constructor(message: string, public readonly filePath?: string) {
    super(message);
    this.name = 'PolicyLoadError';
  }
}

/**
 * Safety system type definitions.
 */

export interface VelocityLimits {
  linearMax: number;   // m/s
  angularMax: number;  // rad/s
}

export interface GeofenceBounds {
  xMin: number;
  xMax: number;
  yMin: number;
  yMax: number;
  zMin: number;
  zMax: number;
}

export interface RateLimitConfig {
  publishHz: number;        // max publishes per second
  servicePerMinute: number; // max service calls per minute
  actionPerMinute: number;  // max action goals per minute
}

export interface SafetyPolicy {
  name: string;
  description: string;
  velocity: VelocityLimits;
  geofence: GeofenceBounds;
  rateLimits: RateLimitConfig;
  blockedTopics: string[];
  blockedServices: string[];
  allowedTopics?: string[];     // if set, only these are allowed
  allowedServices?: string[];   // if set, only these are allowed
}

export type SafetyViolationType =
  | 'velocity_exceeded'
  | 'geofence_violation'
  | 'rate_limit_exceeded'
  | 'blocked_topic'
  | 'blocked_service'
  | 'emergency_stop_active';

export interface SafetyViolation {
  type: SafetyViolationType;
  message: string;
  details: Record<string, unknown>;
  timestamp: number;
}

export interface SafetyCheckResult {
  allowed: boolean;
  violations: SafetyViolation[];
}

export interface AuditEntry {
  id: string;
  timestamp: number;
  command: string;
  target: string;
  params: Record<string, unknown>;
  safetyResult: SafetyCheckResult;
  executionResult?: 'success' | 'error';
  executionError?: string;
}

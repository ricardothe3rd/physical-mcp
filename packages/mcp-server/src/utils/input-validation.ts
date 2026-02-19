/**
 * Input validation for ROS2 topic, service, and action names.
 * Validates names BEFORE they reach the bridge.
 */

export interface ValidationResult {
  valid: boolean;
  error?: string;
}

/**
 * Regex for valid ROS2 name segments (between slashes).
 * Each segment must be alphanumeric or underscore, starting with a letter or underscore.
 */
const ROS2_NAME_SEGMENT = /^[a-zA-Z_][a-zA-Z0-9_]*$/;

/**
 * Validates a ROS2 name (topic, service, or action).
 *
 * Rules:
 * - Must be a non-empty string
 * - Must start with `/`
 * - Root `/` is allowed as a special case
 * - No double slashes (`//`)
 * - No trailing slash (except root `/`)
 * - Each segment between slashes must be alphanumeric + underscores
 * - No spaces or special characters
 */
function validateRos2Name(name: string, kind: string): ValidationResult {
  if (typeof name !== 'string' || name.length === 0) {
    return { valid: false, error: `${kind} name must be a non-empty string` };
  }

  if (name[0] !== '/') {
    return { valid: false, error: `${kind} name must start with '/'` };
  }

  // Root topic `/` is valid on its own
  if (name === '/') {
    return { valid: true };
  }

  if (name.includes('//')) {
    return { valid: false, error: `${kind} name must not contain double slashes '//'` };
  }

  if (name.length > 1 && name.endsWith('/')) {
    return { valid: false, error: `${kind} name must not end with a trailing '/'` };
  }

  // Validate each segment between slashes
  const segments = name.slice(1).split('/');
  for (const segment of segments) {
    if (!ROS2_NAME_SEGMENT.test(segment)) {
      return {
        valid: false,
        error: `${kind} name contains invalid segment '${segment}' — segments must be alphanumeric or underscores, starting with a letter or underscore`,
      };
    }
  }

  return { valid: true };
}

/**
 * Validates a ROS2 topic name.
 * Must start with `/`, contain only alphanumeric + underscores + `/`,
 * no double slashes, no trailing slash (except root).
 */
export function validateTopicName(name: string): ValidationResult {
  return validateRos2Name(name, 'Topic');
}

/**
 * Validates a ROS2 service name.
 * Same rules as topic names.
 */
export function validateServiceName(name: string): ValidationResult {
  return validateRos2Name(name, 'Service');
}

/**
 * Validates a ROS2 action name.
 * Same rules as topic names.
 */
export function validateActionName(name: string): ValidationResult {
  return validateRos2Name(name, 'Action');
}

/**
 * Validates a ROS2 node name.
 * Same rules as topic names (must start with `/`, valid segments, etc.).
 */
export function validateNodeName(name: string): ValidationResult {
  return validateRos2Name(name, 'Node');
}

/**
 * Validates a ROS2 parameter name.
 * Must be a non-empty string containing only alphanumeric characters,
 * underscores, and dots (for nested parameters).
 */
export function validateParamName(name: string): ValidationResult {
  if (typeof name !== 'string' || name.length === 0) {
    return { valid: false, error: 'Parameter name must be a non-empty string' };
  }

  if (/[^a-zA-Z0-9_.]/.test(name)) {
    return {
      valid: false,
      error: `Parameter name '${name}' contains invalid characters — only alphanumeric, underscores, and dots are allowed`,
    };
  }

  return { valid: true };
}

/**
 * Validates a message payload.
 * Must be a non-null, non-array plain object.
 */
export function validateMessagePayload(payload: unknown): ValidationResult {
  if (payload === null || payload === undefined) {
    return { valid: false, error: 'Message payload must not be null or undefined' };
  }

  if (typeof payload !== 'object') {
    return { valid: false, error: `Message payload must be an object, got ${typeof payload}` };
  }

  if (Array.isArray(payload)) {
    return { valid: false, error: 'Message payload must be a plain object, not an array' };
  }

  return { valid: true };
}

/**
 * Sanitizes an input string: trims whitespace, removes control characters,
 * and enforces a maximum length.
 */
export function sanitizeString(input: string, maxLength = 1000): string {
  // Trim leading/trailing whitespace
  let result = input.trim();

  // Remove control characters (U+0000–U+001F and U+007F–U+009F), except common whitespace
  // eslint-disable-next-line no-control-regex
  result = result.replace(/[\x00-\x08\x0B\x0C\x0E-\x1F\x7F-\x9F]/g, '');

  // Enforce max length
  if (result.length > maxLength) {
    result = result.slice(0, maxLength);
  }

  return result;
}

import { describe, it, expect } from 'vitest';
import {
  DEFAULT_BRIDGE_URL,
  BRIDGE_CONNECT_TIMEOUT_MS,
  BRIDGE_COMMAND_TIMEOUT_MS,
  BRIDGE_RECONNECT_INTERVAL_MS,
  BRIDGE_MAX_RECONNECT_ATTEMPTS,
  BRIDGE_PING_INTERVAL_MS,
  DEFAULT_MAX_LINEAR_VELOCITY,
  DEFAULT_MAX_ANGULAR_VELOCITY,
  DEFAULT_MAX_LINEAR_ACCEL,
  DEFAULT_MAX_ANGULAR_ACCEL,
  DEFAULT_GEOFENCE_RADIUS,
  DEFAULT_GEOFENCE_CENTER,
  DEFAULT_GEOFENCE_MIN,
  DEFAULT_GEOFENCE_MAX,
  DEFAULT_GEOFENCE_WARNING_MARGIN,
  EMERGENCY_STOP_TOPIC,
  DEFAULT_PUBLISH_RATE_LIMIT,
  DEFAULT_PUBLISH_RATE_WINDOW_MS,
  DEFAULT_SERVICE_RATE_LIMIT,
  DEFAULT_SERVICE_RATE_WINDOW_MS,
  DEFAULT_ACTION_RATE_LIMIT,
  DEFAULT_ACTION_RATE_WINDOW_MS,
  DEFAULT_AUDIT_MAX_ENTRIES,
  AUDIT_CLEANUP_THRESHOLD,
  DEFAULT_DEADMAN_TIMEOUT_MS,
  DEADMAN_CHECK_INTERVAL_MS,
  TOOL_CATEGORIES,
  DEFAULT_BLOCKED_TOPICS,
  DEFAULT_BLOCKED_SERVICES,
  PROTOCOL_VERSION,
  MAX_MESSAGE_SIZE_BYTES,
  SERVER_NAME,
  SERVER_VERSION,
} from './constants.js';

describe('constants', () => {
  describe('numeric constants are positive numbers', () => {
    const numericConstants: Record<string, number> = {
      BRIDGE_CONNECT_TIMEOUT_MS,
      BRIDGE_COMMAND_TIMEOUT_MS,
      BRIDGE_RECONNECT_INTERVAL_MS,
      BRIDGE_MAX_RECONNECT_ATTEMPTS,
      BRIDGE_PING_INTERVAL_MS,
      DEFAULT_MAX_LINEAR_VELOCITY,
      DEFAULT_MAX_ANGULAR_VELOCITY,
      DEFAULT_MAX_LINEAR_ACCEL,
      DEFAULT_MAX_ANGULAR_ACCEL,
      DEFAULT_GEOFENCE_RADIUS,
      DEFAULT_GEOFENCE_WARNING_MARGIN,
      DEFAULT_PUBLISH_RATE_LIMIT,
      DEFAULT_PUBLISH_RATE_WINDOW_MS,
      DEFAULT_SERVICE_RATE_LIMIT,
      DEFAULT_SERVICE_RATE_WINDOW_MS,
      DEFAULT_ACTION_RATE_LIMIT,
      DEFAULT_ACTION_RATE_WINDOW_MS,
      DEFAULT_AUDIT_MAX_ENTRIES,
      AUDIT_CLEANUP_THRESHOLD,
      DEFAULT_DEADMAN_TIMEOUT_MS,
      DEADMAN_CHECK_INTERVAL_MS,
      MAX_MESSAGE_SIZE_BYTES,
    };

    for (const [name, value] of Object.entries(numericConstants)) {
      it(`${name} is a positive number`, () => {
        expect(typeof value).toBe('number');
        expect(value).toBeGreaterThan(0);
        expect(Number.isFinite(value)).toBe(true);
      });
    }
  });

  describe('DEFAULT_BRIDGE_URL', () => {
    it('is a valid ws:// URL', () => {
      expect(DEFAULT_BRIDGE_URL).toMatch(/^ws:\/\/.+/);
      // Verify it can be parsed as a URL
      const url = new URL(DEFAULT_BRIDGE_URL);
      expect(url.protocol).toBe('ws:');
      expect(url.hostname).toBeTruthy();
      expect(url.port).toBeTruthy();
    });
  });

  describe('TOOL_CATEGORIES', () => {
    it('has exactly 17 entries', () => {
      expect(TOOL_CATEGORIES).toHaveLength(17);
    });

    it('entries are all lowercase strings', () => {
      for (const category of TOOL_CATEGORIES) {
        expect(typeof category).toBe('string');
        expect(category).toBe(category.toLowerCase());
      }
    });

    it('contains all expected categories', () => {
      const expected = [
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
      ];
      expect([...TOOL_CATEGORIES]).toEqual(expected);
    });

    it('has no duplicate entries', () => {
      const unique = new Set(TOOL_CATEGORIES);
      expect(unique.size).toBe(TOOL_CATEGORIES.length);
    });
  });

  describe('DEFAULT_BLOCKED_TOPICS', () => {
    it('all entries start with /', () => {
      for (const topic of DEFAULT_BLOCKED_TOPICS) {
        expect(topic).toMatch(/^\//);
      }
    });
  });

  describe('DEFAULT_BLOCKED_SERVICES', () => {
    it('all entries start with /', () => {
      for (const service of DEFAULT_BLOCKED_SERVICES) {
        expect(service).toMatch(/^\//);
      }
    });
  });

  describe('PROTOCOL_VERSION', () => {
    it('matches semver pattern', () => {
      // Semver: MAJOR.MINOR.PATCH with optional pre-release and build metadata
      const semverRegex = /^\d+\.\d+\.\d+(-[a-zA-Z0-9.]+)?(\+[a-zA-Z0-9.]+)?$/;
      expect(PROTOCOL_VERSION).toMatch(semverRegex);
    });
  });

  describe('MAX_MESSAGE_SIZE_BYTES', () => {
    it('is exactly 1 MB (1048576 bytes)', () => {
      expect(MAX_MESSAGE_SIZE_BYTES).toBe(1_048_576);
      expect(MAX_MESSAGE_SIZE_BYTES).toBe(1024 * 1024);
    });
  });

  describe('SERVER_NAME', () => {
    it('matches the npm package name', () => {
      expect(SERVER_NAME).toBe('@ricardothe3rd/physical-mcp');
    });
  });

  describe('SERVER_VERSION', () => {
    it('matches semver pattern', () => {
      const semverRegex = /^\d+\.\d+\.\d+(-[a-zA-Z0-9.]+)?(\+[a-zA-Z0-9.]+)?$/;
      expect(SERVER_VERSION).toMatch(semverRegex);
    });
  });

  describe('geofence defaults', () => {
    it('min < max for x axis', () => {
      expect(DEFAULT_GEOFENCE_MIN.x).toBeLessThan(DEFAULT_GEOFENCE_MAX.x);
    });

    it('min < max for y axis', () => {
      expect(DEFAULT_GEOFENCE_MIN.y).toBeLessThan(DEFAULT_GEOFENCE_MAX.y);
    });

    it('min < max for z axis', () => {
      expect(DEFAULT_GEOFENCE_MIN.z).toBeLessThan(DEFAULT_GEOFENCE_MAX.z);
    });

    it('center is within min/max bounds', () => {
      expect(DEFAULT_GEOFENCE_CENTER.x).toBeGreaterThanOrEqual(DEFAULT_GEOFENCE_MIN.x);
      expect(DEFAULT_GEOFENCE_CENTER.x).toBeLessThanOrEqual(DEFAULT_GEOFENCE_MAX.x);
      expect(DEFAULT_GEOFENCE_CENTER.y).toBeGreaterThanOrEqual(DEFAULT_GEOFENCE_MIN.y);
      expect(DEFAULT_GEOFENCE_CENTER.y).toBeLessThanOrEqual(DEFAULT_GEOFENCE_MAX.y);
      expect(DEFAULT_GEOFENCE_CENTER.z).toBeGreaterThanOrEqual(DEFAULT_GEOFENCE_MIN.z);
      expect(DEFAULT_GEOFENCE_CENTER.z).toBeLessThanOrEqual(DEFAULT_GEOFENCE_MAX.z);
    });

    it('warning margin is positive and less than the smallest dimension span', () => {
      const spanX = DEFAULT_GEOFENCE_MAX.x - DEFAULT_GEOFENCE_MIN.x;
      const spanY = DEFAULT_GEOFENCE_MAX.y - DEFAULT_GEOFENCE_MIN.y;
      const spanZ = DEFAULT_GEOFENCE_MAX.z - DEFAULT_GEOFENCE_MIN.z;
      const minSpan = Math.min(spanX, spanY, spanZ);

      expect(DEFAULT_GEOFENCE_WARNING_MARGIN).toBeGreaterThan(0);
      expect(DEFAULT_GEOFENCE_WARNING_MARGIN).toBeLessThan(minSpan);
    });
  });

  describe('rate limit windows', () => {
    it('publish rate window is less than service rate window', () => {
      expect(DEFAULT_PUBLISH_RATE_WINDOW_MS).toBeLessThan(DEFAULT_SERVICE_RATE_WINDOW_MS);
    });

    it('publish rate window is less than action rate window', () => {
      expect(DEFAULT_PUBLISH_RATE_WINDOW_MS).toBeLessThan(DEFAULT_ACTION_RATE_WINDOW_MS);
    });

    it('rate limits are whole numbers', () => {
      expect(Number.isInteger(DEFAULT_PUBLISH_RATE_LIMIT)).toBe(true);
      expect(Number.isInteger(DEFAULT_SERVICE_RATE_LIMIT)).toBe(true);
      expect(Number.isInteger(DEFAULT_ACTION_RATE_LIMIT)).toBe(true);
    });

    it('rate windows are whole numbers in milliseconds', () => {
      expect(Number.isInteger(DEFAULT_PUBLISH_RATE_WINDOW_MS)).toBe(true);
      expect(Number.isInteger(DEFAULT_SERVICE_RATE_WINDOW_MS)).toBe(true);
      expect(Number.isInteger(DEFAULT_ACTION_RATE_WINDOW_MS)).toBe(true);
    });
  });

  describe('safety velocity defaults', () => {
    it('linear velocity is positive and within reasonable bounds', () => {
      expect(DEFAULT_MAX_LINEAR_VELOCITY).toBeGreaterThan(0);
      // Should be reasonable for a robot -- not faster than ~10 m/s
      expect(DEFAULT_MAX_LINEAR_VELOCITY).toBeLessThanOrEqual(10);
    });

    it('angular velocity is positive and within reasonable bounds', () => {
      expect(DEFAULT_MAX_ANGULAR_VELOCITY).toBeGreaterThan(0);
      // Should be reasonable -- not faster than ~2*PI rad/s (one full rotation per second)
      expect(DEFAULT_MAX_ANGULAR_VELOCITY).toBeLessThanOrEqual(2 * Math.PI);
    });

    it('linear acceleration is positive and within reasonable bounds', () => {
      expect(DEFAULT_MAX_LINEAR_ACCEL).toBeGreaterThan(0);
      expect(DEFAULT_MAX_LINEAR_ACCEL).toBeLessThanOrEqual(20);
    });

    it('angular acceleration is positive and within reasonable bounds', () => {
      expect(DEFAULT_MAX_ANGULAR_ACCEL).toBeGreaterThan(0);
      expect(DEFAULT_MAX_ANGULAR_ACCEL).toBeLessThanOrEqual(4 * Math.PI);
    });
  });

  describe('immutability of object constants', () => {
    it('DEFAULT_GEOFENCE_CENTER is a frozen object', () => {
      expect(Object.isFrozen(DEFAULT_GEOFENCE_CENTER)).toBe(true);
    });

    it('DEFAULT_GEOFENCE_MIN is a frozen object', () => {
      expect(Object.isFrozen(DEFAULT_GEOFENCE_MIN)).toBe(true);
    });

    it('DEFAULT_GEOFENCE_MAX is a frozen object', () => {
      expect(Object.isFrozen(DEFAULT_GEOFENCE_MAX)).toBe(true);
    });

    it('TOOL_CATEGORIES is a readonly tuple', () => {
      // readonly tuples via `as const` are frozen at the type level;
      // at runtime the array itself is frozen by the engine for `as const`
      // in some environments. We verify the values cannot be accidentally modified.
      expect(Object.isFrozen(TOOL_CATEGORIES)).toBe(true);
    });

    it('DEFAULT_BLOCKED_TOPICS is a readonly tuple', () => {
      expect(Object.isFrozen(DEFAULT_BLOCKED_TOPICS)).toBe(true);
    });

    it('DEFAULT_BLOCKED_SERVICES is a readonly tuple', () => {
      expect(Object.isFrozen(DEFAULT_BLOCKED_SERVICES)).toBe(true);
    });
  });
});

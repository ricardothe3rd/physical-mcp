/**
 * Edge-case tests for ROS2 input validation utilities.
 *
 * Covers boundary conditions, unusual inputs, and stress scenarios
 * that complement the core happy-path tests in input-validation.test.ts.
 */

import { describe, it, expect } from 'vitest';
import {
  validateTopicName,
  validateServiceName,
  validateActionName,
  validateMessagePayload,
  sanitizeString,
} from './input-validation.js';

// ─────────────────────────────────────────────────────────────────────────────
// 1. Topic Name Validation — Edge Cases
// ─────────────────────────────────────────────────────────────────────────────

describe('Topic Name Validation — Edge Cases', () => {
  // ── Rejection of empty / missing input ──────────────────────────────

  it('rejects an empty string', () => {
    const result = validateTopicName('');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('rejects a name that is only whitespace', () => {
    const result = validateTopicName('   ');
    expect(result.valid).toBe(false);
    // Whitespace-only string does not start with '/'
    expect(result.error).toBeDefined();
  });

  // ── Leading slash requirement ───────────────────────────────────────

  it('rejects a name missing the leading /', () => {
    const result = validateTopicName('cmd_vel');
    expect(result.valid).toBe(false);
    expect(result.error).toContain("must start with '/'");
  });

  it('rejects a name that starts with a backslash instead of forward slash', () => {
    const result = validateTopicName('\\cmd_vel');
    expect(result.valid).toBe(false);
    expect(result.error).toContain("must start with '/'");
  });

  // ── Spaces in names ────────────────────────────────────────────────

  it('rejects a topic name with spaces', () => {
    const result = validateTopicName('/cmd vel');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name where space is the only segment content', () => {
    const result = validateTopicName('/ ');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name with a tab character', () => {
    const result = validateTopicName('/cmd\tvel');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  // ── Special characters ─────────────────────────────────────────────

  it('rejects a topic name containing an exclamation mark', () => {
    const result = validateTopicName('/hello!');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name containing a hash #', () => {
    const result = validateTopicName('/topic#1');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name containing a dot', () => {
    const result = validateTopicName('/sensor.data');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name containing a hyphen', () => {
    const result = validateTopicName('/my-topic');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name containing an @ symbol', () => {
    const result = validateTopicName('/user@host');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name containing unicode characters', () => {
    const result = validateTopicName('/topico_café');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name containing emoji', () => {
    const result = validateTopicName('/robot_🤖');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name with curly braces', () => {
    const result = validateTopicName('/topic_{foo}');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name with square brackets', () => {
    const result = validateTopicName('/topic[0]');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a topic name with parentheses', () => {
    const result = validateTopicName('/topic(1)');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  // ── Very long topic names ──────────────────────────────────────────

  it('accepts a valid topic name that is exactly 255 characters long', () => {
    // Build a valid name: /aaa...aaa (254 chars of 'a' after the slash)
    const longSegment = 'a'.repeat(254);
    const name = `/${longSegment}`;
    expect(name.length).toBe(255);
    const result = validateTopicName(name);
    expect(result.valid).toBe(true);
  });

  it('accepts a valid topic name longer than 255 characters', () => {
    // NOTE: The current validation does NOT enforce a max length on topic names.
    // This test documents current behavior. A length check could be added in the future.
    const longSegment = 'a'.repeat(500);
    const name = `/${longSegment}`;
    const result = validateTopicName(name);
    expect(result.valid).toBe(true);
    // TODO: Consider adding max-length validation (e.g., 255 chars) for ROS2 name compliance
  });

  // ── Slash edge cases ───────────────────────────────────────────────

  it('accepts root topic /', () => {
    expect(validateTopicName('/')).toEqual({ valid: true });
  });

  it('rejects double slashes at the start //', () => {
    const result = validateTopicName('//');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('double slashes');
  });

  it('rejects triple slashes ///', () => {
    const result = validateTopicName('///');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('double slashes');
  });

  it('rejects double slashes in the middle /a//b', () => {
    const result = validateTopicName('/a//b');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('double slashes');
  });

  it('rejects trailing slash on a multi-segment name', () => {
    const result = validateTopicName('/a/b/');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('trailing');
  });

  it('rejects a name that is just two slashes with content //topic', () => {
    const result = validateTopicName('//topic');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('double slashes');
  });

  // ── Segment-starting-with-digit ────────────────────────────────────

  it('rejects a segment that starts with a digit', () => {
    const result = validateTopicName('/123');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects a nested segment that starts with a digit', () => {
    const result = validateTopicName('/valid/3invalid');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('accepts a segment containing digits after the first character', () => {
    expect(validateTopicName('/sensor123')).toEqual({ valid: true });
  });

  // ── Underscore patterns ────────────────────────────────────────────

  it('accepts a segment that is a single underscore', () => {
    expect(validateTopicName('/_')).toEqual({ valid: true });
  });

  it('accepts a segment that is all underscores', () => {
    expect(validateTopicName('/___')).toEqual({ valid: true });
  });

  it('accepts a segment starting with underscore followed by digits', () => {
    expect(validateTopicName('/_123')).toEqual({ valid: true });
  });

  // ── Non-string input (TypeScript escape hatch) ─────────────────────

  it('rejects null cast as string', () => {
    const result = validateTopicName(null as unknown as string);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('rejects undefined cast as string', () => {
    const result = validateTopicName(undefined as unknown as string);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('rejects a number cast as string', () => {
    const result = validateTopicName(42 as unknown as string);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('rejects an object cast as string', () => {
    const result = validateTopicName({} as unknown as string);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  // ── Deeply nested valid paths ──────────────────────────────────────

  it('accepts a topic with many nested segments', () => {
    // 50 segments deep
    const segments = Array.from({ length: 50 }, (_, i) => `seg_${i}`).join('/');
    const name = `/${segments}`;
    expect(validateTopicName(name)).toEqual({ valid: true });
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// 2. Service Name Validation — Edge Cases
// ─────────────────────────────────────────────────────────────────────────────

describe('Service Name Validation — Edge Cases', () => {
  it('rejects an empty string', () => {
    const result = validateServiceName('');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('rejects missing leading /', () => {
    const result = validateServiceName('set_speed');
    expect(result.valid).toBe(false);
    expect(result.error).toContain("must start with '/'");
  });

  it('rejects names with spaces', () => {
    const result = validateServiceName('/set speed');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects names with special characters', () => {
    const result = validateServiceName('/set-speed!');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('rejects names with unicode', () => {
    const result = validateServiceName('/servicio_señal');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });

  it('accepts a valid long service name (>255 chars)', () => {
    // Current validation does not enforce max length
    const longSegment = 'a'.repeat(300);
    const name = `/${longSegment}`;
    const result = validateServiceName(name);
    expect(result.valid).toBe(true);
    // TODO: Consider adding max-length validation for service names
  });

  it('rejects double slashes in service name', () => {
    const result = validateServiceName('/robot//set_speed');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('double slashes');
  });

  it('rejects trailing slash on service name', () => {
    const result = validateServiceName('/set_speed/');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('trailing');
  });

  it('rejects null cast as string', () => {
    const result = validateServiceName(null as unknown as string);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('rejects undefined cast as string', () => {
    const result = validateServiceName(undefined as unknown as string);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('includes "Service" in all error messages', () => {
    const tests = [
      validateServiceName(''),
      validateServiceName('no_slash'),
      validateServiceName('/a//b'),
      validateServiceName('/trailing/'),
      validateServiceName('/bad!segment'),
    ];
    for (const result of tests) {
      expect(result.valid).toBe(false);
      expect(result.error).toContain('Service');
    }
  });

  it('accepts root service /', () => {
    expect(validateServiceName('/')).toEqual({ valid: true });
  });

  it('accepts deeply nested valid service name', () => {
    expect(validateServiceName('/robot_1/arm/gripper/open')).toEqual({ valid: true });
  });

  it('rejects segment starting with digit', () => {
    const result = validateServiceName('/1bad_service');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('invalid segment');
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// 3. Action Name Validation — Edge Cases
// ─────────────────────────────────────────────────────────────────────────────

describe('Action Name Validation — Edge Cases', () => {
  it('includes "Action" in all error messages', () => {
    const tests = [
      validateActionName(''),
      validateActionName('no_slash'),
      validateActionName('/a//b'),
      validateActionName('/trailing/'),
      validateActionName('/bad!segment'),
    ];
    for (const result of tests) {
      expect(result.valid).toBe(false);
      expect(result.error).toContain('Action');
    }
  });

  it('rejects null cast as string', () => {
    const result = validateActionName(null as unknown as string);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('non-empty string');
  });

  it('accepts a valid deeply nested action name', () => {
    expect(validateActionName('/robot/arm/move_to_pose')).toEqual({ valid: true });
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// 4. Message/Data Payload Validation — Edge Cases
// ─────────────────────────────────────────────────────────────────────────────

describe('Message Payload Validation — Edge Cases', () => {
  // ── Accepted payloads ──────────────────────────────────────────────

  it('accepts an empty object', () => {
    expect(validateMessagePayload({})).toEqual({ valid: true });
  });

  it('accepts a deeply nested object', () => {
    const deep = { a: { b: { c: { d: { e: { f: { g: 'deep' } } } } } } };
    expect(validateMessagePayload(deep)).toEqual({ valid: true });
  });

  it('accepts an object with null field values', () => {
    // The validator only checks the top-level payload structure, not field contents
    const payload = { x: null, y: null, z: null };
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
  });

  it('accepts an object with undefined field values', () => {
    const payload = { x: undefined };
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
  });

  it('accepts an object containing arrays as field values', () => {
    const payload = { points: [1, 2, 3], flags: [true, false] };
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
  });

  it('accepts an object with mixed types in fields', () => {
    const payload = {
      name: 'robot',
      speed: 1.5,
      enabled: true,
      metadata: null,
      tags: ['fast', 'reliable'],
      config: { mode: 'auto' },
    };
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
  });

  it('accepts a very large object (simulating >1MB conceptual payload)', () => {
    // NOTE: The current validation does NOT enforce payload size limits.
    // This test documents current behavior — a size limit should be considered.
    const largeString = 'x'.repeat(1_000_000); // ~1MB string
    const payload = { data: largeString };
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
    // TODO: Consider adding payload size validation to prevent excessively large messages
  });

  it('accepts an object with many keys', () => {
    const payload: Record<string, number> = {};
    for (let i = 0; i < 1000; i++) {
      payload[`key_${i}`] = i;
    }
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
  });

  it('accepts an object with empty string keys', () => {
    const payload = { '': 'empty key' };
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
  });

  it('accepts an object with numeric string keys', () => {
    const payload = { '0': 'zero', '1': 'one' };
    expect(validateMessagePayload(payload)).toEqual({ valid: true });
  });

  // ── Rejected payloads ──────────────────────────────────────────────

  it('rejects null', () => {
    const result = validateMessagePayload(null);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('null');
  });

  it('rejects undefined', () => {
    const result = validateMessagePayload(undefined);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('null or undefined');
  });

  it('rejects a top-level array', () => {
    const result = validateMessagePayload([1, 2, 3]);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('not an array');
  });

  it('rejects an empty array', () => {
    const result = validateMessagePayload([]);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('not an array');
  });

  it('rejects a string', () => {
    const result = validateMessagePayload('hello');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects an empty string', () => {
    const result = validateMessagePayload('');
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects a number', () => {
    const result = validateMessagePayload(42);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects zero', () => {
    const result = validateMessagePayload(0);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects NaN', () => {
    const result = validateMessagePayload(NaN);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects Infinity', () => {
    const result = validateMessagePayload(Infinity);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects negative Infinity', () => {
    const result = validateMessagePayload(-Infinity);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects a boolean true', () => {
    const result = validateMessagePayload(true);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects a boolean false', () => {
    const result = validateMessagePayload(false);
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects a Symbol', () => {
    const result = validateMessagePayload(Symbol('test'));
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects a BigInt', () => {
    const result = validateMessagePayload(BigInt(42));
    expect(result.valid).toBe(false);
    expect(result.error).toContain('must be an object');
  });

  it('rejects a function', () => {
    const result = validateMessagePayload(() => {});
    expect(result.valid).toBe(false);
    // Functions have typeof 'function', not 'object'
    expect(result.error).toContain('must be an object');
  });

  // ── Edge objects ───────────────────────────────────────────────────

  it('accepts a Date object (it is typeof object and not an array)', () => {
    // NOTE: Date is typeof 'object' and not an array, so current validation accepts it.
    // ROS2 messages would not normally contain Date objects, but the validator doesn't
    // check for specific object types beyond array exclusion.
    const result = validateMessagePayload(new Date());
    expect(result.valid).toBe(true);
    // TODO: Consider rejecting non-plain objects (Date, RegExp, Map, Set, etc.)
  });

  it('accepts a RegExp object (it is typeof object and not an array)', () => {
    const result = validateMessagePayload(/test/);
    expect(result.valid).toBe(true);
    // TODO: Consider rejecting non-plain objects
  });

  it('accepts a Map object (it is typeof object and not an array)', () => {
    const result = validateMessagePayload(new Map());
    expect(result.valid).toBe(true);
    // TODO: Consider rejecting non-plain objects
  });

  it('accepts a Set object (it is typeof object and not an array)', () => {
    const result = validateMessagePayload(new Set());
    expect(result.valid).toBe(true);
    // TODO: Consider rejecting non-plain objects
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// 5. Numeric Parameter Validation — Edge Cases
// ─────────────────────────────────────────────────────────────────────────────

describe('Numeric Parameter Validation — Edge Cases', () => {
  // The input-validation module does not currently have a dedicated numeric
  // validator. These tests document the behavior of numeric values when they
  // appear inside message payloads and exercise sanitizeString with numeric-like
  // inputs. They also serve as a specification for future numeric validation.

  describe('numeric values within message payloads', () => {
    it('accepts payload with negative timeout-like values', () => {
      // NOTE: The message payload validator does not inspect field values.
      // Negative timeouts would need to be caught at the tool-handler level or
      // by a dedicated numeric validator.
      const payload = { timeout_sec: -5 };
      expect(validateMessagePayload(payload)).toEqual({ valid: true });
      // TODO: Add numeric range validation for timeout/count fields
    });

    it('accepts payload with zero timeout', () => {
      const payload = { timeout_sec: 0 };
      expect(validateMessagePayload(payload)).toEqual({ valid: true });
      // TODO: Zero timeout may be meaningless; consider rejecting at handler level
    });

    it('accepts payload with NaN value', () => {
      const payload = { speed: NaN };
      expect(validateMessagePayload(payload)).toEqual({ valid: true });
      // TODO: NaN in message fields is almost certainly a bug; consider validation
    });

    it('accepts payload with Infinity value', () => {
      const payload = { speed: Infinity };
      expect(validateMessagePayload(payload)).toEqual({ valid: true });
      // TODO: Infinity in numeric fields should be rejected
    });

    it('accepts payload with negative Infinity value', () => {
      const payload = { speed: -Infinity };
      expect(validateMessagePayload(payload)).toEqual({ valid: true });
      // TODO: -Infinity in numeric fields should be rejected
    });

    it('accepts payload with extremely large number', () => {
      const payload = { distance: Number.MAX_SAFE_INTEGER + 1 };
      expect(validateMessagePayload(payload)).toEqual({ valid: true });
      // TODO: Consider warning on numbers beyond MAX_SAFE_INTEGER
    });

    it('accepts payload with extremely small number', () => {
      const payload = { precision: Number.MIN_VALUE };
      expect(validateMessagePayload(payload)).toEqual({ valid: true });
    });
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// 6. sanitizeString — Additional Edge Cases
// ─────────────────────────────────────────────────────────────────────────────

describe('sanitizeString — Additional Edge Cases', () => {
  it('removes NUL bytes embedded in a string', () => {
    expect(sanitizeString('abc\x00def')).toBe('abcdef');
  });

  it('removes BEL (\\x07) character', () => {
    expect(sanitizeString('hello\x07world')).toBe('helloworld');
  });

  it('removes DEL (\\x7F) character', () => {
    expect(sanitizeString('hello\x7Fworld')).toBe('helloworld');
  });

  it('removes C1 control characters (\\x80-\\x9F)', () => {
    expect(sanitizeString('hello\x80\x8F\x9Fworld')).toBe('helloworld');
  });

  it('preserves carriage return \\r', () => {
    // \\r is \\x0D which is in the preserved range (not stripped)
    expect(sanitizeString('hello\rworld')).toBe('hello\rworld');
  });

  it('preserves newline \\n', () => {
    expect(sanitizeString('line1\nline2')).toBe('line1\nline2');
  });

  it('preserves tab \\t', () => {
    expect(sanitizeString('col1\tcol2')).toBe('col1\tcol2');
  });

  it('handles a string that is only control characters', () => {
    expect(sanitizeString('\x00\x01\x02\x03')).toBe('');
  });

  it('trims whitespace before measuring max length', () => {
    // '  hello  ' trimmed to 'hello' (5 chars) — fits in maxLength=5
    expect(sanitizeString('  hello  ', 5)).toBe('hello');
  });

  it('truncates after trimming and removing control chars', () => {
    // '\x00abc\x00def\x00' => 'abcdef' after control char removal, then truncate to 3
    expect(sanitizeString('\x00abc\x00def\x00', 3)).toBe('abc');
  });

  it('handles max length of 0', () => {
    expect(sanitizeString('hello', 0)).toBe('');
  });

  it('handles max length of 1', () => {
    expect(sanitizeString('hello', 1)).toBe('h');
  });

  it('preserves unicode characters (non-control)', () => {
    expect(sanitizeString('café ñoño')).toBe('café ñoño');
  });

  it('preserves emoji', () => {
    expect(sanitizeString('robot 🤖')).toBe('robot 🤖');
  });

  it('handles a very long string without hanging', () => {
    const longStr = 'a'.repeat(1_000_000);
    const result = sanitizeString(longStr);
    expect(result.length).toBe(1000); // default max length
  });

  it('handles a very long string with a large custom max', () => {
    const longStr = 'b'.repeat(50_000);
    const result = sanitizeString(longStr, 50_000);
    expect(result.length).toBe(50_000);
  });

  it('does not alter a clean string within limits', () => {
    const clean = 'This is a perfectly normal string.';
    expect(sanitizeString(clean)).toBe(clean);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// 7. Cross-cutting: Consistency across name validators
// ─────────────────────────────────────────────────────────────────────────────

describe('Cross-validator consistency', () => {
  const invalidNames = [
    '',
    'no_slash',
    '/a//b',
    '/trailing/',
    '/with spaces',
    '/special!',
    '/1starts_with_digit',
  ];

  for (const name of invalidNames) {
    it(`all three validators reject "${name}"`, () => {
      expect(validateTopicName(name).valid).toBe(false);
      expect(validateServiceName(name).valid).toBe(false);
      expect(validateActionName(name).valid).toBe(false);
    });
  }

  const validNames = [
    '/',
    '/cmd_vel',
    '/robot_1/cmd_vel',
    '/_hidden',
    '/a/b/c/d/e',
  ];

  for (const name of validNames) {
    it(`all three validators accept "${name}"`, () => {
      expect(validateTopicName(name).valid).toBe(true);
      expect(validateServiceName(name).valid).toBe(true);
      expect(validateActionName(name).valid).toBe(true);
    });
  }

  it('each validator labels errors with the correct kind string', () => {
    const topicErr = validateTopicName('bad');
    const serviceErr = validateServiceName('bad');
    const actionErr = validateActionName('bad');

    expect(topicErr.error).toContain('Topic');
    expect(topicErr.error).not.toContain('Service');
    expect(topicErr.error).not.toContain('Action');

    expect(serviceErr.error).toContain('Service');
    expect(serviceErr.error).not.toContain('Topic');
    expect(serviceErr.error).not.toContain('Action');

    expect(actionErr.error).toContain('Action');
    expect(actionErr.error).not.toContain('Topic');
    expect(actionErr.error).not.toContain('Service');
  });
});

import { describe, it, expect } from 'vitest';
import {
  PhysicalMcpError,
  BridgeConnectionError,
  BridgeTimeoutError,
  SafetyViolationError,
  EmergencyStopError,
  ValidationError,
  UnknownToolError,
  RateLimitError,
  GeofenceViolationError,
  PolicyError,
} from './errors.js';

describe('PhysicalMcpError', () => {
  it('should set name, message, and code', () => {
    const error = new PhysicalMcpError('something broke', 'TEST_CODE');
    expect(error.name).toBe('PhysicalMcpError');
    expect(error.message).toBe('something broke');
    expect(error.code).toBe('TEST_CODE');
  });

  it('should be an instance of Error and PhysicalMcpError', () => {
    const error = new PhysicalMcpError('test', 'CODE');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
  });

  it('should have a stack trace', () => {
    const error = new PhysicalMcpError('test', 'CODE');
    expect(error.stack).toBeDefined();
    expect(error.stack).toContain('PhysicalMcpError');
  });

  it('should be throwable and catchable', () => {
    expect(() => {
      throw new PhysicalMcpError('thrown', 'THROWN');
    }).toThrow(PhysicalMcpError);
  });
});

describe('BridgeConnectionError', () => {
  it('should set name, message, and code', () => {
    const error = new BridgeConnectionError('connection refused');
    expect(error.name).toBe('BridgeConnectionError');
    expect(error.message).toBe('connection refused');
    expect(error.code).toBe('BRIDGE_CONNECTION_ERROR');
  });

  it('should be an instance of Error, PhysicalMcpError, and BridgeConnectionError', () => {
    const error = new BridgeConnectionError('fail');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(BridgeConnectionError);
  });

  it('should preserve bridgeUrl when provided', () => {
    const error = new BridgeConnectionError('fail', 'ws://localhost:9090');
    expect(error.bridgeUrl).toBe('ws://localhost:9090');
  });

  it('should allow bridgeUrl to be undefined', () => {
    const error = new BridgeConnectionError('fail');
    expect(error.bridgeUrl).toBeUndefined();
  });

  it('should have a stack trace', () => {
    const error = new BridgeConnectionError('fail');
    expect(error.stack).toBeDefined();
  });

  it('should be throwable and catchable as PhysicalMcpError', () => {
    try {
      throw new BridgeConnectionError('down', 'ws://host:9090');
    } catch (e) {
      expect(e).toBeInstanceOf(PhysicalMcpError);
      expect(e).toBeInstanceOf(BridgeConnectionError);
      expect((e as BridgeConnectionError).bridgeUrl).toBe('ws://host:9090');
    }
  });
});

describe('BridgeTimeoutError', () => {
  it('should set name, message, and code', () => {
    const error = new BridgeTimeoutError('timed out');
    expect(error.name).toBe('BridgeTimeoutError');
    expect(error.message).toBe('timed out');
    expect(error.code).toBe('BRIDGE_TIMEOUT');
  });

  it('should be an instance of Error, PhysicalMcpError, and BridgeTimeoutError', () => {
    const error = new BridgeTimeoutError('timeout');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(BridgeTimeoutError);
  });

  it('should preserve timeoutMs when provided', () => {
    const error = new BridgeTimeoutError('timed out after 5000ms', 5000);
    expect(error.timeoutMs).toBe(5000);
  });

  it('should allow timeoutMs to be undefined', () => {
    const error = new BridgeTimeoutError('timed out');
    expect(error.timeoutMs).toBeUndefined();
  });

  it('should have a stack trace', () => {
    const error = new BridgeTimeoutError('timeout');
    expect(error.stack).toBeDefined();
  });
});

describe('SafetyViolationError', () => {
  const violations = [
    { type: 'velocity', message: 'exceeds max linear velocity' },
    { type: 'geofence', message: 'outside boundary' },
  ];

  it('should set name, message, and code', () => {
    const error = new SafetyViolationError('blocked', violations);
    expect(error.name).toBe('SafetyViolationError');
    expect(error.message).toBe('blocked');
    expect(error.code).toBe('SAFETY_VIOLATION');
  });

  it('should be an instance of Error, PhysicalMcpError, and SafetyViolationError', () => {
    const error = new SafetyViolationError('blocked', violations);
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(SafetyViolationError);
  });

  it('should preserve violations array', () => {
    const error = new SafetyViolationError('blocked', violations);
    expect(error.violations).toEqual(violations);
    expect(error.violations).toHaveLength(2);
    expect(error.violations[0].type).toBe('velocity');
    expect(error.violations[1].type).toBe('geofence');
  });

  it('should work with an empty violations array', () => {
    const error = new SafetyViolationError('blocked', []);
    expect(error.violations).toEqual([]);
    expect(error.violations).toHaveLength(0);
  });

  it('should have a stack trace', () => {
    const error = new SafetyViolationError('blocked', violations);
    expect(error.stack).toBeDefined();
  });
});

describe('EmergencyStopError', () => {
  it('should use default message when none is provided', () => {
    const error = new EmergencyStopError();
    expect(error.name).toBe('EmergencyStopError');
    expect(error.message).toBe('Emergency stop is active. All commands are blocked.');
    expect(error.code).toBe('EMERGENCY_STOP_ACTIVE');
  });

  it('should allow a custom message', () => {
    const error = new EmergencyStopError('custom e-stop message');
    expect(error.message).toBe('custom e-stop message');
    expect(error.code).toBe('EMERGENCY_STOP_ACTIVE');
  });

  it('should be an instance of Error, PhysicalMcpError, and EmergencyStopError', () => {
    const error = new EmergencyStopError();
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(EmergencyStopError);
  });

  it('should have a stack trace', () => {
    const error = new EmergencyStopError();
    expect(error.stack).toBeDefined();
  });

  it('should be throwable and catchable', () => {
    expect(() => {
      throw new EmergencyStopError();
    }).toThrow(EmergencyStopError);

    expect(() => {
      throw new EmergencyStopError();
    }).toThrow('Emergency stop is active. All commands are blocked.');
  });
});

describe('ValidationError', () => {
  it('should set name, message, and code', () => {
    const error = new ValidationError('invalid input');
    expect(error.name).toBe('ValidationError');
    expect(error.message).toBe('invalid input');
    expect(error.code).toBe('VALIDATION_ERROR');
  });

  it('should be an instance of Error, PhysicalMcpError, and ValidationError', () => {
    const error = new ValidationError('bad');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(ValidationError);
  });

  it('should preserve field when provided', () => {
    const error = new ValidationError('required', 'topic_name');
    expect(error.field).toBe('topic_name');
  });

  it('should allow field to be undefined', () => {
    const error = new ValidationError('bad input');
    expect(error.field).toBeUndefined();
  });

  it('should have a stack trace', () => {
    const error = new ValidationError('bad');
    expect(error.stack).toBeDefined();
  });
});

describe('UnknownToolError', () => {
  it('should format message from tool name', () => {
    const error = new UnknownToolError('ros2_fly');
    expect(error.name).toBe('UnknownToolError');
    expect(error.message).toBe('Unknown tool: ros2_fly');
    expect(error.code).toBe('UNKNOWN_TOOL');
  });

  it('should be an instance of Error, PhysicalMcpError, and UnknownToolError', () => {
    const error = new UnknownToolError('nope');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(UnknownToolError);
  });

  it('should have a stack trace', () => {
    const error = new UnknownToolError('nope');
    expect(error.stack).toBeDefined();
  });

  it('should be throwable and catchable', () => {
    expect(() => {
      throw new UnknownToolError('fake_tool');
    }).toThrow('Unknown tool: fake_tool');
  });
});

describe('RateLimitError', () => {
  it('should set name, message, and code', () => {
    const error = new RateLimitError('too many requests');
    expect(error.name).toBe('RateLimitError');
    expect(error.message).toBe('too many requests');
    expect(error.code).toBe('RATE_LIMIT_EXCEEDED');
  });

  it('should be an instance of Error, PhysicalMcpError, and RateLimitError', () => {
    const error = new RateLimitError('limit hit');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(RateLimitError);
  });

  it('should preserve key when provided', () => {
    const error = new RateLimitError('rate limited', '/cmd_vel');
    expect(error.key).toBe('/cmd_vel');
  });

  it('should allow key to be undefined', () => {
    const error = new RateLimitError('rate limited');
    expect(error.key).toBeUndefined();
  });

  it('should have a stack trace', () => {
    const error = new RateLimitError('limited');
    expect(error.stack).toBeDefined();
  });
});

describe('GeofenceViolationError', () => {
  const position = { x: 10.5, y: -3.2, z: 0 };

  it('should set name, message, and code', () => {
    const error = new GeofenceViolationError('outside boundary');
    expect(error.name).toBe('GeofenceViolationError');
    expect(error.message).toBe('outside boundary');
    expect(error.code).toBe('GEOFENCE_VIOLATION');
  });

  it('should be an instance of Error, PhysicalMcpError, and GeofenceViolationError', () => {
    const error = new GeofenceViolationError('out of bounds');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(GeofenceViolationError);
  });

  it('should preserve position when provided', () => {
    const error = new GeofenceViolationError('out of bounds', position);
    expect(error.position).toEqual(position);
    expect(error.position?.x).toBe(10.5);
    expect(error.position?.y).toBe(-3.2);
    expect(error.position?.z).toBe(0);
  });

  it('should allow position to be undefined', () => {
    const error = new GeofenceViolationError('out');
    expect(error.position).toBeUndefined();
  });

  it('should have a stack trace', () => {
    const error = new GeofenceViolationError('out');
    expect(error.stack).toBeDefined();
  });
});

describe('PolicyError', () => {
  it('should set name, message, and code', () => {
    const error = new PolicyError('invalid policy config');
    expect(error.name).toBe('PolicyError');
    expect(error.message).toBe('invalid policy config');
    expect(error.code).toBe('POLICY_ERROR');
  });

  it('should be an instance of Error, PhysicalMcpError, and PolicyError', () => {
    const error = new PolicyError('bad policy');
    expect(error).toBeInstanceOf(Error);
    expect(error).toBeInstanceOf(PhysicalMcpError);
    expect(error).toBeInstanceOf(PolicyError);
  });

  it('should have a stack trace', () => {
    const error = new PolicyError('bad policy');
    expect(error.stack).toBeDefined();
  });

  it('should be throwable and catchable', () => {
    expect(() => {
      throw new PolicyError('missing required field');
    }).toThrow(PolicyError);
  });
});

describe('inheritance chain', () => {
  it('all errors should be catchable as PhysicalMcpError', () => {
    const errors: PhysicalMcpError[] = [
      new BridgeConnectionError('conn'),
      new BridgeTimeoutError('timeout'),
      new SafetyViolationError('safety', []),
      new EmergencyStopError(),
      new ValidationError('validation'),
      new UnknownToolError('tool'),
      new RateLimitError('rate'),
      new GeofenceViolationError('geo'),
      new PolicyError('policy'),
    ];

    for (const error of errors) {
      expect(error).toBeInstanceOf(Error);
      expect(error).toBeInstanceOf(PhysicalMcpError);
      expect(typeof error.code).toBe('string');
      expect(error.code.length).toBeGreaterThan(0);
      expect(error.stack).toBeDefined();
    }
  });

  it('catching PhysicalMcpError catches all subtypes', () => {
    const caughtCodes: string[] = [];

    const throwers = [
      () => { throw new BridgeConnectionError('a'); },
      () => { throw new BridgeTimeoutError('b'); },
      () => { throw new SafetyViolationError('c', []); },
      () => { throw new EmergencyStopError(); },
      () => { throw new ValidationError('e'); },
      () => { throw new UnknownToolError('f'); },
      () => { throw new RateLimitError('g'); },
      () => { throw new GeofenceViolationError('h'); },
      () => { throw new PolicyError('i'); },
    ];

    for (const thrower of throwers) {
      try {
        thrower();
      } catch (e) {
        if (e instanceof PhysicalMcpError) {
          caughtCodes.push(e.code);
        }
      }
    }

    expect(caughtCodes).toHaveLength(9);
    expect(caughtCodes).toContain('BRIDGE_CONNECTION_ERROR');
    expect(caughtCodes).toContain('BRIDGE_TIMEOUT');
    expect(caughtCodes).toContain('SAFETY_VIOLATION');
    expect(caughtCodes).toContain('EMERGENCY_STOP_ACTIVE');
    expect(caughtCodes).toContain('VALIDATION_ERROR');
    expect(caughtCodes).toContain('UNKNOWN_TOOL');
    expect(caughtCodes).toContain('RATE_LIMIT_EXCEEDED');
    expect(caughtCodes).toContain('GEOFENCE_VIOLATION');
    expect(caughtCodes).toContain('POLICY_ERROR');
  });
});

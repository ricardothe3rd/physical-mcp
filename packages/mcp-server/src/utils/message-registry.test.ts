import { describe, it, expect, beforeEach } from 'vitest';
import { MessageRegistry, MessageTypeDefinition } from './message-registry.js';

describe('MessageRegistry', () => {
  let registry: MessageRegistry;

  beforeEach(() => {
    registry = new MessageRegistry();
  });

  // --- Constructor / Built-in types ---

  describe('constructor', () => {
    it('registers all 10 built-in types', () => {
      const builtins = registry.getBuiltinTypes();
      expect(builtins).toHaveLength(10);
      expect(builtins).toContain('geometry_msgs/msg/Twist');
      expect(builtins).toContain('geometry_msgs/msg/Pose');
      expect(builtins).toContain('geometry_msgs/msg/Point');
      expect(builtins).toContain('geometry_msgs/msg/Quaternion');
      expect(builtins).toContain('geometry_msgs/msg/Vector3');
      expect(builtins).toContain('std_msgs/msg/String');
      expect(builtins).toContain('std_msgs/msg/Bool');
      expect(builtins).toContain('std_msgs/msg/Int32');
      expect(builtins).toContain('std_msgs/msg/Float64');
      expect(builtins).toContain('std_msgs/msg/Header');
    });
  });

  // --- register() ---

  describe('register()', () => {
    it('adds a new custom type', () => {
      const customType: MessageTypeDefinition = {
        typeName: 'my_pkg/msg/Custom',
        fields: [{ name: 'value', type: 'number', required: true }],
        description: 'A custom message',
      };

      registry.register(customType);
      expect(registry.has('my_pkg/msg/Custom')).toBe(true);
      expect(registry.get('my_pkg/msg/Custom')).toEqual(customType);
    });

    it('overwrites existing type with same name', () => {
      const original: MessageTypeDefinition = {
        typeName: 'my_pkg/msg/Overwrite',
        fields: [{ name: 'a', type: 'string', required: true }],
      };
      const replacement: MessageTypeDefinition = {
        typeName: 'my_pkg/msg/Overwrite',
        fields: [{ name: 'b', type: 'number', required: true }],
        description: 'Replaced',
      };

      registry.register(original);
      registry.register(replacement);

      const def = registry.get('my_pkg/msg/Overwrite');
      expect(def).toEqual(replacement);
      expect(def?.fields[0].name).toBe('b');
    });
  });

  // --- unregister() ---

  describe('unregister()', () => {
    it('removes a type and returns true', () => {
      registry.register({
        typeName: 'test/msg/Removable',
        fields: [{ name: 'x', type: 'number' }],
      });

      expect(registry.unregister('test/msg/Removable')).toBe(true);
      expect(registry.has('test/msg/Removable')).toBe(false);
    });

    it('returns false for unknown type', () => {
      expect(registry.unregister('nonexistent/msg/Type')).toBe(false);
    });
  });

  // --- has() ---

  describe('has()', () => {
    it('returns true for registered types', () => {
      expect(registry.has('geometry_msgs/msg/Twist')).toBe(true);
      expect(registry.has('std_msgs/msg/String')).toBe(true);
    });

    it('returns false for unregistered types', () => {
      expect(registry.has('unknown/msg/Type')).toBe(false);
      expect(registry.has('')).toBe(false);
    });
  });

  // --- get() ---

  describe('get()', () => {
    it('returns definition for registered type', () => {
      const def = registry.get('geometry_msgs/msg/Twist');
      expect(def).not.toBeNull();
      expect(def!.typeName).toBe('geometry_msgs/msg/Twist');
      expect(def!.fields).toHaveLength(2);
      expect(def!.fields[0].name).toBe('linear');
      expect(def!.fields[1].name).toBe('angular');
    });

    it('returns null for unregistered type', () => {
      expect(registry.get('unknown/msg/Type')).toBeNull();
    });
  });

  // --- list() ---

  describe('list()', () => {
    it('returns all type names', () => {
      const names = registry.list();
      expect(names).toContain('geometry_msgs/msg/Twist');
      expect(names).toContain('std_msgs/msg/String');
      expect(names).toContain('geometry_msgs/msg/Vector3');
      // Also includes internal helper types
      expect(names.length).toBeGreaterThanOrEqual(10);
    });
  });

  // --- count() ---

  describe('count()', () => {
    it('returns correct count', () => {
      const initialCount = registry.count();
      // Should include the 10 public built-ins plus the internal stamp helper
      expect(initialCount).toBeGreaterThanOrEqual(10);

      registry.register({
        typeName: 'extra/msg/One',
        fields: [{ name: 'val', type: 'number' }],
      });
      expect(registry.count()).toBe(initialCount + 1);
    });
  });

  // --- validate() ---

  describe('validate()', () => {
    it('returns valid:true for valid Twist message', () => {
      const result = registry.validate('geometry_msgs/msg/Twist', {
        linear: { x: 1.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.5 },
      });
      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('returns errors for missing required field', () => {
      const result = registry.validate('geometry_msgs/msg/Twist', {
        linear: { x: 1.0, y: 0.0, z: 0.0 },
        // angular is missing
      });
      expect(result.valid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
      expect(result.errors.some((e) => e.includes('angular'))).toBe(true);
    });

    it('returns errors for wrong field type', () => {
      const result = registry.validate('std_msgs/msg/String', {
        data: 42, // should be string
      });
      expect(result.valid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
      expect(result.errors[0]).toContain('string');
    });

    it('validates nested objects recursively', () => {
      const result = registry.validate('geometry_msgs/msg/Twist', {
        linear: { x: 1.0, y: 'bad', z: 0.0 }, // y should be number
        angular: { x: 0.0, y: 0.0, z: 0.5 },
      });
      expect(result.valid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
      expect(result.errors.some((e) => e.includes('linear.y'))).toBe(true);
    });

    it('allows unknown/extra fields', () => {
      const result = registry.validate('std_msgs/msg/String', {
        data: 'hello',
        extra_field: 42,
        another: true,
      });
      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('returns error for unregistered type', () => {
      const result = registry.validate('unknown/msg/Type', { data: 'test' });
      expect(result.valid).toBe(false);
      expect(result.errors).toHaveLength(1);
      expect(result.errors[0]).toContain('Unknown message type');
    });

    it('handles optional fields correctly', () => {
      registry.register({
        typeName: 'test/msg/WithOptional',
        fields: [
          { name: 'required_field', type: 'string', required: true },
          { name: 'optional_field', type: 'number', required: false },
        ],
      });

      // Missing optional field should be valid
      const result1 = registry.validate('test/msg/WithOptional', {
        required_field: 'hello',
      });
      expect(result1.valid).toBe(true);
      expect(result1.errors).toHaveLength(0);

      // Missing required field should be invalid
      const result2 = registry.validate('test/msg/WithOptional', {
        optional_field: 42,
      });
      expect(result2.valid).toBe(false);
      expect(result2.errors.some((e) => e.includes('required_field'))).toBe(true);

      // Providing both should be valid
      const result3 = registry.validate('test/msg/WithOptional', {
        required_field: 'hello',
        optional_field: 42,
      });
      expect(result3.valid).toBe(true);
      expect(result3.errors).toHaveLength(0);
    });

    it('collects all errors, not just the first', () => {
      const result = registry.validate('geometry_msgs/msg/Twist', {
        // Both linear and angular missing
      });
      expect(result.valid).toBe(false);
      expect(result.errors.length).toBe(2);
      expect(result.errors.some((e) => e.includes('linear'))).toBe(true);
      expect(result.errors.some((e) => e.includes('angular'))).toBe(true);
    });

    it('validates deeply nested structures', () => {
      const result = registry.validate('geometry_msgs/msg/Pose', {
        position: { x: 1.0, y: 2.0, z: 3.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
      });
      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('returns errors for wrong type in nested object fields', () => {
      const result = registry.validate('geometry_msgs/msg/Twist', {
        linear: 'not an object',
        angular: { x: 0.0, y: 0.0, z: 0.5 },
      });
      expect(result.valid).toBe(false);
      expect(result.errors.some((e) => e.includes('linear') && e.includes('object'))).toBe(true);
    });

    it('validates Bool message type', () => {
      expect(registry.validate('std_msgs/msg/Bool', { data: true }).valid).toBe(true);
      expect(registry.validate('std_msgs/msg/Bool', { data: 'true' }).valid).toBe(false);
    });

    it('validates Header message type with nested stamp', () => {
      const result = registry.validate('std_msgs/msg/Header', {
        stamp: { sec: 1234, nanosec: 567000000 },
        frame_id: 'base_link',
      });
      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });
  });

  // --- reset() ---

  describe('reset()', () => {
    it('restores only built-in types', () => {
      // Add a custom type
      registry.register({
        typeName: 'custom/msg/Temp',
        fields: [{ name: 'x', type: 'number' }],
      });
      expect(registry.has('custom/msg/Temp')).toBe(true);

      // Also remove a built-in type
      registry.unregister('std_msgs/msg/String');
      expect(registry.has('std_msgs/msg/String')).toBe(false);

      // Reset
      registry.reset();

      // Custom type should be gone
      expect(registry.has('custom/msg/Temp')).toBe(false);

      // Built-in types should be restored
      expect(registry.has('std_msgs/msg/String')).toBe(true);
      expect(registry.has('geometry_msgs/msg/Twist')).toBe(true);

      // All 10 built-in types present
      const builtins = registry.getBuiltinTypes();
      expect(builtins).toHaveLength(10);
    });
  });

  // --- getBuiltinTypes() ---

  describe('getBuiltinTypes()', () => {
    it('returns the 10 built-in type names', () => {
      const builtins = registry.getBuiltinTypes();
      expect(builtins).toHaveLength(10);

      const expected = [
        'geometry_msgs/msg/Twist',
        'geometry_msgs/msg/Pose',
        'geometry_msgs/msg/Point',
        'geometry_msgs/msg/Quaternion',
        'geometry_msgs/msg/Vector3',
        'std_msgs/msg/String',
        'std_msgs/msg/Bool',
        'std_msgs/msg/Int32',
        'std_msgs/msg/Float64',
        'std_msgs/msg/Header',
      ];

      for (const name of expected) {
        expect(builtins).toContain(name);
      }
    });

    it('returns a copy, not a reference to internal state', () => {
      const builtins1 = registry.getBuiltinTypes();
      const builtins2 = registry.getBuiltinTypes();
      expect(builtins1).toEqual(builtins2);
      expect(builtins1).not.toBe(builtins2);
    });
  });
});

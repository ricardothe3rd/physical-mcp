/**
 * Custom ROS2 message type registry.
 * Allows registration and validation of message type definitions
 * so the system can validate messages before publishing.
 * Comes with built-in definitions for common ROS2 message types.
 */

export interface FieldDefinition {
  name: string;
  type: 'string' | 'number' | 'boolean' | 'object' | 'array';
  required?: boolean;      // Default true
  nestedType?: string;     // For object fields, reference to another message type
  arrayItemType?: string;  // For array fields
}

export interface MessageTypeDefinition {
  typeName: string;        // e.g., 'geometry_msgs/msg/Twist'
  fields: FieldDefinition[];
  description?: string;
}

/**
 * Helper to create number field definitions.
 */
function numField(name: string): FieldDefinition {
  return { name, type: 'number', required: true };
}

/**
 * All built-in ROS2 message type definitions.
 */
function createBuiltinTypes(): MessageTypeDefinition[] {
  return [
    {
      typeName: 'geometry_msgs/msg/Vector3',
      fields: [numField('x'), numField('y'), numField('z')],
      description: 'A 3D vector (x, y, z)',
    },
    {
      typeName: 'geometry_msgs/msg/Point',
      fields: [numField('x'), numField('y'), numField('z')],
      description: 'A point in 3D space (x, y, z)',
    },
    {
      typeName: 'geometry_msgs/msg/Quaternion',
      fields: [numField('x'), numField('y'), numField('z'), numField('w')],
      description: 'A quaternion orientation (x, y, z, w)',
    },
    {
      typeName: 'geometry_msgs/msg/Twist',
      fields: [
        { name: 'linear', type: 'object', required: true, nestedType: 'geometry_msgs/msg/Vector3' },
        { name: 'angular', type: 'object', required: true, nestedType: 'geometry_msgs/msg/Vector3' },
      ],
      description: 'Linear and angular velocity in 3D',
    },
    {
      typeName: 'geometry_msgs/msg/Pose',
      fields: [
        { name: 'position', type: 'object', required: true, nestedType: 'geometry_msgs/msg/Point' },
        { name: 'orientation', type: 'object', required: true, nestedType: 'geometry_msgs/msg/Quaternion' },
      ],
      description: 'A position and orientation in 3D space',
    },
    {
      typeName: 'std_msgs/msg/String',
      fields: [{ name: 'data', type: 'string', required: true }],
      description: 'A single string value',
    },
    {
      typeName: 'std_msgs/msg/Bool',
      fields: [{ name: 'data', type: 'boolean', required: true }],
      description: 'A single boolean value',
    },
    {
      typeName: 'std_msgs/msg/Int32',
      fields: [{ name: 'data', type: 'number', required: true }],
      description: 'A single 32-bit integer value',
    },
    {
      typeName: 'std_msgs/msg/Float64',
      fields: [{ name: 'data', type: 'number', required: true }],
      description: 'A single 64-bit float value',
    },
    {
      typeName: 'std_msgs/msg/Header',
      fields: [
        { name: 'stamp', type: 'object', required: true, nestedType: 'std_msgs/msg/Header__Stamp' },
        { name: 'frame_id', type: 'string', required: true },
      ],
      description: 'Standard message header with timestamp and frame ID',
    },
    // Internal helper type for Header.stamp (sec + nanosec)
    {
      typeName: 'std_msgs/msg/Header__Stamp',
      fields: [numField('sec'), numField('nanosec')],
      description: 'Timestamp with seconds and nanoseconds',
    },
  ];
}

export class MessageRegistry {
  private types: Map<string, MessageTypeDefinition>;
  private builtinTypeNames: string[];

  constructor() {
    this.types = new Map();
    const builtins = createBuiltinTypes();
    this.builtinTypeNames = builtins
      .filter((t) => !t.typeName.includes('__'))
      .map((t) => t.typeName);
    for (const def of builtins) {
      this.types.set(def.typeName, def);
    }
  }

  /**
   * Register a custom message type definition.
   * Overwrites any existing definition with the same typeName.
   */
  register(definition: MessageTypeDefinition): void {
    this.types.set(definition.typeName, definition);
  }

  /**
   * Unregister a message type by name.
   * Returns true if the type was found and removed, false otherwise.
   */
  unregister(typeName: string): boolean {
    return this.types.delete(typeName);
  }

  /**
   * Check if a type is registered.
   */
  has(typeName: string): boolean {
    return this.types.has(typeName);
  }

  /**
   * Get a type definition by name.
   * Returns null if the type is not registered.
   */
  get(typeName: string): MessageTypeDefinition | null {
    return this.types.get(typeName) ?? null;
  }

  /**
   * List all registered type names.
   */
  list(): string[] {
    return Array.from(this.types.keys());
  }

  /**
   * Validate a message against its type definition.
   * Returns all errors found, not just the first.
   * Unknown/extra fields are allowed.
   */
  validate(
    typeName: string,
    data: Record<string, unknown>,
  ): { valid: boolean; errors: string[] } {
    const errors: string[] = [];
    const definition = this.types.get(typeName);

    if (!definition) {
      return { valid: false, errors: [`Unknown message type '${typeName}'`] };
    }

    this.validateFields(definition, data, '', errors);

    return { valid: errors.length === 0, errors };
  }

  /**
   * Get the total count of registered types.
   */
  count(): number {
    return this.types.size;
  }

  /**
   * Reset the registry to only built-in types.
   */
  reset(): void {
    this.types.clear();
    for (const def of createBuiltinTypes()) {
      this.types.set(def.typeName, def);
    }
  }

  /**
   * Get the list of built-in type names (excludes internal helper types).
   */
  getBuiltinTypes(): string[] {
    return [...this.builtinTypeNames];
  }

  /**
   * Recursively validate fields against a type definition.
   */
  private validateFields(
    definition: MessageTypeDefinition,
    data: Record<string, unknown>,
    path: string,
    errors: string[],
  ): void {
    for (const field of definition.fields) {
      const fieldPath = path ? `${path}.${field.name}` : field.name;
      const isRequired = field.required !== false; // Default to true
      const value = data[field.name];

      // Check required field presence
      if (value === undefined || value === null) {
        if (isRequired) {
          errors.push(`Missing required field '${fieldPath}'`);
        }
        continue;
      }

      // Check field type
      if (!this.checkFieldType(field, value, fieldPath, errors)) {
        continue;
      }

      // Recursive validation for nested object types
      if (field.type === 'object' && field.nestedType) {
        const nestedDef = this.types.get(field.nestedType);
        if (nestedDef) {
          this.validateFields(
            nestedDef,
            value as Record<string, unknown>,
            fieldPath,
            errors,
          );
        }
      }
    }
  }

  /**
   * Check that a field value matches its expected type.
   * Returns true if the type matches (or the value is allowed), false otherwise.
   */
  private checkFieldType(
    field: FieldDefinition,
    value: unknown,
    fieldPath: string,
    errors: string[],
  ): boolean {
    switch (field.type) {
      case 'string':
        if (typeof value !== 'string') {
          errors.push(
            `Field '${fieldPath}' expected type 'string', got '${typeof value}'`,
          );
          return false;
        }
        return true;

      case 'number':
        if (typeof value !== 'number') {
          errors.push(
            `Field '${fieldPath}' expected type 'number', got '${typeof value}'`,
          );
          return false;
        }
        return true;

      case 'boolean':
        if (typeof value !== 'boolean') {
          errors.push(
            `Field '${fieldPath}' expected type 'boolean', got '${typeof value}'`,
          );
          return false;
        }
        return true;

      case 'object':
        if (typeof value !== 'object' || value === null || Array.isArray(value)) {
          errors.push(
            `Field '${fieldPath}' expected type 'object', got '${Array.isArray(value) ? 'array' : typeof value}'`,
          );
          return false;
        }
        return true;

      case 'array':
        if (!Array.isArray(value)) {
          errors.push(
            `Field '${fieldPath}' expected type 'array', got '${typeof value}'`,
          );
          return false;
        }
        return true;

      default:
        return true;
    }
  }
}

"""ROS2 message type validation.

Provides lightweight, offline validation of message fields against known
ROS2 message schemas.  This lets the bridge reject obviously invalid
publish requests *before* they reach rclpy, producing clear error messages
instead of cryptic serialisation failures.
"""

from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# ValidationResult
# ---------------------------------------------------------------------------

@dataclass
class ValidationResult:
    """Outcome of a message-field validation check."""

    valid: bool = True
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Known message schemas
# ---------------------------------------------------------------------------

_VECTOR3_FIELDS: dict = {"x": float, "y": float, "z": float}

KNOWN_MESSAGE_TYPES: dict[str, dict] = {
    "geometry_msgs/msg/Twist": {
        "linear": {"x": float, "y": float, "z": float},
        "angular": {"x": float, "y": float, "z": float},
    },
    "geometry_msgs/msg/Vector3": _VECTOR3_FIELDS,
    "std_msgs/msg/String": {"data": str},
    "std_msgs/msg/Bool": {"data": bool},
    "std_msgs/msg/Int32": {"data": int},
    "std_msgs/msg/Float64": {"data": float},
    "sensor_msgs/msg/LaserScan": {
        "ranges": list,
        "angle_min": float,
        "angle_max": float,
    },
    "nav_msgs/msg/Odometry": {
        "pose": dict,
        "twist": dict,
    },
}


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------

def is_known_type(msg_type: str) -> bool:
    """Return ``True`` if *msg_type* has a registered schema."""
    return msg_type in KNOWN_MESSAGE_TYPES


def get_type_fields(msg_type: str) -> dict | None:
    """Return the field schema for *msg_type*, or ``None`` if unknown."""
    return KNOWN_MESSAGE_TYPES.get(msg_type)


# ---------------------------------------------------------------------------
# Core validation
# ---------------------------------------------------------------------------

def _check_type(value, expected_type) -> bool:
    """Check whether *value* matches *expected_type*.

    ``int`` values are accepted where ``float`` is expected (standard
    Python numeric promotion).
    """
    if expected_type is float:
        return isinstance(value, (int, float))
    return isinstance(value, expected_type)


def _validate_fields(
    fields: dict,
    schema: dict,
    path: str,
    result: ValidationResult,
) -> None:
    """Recursively validate *fields* against *schema*, mutating *result*."""
    for key, value in fields.items():
        full_key = f"{path}.{key}" if path else key

        if key not in schema:
            result.warnings.append(f"Unknown field '{full_key}'")
            continue

        expected = schema[key]

        # Nested sub-message (schema value is a dict of further fields)
        if isinstance(expected, dict):
            if not isinstance(value, dict):
                result.valid = False
                result.errors.append(
                    f"Field '{full_key}' expected dict, got {type(value).__name__}"
                )
            else:
                _validate_fields(value, expected, full_key, result)
            continue

        # Leaf field -- expected is a type
        if not _check_type(value, expected):
            result.valid = False
            result.errors.append(
                f"Field '{full_key}' expected {expected.__name__}, "
                f"got {type(value).__name__}"
            )


def validate_message_fields(msg_type: str, fields: dict) -> ValidationResult:
    """Validate *fields* against the schema for *msg_type*.

    * Wrong-type values produce **errors** (``valid=False``).
    * Extra / unknown field names produce **warnings** (``valid`` stays True).
    * An unrecognised *msg_type* returns ``valid=True`` with a warning --
      we cannot reject messages whose schema we simply do not know.
    """
    result = ValidationResult()

    schema = KNOWN_MESSAGE_TYPES.get(msg_type)
    if schema is None:
        result.warnings.append(f"Unknown message type '{msg_type}'")
        return result

    _validate_fields(fields, schema, "", result)
    return result

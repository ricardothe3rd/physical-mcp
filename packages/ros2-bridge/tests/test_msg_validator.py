"""Tests for the ROS2 message type validator."""

import pytest

from physical_mcp_bridge.msg_validator import (
    KNOWN_MESSAGE_TYPES,
    ValidationResult,
    get_type_fields,
    is_known_type,
    validate_message_fields,
)


# ---------------------------------------------------------------------------
# ValidationResult defaults
# ---------------------------------------------------------------------------

class TestValidationResult:
    """ValidationResult dataclass basics."""

    def test_defaults(self):
        r = ValidationResult()
        assert r.valid is True
        assert r.errors == []
        assert r.warnings == []

    def test_independent_lists(self):
        """Each instance should have its own error/warning lists."""
        a = ValidationResult()
        b = ValidationResult()
        a.errors.append("oops")
        assert b.errors == []


# ---------------------------------------------------------------------------
# is_known_type
# ---------------------------------------------------------------------------

class TestIsKnownType:

    @pytest.mark.parametrize("msg_type", [
        "geometry_msgs/msg/Twist",
        "std_msgs/msg/String",
        "sensor_msgs/msg/LaserScan",
        "nav_msgs/msg/Odometry",
    ])
    def test_known_types_return_true(self, msg_type):
        assert is_known_type(msg_type) is True

    @pytest.mark.parametrize("msg_type", [
        "custom_msgs/msg/Foo",
        "nonexistent",
        "",
    ])
    def test_unknown_types_return_false(self, msg_type):
        assert is_known_type(msg_type) is False


# ---------------------------------------------------------------------------
# get_type_fields
# ---------------------------------------------------------------------------

class TestGetTypeFields:

    def test_returns_schema_for_known_type(self):
        schema = get_type_fields("geometry_msgs/msg/Twist")
        assert schema is not None
        assert "linear" in schema
        assert "angular" in schema

    def test_returns_none_for_unknown_type(self):
        assert get_type_fields("custom_msgs/msg/Foo") is None


# ---------------------------------------------------------------------------
# validate_message_fields
# ---------------------------------------------------------------------------

class TestValidateMessageFields:

    # -- valid messages -----------------------------------------------------

    def test_valid_twist_message(self):
        result = validate_message_fields(
            "geometry_msgs/msg/Twist",
            {"linear": {"x": 0.5, "y": 0.0, "z": 0.0},
             "angular": {"x": 0.0, "y": 0.0, "z": 1.0}},
        )
        assert result.valid is True
        assert result.errors == []
        assert result.warnings == []

    def test_valid_string_message(self):
        result = validate_message_fields(
            "std_msgs/msg/String",
            {"data": "hello"},
        )
        assert result.valid is True
        assert result.errors == []

    def test_valid_bool_message(self):
        result = validate_message_fields(
            "std_msgs/msg/Bool",
            {"data": True},
        )
        assert result.valid is True
        assert result.errors == []

    def test_valid_int32_message(self):
        result = validate_message_fields(
            "std_msgs/msg/Int32",
            {"data": 42},
        )
        assert result.valid is True
        assert result.errors == []

    def test_valid_float64_message(self):
        result = validate_message_fields(
            "std_msgs/msg/Float64",
            {"data": 3.14},
        )
        assert result.valid is True
        assert result.errors == []

    def test_int_accepted_for_float_field(self):
        """Python int is valid where float is expected (numeric promotion)."""
        result = validate_message_fields(
            "geometry_msgs/msg/Vector3",
            {"x": 1, "y": 2, "z": 3},
        )
        assert result.valid is True
        assert result.errors == []

    # -- type errors --------------------------------------------------------

    def test_wrong_field_type_string_for_float(self):
        result = validate_message_fields(
            "geometry_msgs/msg/Vector3",
            {"x": "not_a_number", "y": 0.0, "z": 0.0},
        )
        assert result.valid is False
        assert len(result.errors) == 1
        assert "x" in result.errors[0]
        assert "float" in result.errors[0]

    def test_wrong_field_type_in_nested_twist(self):
        result = validate_message_fields(
            "geometry_msgs/msg/Twist",
            {"linear": {"x": "bad", "y": 0.0, "z": 0.0}},
        )
        assert result.valid is False
        assert any("linear.x" in e for e in result.errors)

    def test_nested_expects_dict_gets_scalar(self):
        result = validate_message_fields(
            "geometry_msgs/msg/Twist",
            {"linear": 42},
        )
        assert result.valid is False
        assert any("linear" in e and "dict" in e for e in result.errors)

    def test_wrong_type_for_bool(self):
        result = validate_message_fields(
            "std_msgs/msg/Bool",
            {"data": "yes"},
        )
        assert result.valid is False
        assert any("data" in e for e in result.errors)

    def test_wrong_type_for_int32(self):
        result = validate_message_fields(
            "std_msgs/msg/Int32",
            {"data": 3.14},
        )
        assert result.valid is False
        assert any("data" in e for e in result.errors)

    # -- unknown fields (warnings, not errors) -----------------------------

    def test_unknown_field_produces_warning(self):
        result = validate_message_fields(
            "geometry_msgs/msg/Vector3",
            {"x": 1.0, "y": 2.0, "z": 3.0, "w": 4.0},
        )
        assert result.valid is True
        assert len(result.warnings) == 1
        assert "w" in result.warnings[0]

    def test_unknown_nested_field_produces_warning(self):
        result = validate_message_fields(
            "geometry_msgs/msg/Twist",
            {"linear": {"x": 1.0, "y": 0.0, "z": 0.0, "extra": 9.9}},
        )
        assert result.valid is True
        assert any("linear.extra" in w for w in result.warnings)

    # -- unknown message type ----------------------------------------------

    def test_unknown_message_type(self):
        result = validate_message_fields(
            "custom_msgs/msg/Foo",
            {"bar": 123},
        )
        assert result.valid is True
        assert len(result.warnings) == 1
        assert "custom_msgs/msg/Foo" in result.warnings[0]

    # -- empty fields ------------------------------------------------------

    def test_empty_fields_are_valid(self):
        result = validate_message_fields(
            "geometry_msgs/msg/Twist",
            {},
        )
        assert result.valid is True
        assert result.errors == []
        assert result.warnings == []

    # -- KNOWN_MESSAGE_TYPES sanity ----------------------------------------

    def test_known_types_dict_has_expected_entries(self):
        assert "geometry_msgs/msg/Twist" in KNOWN_MESSAGE_TYPES
        assert "std_msgs/msg/String" in KNOWN_MESSAGE_TYPES
        assert "sensor_msgs/msg/LaserScan" in KNOWN_MESSAGE_TYPES
        assert "nav_msgs/msg/Odometry" in KNOWN_MESSAGE_TYPES

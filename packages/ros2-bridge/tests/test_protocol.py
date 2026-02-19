"""Tests for the WebSocket protocol module."""

import json
import time
import pytest

from physical_mcp_bridge.protocol import (
    CommandType,
    Command,
    parse_command,
    build_response,
)


class TestCommandType:
    """Tests for CommandType enum."""

    def test_has_all_19_types(self):
        assert len(CommandType) == 19

    def test_topic_types(self):
        assert CommandType.TOPIC_LIST == "topic.list"
        assert CommandType.TOPIC_INFO == "topic.info"
        assert CommandType.TOPIC_SUBSCRIBE == "topic.subscribe"
        assert CommandType.TOPIC_PUBLISH == "topic.publish"
        assert CommandType.TOPIC_ECHO == "topic.echo"

    def test_service_types(self):
        assert CommandType.SERVICE_LIST == "service.list"
        assert CommandType.SERVICE_INFO == "service.info"
        assert CommandType.SERVICE_CALL == "service.call"

    def test_action_types(self):
        assert CommandType.ACTION_LIST == "action.list"
        assert CommandType.ACTION_SEND_GOAL == "action.send_goal"
        assert CommandType.ACTION_CANCEL == "action.cancel"
        assert CommandType.ACTION_STATUS == "action.status"

    def test_system_types(self):
        assert CommandType.NODE_LIST == "node.list"
        assert CommandType.GET_PARAMS == "params.get"
        assert CommandType.PING == "ping"
        assert CommandType.EMERGENCY_STOP == "emergency_stop"


class TestParseCommand:
    """Tests for parse_command function."""

    def test_parses_valid_command(self):
        raw = json.dumps({"id": "abc-123", "type": "ping", "params": {}})
        cmd = parse_command(raw)
        assert cmd.id == "abc-123"
        assert cmd.type == CommandType.PING
        assert cmd.params == {}

    def test_parses_command_with_params(self):
        raw = json.dumps({
            "id": "def-456",
            "type": "topic.subscribe",
            "params": {"topic": "/odom", "count": 5},
        })
        cmd = parse_command(raw)
        assert cmd.id == "def-456"
        assert cmd.type == CommandType.TOPIC_SUBSCRIBE
        assert cmd.params["topic"] == "/odom"
        assert cmd.params["count"] == 5

    def test_defaults_params_to_empty_dict(self):
        raw = json.dumps({"id": "ghi-789", "type": "topic.list"})
        cmd = parse_command(raw)
        assert cmd.params == {}

    def test_raises_on_invalid_json(self):
        with pytest.raises(json.JSONDecodeError):
            parse_command("not json")

    def test_raises_on_missing_id(self):
        raw = json.dumps({"type": "ping"})
        with pytest.raises(KeyError):
            parse_command(raw)

    def test_raises_on_invalid_type(self):
        raw = json.dumps({"id": "123", "type": "unknown.command"})
        with pytest.raises(ValueError):
            parse_command(raw)


class TestBuildResponse:
    """Tests for build_response function."""

    def test_builds_ok_response(self):
        before = time.time()
        resp = build_response("abc-123", "ok", {"bridge": "ok"})
        after = time.time()

        assert resp["id"] == "abc-123"
        assert resp["status"] == "ok"
        assert resp["data"] == {"bridge": "ok"}
        assert before <= resp["timestamp"] <= after

    def test_builds_error_response(self):
        resp = build_response("abc-123", "error", {"message": "something failed"})
        assert resp["status"] == "error"
        assert resp["data"]["message"] == "something failed"

    def test_handles_none_id(self):
        resp = build_response(None, "ok", {})
        assert resp["id"] is None

    def test_response_is_json_serializable(self):
        resp = build_response("abc", "ok", {"topics": ["/cmd_vel", "/odom"]})
        serialized = json.dumps(resp)
        parsed = json.loads(serialized)
        assert parsed["data"]["topics"] == ["/cmd_vel", "/odom"]

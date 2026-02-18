"""Tests for the BridgeNode command dispatch.

All rclpy, rosidl, and websockets dependencies are mocked so tests run
without ROS2 installed.

Strategy: BridgeNode inherits from the mocked rclpy Node (a MagicMock),
which makes normal instantiation fragile across test runs.  Instead we
build a lightweight stand-in object that carries the same attributes and
bind the real handle_command / _dispatch methods from BridgeNode to it.
This lets us exercise the full dispatch logic without touching rclpy at all.
"""

import asyncio
import json
import sys
import types
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Mock every ROS2 / rosidl / websockets dependency before importing.
# ---------------------------------------------------------------------------
sys.modules.setdefault("rclpy", MagicMock())
sys.modules.setdefault("rclpy.node", MagicMock())
sys.modules.setdefault("rclpy.action", MagicMock())
sys.modules.setdefault("rosidl_runtime_py.utilities", MagicMock())
sys.modules.setdefault("rosidl_runtime_py", MagicMock())
sys.modules.setdefault("websockets", MagicMock())
sys.modules.setdefault("websockets.server", MagicMock())

from physical_mcp_bridge.protocol import CommandType, build_response
from physical_mcp_bridge.bridge_node import BridgeNode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_command(cmd_id: str, cmd_type: str, params: dict | None = None) -> str:
    """Build a raw JSON command string."""
    return json.dumps({"id": cmd_id, "type": cmd_type, "params": params or {}})


def _parse_response(raw: str) -> dict:
    """Parse a JSON response string."""
    return json.loads(raw)


class _FakeBridge:
    """Lightweight stand-in for BridgeNode that carries the same attributes
    and has the real handle_command / _dispatch methods bound to it."""
    pass


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def bridge():
    """Build a fake bridge object with real dispatch logic and mocked handlers."""
    node = _FakeBridge()

    # Bind the real async methods from BridgeNode onto the fake object.
    node.handle_command = types.MethodType(BridgeNode.handle_command, node)
    node._dispatch = types.MethodType(BridgeNode._dispatch, node)

    # Mock every sub-handler that _dispatch touches.
    node.discovery = MagicMock()
    node.topics = MagicMock()
    node.services = MagicMock()
    node.actions = MagicMock()
    node.telemetry = MagicMock()
    node.telemetry.get_status.return_value = {
        "uptime_seconds": 10.0,
        "commands_processed": 1,
        "errors": 0,
        "pid": 12345,
    }
    node.emergency_stop = False

    # Provide a mock logger so error paths don't blow up.
    mock_logger = MagicMock()
    node.get_logger = MagicMock(return_value=mock_logger)

    return node


# ---------------------------------------------------------------------------
# Discovery dispatch
# ---------------------------------------------------------------------------

class TestDiscoveryDispatch:
    """Verify that discovery commands dispatch to DiscoveryHandler."""

    @pytest.mark.asyncio
    async def test_topic_list(self, bridge):
        bridge.discovery.list_topics.return_value = [
            {"name": "/odom", "types": ["nav_msgs/msg/Odometry"]},
        ]
        raw = _make_command("1", "topic.list")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"][0]["name"] == "/odom"
        bridge.discovery.list_topics.assert_called_once()

    @pytest.mark.asyncio
    async def test_topic_info(self, bridge):
        bridge.discovery.get_topic_info.return_value = {
            "name": "/scan", "type": "sensor_msgs/msg/LaserScan",
            "publishers": 1, "subscribers": 0,
        }
        raw = _make_command("2", "topic.info", {"topic": "/scan"})
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["name"] == "/scan"
        bridge.discovery.get_topic_info.assert_called_once_with("/scan")

    @pytest.mark.asyncio
    async def test_service_list(self, bridge):
        bridge.discovery.list_services.return_value = [
            {"name": "/trigger", "types": ["std_srvs/srv/Trigger"]},
        ]
        raw = _make_command("3", "service.list")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert len(resp["data"]) == 1

    @pytest.mark.asyncio
    async def test_action_list(self, bridge):
        bridge.discovery.list_actions.return_value = [{"name": "/navigate"}]
        raw = _make_command("4", "action.list")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"][0]["name"] == "/navigate"

    @pytest.mark.asyncio
    async def test_node_list(self, bridge):
        bridge.discovery.list_nodes.return_value = [
            {"name": "bridge", "namespace": "/"},
        ]
        raw = _make_command("5", "node.list")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"][0]["name"] == "bridge"

    @pytest.mark.asyncio
    async def test_service_info(self, bridge):
        bridge.discovery.get_service_info.return_value = {
            "name": "/trigger", "type": "std_srvs/srv/Trigger",
        }
        raw = _make_command("6", "service.info", {"service": "/trigger"})
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["type"] == "std_srvs/srv/Trigger"
        bridge.discovery.get_service_info.assert_called_once_with("/trigger")


# ---------------------------------------------------------------------------
# Topic dispatch
# ---------------------------------------------------------------------------

class TestTopicDispatch:
    """Verify that topic commands dispatch to TopicHandler."""

    @pytest.mark.asyncio
    async def test_topic_subscribe(self, bridge):
        bridge.topics.subscribe.return_value = [{"data": "msg1"}]
        raw = _make_command("10", "topic.subscribe", {
            "topic": "/chatter",
            "message_type": "std_msgs/msg/String",
            "count": 1,
            "timeout_sec": 2.0,
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        bridge.topics.subscribe.assert_called_once_with(
            "/chatter", "std_msgs/msg/String", 1, 2.0,
        )

    @pytest.mark.asyncio
    async def test_topic_echo(self, bridge):
        bridge.topics.echo.return_value = {"data": "latest"}
        raw = _make_command("11", "topic.echo", {
            "topic": "/odom",
            "message_type": "nav_msgs/msg/Odometry",
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["data"] == "latest"

    @pytest.mark.asyncio
    async def test_topic_publish(self, bridge):
        bridge.topics.publish.return_value = True
        raw = _make_command("12", "topic.publish", {
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/msg/Twist",
            "message": {"linear": {"x": 0.5}},
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"] is True

    @pytest.mark.asyncio
    async def test_topic_publish_blocked_by_estop(self, bridge):
        bridge.emergency_stop = True
        raw = _make_command("13", "topic.publish", {
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/msg/Twist",
            "message": {"linear": {"x": 1.0}},
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert "Emergency stop" in resp["data"]["error"]
        bridge.topics.publish.assert_not_called()

    @pytest.mark.asyncio
    async def test_topic_subscribe_uses_default_count_and_timeout(self, bridge):
        """When count/timeout are omitted, dispatch passes defaults."""
        bridge.topics.subscribe.return_value = [{"data": "msg"}]
        raw = _make_command("14", "topic.subscribe", {
            "topic": "/chatter",
            "message_type": "std_msgs/msg/String",
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        bridge.topics.subscribe.assert_called_once_with(
            "/chatter", "std_msgs/msg/String", 1, 5.0,
        )


# ---------------------------------------------------------------------------
# Service dispatch
# ---------------------------------------------------------------------------

class TestServiceDispatch:
    """Verify that service commands dispatch to ServiceHandler."""

    @pytest.mark.asyncio
    async def test_service_call(self, bridge):
        bridge.services.call.return_value = {"success": True, "message": "done"}
        raw = _make_command("20", "service.call", {
            "service": "/trigger",
            "service_type": "std_srvs/srv/Trigger",
            "args": {},
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["success"] is True

    @pytest.mark.asyncio
    async def test_service_call_blocked_by_estop(self, bridge):
        bridge.emergency_stop = True
        raw = _make_command("21", "service.call", {
            "service": "/trigger",
            "service_type": "std_srvs/srv/Trigger",
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert "Emergency stop" in resp["data"]["error"]
        bridge.services.call.assert_not_called()

    @pytest.mark.asyncio
    async def test_service_call_default_args(self, bridge):
        """When args is omitted, an empty dict should be passed."""
        bridge.services.call.return_value = {"success": True}
        raw = _make_command("22", "service.call", {
            "service": "/trigger",
            "service_type": "std_srvs/srv/Trigger",
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        bridge.services.call.assert_called_once_with("/trigger", "std_srvs/srv/Trigger", {})


# ---------------------------------------------------------------------------
# Action dispatch
# ---------------------------------------------------------------------------

class TestActionDispatch:
    """Verify that action commands dispatch to ActionHandler."""

    @pytest.mark.asyncio
    async def test_action_send_goal(self, bridge):
        bridge.actions.send_goal.return_value = {
            "status": "completed", "goal_id": "g1", "result": {},
        }
        raw = _make_command("30", "action.send_goal", {
            "action": "/navigate",
            "action_type": "nav2_msgs/action/NavigateToPose",
            "goal": {"pose": {}},
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["status"] == "completed"

    @pytest.mark.asyncio
    async def test_action_send_goal_blocked_by_estop(self, bridge):
        bridge.emergency_stop = True
        raw = _make_command("31", "action.send_goal", {
            "action": "/navigate",
            "action_type": "nav2_msgs/action/NavigateToPose",
            "goal": {},
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert "Emergency stop" in resp["data"]["error"]
        bridge.actions.send_goal.assert_not_called()

    @pytest.mark.asyncio
    async def test_action_cancel(self, bridge):
        bridge.actions.cancel_goal.return_value = {
            "status": "cancelled", "goal_id": "g1",
        }
        raw = _make_command("32", "action.cancel", {
            "action": "/navigate",
            "goal_id": "g1",
        })
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["status"] == "cancelled"

    @pytest.mark.asyncio
    async def test_action_cancel_without_goal_id(self, bridge):
        """When goal_id is omitted, None should be passed to cancel_goal."""
        bridge.actions.cancel_goal.return_value = {
            "status": "cancelled_all", "cancelled": [],
        }
        raw = _make_command("34", "action.cancel", {"action": "/navigate"})
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        bridge.actions.cancel_goal.assert_called_once_with("/navigate", None)

    @pytest.mark.asyncio
    async def test_action_status(self, bridge):
        bridge.actions.get_status.return_value = {
            "action": "/nav", "active_goals": [], "count": 0,
        }
        raw = _make_command("33", "action.status", {"action": "/nav"})
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["count"] == 0


# ---------------------------------------------------------------------------
# System commands
# ---------------------------------------------------------------------------

class TestSystemDispatch:
    """Verify system-level commands: ping, emergency_stop, params."""

    @pytest.mark.asyncio
    async def test_ping(self, bridge):
        raw = _make_command("40", "ping")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["pong"] is True
        assert resp["data"]["bridge"] == "physical-mcp-bridge"
        assert resp["data"]["version"] == "0.1.0"
        assert "telemetry" in resp["data"]

    @pytest.mark.asyncio
    async def test_get_params(self, bridge):
        raw = _make_command("41", "params.get")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"] == {}

    @pytest.mark.asyncio
    async def test_emergency_stop(self, bridge):
        bridge.topics.publish.return_value = True
        raw = _make_command("42", "emergency_stop")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["emergency_stop"] is True
        assert bridge.emergency_stop is True
        bridge.actions.cancel_all.assert_called_once()

    @pytest.mark.asyncio
    async def test_emergency_stop_publishes_zero_velocity(self, bridge):
        bridge.topics.publish.return_value = True
        raw = _make_command("43", "emergency_stop")
        await bridge.handle_command(raw)
        bridge.topics.publish.assert_called_once_with(
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
             "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        )

    @pytest.mark.asyncio
    async def test_emergency_stop_survives_publish_failure(self, bridge):
        """Even if zero-velocity publish fails, estop should still succeed."""
        bridge.topics.publish.side_effect = RuntimeError("publisher broken")
        raw = _make_command("44", "emergency_stop")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "ok"
        assert resp["data"]["emergency_stop"] is True
        assert bridge.emergency_stop is True


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------

class TestErrorHandling:
    """Verify error responses for malformed input and handler exceptions."""

    @pytest.mark.asyncio
    async def test_parse_error_returns_error_response(self, bridge):
        resp = _parse_response(await bridge.handle_command("NOT VALID JSON"))
        assert resp["status"] == "error"
        assert "Parse error" in resp["data"]["error"]
        assert resp["id"] is None

    @pytest.mark.asyncio
    async def test_handler_exception_returns_error_response(self, bridge):
        bridge.discovery.list_topics.side_effect = RuntimeError("rclpy exploded")
        raw = _make_command("50", "topic.list")
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "error"
        assert "rclpy exploded" in resp["data"]["error"]

    @pytest.mark.asyncio
    async def test_handler_exception_records_telemetry_error(self, bridge):
        bridge.discovery.list_topics.side_effect = RuntimeError("boom")
        raw = _make_command("51", "topic.list")
        await bridge.handle_command(raw)
        bridge.telemetry.record_error.assert_called_once()

    @pytest.mark.asyncio
    async def test_telemetry_recorded_on_every_command(self, bridge):
        bridge.discovery.list_topics.return_value = []
        raw = _make_command("52", "topic.list")
        await bridge.handle_command(raw)
        bridge.telemetry.record_command.assert_called_once()

    @pytest.mark.asyncio
    async def test_response_always_has_id_status_data_timestamp(self, bridge):
        bridge.discovery.list_topics.return_value = []
        raw = _make_command("60", "topic.list")
        resp = _parse_response(await bridge.handle_command(raw))
        assert "id" in resp
        assert "status" in resp
        assert "data" in resp
        assert "timestamp" in resp
        assert resp["id"] == "60"

    @pytest.mark.asyncio
    async def test_unknown_command_type_in_json(self, bridge):
        """An invalid command type string should produce a parse error."""
        raw = json.dumps({"id": "99", "type": "totally.unknown", "params": {}})
        resp = _parse_response(await bridge.handle_command(raw))
        assert resp["status"] == "error"
        # parse_command raises ValueError before id is extracted
        assert resp["id"] is None

    @pytest.mark.asyncio
    async def test_handler_exception_logs_error(self, bridge):
        bridge.discovery.list_topics.side_effect = RuntimeError("fail")
        raw = _make_command("70", "topic.list")
        await bridge.handle_command(raw)
        bridge.get_logger().error.assert_called_once()

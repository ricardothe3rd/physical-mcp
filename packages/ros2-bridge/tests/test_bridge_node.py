"""Tests for the BridgeNode command dispatch.

All rclpy, rosidl, and websockets dependencies are mocked via conftest.py
so tests run without ROS2 installed.

conftest.py replaces rclpy.node.Node with a real Python class (_FakeNode),
which means BridgeNode is a proper class with real methods that we can
instantiate and test directly.

handle_command() is async, so each test uses asyncio.run() to drive it
(no pytest-asyncio dependency required).
"""

import asyncio
import json
from unittest.mock import MagicMock

import pytest

from physical_mcp_bridge.bridge_node import BridgeNode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_command(cmd_id: str, cmd_type: str, params: dict | None = None) -> str:
    """Build a raw JSON command string."""
    return json.dumps({"id": cmd_id, "type": cmd_type, "params": params or {}})


def _run(coro):
    """Run an async coroutine synchronously."""
    return asyncio.run(coro)


def _dispatch(bridge, cmd_id, cmd_type, params=None):
    """Build command, send through handle_command, return parsed response."""
    raw = _make_command(cmd_id, cmd_type, params)
    return json.loads(_run(bridge.handle_command(raw)))


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def bridge():
    """Instantiate a real BridgeNode and replace its handlers with mocks.

    BridgeNode.__init__ calls super().__init__('physical_mcp_bridge'),
    which goes to the _FakeNode defined in conftest.py (a no-op).  It then
    creates real handler objects.  We overwrite them with controlled mocks.
    """
    node = BridgeNode()

    # Replace handlers with fresh, isolated mocks.
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

    # Provide a stable mock logger so assertions on get_logger().error work.
    _logger = MagicMock()
    node.get_logger = MagicMock(return_value=_logger)

    return node


# ---------------------------------------------------------------------------
# Discovery dispatch
# ---------------------------------------------------------------------------

class TestDiscoveryDispatch:
    """Verify that discovery commands dispatch to DiscoveryHandler."""

    def test_topic_list(self, bridge):
        bridge.discovery.list_topics.return_value = [
            {"name": "/odom", "types": ["nav_msgs/msg/Odometry"]},
        ]
        resp = _dispatch(bridge, "1", "topic.list")
        assert resp["status"] == "ok"
        assert resp["data"][0]["name"] == "/odom"
        bridge.discovery.list_topics.assert_called_once()

    def test_topic_info(self, bridge):
        bridge.discovery.get_topic_info.return_value = {
            "name": "/scan", "type": "sensor_msgs/msg/LaserScan",
            "publishers": 1, "subscribers": 0,
        }
        resp = _dispatch(bridge, "2", "topic.info", {"topic": "/scan"})
        assert resp["status"] == "ok"
        assert resp["data"]["name"] == "/scan"
        bridge.discovery.get_topic_info.assert_called_once_with("/scan")

    def test_service_list(self, bridge):
        bridge.discovery.list_services.return_value = [
            {"name": "/trigger", "types": ["std_srvs/srv/Trigger"]},
        ]
        resp = _dispatch(bridge, "3", "service.list")
        assert resp["status"] == "ok"
        assert len(resp["data"]) == 1

    def test_action_list(self, bridge):
        bridge.discovery.list_actions.return_value = [{"name": "/navigate"}]
        resp = _dispatch(bridge, "4", "action.list")
        assert resp["status"] == "ok"
        assert resp["data"][0]["name"] == "/navigate"

    def test_node_list(self, bridge):
        bridge.discovery.list_nodes.return_value = [
            {"name": "bridge", "namespace": "/"},
        ]
        resp = _dispatch(bridge, "5", "node.list")
        assert resp["status"] == "ok"
        assert resp["data"][0]["name"] == "bridge"

    def test_service_info(self, bridge):
        bridge.discovery.get_service_info.return_value = {
            "name": "/trigger", "type": "std_srvs/srv/Trigger",
        }
        resp = _dispatch(bridge, "6", "service.info", {"service": "/trigger"})
        assert resp["status"] == "ok"
        assert resp["data"]["type"] == "std_srvs/srv/Trigger"
        bridge.discovery.get_service_info.assert_called_once_with("/trigger")


# ---------------------------------------------------------------------------
# Topic dispatch
# ---------------------------------------------------------------------------

class TestTopicDispatch:
    """Verify that topic commands dispatch to TopicHandler."""

    def test_topic_subscribe(self, bridge):
        bridge.topics.subscribe.return_value = [{"data": "msg1"}]
        resp = _dispatch(bridge, "10", "topic.subscribe", {
            "topic": "/chatter",
            "message_type": "std_msgs/msg/String",
            "count": 1,
            "timeout_sec": 2.0,
        })
        assert resp["status"] == "ok"
        bridge.topics.subscribe.assert_called_once_with(
            "/chatter", "std_msgs/msg/String", 1, 2.0,
        )

    def test_topic_echo(self, bridge):
        bridge.topics.echo.return_value = {"data": "latest"}
        resp = _dispatch(bridge, "11", "topic.echo", {
            "topic": "/odom",
            "message_type": "nav_msgs/msg/Odometry",
        })
        assert resp["status"] == "ok"
        assert resp["data"]["data"] == "latest"

    def test_topic_publish(self, bridge):
        bridge.topics.publish.return_value = True
        resp = _dispatch(bridge, "12", "topic.publish", {
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/msg/Twist",
            "message": {"linear": {"x": 0.5}},
        })
        assert resp["status"] == "ok"
        assert resp["data"] is True

    def test_topic_publish_blocked_by_estop(self, bridge):
        bridge.emergency_stop = True
        resp = _dispatch(bridge, "13", "topic.publish", {
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/msg/Twist",
            "message": {"linear": {"x": 1.0}},
        })
        assert resp["status"] == "ok"
        assert "Emergency stop" in resp["data"]["error"]
        bridge.topics.publish.assert_not_called()

    def test_topic_subscribe_uses_default_count_and_timeout(self, bridge):
        """When count/timeout are omitted, dispatch passes defaults."""
        bridge.topics.subscribe.return_value = [{"data": "msg"}]
        resp = _dispatch(bridge, "14", "topic.subscribe", {
            "topic": "/chatter",
            "message_type": "std_msgs/msg/String",
        })
        assert resp["status"] == "ok"
        bridge.topics.subscribe.assert_called_once_with(
            "/chatter", "std_msgs/msg/String", 1, 5.0,
        )


# ---------------------------------------------------------------------------
# Service dispatch
# ---------------------------------------------------------------------------

class TestServiceDispatch:
    """Verify that service commands dispatch to ServiceHandler."""

    def test_service_call(self, bridge):
        bridge.services.call.return_value = {"success": True, "message": "done"}
        resp = _dispatch(bridge, "20", "service.call", {
            "service": "/trigger",
            "service_type": "std_srvs/srv/Trigger",
            "args": {},
        })
        assert resp["status"] == "ok"
        assert resp["data"]["success"] is True

    def test_service_call_blocked_by_estop(self, bridge):
        bridge.emergency_stop = True
        resp = _dispatch(bridge, "21", "service.call", {
            "service": "/trigger",
            "service_type": "std_srvs/srv/Trigger",
        })
        assert "Emergency stop" in resp["data"]["error"]
        bridge.services.call.assert_not_called()

    def test_service_call_default_args(self, bridge):
        """When args is omitted, an empty dict should be passed."""
        bridge.services.call.return_value = {"success": True}
        resp = _dispatch(bridge, "22", "service.call", {
            "service": "/trigger",
            "service_type": "std_srvs/srv/Trigger",
        })
        assert resp["status"] == "ok"
        bridge.services.call.assert_called_once_with(
            "/trigger", "std_srvs/srv/Trigger", {},
        )


# ---------------------------------------------------------------------------
# Action dispatch
# ---------------------------------------------------------------------------

class TestActionDispatch:
    """Verify that action commands dispatch to ActionHandler."""

    def test_action_send_goal(self, bridge):
        bridge.actions.send_goal.return_value = {
            "status": "completed", "goal_id": "g1", "result": {},
        }
        resp = _dispatch(bridge, "30", "action.send_goal", {
            "action": "/navigate",
            "action_type": "nav2_msgs/action/NavigateToPose",
            "goal": {"pose": {}},
        })
        assert resp["status"] == "ok"
        assert resp["data"]["status"] == "completed"

    def test_action_send_goal_blocked_by_estop(self, bridge):
        bridge.emergency_stop = True
        resp = _dispatch(bridge, "31", "action.send_goal", {
            "action": "/navigate",
            "action_type": "nav2_msgs/action/NavigateToPose",
            "goal": {},
        })
        assert "Emergency stop" in resp["data"]["error"]
        bridge.actions.send_goal.assert_not_called()

    def test_action_cancel(self, bridge):
        bridge.actions.cancel_goal.return_value = {
            "status": "cancelled", "goal_id": "g1",
        }
        resp = _dispatch(bridge, "32", "action.cancel", {
            "action": "/navigate",
            "goal_id": "g1",
        })
        assert resp["status"] == "ok"
        assert resp["data"]["status"] == "cancelled"

    def test_action_cancel_without_goal_id(self, bridge):
        """When goal_id is omitted, None should be passed to cancel_goal."""
        bridge.actions.cancel_goal.return_value = {
            "status": "cancelled_all", "cancelled": [],
        }
        resp = _dispatch(bridge, "34", "action.cancel", {"action": "/navigate"})
        assert resp["status"] == "ok"
        bridge.actions.cancel_goal.assert_called_once_with("/navigate", None)

    def test_action_status(self, bridge):
        bridge.actions.get_status.return_value = {
            "action": "/nav", "active_goals": [], "count": 0,
        }
        resp = _dispatch(bridge, "33", "action.status", {"action": "/nav"})
        assert resp["status"] == "ok"
        assert resp["data"]["count"] == 0


# ---------------------------------------------------------------------------
# System commands
# ---------------------------------------------------------------------------

class TestSystemDispatch:
    """Verify system-level commands: ping, emergency_stop, params."""

    def test_ping(self, bridge):
        resp = _dispatch(bridge, "40", "ping")
        assert resp["status"] == "ok"
        assert resp["data"]["pong"] is True
        assert resp["data"]["bridge"] == "physical-mcp-bridge"
        assert resp["data"]["version"] == "0.1.0"
        assert "telemetry" in resp["data"]

    def test_get_params(self, bridge):
        resp = _dispatch(bridge, "41", "params.get")
        assert resp["status"] == "ok"
        assert resp["data"] == {}

    def test_emergency_stop(self, bridge):
        bridge.topics.publish.return_value = True
        resp = _dispatch(bridge, "42", "emergency_stop")
        assert resp["status"] == "ok"
        assert resp["data"]["emergency_stop"] is True
        assert bridge.emergency_stop is True
        bridge.actions.cancel_all.assert_called_once()

    def test_emergency_stop_publishes_zero_velocity(self, bridge):
        bridge.topics.publish.return_value = True
        _dispatch(bridge, "43", "emergency_stop")
        bridge.topics.publish.assert_called_once_with(
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
             "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        )

    def test_emergency_stop_survives_publish_failure(self, bridge):
        """Even if zero-velocity publish fails, estop should still succeed."""
        bridge.topics.publish.side_effect = RuntimeError("publisher broken")
        resp = _dispatch(bridge, "44", "emergency_stop")
        assert resp["status"] == "ok"
        assert resp["data"]["emergency_stop"] is True
        assert bridge.emergency_stop is True


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------

class TestErrorHandling:
    """Verify error responses for malformed input and handler exceptions."""

    def test_parse_error_returns_error_response(self, bridge):
        resp = json.loads(_run(bridge.handle_command("NOT VALID JSON")))
        assert resp["status"] == "error"
        assert "Parse error" in resp["data"]["error"]
        assert resp["id"] is None

    def test_handler_exception_returns_error_response(self, bridge):
        bridge.discovery.list_topics.side_effect = RuntimeError("rclpy exploded")
        resp = _dispatch(bridge, "50", "topic.list")
        assert resp["status"] == "error"
        assert "rclpy exploded" in resp["data"]["error"]

    def test_handler_exception_records_telemetry_error(self, bridge):
        bridge.discovery.list_topics.side_effect = RuntimeError("boom")
        _dispatch(bridge, "51", "topic.list")
        bridge.telemetry.record_error.assert_called_once()

    def test_telemetry_recorded_on_every_command(self, bridge):
        bridge.discovery.list_topics.return_value = []
        _dispatch(bridge, "52", "topic.list")
        bridge.telemetry.record_command.assert_called_once()

    def test_response_always_has_id_status_data_timestamp(self, bridge):
        bridge.discovery.list_topics.return_value = []
        resp = _dispatch(bridge, "60", "topic.list")
        assert "id" in resp
        assert "status" in resp
        assert "data" in resp
        assert "timestamp" in resp
        assert resp["id"] == "60"

    def test_unknown_command_type_in_json(self, bridge):
        """An invalid command type string should produce a parse error."""
        raw = json.dumps({"id": "99", "type": "totally.unknown", "params": {}})
        resp = json.loads(_run(bridge.handle_command(raw)))
        assert resp["status"] == "error"
        # parse_command raises ValueError before id is extracted
        assert resp["id"] is None

    def test_handler_exception_logs_error(self, bridge):
        bridge.discovery.list_topics.side_effect = RuntimeError("fail")
        _dispatch(bridge, "70", "topic.list")
        bridge.get_logger().error.assert_called_once()

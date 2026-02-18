"""Tests for the ROS2 topic handler.

All rclpy and rosidl dependencies are mocked via conftest.py so tests run
without ROS2 installed.
"""

from collections import OrderedDict
from unittest.mock import MagicMock, patch

import pytest

from physical_mcp_bridge.topic_handler import TopicHandler


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_node():
    """Create a mock rclpy Node."""
    return MagicMock()


@pytest.fixture
def handler(mock_node):
    """Return a TopicHandler backed by the mock node."""
    return TopicHandler(mock_node)


# ---------------------------------------------------------------------------
# _set_message_fields
# ---------------------------------------------------------------------------

class TestSetMessageFields:
    """Tests for the internal _set_message_fields helper."""

    def test_sets_simple_field(self, handler):
        msg = MagicMock()
        msg.x = 0.0
        handler._set_message_fields(msg, {"x": 1.5})
        assert msg.x == 1.5

    def test_sets_nested_field(self, handler):
        inner = MagicMock()
        inner.x = 0.0
        outer = MagicMock()
        outer.linear = inner
        handler._set_message_fields(outer, {"linear": {"x": 3.0}})
        assert inner.x == 3.0

    def test_ignores_unknown_field(self, handler):
        msg = MagicMock(spec=["x"])
        # spec limits hasattr to only "x"
        handler._set_message_fields(msg, {"nonexistent": 42})
        # No exception should be raised.

    def test_sets_multiple_fields(self, handler):
        msg = MagicMock()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        handler._set_message_fields(msg, {"x": 1.0, "y": 2.0, "z": 3.0})
        assert msg.x == 1.0
        assert msg.y == 2.0
        assert msg.z == 3.0

    def test_deeply_nested_fields(self, handler):
        """Three levels of nesting: outer.mid.inner.val."""
        inner = MagicMock()
        inner.val = 0
        mid = MagicMock()
        mid.inner = inner
        outer = MagicMock()
        outer.mid = mid
        handler._set_message_fields(outer, {"mid": {"inner": {"val": 99}}})
        assert inner.val == 99


# ---------------------------------------------------------------------------
# subscribe
# ---------------------------------------------------------------------------

class TestSubscribe:
    """Tests for the subscribe method."""

    @patch("physical_mcp_bridge.topic_handler.get_message")
    @patch("physical_mcp_bridge.topic_handler.message_to_ordereddict")
    def test_subscribe_collects_messages(self, mock_to_dict, mock_get_message, handler, mock_node):
        """subscribe() should collect messages and return them as dicts."""
        mock_msg_class = MagicMock()
        mock_get_message.return_value = mock_msg_class

        sample = OrderedDict({"data": "hello"})
        mock_to_dict.return_value = sample

        def fake_create_subscription(msg_cls, topic, callback, qos):
            callback(MagicMock())
            return MagicMock()

        mock_node.create_subscription.side_effect = fake_create_subscription

        result = handler.subscribe("/chatter", "std_msgs/msg/String", count=1, timeout_sec=1.0)
        assert len(result) == 1
        assert result[0] == sample

    @patch("physical_mcp_bridge.topic_handler.get_message")
    @patch("physical_mcp_bridge.topic_handler.message_to_ordereddict")
    def test_subscribe_returns_empty_on_timeout(self, mock_to_dict, mock_get_message, handler, mock_node):
        """When no messages arrive within timeout, subscribe returns empty list."""
        mock_get_message.return_value = MagicMock()
        mock_node.create_subscription.return_value = MagicMock()

        result = handler.subscribe("/silent", "std_msgs/msg/String", count=1, timeout_sec=0.05)
        assert result == []

    @patch("physical_mcp_bridge.topic_handler.get_message")
    @patch("physical_mcp_bridge.topic_handler.message_to_ordereddict")
    def test_subscribe_destroys_subscription(self, mock_to_dict, mock_get_message, handler, mock_node):
        """The subscription must always be destroyed after subscribe() completes."""
        mock_get_message.return_value = MagicMock()
        mock_sub = MagicMock()
        mock_node.create_subscription.return_value = mock_sub

        handler.subscribe("/topic", "msg/Type", count=1, timeout_sec=0.05)
        mock_node.destroy_subscription.assert_called_once_with(mock_sub)

    @patch("physical_mcp_bridge.topic_handler.get_message")
    @patch("physical_mcp_bridge.topic_handler.message_to_ordereddict")
    def test_subscribe_multiple_messages(self, mock_to_dict, mock_get_message, handler, mock_node):
        """subscribe() with count=3 should collect three messages."""
        mock_get_message.return_value = MagicMock()
        counter = {"n": 0}

        def fake_to_dict(msg):
            counter["n"] += 1
            return OrderedDict({"seq": counter["n"]})

        mock_to_dict.side_effect = fake_to_dict

        def fake_create_subscription(msg_cls, topic, callback, qos):
            for _ in range(3):
                callback(MagicMock())
            return MagicMock()

        mock_node.create_subscription.side_effect = fake_create_subscription

        result = handler.subscribe("/data", "msg/Type", count=3, timeout_sec=1.0)
        assert len(result) == 3
        assert result[0]["seq"] == 1
        assert result[2]["seq"] == 3


# ---------------------------------------------------------------------------
# echo
# ---------------------------------------------------------------------------

class TestEcho:
    """Tests for the echo convenience method."""

    @patch("physical_mcp_bridge.topic_handler.get_message")
    @patch("physical_mcp_bridge.topic_handler.message_to_ordereddict")
    def test_echo_returns_single_message(self, mock_to_dict, mock_get_message, handler, mock_node):
        mock_get_message.return_value = MagicMock()
        sample = OrderedDict({"data": 42})
        mock_to_dict.return_value = sample

        def fake_create_subscription(msg_cls, topic, callback, qos):
            callback(MagicMock())
            return MagicMock()

        mock_node.create_subscription.side_effect = fake_create_subscription

        result = handler.echo("/topic", "msg/Type", timeout_sec=1.0)
        assert result == sample

    @patch("physical_mcp_bridge.topic_handler.get_message")
    @patch("physical_mcp_bridge.topic_handler.message_to_ordereddict")
    def test_echo_returns_none_on_timeout(self, mock_to_dict, mock_get_message, handler, mock_node):
        mock_get_message.return_value = MagicMock()
        mock_node.create_subscription.return_value = MagicMock()

        result = handler.echo("/silent", "msg/Type", timeout_sec=0.05)
        assert result is None


# ---------------------------------------------------------------------------
# publish
# ---------------------------------------------------------------------------

class TestPublish:
    """Tests for the publish method."""

    @patch("physical_mcp_bridge.topic_handler.time")
    @patch("physical_mcp_bridge.topic_handler.get_message")
    def test_publish_returns_true(self, mock_get_message, mock_time, handler, mock_node):
        mock_msg_class = MagicMock()
        mock_msg_instance = MagicMock()
        mock_msg_class.return_value = mock_msg_instance
        mock_get_message.return_value = mock_msg_class

        mock_pub = MagicMock()
        mock_node.create_publisher.return_value = mock_pub

        result = handler.publish("/cmd_vel", "geometry_msgs/msg/Twist", {"linear": {"x": 0.5}})
        assert result is True

    @patch("physical_mcp_bridge.topic_handler.time")
    @patch("physical_mcp_bridge.topic_handler.get_message")
    def test_publish_calls_publisher(self, mock_get_message, mock_time, handler, mock_node):
        mock_msg_class = MagicMock()
        mock_msg_instance = MagicMock()
        mock_msg_class.return_value = mock_msg_instance
        mock_get_message.return_value = mock_msg_class

        mock_pub = MagicMock()
        mock_node.create_publisher.return_value = mock_pub

        handler.publish("/topic", "msg/Type", {"data": 1})
        mock_pub.publish.assert_called_once_with(mock_msg_instance)

    @patch("physical_mcp_bridge.topic_handler.time")
    @patch("physical_mcp_bridge.topic_handler.get_message")
    def test_publish_destroys_publisher(self, mock_get_message, mock_time, handler, mock_node):
        mock_msg_class = MagicMock()
        mock_msg_class.return_value = MagicMock()
        mock_get_message.return_value = mock_msg_class

        mock_pub = MagicMock()
        mock_node.create_publisher.return_value = mock_pub

        handler.publish("/topic", "msg/Type", {})
        mock_node.destroy_publisher.assert_called_once_with(mock_pub)

    @patch("physical_mcp_bridge.topic_handler.time")
    @patch("physical_mcp_bridge.topic_handler.get_message")
    def test_publish_creates_correct_publisher(self, mock_get_message, mock_time, handler, mock_node):
        mock_msg_class = MagicMock()
        mock_msg_class.return_value = MagicMock()
        mock_get_message.return_value = mock_msg_class

        mock_node.create_publisher.return_value = MagicMock()

        handler.publish("/cmd_vel", "geometry_msgs/msg/Twist", {})
        mock_node.create_publisher.assert_called_once_with(mock_msg_class, "/cmd_vel", 10)

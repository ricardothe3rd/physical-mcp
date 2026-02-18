"""Tests for the ROS2 discovery handler.

All rclpy dependencies are mocked via conftest.py so tests run without
ROS2 installed.
"""

from unittest.mock import MagicMock

import pytest

from physical_mcp_bridge.discovery import DiscoveryHandler


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_node():
    """Create a mock rclpy Node with configurable discovery methods."""
    node = MagicMock()
    # Defaults -- callers override per-test as needed.
    node.get_topic_names_and_types.return_value = []
    node.get_service_names_and_types.return_value = []
    node.get_node_names_and_namespaces.return_value = []
    node.count_publishers.return_value = 0
    node.count_subscribers.return_value = 0
    return node


@pytest.fixture
def handler(mock_node):
    """Return a DiscoveryHandler backed by the mock node."""
    return DiscoveryHandler(mock_node)


# ---------------------------------------------------------------------------
# list_topics
# ---------------------------------------------------------------------------

class TestListTopics:
    def test_returns_empty_list_when_no_topics(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = []
        result = handler.list_topics()
        assert result == []

    def test_returns_topics_with_types(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
            ("/odom", ["nav_msgs/msg/Odometry"]),
        ]
        result = handler.list_topics()
        assert len(result) == 2
        assert result[0] == {"name": "/cmd_vel", "types": ["geometry_msgs/msg/Twist"]}
        assert result[1] == {"name": "/odom", "types": ["nav_msgs/msg/Odometry"]}

    def test_topic_with_multiple_types(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/multi", ["type_a", "type_b"]),
        ]
        result = handler.list_topics()
        assert result[0]["types"] == ["type_a", "type_b"]


# ---------------------------------------------------------------------------
# get_topic_info
# ---------------------------------------------------------------------------

class TestGetTopicInfo:
    def test_returns_info_for_existing_topic(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/scan", ["sensor_msgs/msg/LaserScan"]),
        ]
        mock_node.count_publishers.return_value = 2
        mock_node.count_subscribers.return_value = 3
        result = handler.get_topic_info("/scan")
        assert result["name"] == "/scan"
        assert result["type"] == "sensor_msgs/msg/LaserScan"
        assert result["publishers"] == 2
        assert result["subscribers"] == 3

    def test_returns_empty_type_for_unknown_topic(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/other", ["some/msg/Type"]),
        ]
        mock_node.count_publishers.return_value = 0
        mock_node.count_subscribers.return_value = 0
        result = handler.get_topic_info("/nonexistent")
        assert result["type"] == ""

    def test_returns_first_type_when_topic_has_multiple(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/multi", ["type_a", "type_b"]),
        ]
        mock_node.count_publishers.return_value = 1
        mock_node.count_subscribers.return_value = 0
        result = handler.get_topic_info("/multi")
        assert result["type"] == "type_a"

    def test_returns_empty_type_when_types_list_is_empty(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/empty_types", []),
        ]
        mock_node.count_publishers.return_value = 0
        mock_node.count_subscribers.return_value = 0
        result = handler.get_topic_info("/empty_types")
        assert result["type"] == ""


# ---------------------------------------------------------------------------
# list_services
# ---------------------------------------------------------------------------

class TestListServices:
    def test_returns_empty_list_when_no_services(self, handler, mock_node):
        mock_node.get_service_names_and_types.return_value = []
        result = handler.list_services()
        assert result == []

    def test_returns_services_with_types(self, handler, mock_node):
        mock_node.get_service_names_and_types.return_value = [
            ("/set_bool", ["std_srvs/srv/SetBool"]),
            ("/trigger", ["std_srvs/srv/Trigger"]),
        ]
        result = handler.list_services()
        assert len(result) == 2
        assert result[0] == {"name": "/set_bool", "types": ["std_srvs/srv/SetBool"]}
        assert result[1] == {"name": "/trigger", "types": ["std_srvs/srv/Trigger"]}


# ---------------------------------------------------------------------------
# get_service_info
# ---------------------------------------------------------------------------

class TestGetServiceInfo:
    def test_returns_info_for_existing_service(self, handler, mock_node):
        mock_node.get_service_names_and_types.return_value = [
            ("/trigger", ["std_srvs/srv/Trigger"]),
        ]
        result = handler.get_service_info("/trigger")
        assert result["name"] == "/trigger"
        assert result["type"] == "std_srvs/srv/Trigger"

    def test_returns_empty_type_for_unknown_service(self, handler, mock_node):
        mock_node.get_service_names_and_types.return_value = []
        result = handler.get_service_info("/unknown")
        assert result["name"] == "/unknown"
        assert result["type"] == ""


# ---------------------------------------------------------------------------
# list_actions
# ---------------------------------------------------------------------------

class TestListActions:
    def test_returns_empty_when_no_action_topics(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
        ]
        result = handler.list_actions()
        assert result == []

    def test_detects_action_from_feedback_topic(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/navigate_to_pose/_action/feedback", ["nav2/action/NavigateToPose_FeedbackMessage"]),
            ("/navigate_to_pose/_action/status", ["action_msgs/msg/GoalStatusArray"]),
        ]
        result = handler.list_actions()
        assert len(result) == 1
        assert result[0]["name"] == "/navigate_to_pose"

    def test_detects_multiple_actions(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/navigate/_action/feedback", ["msg_type"]),
            ("/dock/_action/feedback", ["msg_type"]),
        ]
        result = handler.list_actions()
        names = {a["name"] for a in result}
        assert names == {"/navigate", "/dock"}

    def test_returns_sorted_list(self, handler, mock_node):
        mock_node.get_topic_names_and_types.return_value = [
            ("/z_action/_action/feedback", ["msg"]),
            ("/a_action/_action/feedback", ["msg"]),
            ("/m_action/_action/feedback", ["msg"]),
        ]
        result = handler.list_actions()
        assert [a["name"] for a in result] == ["/a_action", "/m_action", "/z_action"]

    def test_deduplicates_action_names(self, handler, mock_node):
        # Two feedback topics for the same action should only produce one entry.
        mock_node.get_topic_names_and_types.return_value = [
            ("/nav/_action/feedback", ["msg_a"]),
            ("/nav/_action/feedback", ["msg_b"]),
        ]
        result = handler.list_actions()
        assert len(result) == 1


# ---------------------------------------------------------------------------
# list_nodes
# ---------------------------------------------------------------------------

class TestListNodes:
    def test_returns_empty_when_no_nodes(self, handler, mock_node):
        mock_node.get_node_names_and_namespaces.return_value = []
        result = handler.list_nodes()
        assert result == []

    def test_returns_nodes_with_namespaces(self, handler, mock_node):
        mock_node.get_node_names_and_namespaces.return_value = [
            ("bridge", "/"),
            ("lidar_node", "/sensors"),
        ]
        result = handler.list_nodes()
        assert len(result) == 2
        assert result[0] == {"name": "bridge", "namespace": "/"}
        assert result[1] == {"name": "lidar_node", "namespace": "/sensors"}

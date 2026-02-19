"""Tests for the ROS2 action handler.

All rclpy and rosidl dependencies are mocked via conftest.py so tests run
without ROS2 installed.
"""

from collections import OrderedDict
from unittest.mock import MagicMock, patch, call

import pytest

from physical_mcp_bridge.action_handler import ActionHandler


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_node():
    """Create a mock rclpy Node."""
    return MagicMock()


@pytest.fixture
def handler(mock_node):
    """Return an ActionHandler backed by the mock node."""
    return ActionHandler(mock_node)


# ---------------------------------------------------------------------------
# Helper to build a "completed" send_goal mock chain
# ---------------------------------------------------------------------------

def _make_completed_goal_mocks(goal_id="goal-abc-123", result_dict=None):
    """Return (mock_action_class, mock_client, mock_goal_handle) wired for a
    completed goal flow. Caller still needs to patch get_action and ActionClient.
    """
    if result_dict is None:
        result_dict = OrderedDict({"success": True})

    mock_action_class = MagicMock()
    mock_goal = MagicMock()
    mock_action_class.Goal.return_value = mock_goal

    mock_goal_handle = MagicMock()
    mock_goal_handle.accepted = True
    mock_goal_handle.goal_id = goal_id

    # send_goal_async future
    mock_send_future = MagicMock()
    mock_send_future.done.return_value = True
    mock_send_future.result.return_value = mock_goal_handle
    mock_send_future.add_done_callback.side_effect = lambda cb: cb(mock_send_future)

    # get_result_async future
    mock_result = MagicMock()
    mock_result.result = MagicMock()  # the actual result message

    mock_result_future = MagicMock()
    mock_result_future.done.return_value = True
    mock_result_future.result.return_value = mock_result
    mock_result_future.add_done_callback.side_effect = lambda cb: cb(mock_result_future)

    mock_goal_handle.get_result_async.return_value = mock_result_future

    mock_client = MagicMock()
    mock_client.wait_for_server.return_value = True
    mock_client.send_goal_async.return_value = mock_send_future

    return mock_action_class, mock_client, mock_goal_handle


# ---------------------------------------------------------------------------
# _set_fields
# ---------------------------------------------------------------------------

class TestSetFields:
    """Tests for the internal _set_fields helper."""

    def test_sets_simple_field(self, handler):
        msg = MagicMock()
        msg.order = 0
        handler._set_fields(msg, {"order": 10})
        assert msg.order == 10

    def test_sets_nested_field(self, handler):
        inner = MagicMock()
        inner.x = 0.0
        outer = MagicMock()
        outer.pose = inner
        handler._set_fields(outer, {"pose": {"x": 2.5}})
        assert inner.x == 2.5

    def test_ignores_unknown_field(self, handler):
        msg = MagicMock(spec=["order"])
        handler._set_fields(msg, {"nonexistent": 99})
        # No exception.

    def test_empty_dict_is_noop(self, handler):
        msg = MagicMock()
        msg.val = 1
        handler._set_fields(msg, {})
        assert msg.val == 1


# ---------------------------------------------------------------------------
# send_goal – success / completed
# ---------------------------------------------------------------------------

class TestSendGoalSuccess:
    """Tests for send_goal when the goal completes successfully."""

    @patch("physical_mcp_bridge.action_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_send_goal_returns_completed(self, mock_get_action, mock_action_client_cls,
                                          mock_to_dict, handler, mock_node):
        mock_action_class, mock_client, _ = _make_completed_goal_mocks(goal_id="g-001")
        mock_get_action.return_value = mock_action_class
        mock_action_client_cls.return_value = mock_client

        expected_result = OrderedDict({"success": True})
        mock_to_dict.return_value = expected_result

        result = handler.send_goal("/navigate", "nav2/action/Navigate", {"target_x": 1.0})
        assert result["status"] == "completed"
        assert result["goal_id"] == "g-001"
        assert result["result"] == expected_result

    @patch("physical_mcp_bridge.action_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_send_goal_creates_action_client(self, mock_get_action, mock_action_client_cls,
                                              mock_to_dict, handler, mock_node):
        mock_action_class, mock_client, _ = _make_completed_goal_mocks()
        mock_get_action.return_value = mock_action_class
        mock_action_client_cls.return_value = mock_client
        mock_to_dict.return_value = OrderedDict()

        handler.send_goal("/dock", "pkg/action/Dock", {})
        mock_action_client_cls.assert_called_once_with(mock_node, mock_action_class, "/dock")

    @patch("physical_mcp_bridge.action_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_send_goal_sets_goal_fields(self, mock_get_action, mock_action_client_cls,
                                         mock_to_dict, handler, mock_node):
        mock_action_class, mock_client, _ = _make_completed_goal_mocks()
        mock_goal = MagicMock()
        mock_goal.target_x = 0.0
        mock_action_class.Goal.return_value = mock_goal
        mock_get_action.return_value = mock_action_class
        mock_action_client_cls.return_value = mock_client
        mock_to_dict.return_value = OrderedDict()

        handler.send_goal("/nav", "pkg/action/Nav", {"target_x": 3.14})
        assert mock_goal.target_x == 3.14

    @patch("physical_mcp_bridge.action_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_send_goal_removes_completed_from_active(self, mock_get_action, mock_action_client_cls,
                                                      mock_to_dict, handler, mock_node):
        """After completion, the goal should be removed from _active_goals."""
        mock_action_class, mock_client, _ = _make_completed_goal_mocks(goal_id="g-done")
        mock_get_action.return_value = mock_action_class
        mock_action_client_cls.return_value = mock_client
        mock_to_dict.return_value = OrderedDict()

        handler.send_goal("/nav", "pkg/action/Nav", {})
        assert "g-done" not in handler._active_goals


# ---------------------------------------------------------------------------
# send_goal – server unavailable
# ---------------------------------------------------------------------------

class TestSendGoalUnavailable:
    """Tests when the action server is not available."""

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_returns_error_when_server_unavailable(self, mock_get_action, mock_action_client_cls,
                                                    handler, mock_node):
        mock_get_action.return_value = MagicMock()
        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = False
        mock_action_client_cls.return_value = mock_client

        result = handler.send_goal("/missing", "pkg/action/Missing", {})
        assert "error" in result
        assert "not available" in result["error"]

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_unavailable_still_destroys_client(self, mock_get_action, mock_action_client_cls,
                                                handler, mock_node):
        mock_get_action.return_value = MagicMock()
        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = False
        mock_action_client_cls.return_value = mock_client

        handler.send_goal("/missing", "pkg/action/Missing", {})
        mock_client.destroy.assert_called_once()


# ---------------------------------------------------------------------------
# send_goal – goal rejected
# ---------------------------------------------------------------------------

class TestSendGoalRejected:
    """Tests when the action server rejects the goal."""

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_returns_rejected_status(self, mock_get_action, mock_action_client_cls,
                                      handler, mock_node):
        mock_action_class = MagicMock()
        mock_action_class.Goal.return_value = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = False

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_goal_handle
        mock_future.add_done_callback.side_effect = lambda cb: cb(mock_future)

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_client.send_goal_async.return_value = mock_future
        mock_action_client_cls.return_value = mock_client

        result = handler.send_goal("/nav", "pkg/action/Nav", {})
        assert result["status"] == "rejected"

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_rejected_does_not_store_goal(self, mock_get_action, mock_action_client_cls,
                                           handler, mock_node):
        mock_action_class = MagicMock()
        mock_action_class.Goal.return_value = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = False

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_goal_handle
        mock_future.add_done_callback.side_effect = lambda cb: cb(mock_future)

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_client.send_goal_async.return_value = mock_future
        mock_action_client_cls.return_value = mock_client

        handler.send_goal("/nav", "pkg/action/Nav", {})
        assert len(handler._active_goals) == 0


# ---------------------------------------------------------------------------
# send_goal – submission timeout
# ---------------------------------------------------------------------------

class TestSendGoalTimeout:
    """Tests when send_goal_async times out."""

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_returns_timeout_on_submission(self, mock_get_action, mock_action_client_cls,
                                            handler, mock_node):
        mock_action_class = MagicMock()
        mock_action_class.Goal.return_value = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_future.add_done_callback.side_effect = lambda cb: None  # never fires

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_client.send_goal_async.return_value = mock_future
        mock_action_client_cls.return_value = mock_client

        result = handler.send_goal("/slow", "pkg/action/Slow", {}, timeout_sec=0.01)
        assert "error" in result
        assert "timed out" in result["error"]


# ---------------------------------------------------------------------------
# send_goal – result pending (in_progress)
# ---------------------------------------------------------------------------

class TestSendGoalInProgress:
    """Tests when the goal is accepted but the result has not arrived."""

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_returns_in_progress_when_result_pending(self, mock_get_action, mock_action_client_cls,
                                                      handler, mock_node):
        mock_action_class = MagicMock()
        mock_action_class.Goal.return_value = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = True
        mock_goal_handle.goal_id = "g-pending"

        # send_goal_async completes
        mock_send_future = MagicMock()
        mock_send_future.done.return_value = True
        mock_send_future.result.return_value = mock_goal_handle
        mock_send_future.add_done_callback.side_effect = lambda cb: cb(mock_send_future)

        # get_result_async never completes
        mock_result_future = MagicMock()
        mock_result_future.done.return_value = False
        mock_result_future.add_done_callback.side_effect = lambda cb: None
        mock_goal_handle.get_result_async.return_value = mock_result_future

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_client.send_goal_async.return_value = mock_send_future
        mock_action_client_cls.return_value = mock_client

        result = handler.send_goal("/nav", "pkg/action/Nav", {}, timeout_sec=0.01)
        assert result["status"] == "in_progress"
        assert result["goal_id"] == "g-pending"
        assert "pending" in result["message"]

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_in_progress_keeps_goal_in_active(self, mock_get_action, mock_action_client_cls,
                                               handler, mock_node):
        """An in-progress goal should remain in _active_goals."""
        mock_action_class = MagicMock()
        mock_action_class.Goal.return_value = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = True
        mock_goal_handle.goal_id = "g-active"

        mock_send_future = MagicMock()
        mock_send_future.done.return_value = True
        mock_send_future.result.return_value = mock_goal_handle
        mock_send_future.add_done_callback.side_effect = lambda cb: cb(mock_send_future)

        mock_result_future = MagicMock()
        mock_result_future.done.return_value = False
        mock_result_future.add_done_callback.side_effect = lambda cb: None
        mock_goal_handle.get_result_async.return_value = mock_result_future

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_client.send_goal_async.return_value = mock_send_future
        mock_action_client_cls.return_value = mock_client

        handler.send_goal("/nav", "pkg/action/Nav", {}, timeout_sec=0.01)
        assert "g-active" in handler._active_goals


# ---------------------------------------------------------------------------
# cancel_goal – specific goal
# ---------------------------------------------------------------------------

class TestCancelGoal:
    """Tests for cancelling a specific goal."""

    def test_cancel_specific_goal(self, handler):
        mock_handle = MagicMock()
        mock_cancel_future = MagicMock()
        mock_cancel_future.add_done_callback.side_effect = lambda cb: cb(mock_cancel_future)
        mock_handle.cancel_goal_async.return_value = mock_cancel_future

        handler._active_goals["g-1"] = mock_handle

        result = handler.cancel_goal("/nav", goal_id="g-1")
        assert result["status"] == "cancelled"
        assert result["goal_id"] == "g-1"
        assert "g-1" not in handler._active_goals

    def test_cancel_specific_calls_cancel_async(self, handler):
        mock_handle = MagicMock()
        mock_cancel_future = MagicMock()
        mock_cancel_future.add_done_callback.side_effect = lambda cb: cb(mock_cancel_future)
        mock_handle.cancel_goal_async.return_value = mock_cancel_future

        handler._active_goals["g-x"] = mock_handle

        handler.cancel_goal("/nav", goal_id="g-x")
        mock_handle.cancel_goal_async.assert_called_once()


# ---------------------------------------------------------------------------
# cancel_goal – cancel all for action (no specific goal_id)
# ---------------------------------------------------------------------------

class TestCancelGoalAll:
    """Tests for cancelling all goals when no goal_id is provided."""

    def test_cancel_all_returns_cancelled_list(self, handler):
        handler._active_goals["g-1"] = MagicMock()
        handler._active_goals["g-2"] = MagicMock()

        result = handler.cancel_goal("/nav")
        assert result["status"] == "cancelled_all"
        assert set(result["cancelled"]) == {"g-1", "g-2"}

    def test_cancel_all_clears_active_goals(self, handler):
        handler._active_goals["g-1"] = MagicMock()
        handler._active_goals["g-2"] = MagicMock()

        handler.cancel_goal("/nav")
        assert len(handler._active_goals) == 0

    def test_cancel_all_with_no_active_goals(self, handler):
        result = handler.cancel_goal("/nav")
        assert result["status"] == "cancelled_all"
        assert result["cancelled"] == []

    def test_cancel_all_handles_exception_gracefully(self, handler):
        """If cancel_goal_async raises, the failing goal stays but others are removed."""
        failing_handle = MagicMock()
        failing_handle.cancel_goal_async.side_effect = RuntimeError("connection lost")
        ok_handle = MagicMock()

        handler._active_goals["g-fail"] = failing_handle
        handler._active_goals["g-ok"] = ok_handle

        result = handler.cancel_goal("/nav")
        assert result["status"] == "cancelled_all"
        # The cancelled list includes all goals that were active at the start
        assert set(result["cancelled"]) == {"g-fail", "g-ok"}
        # The failing goal's pop was skipped (inside try before exception),
        # while the successful one was removed.
        assert "g-ok" not in handler._active_goals
        assert "g-fail" in handler._active_goals

    def test_cancel_unknown_goal_id_falls_through(self, handler):
        """If goal_id is given but not in _active_goals, fall through to cancel-all."""
        handler._active_goals["g-real"] = MagicMock()

        result = handler.cancel_goal("/nav", goal_id="g-nonexistent")
        # Falls through to cancel_all branch because goal_id not in _active_goals
        assert result["status"] == "cancelled_all"


# ---------------------------------------------------------------------------
# get_status
# ---------------------------------------------------------------------------

class TestGetStatus:
    """Tests for get_status."""

    def test_returns_empty_when_no_active_goals(self, handler):
        result = handler.get_status("/nav")
        assert result["action"] == "/nav"
        assert result["active_goals"] == []
        assert result["count"] == 0

    def test_returns_active_goals(self, handler):
        handler._active_goals["g-1"] = MagicMock()
        handler._active_goals["g-2"] = MagicMock()

        result = handler.get_status("/nav")
        assert result["action"] == "/nav"
        assert set(result["active_goals"]) == {"g-1", "g-2"}
        assert result["count"] == 2

    def test_status_reflects_different_action_name(self, handler):
        result = handler.get_status("/dock")
        assert result["action"] == "/dock"


# ---------------------------------------------------------------------------
# cancel_all (e-stop)
# ---------------------------------------------------------------------------

class TestCancelAll:
    """Tests for cancel_all (used by e-stop)."""

    def test_cancel_all_clears_goals(self, handler):
        handler._active_goals["g-1"] = MagicMock()
        handler._active_goals["g-2"] = MagicMock()
        handler._active_goals["g-3"] = MagicMock()

        handler.cancel_all()
        assert len(handler._active_goals) == 0

    def test_cancel_all_calls_cancel_on_each(self, handler):
        h1 = MagicMock()
        h2 = MagicMock()
        handler._active_goals["g-1"] = h1
        handler._active_goals["g-2"] = h2

        handler.cancel_all()
        h1.cancel_goal_async.assert_called_once()
        h2.cancel_goal_async.assert_called_once()

    def test_cancel_all_handles_exceptions(self, handler):
        """cancel_all should not raise even if cancel_goal_async fails."""
        failing = MagicMock()
        failing.cancel_goal_async.side_effect = RuntimeError("boom")
        handler._active_goals["g-fail"] = failing

        handler.cancel_all()  # Should not raise
        assert len(handler._active_goals) == 0

    def test_cancel_all_with_no_goals_is_noop(self, handler):
        handler.cancel_all()  # Should not raise
        assert len(handler._active_goals) == 0


# ---------------------------------------------------------------------------
# send_goal – cleanup guarantee
# ---------------------------------------------------------------------------

class TestSendGoalCleanup:
    """Tests that the action client is always destroyed in the finally block."""

    @patch("physical_mcp_bridge.action_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_client_destroyed_on_success(self, mock_get_action, mock_action_client_cls,
                                          mock_to_dict, handler, mock_node):
        mock_action_class, mock_client, _ = _make_completed_goal_mocks()
        mock_get_action.return_value = mock_action_class
        mock_action_client_cls.return_value = mock_client
        mock_to_dict.return_value = OrderedDict()

        handler.send_goal("/nav", "pkg/action/Nav", {})
        mock_client.destroy.assert_called_once()

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_client_destroyed_on_rejection(self, mock_get_action, mock_action_client_cls,
                                            handler, mock_node):
        mock_action_class = MagicMock()
        mock_action_class.Goal.return_value = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = False

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_goal_handle
        mock_future.add_done_callback.side_effect = lambda cb: cb(mock_future)

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_client.send_goal_async.return_value = mock_future
        mock_action_client_cls.return_value = mock_client

        handler.send_goal("/nav", "pkg/action/Nav", {})
        mock_client.destroy.assert_called_once()

    @patch("physical_mcp_bridge.action_handler.ActionClient")
    @patch("physical_mcp_bridge.action_handler.get_action")
    def test_client_destroyed_on_timeout(self, mock_get_action, mock_action_client_cls,
                                          handler, mock_node):
        mock_action_class = MagicMock()
        mock_action_class.Goal.return_value = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_future.add_done_callback.side_effect = lambda cb: None

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_client.send_goal_async.return_value = mock_future
        mock_action_client_cls.return_value = mock_client

        handler.send_goal("/slow", "pkg/action/Slow", {}, timeout_sec=0.01)
        mock_client.destroy.assert_called_once()

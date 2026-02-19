"""Tests for the ROS2 service handler.

All rclpy and rosidl dependencies are mocked via conftest.py so tests run
without ROS2 installed.
"""

from collections import OrderedDict
from unittest.mock import MagicMock, patch, call

import pytest

from physical_mcp_bridge.service_handler import ServiceHandler


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_node():
    """Create a mock rclpy Node."""
    return MagicMock()


@pytest.fixture
def handler(mock_node):
    """Return a ServiceHandler backed by the mock node."""
    return ServiceHandler(mock_node)


# ---------------------------------------------------------------------------
# _set_fields
# ---------------------------------------------------------------------------

class TestSetFields:
    """Tests for the internal _set_fields helper."""

    def test_sets_simple_field(self, handler):
        msg = MagicMock()
        msg.data = False
        handler._set_fields(msg, {"data": True})
        assert msg.data is True

    def test_sets_nested_field(self, handler):
        inner = MagicMock()
        inner.x = 0.0
        outer = MagicMock()
        outer.position = inner
        handler._set_fields(outer, {"position": {"x": 5.0}})
        assert inner.x == 5.0

    def test_ignores_unknown_field(self, handler):
        msg = MagicMock(spec=["data"])
        handler._set_fields(msg, {"nonexistent": 42})
        # No exception should be raised; only known fields are touched.

    def test_sets_multiple_fields(self, handler):
        msg = MagicMock()
        msg.a = 0
        msg.b = ""
        handler._set_fields(msg, {"a": 1, "b": "hello"})
        assert msg.a == 1
        assert msg.b == "hello"

    def test_deeply_nested_fields(self, handler):
        """Three levels deep: outer.mid.inner.val."""
        inner = MagicMock()
        inner.val = 0
        mid = MagicMock()
        mid.inner = inner
        outer = MagicMock()
        outer.mid = mid
        handler._set_fields(outer, {"mid": {"inner": {"val": 42}}})
        assert inner.val == 42

    def test_empty_dict_is_noop(self, handler):
        msg = MagicMock()
        msg.x = 1.0
        handler._set_fields(msg, {})
        assert msg.x == 1.0


# ---------------------------------------------------------------------------
# call – success path
# ---------------------------------------------------------------------------

class TestCallSuccess:
    """Tests for successful service calls."""

    @patch("physical_mcp_bridge.service_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_call_returns_response_dict(self, mock_get_service, mock_to_dict, handler, mock_node):
        """A successful call should return the OrderedDict of the response."""
        mock_srv_class = MagicMock()
        mock_request = MagicMock()
        mock_srv_class.Request.return_value = mock_request
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        mock_response = MagicMock()
        expected = OrderedDict({"success": True, "message": "ok"})
        mock_to_dict.return_value = expected

        # Make the future immediately done
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response

        def fake_add_done_callback(cb):
            cb(mock_future)

        mock_future.add_done_callback.side_effect = fake_add_done_callback
        mock_client.call_async.return_value = mock_future

        result = handler.call("/set_bool", "std_srvs/srv/SetBool", {"data": True})
        assert result == expected

    @patch("physical_mcp_bridge.service_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_call_creates_client_with_correct_params(self, mock_get_service, mock_to_dict, handler, mock_node):
        """The client should be created with the correct service class and name."""
        mock_srv_class = MagicMock()
        mock_srv_class.Request.return_value = MagicMock()
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = MagicMock()
        mock_future.add_done_callback.side_effect = lambda cb: cb(mock_future)
        mock_client.call_async.return_value = mock_future

        mock_to_dict.return_value = OrderedDict()

        handler.call("/trigger", "std_srvs/srv/Trigger", {})
        mock_node.create_client.assert_called_once_with(mock_srv_class, "/trigger")

    @patch("physical_mcp_bridge.service_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_call_sets_request_fields(self, mock_get_service, mock_to_dict, handler, mock_node):
        """Arguments should be applied to the request message."""
        mock_srv_class = MagicMock()
        mock_request = MagicMock()
        mock_request.data = False
        mock_srv_class.Request.return_value = mock_request
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = MagicMock()
        mock_future.add_done_callback.side_effect = lambda cb: cb(mock_future)
        mock_client.call_async.return_value = mock_future

        mock_to_dict.return_value = OrderedDict()

        handler.call("/set_bool", "std_srvs/srv/SetBool", {"data": True})
        assert mock_request.data is True

    @patch("physical_mcp_bridge.service_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_call_passes_request_to_call_async(self, mock_get_service, mock_to_dict, handler, mock_node):
        """call_async must be invoked with the request object."""
        mock_srv_class = MagicMock()
        mock_request = MagicMock()
        mock_srv_class.Request.return_value = mock_request
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = MagicMock()
        mock_future.add_done_callback.side_effect = lambda cb: cb(mock_future)
        mock_client.call_async.return_value = mock_future

        mock_to_dict.return_value = OrderedDict()

        handler.call("/srv", "pkg/srv/Type", {})
        mock_client.call_async.assert_called_once_with(mock_request)


# ---------------------------------------------------------------------------
# call – service unavailable
# ---------------------------------------------------------------------------

class TestCallServiceUnavailable:
    """Tests when the service is not available."""

    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_call_returns_error_when_service_unavailable(self, mock_get_service, handler, mock_node):
        """When wait_for_service returns False, an error dict is returned."""
        mock_srv_class = MagicMock()
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = False
        mock_node.create_client.return_value = mock_client

        result = handler.call("/missing", "pkg/srv/Missing", {})
        assert "error" in result
        assert "not available" in result["error"]

    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_unavailable_still_destroys_client(self, mock_get_service, handler, mock_node):
        """The client must be destroyed even when the service is unavailable."""
        mock_get_service.return_value = MagicMock()

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = False
        mock_node.create_client.return_value = mock_client

        handler.call("/missing", "pkg/srv/Missing", {})
        mock_node.destroy_client.assert_called_once_with(mock_client)


# ---------------------------------------------------------------------------
# call – timeout
# ---------------------------------------------------------------------------

class TestCallTimeout:
    """Tests when the service call times out."""

    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_call_returns_timeout_error(self, mock_get_service, handler, mock_node):
        """When the future is not done before timeout, return a timeout error."""
        mock_srv_class = MagicMock()
        mock_srv_class.Request.return_value = MagicMock()
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        # Future never completes
        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_future.add_done_callback.side_effect = lambda cb: None  # never fires
        mock_client.call_async.return_value = mock_future

        result = handler.call("/slow", "pkg/srv/Slow", {}, timeout_sec=0.01)
        assert "error" in result
        assert "timed out" in result["error"]

    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_timeout_includes_timeout_value(self, mock_get_service, handler, mock_node):
        """The timeout error message should contain the timeout duration."""
        mock_srv_class = MagicMock()
        mock_srv_class.Request.return_value = MagicMock()
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_future.add_done_callback.side_effect = lambda cb: None
        mock_client.call_async.return_value = mock_future

        result = handler.call("/slow", "pkg/srv/Slow", {}, timeout_sec=7.5)
        assert "7.5" in result["error"]

    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_timeout_still_destroys_client(self, mock_get_service, handler, mock_node):
        """The client must be destroyed even on timeout."""
        mock_srv_class = MagicMock()
        mock_srv_class.Request.return_value = MagicMock()
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_future.add_done_callback.side_effect = lambda cb: None
        mock_client.call_async.return_value = mock_future

        handler.call("/slow", "pkg/srv/Slow", {}, timeout_sec=0.01)
        mock_node.destroy_client.assert_called_once_with(mock_client)


# ---------------------------------------------------------------------------
# call – cleanup guarantee
# ---------------------------------------------------------------------------

class TestCallCleanup:
    """Tests that the client is always destroyed in the finally block."""

    @patch("physical_mcp_bridge.service_handler.message_to_ordereddict")
    @patch("physical_mcp_bridge.service_handler.get_service")
    def test_client_destroyed_on_success(self, mock_get_service, mock_to_dict, handler, mock_node):
        mock_srv_class = MagicMock()
        mock_srv_class.Request.return_value = MagicMock()
        mock_get_service.return_value = mock_srv_class

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_node.create_client.return_value = mock_client

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = MagicMock()
        mock_future.add_done_callback.side_effect = lambda cb: cb(mock_future)
        mock_client.call_async.return_value = mock_future

        mock_to_dict.return_value = OrderedDict()

        handler.call("/srv", "pkg/srv/T", {})
        mock_node.destroy_client.assert_called_once_with(mock_client)

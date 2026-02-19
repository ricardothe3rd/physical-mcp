"""Tests for the WebSocket connection manager.

Uses pytest + unittest.mock, following the same patterns as the rest of the
test suite (test_telemetry, test_graceful, etc.).
"""

import time
from unittest.mock import MagicMock

import pytest

from physical_mcp_bridge.ws_manager import (
    BackpressureMonitor,
    ClientInfo,
    WebSocketManager,
)


# ---------------------------------------------------------------------------
# ClientInfo creation
# ---------------------------------------------------------------------------

class TestClientInfo:
    def test_add_client_creates_client_info_with_defaults(self):
        mgr = WebSocketManager()
        ws = MagicMock()
        info = mgr.add_client(ws)

        assert info is not None
        assert isinstance(info, ClientInfo)
        assert info.websocket is ws
        assert info.message_count == 0
        assert info.bytes_sent == 0
        assert info.bytes_received == 0
        assert info.is_authenticated is False
        assert isinstance(info.client_id, str)
        assert len(info.client_id) > 0
        assert info.connected_at > 0
        assert info.last_message_at > 0

    def test_add_client_unique_ids(self):
        mgr = WebSocketManager(max_clients=10)
        ids = {mgr.add_client(MagicMock()).client_id for _ in range(5)}
        assert len(ids) == 5


# ---------------------------------------------------------------------------
# Capacity enforcement
# ---------------------------------------------------------------------------

class TestCapacity:
    def test_add_client_at_capacity_returns_none(self):
        mgr = WebSocketManager(max_clients=2)
        mgr.add_client(MagicMock())
        mgr.add_client(MagicMock())
        result = mgr.add_client(MagicMock())
        assert result is None

    def test_is_at_capacity_true(self):
        mgr = WebSocketManager(max_clients=1)
        mgr.add_client(MagicMock())
        assert mgr.is_at_capacity() is True

    def test_is_at_capacity_false(self):
        mgr = WebSocketManager(max_clients=5)
        mgr.add_client(MagicMock())
        assert mgr.is_at_capacity() is False

    def test_max_clients_enforcement(self):
        mgr = WebSocketManager(max_clients=3)
        results = [mgr.add_client(MagicMock()) for _ in range(5)]
        # First 3 succeed, last 2 fail
        assert results[0] is not None
        assert results[1] is not None
        assert results[2] is not None
        assert results[3] is None
        assert results[4] is None
        assert mgr.get_client_count() == 3


# ---------------------------------------------------------------------------
# Client lookup / removal
# ---------------------------------------------------------------------------

class TestClientLookup:
    def test_remove_client_existing(self):
        mgr = WebSocketManager()
        info = mgr.add_client(MagicMock())
        assert mgr.remove_client(info.client_id) is True
        assert mgr.get_client_count() == 0

    def test_remove_client_nonexistent(self):
        mgr = WebSocketManager()
        assert mgr.remove_client("no-such-id") is False

    def test_get_client_existing(self):
        mgr = WebSocketManager()
        info = mgr.add_client(MagicMock())
        found = mgr.get_client(info.client_id)
        assert found is info

    def test_get_client_nonexistent(self):
        mgr = WebSocketManager()
        assert mgr.get_client("missing") is None

    def test_list_clients_returns_all(self):
        mgr = WebSocketManager(max_clients=10)
        ids = set()
        for _ in range(4):
            info = mgr.add_client(MagicMock())
            ids.add(info.client_id)
        listed_ids = {c.client_id for c in mgr.list_clients()}
        assert listed_ids == ids

    def test_get_client_count(self):
        mgr = WebSocketManager(max_clients=10)
        assert mgr.get_client_count() == 0
        mgr.add_client(MagicMock())
        assert mgr.get_client_count() == 1
        mgr.add_client(MagicMock())
        assert mgr.get_client_count() == 2


# ---------------------------------------------------------------------------
# Rate limiting
# ---------------------------------------------------------------------------

class TestRateLimit:
    def test_check_rate_limit_allows_within_limit(self):
        mgr = WebSocketManager(rate_limit_per_second=5)
        info = mgr.add_client(MagicMock())
        for _ in range(5):
            assert mgr.check_rate_limit(info.client_id) is True

    def test_check_rate_limit_blocks_over_limit(self):
        mgr = WebSocketManager(rate_limit_per_second=3)
        info = mgr.add_client(MagicMock())
        # Use up the limit
        for _ in range(3):
            mgr.check_rate_limit(info.client_id)
        # Next call should be rejected
        assert mgr.check_rate_limit(info.client_id) is False


# ---------------------------------------------------------------------------
# Message tracking
# ---------------------------------------------------------------------------

class TestMessageTracking:
    def test_record_message_updates_sent_stats(self):
        mgr = WebSocketManager()
        info = mgr.add_client(MagicMock())
        mgr.record_message(info.client_id, 128, "sent")
        mgr.record_message(info.client_id, 256, "sent")

        client = mgr.get_client(info.client_id)
        assert client.message_count == 2
        assert client.bytes_sent == 384
        assert client.bytes_received == 0

    def test_record_message_updates_received_stats(self):
        mgr = WebSocketManager()
        info = mgr.add_client(MagicMock())
        mgr.record_message(info.client_id, 64, "received")

        client = mgr.get_client(info.client_id)
        assert client.message_count == 1
        assert client.bytes_received == 64
        assert client.bytes_sent == 0

    def test_record_message_ignores_unknown_client(self):
        mgr = WebSocketManager()
        # Should not raise
        mgr.record_message("unknown-id", 100, "sent")

    def test_get_stats_returns_correct_totals(self):
        mgr = WebSocketManager(max_clients=10)
        c1 = mgr.add_client(MagicMock())
        c2 = mgr.add_client(MagicMock())

        mgr.record_message(c1.client_id, 100, "sent")
        mgr.record_message(c1.client_id, 200, "received")
        mgr.record_message(c2.client_id, 50, "sent")

        stats = mgr.get_stats()
        assert stats["total_clients"] == 2
        assert stats["max_clients"] == 10
        assert stats["total_messages_sent"] == 2
        assert stats["total_messages_received"] == 1
        assert stats["total_bytes_sent"] == 150
        assert stats["total_bytes_received"] == 200


# ---------------------------------------------------------------------------
# Broadcast
# ---------------------------------------------------------------------------

class TestBroadcast:
    def test_broadcast_sends_to_all(self):
        mgr = WebSocketManager(max_clients=5)
        ws1, ws2, ws3 = MagicMock(), MagicMock(), MagicMock()
        mgr.add_client(ws1)
        mgr.add_client(ws2)
        mgr.add_client(ws3)

        mgr.broadcast("hello")
        ws1.send.assert_called_once_with("hello")
        ws2.send.assert_called_once_with("hello")
        ws3.send.assert_called_once_with("hello")

    def test_broadcast_excludes_client(self):
        mgr = WebSocketManager(max_clients=5)
        ws1, ws2 = MagicMock(), MagicMock()
        c1 = mgr.add_client(ws1)
        mgr.add_client(ws2)

        targeted = mgr.broadcast("msg", exclude=c1.client_id)
        ws1.send.assert_not_called()
        ws2.send.assert_called_once_with("msg")
        assert c1.client_id not in targeted


# ---------------------------------------------------------------------------
# Backpressure
# ---------------------------------------------------------------------------

class TestBackpressureMonitor:
    def test_ok_below_high_watermark(self):
        bp = BackpressureMonitor(high_watermark=80, low_watermark=20)
        assert bp.check(10, 100) == "ok"
        assert bp.check(0, 100) == "ok"
        assert bp.check(79, 100) == "ok"

    def test_warning_at_high_watermark(self):
        bp = BackpressureMonitor(high_watermark=80, low_watermark=20)
        assert bp.check(80, 100) == "warning"
        assert bp.check(90, 100) == "warning"
        assert bp.check(99, 100) == "warning"

    def test_critical_at_max(self):
        bp = BackpressureMonitor(high_watermark=80, low_watermark=20)
        assert bp.check(100, 100) == "critical"
        assert bp.check(150, 100) == "critical"

    def test_should_drop_true_at_critical(self):
        bp = BackpressureMonitor(high_watermark=80, low_watermark=20)
        assert bp.should_drop(100, 100) is True
        assert bp.should_drop(200, 100) is True

    def test_should_drop_false_below_critical(self):
        bp = BackpressureMonitor(high_watermark=80, low_watermark=20)
        assert bp.should_drop(50, 100) is False
        assert bp.should_drop(90, 100) is False

    def test_critical_when_max_queue_zero(self):
        bp = BackpressureMonitor()
        assert bp.check(0, 0) == "critical"
        assert bp.should_drop(0, 0) is True


class TestWebSocketManagerBackpressure:
    def test_check_backpressure_false_by_default(self):
        mgr = WebSocketManager(max_queue_size=100)
        info = mgr.add_client(MagicMock())
        assert mgr.check_backpressure(info.client_id) is False

    def test_check_backpressure_true_when_over(self):
        mgr = WebSocketManager(max_queue_size=100)
        info = mgr.add_client(MagicMock())
        mgr.set_queue_size(info.client_id, 101)
        assert mgr.check_backpressure(info.client_id) is True

    def test_check_backpressure_false_at_boundary(self):
        mgr = WebSocketManager(max_queue_size=100)
        info = mgr.add_client(MagicMock())
        mgr.set_queue_size(info.client_id, 100)
        assert mgr.check_backpressure(info.client_id) is False

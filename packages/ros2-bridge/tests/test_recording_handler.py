"""Tests for the topic recording handler.

No ROS2 dependencies are needed; RecordingHandler is a pure-Python module.
"""

import time

import pytest

from physical_mcp_bridge.recording_handler import (
    RecordingHandler,
    RecordingSession,
    RecordedMessage,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def handler():
    """Return a RecordingHandler with default settings."""
    return RecordingHandler()


@pytest.fixture
def small_handler():
    """Return a RecordingHandler limited to 2 sessions."""
    return RecordingHandler(max_sessions=2)


# ---------------------------------------------------------------------------
# start_recording
# ---------------------------------------------------------------------------

class TestStartRecording:
    """Tests for the start_recording method."""

    def test_creates_session_with_defaults(self, handler):
        session_id = handler.start_recording()
        session = handler.get_session(session_id)
        assert session is not None
        assert session.status == "recording"
        assert session.started_at is not None
        assert session.topics == []
        assert session.max_messages == 1000
        assert session.max_duration_s == 60.0
        assert session.messages == []

    def test_returns_session_id_starting_with_rec(self, handler):
        session_id = handler.start_recording()
        assert session_id.startswith("rec_")

    def test_respects_max_sessions_limit(self, small_handler):
        small_handler.start_recording()
        small_handler.start_recording()
        with pytest.raises(RuntimeError, match="Maximum number of sessions"):
            small_handler.start_recording()

    def test_creates_session_with_custom_topics(self, handler):
        session_id = handler.start_recording(topics=["/odom", "/cmd_vel"])
        session = handler.get_session(session_id)
        assert session.topics == ["/odom", "/cmd_vel"]

    def test_creates_session_with_custom_limits(self, handler):
        session_id = handler.start_recording(max_messages=50, max_duration_s=10.0)
        session = handler.get_session(session_id)
        assert session.max_messages == 50
        assert session.max_duration_s == 10.0

    def test_session_id_is_unique(self, handler):
        id1 = handler.start_recording()
        # Small sleep to ensure timestamp-based IDs differ.
        time.sleep(0.002)
        id2 = handler.start_recording()
        assert id1 != id2


# ---------------------------------------------------------------------------
# stop_recording
# ---------------------------------------------------------------------------

class TestStopRecording:
    """Tests for the stop_recording method."""

    def test_stops_active_session(self, handler):
        session_id = handler.start_recording()
        result = handler.stop_recording(session_id)
        assert result is True
        session = handler.get_session(session_id)
        assert session.status == "stopped"
        assert session.stopped_at is not None

    def test_returns_false_for_unknown_session(self, handler):
        result = handler.stop_recording("rec_nonexistent")
        assert result is False

    def test_returns_false_for_already_stopped_session(self, handler):
        session_id = handler.start_recording()
        handler.stop_recording(session_id)
        result = handler.stop_recording(session_id)
        assert result is False


# ---------------------------------------------------------------------------
# record_message
# ---------------------------------------------------------------------------

class TestRecordMessage:
    """Tests for the record_message method."""

    def test_adds_message_to_active_session(self, handler):
        session_id = handler.start_recording()
        handler.record_message("/odom", {"x": 1.0})
        messages = handler.get_messages(session_id)
        assert len(messages) == 1
        assert messages[0]["topic"] == "/odom"
        assert messages[0]["data"] == {"x": 1.0}
        assert messages[0]["sequence_number"] == 1

    def test_ignores_non_recording_sessions(self, handler):
        session_id = handler.start_recording()
        handler.stop_recording(session_id)
        handler.record_message("/odom", {"x": 1.0})
        messages = handler.get_messages(session_id)
        assert len(messages) == 0

    def test_respects_topic_filter(self, handler):
        session_id = handler.start_recording(topics=["/odom"])
        handler.record_message("/odom", {"x": 1.0})
        handler.record_message("/cmd_vel", {"linear": 0.5})
        messages = handler.get_messages(session_id)
        assert len(messages) == 1
        assert messages[0]["topic"] == "/odom"

    def test_records_to_all_matching_active_sessions(self, handler):
        id1 = handler.start_recording()
        id2 = handler.start_recording()
        handler.record_message("/odom", {"x": 1.0})
        msgs1 = handler.get_messages(id1)
        msgs2 = handler.get_messages(id2)
        assert len(msgs1) == 1
        assert len(msgs2) == 1

    def test_auto_stops_when_max_messages_reached(self, handler):
        session_id = handler.start_recording(max_messages=3)
        for i in range(5):
            handler.record_message("/odom", {"seq": i})
        session = handler.get_session(session_id)
        assert session.status == "stopped"
        messages = handler.get_messages(session_id)
        assert len(messages) == 3

    def test_sequence_numbers_increment(self, handler):
        session_id = handler.start_recording()
        handler.record_message("/a", {"v": 1})
        handler.record_message("/b", {"v": 2})
        handler.record_message("/c", {"v": 3})
        messages = handler.get_messages(session_id)
        assert [m["sequence_number"] for m in messages] == [1, 2, 3]

    def test_empty_topics_records_everything(self, handler):
        session_id = handler.start_recording(topics=[])
        handler.record_message("/odom", {"x": 1.0})
        handler.record_message("/cmd_vel", {"v": 0.5})
        handler.record_message("/scan", {"ranges": [1, 2, 3]})
        messages = handler.get_messages(session_id)
        assert len(messages) == 3


# ---------------------------------------------------------------------------
# get_session
# ---------------------------------------------------------------------------

class TestGetSession:
    """Tests for the get_session method."""

    def test_returns_session_by_id(self, handler):
        session_id = handler.start_recording()
        session = handler.get_session(session_id)
        assert session is not None
        assert session.session_id == session_id

    def test_returns_none_for_unknown_id(self, handler):
        session = handler.get_session("rec_nonexistent")
        assert session is None


# ---------------------------------------------------------------------------
# get_session_stats
# ---------------------------------------------------------------------------

class TestGetSessionStats:
    """Tests for the get_session_stats method."""

    def test_returns_correct_stats(self, handler):
        session_id = handler.start_recording(
            topics=["/odom"], max_messages=100, max_duration_s=30.0
        )
        handler.record_message("/odom", {"x": 1.0})
        handler.record_message("/odom", {"x": 2.0})

        stats = handler.get_session_stats(session_id)
        assert stats is not None
        assert stats["session_id"] == session_id
        assert stats["status"] == "recording"
        assert stats["message_count"] == 2
        assert stats["duration_s"] is not None
        assert stats["duration_s"] >= 0
        assert stats["topics_recorded"] == ["/odom"]
        assert stats["max_messages"] == 100
        assert stats["max_duration_s"] == 30.0

    def test_returns_none_for_unknown_id(self, handler):
        stats = handler.get_session_stats("rec_nonexistent")
        assert stats is None

    def test_stopped_session_has_fixed_duration(self, handler):
        session_id = handler.start_recording()
        handler.record_message("/odom", {"x": 1.0})
        handler.stop_recording(session_id)
        stats1 = handler.get_session_stats(session_id)
        time.sleep(0.05)
        stats2 = handler.get_session_stats(session_id)
        # Duration should not keep growing after stop.
        assert stats1["duration_s"] == stats2["duration_s"]


# ---------------------------------------------------------------------------
# get_messages
# ---------------------------------------------------------------------------

class TestGetMessages:
    """Tests for the get_messages method."""

    def test_returns_recorded_messages_as_dicts(self, handler):
        session_id = handler.start_recording()
        handler.record_message("/odom", {"x": 1.0})
        messages = handler.get_messages(session_id)
        assert len(messages) == 1
        msg = messages[0]
        assert isinstance(msg, dict)
        assert "topic" in msg
        assert "timestamp" in msg
        assert "sequence_number" in msg
        assert "data" in msg

    def test_filters_by_topic(self, handler):
        session_id = handler.start_recording()
        handler.record_message("/odom", {"x": 1.0})
        handler.record_message("/cmd_vel", {"v": 0.5})
        handler.record_message("/odom", {"x": 2.0})

        odom_msgs = handler.get_messages(session_id, topic="/odom")
        assert len(odom_msgs) == 2
        assert all(m["topic"] == "/odom" for m in odom_msgs)

        vel_msgs = handler.get_messages(session_id, topic="/cmd_vel")
        assert len(vel_msgs) == 1
        assert vel_msgs[0]["topic"] == "/cmd_vel"

    def test_returns_empty_for_unknown_session(self, handler):
        messages = handler.get_messages("rec_nonexistent")
        assert messages == []


# ---------------------------------------------------------------------------
# list_sessions
# ---------------------------------------------------------------------------

class TestListSessions:
    """Tests for the list_sessions method."""

    def test_returns_all_sessions_with_basic_info(self, handler):
        id1 = handler.start_recording()
        time.sleep(0.002)
        id2 = handler.start_recording()
        handler.stop_recording(id2)

        sessions = handler.list_sessions()
        assert len(sessions) == 2

        ids = {s["session_id"] for s in sessions}
        assert id1 in ids
        assert id2 in ids

        for s in sessions:
            assert "session_id" in s
            assert "status" in s
            assert "message_count" in s
            assert "started_at" in s
            assert "stopped_at" in s

    def test_returns_empty_when_no_sessions(self, handler):
        sessions = handler.list_sessions()
        assert sessions == []


# ---------------------------------------------------------------------------
# delete_session
# ---------------------------------------------------------------------------

class TestDeleteSession:
    """Tests for the delete_session method."""

    def test_removes_session(self, handler):
        session_id = handler.start_recording()
        result = handler.delete_session(session_id)
        assert result is True
        assert handler.get_session(session_id) is None

    def test_returns_false_for_unknown_session(self, handler):
        result = handler.delete_session("rec_nonexistent")
        assert result is False


# ---------------------------------------------------------------------------
# get_active_session_count
# ---------------------------------------------------------------------------

class TestGetActiveSessionCount:
    """Tests for the get_active_session_count method."""

    def test_returns_correct_count(self, handler):
        assert handler.get_active_session_count() == 0
        id1 = handler.start_recording()
        assert handler.get_active_session_count() == 1
        id2 = handler.start_recording()
        assert handler.get_active_session_count() == 2
        handler.stop_recording(id1)
        assert handler.get_active_session_count() == 1
        handler.stop_recording(id2)
        assert handler.get_active_session_count() == 0

"""Topic recording handler for the ROS2 bridge.

Records messages from specified topics into an in-memory buffer.
Supports start/stop control and data export.
"""

import time
import threading
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class RecordedMessage:
    topic: str
    timestamp: float
    sequence_number: int
    data: Any


@dataclass
class RecordingSession:
    session_id: str
    status: str = "idle"  # idle, recording, stopped
    started_at: Optional[float] = None
    stopped_at: Optional[float] = None
    messages: list = field(default_factory=list)
    topics: list = field(default_factory=list)  # empty = record all
    max_messages: int = 1000
    max_duration_s: float = 60.0
    _sequence: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock)


class RecordingHandler:
    """Manages topic recording sessions."""

    def __init__(self, max_sessions: int = 10):
        self._sessions: dict[str, RecordingSession] = {}
        self._max_sessions = max_sessions
        self._lock = threading.Lock()
        self._id_counter = 0

    def start_recording(
        self,
        topics: list[str] | None = None,
        max_messages: int = 1000,
        max_duration_s: float = 60.0,
    ) -> str:
        """Start a new recording session. Returns session ID."""
        with self._lock:
            if len(self._sessions) >= self._max_sessions:
                raise RuntimeError(
                    f"Maximum number of sessions ({self._max_sessions}) reached"
                )

            self._id_counter += 1
            session_id = f"rec_{int(time.time() * 1000)}_{self._id_counter}"
            session = RecordingSession(
                session_id=session_id,
                status="recording",
                started_at=time.time(),
                topics=topics if topics is not None else [],
                max_messages=max_messages,
                max_duration_s=max_duration_s,
            )
            self._sessions[session_id] = session
            return session_id

    def stop_recording(self, session_id: str) -> bool:
        """Stop a recording session. Returns True if found and stopped."""
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return False
            with session._lock:
                if session.status != "recording":
                    return False
                session.status = "stopped"
                session.stopped_at = time.time()
                return True

    def record_message(self, topic: str, data: Any) -> None:
        """Record a message to all active sessions that match the topic."""
        with self._lock:
            active_sessions = [
                s for s in self._sessions.values() if s.status == "recording"
            ]

        for session in active_sessions:
            with session._lock:
                # Re-check status under the session lock (may have been stopped).
                if session.status != "recording":
                    continue

                # Check topic filter: empty list means record everything.
                if session.topics and topic not in session.topics:
                    continue

                # Check max_messages limit.
                if len(session.messages) >= session.max_messages:
                    session.status = "stopped"
                    session.stopped_at = time.time()
                    continue

                # Check max_duration limit.
                now = time.time()
                if session.started_at is not None and (
                    now - session.started_at >= session.max_duration_s
                ):
                    session.status = "stopped"
                    session.stopped_at = now
                    continue

                session._sequence += 1
                message = RecordedMessage(
                    topic=topic,
                    timestamp=now,
                    sequence_number=session._sequence,
                    data=data,
                )
                session.messages.append(message)

    def get_session(self, session_id: str) -> Optional[RecordingSession]:
        """Get a recording session by ID."""
        with self._lock:
            return self._sessions.get(session_id)

    def get_session_stats(self, session_id: str) -> Optional[dict]:
        """Get stats for a recording session."""
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None

        with session._lock:
            duration = None
            if session.started_at is not None:
                end = session.stopped_at if session.stopped_at else time.time()
                duration = round(end - session.started_at, 3)

            unique_topics = list({m.topic for m in session.messages})

            return {
                "session_id": session.session_id,
                "status": session.status,
                "message_count": len(session.messages),
                "duration_s": duration,
                "topics_recorded": unique_topics,
                "max_messages": session.max_messages,
                "max_duration_s": session.max_duration_s,
            }

    def get_messages(self, session_id: str, topic: str | None = None) -> list[dict]:
        """Get recorded messages, optionally filtered by topic."""
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return []

        with session._lock:
            messages = session.messages
            if topic is not None:
                messages = [m for m in messages if m.topic == topic]

            return [
                {
                    "topic": m.topic,
                    "timestamp": m.timestamp,
                    "sequence_number": m.sequence_number,
                    "data": m.data,
                }
                for m in messages
            ]

    def list_sessions(self) -> list[dict]:
        """List all recording sessions with basic info."""
        with self._lock:
            sessions = list(self._sessions.values())

        result = []
        for session in sessions:
            with session._lock:
                result.append(
                    {
                        "session_id": session.session_id,
                        "status": session.status,
                        "message_count": len(session.messages),
                        "started_at": session.started_at,
                        "stopped_at": session.stopped_at,
                    }
                )
        return result

    def delete_session(self, session_id: str) -> bool:
        """Delete a recording session."""
        with self._lock:
            if session_id not in self._sessions:
                return False
            del self._sessions[session_id]
            return True

    def get_active_session_count(self) -> int:
        """Count currently recording sessions."""
        with self._lock:
            return sum(
                1 for s in self._sessions.values() if s.status == "recording"
            )

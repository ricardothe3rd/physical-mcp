"""WebSocket connection manager with backpressure and concurrent client support.

Manages multiple WebSocket client connections, tracks per-client metrics,
enforces rate limits, and monitors backpressure to prevent slow consumers
from degrading the bridge.
"""

import time
import uuid
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ClientInfo:
    """Metadata and statistics for a single WebSocket client."""

    client_id: str
    websocket: Any
    connected_at: float
    last_message_at: float
    message_count: int = 0
    bytes_sent: int = 0
    bytes_received: int = 0
    is_authenticated: bool = False


class BackpressureMonitor:
    """Monitors queue depth and signals when to apply backpressure.

    Uses a high/low watermark scheme:
      - Below high_watermark %  -> "ok"
      - Between high and 100%  -> "warning"
      - At or above 100%       -> "critical"
    """

    def __init__(self, high_watermark: int = 80, low_watermark: int = 20):
        self.high_watermark = high_watermark
        self.low_watermark = low_watermark

    def check(self, current_queue_size: int, max_queue_size: int) -> str:
        """Return the backpressure status for the given queue depth.

        Returns one of ``"ok"``, ``"warning"``, or ``"critical"``.
        """
        if max_queue_size <= 0:
            return "critical"

        pct = (current_queue_size / max_queue_size) * 100

        if pct >= 100:
            return "critical"
        if pct >= self.high_watermark:
            return "warning"
        return "ok"

    def should_drop(self, current_queue_size: int, max_queue_size: int) -> bool:
        """Return ``True`` when the queue is critically full and messages
        should be dropped rather than enqueued."""
        return self.check(current_queue_size, max_queue_size) == "critical"


class WebSocketManager:
    """Manages concurrent WebSocket client connections.

    Features:
      - Client registration / deregistration with auto-generated IDs
      - Per-client message tracking and byte accounting
      - Token-bucket rate limiting per client
      - Backpressure monitoring per client queue
      - Broadcast to all connected clients
    """

    def __init__(
        self,
        max_clients: int = 5,
        max_queue_size: int = 100,
        rate_limit_per_second: int = 50,
    ):
        self.max_clients = max_clients
        self.max_queue_size = max_queue_size
        self.rate_limit_per_second = rate_limit_per_second

        # client_id -> ClientInfo
        self._clients: dict[str, ClientInfo] = {}

        # Rate-limit tracking: client_id -> deque of timestamps
        self._rate_windows: dict[str, deque] = defaultdict(deque)

        # Simulated per-client queue sizes for backpressure monitoring
        self._queue_sizes: dict[str, int] = defaultdict(int)

        # Global counters
        self._total_messages_sent: int = 0
        self._total_messages_received: int = 0
        self._total_bytes_sent: int = 0
        self._total_bytes_received: int = 0

        self.backpressure = BackpressureMonitor()

    # ── Client lifecycle ────────────────────────────────────────────────

    def add_client(self, websocket: Any) -> ClientInfo | None:
        """Register a new WebSocket connection.

        Returns ``None`` if the server is already at capacity.
        """
        if self.is_at_capacity():
            return None

        now = time.time()
        client_id = str(uuid.uuid4())
        info = ClientInfo(
            client_id=client_id,
            websocket=websocket,
            connected_at=now,
            last_message_at=now,
        )
        self._clients[client_id] = info
        return info

    def remove_client(self, client_id: str) -> bool:
        """Deregister a client.  Returns ``True`` if the client existed."""
        if client_id in self._clients:
            del self._clients[client_id]
            self._rate_windows.pop(client_id, None)
            self._queue_sizes.pop(client_id, None)
            return True
        return False

    def get_client(self, client_id: str) -> ClientInfo | None:
        """Look up a client by ID."""
        return self._clients.get(client_id)

    def list_clients(self) -> list[ClientInfo]:
        """Return a list of all connected clients."""
        return list(self._clients.values())

    def get_client_count(self) -> int:
        """Return the number of currently connected clients."""
        return len(self._clients)

    def is_at_capacity(self) -> bool:
        """Return ``True`` when the maximum number of clients is reached."""
        return len(self._clients) >= self.max_clients

    # ── Rate limiting ───────────────────────────────────────────────────

    def check_rate_limit(self, client_id: str) -> bool:
        """Return ``True`` if the client is within the per-second rate limit.

        Uses a sliding-window approach: keeps timestamps of recent messages
        and counts how many fall within the last second.
        """
        now = time.time()
        window = self._rate_windows[client_id]

        # Expire timestamps older than 1 second
        while window and window[0] <= now - 1.0:
            window.popleft()

        if len(window) >= self.rate_limit_per_second:
            return False

        window.append(now)
        return True

    # ── Message tracking ────────────────────────────────────────────────

    def record_message(self, client_id: str, size_bytes: int, direction: str) -> None:
        """Record a message for the given client.

        ``direction`` must be ``"sent"`` or ``"received"``.
        """
        client = self._clients.get(client_id)
        if client is None:
            return

        client.message_count += 1
        client.last_message_at = time.time()

        if direction == "sent":
            client.bytes_sent += size_bytes
            self._total_messages_sent += 1
            self._total_bytes_sent += size_bytes
        elif direction == "received":
            client.bytes_received += size_bytes
            self._total_messages_received += 1
            self._total_bytes_received += size_bytes

    # ── Backpressure ────────────────────────────────────────────────────

    def check_backpressure(self, client_id: str) -> bool:
        """Return ``True`` if the client's queue exceeds ``max_queue_size``."""
        return self._queue_sizes.get(client_id, 0) > self.max_queue_size

    def set_queue_size(self, client_id: str, size: int) -> None:
        """Update the tracked queue depth for a client."""
        self._queue_sizes[client_id] = size

    # ── Broadcast ───────────────────────────────────────────────────────

    def broadcast(self, message: str, exclude: str | None = None) -> list[str]:
        """Send *message* to every connected client except *exclude*.

        Because the underlying WebSocket ``send()`` is async, this method
        **queues** the message by calling each websocket's ``send`` method
        and returns the list of client IDs that were targeted.  Callers
        running inside an event loop should ``await`` the coroutines if
        needed.

        Returns the list of client IDs the message was sent to.
        """
        targeted: list[str] = []
        for client_id, info in self._clients.items():
            if client_id == exclude:
                continue
            try:
                info.websocket.send(message)
                targeted.append(client_id)
            except Exception:
                pass
        return targeted

    # ── Stats ───────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        """Return aggregate statistics across all clients."""
        return {
            "total_clients": self.get_client_count(),
            "max_clients": self.max_clients,
            "total_messages_sent": self._total_messages_sent,
            "total_messages_received": self._total_messages_received,
            "total_bytes_sent": self._total_bytes_sent,
            "total_bytes_received": self._total_bytes_received,
        }

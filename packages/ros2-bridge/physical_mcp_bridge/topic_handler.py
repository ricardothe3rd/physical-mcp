"""ROS2 topic operations: subscribe, publish, echo."""

import json
import threading
import time
from typing import Any, Optional

from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict


class TopicHandler:
    """Handles topic subscribe, publish, and echo operations."""

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions: dict[str, Any] = {}

    def subscribe(self, topic: str, message_type: str, count: int = 1, timeout_sec: float = 5.0) -> list[dict]:
        """Subscribe and collect N messages."""
        msg_class = get_message(message_type)
        messages: list[dict] = []
        event = threading.Event()

        def callback(msg):
            messages.append(message_to_ordereddict(msg))
            if len(messages) >= count:
                event.set()

        sub = self.node.create_subscription(msg_class, topic, callback, 10)

        try:
            event.wait(timeout=timeout_sec)
        finally:
            self.node.destroy_subscription(sub)

        return messages

    def echo(self, topic: str, message_type: str, timeout_sec: float = 3.0) -> Optional[dict]:
        """Get the latest message from a topic (one-shot)."""
        messages = self.subscribe(topic, message_type, count=1, timeout_sec=timeout_sec)
        return messages[0] if messages else None

    def publish(self, topic: str, message_type: str, message_data: dict) -> bool:
        """Publish a single message to a topic."""
        msg_class = get_message(message_type)
        msg = msg_class()

        # Set message fields from dict
        self._set_message_fields(msg, message_data)

        pub = self.node.create_publisher(msg_class, topic, 10)

        try:
            # Small delay to let publisher be discovered
            time.sleep(0.1)
            pub.publish(msg)
            return True
        finally:
            self.node.destroy_publisher(pub)

    def _set_message_fields(self, msg: Any, data: dict) -> None:
        """Recursively set message fields from a dictionary."""
        for key, value in data.items():
            if not hasattr(msg, key):
                continue
            field = getattr(msg, key)
            if isinstance(value, dict):
                self._set_message_fields(field, value)
            else:
                setattr(msg, key, value)

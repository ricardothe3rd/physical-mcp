"""ROS2 service call handler."""

import threading
from typing import Any, Optional

from rclpy.node import Node
from rosidl_runtime_py.utilities import get_service
from rosidl_runtime_py import message_to_ordereddict


class ServiceHandler:
    """Handles ROS2 service calls."""

    def __init__(self, node: Node):
        self.node = node

    def call(self, service_name: str, service_type: str, args: dict, timeout_sec: float = 10.0) -> dict:
        """Call a ROS2 service and return the response."""
        srv_class = get_service(service_type)
        client = self.node.create_client(srv_class, service_name)

        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return {'error': f'Service {service_name} not available'}

            request = srv_class.Request()
            self._set_fields(request, args)

            future = client.call_async(request)

            # Spin until complete or timeout
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            event.wait(timeout=timeout_sec)

            if future.done():
                response = future.result()
                return message_to_ordereddict(response)
            else:
                return {'error': f'Service call timed out after {timeout_sec}s'}
        finally:
            self.node.destroy_client(client)

    def _set_fields(self, msg: Any, data: dict) -> None:
        """Recursively set message fields from a dictionary."""
        for key, value in data.items():
            if not hasattr(msg, key):
                continue
            field = getattr(msg, key)
            if isinstance(value, dict):
                self._set_fields(field, value)
            else:
                setattr(msg, key, value)

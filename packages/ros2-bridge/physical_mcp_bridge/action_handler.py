"""ROS2 action client handler."""

import threading
from typing import Any, Optional

from rclpy.node import Node
from rclpy.action import ActionClient
from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py import message_to_ordereddict


class ActionHandler:
    """Handles ROS2 action goals."""

    def __init__(self, node: Node):
        self.node = node
        self._active_goals: dict[str, Any] = {}

    def send_goal(self, action_name: str, action_type: str, goal_data: dict, timeout_sec: float = 30.0) -> dict:
        """Send a goal to an action server."""
        action_class = get_action(action_type)
        client = ActionClient(self.node, action_class, action_name)

        try:
            if not client.wait_for_server(timeout_sec=5.0):
                return {'error': f'Action server {action_name} not available'}

            goal = action_class.Goal()
            self._set_fields(goal, goal_data)

            future = client.send_goal_async(goal)
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            event.wait(timeout=timeout_sec)

            if not future.done():
                return {'error': 'Goal submission timed out'}

            goal_handle = future.result()
            if not goal_handle.accepted:
                return {'status': 'rejected'}

            goal_id = str(goal_handle.goal_id)
            self._active_goals[goal_id] = goal_handle

            # Wait for result
            result_future = goal_handle.get_result_async()
            result_event = threading.Event()
            result_future.add_done_callback(lambda _: result_event.set())
            result_event.wait(timeout=timeout_sec)

            if result_future.done():
                result = result_future.result()
                self._active_goals.pop(goal_id, None)
                return {
                    'status': 'completed',
                    'goal_id': goal_id,
                    'result': message_to_ordereddict(result.result),
                }
            else:
                return {
                    'status': 'in_progress',
                    'goal_id': goal_id,
                    'message': 'Goal accepted, result pending',
                }
        finally:
            client.destroy()

    def cancel_goal(self, action_name: str, goal_id: Optional[str] = None) -> dict:
        """Cancel an active goal."""
        if goal_id and goal_id in self._active_goals:
            handle = self._active_goals[goal_id]
            cancel_future = handle.cancel_goal_async()
            # Wait briefly for cancellation
            event = threading.Event()
            cancel_future.add_done_callback(lambda _: event.set())
            event.wait(timeout=5.0)
            self._active_goals.pop(goal_id, None)
            return {'status': 'cancelled', 'goal_id': goal_id}

        # Cancel all goals for this action
        cancelled = list(self._active_goals.keys())
        for gid in cancelled:
            try:
                handle = self._active_goals[gid]
                handle.cancel_goal_async()
                self._active_goals.pop(gid, None)
            except Exception:
                pass
        return {'status': 'cancelled_all', 'cancelled': cancelled}

    def get_status(self, action_name: str) -> dict:
        """Get status of active goals."""
        active = [gid for gid in self._active_goals.keys()]
        return {
            'action': action_name,
            'active_goals': active,
            'count': len(active),
        }

    def cancel_all(self) -> None:
        """Cancel all active goals (used by e-stop)."""
        for gid in list(self._active_goals.keys()):
            try:
                handle = self._active_goals[gid]
                handle.cancel_goal_async()
            except Exception:
                pass
        self._active_goals.clear()

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

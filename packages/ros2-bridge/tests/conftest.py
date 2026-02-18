"""Shared test configuration.

Mocks all ROS2 and related dependencies so every test module can import
the bridge code without having ROS2 installed.  This module is loaded by
pytest before any test file.

The key challenge: ``BridgeNode(Node)`` inherits from ``rclpy.node.Node``.
If ``Node`` is a plain MagicMock *instance*, then ``BridgeNode`` becomes a
MagicMock subclass with broken attribute access.  We solve this by making
``Node`` a normal Python class whose constructor accepts any arguments.
"""

import sys
from unittest.mock import MagicMock


# ── Provide a real (but empty) Node base class ──────────────────────────────

class _FakeNode:
    """Minimal stand-in for rclpy.node.Node.

    Accepts any constructor arguments and exposes a mock ``get_logger()``.
    """

    def __init__(self, *args, **kwargs):
        pass

    def get_logger(self):
        return MagicMock()


# ── Build mock module hierarchy ─────────────────────────────────────────────

_rclpy = MagicMock()
_rclpy_node = MagicMock()
_rclpy_node.Node = _FakeNode      # <-- the important bit

_rclpy_action = MagicMock()

_rosidl_utils = MagicMock()
_rosidl_runtime = MagicMock()

_websockets = MagicMock()
_websockets_server = MagicMock()


# ── Install into sys.modules (setdefault so first writer wins) ──────────────

_modules = {
    "rclpy": _rclpy,
    "rclpy.node": _rclpy_node,
    "rclpy.action": _rclpy_action,
    "rosidl_runtime_py": _rosidl_runtime,
    "rosidl_runtime_py.utilities": _rosidl_utils,
    "websockets": _websockets,
    "websockets.server": _websockets_server,
}

for name, mod in _modules.items():
    sys.modules.setdefault(name, mod)

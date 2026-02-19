"""Tests for the graceful degradation module.

Uses unittest.mock.patch to simulate presence / absence of optional
dependencies (rclpy, websockets) without requiring either to be installed.
"""

import builtins
import types
from unittest.mock import MagicMock, patch

import pytest

# We need to test *actual* import logic inside the graceful module, so we
# must NOT rely on the conftest mocks that inject rclpy / websockets into
# sys.modules.  Instead every test patches ``builtins.__import__`` (or
# sys.modules) so that the graceful helpers see exactly what we want.

from physical_mcp_bridge.graceful import (
    check_rclpy,
    check_websockets,
    check_all_dependencies,
    format_dependency_report,
    ensure_dependencies,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_fake_module(name: str, *, version: str | None = None) -> types.ModuleType:
    """Return a minimal fake module with an optional ``__version__``."""
    mod = types.ModuleType(name)
    if version is not None:
        mod.__version__ = version
    return mod


def _import_raiser(blocked: set[str]):
    """Return an __import__ replacement that raises ImportError for *blocked*.

    All other imports are forwarded to the real built-in import.
    """
    _real_import = builtins.__import__

    def _fake_import(name, *args, **kwargs):
        if name in blocked:
            raise ImportError(f"No module named '{name}'")
        return _real_import(name, *args, **kwargs)

    return _fake_import


# ---------------------------------------------------------------------------
# check_rclpy
# ---------------------------------------------------------------------------

class TestCheckRclpy:
    def test_check_rclpy_available(self):
        fake = _make_fake_module("rclpy", version="1.2.3")
        with patch.dict("sys.modules", {"rclpy": fake}):
            available, message = check_rclpy()
        assert available is True
        assert "rclpy" in message
        assert "1.2.3" in message

    def test_check_rclpy_not_installed(self):
        with patch("builtins.__import__", side_effect=_import_raiser({"rclpy"})):
            with patch.dict("sys.modules", {}, clear=False):
                # Remove any conftest mock of rclpy so the import truly fails.
                import sys
                saved = sys.modules.pop("rclpy", None)
                saved_node = sys.modules.pop("rclpy.node", None)
                saved_action = sys.modules.pop("rclpy.action", None)
                try:
                    available, message = check_rclpy()
                finally:
                    if saved is not None:
                        sys.modules["rclpy"] = saved
                    if saved_node is not None:
                        sys.modules["rclpy.node"] = saved_node
                    if saved_action is not None:
                        sys.modules["rclpy.action"] = saved_action
        assert available is False
        assert "not installed" in message
        assert "rclpy" in message


# ---------------------------------------------------------------------------
# check_websockets
# ---------------------------------------------------------------------------

class TestCheckWebsockets:
    def test_check_websockets_available(self):
        fake = _make_fake_module("websockets", version="12.0")
        with patch.dict("sys.modules", {"websockets": fake}):
            available, message = check_websockets()
        assert available is True
        assert "websockets" in message
        assert "12.0" in message

    def test_check_websockets_not_installed(self):
        with patch("builtins.__import__", side_effect=_import_raiser({"websockets"})):
            import sys
            saved = sys.modules.pop("websockets", None)
            saved_server = sys.modules.pop("websockets.server", None)
            try:
                available, message = check_websockets()
            finally:
                if saved is not None:
                    sys.modules["websockets"] = saved
                if saved_server is not None:
                    sys.modules["websockets.server"] = saved_server
        assert available is False
        assert "not installed" in message
        assert "websockets" in message


# ---------------------------------------------------------------------------
# check_all_dependencies
# ---------------------------------------------------------------------------

class TestCheckAllDependencies:
    def test_all_available(self):
        fake_rclpy = _make_fake_module("rclpy", version="1.0.0")
        fake_ws = _make_fake_module("websockets", version="12.0")
        with patch.dict("sys.modules", {"rclpy": fake_rclpy, "websockets": fake_ws}):
            result = check_all_dependencies()
        assert result["rclpy"]["available"] is True
        assert result["websockets"]["available"] is True
        assert result["all_available"] is True
        assert result["missing"] == []

    def test_rclpy_missing(self):
        fake_ws = _make_fake_module("websockets", version="12.0")
        with patch("builtins.__import__", side_effect=_import_raiser({"rclpy"})):
            import sys
            saved = sys.modules.pop("rclpy", None)
            saved_node = sys.modules.pop("rclpy.node", None)
            saved_action = sys.modules.pop("rclpy.action", None)
            sys.modules["websockets"] = fake_ws
            try:
                result = check_all_dependencies()
            finally:
                if saved is not None:
                    sys.modules["rclpy"] = saved
                if saved_node is not None:
                    sys.modules["rclpy.node"] = saved_node
                if saved_action is not None:
                    sys.modules["rclpy.action"] = saved_action
        assert result["rclpy"]["available"] is False
        assert result["websockets"]["available"] is True
        assert result["all_available"] is False
        assert result["missing"] == ["rclpy"]

    def test_websockets_missing(self):
        fake_rclpy = _make_fake_module("rclpy", version="1.0.0")
        with patch("builtins.__import__", side_effect=_import_raiser({"websockets"})):
            import sys
            saved = sys.modules.pop("websockets", None)
            saved_server = sys.modules.pop("websockets.server", None)
            sys.modules["rclpy"] = fake_rclpy
            try:
                result = check_all_dependencies()
            finally:
                if saved is not None:
                    sys.modules["websockets"] = saved
                if saved_server is not None:
                    sys.modules["websockets.server"] = saved_server
        assert result["rclpy"]["available"] is True
        assert result["websockets"]["available"] is False
        assert result["all_available"] is False
        assert result["missing"] == ["websockets"]

    def test_both_missing(self):
        with patch("builtins.__import__", side_effect=_import_raiser({"rclpy", "websockets"})):
            import sys
            saved_rclpy = sys.modules.pop("rclpy", None)
            saved_node = sys.modules.pop("rclpy.node", None)
            saved_action = sys.modules.pop("rclpy.action", None)
            saved_ws = sys.modules.pop("websockets", None)
            saved_ws_server = sys.modules.pop("websockets.server", None)
            try:
                result = check_all_dependencies()
            finally:
                if saved_rclpy is not None:
                    sys.modules["rclpy"] = saved_rclpy
                if saved_node is not None:
                    sys.modules["rclpy.node"] = saved_node
                if saved_action is not None:
                    sys.modules["rclpy.action"] = saved_action
                if saved_ws is not None:
                    sys.modules["websockets"] = saved_ws
                if saved_ws_server is not None:
                    sys.modules["websockets.server"] = saved_ws_server
        assert result["rclpy"]["available"] is False
        assert result["websockets"]["available"] is False
        assert result["all_available"] is False
        assert set(result["missing"]) == {"rclpy", "websockets"}


# ---------------------------------------------------------------------------
# format_dependency_report
# ---------------------------------------------------------------------------

class TestFormatDependencyReport:
    def test_all_ok(self):
        deps = {
            "rclpy": {"available": True, "message": "rclpy 1.0.0 available"},
            "websockets": {"available": True, "message": "websockets 12.0 available"},
            "all_available": True,
            "missing": [],
        }
        report = format_dependency_report(deps)
        assert "Dependency Check:" in report
        assert "\u2713" in report  # checkmark
        assert "All dependencies satisfied." in report
        assert "\u2717" not in report  # no cross-mark

    def test_missing(self):
        deps = {
            "rclpy": {"available": False, "message": "rclpy not installed: No module named 'rclpy'"},
            "websockets": {"available": True, "message": "websockets 12.0 available"},
            "all_available": False,
            "missing": ["rclpy"],
        }
        report = format_dependency_report(deps)
        assert "Dependency Check:" in report
        assert "\u2717" in report  # cross-mark for rclpy
        assert "\u2713" in report  # checkmark for websockets
        assert "Missing dependencies: rclpy" in report
        assert "sudo apt install" in report


# ---------------------------------------------------------------------------
# ensure_dependencies
# ---------------------------------------------------------------------------

class TestEnsureDependencies:
    def test_passes_when_all_available(self):
        fake_rclpy = _make_fake_module("rclpy", version="1.0.0")
        fake_ws = _make_fake_module("websockets", version="12.0")
        with patch.dict("sys.modules", {"rclpy": fake_rclpy, "websockets": fake_ws}):
            # Should return without raising.
            ensure_dependencies()

    def test_raises_when_missing(self):
        with patch("builtins.__import__", side_effect=_import_raiser({"rclpy", "websockets"})):
            import sys
            saved_rclpy = sys.modules.pop("rclpy", None)
            saved_node = sys.modules.pop("rclpy.node", None)
            saved_action = sys.modules.pop("rclpy.action", None)
            saved_ws = sys.modules.pop("websockets", None)
            saved_ws_server = sys.modules.pop("websockets.server", None)
            try:
                with pytest.raises(ImportError, match="Missing dependencies"):
                    ensure_dependencies()
            finally:
                if saved_rclpy is not None:
                    sys.modules["rclpy"] = saved_rclpy
                if saved_node is not None:
                    sys.modules["rclpy.node"] = saved_node
                if saved_action is not None:
                    sys.modules["rclpy.action"] = saved_action
                if saved_ws is not None:
                    sys.modules["websockets"] = saved_ws
                if saved_ws_server is not None:
                    sys.modules["websockets.server"] = saved_ws_server

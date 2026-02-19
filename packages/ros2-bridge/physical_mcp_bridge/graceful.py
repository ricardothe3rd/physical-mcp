"""Graceful degradation for optional dependencies.

Provides helpers to check whether rclpy and websockets are available at
runtime so the bridge can start in a reduced-functionality mode (or emit
a clear error message) instead of crashing with an opaque ImportError.
"""

from __future__ import annotations


def check_rclpy() -> tuple[bool, str]:
    """Check whether *rclpy* can be imported.

    Returns
    -------
    tuple[bool, str]
        ``(True, "rclpy <version> available")`` on success, or
        ``(False, "rclpy not installed: <error>")`` on failure.
    """
    try:
        import rclpy  # noqa: F811
        version = getattr(rclpy, "__version__", "unknown")
        return True, f"rclpy {version} available"
    except ImportError as exc:
        return False, f"rclpy not installed: {exc}"


def check_websockets() -> tuple[bool, str]:
    """Check whether *websockets* can be imported.

    Returns
    -------
    tuple[bool, str]
        ``(True, "websockets <version> available")`` on success, or
        ``(False, "websockets not installed: <error>")`` on failure.
    """
    try:
        import websockets  # noqa: F811
        version = getattr(websockets, "__version__", "unknown")
        return True, f"websockets {version} available"
    except ImportError as exc:
        return False, f"websockets not installed: {exc}"


def check_all_dependencies() -> dict:
    """Run all dependency checks and return a summary dict.

    Returns
    -------
    dict
        Keys:

        * ``rclpy``        -- ``{"available": bool, "message": str}``
        * ``websockets``   -- ``{"available": bool, "message": str}``
        * ``all_available`` -- ``True`` if every dependency is present
        * ``missing``      -- list of missing dependency names
    """
    rclpy_ok, rclpy_msg = check_rclpy()
    ws_ok, ws_msg = check_websockets()

    missing: list[str] = []
    if not rclpy_ok:
        missing.append("rclpy")
    if not ws_ok:
        missing.append("websockets")

    return {
        "rclpy": {"available": rclpy_ok, "message": rclpy_msg},
        "websockets": {"available": ws_ok, "message": ws_msg},
        "all_available": rclpy_ok and ws_ok,
        "missing": missing,
    }


def format_dependency_report(deps: dict) -> str:
    """Format the output of :func:`check_all_dependencies` for humans.

    Parameters
    ----------
    deps:
        The dict returned by :func:`check_all_dependencies`.

    Returns
    -------
    str
        A multiline, human-readable dependency report.
    """
    lines: list[str] = ["Dependency Check:"]

    for name in ("rclpy", "websockets"):
        info = deps[name]
        icon = "\u2713" if info["available"] else "\u2717"
        lines.append(f"  {icon} {name}: {info['message']}")

    if deps["all_available"]:
        lines.append("All dependencies satisfied.")
    else:
        names = ", ".join(deps["missing"])
        lines.append(f"Missing dependencies: {names}")
        lines.append("Install with: sudo apt install ros-humble-rclpy")

    return "\n".join(lines)


def ensure_dependencies() -> None:
    """Raise :class:`ImportError` if any required dependency is missing.

    If all dependencies are present the function returns normally.
    """
    deps = check_all_dependencies()
    if deps["all_available"]:
        return
    raise ImportError(format_dependency_report(deps))

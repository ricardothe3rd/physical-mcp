"""Tests for the telemetry module."""

import os
import time

from physical_mcp_bridge.telemetry import TelemetryReporter


class TestTelemetryReporter:
    """Tests for TelemetryReporter."""

    def test_initial_state(self):
        reporter = TelemetryReporter()
        status = reporter.get_status()
        assert status["commands_processed"] == 0
        assert status["errors"] == 0
        assert status["uptime_seconds"] >= 0
        assert status["pid"] == os.getpid()

    def test_record_command(self):
        reporter = TelemetryReporter()
        reporter.record_command()
        reporter.record_command()
        reporter.record_command()
        assert reporter.get_status()["commands_processed"] == 3

    def test_record_error(self):
        reporter = TelemetryReporter()
        reporter.record_error()
        reporter.record_error()
        assert reporter.get_status()["errors"] == 2

    def test_mixed_commands_and_errors(self):
        reporter = TelemetryReporter()
        reporter.record_command()
        reporter.record_error()
        reporter.record_command()
        reporter.record_command()
        reporter.record_error()

        status = reporter.get_status()
        assert status["commands_processed"] == 3
        assert status["errors"] == 2

    def test_uptime_increases(self):
        reporter = TelemetryReporter()
        time.sleep(0.1)
        status = reporter.get_status()
        assert status["uptime_seconds"] >= 0.1

    def test_pid_is_current_process(self):
        reporter = TelemetryReporter()
        assert reporter.get_status()["pid"] == os.getpid()

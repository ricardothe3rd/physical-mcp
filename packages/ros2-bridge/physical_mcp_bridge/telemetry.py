"""Periodic telemetry reporting for bridge health."""

import time
import psutil
from typing import Any


class TelemetryReporter:
    """Reports bridge health metrics."""

    def __init__(self):
        self.start_time = time.time()
        self.commands_processed = 0
        self.errors = 0

    def record_command(self):
        self.commands_processed += 1

    def record_error(self):
        self.errors += 1

    def get_status(self) -> dict[str, Any]:
        uptime = time.time() - self.start_time
        return {
            'uptime_seconds': round(uptime, 1),
            'commands_processed': self.commands_processed,
            'errors': self.errors,
            'pid': psutil.Process().pid if psutil else None,
        }

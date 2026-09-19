"""
telemetry_log.py - CSV logging of every control-loop tick.

One row per tick of the control loop, holding the commands sent and the
sensor values in effect at that moment. This is the raw material for
system identification, controller tuning, and any offline/ML work - none
of which is possible if the telemetry is only ever rendered to a label
and thrown away.

Values are stored as raw numbers (not display strings) so the log can be
loaded straight into pandas/numpy.
"""

import csv
import os
import threading
import time

DEFAULT_DIR = "logs"

# Column order is fixed so every log file is directly comparable.
FIELDS = (
    "t_wall",        # unix timestamp (s) - for correlating with video
    "t_mono",        # seconds since loop start - use this for analysis
    "dt",            # measured seconds since the previous tick
    "loop_hz",       # instantaneous 1/dt
    "send_ms",       # how long the BLE write blocked for
    "drive_cmd",     # -100..100 as sent to the hub
    "steer_cmd",     # -100..100 as sent to the hub
    "throttle_raw",  # 0..1 right trigger, before mixing
    "brake_raw",     # 0..1 left trigger, before mixing
    "steer_raw",     # -1..1 left stick, after deadzone
    "headlights",    # 0/1
    "braking",       # 0/1
    "failsafe",      # 0/1 - watchdog had stopped the car on this tick
    "battery_pct",
    "temp_c",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "tilt_x", "tilt_y", "tilt_z",
)


class TelemetryLogger:
    """Buffered CSV writer. Safe to call log() from the control loop at
    20+ Hz; rows are batched and flushed on an interval so the loop never
    waits on disk.

    A logger that fails to open its file degrades to a no-op rather than
    taking the control loop down with it - losing the log is annoying,
    losing motor control is not acceptable.
    """

    def __init__(self, directory=DEFAULT_DIR, prefix="drive",
                 flush_interval=2.0, max_buffer=200):
        self.flush_interval = flush_interval
        self.max_buffer = max_buffer
        self.path = None
        self.rows_written = 0
        self._buffer = []
        self._lock = threading.Lock()
        self._last_flush = time.monotonic()
        self._fh = None
        self._writer = None

        try:
            os.makedirs(directory, exist_ok=True)
            name = "{}_{}.csv".format(prefix, time.strftime("%Y%m%d_%H%M%S"))
            self.path = os.path.join(directory, name)
            self._fh = open(self.path, "w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._fh, fieldnames=FIELDS,
                                          extrasaction="ignore", restval="")
            self._writer.writeheader()
            self._fh.flush()
        except OSError as e:
            self.path = None
            self._fh = None
            self._writer = None
            self.error = str(e)

    @property
    def enabled(self):
        return self._writer is not None

    @property
    def total_rows(self):
        """Rows accepted so far, including ones still buffered."""
        with self._lock:
            return self.rows_written + len(self._buffer)

    def log(self, row):
        """Queue one row (a dict keyed by FIELDS; missing keys are blank)."""
        if self._writer is None:
            return
        with self._lock:
            self._buffer.append(row)
            due = (len(self._buffer) >= self.max_buffer
                   or time.monotonic() - self._last_flush >= self.flush_interval)
            if due:
                self._flush_locked()

    def _flush_locked(self):
        if not self._buffer or self._writer is None:
            return
        try:
            self._writer.writerows(self._buffer)
            self._fh.flush()
            self.rows_written += len(self._buffer)
        except (OSError, ValueError):
            # Disk full, file removed under us, etc. Drop the batch and
            # disable logging; the control loop keeps running.
            self._writer = None
        finally:
            self._buffer.clear()
            self._last_flush = time.monotonic()

    def flush(self):
        with self._lock:
            self._flush_locked()

    def close(self):
        with self._lock:
            self._flush_locked()
            if self._fh is not None:
                try:
                    self._fh.close()
                except OSError:
                    pass
            self._fh = None
            self._writer = None


def flatten_xyz(prefix, triple, row):
    """Write a 3-tuple into row as prefix_x/_y/_z, tolerating None."""
    if triple is None:
        return
    for axis, value in zip("xyz", triple):
        row["{}_{}".format(prefix, axis)] = value

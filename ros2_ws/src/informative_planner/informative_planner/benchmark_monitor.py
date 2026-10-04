"""System resource monitor for benchmark runs.

Samples CPU, RAM and NVIDIA GPU metrics in a background thread, writing each
sample directly to disk so data survives crashes.  Also registers a SIGTERM
handler so a ``ros2 node kill`` or container stop still triggers a clean
crash report before the process exits.
"""

from __future__ import annotations

import csv
import json
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# NVIDIA GPU reader — runs nvidia-smi in a background thread, caches latest
# ---------------------------------------------------------------------------

_NVIDIA_QUERY = (
    'utilization.gpu,'
    'utilization.memory,'
    'memory.used,'
    'clocks.current.graphics,'
    'temperature.gpu,'
    'power.draw'
)
_NVIDIA_KEYS = (
    'gpu_util_pct',
    'gpu_mem_util_pct',
    'gpu_mem_used_mb',
    'gpu_freq_mhz',
    'gpu_temp_c',
    'gpu_power_w',
)


class _NvidiaReader:
    """Poll nvidia-smi periodically and cache the latest reading."""

    def __init__(self, poll_interval_sec: float = 1.0) -> None:
        self._interval = max(float(poll_interval_sec), 0.5)
        self._latest: dict[str, float] | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name='NvidiaReader', daemon=True
        )
        self._thread.start()

    def read(self) -> dict[str, float] | None:
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)

    def _run(self) -> None:
        while not self._stop.wait(self._interval):
            self._poll()

    def _poll(self) -> None:
        try:
            result = subprocess.run(
                [
                    'nvidia-smi',
                    '--query-gpu=' + _NVIDIA_QUERY,
                    '--format=csv,noheader,nounits',
                ],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return
        if result.returncode != 0:
            return
        parts = [p.strip() for p in result.stdout.strip().split(',')]
        if len(parts) != len(_NVIDIA_KEYS):
            return
        try:
            sample = {
                key: float(val)
                for key, val in zip(_NVIDIA_KEYS, parts)
                if val not in ('N/A', '[N/A]', '')
            }
        except ValueError:
            return
        with self._lock:
            self._latest = sample


def _try_build_nvidia_reader(interval_sec: float) -> _NvidiaReader | None:
    """Return an _NvidiaReader if nvidia-smi is available, else None."""
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
            capture_output=True,
            text=True,
            timeout=3.0,
        )
        if result.returncode == 0 and result.stdout.strip():
            return _NvidiaReader(poll_interval_sec=interval_sec)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return None


# ---------------------------------------------------------------------------
# SystemMonitor
# ---------------------------------------------------------------------------

_CSV_FIELDS = (
    'elapsed_sec',
    'cpu_total_pct',
    'ram_used_mb',
    'ram_pct',
    'proc_cpu_pct',
    'proc_ram_mb',
    'gpu_util_pct',
    'gpu_mem_util_pct',
    'gpu_mem_used_mb',
    'gpu_freq_mhz',
    'gpu_temp_c',
    'gpu_power_w',
)

_SIGNALS_TO_HANDLE = (signal.SIGTERM, signal.SIGABRT)


class SystemMonitor:
    """Sample CPU/RAM/GPU usage in a background thread and detect crashes.

    Usage::

        monitor = SystemMonitor(output_dir)
        monitor.start()
        # ... benchmark runs ...
        summary = monitor.stop()   # returns dict with peak/mean stats
    """

    def __init__(
        self,
        output_dir: Path,
        sample_interval_sec: float = 1.0,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.sample_interval_sec = max(float(sample_interval_sec), 0.1)

        self._monotonic_start: float | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._csv_file = None
        self._csv_writer = None
        self._prev_signals: dict[int, Any] = {}
        self._crash_written = False
        self._samples: list[dict[str, Any]] = []
        self._lock = threading.Lock()

        self._gpu: _NvidiaReader | None = None

        try:
            import psutil as _psutil
            self._psutil = _psutil
            try:
                self._process = _psutil.Process()
            except Exception:
                self._process = None
            # Warm up cpu_percent (first call always returns 0.0)
            if self._process is not None:
                self._process.cpu_percent()
            _psutil.cpu_percent()
        except ImportError:
            self._psutil = None
            self._process = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the CSV, start GPU reader, register signals, begin sampling."""
        self._monotonic_start = time.monotonic()

        csv_path = self.output_dir / 'system_monitor.csv'
        self._csv_file = csv_path.open('w', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(
            self._csv_file, fieldnames=_CSV_FIELDS
        )
        self._csv_writer.writeheader()
        self._csv_file.flush()

        self._gpu = _try_build_nvidia_reader(self.sample_interval_sec)

        self._register_signal_handlers()

        self._thread = threading.Thread(
            target=self._sample_loop,
            name='SystemMonitor',
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> dict[str, Any]:
        """Stop sampling and return a summary dict with peak/mean stats."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=self.sample_interval_sec + 2.0)
        if self._gpu is not None:
            self._gpu.stop()
        if self._csv_file is not None:
            self._csv_file.close()
        self._restore_signal_handlers()
        return self._build_summary()

    def write_crash_report(
        self,
        reason: str,
        context: dict[str, Any] | None = None,
    ) -> None:
        """Write a crash_report.json.  Safe to call from signal handlers."""
        if self._crash_written:
            return
        self._crash_written = True
        report = {
            'crash_reason': reason,
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'elapsed_sec': self._elapsed(),
            'context': context or {},
        }
        try:
            path = self.output_dir / 'crash_report.json'
            path.write_text(
                json.dumps(report, indent=2, sort_keys=True),
                encoding='utf-8',
            )
            if self._csv_file is not None:
                try:
                    self._csv_file.flush()
                except Exception:
                    pass
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal sampling
    # ------------------------------------------------------------------

    def _sample_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._take_sample()
            except Exception:
                pass
            self._stop_event.wait(self.sample_interval_sec)

    def _take_sample(self) -> None:
        row: dict[str, Any] = {'elapsed_sec': round(self._elapsed(), 2)}

        if self._psutil is not None:
            cpu_pct = self._psutil.cpu_percent(interval=None)
            mem = self._psutil.virtual_memory()
            row['cpu_total_pct'] = round(float(cpu_pct), 1)
            row['ram_used_mb'] = round(float(mem.used) / 1e6, 1)
            row['ram_pct'] = round(float(mem.percent), 1)

            if self._process is not None:
                try:
                    with self._process.oneshot():
                        row['proc_cpu_pct'] = round(
                            float(self._process.cpu_percent()), 1
                        )
                        row['proc_ram_mb'] = round(
                            float(self._process.memory_info().rss) / 1e6, 1
                        )
                except (
                    self._psutil.NoSuchProcess,
                    self._psutil.AccessDenied,
                ):
                    pass

        if self._gpu is not None:
            gpu = self._gpu.read()
            if gpu:
                for key in _NVIDIA_KEYS:
                    if key in gpu:
                        row[key] = round(float(gpu[key]), 1)

        with self._lock:
            self._samples.append(row)

        if self._csv_writer is not None:
            self._csv_writer.writerow(
                {field: row.get(field, '') for field in _CSV_FIELDS}
            )
            self._csv_file.flush()

    # ------------------------------------------------------------------
    # Summary statistics
    # ------------------------------------------------------------------

    def _build_summary(self) -> dict[str, Any]:
        with self._lock:
            samples = list(self._samples)

        if not samples:
            return {'sample_count': 0, 'gpu_available': self._gpu is not None}

        numeric_fields = [f for f in _CSV_FIELDS if f != 'elapsed_sec']
        peaks: dict[str, float] = {}
        means: dict[str, float] = {}

        for field in numeric_fields:
            values = [
                float(s[field])
                for s in samples
                if field in s and _is_finite(s[field])
            ]
            if values:
                peaks['peak_' + field] = max(values)
                means['mean_' + field] = round(
                    sum(values) / len(values), 2
                )

        duration = samples[-1].get('elapsed_sec', 0.0) if samples else 0.0

        return {
            'sample_count': len(samples),
            'monitor_duration_sec': duration,
            'gpu_available': self._gpu is not None,
            **peaks,
            **means,
        }

    # ------------------------------------------------------------------
    # Elapsed time
    # ------------------------------------------------------------------

    def _elapsed(self) -> float:
        if self._monotonic_start is None:
            return 0.0
        return time.monotonic() - self._monotonic_start

    # ------------------------------------------------------------------
    # Signal handling
    # ------------------------------------------------------------------

    def _register_signal_handlers(self) -> None:
        for sig in _SIGNALS_TO_HANDLE:
            try:
                prev = signal.signal(sig, self._on_signal)
                self._prev_signals[sig] = prev
            except (OSError, ValueError):
                pass

    def _restore_signal_handlers(self) -> None:
        for sig, handler in self._prev_signals.items():
            try:
                signal.signal(sig, handler)
            except (OSError, ValueError):
                pass
        self._prev_signals.clear()

    def _on_signal(self, signum: int, frame: Any) -> None:
        sig_name = signal.Signals(signum).name
        self.write_crash_report('signal_%s' % sig_name)
        # Flush CSV before propagating
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
            except Exception:
                pass
        # Restore default and re-raise so the process actually exits
        prev = self._prev_signals.get(signum, signal.SIG_DFL)
        if callable(prev) and prev not in (signal.SIG_DFL, signal.SIG_IGN):
            prev(signum, frame)
        else:
            signal.signal(signum, signal.SIG_DFL)
            os.kill(os.getpid(), signum)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False

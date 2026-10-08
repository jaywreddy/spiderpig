"""A small stage profiler: nested wall-clock timers, counters and metrics, summarised
through a logger (never ``print``).

The bake's ``_Profiler`` (:mod:`spiderpig.bake`, logger ``bake_gltf``) generalised: a
title, a total label and a logger of its own. ``spiderpig build --profile``
(:mod:`spiderpig.tools.build_profile`, logger ``spiderpig.build``, stages ``import`` ...
``order``) uses it. It lives under ``tools/``, outside :func:`spiderpig.design.engine_version`'s
hash, so measuring never re-keys a store or a cache; the bake keeps its own copy until an
engine change folds it in::

    prof = Profiler(name="build", total="build_total", logger_name="spiderpig.build")
    with prof.timed("plan"):
        ...
    prof.bump("counter_name")
    prof.set_metric("metric_key", value)
    prof.log_summary()
    prof.as_dict()          # {"stages": {label: seconds}, "calls", "counters", "metrics"}
"""

from __future__ import annotations

import logging
import os
import time
from collections import defaultdict
from contextlib import contextmanager
from dataclasses import dataclass, field


@dataclass
class Profiler:
    """Lightweight perf recorder: nested wall-clock timers + counters + metrics.

    Durations are accumulated per label so the same bracket can be entered
    many times (e.g. once per animation frame) and reported as total / mean /
    p50 / p95. ``name`` titles the summary (``<name> profile summary:``, the
    ``%<name>`` column: each label's share of the ``total`` label's time);
    ``logger_name`` is the logger it goes to, at INFO.
    """

    enabled: bool = True
    name: str = "profile"
    total: str = "total"
    logger_name: str = "spiderpig.profiler"
    _durations: dict[str, list[float]] = field(
        default_factory=lambda: defaultdict(list)
    )
    _counters: dict[str, int] = field(default_factory=lambda: defaultdict(int))
    _metrics: dict[str, float] = field(default_factory=dict)

    @contextmanager
    def timed(self, label: str):
        if not self.enabled:
            yield
            return
        start = time.perf_counter()
        try:
            yield
        finally:
            self._durations[label].append(time.perf_counter() - start)

    def add(self, label: str, seconds: float) -> None:
        """Record a duration measured elsewhere (e.g. the imports before ``main``)."""
        if self.enabled:
            self._durations[label].append(float(seconds))

    def bump(self, label: str, n: int = 1) -> None:
        if self.enabled:
            self._counters[label] += n

    def set_metric(self, key: str, value: float) -> None:
        if self.enabled:
            self._metrics[key] = float(value)

    def as_dict(self) -> dict:
        """Every label's total seconds and call count, the counters and the metrics."""
        return {
            "stages": {k: sum(v) for k, v in self._durations.items()},
            "calls": {k: len(v) for k, v in self._durations.items()},
            "counters": dict(self._counters),
            "metrics": dict(self._metrics),
        }

    def log_summary(self) -> None:
        if not self.enabled:
            return

        grand_total = sum(self._durations.get(self.total, [])) or None

        rows = []
        for label, times_ in self._durations.items():
            n = len(times_)
            total = sum(times_)
            mean_ms = (total / n) * 1000.0 if n else 0.0
            ts = sorted(times_)
            p50 = ts[n // 2] * 1000.0 if n else 0.0
            p95 = ts[min(n - 1, int(n * 0.95))] * 1000.0 if n else 0.0
            pct = (total / grand_total * 100.0) if grand_total else 0.0
            rows.append((label, n, total, mean_ms, p50, p95, pct))
        rows.sort(key=lambda r: -r[2])

        share = f"%{self.name}"
        lines = [
            f"{self.name} profile summary:",
            f"  {'label':40} {'calls':>6} {'total_s':>9} "
            f"{'mean_ms':>9} {'p50_ms':>9} {'p95_ms':>9} {share:>6}",
            f"  {'-' * 40} {'-' * 6} {'-' * 9} {'-' * 9} "
            f"{'-' * 9} {'-' * 9} {'-' * 6}",
        ]
        for label, n, total, mean_ms, p50, p95, pct in rows:
            lines.append(
                f"  {label:40} {n:6d} {total:9.3f} "
                f"{mean_ms:9.3f} {p50:9.3f} {p95:9.3f} {pct:6.1f}"
            )
        if self._counters:
            lines.append("  counters:")
            for k, v in sorted(self._counters.items()):
                lines.append(f"    {k}: {v}")
        if self._metrics:
            lines.append("  metrics:")
            for k, v in sorted(self._metrics.items()):
                # Integers stay integers for readability (vert counts, bytes).
                if v == int(v):
                    lines.append(f"    {k}: {int(v)}")
                else:
                    lines.append(f"    {k}: {v:.3f}")

        logging.getLogger(self.logger_name).info("\n".join(lines))


def process_age() -> float | None:
    """Seconds since this process started (the interpreter's start-up and every import
    included), from ``/proc/self/stat``; None where there is no ``/proc``. Resolution: one
    clock tick (10 ms)."""
    try:
        with open("/proc/self/stat") as f:
            stat = f.read()
        start_ticks = int(stat.rsplit(")", 1)[1].split()[19])     # field 22, starttime
        return time.clock_gettime(time.CLOCK_BOOTTIME) - start_ticks / os.sysconf("SC_CLK_TCK")
    except (OSError, ValueError, IndexError, AttributeError):
        return None

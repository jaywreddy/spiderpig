"""A stage's record on the handle and in the store: the reports put on the handle, logged
and written (:func:`_commit`), served again while valid for the running engine
(:func:`_cached`), and the constructions' warnings a stage collects
(:func:`capture_warnings`)."""


from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import TYPE_CHECKING, Protocol, Self, cast

from spiderpig.design import Design
from spiderpig.fabricate import template_for as _template_for

if TYPE_CHECKING:
    from spiderpig.failure import Failure

log = logging.getLogger("spiderpig")


class StageReport(Protocol):
    """What every stage's report has (the :class:`Report` dataclasses, a
    :class:`spiderpig.verify.VerifyReport`)."""

    ok: bool
    failures: list[Failure]
    seconds: float

    @classmethod
    def from_dict(cls, d: dict) -> Self: ...


def _record(design: Design, op: str, seconds: float, ok: bool, cached: bool = False) -> None:
    entry = design.record(op, seconds, ok, cached)
    if design.store is not None:
        design.store.log(design.id, entry)


def _report[R: StageReport](design: Design, stage: str, cls: type[R]) -> R | None:
    """The handle's report of ``stage`` (``design.reports.get(stage)``) as its class."""
    return cast("R | None", design.reports.get(stage))  # _commit: each stage's own class


def _commit[R: StageReport](design: Design, stage: str, rep: R, op: str | None = None,
                            write: bool = True, cached: bool = False,
                            seconds: float | None = None) -> R:
    """Put a finished report on the handle, log the operation (``seconds``: what this
    call took, else the report's), and write it to the store (``write``; one served from
    the store, ``cached``, is only logged)."""
    design.reports[stage] = rep
    _record(design, op or stage, rep.seconds if seconds is None else seconds, rep.ok, cached)
    if ran_out(rep):
        write = False       # the planner's CPU budget, not the design: never a stored verdict
    if design.edited and stage in EDITED_STAGES:
        write = False       # the edited parts' (the store's are the unedited design's)
    if write and not cached and design.store is not None:
        design.store.write_report(design, stage, rep)
    return rep


def _finish[R: StageReport](design: Design, stage: str, rep: R, t0: float, **kw) -> R:
    rep.ok = not rep.failures
    rep.seconds = round(time.time() - t0, 3)
    return _commit(design, stage, rep, **kw)


def _stored(design: Design, stage: str, current: bool = True, variant: str | None = None
            ) -> dict | None:
    """The stage's file in the design's store (``current``: only one written by the
    running engine version; ``variant``: the copy kept per variant, a verify's level),
    else ``None``."""
    if design.store is None:
        return None
    doc = design.store.read_report(design.id, stage, variant)
    if doc is None or (current and doc.get("engine_version") != design.engine_version):
        return None
    return doc


def _cached[R: StageReport](design: Design, stage: str, cls: type[R], op: str | None = None,
                            **need) -> R | None:
    """The stage's report from the handle, else from the store when valid for the running
    engine (then put on the handle and logged as cached); ``need`` are field values it
    must match (a verify's ``level``: the store keeps one report per level, so the levels
    don't evict each other)."""
    rep = _report(design, stage, cls)
    if (rep is not None and not ran_out(rep)
            and all(getattr(rep, k, None) == v for k, v in need.items())):
        return rep
    if design.edited and stage in EDITED_STAGES:
        return None         # the store's are the unedited design's
    t0 = time.time()
    level = need.get("level") if stage == "verify" else None
    doc = _stored(design, stage, variant=str(level)) if level else None
    if doc is None:
        doc = _stored(design, stage)
    if doc is None or any(doc.get(k) != v for k, v in need.items()):
        return None
    rep = cls.from_dict(doc)
    if ran_out(rep):            # (written before such reports stopped being stored)
        return None
    return _commit(design, stage, rep, op, cached=True, seconds=time.time() - t0)


def _template(design: Design):
    """The side's kinematic template (built once per handle)."""
    if design.template is None:
        design.template = _template_for(design.config)
    return design.template


def ran_out(rep) -> bool:
    """Did any of this report's failures come from the planner's CPU budget running out
    (``no_plan_in_time``)? Such a report (a plan, or a build or verify behind it) is not a
    verdict on the design: it is never written to the store or served from it."""
    return any(getattr(f, "code", None) == "no_plan_in_time"
               for f in getattr(rep, "failures", None) or ())


WARNING_LOGGERS = ("spiderpig.construction", "spiderpig.servos", "spiderpig.hardware")


@contextmanager
def capture_warnings(names: tuple[str, ...] = WARNING_LOGGERS):
    """Collect what the constructions warn about while a stage runs (a printed snap that
    overstrains, a servo model that can't be had), deduplicated in order, so a report
    carries them instead of only the server's stderr."""
    seen: dict[str, None] = {}

    class _Collect(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            seen.setdefault(record.getMessage(), None)

    handler = _Collect(level=logging.WARNING)
    loggers = [logging.getLogger(n) for n in names]
    propagated = [lg.propagate for lg in loggers]
    for lg in loggers:
        lg.addHandler(handler)
        lg.propagate = False      # on the report, not (again) on a terminal's stderr
    out: list[str] = []
    try:
        yield out
    finally:
        for lg, p in zip(loggers, propagated, strict=True):
            lg.removeHandler(handler)
            lg.propagate = p
        out += list(seen)


EDITED_STAGES = ("export", "verify")
"""What an edited handle (:attr:`Design.edited`) neither reads from nor writes to the store:
the store's are the unedited design's."""

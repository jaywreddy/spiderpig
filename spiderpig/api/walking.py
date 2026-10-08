"""The walk (:func:`walk`) and a module's stride."""


from __future__ import annotations

import math
import time

from spiderpig import linkage
from spiderpig import walk as walk_model
from spiderpig.api.reports import WalkReport
from spiderpig.api.store_ops import _cached, _finish
from spiderpig.config import (
    BuildConfig,
    ParamError,
)
from spiderpig.design import (
    Design,
)
from spiderpig.failure import Failure

# ---------------------------------------------------------------------------
# walk
# ---------------------------------------------------------------------------


def walk(design: Design, force: bool = False) -> WalkReport:
    """The quasi-static walk model of the robot (:func:`walk.api_payload`; no parts):
    its metrics and each motion target's verdict. A mechanism is skipped."""
    if not force and (rep := _cached(design, "walk", WalkReport)) is not None:
        return rep
    t0 = time.time()
    rep = WalkReport()
    cfg = design.config
    if design.kind == "mechanism":
        rep.skipped = f"{cfg.linkage} is a mechanism: it has an output, not feet"
        return _finish(design, "walk", rep, t0)
    feet_z = None
    if design.side is not None:
        try:
            feet_z = walk_model.foot_z_planned(cfg, design.side)
        except (ValueError, KeyError):
            feet_z = None
    payload = walk_model.api_payload(cfg, feet_z=feet_z)
    rep.servo = payload["servo"]
    rep.feet_z_planned = feet_z is not None
    if not payload["valid"]:
        rep.failures.append(Failure("walk", "linkage_invalid", payload["error"]))
        return _finish(design, "walk", rep, t0)
    rep.metrics = payload["metrics"]
    rep.mass_g = payload["mass_g"]
    from spiderpig import verify as _verify

    rep.rows = _verify.walk_rows(design, rep.metrics)
    if (note := no_travel_note(cfg, rep.metrics)) is not None:
        rep.notes.append(note)
        for r in rep.rows:
            if r.requirement in ("motion.stride_mm", "motion.speed_mm_s"):
                r.detail = note
    return _finish(design, "walk", rep, t0)


NO_TRAVEL_MM = 1.0      # a stride under this is no travel at all (the feet cancel)
WALKS_MM = 20.0         # a module "walks" from this stride a turn on (a few mm is a shuffle)


def no_travel_note(cfg: BuildConfig, metrics: dict) -> str | None:
    """Why a walker's stride is (near) zero, when it is: the module's feet cancel each
    other in the quasi-static model, so the body stands and bobs (a mirrored pair at one
    phase, or two legs one way with nothing to take turns with); or that a stride of a
    few millimetres is a shuffle, not a walk (:data:`WALKS_MM`); and which module of the
    linkage walks (:func:`describe` gives every module's stride)."""
    stride = float(metrics.get("stride_mm") or 0.0)
    if stride >= WALKS_MM:
        return None
    lk = linkage.get(cfg.linkage)
    walking = {m: s for m in lk.leg_modules if m != cfg.module
               and (s := module_stride(cfg.linkage, m)) is not None and s >= WALKS_MM}
    duty = metrics.get("duty") or []
    stands = all(float(d) >= 0.999 for d in duty) if duty else False
    phases = ([round(math.degrees(p), 1) for p in cfg.phases] if cfg.phases
              else "its default phases")
    others = ", ".join(f"{m} ({s:.0f} mm/rev)" for m, s in walking.items())
    head = (f"{cfg.linkage}'s {cfg.module} module at {phases} walks {stride:.2g} mm per "
            f"revolution in the quasi-static model")
    if stride < NO_TRAVEL_MM:
        why = ("no net travel: " + head
               + (": every foot stays on the ground (duty 1.0) and the feet's pushes cancel, "
                  "so the body stands and bobs" if stands else
                  ": the feet's pushes cancel over the cycle"))
    else:
        why = f"a shuffle, not a walk: {head} (a module walks from {WALKS_MM:g} mm a turn)"
    return (why
            + (f"; of this linkage's modules, {others} walk" if walking
               else "; no other module of this linkage walks at its default phases")
            + "; describe(linkage) lists each module's stride_mm")


_MODULE_STRIDES: dict[tuple[str, str], float | None] = {}


def module_stride(key: str, module: str) -> float | None:
    """The walk model's stride (mm per revolution, the two-sided robot) of a linkage's
    module at its default phases and proportions, the feet at a nominal spacing
    (:func:`walk.foot_z_guess`: no layer plan is searched for a card); ``None`` when the
    model can't use it."""
    k = (key, module)
    if k not in _MODULE_STRIDES:
        try:
            _MODULE_STRIDES[k] = _module_stride(BuildConfig(linkage=key, module=module))
        except (ValueError, KeyError, ParamError):
            _MODULE_STRIDES[k] = None
    return _MODULE_STRIDES[k]


def _module_stride(cfg: BuildConfig) -> float | None:
    """:func:`walk.api_payload`'s ``metrics["stride_mm"]`` (rounded to 0.01 mm) at the
    nominal feet, ``None`` where the payload isn't ``valid``: the same steps in the same
    order (the payload's head, the walk model, its straight-walk metrics), without making
    JSON of the feet's and legs' paths, which the stride doesn't read (a second a card)."""
    feet_z = walk_model.foot_z_guess(cfg)
    servo = walk_model.servo_info(cfg.servo)
    cfg.design_json()
    walk_model.links_of(cfg.lk)
    try:
        model = walk_model.walker(cfg, feet_z=feet_z)
    except walk_model.LinkageError:
        return None
    m = walk_model.straight_walk_metrics(model, rpm_max=servo["rpm_max"])
    return round(float(walk_model.jsonable(m["stride_mm"])), 2)

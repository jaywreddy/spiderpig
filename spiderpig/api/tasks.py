"""A task's candidates (:func:`rank_candidates`, the API's ``candidates``): which
registered linkages can do what a :class:`spiderpig.task.Task` asks, ranked, with the
numbers that justify each verdict.

The linkages are read by what they declare, never by their kind's name: feet
(:attr:`linkage.Linkage.walks`), an output point (:data:`spec.CAPABILITIES` ``output``)
and its motion, inputs that turn over ranges (``ranged``), how many inputs, and the roles
a design of theirs fills (a ``turntable`` adds a yaw: :mod:`spiderpig.roles`). Cheap
evidence first, nothing fabricated:

1. **screen** (every linkage, seconds the first time, then the store's cards): a walker's
   card (:func:`describe`: each module's stride in the walk model, one foot's lift), a
   cranked mechanism's output check (stroke, straightness, swing, dwell...), an arm's
   output over its inputs' box (how far out and how high it reaches about the yaw axis).
   An arm's reach there is a bound: the bench the turret stands it on is the plan's. An arm
   traces under control: the task's path (else a line of its stroke) is solved point by
   point by the controller's inverse kinematics (:func:`trace_path`,
   :class:`spiderpig.control.arm.ArmModel`), its band across the line a bound on the
   straightness until its precision budget adds the lost motion.
2. **stored** (free): a design of the candidate's spec already in the store (whatever
   targets it carried) lends its reports, this engine's only: the plan, the walk with the
   feet at their layers, the precision budget (its worst-case bound; an arm's ISO 9283 RP
   beside it), a verify's cost and mass.
3. **measure** (``measure=N``, opt-in): the best ``N`` that pass are resolved into the
   store, checked and planned (an arm's plan is minutes cold: the store keeps it), walked,
   and a cranked mechanism's precision budget taken (an arm's needs its side fabricated:
   ``precision(design)``, a job); an arm's bench reach is then its turret's
   (:func:`construction.rotary.base_pair.table_reach`).

Each candidate carries a ``spec``: the task's numbers as the Spec's targets and the task
itself (its ``task`` section: ``resolve`` takes it as is, and ``verify`` re-checks the
task's geometry on the design, :func:`spiderpig.verify.task_rows` over
:func:`bench_reach`, :func:`trace_path` and the output check), and, where the linkage
misses a length the task asks for, the ``scale`` of its proportions that would meet it
(unchecked: ``recommend`` re-plans a scale before it offers one).
"""

from __future__ import annotations

import itertools
import math
import time
from collections.abc import Sequence
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import cast

import numpy as np

from spiderpig import linkage
from spiderpig import roles as rolemod
from spiderpig.api import walking
from spiderpig.api.cards import describe, scale_params_table
from spiderpig.config import BuildConfig, default_module, robot_layout, role_field_default
from spiderpig.control.arm import ArmModel, IKError
from spiderpig.design import canonical_json, jsonable
from spiderpig.spec import (
    CAPABILITIES,
    OUTPUT_TARGETS,
    PLACE_AXES,
    SECTIONS,
    TARGET_FIELDS,
    Target,
    output_metrics,
    validate,
)
from spiderpig.stages.resolve import undriven_inputs
from spiderpig.store import PROJECT, Store
from spiderpig.task import TASK_FIELDS, Task

DOWN_MM = 2.0
"""A screen's sampling of an arm's box: a pose this close over its lowest reaches it."""


@dataclass
class Check:
    """One requirement of the task on one candidate: its value, what the task needs,
    whether it is met (``None``: not measured at this tier, ``source`` says by what), the
    evidence tier (``proven``, ``measured``, ``estimated``, ``bound``: a necessary
    condition only) and where the number comes from."""

    requirement: str
    value: float | None
    need: str
    ok: bool | None
    tier: str
    source: str
    detail: str = ""


@dataclass
class Candidate:
    """A linkage (and module) that can do the task, or nearly: ``ok`` (every check met;
    ``None``: none fails, some unmeasured; ``False``: one fails), ``score`` (the mean over
    the measured checks of ``max(0, 1 - miss / need)``), its ``checks``, its ``numbers``,
    the ``spec`` to resolve, the stored ``design`` its numbers came from, the ``scale`` of
    its proportions that meets the task's lengths, and ``why`` in a sentence."""

    linkage: str
    name: str
    kind: str
    module: str
    ok: bool | None = None
    score: float = 1.0
    checks: list[Check] = field(default_factory=list)
    numbers: dict = field(default_factory=dict)
    spec: dict = field(default_factory=dict)
    design: str | None = None
    scale: dict | None = None
    why: str = ""
    rank: int = 0
    evidence: dict = field(default_factory=dict, repr=False)   # name -> (tier, source, detail)


@dataclass
class CandidatesReport:
    """:func:`rank_candidates`' answer: the task as read, the ranked ``candidates``, the
    ``excluded`` linkages with why (the capability each lacks), ``notes``."""

    task: dict
    do: str
    candidates: list[Candidate] = field(default_factory=list)
    excluded: list[dict] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    seconds: float = 0.0
    ok: bool = True
    failures: list = field(default_factory=list)

    def to_dict(self) -> dict:
        doc = cast(dict, jsonable(asdict(self)))
        for c in doc["candidates"]:
            c.pop("evidence", None)
        return doc

    def describe(self) -> str:
        lines = [f"task: {Task.from_dict(self.task).describe()}"]
        for c in self.candidates:
            verdict = {True: "meets it", False: "misses", None: "not ruled out"}[c.ok]
            lines.append(f"{c.rank}. {c.linkage} {c.module} ({c.kind}): {verdict}, score "
                         f"{c.score:.2f}; {c.why}")
            for k in c.checks:
                mark = {True: "ok", False: "MISS", None: "?"}[k.ok]
                v = "-" if k.value is None else f"{k.value:.4g}"
                lines.append(f"   [{mark}] {k.requirement} {v} (needs {k.need}; {k.tier}, "
                             f"{k.source}){': ' + k.detail if k.detail else ''}")
        lines += [f"   excluded {x['linkage']}: {x['why']}" for x in self.excluded]
        lines += self.notes
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# the operation
# ---------------------------------------------------------------------------


def rank_candidates(task: Task | dict, store: Store | str | Path | None = PROJECT, *,
               measure: int = 0, limit: int | None = None) -> CandidatesReport:
    """The registered linkages that can do ``task`` (a :class:`spiderpig.task.Task` or its
    document; :class:`spiderpig.spec.SpecErrors` for an invalid one), ranked: those that
    meet every requirement first, then by score and the task's merit (the longest stride,
    the best repeatability, the straightest line). ``measure``: how many of the best that
    pass the screen to resolve into ``store`` and plan (and walk, or budget), so their
    numbers are the stages' (module docstring). ``limit``: keep the best this many."""
    t0 = time.time()
    if not isinstance(task, Task):
        task = Task.from_dict(task)
    st = Store.of(store)
    rep = CandidatesReport(task.to_dict(), task.do)
    scales = scale_params_table(st)        # (the store's per-code cache, else computed)
    for key in linkage.available():
        lk = linkage.get(key)
        if lk.product:          # (a decanting cradle: a hand-cranked product, its own design)
            rep.excluded.append({"linkage": key, "kind": lk.kind,
                                 "why": f"{key} is a product ({lk.product}): a design of its "
                                        "own, hand-cranked, not a candidate for a task"})
            continue
        got = _SCREENS[task.do](lk, task, st)
        if isinstance(got, str):
            rep.excluded.append({"linkage": key, "kind": lk.kind, "why": got})
            continue
        rep.candidates += got
    for c in rep.candidates:
        c.spec = _spec(c, task)
        _stored(c, st)
        _judge(c, task, scales)
    _rank(rep.candidates, task)
    if measure > 0:
        done = 0
        for c in rep.candidates:
            if done >= measure:
                break
            if c.ok is False:
                continue
            _measure(c, task, st)
            _judge(c, task, scales)
            done += 1
        _rank(rep.candidates, task)
    rep.notes = _notes(task, rep)
    if limit is not None and len(rep.candidates) > limit:
        rep.notes.append(f"{len(rep.candidates) - limit} more candidates past the best {limit} "
                         "(limit)")
        rep.candidates = rep.candidates[:limit]
    rep.seconds = round(time.time() - t0, 2)
    return rep


# ---------------------------------------------------------------------------
# screens: what each linkage declares, and its cheap numbers
# ---------------------------------------------------------------------------


def _need(c: Candidate, name: str, value: float | None, tier: str, source: str,
          detail: str = "") -> None:
    """Record a value of the requirement ``name`` (checked by :func:`_judge`)."""
    c.numbers[name] = None if value is None else round(float(value), 4)
    c.evidence[name] = (tier, source, detail)


def _walk(lk: linkage.Linkage, task: Task, st: Store | None) -> list[Candidate] | str:
    if not lk.walks:
        motion = lk.output.motion if lk.output else "no"
        return f"{lk.key} has no feet (its output is a {motion}): it doesn't walk"
    card = describe(lk.key, st)
    rpm = walking.walk_model.servo_info(role_field_default(lk, "servo"))["rpm_max"]
    lift = (card.get("foot_path") or {}).get("lift_mm")
    out = []
    shuffles = []
    for m, doc in card["modules"].items():
        stride = doc.get("stride_mm")
        if not doc.get("walks"):
            shuffles.append(f"{m} {stride or 0:.0f} mm")
            continue
        c = Candidate(lk.key, lk.name, lk.kind, m)
        _need(c, "stride_mm", stride, "measured", "card: the walk model, nominal feet")
        _need(c, "speed_mm_s", stride * rpm / 60.0, "estimated",
              f"card: the stride at the servo's {rpm:g} rpm no load")
        _need(c, "lift_mm", lift, "measured", "card: one foot's path")
        c.numbers["legs_per_side"] = doc.get("legs")
        out.append(c)
    if not out:
        return (f"{lk.key}: no module walks in the walk model (strides a turn: "
                f"{', '.join(shuffles)}; from {walking.WALKS_MM:g} mm)")
    return out


def _dof(lk: linkage.Linkage) -> tuple[int, str]:
    """The output's positional axes: its inputs in the plane, and a yaw when its robot
    stands on a turntable (the role ``turntable``)."""
    n = len(lk.inputs)
    yaw = rolemod.applies("turntable", lk.kind)
    what = f"{n} input{'s' if n > 1 else ''} in its plane" + (" and its turntable's yaw"
                                                              if yaw else "")
    return n + int(yaw), what


def _drivable(lk: linkage.Linkage) -> str:
    """Why its drives can't be built, or ``""``: a cranked linkage's servo turns its crank,
    and each input past it is driven only where the linkage declares that input's drive
    (:attr:`linkage.Linkage.drives`, a stepper's ``JointDrive``:
    :func:`spiderpig.stages.resolve.undriven_inputs`)."""
    if free := undriven_inputs(lk):
        return (f"{len(lk.inputs)} inputs, {', '.join(free)} declaring no drive "
                "(Linkage.drives): the drive stage stops it (second_input_no_drive)")
    return ""


def _yaw_range(lk: linkage.Linkage) -> tuple[float, float] | None:
    if not rolemod.applies("turntable", lk.kind):
        return None
    from spiderpig.construction.rotary import PRESETS

    key = rolemod.default_key("turntable", BuildConfig(linkage=lk.key))
    return PRESETS[key].range_deg if key in PRESETS else None


def box_reach(lk: linkage.Linkage, values: tuple[float, ...] = (), n: int = 41) -> dict:
    """An arm's output over its inputs' box (``n`` per side), about the vertical through
    its shoulder ``O`` (a turret's yaw axis): how far out it gets (``r_mm``: least and
    most), how high it spans (``rise_mm``), and how far out it gets within
    :data:`DOWN_MM` of its lowest (``low_r_mm``)."""
    from spiderpig.linkage.constraints import layout

    pts = linkage.LegSolution(1, 0.0, values, lk.key).at(*lk.box(n))
    f = np.asarray(pts[layout(lk).output_point], dtype=float).reshape(-1, 2)
    o = np.asarray(pts["O"], dtype=float).reshape(-1, 2)[0] if "O" in pts else np.zeros(2)
    r, y = f[:, 0] - o[0], f[:, 1]
    fwd = r > 0
    low = fwd & (y <= y[fwd].min() + DOWN_MM) if fwd.any() else fwd
    return {"r_mm": (float(r[fwd].min()), float(r[fwd].max())) if fwd.any() else None,
            "rise_mm": float(np.ptp(y)),
            "low_r_mm": (float(r[low].min()), float(r[low].max())) if low.any() else None}


def sector_fit(w: float, d: float, lo: float, hi: float, span_deg: float | None
               ) -> tuple[float, str]:
    """How a ``w`` x ``d`` rectangle fits a band ``lo``..``hi`` out from an axis over a yaw
    of ``span_deg`` (``None``: no yaw, a band in the plane: depth only): the slack in mm (a
    negative one misses by that much) and how it is placed, the better of the two ways
    round."""
    slack, how, _ = sector_place(w, d, lo, hi, span_deg)
    return slack, how


def sector_place(w: float, d: float, lo: float, hi: float, span_deg: float | None
                 ) -> tuple[float, str, tuple[float, float]]:
    """:func:`sector_fit` and where the rectangle then lies: ``(slack, how, (r_in,
    r_out))``, the radii out from the axis its nearest and farthest points are at (what a
    rise over the area is read over: :func:`rise_over`)."""
    best: tuple[float, str, tuple[float, float]] = (-math.inf, "", (lo, hi))
    for across, deep in ((w, d), (d, w)):
        half = across / 2
        if span_deg is None:            # a line out from the axis: its depth only
            slack, how = (hi - lo) - deep, f"{deep:g} mm deep in a {hi - lo:.0f} mm band"
            span = (lo, min(hi, lo + deep))
            if across > 0:
                slack, how = -across, "no yaw: the output sweeps a plane, not an area"
        else:
            t = math.radians(min(span_deg, 359.0)) / 2
            near = max(lo, half / math.tan(t)) if t < math.pi / 2 else lo
            far = math.sqrt(max(hi ** 2 - half ** 2, 0.0)) - deep
            slack = far - near
            span = (near, min(hi, math.hypot(near + deep, half)))
            how = (f"{across:g} mm across, {deep:g} mm deep from {near:.0f} mm out "
                   f"(the far corners at {math.hypot(near + deep, half):.0f} of {hi:.0f} mm)")
        if slack > best[0]:
            best = (slack, how, span)
    return best


def _place(lk: linkage.Linkage, task: Task, st: Store | None) -> list[Candidate] | str:
    if not CAPABILITIES["output"][0](lk):
        return f"{lk.key} {CAPABILITIES['output'][2]}: nothing places a part"
    axes, what = _dof(lk)
    if axes < PLACE_AXES:
        motion = lk.output.motion if lk.output else ""
        return (f"{lk.key}'s {motion} output has {axes} ax{'es' if axes > 1 else 'is'} "
                f"({what}): placing over an area and lifting off it takes {PLACE_AXES}")
    if why := _drivable(lk):
        return f"{lk.key}: {why}"
    c = Candidate(lk.key, lk.name, lk.kind, default_module(lk.key))
    reach = box_reach(lk)
    c.numbers.update({"axes": axes, "rise_mm": round(reach["rise_mm"], 1),
                      "r_mm": [round(v, 1) for v in reach["r_mm"] or ()]})
    yaw = _yaw_range(lk)
    if yaw is not None:
        c.numbers["yaw_deg"] = list(yaw)
    _place_reach(c, task, reach["r_mm"], yaw, "bound",
                 "kinematics: the output over its inputs' box, out from its shoulder (the "
                 "bench the turret stands it on is the plan's)")
    _need(c, "height_mm", reach["rise_mm"], "bound", "kinematics: the output's rise over "
          "its box")
    return [c]


def _place_reach(c: Candidate, task: Task, band, yaw, tier: str, source: str) -> None:
    area = task.area()
    if area is None or band is None:
        return
    lo, hi = band
    slack, how = sector_fit(area[0], area[1], lo, hi,
                            None if yaw is None else yaw[1] - yaw[0])
    c.numbers["reach_band_mm"] = [round(lo, 1), round(hi, 1)]
    _need(c, "area_mm", slack, tier, source, how)


BENCH_GRID = 81
"""Poses a side of an arm's inputs' box :func:`bench_reach` reads (a finer grid than the
turret plan's 41: the rise over an area is read per radius)."""
RISE_BIN_MM = 4.0
"""The radial bins :func:`rise_over` reads the tool's highest point in."""


def bench_reach(design, n: int = BENCH_GRID) -> dict | None:
    """Where a planned arm's robot gets its tool down to the bench its turret stands on,
    and how high it lifts it there (the turret's plan, ``plans(side, config)``:
    :func:`spiderpig.construction.rotary.base_pair.table_reach`): ``lo`` / ``hi`` the band
    out from the yaw axis (``nan``: it never gets down), ``yaw_deg`` its turntable's range,
    ``note`` the plan's words, and over the inputs' box (``n`` a side, the design's own
    proportions) each pose's ``r_mm`` out from the axis and ``h_mm``, the tool's tip over
    the bench. ``None`` when no turret stands under it (its side alone, a linkage whose
    robot has no turret plan), no tool rides it, or it isn't planned."""
    from spiderpig.construction.robot import LAYOUTS
    from spiderpig.linkage.constraints import layout

    lk, cfg = design.lk, design.config
    key = robot_layout(lk)
    if not key or not cfg.robot or design.side is None:
        return None
    plans = getattr(LAYOUTS[key], "plans", None)
    if plans is None:
        return None
    tp, tool = plans(design.side, cfg)
    if tool is None:
        return None
    pts = linkage.LegSolution(1, 0.0, cfg.values, lk.key).at(*lk.box(n))
    f = np.asarray(pts[layout(lk).output_point], dtype=float).reshape(-1, 2)
    tip = (f[:, 1] - design.side.ctx.topo.geometry.lateral.get(tool.point, 0.0)
           - tool.tool.drop())
    return {"lo": float(tp.h.get("reach_lo", math.nan)),
            "hi": float(tp.h.get("reach_hi", math.nan)),
            "yaw_deg": tuple(tp.range_deg), "keep_r": float(tp.keep_r),
            "note": next((x for x in tp.notes if "bench" in x), ""),
            "r_mm": f[:, 0] - tp.x_a, "h_mm": tip - tp.y_ground}


def rise_over(r: np.ndarray, h: np.ndarray, r_in: float, r_out: float,
              bin_mm: float = RISE_BIN_MM) -> tuple[float, float] | None:
    """How high a tool lifts everywhere between ``r_in`` and ``r_out`` out from its axis:
    over radial bins of ``bin_mm``, the highest ``h`` of the poses in each, the least of
    them, and the radius of that bin; ``None`` when no pose lies in the span. (At a radius
    an arm's reach in height is one interval; the band says it gets down to the bench
    there, so this is the rise it can make over every point of the area.)"""
    edges = np.arange(r_in, max(r_out, r_in + 1e-9) + bin_mm, bin_mm)
    worst: tuple[float, float] | None = None
    for a, b in itertools.pairwise(edges):
        inside = (r >= a) & (r < b)
        if not inside.any():
            continue
        top = float(h[inside].max())
        if worst is None or top < worst[0]:
            worst = (top, float((a + b) / 2))
    return worst


def _trace(lk: linkage.Linkage, task: Task, st: Store | None) -> list[Candidate] | str:
    if not CAPABILITIES["output"][0](lk):
        return f"{lk.key} {CAPABILITIES['output'][2]}: it traces no path"
    out = lk.output
    assert out is not None
    if task.motion and out.motion != task.motion and not (lk.ranged and task.motion in
                                                          ("line", "path", "xy")):
        return f"{lk.key}'s output is a {out.motion}, not a {task.motion}"
    c = Candidate(lk.key, lk.name, lk.kind, default_module(lk.key))
    c.numbers["motion"] = out.motion
    needs = task.needs()
    if lk.ranged:     # any path its inverse kinematics reach, its drives coordinated
        asked = [k for k in needs if k in OUTPUT_TARGETS and k not in RANGED_TRACE]
        if asked:
            return (f"{lk.key} traces under control (its drives coordinated, a path at a "
                    f"time): {', '.join(asked)} measures a crank's turn, which its ranged "
                    "inputs don't make")
        _ranged_trace(c, lk, task, needs)
        return [c]
    has = output_metrics(lk)
    lack = [k for k in needs if k in OUTPUT_TARGETS and k not in has]
    if lack:
        return f"{lk.key}'s {out.motion} output has no {', '.join(lack)}"
    card = describe(lk.key, st)
    oc = card.get("output_check") or {}
    if "error" in oc:
        return f"{lk.key}: its output check fails at its defaults ({oc['error']})"
    for k in has:
        _need(c, k, oc.get(k), "measured", "card: the output check over a turn")
    c.numbers["extent_mm"] = oc.get("extent_mm")
    if why := _drivable(lk):
        _need(c, "drive", 0.0, "proven", "the linkage's inputs", why)
    return [c]


RANGED_TRACE = ("stroke_mm", "straightness_mm")
"""The output metrics an arm's traced path has (a line's length and its band across); the
others measure a crank's turn."""
SEG_MM = 5.0
"""The controller's segment along a straight move (:func:`spiderpig.control.host.line`'s
``seg_mm``): an inverse-kinematics waypoint every this many mm, the joints moving
linearly between them."""
CHORD_SAMPLES = 4
"""Poses read between two waypoints (the tip's sag off the line there)."""
LINE_DEG = (0.0, 90.0, 45.0, -45.0)
"""The directions a line of the task's stroke is tried in, in the arm's plane, when the
task gives no path."""


def arm_model(lk: linkage.Linkage, values: tuple[float, ...] | None = None) -> ArmModel:
    """An arm's side alone (:class:`spiderpig.control.arm.ArmModel`, no design: no
    turntable, no tool), its output point in its plane, at ``values`` (a design's
    proportions, :attr:`config.BuildConfig.values`; default the linkage's own)."""
    assert lk.output is not None
    ranges = {q: (math.radians(lo), math.radians(hi)) for q, (lo, hi) in lk.ranges.items()}
    return ArmModel(lk.key, values if values is not None else lk.values(), lk.output.point,
                    tuple(lk.inputs), ranges)


def _densify(path: np.ndarray, seg: float) -> np.ndarray:
    """``path``'s points with a point every ``seg`` mm or less along each leg between them."""
    out = [path[:1]]
    for a, b in itertools.pairwise(path):
        n = max(1, math.ceil(float(np.linalg.norm(b - a)) / seg))
        out.append(a + (b - a) * (np.arange(1, n + 1)[:, None] / n))
    return np.concatenate(out)


def trace_path(lk: linkage.Linkage, path, n: int = 41, tries: int = 8,
               values: tuple[float, ...] | None = None) -> dict:
    """Where an arm traces ``path`` (``[x, y]`` points in its plane, y up), as the
    controller drives a line (:func:`spiderpig.control.host.line`): a waypoint every
    :data:`SEG_MM` along each leg of the path, each solved by :meth:`ArmModel.ik` (its
    inputs' ranges and band), the joints moving linearly between them. The arm stands where
    the path fits (the path translated, not turned): the placements whose points lie
    deepest inside the inputs' box (its ``n`` x ``n`` grid) are tried first, ``tries`` of
    them. ``ok``; the ``offset_mm`` added to the path's points; ``band_mm``, the tip's band
    across the path's principal line (the path's own band and the sag between waypoints);
    ``chord_mm``, the sag alone; ``why`` a miss. ``values``: a design's proportions
    (default the linkage's own)."""
    from scipy.spatial import KDTree

    model = arm_model(lk, values)
    p = np.asarray(path, dtype=float)[:, :2]
    pts = _densify(p, SEG_MM)
    axes = [np.linspace(*model.ranges[q], n) for q in model.inputs]
    flat = [m.ravel() for m in np.meshgrid(*axes, indexing="ij")]
    idx = [m.ravel() for m in np.meshgrid(*[np.arange(n)] * len(axes), indexing="ij")]
    with np.errstate(all="ignore"):
        f = np.asarray(model.solution().at(*flat)[model.output], dtype=float).reshape(-1, 2)
    depth = np.min([np.minimum(i, n - 1 - i) for i in idx], axis=0).astype(float)
    if lk.band is not None and len(flat) == 2:      # the band's edges, in grid steps
        step = min(math.degrees(float(a[1] - a[0])) for a in axes)
        d = np.degrees(flat[0] - flat[1])
        depth = np.minimum(depth, np.minimum(d - lk.band[0], lk.band[1] - d) / step)
    ok = np.isfinite(f).all(axis=1) & (depth >= 0)
    f, depth = f[ok], depth[ok]
    if not len(f):
        return {"ok": False, "why": "its inputs' box reaches nowhere inside its band"}
    tree = KDTree(f)
    h = 1.5 * float(np.median(tree.query(f, k=2)[0][:, 1]))   # the grid's spacing, about
    offsets = f - pts.mean(axis=0)
    dist, near = tree.query(pts[None, :, :] + offsets[:, None, :])
    score = np.where((dist <= h).all(axis=1), depth[near].min(axis=1), -1.0)
    why = (f"no placement of its {len(pts)} points lies inside its inputs' box (the nearest "
           f"leaves a point {float(dist.max(axis=1).min()):.1f} mm out)")
    for k in np.argsort(-score, kind="stable")[:tries]:
        if score[k] < 0:
            break
        off = offsets[k]
        try:
            qs = [model.ik((float(x), 0.0, float(y))) for x, y in pts + off]
        except IKError as e:
            why = f"no placement tried reaches every point ({e})"
            continue
        q = np.array([[s[name] for name in model.inputs] for s in qs])
        s = np.linspace(0.0, 1.0, CHORD_SAMPLES + 2)[1:-1]
        mid = (q[:-1, None, :] + (q[1:] - q[:-1])[:, None, :] * s[None, :, None]).reshape(
            -1, q.shape[1])
        with np.errstate(all="ignore"):
            tip = np.asarray(model.solution().at(*mid.T)[model.output],
                             dtype=float).reshape(-1, 2) - off
        a = np.repeat(pts[:-1], CHORD_SAMPLES, axis=0)
        b = np.repeat(pts[1:], CHORD_SAMPLES, axis=0)
        ab = b - a
        t = np.clip(((tip - a) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-12),
                    0.0, 1.0)
        chord = float(np.linalg.norm(tip - (a + ab * t[:, None]), axis=1).max())
        c0 = p - p.mean(axis=0)
        across = np.linalg.svd(c0, full_matrices=False)[2][1]
        band = float(np.ptp(np.concatenate([pts, tip]) @ across))
        return {"ok": True, "offset_mm": [round(float(v), 2) for v in off],
                "band_mm": band, "chord_mm": chord, "waypoints": len(pts)}
    return {"ok": False, "why": why}


def _ranged_trace(c: Candidate, lk: linkage.Linkage, task: Task, needs: dict) -> None:
    """An arm's trace: the task's path (else a line of its stroke, :data:`LINE_DEG`) traced
    by its inverse kinematics (:func:`trace_path`): the path reached is the stroke, and the
    tip's band across it, the drives' interpolation alone, a bound on the straightness
    (the precision budget's lost motion adds to it: :func:`_precision`). Without a length
    asked: its box's diagonal, a bound on the stroke."""
    reach = box_reach(lk)
    ext = (reach["r_mm"][1] - reach["r_mm"][0]) if reach["r_mm"] else 0.0
    diag = math.hypot(ext, reach["rise_mm"])
    stroke = needs.get("stroke_mm")
    length = None if stroke is None else (stroke.min if stroke.min is not None
                                          else stroke.value)
    if task.path_mm:
        info = task.path()
        assert info is not None
        tries = [(np.asarray([q[:2] for q in task.path_mm], dtype=float), "the task's path",
                  info["length_mm"])]
    elif length:
        tries = [(np.array([[0.0, 0.0], [length * math.cos(math.radians(a)),
                                         length * math.sin(math.radians(a))]]),
                  f"a {length:g} mm line at {a:g} deg", length) for a in LINE_DEG]
    else:
        _need(c, "stroke_mm", diag, "bound",
              "kinematics: its box's diagonal (a line inside it may be shorter)")
        return
    src = f"kinematics: ArmModel.ik at every point, a waypoint each {SEG_MM:g} mm"
    got: dict = {}
    for path, what, along in tries:
        got = trace_path(lk, path)
        if got["ok"]:
            c.numbers["path_band_mm"] = round(got["band_mm"], 4)
            c.numbers["path_offset_mm"] = got["offset_mm"]
            _need(c, "path_mm", along, "measured", src,
                  f"{what}, {got['waypoints']} waypoints, its points moved by "
                  f"{got['offset_mm']} mm (the arm stands where it fits)")
            _need(c, "stroke_mm", along, "measured", src, what)
            if "straightness_mm" in needs:
                _need(c, "straightness_mm", got["band_mm"], "bound", src,
                      f"the tip's band across the line, {got['chord_mm']:.3g} mm of it the "
                      "sag between waypoints: the precision budget's lost motion adds to it")
            return
    _need(c, "path_mm", None, "measured", src, f"{tries[-1][1]}: {got['why']}")
    _need(c, "stroke_mm", diag, "bound",
          "kinematics: its box's diagonal (a line inside it may be shorter)")


_SCREENS = {"walk": _walk, "place": _place, "trace": _trace}


# ---------------------------------------------------------------------------
# the spec each candidate resolves to
# ---------------------------------------------------------------------------


def _spec(c: Candidate, task: Task) -> dict:
    """The candidate as a Spec, the task's numbers its targets (those the Spec can hold
    for this linkage; :func:`spiderpig.spec.validate` has the last word)."""
    lk = linkage.get(c.linkage)
    doc: dict = {"kind": lk.kind, "linkage": {"key": lk.key}}
    if c.module != default_module(lk.key):
        doc["legs"] = {"module": c.module}
    needs = task.needs()
    if "obstacle_mm" in task.plain and lk.walks:
        needs.setdefault("ground_clearance_mm", Target(min=task.plain["obstacle_mm"]))
    for name, t in needs.items():
        path = TASK_FIELDS[name].spec if name in TASK_FIELDS else f"motion.{name}"
        if not path:
            continue
        if path == "precision.repeatability_mm" and TARGET_FIELDS["precision"]["rp_mm"].applies(lk):
            path = "precision.rp_mm"         # a positioner's repeatability is ISO 9283 RP
        section, metric = path.split(".")
        tf = TARGET_FIELDS[section].get(metric)
        if tf is None or lk.kind not in tf.kinds or not tf.applies(lk):
            continue
        if lk.ranged and metric in OUTPUT_TARGETS:   # the drives' coordination, not a target
            continue                                 # (its tour's output check: a minute)
        doc.setdefault(section, {})[metric] = t.to_dict()
    if "payload_g" in task.plain and CAPABILITIES["ranged"][0](lk):
        doc.setdefault("precision", {})["payload_g"] = task.plain["payload_g"]
    _name_controller(doc, lk, c.module)
    for e in validate(doc):          # (nothing should be refused: drop it if it is)
        section, _, metric = e.path.partition(".")
        if section in SECTIONS and metric in doc.get(section, {}):
            del doc[section][metric]
    doc["task"] = task.to_dict()     # (what verify's task.* rows re-check on the design)
    return doc


def _name_controller(doc: dict, lk: linkage.Linkage, module: str) -> None:
    """A candidate whose kind leaves its controller unnamed (``none``: a mechanism's, so a
    stored one keeps its BOM) names the board that drives its motors, the one that speaks
    its servo's protocol (:func:`actuators.controllers.default_key`): the machine a task
    builds is wired and powered, its spec saying so (2026-10-14)."""
    from spiderpig.actuators import controllers

    cfg = BuildConfig(linkage=lk.key, module=module)
    if not rolemod.applies("controller", lk.kind) or cfg.role("controller"):
        return
    doc.setdefault("constructions", {})["controller"] = controllers.default_key(cfg)


# ---------------------------------------------------------------------------
# stored and measured numbers
# ---------------------------------------------------------------------------


def _body(resolved: dict) -> str:
    """A resolved spec without its targets, outputs and task: what its stages' numbers
    share."""
    out = {k: v for k, v in resolved.items() if k not in ("motion", "size", "budget",
                                                          "outputs", "task")}
    payload = {k: v for k, v in (out.pop("precision", None) or {}).items() if k == "payload_g"}
    if payload:             # (the load the budget assumed: it shapes the precision numbers)
        out["precision"] = payload
    return canonical_json(out)


def _stored(c: Candidate, st: Store | None) -> None:
    """Numbers a design of the candidate's spec (whatever its targets) already holds in
    the store, from this engine."""
    if st is None or not st.designs.is_dir():
        return
    from spiderpig.stages.resolve import resolve

    mine = resolve(c.spec, None)
    want = _body(mine.resolved)
    for i in sorted(st.ids(), key=lambda i: i != mine.id):
        rec = st.read_design(i) or {}
        if rec.get("engine_version") != mine.engine_version or \
                _body(rec.get("resolved") or {}) != want:
            continue
        if _read_stored(c, st, i):
            c.design = c.design or i


def _report(st: Store, i: str, stage: str) -> dict | None:
    doc = st.read_report(i, stage)
    return doc if doc is not None and doc.get("ok", True) else None


def _read_stored(c: Candidate, st: Store, i: str) -> bool:
    used = False
    src = f"store: {i}"
    if (walk := _report(st, i, "walk")) and walk.get("metrics"):
        m = walk["metrics"]
        for k in ("stride_mm", "speed_mm_s"):
            if m.get(k) is not None:
                _need(c, k, m[k], "estimated" if k == "speed_mm_s" else "measured",
                      f"{src} walk (the feet at their planned layers)")
        used = True
    if (chk := _report(st, i, "check")) and chk.get("ground_clearance_mm") is not None:
        _need(c, "ground_clearance_mm", chk["ground_clearance_mm"], "measured", f"{src} check",
              chk.get("lowest_body_part") or "")
        used = True
    if (prec := _report(st, i, "precision")) and prec.get("metrics"):
        _precision(c, prec, src)
        used = True
    if (ver := _report(st, i, "verify")) is not None:
        for r in ver.get("rows") or []:
            req = r.get("requirement")
            if req in ("budget.cost_usd", "size.mass_g") and r.get("value") is not None:
                _need(c, req.split(".")[1], r["value"], r.get("tier", "measured"),
                      f"{src} verify {ver.get('level', '')}".strip(), r.get("detail") or "")
                used = True
    return used


def _precision(c: Candidate, prec: dict, src: str) -> None:
    """A precision budget's numbers on the candidate: the task's repeatability is ISO 9283
    RP where the design has one (a positioner: ``precision.rp_mm``, the mean + 3 sigma a
    maker quotes, the user's own measure of the arm, 2026-10-14), else the worst-case bound
    (``precision.repeatability_mm``, which stays beside RP as information); its accuracy
    the budget's. A metric the budget holds as
    ``None`` (unbounded: a singular pose) is a miss, never unmeasured. An arm's traced
    path's band gains twice the lost motion (either side of the line)."""
    m = prec["metrics"]
    lk = linkage.get(c.linkage)
    if m.get("rp_mm") is not None:
        c.numbers["rp_mm"] = round(float(m["rp_mm"]), 4)
    if m.get("bound_mm") is None and m.get("repeatability_mm") is not None:
        c.numbers["bound_mm"] = round(float(m["repeatability_mm"]), 4)
    for name in ("repeatability_mm", "accuracy_mm"):
        if name not in m:
            continue
        v = m["rp_mm"] if name == "repeatability_mm" and m.get("rp_mm") is not None else m[name]
        if v is None or not math.isfinite(float(v)):
            _need(c, name, math.inf, "estimated", f"{src} precision",
                  "unbounded: the joint system is singular at a pose of the budget's")
            continue
        detail = ((f"ISO 9283 RP, mean + 3 sigma (the worst-case bound "
                   f"{m['repeatability_mm']} mm beside it)" if m.get("rp_mm") is not None
                   else "the worst-case bound") if name == "repeatability_mm" else "")
        _need(c, name, v, "estimated", f"{src} precision", detail)
    rep = c.numbers.get("bound_mm", c.numbers.get("repeatability_mm"))   # (lost motion:
    band = c.numbers.get("path_band_mm")                                  #  the bound)
    if (lk.ranged and band is not None and rep is not None and math.isfinite(rep)
            and "straightness_mm" in c.evidence):
        _need(c, "straightness_mm", band + 2 * rep, "estimated", f"{src} precision",
              f"the tip's band across the line ({band:.3g} mm) and the worst-case lost "
              f"motion either side ({rep:.3g} mm); what calibrates out (accuracy) left out")


def _measure(c: Candidate, task: Task, st: Store | None) -> None:
    """Resolve, check and plan the candidate (walk it, budget it), its numbers the
    stages'."""
    from spiderpig import api
    from spiderpig.api.accuracy import measure as precision_measure

    d = api.resolve(c.spec, st)
    c.design = d.id
    chk = api.check(d)
    if not chk.ok:
        _need(c, "check", 0.0, "proven", f"check of {d.id}", chk.failures[0].message)
        return
    if chk.ground_clearance_mm is not None:
        _need(c, "ground_clearance_mm", chk.ground_clearance_mm, "measured", f"check of {d.id}",
              chk.lowest_body_part)
    pr = api.plan(d)
    if not pr.ok:
        _need(c, "plan", 0.0, "proven", f"plan of {d.id}", pr.failures[0].message)
        return
    c.numbers["stack_mm"] = pr.height_mm
    lk = d.lk
    if lk.walks:
        w = api.walk(d)
        if w.ok and w.metrics:
            _need(c, "stride_mm", w.metrics.get("stride_mm"), "measured",
                  f"walk of {d.id} (feet at their planned layers)")
            _need(c, "speed_mm_s", w.metrics.get("speed_mm_s"), "estimated", f"walk of {d.id}")
    elif not lk.ranged:          # the whole budget, no fabrication
        p = api.precision(d)
        if p.ok and p.metrics:
            _precision(c, {"metrics": p.metrics}, f"precision of {d.id}")
    else:                        # what needs no fabrication: the accuracy, the bench's reach
        p = precision_measure(d, planar=True)
        if p.ok and p.metrics.get("accuracy_mm") is not None:
            _need(c, "accuracy_mm", p.metrics["accuracy_mm"], "estimated",
                  f"precision of {d.id} (planar)")
        if task.do == "place" and (bench := bench_reach(d)) is not None:
            lo, hi = bench["lo"], bench["hi"]
            src = f"plan of {d.id}: where the tool's tip gets down to the bench"
            c.numbers["yaw_deg"] = list(bench["yaw_deg"])
            if math.isfinite(lo) and math.isfinite(hi):
                _place_reach(c, task, (lo, hi), bench["yaw_deg"], "estimated",
                             src + ", past the base")
            elif task.area() is not None:        # it never gets down to the bench
                c.numbers.pop("reach_band_mm", None)
                _need(c, "area_mm", None, "estimated", src, bench["note"] or "it never does")


# ---------------------------------------------------------------------------
# judging and ranking
# ---------------------------------------------------------------------------


def _judge(c: Candidate, task: Task, scales: dict[str, list[str]]) -> None:
    """Every requirement against the candidate's numbers: its checks, ``ok``, ``score``,
    the ``scale`` that meets the task's lengths, ``why``."""
    ev = c.evidence
    needs = task.needs()
    if "obstacle_mm" in task.plain:
        needs.setdefault("ground_clearance_mm", Target(min=task.plain["obstacle_mm"]))
    checks: list[Check] = []
    misses: list[tuple[str, float]] = []
    for name in ("drive", "check", "plan"):      # a stage that stops it
        if name in ev:
            tier, src, detail = ev[name]
            checks.append(Check(name, None, "to pass", False, tier, src, detail))
    for name, t in needs.items():
        v = c.numbers.get(name)
        tier, src, detail = ev.get(name, ("", _unmeasured(c, name), ""))
        if v is None:
            checks.append(Check(name, None, t.describe(), None, "unmeasured", src, detail))
            continue
        met, miss = t.check(float(v))
        checks.append(Check(name, float(v), t.describe(), _verdict(met, tier), tier, src,
                            detail))
        if not met:
            misses.append((name, max(0.0, 1.0 - miss / t.scale_at(float(v)))))
    if "path_mm" in ev:                  # an arm's: every point of the path in reach
        tier, src, detail = ev["path_mm"]
        v = c.numbers.get("path_mm")
        checks.append(Check("path_mm", v, "every point in reach", v is not None, tier, src,
                            detail))
        if v is None:
            misses.append(("path_mm", 0.0))
    area = task.area()
    if "area_mm" in ev and area is not None:
        tier, src, detail = ev["area_mm"]
        v = c.numbers["area_mm"]
        slack = -math.inf if v is None else float(v)     # (None: it never gets down there)
        met = slack >= 0
        checks.append(Check("area_mm", None if v is None else slack,
                            f"{area[0]:g} x {area[1]:g} mm in reach", _verdict(met, tier), tier,
                            src, detail if v is None else ("slack " if met else "misses by ")
                            + f"{abs(slack):.1f} mm: " + detail))
        if not met:
            misses.append(("area_mm", 0.0))
    if task.height() and "height_mm" in ev:
        tier, src, detail = ev["height_mm"]
        v = float(c.numbers["height_mm"])
        met = v >= task.height()
        checks.append(Check("height_mm", v, f">= {task.height():g}", _verdict(met, tier),
                            tier, src, detail))
        if not met:
            misses.append(("height_mm", max(0.0, v / task.height())))
    c.checks = checks
    fails = [k for k in checks if k.ok is False]
    c.ok = False if fails else (None if any(k.ok is None for k in checks) else True)
    scored = [k for k in checks if k.ok is not None]
    c.score = round((sum(1.0 for k in scored if k.ok) + sum(s for _, s in misses))
                    / len(scored), 3) if scored else 1.0
    c.scale = _scale(c, needs, scales.get(c.linkage, ()))
    if area is not None and (c.numbers.get("area_mm") or 0.0) < 0:
        c.scale = _area_scale(c, area, scales.get(c.linkage, ())) or c.scale
    c.why = _why(c, task)


def _verdict(met: bool, tier: str) -> bool | None:
    """A check's verdict: a bound (a necessary condition: an arm's box's diagonal for its
    stroke, its traced band before its lost motion is known) refutes but never confirms."""
    return (None if met else False) if tier == "bound" else met


def _unmeasured(c: Candidate, name: str) -> str:
    """What measures a requirement the candidate has no number for yet."""
    lk = linkage.get(c.linkage)
    return {"repeatability_mm": ("precision(design): an arm's side is fabricated for its "
                                 "links' tilt (a job)" if lk.ranged else
                                 "precision(design), or candidates(measure=N)"),
            "accuracy_mm": "precision(design), or candidates(measure=N)",
            "ground_clearance_mm": "check(design), or candidates(measure=N)",
            "cost_usd": "verify(design, 'standard'): the BOM",
            "mass_g": "verify(design, 'quick') estimates it, 'standard' builds it"}.get(
        name, "the stages (resolve the spec)")


LENGTHS = ("stride_mm", "speed_mm_s", "lift_mm", "stroke_mm", "straightness_mm")
"""The requirements that scale with the linkage (every length of it times s)."""


def _scale(c: Candidate, needs: dict[str, Target], scale: Sequence[str]) -> dict | None:
    """The scale of the linkage's proportions that meets the task's length bounds (its
    scale parameters, else every length), when the defaults miss one and the scaled
    lengths meet them all; unchecked (``recommend`` plans a scale before offering it)."""
    lens = {k: float(c.numbers[k]) for k in LENGTHS
            if k in needs and c.numbers.get(k) not in (None, 0)}
    if not lens or all(needs[k].check(v)[0] for k, v in lens.items()):
        return None
    s = 1.0
    for k, v in lens.items():
        t = needs[k]
        if t.min is not None and v < t.min:
            s = max(s, t.min / v)
        if t.max is not None and v > t.max:
            s = min(s, t.max / v)
    meets = all(needs[k].check(v * s)[0] for k, v in lens.items())
    lk = linkage.get(c.linkage)
    names = tuple(scale[:1]) or tuple(k for k in lk.params if k not in lk.angles)
    return {"factor": round(s, 3), "meets": meets,
            "params": {k: round(float(lk.params[k]) * s, 3) for k in names},
            "lengths": {k: round(v * s, 2) for k, v in lens.items()}}


def _area_scale(c: Candidate, area: tuple[float, float], scale: Sequence[str]
                ) -> dict | None:
    """The least scale of an arm's lengths whose reach band (out from its yaw axis, scaled
    with it: an estimate, the base's keep-out doesn't scale) holds the task's area, up to
    x3; ``None`` when none does."""
    band, yaw = c.numbers.get("reach_band_mm"), c.numbers.get("yaw_deg")
    if not band:
        return None
    span = None if not yaw else yaw[1] - yaw[0]

    def slack(f: float) -> float:
        return sector_fit(area[0], area[1], band[0] * f, band[1] * f, span)[0]

    lo, hi = 1.0, 3.0
    if slack(hi) < 0:
        return None
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if slack(mid) >= 0 else (mid, hi)
    lk = linkage.get(c.linkage)
    names = tuple(scale[:1]) or tuple(k for k in lk.params if k not in lk.angles)
    return {"factor": round(hi, 3), "meets": True,
            "params": {k: round(float(lk.params[k]) * hi, 3) for k in names},
            "lengths": {"reach_band_mm": [round(band[0] * hi, 1), round(band[1] * hi, 1)]},
            "note": "the reach band scaled with the arm (the base's keep-out doesn't): an "
                    "estimate; the precision and the drives' torque move with it"}


def _why(c: Candidate, task: Task) -> str:
    bad = [k for k in c.checks if k.ok is False]
    open_ = [k.requirement + (" (a bound only)" if k.tier == "bound" else "")
             for k in c.checks if k.ok is None]
    if bad:
        head = "misses " + "; ".join(
            f"{k.requirement} ({'-' if k.value is None else f'{k.value:.4g}'}, needs "
            f"{k.need}{': ' + k.detail if k.requirement in ('drive', 'check', 'plan') else ''})"
            for k in bad)
        if c.scale and c.scale["meets"]:
            what = "the area" if "reach_band_mm" in c.scale["lengths"] else "the lengths"
            head += f"; x{c.scale['factor']:g} of its proportions would meet {what}"
        return head
    if open_:
        return "meets what is measured; unmeasured: " + ", ".join(open_)
    return "meets every requirement"


_MERIT = {"walk": (("stride_mm", 1.0), ("lift_mm", 1.0)),
          "place": (("repeatability_mm", -1.0), ("area_mm", 1.0)),
          "trace": (("straightness_mm", -1.0), ("stroke_mm", 1.0))}
"""Each task's tie-breakers after the verdict and the score: (number, +1 more is better or
-1 less is), in order."""


def _rank(cs: list[Candidate], task: Task) -> None:
    def merit(c: Candidate) -> tuple:
        out = []
        for metric, sign in _MERIT[task.do]:
            v = c.numbers.get(metric)
            bound = c.evidence.get(metric, ("",))[0] == "bound"   # (not the machine's value)
            out.append(-(sign * float(v)) if v is not None and not bound else math.inf)
        return tuple(out)

    def key(c: Candidate):
        return ({True: 0, None: 1, False: 2}[c.ok],
                0 if c.ok is not False or (c.scale and c.scale["meets"]) else 1,
                -c.score, *merit(c), c.linkage, c.module)

    cs.sort(key=key)
    for i, c in enumerate(cs, 1):
        c.rank = i


def _notes(task: Task, rep: CandidatesReport) -> list[str]:
    out = []
    if any(c.evidence.get("area_mm", ("",))[0] == "bound" for c in rep.candidates):
        out.append("an arm's area check at the screen is a bound (its output's whole reach "
                   "out from the shoulder): candidates(measure=N) reads the band where its "
                   "tool gets down to the bench from the turret's plan")
    if not rep.candidates:
        out.append("no registered linkage can do this task: excluded lists what each lacks")
    out.append("next: resolve(candidate.spec) -> verify(design, 'quick') -> recommend "
               "(a checked scale for a missed length) -> verify 'standard'")
    return out

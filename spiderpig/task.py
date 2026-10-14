"""Tasks: what a machine must do, before any linkage is named (2026-10-14).

A :class:`Task` says what the machine is for, in the task's own words: **walk** (a stride,
a speed, an obstacle to step over), **place** (an area on the bench, or points, to put
things on, a payload, a repeatability) or **trace** (an output motion, a stroke, a
straightness, or a path's points). :func:`spiderpig.api.candidates` answers with the
registered linkages that can do it, ranked, each with the numbers behind its verdict and a
Spec that asks for the task's numbers as targets.

The numbers are the Spec's targets where the Spec has them (:data:`TASK_FIELDS`' ``spec``:
``stride_mm`` is ``motion.stride_mm``), so a candidate's spec is the task's numbers written
into the Spec's own vocabulary. A bare number is the bound the task means (a stride's
least, a repeatability's most: :attr:`TaskField.bound`); a target object
``{min, max, value, tol, hard}`` says it exactly. The task's own geometry (``area_mm``,
``height_mm``, ``points_mm``, ``path_mm``, ``obstacle_mm``) has no Spec field: the
candidates' checks read it. Like a Spec, a task is validated whole: unknown fields, a
field of another task and wildcards are errors with the nearest allowed key
(:func:`validate`, :class:`spiderpig.spec.SpecErrors`).
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field, replace

import numpy as np

from spiderpig.linkage.engine import MOTIONS
from spiderpig.spec import WILDCARDS, SpecError, SpecErrors, Target, nearest

DOES = ("walk", "place", "trace")
"""What a task asks of the machine."""


@dataclass(frozen=True)
class TaskField:
    """One number or shape a task may give: the tasks it belongs to, its unit, the bound a
    bare number means (``min``, ``max``) or its form (``plain`` a number, ``size`` a list
    of numbers, ``points`` a list of points, ``motion`` a word), the Spec target it becomes
    (``""``: the task's own geometry, read by the checks) and what it means."""

    name: str
    does: tuple[str, ...]
    unit: str
    bound: str
    doc: str
    spec: str = ""


def _f(*rows) -> dict[str, TaskField]:
    return {r[0]: TaskField(*r) for r in rows}


TASK_FIELDS: dict[str, TaskField] = _f(
    # walk
    ("stride_mm", ("walk",), "mm/rev", "min",
     "the body's travel per crank turn (the walk model)", "motion.stride_mm"),
    ("speed_mm_s", ("walk",), "mm/s", "min",
     "the stride at the drive's no-load speed (estimated)", "motion.speed_mm_s"),
    ("lift_mm", ("walk",), "mm", "min", "how high a foot lifts over its turn",
     "motion.lift_mm"),
    ("obstacle_mm", ("walk",), "mm", "plain",
     "the height of what it must step over: a foot lifts that high and the body clears it "
     "(the spec's lift_mm and ground_clearance_mm, each at least this)"),
    # place
    ("area_mm", ("place",), "mm", "size",
     "[width, depth] of the area on the bench it puts things down on"),
    ("height_mm", ("place",), "mm", "plain",
     "how high over the bench the tool must rise inside the area (default 0)"),
    ("points_mm", ("place",), "mm", "points",
     "points to reach, [x, y] on the bench or [x, y, z] (z up): the machine stands where "
     "they fit, so their extent is the area (and z the height)"),
    ("payload_g", ("place",), "g", "plain", "the load at the tool", "precision.payload_g"),
    ("repeatability_mm", ("place", "trace"), "mm", "max",
     "how close it returns to a taught point: the worst-case bound on the output's lost "
     "motion, every kind's (precision.repeatability_mm; an arm's ISO 9283 RP, the mean + "
     "3 sigma a maker quotes, is under it and shown beside it)",
     "precision.repeatability_mm"),
    ("accuracy_mm", ("place", "trace"), "mm", "max",
     "what calibrates out (holes off their drawing, heat, the drives' systematic errors)",
     "precision.accuracy_mm"),
    # trace
    ("motion", ("trace",), "", "motion",
     "the output's motion: " + ", ".join(MOTIONS)),
    ("path_mm", ("trace",), "mm", "points",
     "the path's points [x, y] in the output's plane (y up): its extent along its principal "
     "axis is the stroke needed, and a path within 2 % of a line asks for a line within "
     "tol_mm; an arm traces it under control, where its inverse kinematics reach every "
     "point (it stands where the path fits)"),
    ("tol_mm", ("trace",), "mm", "plain",
     "how far a straight path's points may stray from its line (default 1 mm)"),
    ("stroke_mm", ("trace",), "mm", "min", "the output's travel along its line",
     "motion.stroke_mm"),
    ("straightness_mm", ("trace",), "mm", "max", "the band across the line",
     "motion.straightness_mm"),
    ("on_line_fraction", ("trace",), "fraction", "min", "the part of the turn on the line",
     "motion.on_line_fraction"),
    ("rotation_deg", ("trace",), "deg", "max", "a translating platform's rotation",
     "motion.rotation_deg"),
    ("swing_deg", ("trace",), "deg", "min", "a rocker's swing", "motion.swing_deg"),
    ("dwell_deg", ("trace",), "deg", "min", "crank degrees the output stands still",
     "motion.dwell_deg"),
    # every task
    ("cost_usd", DOES, "USD", "max", "the purchase total (measured by a build's BOM)",
     "budget.cost_usd"),
    ("mass_g", DOES, "g", "max", "the machine's mass", "size.mass_g"),
)
"""Every field a task may give (beside ``do``), by name."""

STRAIGHT = 0.02
"""A path whose band across is within this share of its length is a line."""
TOL_MM = 1.0


@dataclass(frozen=True)
class Task:
    """What the machine must do (module docstring). ``targets``: the task's numbers that
    are bounds, as :class:`spiderpig.spec.Target`; ``plain``: its plain numbers; the shapes
    as given. Build one with :meth:`from_dict`, which validates."""

    do: str
    targets: Mapping[str, Target] = field(default_factory=dict)
    plain: Mapping[str, float] = field(default_factory=dict)
    area_mm: tuple[float, float] | None = None
    points_mm: tuple[tuple[float, ...], ...] | None = None
    path_mm: tuple[tuple[float, ...], ...] | None = None
    motion: str | None = None

    @classmethod
    def from_dict(cls, data: Mapping) -> Task:
        errors = validate(data)
        if errors:
            raise SpecErrors(errors)
        targets, plain = {}, {}
        for k, v in data.items():
            f = TASK_FIELDS.get(k)
            if f is None:
                continue
            if f.bound in ("min", "max"):
                targets[k] = _target(f.bound, v)
            elif f.bound == "plain":
                plain[k] = float(v)
        pts = data.get("points_mm")
        path = data.get("path_mm")
        area = data.get("area_mm")
        return cls(data["do"], targets, plain,
                   (float(area[0]), float(area[1])) if area is not None else None,
                   tuple(tuple(float(c) for c in p) for p in pts) if pts else None,
                   tuple(tuple(float(c) for c in p) for p in path) if path else None,
                   data.get("motion"))

    def to_dict(self) -> dict:
        out: dict = {"do": self.do}
        out.update({k: t.to_dict() for k, t in self.targets.items()})
        out.update(self.plain)
        for k in ("area_mm", "points_mm", "path_mm"):
            v = getattr(self, k)
            if v is not None:
                out[k] = [list(p) for p in v] if k != "area_mm" else list(v)
        if self.motion is not None:
            out["motion"] = self.motion
        return out

    # -- the task's geometry -------------------------------------------------

    def area(self) -> tuple[float, float] | None:
        """``[width, depth]`` on the bench: ``area_mm``, else the points' extent (the
        larger first)."""
        if self.area_mm is not None:
            return self.area_mm
        if self.points_mm:
            p = np.asarray([q[:2] for q in self.points_mm], dtype=float)
            ext = sorted((float(np.ptp(p[:, 0])), float(np.ptp(p[:, 1]))), reverse=True)
            return ext[0], ext[1]
        return None

    def height(self) -> float:
        """How high over the bench the tool rises: ``height_mm``, else the points' highest
        z, else 0."""
        if "height_mm" in self.plain:
            return self.plain["height_mm"]
        zs = [q[2] for q in self.points_mm or () if len(q) > 2]
        return max(zs) if zs else 0.0

    def path(self) -> dict | None:
        """``path_mm`` read: its length along its principal axis, its band across, and
        whether that makes it a line (:data:`STRAIGHT`)."""
        if not self.path_mm:
            return None
        p = np.asarray([q[:2] for q in self.path_mm], dtype=float)
        c = p - p.mean(axis=0)
        _, _, vt = np.linalg.svd(c, full_matrices=False)
        along, across = c @ vt[0], c @ vt[1]
        length, band = float(np.ptp(along)), float(np.ptp(across))
        return {"length_mm": length, "band_mm": band,
                "line": length > 0 and band <= STRAIGHT * length}

    def needs(self) -> dict[str, Target]:
        """The bounds the task puts on the machine, by task field: its targets, the
        obstacle as a lift, the path's length as a stroke and, a straight path, its tol as
        a straightness."""
        out = dict(self.targets)
        if "obstacle_mm" in self.plain:      # (the user's lift kept whole, its min raised)
            ob = self.plain["obstacle_mm"]
            lift = out.get("lift_mm")
            if lift is None:
                out["lift_mm"] = Target(min=ob)
            elif (lift.min or 0.0) < ob:
                out["lift_mm"] = replace(lift, min=ob)
        path = self.path()
        if path is not None:
            if "stroke_mm" not in out:
                out["stroke_mm"] = Target(min=path["length_mm"])
            if path["line"] and "straightness_mm" not in out:
                out["straightness_mm"] = Target(max=self.plain.get("tol_mm", TOL_MM))
        return out

    def describe(self) -> str:
        parts = [self.do]
        for k, t in self.needs().items():
            parts.append(f"{k} {t.describe()}")
        if self.area() is not None:
            w, d = self.area()  # type: ignore[misc]
            parts.append(f"area {w:g} x {d:g} mm")
        if self.height():
            parts.append(f"rising {self.height():g} mm")
        parts += [f"{k} {self.plain[k]:g}" for k in ("obstacle_mm", "payload_g")
                  if k in self.plain]
        if self.motion:
            parts.append(f"a {self.motion} output")
        return ", ".join(parts)


def _target(bound: str, v) -> Target:
    """A task's bound: a bare number is ``bound`` (``min`` or ``max``), else the object's."""
    if _number(v):
        return Target(min=float(v)) if bound == "min" else Target(max=float(v))

    def num(a: str) -> float | None:
        return None if v.get(a) is None else float(v[a])

    return Target(num("min"), num("max"), num("value"), num("tol"),
                  float(v.get("weight", 1.0)), v.get("hard"))


def _number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def validate(data: Mapping) -> list[SpecError]:
    """Everything wrong with a task document (empty: a valid task)."""
    if not isinstance(data, Mapping):
        return [SpecError("", "a task is an object: {do: walk | place | trace, ...}")]
    errs: list[SpecError] = []
    do = data.get("do")
    if do is None:
        errs.append(SpecError("do", "required: what the machine must do", DOES))
    elif not isinstance(do, str) or do not in DOES:
        near = nearest(str(do), DOES)
        errs.append(SpecError("do", f"unknown task {do!r}" + (
            " (a wildcard: name one; candidates ranks the linkages for it)"
            if str(do).strip().lower() in WILDCARDS else ""), DOES, near))
        do = None
    allowed = [k for k, f in TASK_FIELDS.items() if do is None or do in f.does]
    for k, v in data.items():
        if k == "do":
            continue
        f = TASK_FIELDS.get(k)
        if f is None:
            errs.append(SpecError(k, "unknown field", ("do", *allowed), nearest(k, allowed)))
            continue
        if do is not None and do not in f.does:
            errs.append(SpecError(k, f"{k} is a {' or '.join(f.does)} task's, not a {do}'s",
                                  tuple(allowed)))
            continue
        errs += _value(k, f, v)
    if do == "place" and data.get("area_mm") is None and not data.get("points_mm"):
        errs.append(SpecError("area_mm", "a place task names its area_mm or its points_mm",
                              ("area_mm", "points_mm")))
    return errs


def _value(k: str, f: TaskField, v) -> list[SpecError]:
    if f.bound in ("min", "max"):
        if _number(v):
            return [] if v >= 0 else [SpecError(k, f"must be >= 0, got {v:g}")]
        if not isinstance(v, Mapping):
            return [SpecError(k, f"a number (the {f.bound}) or a target {{min, max, value, "
                                 f"tol, hard}}, got {type(v).__name__}")]
        keys = ("min", "max", "value", "tol", "weight", "hard")
        bad = [a for a in v if a not in keys]
        out = [SpecError(f"{k}.{a}", "unknown field", keys, nearest(a, keys)) for a in bad]
        if not any(a in v for a in ("min", "max", "value")):
            out.append(SpecError(k, "a target needs min, max or value"))
        out += [SpecError(f"{k}.{a}", "must be a number") for a in ("min", "max", "value",
                                                                   "tol", "weight")
                if a in v and not _number(v[a])]
        if "hard" in v and not isinstance(v["hard"], bool):
            out.append(SpecError(f"{k}.hard", "must be true or false"))
        return out
    if f.bound == "plain":
        return [] if _number(v) and v >= 0 else [SpecError(k, "must be a number >= 0")]
    if f.bound == "motion":
        return ([] if v in MOTIONS else
                [SpecError(k, f"unknown motion {v!r}", MOTIONS, nearest(str(v), MOTIONS))])
    if f.bound == "size":
        ok = (isinstance(v, (list, tuple)) and len(v) == 2
              and all(_number(x) and x > 0 for x in v))
        return [] if ok else [SpecError(k, "must be [width, depth] in mm, each > 0")]
    # points
    ok = (isinstance(v, (list, tuple)) and len(v) >= 2
          and all(isinstance(p, (list, tuple)) and len(p) in (2, 3)
                  and all(_number(c) for c in p) for p in v))
    return [] if ok else [SpecError(k, "must be two or more points, each [x, y] or [x, y, z] "
                                       "in mm")]


def task_schema() -> dict:
    """The JSON Schema (draft 2020-12) of a task document."""
    target = {"type": "object", "additionalProperties": False,
              "properties": {"min": {"type": "number"}, "max": {"type": "number"},
                             "value": {"type": "number"},
                             "tol": {"type": "number", "minimum": 0},
                             "weight": {"type": "number", "minimum": 0},
                             "hard": {"type": "boolean"}}}
    point = {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 3}
    props: dict = {"do": {"enum": list(DOES), "description": "what the machine must do"}}
    for k, f in TASK_FIELDS.items():
        doc = f"{f.doc} [{f.unit}; {'/'.join(f.does)}]" if f.unit else f"{f.doc} [{f.does[0]}]"
        if f.bound in ("min", "max"):
            props[k] = {"description": f"{doc}; a bare number is the {f.bound}",
                        "anyOf": [{"type": "number", "minimum": 0}, target]}
        elif f.bound == "plain":
            props[k] = {"type": "number", "minimum": 0, "description": doc}
        elif f.bound == "motion":
            props[k] = {"enum": list(MOTIONS), "description": doc}
        elif f.bound == "size":
            props[k] = {"type": "array", "items": {"type": "number", "exclusiveMinimum": 0},
                        "minItems": 2, "maxItems": 2, "description": doc}
        else:
            props[k] = {"type": "array", "items": point, "minItems": 2, "description": doc}
    return {"$schema": "https://json-schema.org/draft/2020-12/schema",
            "$id": "https://spiderpig/schema/task-v1.json", "title": "spiderpig Task",
            "description": (__doc__ or "").split("\n\n")[0], "type": "object",
            "additionalProperties": False, "required": ["do"], "properties": props}

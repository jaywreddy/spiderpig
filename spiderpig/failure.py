"""Failures as data: every engine exception crosses the API boundary as a :class:`Failure`.

The engine raises (:class:`linkage.AssemblyError`, :class:`stack.PlanError`,
:class:`construction.ConstructionError`, ...) with the numbers in its text;
:meth:`Failure.from_exception` maps each onto a stage and a code and lifts
what the exception carries as data (``recommendations``, ``notes``,
``blockers``) into the payload. Operations in :mod:`spiderpig.api` build
richer failures where they hold the stage's own values (a failing
:class:`linkage.StepCheck`, the static stage's :class:`NoCrankPoint`).

=============  ==============================  =======================================
stage          code                            raised by
=============  ==============================  =======================================
``spec``       ``invalid_spec``                :class:`spiderpig.spec.SpecErrors`
``spec``       ``bad_parameter``               :class:`config.ParamError`
``program``    ``loop_cannot_close``           :class:`linkage.AssemblyError`
``output``     ``promise_broken``              :class:`linkage.OutputError`
``drive``      ``second_input_no_drive``       :class:`construction.ConstructionError`
                                               from the drive (a second input)
``construction`` ``unbuildable``               any other :class:`ConstructionError`
                                               before planning
``static``     ``link_no_layer``               :class:`stack.ClearanceError`
``plan``       ``no_plan`` /                   :class:`stack.PlanError` (in budget:
               ``no_plan_in_budget``           the search hit ``max_total_nodes``)
``fabricate``  ``unbuildable``                 :class:`ConstructionError` while
                                               building parts
``contract``   ``part_outside_claim``          :func:`construction.contract.check_side`
``clash``      ``parts_clash`` /               :func:`construction.contract.clashes` /
               ``bad_solid``                   ``bad_solids``
``layout``     ``part_exceeds_sheet``          :func:`layout.pack`
``bom``        ``unknown_catalog_key``         the BOM's ``KeyError``
``walk``       ``linkage_invalid``             :func:`walk.api_payload` ``valid: false``
``sim``        ``sim_failed``                  MuJoCo
=============  ==============================  =======================================

A :class:`Recommendation` is the engine's (:class:`stack.Recommendation`:
checked by re-running the stage) plus the spec ``patch`` that applies it
(:func:`apply_patch`).
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass, field

import linkage
from config import ParamError
from construction.base import ConstructionError
from spiderpig.spec import FIT_FIELDS, SpecErrors
from stack import ClearanceError, PlanError, StackSpec

STAGES = ("spec", "program", "output", "drive", "construction", "static", "plan", "fabricate",
          "contract", "clash", "layout", "bom", "walk", "sim")

_BLOCKER = re.compile(r"^\s*(\d+) x (.+?) vs (.+?): (-?[\d.]+) mm apart in one layer, need "
                      r"([\d.]+)\s*$")
_BLOCKER_PLATE = re.compile(r"^\s*(\d+) x (.+?) vs (.+?): (it would sit in a frame plate's "
                            r"layer)\s*$")
_BLOCKER_ANY = re.compile(r"^\s*(\d+) x (.*)$")
_STEPS = re.compile(r"after (\d+) search steps")
_JOINT = re.compile(r"joint (\S+) can't be placed")


@dataclass
class Recommendation:
    """A change that clears a failure, checked by re-running the stage with it, and the
    spec patch that applies it."""

    changes: list[dict]                      # [{"name", "before", "after"}]
    why: str = ""
    effects: str = ""
    verified: str = ""
    patch: dict = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    @classmethod
    def from_engine(cls, rec, lk: linkage.Linkage | None) -> Recommendation:
        """A :class:`stack.Recommendation` with its patch: a linkage parameter goes under
        ``linkage.params``, a :class:`construction.base.Params` field under ``fit``."""
        patch: dict = {}
        notes = []
        for name, _before, after in rec.changes:
            if lk is not None and name in lk.params:
                patch.setdefault("linkage", {}).setdefault("params", {})[name] = after
            elif name in FIT_FIELDS:
                patch.setdefault("fit", {})[name] = after
            else:
                notes.append(f"{name} is not a spec field: apply it by hand")
        return cls([{"name": n, "before": b, "after": a} for n, b, a in rec.changes],
                   rec.why, rec.effects, rec.verified, patch, notes)

    def describe(self) -> str:
        out = ", ".join(f"{c['name']} {c['before']:g} -> {c['after']:g}" for c in self.changes)
        if self.why:
            out += f": {self.why}"
        if self.effects:
            out += f" ({self.effects})"
        if self.verified:
            out += f"; {self.verified}"
        return out

    def to_dict(self) -> dict:
        return {"changes": list(self.changes), "why": self.why, "effects": self.effects,
                "verified": self.verified, "patch": self.patch, "notes": list(self.notes)}

    @classmethod
    def from_dict(cls, d: Mapping) -> Recommendation:
        return cls(list(d.get("changes", [])), d.get("why", ""), d.get("effects", ""),
                   d.get("verified", ""), dict(d.get("patch") or {}), list(d.get("notes", [])))


@dataclass
class Failure:
    """What a stage said went wrong, as data: the stage and code (the table in the module
    docstring), today's message unchanged, the parts involved (``culprits``: body, group,
    joint, layer, side, point, ...), the numbers behind it (``numbers``: mm, degrees,
    counts), the plan's ``blockers`` (count, the two shapes, gap, need), checked
    ``recommendations`` and ``notes``."""

    stage: str
    code: str
    message: str
    culprits: list[dict] = field(default_factory=list)
    numbers: dict = field(default_factory=dict)
    recommendations: list[Recommendation] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    blockers: list[dict] = field(default_factory=list)

    def describe(self) -> str:
        return f"{self.stage} ({self.code}): {self.message}"

    def to_dict(self) -> dict:
        return {"stage": self.stage, "code": self.code, "message": self.message,
                "culprits": list(self.culprits), "numbers": dict(self.numbers),
                "recommendations": [r.to_dict() for r in self.recommendations],
                "notes": list(self.notes), "blockers": list(self.blockers)}

    @classmethod
    def from_dict(cls, d: Mapping) -> Failure:
        """A failure back from its JSON form (a store's stage file)."""
        return cls(d["stage"], d["code"], d.get("message", ""),
                   culprits=list(d.get("culprits", [])), numbers=dict(d.get("numbers", {})),
                   recommendations=[Recommendation.from_dict(r)
                                    for r in d.get("recommendations", [])],
                   notes=list(d.get("notes", [])), blockers=list(d.get("blockers", [])))

    @classmethod
    def from_exception(cls, exc: BaseException, stage: str | None = None,
                       lk: linkage.Linkage | None = None) -> Failure:
        """The failure an engine exception stands for (see the module docstring); ``stage``
        overrides the stage the exception type implies (a ``ConstructionError`` while
        building parts is ``fabricate``); ``lk`` resolves recommendation patches."""
        msg = str(exc)
        if isinstance(exc, SpecErrors):
            return cls("spec", "invalid_spec", msg,
                       culprits=[{"path": e.path} for e in exc.errors],
                       notes=[e.describe() for e in exc.errors])
        if isinstance(exc, ParamError):
            return cls("spec", "bad_parameter", msg)
        if isinstance(exc, linkage.OutputError):
            return cls(stage or "output", "promise_broken", msg)
        if isinstance(exc, linkage.AssemblyError):
            m = _JOINT.search(msg)
            return cls(stage or "program", "loop_cannot_close", msg,
                       culprits=[{"joint": m.group(1)}] if m else [])
        if isinstance(exc, ClearanceError):
            return cls(stage or "static", "link_no_layer", msg,
                       recommendations=[Recommendation.from_engine(r, lk)
                                        for r in exc.recommendations],
                       notes=[n for n in exc.notes if n])
        if isinstance(exc, PlanError):
            m = _STEPS.search(exc.summary)
            spent = int(m.group(1)) if m else 0
            code = "no_plan_in_budget" if spent >= StackSpec().max_total_nodes else "no_plan"
            return cls(stage or "plan", code, msg,
                       numbers={"search_steps": spent} if m else {},
                       recommendations=[Recommendation.from_engine(r, lk)
                                        for r in exc.recommendations],
                       notes=[n for n in exc.notes if n],
                       blockers=[parse_blocker(b) for b in exc.blockers])
        if isinstance(exc, ConstructionError):
            if "the drive turns one input" in msg:
                return cls("drive", "second_input_no_drive", msg)
            return cls(stage or "construction", "unbuildable", msg)
        if isinstance(exc, KeyError) and "catalog" in msg:
            return cls(stage or "bom", "unknown_catalog_key", msg.strip("'\""))
        if isinstance(exc, ValueError) and msg.startswith("sheet packing dropped"):
            parts = re.findall(r"'([^']+)'", msg)
            return cls(stage or "layout", "part_exceeds_sheet", msg,
                       culprits=[{"body": p} for p in parts])
        return cls(stage or "engine", type(exc).__name__, msg or repr(exc))


def parse_blocker(text: str) -> dict:
    """A :meth:`stack.StackProblem.blockers` line as data (the text is kept)."""
    if (m := _BLOCKER.match(text)):
        count, a, b, gap, need = m.groups()
        return {"count": int(count), "a": a, "b": b, "gap_mm": float(gap),
                "need_mm": float(need), "text": text.strip()}
    if (m := _BLOCKER_PLATE.match(text)):
        count, a, b, why = m.groups()
        return {"count": int(count), "a": a, "b": b, "why": why, "text": text.strip()}
    if (m := _BLOCKER_ANY.match(text)):
        count, why = m.groups()
        return {"count": int(count), "why": why, "text": text.strip()}
    return {"text": text.strip()}


def apply_patch(spec: Mapping, patch: Mapping) -> dict:
    """``spec`` (a spec document) with ``patch`` merged in, as a JSON merge patch (RFC
    7386): objects merge key by key, ``None`` removes a key, any other value replaces.
    Validate the result with :meth:`spiderpig.spec.Spec.from_dict`."""
    out = dict(spec)
    for k, v in patch.items():
        if v is None:
            out.pop(k, None)
        elif isinstance(v, Mapping) and isinstance(out.get(k), Mapping):
            out[k] = apply_patch(out[k], v)
        else:
            out[k] = v
    return out


def merge_patch(a: Mapping, b: Mapping) -> dict:
    """The smallest merge patch (RFC 7386) that turns document ``a`` into ``b``
    (``apply_patch(a, merge_patch(a, b)) == b``): keys ``b`` lacks become ``None``."""
    out: dict = {}
    for k in sorted(set(a) | set(b), key=str):
        if k not in b:
            out[k] = None
        elif k not in a:
            out[k] = b[k]
        elif isinstance(a[k], Mapping) and isinstance(b[k], Mapping):
            sub = merge_patch(a[k], b[k])
            if sub:
                out[k] = sub
        elif a[k] != b[k]:
            out[k] = b[k]
    return out

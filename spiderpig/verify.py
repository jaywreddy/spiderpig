"""The harness: ``verify(design, level)`` -> :class:`VerifyReport`, pass/fail per requirement
with an evidence tier.

Every :class:`Row` names a requirement (a stage that must pass, or a spec
target ``section.name``), where its value came from (``source``), the value,
the target, whether it passed, and its ``tier``:

* ``proven``: the engine's guarantees: every loop closes (the program check's
  margins), a mechanism's promises hold, a plan exists and re-verifies on
  fresh sampling (:func:`stack.verify_plan`), every part stays inside its
  claims (:func:`construction.contract.check_side`), the stack height;
* ``measured``: numbers read off models or solids: the walk model's metrics,
  a foot path, the ground clearance, OCCT clashes and solids, the mass of
  the parts, the envelope, the DXF sheet count, the filament;
* ``estimated``: nominal or unverified inputs: the walk model's nominal mass
  before a build, the envelope before a build, ``speed_mm_s`` (the servo's
  no-load rpm), prices with unverified links.

Levels: ``quick`` (check + plan + walk; ~1 s), ``standard`` (+ build, the
contract at two crank angles, clashes and solids at the build's, the DXF
pack, the BOM), ``full`` (the audit's four contract angles, clashes at two,
and a MuJoCo run when ``mujoco`` imports: its speed is reported beside the
walk model's, informational).

A hard target's miss fails the report (``ok``); a soft miss lowers ``score``
(the weighted mean over the soft targets of ``max(0, 1 - miss / |bound|)``).
Targets a level doesn't measure are listed in ``unverified``.
"""

from __future__ import annotations

import importlib.util
import time
from dataclasses import dataclass, field, replace

from spiderpig import api
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.design import Design
from spiderpig.failure import Failure
from spiderpig.hardware.bom import BomLine, bom_from_mechanism
from spiderpig.hardware.catalog import sheet_size
from spiderpig.layout import pack
from spiderpig.spec import Target, TargetField, effective_hard, target_field
from spiderpig.stack import verify_plan

LEVELS = ("quick", "standard", "full")
CONTRACT_TS = {"standard": (0.0, 3.2), "full": (0.0, 1.6, 3.2, 4.8)}
CLASH_TS = {"standard": (1.0,), "full": (1.0, 4.38)}
SIM_SECONDS = 4.0


@dataclass
class Row:
    """One requirement's verdict (``pass`` in the JSON form is ``passed`` here)."""

    requirement: str
    source: str
    value: object
    target: str | None
    passed: bool
    tier: str
    hard: bool = True
    detail: str = ""
    score: float | None = None
    weight: float = 1.0
    unit: str = ""

    def to_dict(self) -> dict:
        return {"requirement": self.requirement, "source": self.source, "value": self.value,
                "target": self.target, "pass": self.passed, "tier": self.tier,
                "hard": self.hard, "detail": self.detail, "score": self.score,
                "weight": self.weight, "unit": self.unit}

    @classmethod
    def from_dict(cls, d: dict) -> Row:
        return cls(d["requirement"], d.get("source", ""), d.get("value"), d.get("target"),
                   bool(d["pass"] if "pass" in d else d.get("passed")), d.get("tier", ""),
                   d.get("hard", True), d.get("detail", ""), d.get("score"),
                   d.get("weight", 1.0), d.get("unit", ""))

    def describe(self) -> str:
        v = f"{self.value:.4g}" if isinstance(self.value, float) else str(self.value)
        t = "" if self.target is None else f" vs {self.target}"
        mark = "ok" if self.passed else ("MISS" if not self.hard else "FAIL")
        return f"{self.requirement}: {v}{self.unit and ' ' + self.unit}{t} [{mark}, {self.tier}" \
               f"{'' if self.hard else ', soft'}]" + (f" {self.detail}" if self.detail else "")


@dataclass
class VerifyReport:
    level: str
    ok: bool = True
    score: float = 1.0
    rows: list[Row] = field(default_factory=list)
    failures: list[Failure] = field(default_factory=list)
    unverified: list[str] = field(default_factory=list)
    seconds: float = 0.0

    def to_dict(self) -> dict:
        return {"level": self.level, "ok": self.ok, "score": self.score,
                "rows": [r.to_dict() for r in self.rows],
                "failures": [f.to_dict() for f in self.failures],
                "unverified": list(self.unverified), "seconds": self.seconds}

    @classmethod
    def from_dict(cls, d: dict) -> VerifyReport:
        return cls(d["level"], bool(d.get("ok")), float(d.get("score", 1.0)),
                   [Row.from_dict(r) for r in d.get("rows", [])],
                   [Failure.from_dict(f) for f in d.get("failures", [])],
                   list(d.get("unverified", [])), float(d.get("seconds", 0.0)))

    def describe(self) -> str:
        head = (f"verify {self.level}: {'ok' if self.ok else 'FAIL'}, score {self.score:.2f} "
                f"({self.seconds:.1f} s)")
        lines = [head, *(f"  {r.describe()}" for r in self.rows)]
        if self.failures:
            lines += ["failures:", *(f"  {f.describe().splitlines()[0]}" for f in self.failures)]
        if self.unverified:
            lines.append("not verified at this level: " + ", ".join(self.unverified))
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Target rows
# ---------------------------------------------------------------------------


def target_row(design: Design, f: TargetField, value: float | None, source: str,
               tier: str | None = None, detail: str = "", hard: bool | None = None
               ) -> Row | None:
    """The row of a metric against the spec's target for it (informational when the spec
    sets none; ``None`` when there is neither a value nor a target). ``hard`` overrides the
    target's (an informational second measurement)."""
    t: Target | None = getattr(design.spec, f.section).get(f.name)
    tier = tier or f.tier
    if value is None:
        if t is None:
            return None
        return Row(f.path, source, None, t.describe(), False, tier, effective_hard(t, f),
                   detail=detail or "not measured by this design", unit=f.unit)
    if t is None:
        return Row(f.path, source, float(value), None, True, tier, detail=detail, unit=f.unit)
    is_hard = effective_hard(t, f) if hard is None else hard
    met, miss = t.check(float(value))
    score = None if is_hard else (1.0 if met else max(0.0, 1.0 - miss / t.scale))
    return Row(f.path, source, float(value), t.describe(), met, tier, is_hard, detail, score,
               t.weight, f.unit)


WALK_METRICS = {   # spec field -> the walk model's key
    "stride_mm": "stride_mm", "speed_mm_s": "speed_mm_s", "bob_mm": "bob_mm",
    "slip_mm_per_rev": "slip_rms_mm_per_rev", "tipping_fraction": "tipping_fraction",
}
OUTPUT_METRICS = ("stroke_mm", "straightness_mm", "on_line_fraction", "rotation_deg",
                  "swing_deg", "dwell_deg")


def walk_rows(design: Design, metrics: dict) -> list[Row]:
    """The walk model's metrics as rows against the spec's motion targets."""
    rows = []
    for name, key in WALK_METRICS.items():
        f = target_field("motion", name)
        detail = ""
        if name == "speed_mm_s":
            detail = f"at the servo's no-load {metrics['rpm_max']:g} rpm"
        _push(rows, target_row(design, f, metrics.get(key), "walk", detail=detail))
    return rows


def _push(rows: list[Row], row: Row | None) -> None:
    if row is not None:
        rows.append(row)


def _stage_row(name: str, source: str, failures: list[Failure], value: str, tier: str = "proven"
               ) -> Row:
    if failures:
        return Row(name, source, failures[0].code, None, False, tier, True,
                   failures[0].message.splitlines()[0])
    return Row(name, source, value, None, True, tier)


# ---------------------------------------------------------------------------
# verify
# ---------------------------------------------------------------------------


def verify(design: Design, level: str = "quick") -> VerifyReport:
    """See the module docstring."""
    if level not in LEVELS:
        raise ValueError(f"level is one of {LEVELS}, got {level!r}")
    rep = api._cached(design, "verify", VerifyReport, op=f"verify:{level}", level=level)
    if rep is not None:
        return rep
    t0 = time.time()
    rep = VerifyReport(level)
    rows = rep.rows
    cfg = design.config

    # -- check: program, output, drive, static ------------------------------------
    cr = api.check(design)
    rep.failures += cr.failures
    closures = [s for s in cr.steps if s["kind"] == "closure"]
    prog = [f for f in cr.failures if f.stage == "program"]
    if closures:
        margins = [s["margin_mm"] for s in closures if s["margin_mm"] is not None]
        worst = min(closures, key=lambda s: s["margin_mm"] if s["margin_mm"] is not None
                    else float("-inf"))
        rows.append(Row("program.loops_close", "check", min(margins) if margins else None,
                        ">= 0", not prog, "proven", True,
                        f"{len(closures)} closures; least margin at {worst['point']}", unit="mm"))
    if design.kind == "mechanism" and cr.output is not None:
        rows.append(Row("output.promises", "check", cr.output["broken"] or "hold", None,
                        not cr.output["broken"], "proven"))
        for name in OUTPUT_METRICS:
            _push(rows, target_row(design, target_field("motion", name), cr.output.get(name),
                                   "output"))
    if cr.foot_path is not None:
        _push(rows, target_row(design, target_field("motion", "lift_mm"),
                               cr.foot_path["lift_mm"], "foot_path"))
    if prog or any(f.stage == "output" for f in cr.failures):
        return _done(design, rep, t0)
    drive = [f for f in cr.failures if f.stage in ("drive", "construction")]
    rows.append(_stage_row("drive.one_servo", "check", drive, cr.drive.get("servo", "")))
    if drive:
        return _done(design, rep, t0)
    static = [f for f in cr.failures if f.stage == "static"]
    rows.append(_stage_row("static.crank_route", "check", static,
                           f"{len(cr.clearances)} static clearances, every link has a "
                           "crank point"))
    if design.kind == "walker":
        _push(rows, target_row(design, target_field("motion", "ground_clearance_mm"),
                               cr.ground_clearance_mm, "static"))

    # -- plan --------------------------------------------------------------------
    pr = api.plan(design)
    if not static:
        rep.failures += [f for f in pr.failures if f not in rep.failures]
        rows.append(_stage_row("plan.exists", "plan", pr.failures,
                               f"{pr.n_layers} layers, {pr.height_mm:g} mm"
                               if pr.ok else ""))
    if pr.ok:
        rows.append(Row("plan.optimal", "plan", bool(pr.optimal), None, True, "proven", True,
                        pr.proof))
        _push(rows, target_row(design, target_field("size", "stack_mm"), pr.height_mm, "plan"))

    # -- walk --------------------------------------------------------------------
    wr = api.walk(design)
    if wr.failures:
        rep.failures += wr.failures
        rows.append(_stage_row("walk.model", "walk", wr.failures, "", "measured"))
    rows += wr.rows

    if level == "quick":
        if wr.mass_g is not None:
            _push(rows, target_row(design, target_field("size", "mass_g"), wr.mass_g, "walk",
                                   "estimated", "the walk model's nominal mass"))
        if pr.ok:
            rows += _envelope_estimate(design)
        return _done(design, rep, t0)
    if not pr.ok:
        return _done(design, rep, t0)

    # -- build, contract, clash ---------------------------------------------------
    br = api.build(design)
    rep.failures += br.failures
    rows.append(_stage_row("build.parts", "build", br.failures,
                           f"{br.n_parts} parts" if br.ok else "", "measured"))
    if not br.ok:
        return _done(design, rep, t0)
    _push(rows, target_row(design, target_field("size", "mass_g"), br.mass_g, "build"))
    for axis, v in zip("xyz", br.envelope_mm, strict=True):
        _push(rows, target_row(design, target_field("size", f"envelope_{axis}_mm"), v, "build",
                               detail=f"at t = {br.t:g}"))
    side, tmpl, mech = design.side, design.template, design.mech
    bad = verify_plan(side.plan, tmpl)
    rows.append(Row("plan.verified", "verify_plan", len(bad), "0", not bad, "proven", True,
                    "; ".join(bad[:3]) or "re-checked on 2880 fresh samples"))
    _fail(rep, bad, "plan", "plan_verification_failed")
    for t in CONTRACT_TS[level]:
        problems = check_side(side, tmpl.freeze_at(t))
        rows.append(Row(f"contract@t={t:g}", "check_side", len(problems), "0", not problems,
                        "proven", True, "; ".join(problems[:3])))
        _fail(rep, problems, "contract", "part_outside_claim")
    for t in CLASH_TS[level]:
        m = mech if t == design.build_t else api.fabricate_at(design, t)
        cl, solids = clashes(m), bad_solids(m)
        rows.append(Row(f"clash@t={t:g}", "clashes", len(cl), "0", not cl, "measured", True,
                        "; ".join(f"{c['a']} x {c['b']} {c['mm3']} mm^3" for c in cl[:3])))
        rows.append(Row(f"solids@t={t:g}", "bad_solids", len(solids), "0", not solids,
                        "measured", True, "; ".join(s["part"] for s in solids[:3])))
        _fail(rep, [f"{c['a']} x {c['b']}" for c in cl], "clash", "parts_clash")
        _fail(rep, [s["part"] for s in solids], "clash", "bad_solid")

    # -- layout, bom -------------------------------------------------------------
    size = tuple(design.spec.fit.sheet_size_mm or sheet_size(cfg.sheet))
    extras = list(mech.bom_extras)
    try:
        sheets = pack(mech, size)
        _push(rows, target_row(design, target_field("budget", "sheets"), len(sheets), "layout",
                               detail=f"of {size[0]:g} x {size[1]:g} mm"))
        extras.append(BomLine(cfg.sheet, len(sheets), "laser-cut parts"))
    except ValueError as e:
        f = Failure.from_exception(e, stage="layout")
        rep.failures.append(f)
        rows.append(Row("budget.sheets", "layout", None, None, False, "measured", True,
                        f.message))
    try:
        bom = bom_from_mechanism(replace(mech, bom_extras=extras), group=False,
                                 filament=mech.meta.get("filament", "pla_filament"))
        unpriced = [r.key for r in bom.unpriced]
        unverified = [r.key for r in bom.purchased if not r.verified and not r.same_pack_as]
        _push(rows, target_row(design, target_field("budget", "cost_usd"), bom.cost_usd, "bom",
                               detail=(f"{len(bom.purchased)} items; "
                                       + (f"{len(unpriced)} unpriced; " if unpriced else "")
                                       + f"{len(unverified)} unverified links")))
        _push(rows, target_row(design, target_field("budget", "print_g"), bom.printed_g, "bom",
                               detail="at 100 % infill"))
    except KeyError as e:
        f = Failure.from_exception(e, stage="bom")
        rep.failures.append(f)
        rows.append(Row("bom.resolves", "bom", None, None, False, "measured", True, f.message))

    # -- sim (full) --------------------------------------------------------------
    if level == "full" and design.kind == "walker":
        rows += _sim_rows(design, rep)
    return _done(design, rep, t0)


def _fail(rep: VerifyReport, problems: list[str], stage: str, code: str) -> None:
    if problems:
        rep.failures.append(Failure(stage, code, "; ".join(problems),
                                    culprits=[{"text": p} for p in problems]))


def _envelope_estimate(design: Design) -> list[Row]:
    """The envelope before a build: the joints' sweep over the cycle plus a frame arm's
    half-width (x, y) and the robot's width from the mid-plane (z)."""
    import numpy as np

    from spiderpig.construction.robot import mid_plane

    side, cfg = design.side, design.config
    pts = side.plan.topo.geometry.points
    xy = np.concatenate([np.asarray(v, dtype=float) for v in pts.values()])
    r = max(cfg.params.link_radius, cfg.params.frame_radius)
    x, y = float(np.ptp(xy[:, 0])) + 2 * r, float(np.ptp(xy[:, 1])) + 2 * r
    z_mid = mid_plane(side)
    z = 2 * z_mid if cfg.robot else z_mid
    rows = []
    for axis, v in zip("xyz", (x, y, z), strict=True):
        _push(rows, target_row(design, target_field("size", f"envelope_{axis}_mm"), v, "sweep",
                               "estimated", "joints' sweep + plates; measured after a build"))
    return rows


def _sim_rows(design: Design, rep: VerifyReport) -> list[Row]:
    if importlib.util.find_spec("mujoco") is None:
        return [Row("sim.mujoco", "sim", "not installed", None, True, "estimated", False,
                    "pip install mujoco to simulate")]
    from spiderpig.sim.run import simulate, walk_metrics

    try:
        result = simulate(design.config, seconds=SIM_SECONDS)
        m = walk_metrics(result)
    except Exception as e:  # noqa: BLE001 - MuJoCo's own errors are not ours to classify
        rep.failures.append(Failure("sim", "sim_failed", str(e)))
        return [Row("sim.run", "sim", "failed", None, False, "measured", True, str(e))]
    rows = [
        target_row(design, target_field("motion", "speed_mm_s"), m["speed"], "sim", "measured",
                   f"{SIM_SECONDS:g} s at the drives' full speed; the walk model's row above "
                   "is the spec's", hard=False),
        target_row(design, target_field("motion", "stride_mm"), m["stride"], "sim", "measured",
                   "forward travel per crank revolution in the sim", hard=False),
        Row("sim.stays_up", "sim", not m["fell"], None, not m["fell"], "measured", True,
            f"max tilt {m['max_tilt']:.1f} deg"),
        Row("sim.torque", "sim", m["torque_peak"], f"<= {m['torque_limit']:g}",
            not m["saturates"], "measured", False,
            f"peak {m['torque_peak']:.3f} N·m of {m['torque_limit']:g} stall", unit="N·m"),
    ]
    return rows


def _done(design: Design, rep: VerifyReport, t0: float) -> VerifyReport:
    targeted = {f.path for f, _ in design.spec.targets()}
    seen = {r.requirement for r in rep.rows if r.value is not None}
    rep.unverified = sorted(targeted - seen)
    rep.ok = not rep.failures and all(r.passed for r in rep.rows if r.hard)
    soft = [r for r in rep.rows if r.score is not None and r.target is not None]
    if soft:
        w = sum(r.weight for r in soft) or 1.0
        rep.score = round(sum(r.score * r.weight for r in soft) / w, 4)
    rep.seconds = round(time.time() - t0, 3)
    return api._commit(design, "verify", rep, op=f"verify:{rep.level}")

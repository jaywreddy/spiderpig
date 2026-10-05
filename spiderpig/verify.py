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

from spiderpig import api, servos
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.design import Design
from spiderpig.failure import Failure
from spiderpig.hardware.bom import bom_from_mechanism
from spiderpig.hardware.catalog import adhesive
from spiderpig.hardware.catalog import get as catalog_item
from spiderpig.layout import sheet_lines
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


def least_transmission_angle(closures: list[dict]) -> tuple[float | None, str]:
    """``(angle, joint)``: the least transmission angle over the closures, folded about 90°
    (an angle of 140° is as poor as one of 40°: ``min(lo, 180 - hi)`` of each closure's
    range), and the joint it is at; ``(None, "")`` when no closure has a range."""
    best: tuple[float, str] | None = None
    for s in closures:
        rng = s.get("transmission_deg")
        if not rng or rng[0] is None:
            continue
        folded = min(float(rng[0]), 180.0 - float(rng[1]))
        if best is None or folded < best[0]:
            best = (folded, str(s.get("point", "")))
    return (None, "") if best is None else best


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
        angle, at = least_transmission_angle(closures)
        if angle is not None:
            _push(rows, target_row(design, target_field("motion", "transmission_angle_deg"),
                                   angle, "check", detail=f"the least over the closures is at "
                                   f"{at}, folded about 90°; the ranges are on the card"))
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
    drive = [f for f in cr.failures if f.stage == "drive"]
    rows.append(_stage_row("drive.one_servo", "check", drive, cr.drive.get("servo", "")))
    if drive:
        return _done(design, rep, t0)
    built = [f for f in cr.failures if f.stage == "construction"]
    rows.append(_stage_row("construction.buildable", "check", built,
                           f"{cfg.pillar} pillars, {cfg.pin} pins, {cfg.crank} crank in "
                           f"{cfg.pitch:g} mm layers"))
    if built:
        return _done(design, rep, t0)
    static = [f for f in cr.failures if f.stage == "static"]
    rows.append(_stage_row("static.crank_route", "check", static,
                           f"{len(cr.clearances)} static clearances, every link has a "
                           "crank point"))
    if design.kind == "walker":
        _push(rows, target_row(design, target_field("motion", "ground_clearance_mm"),
                               cr.ground_clearance_mm, "static",
                               detail=(f"the body's lowest point is {cr.lowest_body_part}"
                                       if cr.lowest_body_part else "")))

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
        stack = target_row(design, target_field("size", "stack_mm"), pr.height_mm, "plan")
        if stack is not None:
            if not stack.passed and pr.optimal:     # a proven floor: say what could be thinner
                stack.detail = stack_floor_note(design, pr)
            rows.append(stack)
        if pr.warnings:      # informational: what the constructions warned about
            rows.append(Row("plan.warnings", "plan", len(pr.warnings), None, True, "measured",
                            False, "; ".join(pr.warnings[:4])))

    # -- walk --------------------------------------------------------------------
    wr = api.walk(design)
    if wr.failures:
        rep.failures += wr.failures
        rows.append(_stage_row("walk.model", "walk", wr.failures, "", "measured"))
    rows += wr.rows

    if level == "quick":
        _push(rows, _mass_estimate(design, wr))
        if pr.ok:
            rows += _envelope_estimate(design)
        rows += _cost_floor_rows(design)
        return _done(design, rep, t0)
    if not pr.ok:
        return _done(design, rep, t0)

    # -- build, contract, clash ---------------------------------------------------
    # the contract's crank angles are each a fabrication of the side from the plan alone:
    # workers check them (the design loaded from the store) while this process builds
    contract = _start_contracts(design, CONTRACT_TS[level])
    br = api.build(design)
    rep.failures += br.failures
    rows.append(_stage_row("build.parts", "build", br.failures,
                           f"{br.n_parts} parts" if br.ok else "", "measured"))
    if not br.ok:
        return _done(design, rep, t0)
    if br.warnings:      # informational: what the constructions warned about while building
        rows.append(Row("build.warnings", "build", len(br.warnings), None, True, "measured",
                        False, "; ".join(br.warnings[:4])))
    _push(rows, target_row(design, target_field("size", "mass_g"), br.mass_g, "build",
                           detail=mass_by_group(br)))
    for axis, v in zip("xyz", br.envelope_mm, strict=True):
        _push(rows, target_row(design, target_field("size", f"envelope_{axis}_mm"), v, "build",
                               detail=f"at t = {br.t:g}"))
    side, tmpl, mech = design.side, design.template, design.mech
    bad = verify_plan(side.plan, tmpl)
    rows.append(Row("plan.verified", "verify_plan", len(bad), "0", not bad, "proven", True,
                    "; ".join(bad[:3]) or "re-checked on 2880 fresh samples"))
    _fail(rep, bad, "plan", "plan_verification_failed")
    # the parts are realized again at every other crank angle: what the constructions
    # warn about then is the build's (already on build.warnings), not a terminal's
    with api.capture_warnings():
        for i, t in enumerate(CONTRACT_TS[level]):
            problems = (contract[i].result() if contract is not None
                        else check_side(side, tmpl.freeze_at(t)))
            rows.append(Row(f"contract@t={t:g}", "check_side", len(problems), "0",
                            not problems, "proven", True, "; ".join(problems[:3])))
            _fail(rep, problems, "contract", "part_outside_claim")
        for t in CLASH_TS[level]:
            m = mech if t == design.build_t else api.fabricate_at(design, t)
            cl, solids = clashes(m), bad_solids(m)
            rows.append(Row(f"clash@t={t:g}", "clashes", len(cl), "0", not cl, "measured",
                            True, "; ".join(f"{c['a']} x {c['b']} {c['mm3']} mm^3"
                                            for c in cl[:3])))
            rows.append(Row(f"solids@t={t:g}", "bad_solids", len(solids), "0", not solids,
                            "measured", True, "; ".join(s["part"] for s in solids[:3])))
            _fail(rep, [f"{c['a']} x {c['b']}" for c in cl], "clash", "parts_clash")
            _fail(rep, [s["part"] for s in solids], "clash", "bad_solid")

    # -- layout, bom -------------------------------------------------------------
    size = tuple(design.spec.fit.sheet_size_mm) if design.spec.fit.sheet_size_mm else None
    extras = list(mech.bom_extras)
    try:
        lines = sheet_lines(mech, cfg.sheet, size)
        n = sum(int(x.qty) for x in lines)
        _push(rows, target_row(design, target_field("budget", "sheets"), n, "layout",
                               detail=", ".join(f"{int(x.qty)} x {x.key}" for x in lines)))
        extras += lines
    except ValueError as e:
        f = Failure.from_exception(e, stage="layout")
        rep.failures.append(f)
        rows.append(Row("budget.sheets", "layout", None, None, False, "measured", True,
                        f.message))
    try:
        bom = bom_from_mechanism(replace(mech, bom_extras=extras), group=False,
                                 filament=mech.meta.get("filament", "pla_filament"))
        _push(rows, cost_row(design, bom))
        _push(rows, target_row(design, target_field("budget", "print_g"), bom.printed_g, "bom",
                               detail="at 100 % infill"))
    except KeyError as e:
        f = Failure.from_exception(e, stage="bom")
        rep.failures.append(f)
        rows.append(Row("bom.resolves", "bom", None, None, False, "measured", True, f.message))

    rows += _cut_rule_rows(br.cut_rules or api.cut_rules_of(mech, cfg.sheet), rep)

    # -- joint strength -----------------------------------------------------------
    if design.kind == "walker":
        rows += _strength_rows(design, rep, level)

    # -- sim (full) --------------------------------------------------------------
    if level == "full" and design.kind == "walker":
        rows += _sim_rows(design, rep)
    return _done(design, rep, t0)


def _cut_rule_rows(m: dict, rep: VerifyReport) -> list[Row]:
    """Every laser-cut part against its service's cut rules (:mod:`spiderpig.manufacture`):
    ``manufacture.cut_rules``, the errors (a hole under 1 x the thickness from an edge in
    metal, a hole the service won't cut; ``manufacture`` / ``cut_rule`` with each part as a
    culprit), and a soft ``manufacture.warnings`` row with the rest. ``m``: the build's
    :attr:`api.BuildReport.cut_rules`."""
    from spiderpig import manufacture

    errors = [i for i in m["issues"] if i.get("level") == "error"]
    n_warn = len(m["issues"]) - len(errors)
    if errors:
        rep.failures.append(Failure(
            "manufacture", "cut_rule", "; ".join(manufacture.messages(m, "error")),
            culprits=[{k: i.get(k) for k in ("part", "sheet", "rule", "value", "limit",
                                             "detail", "why", "fix")} for i in errors]))
    rows = [Row("manufacture.cut_rules", "manufacture", len(errors), "0", not errors,
                "measured", True,
                "; ".join(manufacture.messages(m, "error"))
                or f"{m['parts']} laser-cut parts within every service's hard limits")]
    if n_warn:
        rows.append(Row("manufacture.warnings", "manufacture", n_warn, None, True, "measured",
                        False, "; ".join(manufacture.messages(m, "warning"))))
    return rows


def _strength_rows(design: Design, rep: VerifyReport, level: str) -> list[Row]:
    """Every joint's safety factor at the design's loads (:func:`strength.check`): a
    joint under jam SF 1 fails (``strength`` / ``joint_overload``, the joint, its
    numbers and fixes as culprits) when the loads are the design's own (simulated, or
    given); one under the warning limits is a soft row. ``full`` simulates the loads
    (cached per design); ``standard`` uses them when stored, else the family's, whose
    verdict is an estimate: a soft row, no failure (``verify full`` decides)."""
    from spiderpig import strength

    mech = design.mech
    try:
        loads = strength.design_loads(design.config, design.store,
                                      sim=True if level == "full" else "cached")
    except Exception as e:  # noqa: BLE001 - never fail verify over the loads themselves
        return [Row("strength.joints", "strength", None, None, True, "estimated", False,
                    f"no loads: {e}")]
    st = strength.check(mech.meta.get("wobble") or {}, mech.meta, design.config, loads)
    tier = "measured" if loads.get("source") == "sim" else "estimated"
    own = loads.get("source") in ("sim", "override")
    sfs = [r["jam"]["safety"] for r in st["rows"] if r.get("jam")]
    errors = [f for f in st["findings"] if f["level"] == "error"]
    warns = [f for f in st["findings"] if f["level"] == "warning"]
    if errors and own:
        rep.failures.append(Failure(
            "strength", "joint_overload", "; ".join(f["message"] for f in errors),
            culprits=[{"joint": f["joint"], "kind": f["kind"], "sf_jam": f["sf_jam"],
                       "sf_walk": f["sf_walk"], "load_jam": f["load_jam"],
                       "fixes": f["fixes"]} for f in errors],
            numbers={"sf_jam_min": min(sfs) if sfs else None},
            notes=[f"loads: {loads.get('note', '')}"]))
    rows = [Row("strength.joints", "strength", min(sfs) if sfs else None,
                f">= {strength.JAM_ERROR:g}", not errors, tier, own,
                (f"the weakest joint's jam safety factor; loads: {loads.get('note', '')}"
                 + ("" if own else " (an estimate: verify full simulates the design's own)")
                 + ("; " + "; ".join(f["message"] for f in errors) if errors else "")))]
    if warns:
        rows.append(Row("strength.warnings", "strength", len(warns), None, True, tier, False,
                        "; ".join(f"{f['message']} (fix: {f['fixes'][0]})"
                                  for f in warns[:4])))
    return rows


def _start_contracts(design: Design, ts) -> list | None:
    """:func:`check_side` at each of ``ts`` in a worker process of its own
    (:mod:`spiderpig.workers`), the design loaded from its store with its plan re-made:
    futures of the problems, in order. ``None`` (checked here, one after the other)
    without a store, or with workers off (``SPIDERPIG_WORKERS=0``)."""
    from spiderpig import workers

    if not ts or design.store is None or not workers.enabled():
        return None
    return [workers.submit(_contract_job, str(design.store.root), design.id, t) for t in ts]


def _contract_job(root: str, id: str, t: float) -> list[str]:
    """In a worker: the contract of the stored design's side at crank angle ``t``."""
    design = api.load(id, root)
    with api.capture_warnings():
        if not api.plan(design).ok:
            raise RuntimeError(f"{id}: the stored plan no longer holds")
        return check_side(design.side, design.template.freeze_at(t))


def _mass_estimate(design: Design, wr) -> Row | None:
    """The ``size.mass_g`` row before a build: the nominal model's total for what the
    design builds (one side or the robot), its detail saying what the estimate is made
    of (:func:`walk.nominal_mass_breakdown`), so the lever (the sheet, the servo) is
    plain before the build measures it."""
    from spiderpig import walk as walk_model

    cfg = design.config
    f = target_field("size", "mass_g")
    try:
        b = walk_model.nominal_mass_breakdown(cfg, walk_model.side_legs(cfg), robot=cfg.robot)
    except (ValueError, KeyError):
        if wr.mass_g is None:
            return None
        return target_row(design, f, wr.mass_g, "walk", "estimated",
                          "the walk model's nominal mass")
    n = 2 if cfg.robot else 1
    servos_ = f"{n} servo{'s' if n > 1 else ''}"
    plates = f"frame{' and centre' if cfg.robot else ''} plates"
    printed = f"printed crank, pillars, pins{' and ties' if cfg.robot else ''}"
    detail = (f"estimated before a build: links {b['links']:.0f} g, {servos_} {b['servos']:.0f} g, "
              f"{plates} {b['plates']:.0f} g, {printed} {b['printed']:.0f} g"
              + (f", electronics deck {b['deck']:.0f} g" if b.get("deck") else "")
              + f" ({b['note']})")
    return target_row(design, f, b["total"], "walk", "estimated", detail)


def mass_by_group(br) -> str:
    """``by group: links 258 g, drive 115 g, ...`` from a build report's parts manifest,
    heaviest first: what the measured mass is made of."""
    by: dict[str, float] = {}
    for p in br.parts or ():
        g = str(p.get("group", "") or "other").split(":")[0]
        by[g] = by.get(g, 0.0) + float(p.get("mass_g") or 0.0)
    if not by:
        return ""
    return "by group: " + ", ".join(f"{g} {m:.0f} g" for g, m in
                                    sorted(by.items(), key=lambda kv: -kv[1]))


def stack_floor_note(design: Design, pr) -> str:
    """Why a stack that is proven the thinnest can't meet a target it misses, and what
    could be thinner: fewer legs a side (and whether those modules walk), or a thinner
    sheet (the layer pitch)."""
    cfg = design.config
    note = (f"{pr.height_mm:g} mm is proven the thinnest for {cfg.linkage}'s {cfg.module} "
            f"module on {cfg.pitch:g} mm layers ({pr.n_layers} layers, the frame plates "
            f"included)")
    if design.kind == "walker":
        lk = design.lk
        legs = len(lk.leg_modules[cfg.module])
        fewer = [m for m, ls in lk.leg_modules.items() if len(ls) < legs]
        walking = [m for m in fewer if (s := api.module_stride(cfg.linkage, m)) is not None
                   and s >= api.NO_TRAVEL_MM]
        if not fewer:
            note += "; no module of this linkage has fewer legs a side"
        elif walking:
            note += (f"; a thinner stack needs fewer legs a side: of {cfg.linkage}'s modules "
                     f"with fewer, {', '.join(walking)} walk")
        else:
            note += (f"; a thinner stack needs fewer legs a side, and no module of "
                     f"{cfg.linkage} with fewer walks ({', '.join(fewer)} stand still in the "
                     f"walk model)")
    from spiderpig import construction

    crank = construction.crank(cfg.crank)
    least = crank.least_pitch(1.0) if hasattr(crank, "least_pitch") else None
    return note + ("; the sheet sets the layer pitch" + (
        f" (the {cfg.crank} crank's joints need at least {least:g} mm)" if least else ""))


def _cost_item(r) -> str:
    """``name x qty $cost (a pack of N)``: one BOM line for the cost row's detail."""
    qty = f" x {r.qty:g}" if r.qty != 1 else ""
    pack = f" (a pack of {r.pack_qty})" if r.pack_qty and r.pack_qty > r.qty else ""
    return f"{r.name}{qty} ${r.cost_usd:.2f}{pack}"


def _unpriced_item(r) -> str:
    """``qty x name (N packs of M at vendor)``: an unpriced BOM line, so the reader sees what
    the lower bound leaves out and how much of it."""
    packs = f"{r.packs} pack{'s' if r.packs != 1 else ''} of {r.pack_qty}" if r.pack_qty > 1 \
        else f"{r.packs}"
    return f"{r.qty:g} x {r.name} ({packs}{' at ' + r.vendor if r.vendor else ''})"


def cost_row(design: Design, bom) -> Row | None:
    """The ``budget.cost_usd`` row from a BOM. With unpriced items the total is a lower
    bound, and a hard ``max`` (or ``value``) target can't be called met by a lower bound:
    the row then fails and says so, naming every unpriced item (largest quantities first)
    so they can be priced or accepted by hand; a soft target keeps the priced part's
    verdict with the same note."""
    f = target_field("budget", "cost_usd")
    unpriced = sorted(bom.unpriced, key=lambda r: (-r.qty, r.name))
    unverified = [r.key for r in bom.purchased if not r.verified and not r.same_pack_as]
    top = sorted((r for r in bom.purchased if r.cost_usd), key=lambda r: -r.cost_usd)[:4]
    detail = (f"{len(bom.purchased)} items"
              + (f", the largest {'; '.join(_cost_item(r) for r in top)}" if top else "")
              + (f"; {len(unpriced)} unpriced, so the total is a lower bound: "
                 f"{'; '.join(_unpriced_item(r) for r in unpriced)}" if unpriced else "")
              + f"; {len(unverified)} unverified links")
    allowance = design.spec.allowance_usd
    if unpriced and allowance is not None:
        # the spec accepts the unpriced items at this much in all: the row is the priced
        # total plus the allowance, no longer a lower bound
        n = len(unpriced)
        return target_row(design, f, round(bom.cost_usd + allowance, 2), "bom",
                          detail=f"${bom.cost_usd:.2f} priced + ${allowance:.2f} allowed "
                                 f"(budget.allowance_usd) for the {n} unpriced item"
                                 f"{'s' if n != 1 else ''}: {detail}")
    row = target_row(design, f, bom.cost_usd, "bom", detail=detail)
    if row is None or not unpriced:
        return row
    t: Target | None = design.spec.budget.get("cost_usd")
    bounded = t is not None and (t.max is not None or t.value is not None)
    if bounded and row.passed:
        if row.hard:
            row.passed = False
            row.detail = ("at least; the target can't be verified while items are unpriced "
                          "(price them in the catalog, or accept them with an allowance: "
                          "budget.allowance_usd, USD for all of them, is added to the total "
                          "and the row then verifies): " + row.detail)
        else:
            row.detail = "at least (the verdict is on the priced part): " + row.detail
    return row


GLUED_PILLARS = ("printed", "rod", "bearing", "bushing")  # anchored in the plates with CA glue
# threadlocker on its end screws (M4; M3 for standoff_m3)
LOCKED_PILLARS = ("standoff", "standoff_bench", "standoff_m3")
GLUED_PINS = ("bearing", "bushing")                         # an insert glued into each link
EPOXY_PINS = ("chicago", "chicago_bushing")                 # the barrel bonded in its lowest link
LOCKED_PINS = ("chicago", "chicago_bushing")                # threadlocker on each screw
FLOOR_LEAVES_OUT = ("the sheets' count, the crank's screws, the pivots' hardware, rod and "
                    "clips are counted after a build (verify standard)")


def cost_floor(design: Design) -> tuple[float, list[str], list[str]]:
    """What the design buys whatever its parts turn out to be, priced from the catalog before
    any build: the servos (one per side), a spool of filament (the crank is printed), one
    blank of each sheet the parts are cut from, for the robot the centre plates' cement and
    the frame ties' inserts, and what the constructions buy whatever the parts' sizes: a
    bottle of CA glue when a pillar is anchored in the plates or an insert glued into its
    links (the robot's chassis and battery cradle are screwed since 2026-10-05), and the
    printed crank's crankpin nuts (a pack) and, keyed, its hex standoffs (a pack), the bolt
    crank's nylocks (a pack) and its plates' cement, and a bottle of each threadlocker a
    crank's screws, a Chicago screw pin or a standoff pillar's screws take. ``(total, priced
    lines, unpriced names)``: a lower bound on the BOM's total; :data:`FLOOR_LEAVES_OUT`
    says what a build adds."""
    from spiderpig import construction
    from spiderpig.construction.crank import NUT_KEY

    cfg = design.config
    sides = 2 if cfg.robot else 1
    from spiderpig.materials import link_sheets

    crank_plates = getattr(construction.crank(cfg.crank), "plates", False)
    sheets = dict.fromkeys([cfg.sheet, cfg.frame_sheet, *link_sheets(cfg).values()]
                           + ([cfg.crank_sheet] if crank_plates else []))
    lines = [(servos.get(cfg.servo).bom_key, sides), ("pla_filament", 1)]
    lines += [(k, 1) for k in sheets]             # one blank of each sheet at least
    if cfg.robot:
        lines += [("m3_heat_set_insert", 4)]      # the deck's (the centre plates: no adhesive)
    if cfg.pillar in GLUED_PILLARS or cfg.pin in GLUED_PINS:
        lines.append(("ca_glue", 1))
    if cfg.pin in EPOXY_PINS:
        lines.append(("epoxy_2part", 1))
    crank = construction.crank(cfg.crank)
    if hasattr(crank, "for_sheet"):
        crank = crank.for_sheet(cfg.crank_sheet)
    if hasattr(crank, "post_joint"):
        lines.append((NUT_KEY, 1))
    if getattr(crank, "nut_key", None) and not getattr(crank, "single", False):
        # the two-plate bolt crank's nylocks
        lines.append((crank.nut_key, 1))
    if hasattr(crank, "standoff_key"):
        lines.append((crank.standoff_key, 1))
    if getattr(crank, "cement_per_plate", 0) and not any(k == adhesive(cfg.crank_sheet)
                                                         for k, _ in lines):
        lines.append((adhesive(cfg.crank_sheet), 1))   # the bolt crank's bonded plate stacks
    locks = {getattr(crank, "lock_key", None)}
    if cfg.pin in LOCKED_PINS or cfg.pillar in LOCKED_PILLARS:
        locks.add("threadlocker_222")
    lines += [(k, 1) for k in sorted(k for k in locks if k)]
    total, priced, unpriced = 0.0, [], []
    for key, qty in lines:
        item = catalog_item(key)
        offer = item.offer
        if offer is None or offer.price_usd is None:
            unpriced.append(item.name)
            continue
        packs = max(1, -(-qty // max(offer.pack_qty, 1)))
        cost = packs * offer.price_usd
        total += cost
        priced.append(f"{item.name}{f' x {qty}' if qty != 1 else ''} ${cost:.2f}"
                      + (f" (a pack of {offer.pack_qty})" if offer.pack_qty > qty else ""))
    return round(total, 2), priced, unpriced


def _cost_floor_rows(design: Design) -> list[Row]:
    """Before a build: the catalog floor as an informational row, or, when it already
    exceeds a ``max`` target, as the target's failing row (a lower bound can refute a
    ceiling, never confirm it)."""
    try:
        floor, priced, unpriced = cost_floor(design)
    except KeyError:
        return []
    f = target_field("budget", "cost_usd")
    t: Target | None = design.spec.budget.get("cost_usd")
    detail = ("before a build, from the catalog: " + "; ".join(priced)
              + (f"; unpriced: {', '.join(unpriced)}" if unpriced else "")
              + "; " + FLOOR_LEAVES_OUT)
    ceiling = None if t is None else (t.max if t.max is not None else
                                      (t.value + (t.tol if t.tol is not None
                                                  else abs(t.value) * 0.05)
                                       if t.value is not None else None))
    if ceiling is not None and floor > ceiling:
        row = target_row(design, f, floor, "catalog", "estimated",
                         "a lower bound already over the target: " + detail)
        return [row] if row is not None else []
    return [Row("budget.cost_floor_usd", "catalog", floor, None, True, "estimated", False,
                detail, unit="USD")]


def _fail(rep: VerifyReport, problems: list[str], stage: str, code: str) -> None:
    if problems:
        rep.failures.append(Failure(stage, code, "; ".join(problems),
                                    culprits=[{"text": p} for p in problems]))


def _envelope_estimate(design: Design) -> list[Row]:
    """The envelope before a build: the joints' sweep over the cycle plus a frame arm's
    half-width (x, y), and across the sides (z) the robot's width from the mid-plane plus
    what the plan places outside the outer frame plate (an axle's head or clip)."""
    import numpy as np

    from spiderpig.construction.robot import mid_plane

    side, cfg = design.side, design.config
    plan = side.plan
    pts = plan.topo.geometry.points
    xy = np.concatenate([np.asarray(v, dtype=float) for v in pts.values()])
    r = max(cfg.params.link_radius, cfg.params.frame_radius)
    x, y = float(np.ptp(xy[:, 0])) + 2 * r, float(np.ptp(xy[:, 1])) + 2 * r
    outside = max(0, -min((p.layer for p in plan.placed), default=0)) * plan.spec.pitch
    z_mid = mid_plane(side) + outside
    z = 2 * z_mid if cfg.robot else z_mid
    rows = []
    across = (f"the two stacks + the chassis + {outside:g} mm of axle heads outside each "
              f"outer plate" if cfg.robot else
              f"the stack + the servo on the inner plate + {outside:g} mm of axle heads "
              f"outside the outer plate")
    for axis, v in zip("xyz", (x, y, z), strict=True):
        _push(rows, target_row(
            design, target_field("size", f"envelope_{axis}_mm"), v, "sweep", "estimated",
            "the joints' sweep over the cycle + the plates; a build measures one crank angle"
            if axis != "z" else f"{across}; measured after a build"))
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
    # the sim's own rows (``sim.*``): the walk model's rows keep the spec's names, and a
    # report never carries two rows of one name
    speed = target_row(design, target_field("motion", "speed_mm_s"), m["speed"], "sim",
                       "measured", f"{SIM_SECONDS:g} s at the drives' full speed; the walk "
                       "model's motion.speed_mm_s row is the spec's", hard=False)
    stride = target_row(design, target_field("motion", "stride_mm"), m["stride"], "sim",
                        "measured", "forward travel per crank revolution in the sim",
                        hard=False)
    rows = [
        replace(speed, requirement="sim.speed_mm_s"),
        replace(stride, requirement="sim.stride_mm"),
        Row("sim.stays_up", "sim", not m["fell"], None, not m["fell"], "measured", True,
            fall_detail(m, design)),
        Row("sim.torque", "sim", m["torque_peak"], f"<= {m['torque_limit']:g}",
            not m["saturates"], "measured", False,
            f"peak {m['torque_peak']:.3f} N·m of {m['torque_limit']:g} stall", unit="N·m"),
    ]
    return rows


def fall_detail(m: dict, design: Design | None = None) -> str:
    """The ``sim.stays_up`` row's detail: the max tilt, and for a fall when it happened
    (from the drives' start), about which axis, and how far the quasi-static walk model
    was from predicting it (its tipping fraction), so the reader knows whether the fall is
    a dynamic effect the model can't see or a body the sim landed badly."""
    out = f"max tilt {m['max_tilt']:.1f} deg"
    if not m.get("fell"):
        return out
    at, axis = m.get("fell_at_s"), m.get("fell_axis")
    if at is not None:
        out += (f"; fell over {axis + ' ' if axis else ''}at {at:.1f} s into the "
                f"{SIM_SECONDS:g} s run (the drives run from the start)")
    wr = design.reports.get("walk") if design is not None else None
    tip = (wr.metrics or {}).get("tipping_fraction") if wr is not None and wr.ok else None
    if tip is not None:
        out += (f"; the quasi-static model's tipping fraction is {tip:.2f}"
                + (" (it saw no tipping: the fall is dynamic, or the sim's contacts; a lower "
                   "stack, a slower drive or other phases are the levers)" if tip < 0.05
                   else " (it predicted the risk)"))
    return out


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

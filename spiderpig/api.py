"""The operations: pure, synchronous functions of a :class:`spiderpig.design.Design`.

    from spiderpig import api
    design = api.resolve({"kind": "walker", "linkage": {"key": "klann"}})
    api.check(design); api.plan(design); api.walk(design)
    api.build(design); api.verify(design, "standard"); api.export(design, out_dir="out")

Each operation maps onto the engine's own pass (:func:`check` onto the
program checks and the planner's static stage, :func:`plan` onto
:func:`fabricate.design_side`, :func:`walk` onto :func:`walk.api_payload`,
:func:`build` onto :func:`fabricate.fabricate`, :func:`export` onto what
``main.py`` writes), stores its report on the handle
(``design.reports[stage]``) and returns it. A design that merely fails a
stage gets a report with ``ok = False`` and a :class:`spiderpig.failure.Failure`
(stage, code, culprits, numbers, checked recommendations); operations raise
only for programming errors and an invalid spec (:class:`SpecErrors` from
:func:`resolve`).

Every operation is memoised on the handle: :func:`plan` runs :func:`check`
first if it hasn't run, :func:`build` runs :func:`plan`; a report already on
the handle is returned as is (``force=True`` recomputes).
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

import linkage
import servos
import walk as walk_model
from config import BuildConfig, ParamError
from construction.base import Build, ConstructionError
from construction.contract import MAX_OUTSIDE, TOL, _outside, bad_solids, clashes
from construction.envelope import claimed_solid
from fabricate import design_side, fabricate, ground_clearance, side_problem, static_stage
from fabricate import template_for as _template_for
from hardware.bom import BomLine, bom_from_mechanism, group_made
from hardware.catalog import sheet_name, sheet_size
from hardware.mass import filament_density, material_of, part_props
from layout import DEFAULT_KERF, save_sheets
from spiderpig.design import ROOT, Design, Part, design_id, engine_version, jsonable
from spiderpig.failure import Failure, Recommendation
from spiderpig.spec import (
    SECTIONS,
    Spec,
    SpecError,
    SpecErrors,
    default_module,
    effective_hard,
    nearest,
    validate,
)
from stack import ClearanceError, PlanError

# ---------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------


@dataclass
class CheckReport:
    """The stages before any layer: the program's loop closures (``steps``), a mechanism's
    ``output`` check, one leg's ``foot_path`` numbers (a walker), the ``drive``, the static
    ``clearances`` (links that can never share a keep-out's layers), the crank's
    ``crank_facts`` (links that need O free and the crank points that clear them, the body's
    underside) and the ``ground_clearance_mm``."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    steps: list[dict] = field(default_factory=list)
    output: dict | None = None
    foot_path: dict | None = None
    drive: dict = field(default_factory=dict)
    clearances: list[dict] = field(default_factory=list)
    crank_facts: dict | None = None
    ground_clearance_mm: float | None = None
    seconds: float = 0.0


@dataclass
class PlanReport:
    """The layer plan of one side: every link's layer, the stack (``n_layers`` of
    ``pitch_mm``: ``height_mm``), the crank's ``route`` (runs along its posts, whether it
    keeps its bottom bearing), whether it is proven the thinnest (``optimal``, ``proof``),
    and the plan's table (``table``). A failure carries the blockers and recommendations."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    layers: dict[str, int] = field(default_factory=dict)
    top: int | None = None
    n_layers: int | None = None
    height_mm: float | None = None
    pitch_mm: float | None = None
    route: dict | None = None
    optimal: bool | None = None
    proof: str = ""
    cost: int | None = None
    ground_clearance_mm: float | None = None
    table: str = ""
    seconds: float = 0.0


@dataclass
class WalkReport:
    """The quasi-static walk model's metrics (:func:`walk.api_payload`) with each motion
    target's verdict (``rows``); ``skipped`` for a mechanism. Feet sit at their planned
    layers when the side is planned (``feet_z_planned``), the mass is the model's nominal
    one (``mass_nominal``) unless a build gave it."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    skipped: str | None = None
    metrics: dict | None = None
    mass_g: float | None = None
    mass_nominal: bool = True
    feet_z_planned: bool = False
    servo: dict = field(default_factory=dict)
    rows: list = field(default_factory=list)
    seconds: float = 0.0


@dataclass
class BuildReport:
    """Every part built at crank angle ``t`` (the manifest: ``parts``), how many of each
    fabrication, the total mass, the envelope (x, y, z extents) and the mechanism's meta."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    t: float | None = None
    n_parts: int = 0
    counts: dict = field(default_factory=dict)
    mass_g: float | None = None
    envelope_mm: tuple[float, float, float] | None = None
    meta: dict = field(default_factory=dict)
    parts: list[dict] = field(default_factory=list)
    seconds: float = 0.0


@dataclass
class RecheckReport:
    """The contract and clash checks over the parts as they now are: which parts were
    ``edited``, which the contract ``checked`` (inside their group's claims), the
    violations, the pairwise ``clashes`` and the ``bad_solids``."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    edited: list[str] = field(default_factory=list)
    checked: list[str] = field(default_factory=list)
    contract: list[dict] = field(default_factory=list)
    clashes: list[dict] = field(default_factory=list)
    bad_solids: list[dict] = field(default_factory=list)
    seconds: float = 0.0


@dataclass
class ExportReport:
    """What :func:`export` wrote: the ``files`` and the ``manifest`` (also written as
    ``manifest.json``)."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    out_dir: str = ""
    files: list[str] = field(default_factory=list)
    manifest: dict = field(default_factory=dict)
    seconds: float = 0.0


# ---------------------------------------------------------------------------
# resolve
# ---------------------------------------------------------------------------


def resolve(spec: Spec | dict) -> Design:
    """Validate ``spec`` (a :class:`Spec` or its document), infer what it leaves out, build
    its :class:`config.BuildConfig` and return the :class:`Design` handle. The inferred
    values are in ``design.resolved`` (the complete spec the id is computed from);
    :class:`SpecErrors` lists everything wrong with an invalid spec."""
    if not isinstance(spec, Spec):
        spec = Spec.from_dict(spec)
    else:
        errors = validate(spec.to_dict())
        if errors:
            raise SpecErrors(errors)
    lk = linkage.get(spec.linkage.key)
    module = spec.legs.module or default_module(lk)
    sides = spec.legs.sides or (2 if lk.kind == "walker" else 1)
    phases = (None if spec.legs.phases_deg is None
              else tuple(math.radians(p) for p in spec.legs.phases_deg))
    d = BuildConfig()
    try:
        config = BuildConfig(
            linkage=lk.key, module=module, robot=sides == 2, phases=phases,
            proportions=tuple(sorted(spec.linkage.params.items())),
            sheet=spec.materials.sheet or d.sheet, thickness=spec.materials.thickness_mm,
            servo=spec.materials.servo or d.servo,
            pillar=spec.constructions.pillar or d.pillar, pin=spec.constructions.pin or d.pin,
            crank=spec.constructions.crank or d.crank, params=spec.fit.params(),
        )
    except ParamError as e:     # the validator should have said it first
        raise SpecErrors([SpecError("", str(e))]) from None
    warnings: list[str] = []
    if lk.kind == "walker" and sides == 1:
        warnings.append("sides = 1 builds one side (no chassis); the walk metrics still "
                        "model the two-sided robot")
    if servos.get(config.servo).speed_rpm is None:
        warnings.append(f"servo {config.servo} lists no speed: speed_mm_s assumes "
                        f"{walk_model.DEFAULT_RPM:g} rpm")
    resolved = _resolved(spec, config, module, sides)
    engine = engine_version()
    return Design(design_id(resolved, engine), spec, resolved, config, engine, warnings)


def _resolved(spec: Spec, config: BuildConfig, module: str, sides: int) -> dict:
    """The spec with every inferred value written in (defaults of the engine included, so
    a stored design never depends on a default that later moves)."""
    design = config.design_json()
    fit = {k: getattr(config.params, k) for k in config.params.__dataclass_fields__}
    fit["kerf_mm"] = spec.fit.kerf_mm if spec.fit.kerf_mm is not None else DEFAULT_KERF
    fit["sheet_size_mm"] = list(spec.fit.sheet_size_mm or sheet_size(config.sheet))
    targets = {s: {} for s in SECTIONS}
    for f, t in spec.targets():
        targets[f.section][f.name] = t.to_dict(hard=effective_hard(t, f))
    return {
        "version": spec.version, "kind": spec.kind,
        "linkage": {"key": config.linkage, "params": design["proportions"]},
        "legs": {"module": module, "phases_deg": design["phases_deg"], "sides": sides},
        "motion": targets["motion"], "size": targets["size"], "budget": targets["budget"],
        "materials": {"sheet": config.sheet, "thickness_mm": config.thickness,
                      "pitch_mm": config.pitch, "servo": config.servo},
        "constructions": {"pillar": config.pillar, "pin": config.pin, "crank": config.crank},
        "fit": fit,
        "outputs": list(spec.outputs),
    }


# ---------------------------------------------------------------------------
# Linkage cards
# ---------------------------------------------------------------------------


def list_linkages(kind: str | None = None) -> list[dict]:
    """Every registered linkage (``kind``: ``walker`` / ``mechanism`` keeps those): key,
    name, family, kind, its modules (legs per side) and parameters."""
    out = []
    for key in linkage.available(kind):
        lk = linkage.get(key)
        out.append({"key": key, "name": lk.name, "family": lk.family or key, "kind": lk.kind,
                    "modules": {m: len(legs) for m, legs in lk.leg_modules.items()},
                    "params": {k: float(v) for k, v in lk.params.items()},
                    "feet_per_leg": len(lk.feet),
                    "output": lk.output.motion if lk.output else None})
    return out


def describe(key: str) -> dict:
    """One linkage's card: its parameters (default, angle or length, which only scale it),
    links and labels, feet or output, modules with their default phases, the closures at
    the defaults (margins, transmission angles, toggles) and one foot's path numbers (a
    walker) or the output check (a mechanism)."""
    try:
        lk = linkage.get(key)
    except KeyError:
        near = nearest(key, linkage.available())
        raise KeyError(f"unknown linkage {key!r}" + (f"; did you mean {near!r}?" if near
                                                     else "")) from None
    scale = linkage.scale_params(lk)
    card = {
        "key": key, "name": lk.name, "family": lk.family or key, "kind": lk.kind,
        "source": lk.source, "notes": lk.notes,
        "params": [{"name": k, "default": float(v), "angle": k in lk.angles,
                    "scale": k in scale} for k, v in lk.params.items()],
        "scale_params": list(scale),
        "modules": {m: {"legs": len(legs),
                        "phases_deg": [round(math.degrees(ph), 6) for _, ph in legs],
                        "orientations": [o for o, _ in legs]}
                    for m, legs in lk.leg_modules.items()},
        "links": {b: {"joints": list(js), "label": lk.labels.get(b, "")}
                  for b, (js, _) in lk.links.items()},
        "frame": list(lk.frame), "crank": list(lk.crank), "inputs": list(lk.inputs),
        "feet": [list(f) for f in lk.feet],
        "output": jsonable(lk.output) if lk.output else None,
        "closures": [_step_dict(s) for s in lk.check() if s.kind == "closure"],
    }
    if lk.kind == "walker":
        card["foot_path"] = foot_path(lk, {})
    else:
        try:
            card["output_check"] = _output_dict(lk.output_check())
        except linkage.AssemblyError as e:
            card["output_check"] = {"error": str(e)}
    return card


def foot_path(lk: linkage.Linkage, params: dict, n: int = 720) -> dict:
    """One foot's path over a revolution (leg 0's first foot, its defaults unless
    ``params``): ``lift_mm`` (vertical travel), the stance stride and fraction within 2 mm
    of the lowest point, the crank radius, the leg's height and width."""
    ts = 2.0 * math.pi * np.arange(n) / n
    pts = lk.solve(params=params or None).evaluate(ts)
    f = pts[lk.feet[0][1]]
    y0 = float(f[:, 1].min())
    stance = f[:, 1] <= y0 + 2.0
    top = max(float(pts[j][:, 1].max()) for j in lk.points)
    xs = np.concatenate([pts[j][:, 0] for j in lk.points])
    return {
        "lift_mm": float(np.ptp(f[:, 1])),
        "stance_stride_mm": float(np.ptp(f[stance, 0])) if stance.any() else 0.0,
        "stance_fraction": float(stance.mean()),
        "crank_radius_mm": float(np.linalg.norm(pts[lk.crank[1]][0])),
        "height_mm": top - y0,
        "width_mm": float(np.ptp(xs)),
    }


def _step_dict(s) -> dict:
    return {"point": s.point, "kind": s.kind, "refs": list(s.refs), "radii_mm": s.radii,
            "margin_mm": s.margin_mm, "worst_deg": s.worst_deg, "fails_deg": s.fails_deg,
            "transmission_deg": s.angle_deg, "fail_fraction": s.fail_fraction,
            "toggles": s.toggles, "text": s.describe()}


def _output_dict(c) -> dict:
    return {"name": c.output.name, "motion": c.output.motion, "point": c.output.point,
            "extent_mm": list(c.extent_mm), "stroke_mm": c.stroke_mm,
            "straightness_mm": c.straightness_mm, "on_line_fraction": c.on_line,
            "rotation_deg": c.rotation_deg, "swing_deg": c.swing_deg, "dwell_deg": c.dwell_deg,
            "broken": c.broken, "text": c.describe()}


# ---------------------------------------------------------------------------
# check, plan, explain, recommend
# ---------------------------------------------------------------------------


def _finish(design: Design, stage: str, rep, t0: float):
    rep.ok = not rep.failures
    rep.seconds = round(time.time() - t0, 3)
    design.reports[stage] = rep
    design.record(stage, rep.seconds, rep.ok)
    return rep


def check(design: Design, force: bool = False) -> CheckReport:
    """The stages before any layer (:class:`CheckReport`): every loop closes (else
    ``program``), a mechanism's output keeps its promises (else ``output``), the drive
    (``drive``: one servo turns ``t``), the static facts and the crank's route points
    (else ``static``, with what would clear it), the ground clearance."""
    if not force and "check" in design.reports:
        return design.reports["check"]
    t0 = time.time()
    cfg, lk = design.config, design.lk
    params = dict(cfg.proportions)
    rep = CheckReport()
    steps = lk.check(params)
    rep.steps = [_step_dict(s) for s in steps]
    bad = next((s for s in steps if s.fails_deg is not None), None)
    if bad is not None:
        rep.failures.append(Failure(
            "program", "loop_cannot_close", f"{lk.key}: {bad.describe()}",
            culprits=[{"joint": bad.point, "refs": list(bad.refs)}],
            numbers={"margin_mm": bad.margin_mm, "worst_deg": bad.worst_deg,
                     "fails_deg": bad.fails_deg, "fail_fraction": bad.fail_fraction,
                     "radii_mm": bad.radii}))
        return _finish(design, "check", rep, t0)
    if lk.output is not None:
        oc = lk.output_check(params)
        rep.output = _output_dict(oc)
        if oc.broken:
            rep.failures.append(Failure(
                "output", "promise_broken", f"{lk.key}: {oc.broken}",
                culprits=[{"output": oc.output.name, "point": oc.output.point,
                           "body": oc.output.link}],
                numbers={k: v for k, v in rep.output.items()
                         if isinstance(v, (int, float)) and v is not None}))
            return _finish(design, "check", rep, t0)
    else:
        rep.foot_path = foot_path(lk, params)
    rep.drive = {"servo": cfg.servo, "rpm_max": walk_model.servo_info(cfg.servo)["rpm_max"],
                 "inputs": list(lk.inputs)}
    try:
        tmpl = design.template = _template_for(cfg)
        ctx, _, problem = side_problem(tmpl, replace(cfg, robot=False))
    except ValueError as e:      # AssemblyError / OutputError (caught above) / ConstructionError
        rep.failures.append(Failure.from_exception(e, lk=lk))
        return _finish(design, "check", rep, t0)
    rep.clearances = [{"link": c.link, "keepout": c.keepout.owner, "where": c.keepout.where,
                       "dist_mm": c.dist, "need_mm": c.need, "text": c.describe()}
                      for c in problem.clearances]
    if problem.router is not None:
        f = problem.router.facts
        rep.crank_facts = {
            "o_free": {k: max(v, 0.0) for k, v in f.o_free.items()},
            "hosts": {k: list(v) for k, v in f.hosts.items()},
            "detours": [{"name": d.name, "r_mm": d.r, "angle_deg": d.angle, "sweep_mm": d.sweep}
                        for d in f.detours],
            "underside_lowest_mm": f.envelope.lowest if f.envelope is not None else None,
            "allow_mm": f.allow if math.isfinite(f.allow) else None,
        }
    rep.ground_clearance_mm = ground_clearance(tmpl, ctx)
    try:
        static_stage(tmpl, problem, cfg)
    except ClearanceError as e:
        fl = Failure.from_exception(e, lk=lk)
        fails = problem.router.facts.failures
        fl.culprits = [{"body": f.link, "point": f.pin, "dist_mm": f.dist, "need_mm": f.need,
                        "post_mm": f.post, "link_radius_mm": f.link_r, "margin_mm": f.margin,
                        "detour": f.detour, "allow_mm": f.allow} for f in fails]
        if fails:
            fl.numbers = {"dist_mm": fails[0].dist, "need_mm": fails[0].need}
        rep.failures.append(fl)
    return _finish(design, "check", rep, t0)


def plan(design: Design, force: bool = False) -> PlanReport:
    """The layer plan of one side (:class:`PlanReport`), from :func:`fabricate.design_side`
    (cached by the engine per template and config). A ``plan`` failure carries the
    blockers (count, the two shapes, gap, need) and checked recommendations."""
    if not force and "plan" in design.reports:
        return design.reports["plan"]
    t0 = time.time()
    rep = PlanReport()
    cr = check(design, force)
    if not cr.ok:
        rep.failures = list(cr.failures)
        return _finish(design, "plan", rep, t0)
    cfg = design.config
    try:
        d = design.side = design_side(design.template, cfg)
    except PlanError as e:
        rep.failures.append(Failure.from_exception(e, lk=design.lk))
        return _finish(design, "plan", rep, t0)
    except ConstructionError as e:
        rep.failures.append(Failure.from_exception(e, lk=design.lk))
        return _finish(design, "plan", rep, t0)
    p = d.plan
    route = p.choices.get("crank")
    rep.layers = dict(p.layers)
    rep.top, rep.n_layers, rep.height_mm, rep.pitch_mm = p.top, p.top + 1, p.height, p.spec.pitch
    rep.route = (None if route is None else
                 {"runs": [{"at": r.at, "lo": r.lo, "hi": r.hi} for r in route.runs],
                  "bearing": route.bearing})
    rep.optimal, rep.proof, rep.cost = p.optimal, p.proof, p.cost
    rep.ground_clearance_mm = d.ground_clearance_mm
    rep.table = p.describe()
    return _finish(design, "plan", rep, t0)


def explain(design: Design) -> str:
    """Today's ``explain`` text: each stage's verdict on one side of the design."""
    import explain as explain_module

    cfg = design.config
    return explain_module.explain(cfg.linkage, cfg.module, dict(cfg.proportions) or None,
                                  cfg.phases)


def recommend(design: Design) -> list[Recommendation]:
    """The checked recommendations of the stage that fails (the static stage's, else the
    planner's), each with the spec patch that applies it; empty when the design plans."""
    rep = check(design)
    if rep.ok:
        rep = plan(design)
    return [r for f in rep.failures for r in f.recommendations]


# ---------------------------------------------------------------------------
# walk
# ---------------------------------------------------------------------------


def walk(design: Design, force: bool = False) -> WalkReport:
    """The quasi-static walk model of the robot (:func:`walk.api_payload`; no parts):
    its metrics and each motion target's verdict. A mechanism is skipped."""
    if not force and "walk" in design.reports:
        return design.reports["walk"]
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
    return _finish(design, "walk", rep, t0)


# ---------------------------------------------------------------------------
# build, recheck
# ---------------------------------------------------------------------------


def build(design: Design, t: float = 1.0, force: bool = False) -> BuildReport:
    """Fabricate every part at crank angle ``t`` (:func:`fabricate.fabricate`: the robot,
    or one side): the parts land in ``design.parts`` with their live solids, the report
    is the manifest (masses, envelope, counts)."""
    if not force and "build" in design.reports and design.build_t == t:
        return design.reports["build"]
    t0 = time.time()
    pr = plan(design, force)
    if not pr.ok:
        rep = BuildReport(failures=list(pr.failures), t=t)
        return _finish(design, "build", rep, t0)
    try:
        mech = fabricate(design.template, design.config, t)
    except ConstructionError as e:
        rep = BuildReport(failures=[Failure.from_exception(e, stage="fabricate")], t=t)
        return _finish(design, "build", rep, t0)
    return attach_build(design, mech, t, t0)


def attach_build(design: Design, mech, t: float, t0: float | None = None) -> BuildReport:
    """Adopt a fabricated mechanism as the design's build (what :func:`build` does after
    fabricating; a store loading part files, or a test holding a fabricated robot, uses it
    directly). Needs the plan (runs it if it hasn't)."""
    t0 = time.time() if t0 is None else t0
    pr = plan(design)
    if not pr.ok:
        return _finish(design, "build", BuildReport(failures=list(pr.failures), t=t), t0)
    cfg, side = design.config, design.side
    design.mech, design.build_t = mech, t
    z_mid = mech.meta.get("mid_plane")
    servo = servos.get(cfg.servo)
    filament = mech.meta.get("filament")
    parts: dict[str, Part] = {}
    lo = np.full(3, math.inf)
    hi = np.full(3, -math.inf)
    for b in mech.bodies:
        if b.part is None:
            continue
        tag, base = split_side(b.name)
        material, density, fixed = material_of(b, cfg.sheet, filament, servo)
        props = part_props(b.part)
        mass = fixed if fixed is not None else props.volume / 1000.0 * density
        bb = b.placed_part().bounding_box()
        lo, hi = np.minimum(lo, [bb.min.X, bb.min.Y, bb.min.Z]), np.maximum(hi, [bb.max.X,
                                                                                  bb.max.Y,
                                                                                  bb.max.Z])
        parts[b.name] = Part(
            name=b.name, solid=b.part, group=group_of(base, side.plan), side=tag, fab=b.fab,
            material=material, mass_g=mass, volume_mm3=props.volume,
            dims_mm=(bb.size.X, bb.size.Y, bb.size.Z),
            layers=side_layers(side.plan, side_z(tag, z_mid, (bb.min.Z, bb.max.Z))),
            bom_key=b.bom_key, rigid_with=b.rigid_with, pose=b.pose.matrix.tolist(),
            built=b.part,
        )
    design.parts = parts
    counts = {}
    for p in parts.values():
        counts[p.fab] = counts.get(p.fab, 0) + 1
    rep = BuildReport(
        t=t, n_parts=len(parts), counts=counts,
        mass_g=round(sum(p.mass_g for p in parts.values()), 2),
        envelope_mm=tuple(float(v) for v in (hi - lo)) if parts else None,
        meta=jsonable({k: v for k, v in mech.meta.items() if k != "fastened"}),
        parts=[p.to_dict() for p in parts.values()],
    )
    return _finish(design, "build", rep, t0)


def split_side(name: str) -> tuple[str | None, str]:
    """``"L.b1_leg0"`` -> ``("L", "b1_leg0")``; a chassis part has no side."""
    if len(name) > 2 and name[1] == "." and name[0] in "LR":
        return name[0], name[2:]
    return None, name


def side_z(tag: str | None, z_mid: float | None, z: tuple[float, float]) -> tuple[float, float]:
    """A robot part's z range back in its side's coordinates (the left side is the side
    moved down by the mid-plane, the right side its mirror image moved up)."""
    if z_mid is None or tag is None:
        return z
    if tag == "L":
        return z[0] + z_mid, z[1] + z_mid
    return z_mid - z[1], z_mid - z[0]


def to_side(solid, tag: str | None, z_mid: float | None):
    """A robot part's placed solid back in its side's coordinates (see :func:`side_z`)."""
    from build123d import Location, Plane

    if z_mid is None or tag is None:
        return solid
    if tag == "L":
        return solid.moved(Location((0.0, 0.0, z_mid)))
    return solid.moved(Location((0.0, 0.0, -z_mid))).mirror(Plane.XY)


def side_layers(plan, z: tuple[float, float], eps: float = 1e-6) -> tuple[int, ...]:
    """The layers a z range (side coordinates) reaches into (outside the plates too)."""
    pitch = plan.spec.pitch
    lo, hi = math.floor(z[0] / pitch + eps) - 1, math.ceil(z[1] / pitch - eps) + 1
    out = []
    for k in range(lo, hi + 1):
        z0, z1 = plan.z(k)
        if z0 < z[1] - eps and z1 > z[0] + eps:
            out.append(k)
    return tuple(out)


def group_of(name: str, plan) -> str:
    """The construction group that built a side body, from its name: a link (``links``),
    the plates (``frame``), the servo and its screws (``drive``), the crank's segments and
    screws (``crank``), an axle's segments and caps (``pillar:<axis>`` / ``pin:<axis>``),
    the robot's chassis (``chassis``)."""
    if name in plan.layers:
        return "links"
    if name in plan.topo.frame_bodies or name.startswith("frame"):
        return "frame"
    if name.startswith("servo"):
        return "drive"
    groups = sorted({p.group for p in plan.placed if p.group not in plan.layers},
                    key=len, reverse=True)
    for g in groups:
        stem = g.replace(":", "_")
        if name == stem or name.startswith(stem + "_"):
            return g
    if name.startswith("crank"):
        return "crank"
    if name.startswith(("centre_plate", "tie_", "rear_screw")):
        return "chassis"
    return "other"


CLAIMED_GROUPS = ("links", "crank", "pillar", "pin")


def recheck(design: Design, all_parts: bool = False) -> RecheckReport:
    """Re-run the checks over the parts as they now are (an agent may have replaced a
    :class:`Part`'s ``solid``): every part one valid solid, no two parts intersecting
    (:func:`construction.contract.clashes`), and the edited parts (``all_parts``: every
    part) of a claim-bound group (links, crank, pillars, pins) inside their group's claims
    at the build's crank angle. A passing recheck accepts the edited solids as the
    design's; until then an edited solid is outside the correct-by-construction
    guarantee (the plates, the drive and the chassis are covered by the clash check
    alone)."""
    t0 = time.time()
    if design.mech is None:
        raise ValueError("nothing built yet: build(design) first")
    from build123d import Shape

    rep = RecheckReport(edited=[n for n, p in design.parts.items() if p.edited])
    mech, side = design.mech, design.side
    for n, part in design.parts.items():       # the mechanism mirrors the parts as they are
        if not isinstance(part.solid, Shape):
            raise TypeError(f"parts[{n!r}].solid must be a build123d Shape (a Part, Solid or "
                            f"Compound), got {type(part.solid).__name__}")
        mech.body(n).part = part.solid
    rep.bad_solids = bad_solids(mech)
    rep.clashes = clashes(mech)
    z_mid = mech.meta.get("mid_plane")
    build_ = Build(side.ctx, side.plan, design.template.freeze_at(design.build_t))
    for name in (list(design.parts) if all_parts else rep.edited):
        part = design.parts[name]
        group = part.group
        if group.split(":")[0] not in CLAIMED_GROUPS:
            continue
        _, base = split_side(name)
        shapes = build_.shapes(base if group == "links" else group)
        if not shapes:
            continue
        rep.checked.append(name)
        env = claimed_solid(build_, shapes, TOL)
        vol = _outside(to_side(part.placed(), part.side, z_mid), env)
        if vol > MAX_OUTSIDE:
            rep.contract.append({"part": name, "group": group, "mm3_outside": round(vol, 3)})
    if rep.bad_solids:
        rep.failures.append(Failure(
            "clash", "bad_solid", "; ".join(f"{s['part']}: {s['solids']} solids, valid="
                                            f"{s['valid']}" for s in rep.bad_solids),
            culprits=[{"body": s["part"]} for s in rep.bad_solids]))
    if rep.clashes:
        rep.failures.append(Failure(
            "clash", "parts_clash", "; ".join(f"{c['a']} x {c['b']}: {c['mm3']} mm^3"
                                              for c in rep.clashes),
            culprits=[{"body": c["a"], "other": c["b"]} for c in rep.clashes],
            numbers={"mm3": max(c["mm3"] for c in rep.clashes)}))
    if rep.contract:
        rep.failures.append(Failure(
            "contract", "part_outside_claim",
            "; ".join(f"{c['part']}: {c['mm3_outside']} mm^3 outside its claims"
                      for c in rep.contract),
            culprits=[{"body": c["part"], "group": c["group"]} for c in rep.contract],
            numbers={"mm3": max(c["mm3_outside"] for c in rep.contract)}))
    if not rep.failures:
        for n in rep.edited:
            design.parts[n].built = design.parts[n].solid
    return _finish(design, "recheck", rep, t0)


# ---------------------------------------------------------------------------
# verify, export
# ---------------------------------------------------------------------------


def verify(design: Design, level: str = "quick"):
    """:func:`spiderpig.verify.verify`: pass/fail per requirement with an evidence tier."""
    from spiderpig import verify as _verify

    return _verify.verify(design, level)


def export(design: Design, formats=None, out_dir: str | Path = "build") -> ExportReport:
    """Write the chosen ``formats`` (default: the spec's ``outputs``) into ``out_dir``, as
    ``main.py`` does: ``step`` / ``stl`` (the whole machine), ``print`` (one STL per
    different printed part + ``parts.csv``), ``dxf`` (kerf-compensated sheets +
    ``parts.csv``), ``bom`` (csv, md, json), ``glb`` (the viewer's animated bake),
    ``mjcf`` (the MuJoCo model + its metadata); and always ``manifest.json``. Builds first
    if nothing is built; a sheet-packing failure is ``layout``, a catalog miss ``bom``."""
    from spiderpig.spec import OUTPUTS

    t0 = time.time()
    formats = list(formats or design.spec.outputs)
    bad = [f for f in formats if f not in OUTPUTS]
    if bad:
        raise ValueError(f"unknown formats {bad}; have {list(OUTPUTS)}")
    out = Path(out_dir)
    rep = ExportReport(out_dir=str(out))
    if design.mech is None:
        br = build(design)
        if not br.ok:
            rep.failures = list(br.failures)
            return _finish(design, "export", rep, t0)
    out.mkdir(parents=True, exist_ok=True)
    cfg, mech, spec = design.config, design.mech, design.spec
    name = cfg.linkage
    filament = mech.meta.get("filament", "pla_filament")
    files: list[Path] = []
    if "step" in formats:
        mech.export_step(out / f"{name}.step")
        files.append(out / f"{name}.step")
    if "stl" in formats:
        mech.export_stl(out / f"{name}.stl")
        files.append(out / f"{name}.stl")
    groups = None
    if "print" in formats or "bom" in formats:
        groups = {m: group_made(mech.bodies, m) for m in ("laser", "printed")}
    if "print" in formats:
        import main as build_cli

        build_cli.export_prints(groups["printed"], out / "print",
                                density=filament_density(filament))
        files += sorted((out / "print").glob("*"))
    extras = list(mech.bom_extras)
    bom_summary = None
    if "dxf" in formats:
        size = tuple(spec.fit.sheet_size_mm or sheet_size(cfg.sheet))
        kerf = spec.fit.kerf_mm if spec.fit.kerf_mm is not None else DEFAULT_KERF
        try:
            sheets = save_sheets(mech, out / "laser" / f"{name}_sheet", sheet_size=size,
                                 kerf=kerf)
            files += sheets + [out / "laser" / f"{name}_sheet_parts.csv"]
            extras.append(BomLine(cfg.sheet, len(sheets), "laser-cut parts"))
        except ValueError as e:
            rep.failures.append(Failure.from_exception(e, stage="layout"))
    if "bom" in formats:
        title = (f"{cfg.module} {'robot' if cfg.robot else 'side'}, {cfg.servo}, {cfg.pillar} "
                 f"pillars, {cfg.pin} pins, {cfg.crank} crank, {cfg.sheet}")
        try:
            bom = bom_from_mechanism(replace(mech, bom_extras=extras), title=title,
                                     filament=filament, groups=groups)
            files += bom.write(out)
            bom_summary = {"items": len(bom.purchased), "cost_usd": round(bom.cost_usd, 2),
                           "printed_g": bom.printed_g,
                           "unpriced": [r.key for r in bom.unpriced]}
        except KeyError as e:
            rep.failures.append(Failure.from_exception(e, stage="bom"))
    if "glb" in formats:
        if str(ROOT / "viewer") not in sys.path:
            sys.path.append(str(ROOT / "viewer"))
        from bake_gltf import bake_gltf

        bake_gltf(out / f"{name}.glb", cfg, profile=False)
        files.append(out / f"{name}.glb")
    if "mjcf" in formats:
        import json

        from sim.mjcf import build_mjcf

        xml, meta = build_mjcf(cfg)
        (out / f"{name}.xml").write_text(xml)
        (out / f"{name}.json").write_text(json.dumps(meta, indent=1))
        files += [out / f"{name}.xml", out / f"{name}.json"]
    pr, br = design.reports["plan"], design.reports["build"]
    vr = design.reports.get("verify")
    rep.manifest = jsonable({
        "design": design.id, "engine_version": design.engine_version, "t_ref": design.build_t,
        "formats": formats, "files": [str(f.relative_to(out)) for f in files],
        "parts": br.parts, "counts": br.counts, "mass_g": br.mass_g,
        "envelope_mm": br.envelope_mm, "sheet": sheet_name(cfg.sheet),
        "plan": {"layers": pr.n_layers, "height_mm": pr.height_mm, "route": pr.route,
                 "optimal": pr.optimal},
        "bom": bom_summary,
        "verify": None if vr is None else {"level": vr.level, "ok": vr.ok, "score": vr.score,
                                           "failed": [r.requirement for r in vr.rows
                                                      if not r.passed]},
    })
    import json

    (out / "manifest.json").write_text(json.dumps(rep.manifest, indent=1))
    files.append(out / "manifest.json")
    rep.files = [str(f) for f in files]
    return _finish(design, "export", rep, t0)


__all__ = [
    "BuildReport", "CheckReport", "ExportReport", "PlanReport", "RecheckReport", "WalkReport",
    "attach_build", "build", "check", "describe", "explain", "export", "foot_path",
    "list_linkages", "plan", "recheck", "recommend", "resolve", "verify", "walk",
]

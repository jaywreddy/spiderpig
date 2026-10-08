"""The build (:func:`build`, :func:`fabricate_at`) and the recheck of edited parts
(:func:`recheck`)."""


from __future__ import annotations

import math
import time

import numpy as np

from spiderpig import servos
from spiderpig.api.planning import plan
from spiderpig.api.reports import BuildReport, RecheckReport
from spiderpig.api.store_ops import (
    _commit,
    _drop_stored,
    _finish,
    _forget,
    _stored,
    _template,
    capture_warnings,
    design_lock,
    log,
    ran_out,
)
from spiderpig.construction.base import Build, ConstructionError
from spiderpig.construction.contract import MAX_OUTSIDE, TOL, _outside, bad_solids, clashes
from spiderpig.construction.envelope import claimed_solid
from spiderpig.construction.robot import FrameTies, assemble_robot
from spiderpig.design import (
    Design,
    Part,
    jsonable,
)
from spiderpig.fabricate import (
    fabricate_side,
)
from spiderpig.failure import Failure
from spiderpig.hardware.mass import material_of, part_props


def build(design: Design, t: float = 1.0, force: bool = False) -> BuildReport:
    """Fabricate every part at crank angle ``t`` (:func:`fabricate.fabricate`: the robot,
    or one side): the parts land in ``design.parts`` with their live solids, the report
    is the manifest (masses, envelope, counts). A build the store holds at this ``t``
    (from the running engine) comes back from its STEP files instead. Under the design's
    lock (:func:`design_lock`)."""
    with design_lock(design):
        return _build(design, t, force)


def _build(design: Design, t: float, force: bool) -> BuildReport:
    if not force and "build" in design.reports and design.build_t == t and (
            design.mech is not None or not (design.reports["build"].ok
                                            or ran_out(design.reports["build"]))):
        return design.reports["build"]
    t0 = time.time()
    if not force:
        rep = _reload_build(design, t, t0)
        if rep is not None:
            return rep
    pr = plan(design, force)
    if not pr.ok:
        rep = BuildReport(failures=list(pr.failures), t=t)
        return _finish(design, "build", rep, t0)
    try:
        with capture_warnings() as warned:
            mech = fabricate_at(design, t)
    except ConstructionError as e:
        rep = BuildReport(failures=[Failure.from_exception(e, stage="fabricate")], t=t)
        return _finish(design, "build", rep, t0)
    return attach_build(design, mech, t, t0, warnings=warned)


def fabricate_at(design: Design, t: float):
    """The design fabricated at crank angle ``t`` from its own side (what
    :func:`fabricate.fabricate` does, without planning again): one side, or the robot
    with its frame ties and chassis. From the store's fabrication cache when it holds it
    (:mod:`spiderpig.fabcache`)."""
    from spiderpig import fabcache

    tmpl, cfg, d = _template(design), design.config, design.side
    if d is None:
        raise ValueError("plan(design) first")

    def make():
        ties = [FrameTies(d.drive)] if cfg.robot else []
        side = fabricate_side(d, tmpl.freeze_at(t), ties)
        return assemble_robot(side, d) if cfg.robot else side

    return fabcache.fabricated(design.store, tmpl, cfg, d, t, make)


def _reload_build(design: Design, t: float, t0: float) -> BuildReport | None:
    """The build the store holds at ``t`` from the running engine, its parts back from
    their STEP files (a failed build's report as is); ``None`` when there is none or a
    file is missing."""
    doc = _stored(design, "build")
    if doc is None or doc.get("t") != t:
        return None
    if not doc.get("ok"):
        rep = BuildReport.from_dict(doc)
        if ran_out(rep):                # the planner's budget ran out: plan again
            return None
        return _commit(design, "build", rep, cached=True)
    if not plan(design).ok:
        return None
    try:
        mech = design.store.load_mechanism(design.id, doc)
    except (OSError, KeyError, ValueError) as e:
        log.warning("%s: stored build unreadable (%s): rebuilding", design.id, e)
        return None
    # the files hold solids and poses; the joints and outlines (what an edit is placed by)
    # come from the side's template at the build's crank angle, as a fresh build's do
    frozen = {b.name: b for b in _template(design).freeze_at(t).bodies}
    for b in mech.bodies:
        kin = frozen.get(split_side(b.name)[1])
        if kin is not None:
            b.joints, b.outline = list(kin.joints), kin.outline
    return attach_build(design, mech, t, t0, cached=True)


def attach_build(design: Design, mech, t: float, t0: float | None = None, *,
                 cached: bool = False, warnings: list[str] | None = None,
                 props: dict | None = None) -> BuildReport:
    """Adopt a fabricated mechanism as the design's build (what :func:`build` does after
    fabricating; a store loading part files, or a test holding a fabricated robot, uses it
    directly). Needs the plan (runs it if it hasn't). ``cached``: the parts came from the
    store, so they are logged as such and not written again. ``warnings``: what the
    constructions warned about while fabricating (:func:`capture_warnings`). ``props``:
    body name -> :class:`hardware.mass.PartProps` of ``mech``'s parts as they are (the
    bake's and the MJCF's ``props``): a body found there is not measured again, one
    missing is measured and added, so the caller can hand the dict on to the next
    consumer of the same parts."""
    t0 = time.time() if t0 is None else t0
    pr = plan(design)
    if not pr.ok:
        return _finish(design, "build", BuildReport(failures=list(pr.failures), t=t), t0)
    cfg, side = design.config, design.side
    old_t = design.build_t
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
        pp = None if props is None else props.get(b.name)
        if pp is None:
            pp = part_props(b.part)
            if props is not None:
                props[b.name] = pp
        mass = fixed if fixed is not None else pp.volume / 1000.0 * density
        bb = b.placed_part().bounding_box()
        lo, hi = np.minimum(lo, [bb.min.X, bb.min.Y, bb.min.Z]), np.maximum(hi, [bb.max.X,
                                                                                  bb.max.Y,
                                                                                  bb.max.Z])
        z_side = side_z(tag, z_mid, (bb.min.Z, bb.max.Z))
        parts[b.name] = Part(
            name=b.name, solid=b.part, group=group_of(base, side.plan), side=tag, fab=b.fab,
            material=material, dims_mm=(bb.size.X, bb.size.Y, bb.size.Z),
            layers=side_layers(side.plan, z_side), density=density, fixed_mass_g=fixed,
            bom_key=b.bom_key, rigid_with=b.rigid_with, pose=b.pose.matrix.tolist(),
            sheet=b.sheet,
            z_mid=z_mid, z_side=(float(z_side[0]), float(z_side[1])),
            built=b.part, _measured=(b.part, float(pp.volume)),
        )
        assert abs(parts[b.name].mass_g - mass) < 1e-9
    design.parts = parts
    if design.edited or (old_t is not None and old_t != t):
        # the parts as built at ``t``: the accepted edits, or the other crank angle's parts,
        # are gone, and with them what was exported and verified of them
        _forget(design, "export", "verify")
    if not cached and design.store is not None:
        prev = design.store.read_report(design.id, "build")
        if prev is not None and prev.get("t") != t:
            # the store's build moves to ``t``: its export and verify were of the other's
            _drop_stored(design, "export", "verify")
    design.edited = False
    counts = {}
    for p in parts.values():
        counts[p.fab] = counts.get(p.fab, 0) + 1
    rep = BuildReport(
        t=t, n_parts=len(parts), counts=counts,
        mass_g=round(sum(p.mass_g for p in parts.values()), 2),
        envelope_mm=tuple(float(v) for v in (hi - lo)) if parts else None,
        meta=jsonable({k: v for k, v in mech.meta.items() if k != "fastened"}),
        parts=[p.to_dict() for p in parts.values()],
        warnings=list(warnings or []),
        cut_rules=cut_rules_of(mech, cfg.sheet),
    )
    return _finish(design, "build", rep, t0, cached=cached)


def _envelope(mech) -> tuple[float, float, float] | None:
    """The mechanism's extent (mm) over its parts as placed, as a build reports it."""
    lo, hi = np.full(3, math.inf), np.full(3, -math.inf)
    for b in mech.bodies:
        if b.part is None:
            continue
        bb = b.placed_part().bounding_box()
        lo = np.minimum(lo, [bb.min.X, bb.min.Y, bb.min.Z])
        hi = np.maximum(hi, [bb.max.X, bb.max.Y, bb.max.Z])
    return tuple(float(v) for v in (hi - lo)) if np.isfinite(lo).all() else None


def cut_rules_of(mech, default_sheet: str) -> dict:
    """Every laser-cut part of ``mech`` against its service's cut rules
    (:func:`manufacture.check`): :func:`manufacture.summary` (``ok``, errors and warnings
    per rule, the sheets, a message per rule with why and the fix) plus every ``issue``."""
    from spiderpig import manufacture

    m = manufacture.check(mech, default_sheet)
    return jsonable(dict(manufacture.summary(m), issues=m["issues"]))


def cut_rules(design: Design) -> dict | None:
    """The cut-rule summary of the design's build (:attr:`BuildReport.cut_rules`), from the
    build held in memory or stored by the running engine; ``None`` when it hasn't been
    built (the design card's ``cut_rules``: it never fabricates)."""
    rep = design.reports.get("build")
    if rep is not None:
        cr = rep.cut_rules if rep.ok else None
    else:
        doc = _stored(design, "build") or {}
        cr = doc.get("cut_rules") if doc.get("ok") else None
    return {k: v for k, v in cr.items() if k != "issues"} if cr else None


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
    """The layers a z range (side coordinates) reaches into (outside the plates too; its
    clearance gaps and thicker plates at their own z)."""
    return tuple(plan.layout.layers_between(z[0] + eps, z[1] - eps))


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
    design's on this handle (its build report, and an ``export`` or ``verify`` after it,
    which run afresh: the earlier ones are dropped); the store's build keeps the parts as
    built (a reloaded design is the unedited one), so export the edited design from this
    handle. Until then an edited solid is outside the correct-by-construction guarantee
    (the plates, the drive and the chassis are covered by the clash check alone)."""
    t0 = time.time()
    if design.mech is None:
        raise ValueError("nothing built yet: build(design) first")
    from build123d import Shape

    rep = RecheckReport(edited=[n for n, p in design.parts.items() if p.edited])
    mech = design.mech
    for n, part in design.parts.items():       # every solid a shape before any is taken
        if not isinstance(part.solid, Shape):
            raise TypeError(f"parts[{n!r}].solid must be a build123d Shape (a Part, Solid or "
                            f"Compound), got {type(part.solid).__name__}")
    for n, part in design.parts.items():       # the mechanism mirrors the parts as they are
        mech.body(n).part = part.solid
    try:
        _recheck_parts(design, rep, all_parts)
    except BaseException:
        # nothing accepted: the mechanism (what export and verify read) back to the parts
        # last accepted
        for n, part in design.parts.items():
            mech.body(n).part = part.built
        raise
    if not rep.failures and rep.edited:
        for n in rep.edited:
            design.parts[n].built = design.parts[n].solid
        br = design.reports.get("build")
        if br is not None:      # the handle's build now describes the edited parts
            br.mass_g = round(sum(p.mass_g for p in design.parts.values()), 2)
            br.parts = [p.to_dict() for p in design.parts.values()]
            br.envelope_mm = _envelope(mech)
            br.cut_rules = cut_rules_of(mech, design.config.sheet)
        # what was written or checked from the parts before the edit (the cut files, the
        # verify's rows) no longer describes them: exported and verified again on asking,
        # and kept off the store (whose build is the unedited design: a reload's)
        design.edited = True
        _forget(design, "export", "verify")
    elif rep.failures:
        # rejected: the mechanism (what an export or a verify reads) goes back to the parts
        # last accepted; the rejected solids stay only on the parts, to be edited again
        for n, part in design.parts.items():
            mech.body(n).part = part.built
    # a recheck of handle-local edits says nothing about the store's (unedited) build
    return _finish(design, "recheck", rep, t0, write=not (rep.edited or design.edited))


def _recheck_parts(design: Design, rep: RecheckReport, all_parts: bool) -> None:
    """:func:`recheck`'s checks over the mechanism as it now is: no-op edits, solids,
    clashes, each edited part inside its group's claims; the failures on ``rep``."""
    mech, side = design.mech, design.side
    for n in rep.edited:      # an edit that missed its part (a cut placed in the wrong frame)
        part = design.parts[n]
        built = float(part_props(part.built).volume)
        if abs(part.volume_mm3 - built) <= 1e-6 * max(built, 1.0):
            rep.notes.append(
                f"{n}: the edited solid has the build's volume ({built:.2f} mm3), so the edit "
                f"changed nothing; a cut placed by the side's coordinates (a joint's xy, a "
                f"layer's z) goes through Part.locate: a robot's part sits in the world frame")
    rep.bad_solids = bad_solids(mech)
    rep.clashes = clashes(mech)
    z_mid = mech.meta.get("mid_plane")
    build_ = Build(side.ctx, side.plan, _template(design).freeze_at(design.build_t))
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

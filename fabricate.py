"""Fabrication: from a kinematic template to parts, a layer plan and a bill of materials.

:func:`design_side` rationalizes one side of the walker (see
:mod:`construction.base`): it builds the construction groups for the
template (servo drive, crank, one axle per pillar and pin, the link plates,
the frame plates), collects their claims, and solves the layer plan
(:mod:`stack`). :func:`fabricate` then builds every part at a crank angle
``t``: groups realize in dependency order, and the laser-cut plates are cut
last with every hole the other groups asked for.

With ``BuildConfig.robot`` (the default) the result is the whole robot: two
mirror-image sides with their servos back to back in one frame
(:mod:`construction.robot`). The robot's side is the side on its own with
the frame ties added at build time, so the two share one design.

Every body carries ``fab`` ("laser", "printed" or "purchased"), purchased
ones a catalog ``bom_key``; non-kinematic bodies carry ``rigid_with`` (what
they move with). ``mech.bom_extras`` lists unmodelled purchases.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import construction
import linkage
import servos
from config import BuildConfig
from construction.base import Build, Context, Realized
from construction.robot import FrameTies, assemble_robot
from construction.underside import underside
from hardware.catalog import sheet_name
from mechanism import Mechanism
from servos.mount import DriveGroup
from stack import (
    Clearance,
    ClearanceError,
    Deadline,
    PlanError,
    StackPlan,
    StackProblem,
    StackSpec,
    static_clearances,
    topology_from_template,
    verify_plan,
)


def template_for(config: BuildConfig):
    """The kinematic template of one side for ``config`` (linkage, module, phases, params)."""
    return linkage.build_module_template(config.module, config.phases, dict(config.proportions),
                                         config.linkage)


@dataclass
class SideDesign:
    """One side, rationalized: its groups (in build order) and its layer plan.

    ``clearances`` are the static facts the plan had to respect (links that
    can never share a layer with some group's keep-out); :attr:`checks` how
    well each loop of the linkage closes. ``ground_clearance_mm``: how far the
    body's lowest point (:mod:`construction.underside`) rides above the lowest
    foot point over the cycle (``None`` without feet).
    """

    config: BuildConfig
    ctx: Context
    groups: list
    plan: StackPlan
    clearances: list[Clearance] = field(default_factory=list)
    ground_clearance_mm: float | None = None

    @property
    def drive(self) -> DriveGroup:
        return self.groups[0]

    @property
    def checks(self):
        return self.config.lk.check(dict(self.config.proportions))


def side_clearances(ctx: Context, groups: list) -> list[Clearance]:
    """Static clearance check: every link against every group's keep-outs."""
    keepouts = [k for g in groups for k in g.keepouts(ctx)]
    return static_clearances(ctx.topo, keepouts, ctx.params.link_radius, ctx.params.margin)


def side_problem(tmpl, config: BuildConfig, deadline: Deadline | None = None,
                 hint: bool = True) -> tuple[Context, list, StackProblem]:
    """One side's groups (interfaces resolved) and the layer problem their claims pose: the
    static clearances, the body's underside (``ctx.interfaces["underside"]``) and the
    crank's router (its static facts in ``problem.router.facts``). ``deadline``: what is
    left of it caps the search (within ``StackSpec.max_seconds``); ``hint``: plan the
    single module first for the leg hint (not needed for the static stage alone)."""
    topo = topology_from_template(tmpl)
    ctx = Context(topo=topo, params=config.params, pitch=config.pitch,
                  servo=servos.get(config.servo), config=config)
    groups = construction.side_groups(ctx, config)
    for g in groups:
        iface = g.interface(ctx)
        if iface is not None:
            ctx.interfaces[g.name] = iface
    claims = [c for g in groups for c in g.claims(ctx)]
    spec = StackSpec(pitch=ctx.pitch, margin=config.params.margin)
    if deadline is not None:
        spec = replace(spec, max_seconds=min(spec.max_seconds, deadline.remaining))
    crank = next((g for g in groups if isinstance(g, construction.CrankGroup)), None)
    ctx.interfaces["underside"] = envelope = underside(ctx, crank and crank.reach(ctx))
    router = crank and crank.router(ctx, envelope, spec.margin, spec.drop_bearing)
    return ctx, groups, StackProblem(topo, claims, spec, router, side_clearances(ctx, groups),
                                     hint=_leg_hint(config, deadline) if hint else None)


def _leg_hint(config: BuildConfig, deadline: Deadline | None = None) -> dict[str, int] | None:
    """One leg's plan (the single module's), for the planner to try each leg of a bigger
    module at (a hint for the order it tries layers in, nothing more)."""
    if config.module == "single":
        return None
    one = replace(config, module="single", phases=None, robot=False)
    try:
        return design_side(template_for(one), one, advise=False, deadline=deadline).plan.layers
    except ValueError:
        return None


def ground_clearance(tmpl, ctx: Context) -> float | None:
    """How far the body's lowest point rides above the lowest foot point (mm)."""
    import linkage

    feet = linkage.feet_of(tmpl)
    if not feet:
        return None
    pts = ctx.topo.geometry.points
    low = min(float(pts[ctx.topo.point_of[f]][:, 1].min()) for f in feet)
    return ctx.interfaces["underside"].clearance(low)


_DESIGNS: dict[tuple, SideDesign] = {}
_LAYOUTS: dict[tuple, StackPlan] = {}


def static_stage(tmpl, problem: StackProblem, config: BuildConfig | None = None) -> None:
    """The planner's static stage: a link no crank route can let through stops here (with
    ``config``: and what would clear it, checked; :mod:`recommend`)."""
    if problem.router is None or not problem.router.facts.failures:
        return
    failures = problem.router.facts.failures
    err = ClearanceError(f"{tmpl.name}: " + "\n  ".join(f.describe() for f in failures))
    if config is not None:
        from recommend import recommend

        recs, notes = recommend(config, failures=tuple(failures))
        err = err.with_notes(*notes).with_recommendations(recs)
    raise err


def _reuse(problem: StackProblem, solved: StackPlan | None) -> StackPlan | None:
    """``solved``'s layout in ``problem``, if every claim still clears (checked)."""
    if solved is None:
        return None
    try:
        plan = problem.plan(solved.layers, solved.top, solved.choices)
    except ValueError:
        return None
    if verify_plan(plan):
        return None
    plan.optimal, plan.proof, plan.cost = solved.optimal, solved.proof, solved.cost
    return plan


def design_side(tmpl, config: BuildConfig | None = None, advise: bool = True,
                deadline: Deadline | None = None) -> SideDesign:
    """Rationalize and plan one side (cached per template and config). A failure of the
    planner's stages says what would clear it (``advise``, :mod:`recommend`).

    The planner's search is bounded by its node budgets and ``StackSpec.max_seconds``
    (a plan cut short is returned unproven; none found raises :class:`stack.PlanError`),
    and the checks of the recommendations share one more such deadline, so this returns
    within a few minutes at worst. ``deadline``: a caller's (the recommendation checks'):
    what is left of it caps every search here, and the design isn't cached.
    """
    config = replace(config or BuildConfig(), robot=False)
    meta = tuple(sorted((k, v) for k, v in tmpl.meta.items()))
    key = (tmpl.name, tuple(b.name for b in tmpl.bodies), tuple(tmpl.connections), meta, config)
    if key in _DESIGNS:
        return _DESIGNS[key]
    ctx, groups, problem = side_problem(tmpl, config, deadline)
    static_stage(tmpl, problem, config if advise else None)
    # The robot's side has the same layout as the side on its own; reuse
    # a solved layout when every claim still clears (checked, not assumed).
    layout_key = key[:4] + (replace(config, robot=False),)
    plan = _reuse(problem, _LAYOUTS.get(layout_key))
    if plan is None:
        try:
            plan = problem.solve()
        except PlanError as e:
            involved = [c for c in problem.clearances
                        if any(c.link in b and c.keepout.owner in b for b in e.blockers)]
            e = e.with_notes(*(["static clearances behind it:",
                                *(c.describe() for c in involved[:8])] if involved else []))
            if advise and involved:
                from recommend import recommend

                recs, notes = recommend(config, clearances=tuple(involved), plan=True)
                e = e.with_notes(*notes).with_recommendations(recs)
            raise e from None
        if deadline is None:
            _LAYOUTS[layout_key] = plan
    design = SideDesign(config, ctx, groups, plan, list(problem.clearances),
                        ground_clearance(tmpl, ctx))
    if deadline is None:
        _DESIGNS[key] = design
    return design


def fabricate_side(design: SideDesign, mech: Mechanism, extra_groups=()) -> Mechanism:
    """Build every part of one side for ``mech`` (the side's template frozen at some ``t``).

    ``extra_groups`` (the robot's frame ties) realize after the design's
    groups and before the plates, which cut what they ask for.
    """
    build = Build(design.ctx, design.plan, mech)
    done = Realized()
    plates = [g for g in design.groups if g.cuts]
    for g in [g for g in design.groups if not g.cuts] + list(extra_groups) + plates:
        done.merge(g.realize(build, done))
    bodies = {b.name: replace(b, part=None) for b in mech.bodies}
    extra = []
    for b in done.bodies:
        if b.name in bodies:    # a group built a kinematic body's part (a link, the frame)
            kin = bodies[b.name]
            kin.part, kin.fab, kin.bom_key = b.part, b.fab, b.bom_key
            kin.color = b.color or kin.color
        else:
            extra.append(b)
    cfg = design.config
    meta = dict(mech.meta)
    meta.update(
        sheet=cfg.sheet, sheet_name=sheet_name(cfg.sheet), pitch=design.ctx.pitch,
        servo=cfg.servo, pillar=cfg.pillar, pin=cfg.pin, crank=cfg.crank,
        layers=design.plan.top + 1, stack_mm=design.plan.height,
    )
    return Mechanism(
        name=mech.name,
        bodies=list(bodies.values()) + extra,
        connections=list(mech.connections),
        meta=meta,
        bom_extras=list(mech.bom_extras) + done.extras,
    )


def fabricate(tmpl, config: BuildConfig | None = None, t: float = 1.0) -> Mechanism:
    """The fabricated walker at crank angle ``t`` (one side unless ``config.robot``)."""
    config = config or BuildConfig()
    design = design_side(tmpl, config)
    ties = [FrameTies(design.drive)] if config.robot else []
    side = fabricate_side(design, tmpl.freeze_at(t), ties)
    return assemble_robot(side, design) if config.robot else side

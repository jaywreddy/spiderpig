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
from typing import TYPE_CHECKING, cast

from spiderpig import construction, linkage, servos
from spiderpig.config import BuildConfig
from spiderpig.construction.base import Build, Context, Realized
from spiderpig.construction.robot import FrameTies, assemble_robot
from spiderpig.construction.underside import underside
from spiderpig.hardware.catalog import sheet_name
from spiderpig.mechanism import Mechanism
from spiderpig.servos.mount import DriveGroup
from spiderpig.stack import (
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

if TYPE_CHECKING:
    from spiderpig.construction.route import CrankFacts, CrankRouter
    from spiderpig.construction.underside import Underside


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
    facts: CrankFacts | None = None     # the crank router's static facts

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
    heads = config.heads
    if heads == "best" and any(isinstance(g, construction.CrankGroup) for g in groups):
        # the crank's single-plate webs keep their screws' heads in clearance gaps (a sunk
        # head would stand in a rider's layer): no plan with every head sunk exists; in gaps,
        # else the pivots' heads sunk with the crank's in gaps (stack.HEADS_ORDER)
        heads = "gap_sink"
    spec = StackSpec(pitch=ctx.pitch, margin=config.params.margin, heads=heads,
                     **plate_z(ctx))
    if deadline is not None:
        spec = replace(spec, max_seconds=min(spec.max_seconds, deadline.remaining))
    crank = next((g for g in groups if isinstance(g, construction.CrankGroup)), None)
    ctx.interfaces["underside"] = envelope = underside(ctx, crank and crank.reach(ctx))
    router = crank and crank.router(ctx, envelope, spec.margin, spec.drop_bearing)
    problem = StackProblem(topo, claims, spec, router, side_clearances(ctx, groups),
                           hint=_leg_hint(config, deadline) if hint else None)
    return ctx, groups, problem


def plate_z(ctx: Context) -> dict:
    """What the plan's z needs from the materials (:class:`stack.StackSpec`): the frame
    plates' thickness, each link's that isn't cut from the default sheet, and the
    thicknesses a clearance gap may have (:func:`materials.gap_options`)."""
    from spiderpig.materials import gap_options

    link_t = tuple(sorted((n, ctx.sheet_t("link", n)) for n in ctx.topo.links
                          if abs(ctx.sheet_t("link", n) - ctx.pitch) > 1e-9))
    return {"frame_t": ctx.sheet_t("frame"), "link_t": link_t, "gaps": gap_options()}


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
    from spiderpig import linkage

    feet = linkage.feet_of(tmpl)
    if not feet:
        return None
    pts = ctx.topo.geometry.points
    low = min(float(pts[ctx.topo.point_of[f]][:, 1].min()) for f in feet)
    envelope = cast("Underside", ctx.interfaces["underside"])  # side_problem sets it so
    return envelope.clearance(low)


_DESIGNS: dict[tuple, SideDesign] = {}
_LAYOUTS: dict[tuple, StackPlan] = {}


def _key(tmpl, config: BuildConfig) -> tuple:
    """What a side design is cached by: the template (its name, bodies, connections and
    the ``meta`` that made it) and the side's config."""
    meta = tuple(sorted((k, v) for k, v in tmpl.meta.items()))
    return (tmpl.name, tuple(b.name for b in tmpl.bodies), tuple(tmpl.connections), meta,
            replace(config, robot=False))


def remember(tmpl, design: SideDesign) -> None:
    """Make ``design`` (a side the store re-made and verified, :func:`spiderpig.api.plan`)
    what :func:`design_side` answers for its template and config, so a build of it
    (:func:`fabricate`) plans nothing again. A design this process already holds for them
    stays."""
    key = _key(tmpl, design.config)
    _DESIGNS.setdefault(key, design)
    _LAYOUTS.setdefault((*key[:4], replace(design.config, robot=False)), design.plan)


def router_facts(problem: StackProblem) -> CrankFacts | None:
    """The static facts of ``problem``'s crank router (``None``: no router)."""
    if problem.router is None:
        return None
    router = cast("CrankRouter", problem.router)  # side_problem's router is the crank's
    return router.facts


def static_stage(tmpl, problem: StackProblem, config: BuildConfig | None = None) -> None:
    """The planner's static stage: a link no crank route can let through stops here (with
    ``config``: and what would clear it, checked; :mod:`recommend`)."""
    facts = router_facts(problem)
    if facts is None or not facts.failures:
        return
    failures = facts.failures
    err = ClearanceError(f"{tmpl.name}: " + "\n  ".join(f.describe() for f in failures))
    if config is not None:
        from spiderpig.recommend import recommend

        recs, notes = recommend(config, failures=tuple(failures))
        err = err.with_notes(*notes).with_recommendations(recs)
    raise err


def _reuse(problem: StackProblem, solved: StackPlan | None) -> StackPlan | None:
    """``solved``'s layout in ``problem``, if every claim still clears (checked)."""
    if solved is None:
        return None
    try:
        plan = problem.plan(solved.layers, solved.top, solved.choices, solved.heads)
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
    key = _key(tmpl, config)
    if key in _DESIGNS:
        return _DESIGNS[key]
    ctx, groups, problem = side_problem(tmpl, config, deadline)
    static_stage(tmpl, problem, config if advise else None)
    # The robot's side has the same layout as the side on its own; reuse
    # a solved layout when every claim still clears (checked, not assumed).
    layout_key = (*key[:4], replace(config, robot=False))
    plan = _reuse(problem, _LAYOUTS.get(layout_key))
    if plan is None:
        try:
            plan = problem.solve()
        except PlanError as e:
            involved = [c for c in problem.clearances
                        if any(c.link in b and c.keepout.owner in b for b in e.blockers)]
            e = e.with_notes(*(["static clearances behind it:",
                                *(c.describe() for c in involved[:8])] if involved else []))
            if advise:      # no involved clearance: recommend still says what it can
                from spiderpig.recommend import recommend

                recs, notes = recommend(config, clearances=tuple(involved), plan=True)
                e = e.with_notes(*notes).with_recommendations(recs)
            raise e from None
        if deadline is None:
            _LAYOUTS[layout_key] = plan
    design = SideDesign(config, ctx, groups, plan, list(problem.clearances),
                        ground_clearance(tmpl, ctx),
                        router_facts(problem))
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
            kin.part, kin.fab, kin.bom_key, kin.sheet = b.part, b.fab, b.bom_key, b.sheet
            kin.color = b.color or kin.color
        else:
            extra.append(b)
    cfg = design.config
    meta = dict(mech.meta)
    meta.update(
        sheet=cfg.sheet, sheet_name=sheet_name(cfg.sheet), pitch=design.ctx.pitch,
        servo=cfg.servo, pillar=cfg.pillar, pin=cfg.pin, crank=cfg.crank,
        layers=design.plan.top + 1, stack_mm=design.plan.height,
        **done.notes,
    )
    return Mechanism(
        name=mech.name,
        bodies=list(bodies.values()) + extra,
        connections=list(mech.connections),
        meta=meta,
        bom_extras=list(mech.bom_extras) + done.extras,
    )


def fabricate(tmpl, config: BuildConfig | None = None, t: float = 1.0, *,
              store=None) -> Mechanism:
    """The fabricated walker at crank angle ``t`` (one side unless ``config.robot``).

    ``store`` (else the one :func:`spiderpig.fabcache.serving` names, else none): served
    from that store's fabrication cache when it holds this design, plan and ``t``, else
    fabricated and kept there (:mod:`spiderpig.fabcache`). Either way the mechanism is
    the caller's own."""
    from spiderpig import fabcache

    config = config or BuildConfig()
    design = design_side(tmpl, config)

    def build() -> Mechanism:
        ties = [FrameTies(design.drive)] if config.robot else []
        side = fabricate_side(design, tmpl.freeze_at(t), ties)
        return assemble_robot(side, design) if config.robot else side

    store = store if store is not None else fabcache.current()
    return fabcache.fabricated(store, tmpl, config, design, t, build)


MADE = ("laser", "printed")
"""The fabrications the design makes itself: each part one solid (a purchased model may be
several)."""


def split_parts(mech: Mechanism) -> list[tuple[str, int]]:
    """``(part, solids)`` of every laser-cut or printed part that isn't exactly one solid
    (a plate its holes cut in two: :func:`shapes.difference` keeps the pieces as one
    ``Compound``). A count only, no validity check (that is
    :func:`construction.contract.bad_solids`, the audit's and verify's): cheap enough for
    every build."""
    return [(b.name, n) for b in mech.bodies
            if b.part is not None and getattr(b, "fab", None) in MADE
            and (n := len(b.part.solids())) != 1]

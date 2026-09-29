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
(:mod:`construction.robot`).

Every body carries ``fab`` ("laser", "printed" or "purchased"), purchased
ones a catalog ``bom_key``; non-kinematic bodies carry ``rigid_with`` (what
they move with). ``mech.bom_extras`` lists unmodelled purchases.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field, replace

import construction
import servos
from construction.base import Build, Context, Params, Realized
from construction.plates import FramePlates, LinkPlates
from mechanism import Mechanism
from servos.mount import DriveGroup
from stack import (
    Clearance,
    ClearanceError,
    PlanError,
    StackPlan,
    StackProblem,
    StackSpec,
    impossible,
    static_clearances,
    topology_from_template,
    verify_plan,
)

MODULES = ("single", "double", "decker", "quad")


@dataclass(frozen=True)
class BuildConfig:
    """What to build and how. Construction keys refer to :mod:`construction` registries."""

    module: str = "quad"              # legs per side (see MODULES)
    robot: bool = True                # two mirrored sides, servos back to back in one frame
    sheet: str = "acrylic_3mm"        # catalog item for the sheet stock (sets the layer pitch)
    servo: str = servos.DEFAULT
    pillar: str = "printed"           # frame pivots
    pin: str = "printed"              # pivots between links
    crank: str = "printed"
    params: Params = field(default_factory=Params)
    thickness: float | None = None    # override the sheet's nominal thickness
    phases: tuple[float, ...] | None = None        # crank phase per leg (rad); None = module's
    proportions: tuple[tuple[str, float], ...] = ()  # overrides of the linkage's params
    linkage: str = "klann"            # see linkage.available()


def template_for(config: BuildConfig):
    """The kinematic template of one side for ``config`` (linkage, module, phases, params)."""
    from linkage import build_module_template

    return build_module_template(config.module, config.phases, dict(config.proportions),
                                 config.linkage)


def sheet_thickness(config: BuildConfig) -> float:
    if config.thickness is not None:
        return config.thickness
    from hardware.catalog import get

    return float(get(config.sheet).dims["thickness"])


@dataclass
class SideDesign:
    """One side, rationalized: its groups (in build order) and its layer plan.

    ``clearances`` are the static facts the plan had to respect (links that
    can never share a layer with some group's keep-out); :attr:`checks` how
    well each loop of the linkage closes.
    """

    config: BuildConfig
    ctx: Context
    groups: list
    plan: StackPlan
    clearances: list[Clearance] = field(default_factory=list)

    @property
    def drive(self) -> DriveGroup:
        return self.groups[0]

    @property
    def checks(self):
        import linkage

        return linkage.get(self.config.linkage).check(dict(self.config.proportions))


def side_groups(ctx: Context, config: BuildConfig) -> list:
    """The groups of one side, in dependency order (frame plates last)."""
    topo = ctx.topo
    groups: list = [DriveGroup(ctx.servo)]
    if topo.center is not None:
        groups.append(construction.CrankGroup(construction.crank(config.crank)))
    for ax in topo.axes:
        if ax.kind == "frame":
            groups.append(construction.AxleGroup(ax, construction.axle(config.pillar)))
        elif ax.kind == "pin":
            groups.append(construction.AxleGroup(ax, construction.axle(config.pin)))
    if config.robot:   # the inner plate's share of the frame ties between the two sides
        from construction.robot import FrameTies

        groups.append(FrameTies(groups[0]))
    groups += [LinkPlates(), FramePlates()]
    return groups


def side_clearances(ctx: Context, groups: list) -> list[Clearance]:
    """Static clearance check: every link against every group's keep-outs."""
    keepouts = [k for g in groups if hasattr(g, "keepouts") for k in g.keepouts(ctx)]
    return static_clearances(ctx.topo, keepouts, ctx.params.link_radius, ctx.params.margin)


def stacked_plan(tmpl, config: BuildConfig, problem: StackProblem) -> tuple[StackPlan | None, str]:
    """A multi-leg side as copies of a smaller module's plan stacked up the frame.

    When every link of one leg sweeps across another leg's pins, the legs
    must sit in disjoint blocks of layers, which the budgeted search is poor
    at finding. Copies of the plan of the first two legs (or of one), each
    ``k`` layers above the last, are checked by the same claims and
    :func:`verify_plan`: accepted only if nothing collides.
    """
    import linkage

    legs = linkage.module_legs(config.module, config.linkage)
    phases = tmpl.meta.get("phases") or tuple(ph for _, ph in legs)
    blocks = []
    pair = linkage.get(config.linkage).leg_modules.get("double")
    if len(legs) >= 4 and len(legs) % 2 == 0 and pair and [o for o, _ in pair] == [
            o for o, _ in legs[:2]]:
        blocks.append((replace(config, module="double", phases=tuple(phases[:2]), robot=False), 2))
    if len(legs) >= 2:
        blocks.append((replace(config, module="single", phases=(phases[0],), robot=False), 1))
    why = "no smaller module to stack"
    for sub, size in blocks:
        try:
            base = design_side(template_for(sub), sub).plan
        except ValueError as e:
            why = f"its {sub.module} module has no plan either ({str(e).splitlines()[0]})"
            continue
        n = len(legs) // size
        for k in range(1, problem.spec.max_top):
            top = base.top + (n - 1) * k
            if top > problem.spec.max_top:
                break
            layers = {}
            for name in problem.links:
                cls, leg = re.fullmatch(r"(b\d+)_leg(\d+)", name).groups()
                block, j = divmod(int(leg), size)
                layers[name] = base.layers[cls if size == 1 else f"{cls}_leg{j}"] + block * k
            try:
                plan = problem.plan(layers, top)
            except ValueError as e:
                why = str(e)
                continue
            bad = verify_plan(plan)
            if not bad:
                return plan, f"stacked {n} copies of the {sub.module} plan, {k} layers apart"
            why = bad[0]
        why = f"stacking {sub.module} plans at every spacing collides (last: {why})"
    return None, why


def side_problem(tmpl, config: BuildConfig) -> tuple[Context, list, StackProblem]:
    """One side's groups (interfaces resolved) and the layer problem their claims pose."""
    topo = topology_from_template(tmpl)
    ctx = Context(topo=topo, params=config.params, pitch=sheet_thickness(config),
                  servo=servos.get(config.servo), config=config)
    groups = side_groups(ctx, config)
    for g in groups:
        if hasattr(g, "interface"):
            ctx.interfaces[g.name] = g.interface(ctx)
    claims = [c for g in groups for c in g.claims(ctx)]
    spec = StackSpec(pitch=ctx.pitch, margin=config.params.margin)
    return ctx, groups, StackProblem(topo, claims, spec)


_DESIGNS: dict[tuple, SideDesign] = {}
_LAYOUTS: dict[tuple, tuple[dict[str, int], int]] = {}


def design_side(tmpl, config: BuildConfig | None = None) -> SideDesign:
    """Rationalize and plan one side (cached per template and config)."""
    config = config or BuildConfig()
    meta = tuple(sorted((k, v) for k, v in tmpl.meta.items()))
    key = (tmpl.name, tuple(b.name for b in tmpl.bodies), tuple(tmpl.connections), meta, config)
    if key not in _DESIGNS:
        ctx, groups, problem = side_problem(tmpl, config)
        # The robot's side has the same layout as the side on its own; reuse
        # a solved layout when every claim still clears (checked, not assumed).
        layout_key = key[:4] + (replace(config, robot=False),)
        plan = None
        if layout_key in _LAYOUTS:
            layers, top = _LAYOUTS[layout_key]
            try:
                plan = problem.plan(layers, top)
            except ValueError:
                plan = None
            if plan is not None and verify_plan(plan):
                plan = None
        clearances = side_clearances(ctx, groups)
        if no := impossible(ctx.topo, clearances, config.params.link_radius, config.params.margin):
            raise ClearanceError(f"{tmpl.name}: " + "\n  ".join(no))
        if plan is None:
            try:
                plan = problem.solve()
            except PlanError as e:
                plan, how = stacked_plan(tmpl, config, problem)
                if plan is None:
                    said = [*e.blockers, how]
                    involved = [c.describe() for c in clearances
                                if any(c.link in b and c.keepout.owner in b for b in said)]
                    raise e.with_notes(f"stacking a smaller module's plan: {how}",
                                       *(["static clearances behind it:", *involved[:8]]
                                         if involved else [])) from None
            _LAYOUTS[layout_key] = (dict(plan.layers), plan.top)
        _DESIGNS[key] = SideDesign(config, ctx, groups, plan, clearances)
    return _DESIGNS[key]


def plan_for(tmpl, config: BuildConfig | None = None) -> StackPlan:
    """The layer plan of one side."""
    return design_side(tmpl, config).plan


def fabricate_side(design: SideDesign, mech: Mechanism) -> Mechanism:
    """Build every part of one side for ``mech`` (the side's template frozen at some ``t``)."""
    build = Build(design.ctx, design.plan, mech)
    done = Realized()
    for g in design.groups:
        if isinstance(g, (LinkPlates, FramePlates)):
            done.merge(g.realize(build, done))
        else:
            done.merge(g.realize(build))
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
        sheet=cfg.sheet, sheet_name=_sheet_name(cfg), pitch=design.ctx.pitch,
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
    side = fabricate_side(design, tmpl.freeze_at(t))
    if not config.robot:
        return side
    from construction.robot import assemble_robot

    return assemble_robot(side, design)


def _sheet_name(config: BuildConfig) -> str:
    from hardware.catalog import get

    try:
        return get(config.sheet).name
    except KeyError:
        return config.sheet


def adhesive(config: BuildConfig) -> str:
    return "wood_glue" if "plywood" in config.sheet else "acrylic_cement"


__all__ = [
    "MODULES", "BuildConfig", "SideDesign", "design_side", "fabricate", "fabricate_side",
    "plan_for", "sheet_thickness", "template_for",
]

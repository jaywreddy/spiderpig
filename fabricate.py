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

from dataclasses import dataclass, field, replace

import construction
import servos
from construction.base import Build, Context, Params, Realized
from construction.plates import FramePlates, LinkPlates
from mechanism import Mechanism
from servos.mount import DriveGroup
from stack import StackPlan, StackProblem, StackSpec, topology_from_template, verify_plan

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
    """One side, rationalized: its groups (in build order) and its layer plan."""

    config: BuildConfig
    ctx: Context
    groups: list
    plan: StackPlan

    @property
    def drive(self) -> DriveGroup:
        return self.groups[0]


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


_DESIGNS: dict[tuple, SideDesign] = {}
_LAYOUTS: dict[tuple, tuple[dict[str, int], int]] = {}


def design_side(tmpl, config: BuildConfig | None = None) -> SideDesign:
    """Rationalize and plan one side (cached per template and config)."""
    config = config or BuildConfig()
    meta = tuple(sorted((k, v) for k, v in tmpl.meta.items()))
    key = (tmpl.name, tuple(b.name for b in tmpl.bodies), tuple(tmpl.connections), meta, config)
    if key not in _DESIGNS:
        topo = topology_from_template(tmpl)
        ctx = Context(topo=topo, params=config.params, pitch=sheet_thickness(config),
                      servo=servos.get(config.servo), config=config)
        groups = side_groups(ctx, config)
        for g in groups:
            if hasattr(g, "interface"):
                ctx.interfaces[g.name] = g.interface(ctx)
        claims = [c for g in groups for c in g.claims(ctx)]
        spec = StackSpec(pitch=ctx.pitch, margin=config.params.margin)
        problem = StackProblem(topo, claims, spec)
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
        if plan is None:
            plan = problem.solve()
            _LAYOUTS[layout_key] = (dict(plan.layers), plan.top)
        _DESIGNS[key] = SideDesign(config, ctx, groups, plan)
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

"""Fabrication: turn a stack plan into parts, hardware and a bill of materials.

:func:`fabricate` takes a mechanism frozen at a crank angle ``t``, the
:class:`stack.StackPlan` for its assembly and a :class:`BuildConfig`, and
returns a copy whose bodies carry build123d parts in world coordinates:

* **laser-cut** (``fab="laser"``): every leg link in its slot; the frame
  plate (arms out to every fixed pivot plus the servo pad); the crankshaft
  plates (webs, journal plates, cheek webs, the horn adapter and the plate
  its screw heads sink into); spacer rings; the servo's idler bracket;
* **purchased** (``fab="purchased"``, ``bom_key`` set): the servo, bolts,
  nuts, bearings, bushings, dowels, standoffs... as the chosen options model
  them;
* **printed** (``fab="printed"``): only where an option asks for it (the
  printed-pin joinery).

The work is delegated: :mod:`joinery` options turn each pivot site into
holes and hardware, :mod:`servos.mount` places the servo and says what the
frame plate and the adapter plates must cut; this module only walks the plan
and cuts plates. Hardware bodies carry ``rigid_with`` (what they move with).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

import joinery
import servos
from hardware.bom import BomLine
from joinery.base import JoineryParams, Member, PivotSite
from mechanism import Body, Mechanism
from servos.mount import hub as servo_hub
from servos.mount import mount as servo_mount
from shapes import BUFF, Cut, link_plate, plate
from stack import StackPlan, StackSpec, body_class, problem_from_template

SPACER_OD = 8.0          # laser-cut spacer ring outer diameter
ADAPTER_SLOTS = 2        # horn adapter + the crank plate its screw heads sink into
JOURNAL_R = BUFF         # crank plates on the axis O are discs of the link half-width


@dataclass(frozen=True)
class BuildConfig:
    """What to build with. Keys refer to :mod:`joinery`, :mod:`servos` and the catalog."""

    sheet: str = "acrylic_3mm"        # catalog item for the sheet stock (sets the pitch)
    pin: str = joinery.DEFAULTS["pin"]
    frame: str = joinery.DEFAULTS["frame"]
    crankpin: str = joinery.DEFAULTS["crankpin"]
    params: JoineryParams = field(default_factory=JoineryParams)
    servo: str = servos.DEFAULT
    idler_bracket: bool = True
    thickness: float | None = None    # override the sheet's nominal thickness


def sheet_thickness(config: BuildConfig) -> float:
    if config.thickness is not None:
        return config.thickness
    from hardware.catalog import get

    return float(get(config.sheet).dims["thickness"])


def spec_for(config: BuildConfig) -> StackSpec:
    """The stack planner's view of a build: envelopes from joinery, hub from the servo."""
    pitch = sheet_thickness(config)
    env = {kind: joinery.get(getattr(config, kind)).envelope(config.params, pitch)
           for kind in ("pin", "frame", "crankpin")}
    hub = servo_hub(servos.get(config.servo), pitch)
    return StackSpec(
        pitch=pitch, pin=env["pin"], frame=env["frame"], crankpin=env["crankpin"],
        journal_radius=JOURNAL_R, hub_radius=hub.radius, hub_slots=hub.slots,
        adapter_radius=max(hub.adapter_radius, JOURNAL_R), adapter_slots=ADAPTER_SLOTS,
    )


_PLANS: dict[tuple, StackPlan] = {}


def plan_for(tmpl, spec: StackSpec | BuildConfig | None = None) -> StackPlan:
    """Solve (once per process) the stack plan for a template."""
    if isinstance(spec, BuildConfig):
        spec = spec_for(spec)
    spec = spec or StackSpec()
    key = (tmpl.name, tuple(b.name for b in tmpl.bodies), tuple(tmpl.connections), spec)
    if key not in _PLANS:
        _PLANS[key] = problem_from_template(tmpl, spec).solve()
    return _PLANS[key]


def _runs(slots) -> list[list[int]]:
    """Split sorted slots into runs of consecutive integers."""
    runs: list[list[int]] = []
    for s in sorted(slots):
        if runs and s == runs[-1][-1] + 1:
            runs[-1].append(s)
        else:
            runs.append([s])
    return runs


def fabricate(
    mech: Mechanism,
    plan: StackPlan,
    config: BuildConfig | None = None,
    *,
    joinery_on: bool = True,
    joinery: bool | None = None,
) -> Mechanism:
    """Attach parts, hardware and BOM extras for ``plan`` to a copy of ``mech``.

    ``joinery_on=False`` (alias ``joinery=False``) still cuts every hole but
    leaves out the pivot hardware bodies (pins, bolts, bearings, spacers);
    the crankshaft and servo are structure and always included.
    """
    config = config or BuildConfig()
    if joinery is not None:
        joinery_on = joinery
    return _Fabricator(mech, plan, config, joinery_on).run()


class _Fabricator:
    def __init__(self, mech: Mechanism, plan: StackPlan, config: BuildConfig, joinery_on: bool):
        self.mech = mech
        self.plan = plan
        self.config = config
        self.joinery_on = joinery_on
        self.bodies = {b.name: replace(b) for b in mech.bodies}
        self.hardware: list[Body] = []
        self.extras: list[BomLine] = []
        self.cuts: dict[str, list[Cut]] = {}      # plate body -> holes to cut
        self.frame = next(n for n in self.bodies if body_class(n) == "torso")
        cranks = [n for n in self.bodies if body_class(n).startswith("conn")]
        self.crank_host = cranks[0] if cranks else None
        crank = plan.crank
        self.center = self.pos(*crank.center_joints[0]) if crank else np.zeros(2)

    # -- helpers --------------------------------------------------------------

    def pos(self, body: str, joint: str) -> np.ndarray:
        b = self.bodies[body]
        return (b.pose @ b.joint(joint).pose).matrix[:2, 3]

    def z(self, slot: int) -> tuple[float, float]:
        return self.plan.z(slot)

    def cut(self, body: str, cut: Cut) -> None:
        self.cuts.setdefault(body, []).append(cut)

    # -- crank plate names ------------------------------------------------------

    def crank_plate_names(self) -> dict[int, str]:
        """slot -> body name. The crank host carries the horn adapter (the driven plate)."""
        plan = self.plan
        slots = list(plan.journal_slots)
        if not slots or self.crank_host is None:
            return {}
        top = max(plan.adapter_slots) if plan.adapter_slots else max(slots)
        return {k: (self.crank_host if k == top else f"crank_{k}") for k in slots}

    # -- pivot sites --------------------------------------------------------------

    def sites(self, plate_names: dict[int, str]) -> list[tuple[str, PivotSite]]:
        plan, cfg = self.plan, self.config
        out: list[tuple[str, PivotSite]] = []
        extra: dict[int, dict[int, float]] = {}
        spacer_r = SPACER_OD / 2
        for i, ax in enumerate(plan.axes):
            members = sorted(ax.members, key=lambda n: plan.slots[n])
            slots = {plan.slots[n] for n in members}
            lo, hi = min(slots), max(slots)
            ms = [Member(n, *self.z(plan.slots[n])) for n in members]
            if ax.kind == "frame":
                ms.append(Member(self.frame, *self.z(plan.plate), fixed=True))
                top, host = plan.plate, self.frame
            else:
                top, host = hi, members[0]
            gaps = []
            for k in range(lo + 1, top):
                if k in slots or not cfg.params.spacers:
                    continue
                if plan.disc_fits(i, k, spacer_r, extra):
                    extra.setdefault(i, {})[k] = spacer_r
                    gaps.append(self.z(k))
            out.append((getattr(cfg, ax.kind), PivotSite(
                name=ax.name, kind=ax.kind, xy=tuple(self.pos(*ax.joints[0])),
                members=tuple(ms), host=host, gap_slots=tuple(gaps),
                pitch=plan.spec.pitch, params=cfg.params,
            )))
        crank = plan.crank
        if crank is not None:
            for p, nodes in crank.joints.items():
                riders = [b for b, q in crank.riders.items() if q == p]
                ms = [Member(b, *self.z(plan.slots[b])) for b in riders]
                ms += [Member(plate_names[k], *self.z(k), fixed=True)
                       for k, pins in plan.webs.items() if p in pins]
                ms.sort(key=lambda m: m.z0)
                out.append((cfg.crankpin, PivotSite(
                    name=p, kind="crankpin", xy=tuple(self.pos(*nodes[0])),
                    members=tuple(ms), host=self.crank_host, gap_slots=(),
                    pitch=plan.spec.pitch, params=cfg.params,
                )))
        return out

    # -- the build --------------------------------------------------------------

    def run(self) -> Mechanism:
        plan, cfg = self.plan, self.config
        plate_names = self.crank_plate_names()

        # 1. pivots: holes for every plate, hardware bodies
        for key, site in self.sites(plate_names):
            hw = joinery.get(key).build(site)
            for member, hole in hw.holes.items():
                self.cut(member, Cut(site.xy, hole.d, hole.flat, self._radial(site.xy)))
            if self.joinery_on or site.kind == "crankpin":
                self.hardware.extend(hw.bodies)
                self.extras.extend(hw.extras)

        # 2. servo: frame plate holes/pads, adapter holes, servo bodies
        spec = servos.get(cfg.servo)
        crank = plan.crank
        first_pin = next(iter(crank.joints.values()))[0] if crank else None
        crank_angle = 0.0
        if first_pin is not None:
            d = self.pos(*first_pin) - self.center
            crank_angle = math.atan2(d[1], d[0])
        pivots = [self.pos(*ax.joints[0]) for ax in plan.axes if ax.kind == "frame"]
        away = -sum((p - self.center for p in pivots), np.zeros(2))
        u = away / (np.linalg.norm(away) or 1.0)
        mount = servo_mount(
            spec, center=tuple(self.center), u=tuple(u), plate_top=self.z(plan.plate)[1],
            pitch=plan.spec.pitch, crank_angle=crank_angle, frame_host=self.frame,
            crank_host=self.crank_host or self.frame, idler_bracket=cfg.idler_bracket,
        )
        self.hardware.extend(mount.bodies)
        self.extras.extend(mount.extras)
        for xy, d in mount.plate_holes:
            self.cut(self.frame, Cut(tuple(xy), d))
        if plan.adapter_slots and plate_names:
            adapter = plate_names[max(plan.adapter_slots)]
            recess = plate_names.get(max(plan.adapter_slots) - 1)
            for xy, d in mount.adapter_holes:
                self.cut(adapter, Cut(tuple(xy), d))
            for xy, d in mount.recess_holes:
                if recess is not None:
                    self.cut(recess, Cut(tuple(xy), d))

        # 3. plates
        self.cut_links()
        self.cut_frame(pivots, mount.plate_pads)
        self.cut_crank(plate_names)
        for name, b in self.bodies.items():
            if body_class(name) == "coupler" or (
                body_class(name).startswith("conn") and name != self.crank_host
            ) or (body_class(name) == "torso" and name != self.frame):
                b.part = None   # the horn + adapter replace the old coupler
        meta = dict(self.mech.meta)
        meta.update(
            sheet=cfg.sheet, sheet_name=_sheet_name(cfg), pitch=plan.spec.pitch,
            servo=cfg.servo, pin=cfg.pin, frame=cfg.frame, crankpin=cfg.crankpin,
        )
        if any(len(r) > 1 for r in _runs(plate_names)):
            self.extras.append(BomLine(_adhesive(cfg), 1, "crankshaft plate stacks"))
        return Mechanism(
            name=self.mech.name,
            bodies=list(self.bodies.values()) + self.hardware,
            connections=list(self.mech.connections),
            meta=meta,
            bom_extras=list(self.mech.bom_extras) + self.extras,
        )

    def _radial(self, xy) -> float:
        d = np.asarray(xy, float) - self.center
        return math.atan2(d[1], d[0])

    def cut_links(self) -> None:
        for name, slot in self.plan.slots.items():
            b = self.bodies[name]
            segs = [(self.pos(name, p), self.pos(name, q)) for p, q in b.outline]
            b.part = link_plate(segs, *self.z(slot), holes=self.cuts.get(name, []))
            b.fab = "laser"

    def cut_frame(self, pivots, pads) -> None:
        pills = [(self.center, p, BUFF) for p in pivots]
        pills += [(p, q, r) for p, q, r in pads]
        body = self.bodies[self.frame]
        body.part = plate(pills, *self.z(self.plan.plate), self.cuts.get(self.frame, []))
        body.fab = "laser"

    def cut_crank(self, plate_names: dict[int, str]) -> None:
        plan, crank = self.plan, self.plan.crank
        if crank is None:
            return
        pin_xy = {p: self.pos(*nodes[0]) for p, nodes in crank.joints.items()}
        center_hole = Cut(tuple(self.center), self.config.params.axle_d
                          + self.config.params.running_clearance)
        for k, name in plate_names.items():
            pills = [(self.center, pin_xy[p], BUFF) for p in plan.webs.get(k, ())]
            cuts = self.cuts.get(name, [])
            if not any(np.allclose(c.xy, self.center) for c in cuts):
                cuts = [*cuts, center_hole]
            part = plate(pills, *self.z(k), cuts, discs=[(self.center, plan.crank_radius(k))])
            if name in self.bodies:
                self.bodies[name].part = part
                self.bodies[name].fab = "laser"
            else:
                self.hardware.append(Body(name=name, part=part, color="orange",
                                          rigid_with=self.crank_host, fab="laser"))


def _sheet_name(config: BuildConfig) -> str:
    from hardware.catalog import get

    try:
        return get(config.sheet).name
    except KeyError:
        return config.sheet


def _adhesive(config: BuildConfig) -> str:
    return "wood_glue" if "plywood" in config.sheet else "acrylic_cement"


__all__ = ["BuildConfig", "fabricate", "plan_for", "spec_for"]

"""How precisely a design places its output: the error budget of its joints and links.

Precision is kept in two parts, as :mod:`construction.joints` keeps the error terms:

* **repeatability**: what moves the output when the same move is repeated. The joints'
  lost motion: every link's clearance on its axle (:attr:`ErrorTerms.radial_play_mm`; a
  link bonded to its axle, and a frame plate the pillar is screwed into, have none), and
  each input's backlash and resolution (its transmission's; none until a design has
  transmissions);
* **accuracy**: what is the same every time and calibrates out. Every hole off its
  drawing (a laser-cut hole's position tolerance, in each body independently: a
  parallelogram whose opposite sides differ stops keeping its plate level), the frame's
  growth with temperature (the link sheet's expansion), a resolution step.

**Joint shifts to the output.** :mod:`linkage.constraints` models every pin as an axle and
each body on it as one interface; with every input held, the linearised rigid-body
constraints say how far the output moves per unit shift at each interface (a 2 x 2 gain
per interface, at every pose sampled). A clearance ``c`` is a shift of up to ``c / 2``;
the worst case of independent shifts adds each gain's norm times its shift, the root sum
square squares them (:func:`play_map`). A pose where the system is singular (a
parallelogram flat, an input not declared as a drive) makes the error unbounded: it is
reported, never skipped.

**Levers.** ``∂F/∂q_i`` (mm per radian) is how far the output moves per radian of input
``i``: the resolution an input needs to place the output within a target is the target
over that lever (:attr:`Precision.need_deg`). ``∂F/∂p`` per length parameter (per mm: a
parameter in drawing units is divided by the linkage's ``unit``) says what each
proportion's error does, all its parts together.

Every number is the worst over the arm's inputs' box (:meth:`Linkage.box`), or over the
crank's turn (a walker's first foot, a mechanism's output).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from spiderpig import linkage
from spiderpig.config import BuildConfig, add_build_args, add_design_args, config_from_args
from spiderpig.linkage.constraints import gains, layout, sample_inputs

DEFAULT_TARGET_MM = 0.1     # the placement the report asks each input's resolution for
DEFAULT_DT_K = 10.0         # the temperature swing the thermal row assumes (K)
DEFAULT_HOLE_TOL_MM = 0.13  # a laser-cut hole's position tolerance (SendCutSend, about)
DEFAULT_PAYLOAD_G = 50.0    # the load at the output the sag row assumes (a few glass slides)
G = 9.81

Interface = tuple[str, str]     # (joint, body)


@dataclass(frozen=True)
class PlayMap:
    """What the interfaces' shifts do to the output over the box. ``gain`` interface -> its
    worst ``||∂F/∂e||``; ``clearance_mm`` interface -> its diametral clearance;
    ``worst_mm`` / ``rss_mm`` the worst pose's sum / root sum square of every half
    clearance times its gain; ``at_worst`` joint -> its interfaces' share at that pose (the
    rows that add up to ``worst_mm``); ``accuracy_mm`` the same with every hole off by the
    hole tolerance; ``conditioning`` the worst condition number; ``singular`` the poses
    where the system is singular (then the errors are infinite)."""

    gain: dict[Interface, float]
    clearance_mm: dict[Interface, float]
    worst_mm: float
    rss_mm: float
    at_worst: dict[str, float]
    accuracy_mm: float
    conditioning: float
    singular: int = 0

    @property
    def per_joint(self) -> dict[str, float]:
        """Each joint's worst gain over its interfaces."""
        out: dict[str, float] = {}
        for (j, _), g in self.gain.items():
            out[j] = max(out.get(j, 0.0), g)
        return out


@dataclass(frozen=True)
class InputBudget:
    """One input's drive at the output: its transmission's and motor's terms
    (:meth:`construction.transmissions.DriveGroup.error_terms`) through the input's lever:
    backlash (repeatability), resolution, and the sag a payload's moment on the lever
    makes against the drive's stiffness."""

    name: str
    ratio: float
    backlash_deg: float
    resolution_deg: float
    stiffness_nm_per_rad: float
    backlash_mm: float
    resolution_mm: float
    sag_mm: float
    source: str = ""
    accuracy_mm: float = 0.0       # its systematic error (a microstep off its place)
    capacity_nm: float = 0.0       # what it holds at the joint (holding torque x ratio)
    load_nm: float = 0.0           # the payload's moment on its lever (an upper bound)


@dataclass(frozen=True)
class Precision:
    """A design's error budget at its output (module docstring), worst over the box."""

    linkage: str
    output: str
    target_mm: float
    lever_mm_per_rad: dict[str, float]
    need_deg: dict[str, float]
    plays: PlayMap
    tolerance_mm_per_mm: dict[str, float]
    hole_tol_mm: float
    reach_mm: float
    cte_um_per_m_k: float
    thermal_mm: float
    dt_k: float
    notes: list[str] = field(default_factory=list)
    inputs: dict[str, InputBudget] = field(default_factory=dict)
    payload_g: float = DEFAULT_PAYLOAD_G

    @property
    def repeatability_mm(self) -> float:
        """The joints' lost motion at the output and the drives' backlash through their
        levers (worst case: every play the same way)."""
        return self.plays.worst_mm + sum(b.backlash_mm for b in self.inputs.values())

    @property
    def accuracy_links_mm(self) -> float:
        return self.plays.accuracy_mm

    def as_dict(self) -> dict:
        pm = self.plays
        return {"linkage": self.linkage, "output": self.output, "target_mm": self.target_mm,
                "repeatability_mm": _num(self.repeatability_mm),
                "repeatability_rss_mm": _num(pm.rss_mm),
                "at_worst_mm": _rounded(pm.at_worst),
                "lever_mm_per_rad": _rounded(self.lever_mm_per_rad),
                "need_deg": _rounded(self.need_deg, 5),
                "joint_gain": _rounded(pm.per_joint),
                "clearance_mm": {f"{j}/{b}": round(c, 4) for (j, b), c in
                                 pm.clearance_mm.items()},
                "conditioning": _num(pm.conditioning, 1), "singular_poses": pm.singular,
                "tolerance_mm_per_mm": _rounded(self.tolerance_mm_per_mm),
                "accuracy_links_mm": _num(self.accuracy_links_mm),
                "hole_tol_mm": self.hole_tol_mm, "reach_mm": round(self.reach_mm, 2),
                "thermal_mm": round(self.thermal_mm, 4), "dt_k": self.dt_k,
                "payload_g": self.payload_g,
                "inputs": {k: {f: (_num(v) if isinstance(v, float) else v)
                               for f, v in b.__dict__.items()} for k, b in self.inputs.items()},
                "notes": list(self.notes)}

    def lines(self) -> list[str]:
        pm = self.plays
        out = [f"precision of {self.linkage}'s {self.output} (worst over the box):",
               f"  repeatability (joint play): {self.repeatability_mm:.3f} mm worst case, "
               f"{pm.rss_mm:.3f} mm root sum square; at the worst pose, by joint:"]
        for j, mm in sorted(pm.at_worst.items(), key=lambda kv: -kv[1]):
            if mm > 0:
                cl = max(c for (jj, _), c in pm.clearance_mm.items() if jj == j)
                out.append(f"    {j:8} {mm:.3f} mm (clearance {cl:.3f} mm, gain up to "
                           f"{pm.per_joint[j]:.2f})")
        out.append(f"  each input's lever, and its resolution for {self.target_mm:g} mm:")
        for k, lev in self.lever_mm_per_rad.items():
            out.append(f"    {k}: {lev:.1f} mm/rad -> {self.need_deg[k]:.4f} deg "
                       f"({self.need_deg[k] * 60:.2f} arcmin)")
            b = self.inputs.get(k)
            if b is not None:
                out.append(f"      its drive ({b.source}): {b.ratio:.3g}:1, backlash "
                           f"{b.backlash_mm:.3f} mm, a step {b.resolution_mm:.4f} mm, "
                           f"systematic {b.accuracy_mm:.3f} mm, {self.payload_g:g} g sags it "
                           f"{b.sag_mm:.3f} mm; holds {b.capacity_nm:.2f} N·m for "
                           f"{b.load_nm:.3f}")
        out.append(f"  accuracy, every hole off by {self.hole_tol_mm:g} mm: "
                   f"{self.accuracy_links_mm:.3f} mm (calibrates out)")
        out.append(f"  accuracy, {self.dt_k:g} K at {self.cte_um_per_m_k:g} um/m/K over "
                   f"{self.reach_mm:.0f} mm: {self.thermal_mm:.3f} mm")
        out += [f"  note: {n}" for n in self.notes]
        return out


def _num(v: float, nd: int = 4):
    return round(float(v), nd) if math.isfinite(v) else None


def _rounded(d: dict, nd: int = 4) -> dict:
    return {k: _num(v, nd) for k, v in d.items()}


# -- levers and parameters ---------------------------------------------------------------


def _unit(lk: linkage.Linkage, values: tuple[float, ...]) -> float:
    """The linkage's drawing unit in mm (``unit``, else 1)."""
    v = dict(zip(lk.params, values or lk.defaults, strict=True))
    return float(v.get("unit", 1.0))


def levers(lk: linkage.Linkage, values: tuple[float, ...], n: int = 41,
           h: float = 1e-6) -> dict[str, float]:
    """``max ||∂F/∂q_i||`` over the box (mm per radian of each input), by central
    differences of the compiled program."""
    out = layout(lk).output_point
    sol = linkage.LegSolution(1, 0.0, values, lk.key)
    ins = list(sample_inputs(lk, n))
    res = {}
    for i, name in enumerate(lk.inputs):
        up = [a + (h if k == i else 0.0) for k, a in enumerate(ins)]
        dn = [a - (h if k == i else 0.0) for k, a in enumerate(ins)]
        with np.errstate(all="ignore"):
            d = (sol.at(*up)[out] - sol.at(*dn)[out]) / (2 * h)
        res[name] = float(np.nanmax(np.linalg.norm(d, axis=-1)))
    return res


def tolerances(lk: linkage.Linkage, values: tuple[float, ...], n: int = 21,
               h: float = 1e-4) -> dict[str, float]:
    """``max ||∂F/∂p||`` over the box per length parameter, per mm (a parameter in drawing
    units divided by ``unit``; the angles and the unit left out): a proportion's error with
    every part it sets moving together."""
    out = layout(lk).output_point
    base = list(values or lk.defaults)
    unit = _unit(lk, values)
    ins = list(sample_inputs(lk, n))
    res = {}
    for i, name in enumerate(lk.params):
        if name in lk.angles or name == "unit":
            continue
        up, dn = list(base), list(base)
        up[i] += h
        dn[i] -= h
        with np.errstate(all="ignore"):
            a = linkage.LegSolution(1, 0.0, tuple(up), lk.key).at(*ins)[out]
            b = linkage.LegSolution(1, 0.0, tuple(dn), lk.key).at(*ins)[out]
        res[name] = float(np.nanmax(np.linalg.norm((a - b) / (2 * h), axis=-1))) / unit
    return res


# -- the joints ------------------------------------------------------------------------


def play_map(lk: linkage.Linkage, values: tuple[float, ...],
             clearances: dict[Interface, float], n: int = 21,
             hole_tol_mm: float = DEFAULT_HOLE_TOL_MM) -> PlayMap:
    """What every interface's clearance (``clearances``: ``(joint, body)`` -> diametral mm;
    one not named has none) and every hole's tolerance do to the output (module
    docstring)."""
    lay = layout(lk)
    g = gains(lk, values, sample_inputs(lk, n))
    norms = {i: np.where(np.isfinite(m).all(axis=(1, 2)),
                         np.linalg.norm(np.nan_to_num(m), ord=2, axis=(1, 2)), np.inf)
             for i, m in g.per.items()}
    half = {i: clearances.get(i, 0.0) / 2 for i in lay.interfaces}
    zero = np.zeros(len(g.cond))
    total = sum((norms[i] * half[i] for i in lay.interfaces), zero)
    sq = sum(((norms[i] * half[i]) ** 2 for i in lay.interfaces), zero)
    acc = sum((norms[i] for i in lay.interfaces), zero) * hole_tol_mm
    gain = {i: float(np.nanmax(v)) if np.isfinite(v).any() else math.inf
            for i, v in norms.items()}
    if g.singular:
        worst = rss = accuracy = math.inf
        at_worst = dict.fromkeys(lay.axles, math.inf)
    else:
        k = int(np.argmax(total))
        worst, rss, accuracy = float(total[k]), float(np.sqrt(sq).max()), float(np.max(acc))
        at_worst = {j: float(sum(norms[(jj, b)][k] * half[(jj, b)]
                                 for jj, b in lay.interfaces if jj == j)) for j in lay.axles}
    finite = g.cond[np.isfinite(g.cond)]
    cond = float(finite.max()) if g.singular == 0 and finite.size else math.inf
    return PlayMap(gain, {i: clearances.get(i, 0.0) for i in lay.interfaces}, worst, rss,
                   at_worst, accuracy, cond, g.singular)


def joint_clearances(config: BuildConfig, design=None) -> tuple[dict[Interface, float],
                                                                 list[str]]:
    """Every interface's diametral clearance from the design's axle groups' error terms:
    each link on a pin or pillar its construction's radial play, but the link a pin's
    shaft is bonded to (``bonded_host``: its lowest) and the frame plates a pillar is
    screwed into, which have none; and notes on the joints no group builds (a crank's)."""
    from spiderpig.construction.axle import AxleGroup

    if design is None:
        from spiderpig.fabricate import design_side, template_for

        design = design_side(template_for(config), config)
    out: dict[Interface, float] = {}
    notes = []
    covered: set[str] = set()
    for g in design.groups:
        if not isinstance(g, AxleGroup):
            continue
        terms = g.error_terms(design.ctx)
        covered.update(j for _, j in g.axis.joints)
        if terms is None:
            notes.append(f"{g.name}: no error terms (taken as rigid)")
            continue
        host = None
        if getattr(g.construction, "bonded_host", False) and g.axis.members:
            host = min(g.axis.members, key=lambda m: (design.plan.layers.get(m, 0), m))
        for body, j in g.axis.joints:
            rigid = body in (host, "torso")
            out[(j, body)] = 0.0 if rigid else terms.radial_play_mm
    lay = layout(config.lk)
    missing = sorted({j for j in lay.axles if j not in covered})
    if missing:
        notes.append(f"joints with no pin construction's error terms (taken as rigid, so "
                     f"the repeatability is understated): {', '.join(missing)} (the crank's "
                     "and its crankpins')")
    return out, notes


def precision(config: BuildConfig, design=None, *, target_mm: float = DEFAULT_TARGET_MM,
              dt_k: float = DEFAULT_DT_K, hole_tol_mm: float = DEFAULT_HOLE_TOL_MM,
              n: int = 21, payload_g: float = DEFAULT_PAYLOAD_G) -> Precision:
    """The error budget of ``config``'s side (module docstring)."""
    from dataclasses import replace

    from spiderpig.materials import sheet as sheet_record
    from spiderpig.materials import sheet_of

    lk = config.lk
    values = config.values
    if config.module != "single" or config.robot:     # one leg: the linkage's own bodies
        config, design = replace(config, module="single", phases=None, robot=False), None
    clearances, notes = joint_clearances(config, design)
    pm = play_map(lk, values, clearances, n, hole_tol_mm)
    lev = levers(lk, values)
    need = {k: math.degrees(target_mm / v) if v > 0 else math.inf for k, v in lev.items()}
    tol = tolerances(lk, values)
    with np.errstate(all="ignore"):
        pts = linkage.LegSolution(1, 0.0, values, lk.key).at(*sample_inputs(lk, n))
    f = pts[layout(lk).output_point]
    reach = float(np.nanmax(np.linalg.norm(f - pts["O"], axis=-1)))
    cte = sheet_record(sheet_of(config, "link")).cte_um_per_m_k
    thermal = cte * 1e-6 * dt_k * reach
    if pm.singular:
        notes.append(f"the joint system is singular at {pm.singular} of the poses sampled (a "
                     "parallelogram flat, or an input not declared as a drive): the error "
                     "there is unbounded")
    inputs = drive_budgets(config, design, lev, payload_g)
    for b in inputs.values():
        if b.capacity_nm < 2 * b.load_nm:
            notes.append(f"{b.name}'s drive holds {b.capacity_nm:.2f} N·m, under twice the "
                         f"payload's {b.load_nm:.3f} N·m (the arm's own weight not counted)")
    if inputs:
        notes.append("the drives' sag counts the payload only (the links' own weight is "
                     "constant: it calibrates out at a given pose)")
    if lk.kind == "arm" and not inputs:
        notes.append("no transmission: the inputs' own backlash and resolution are not in "
                     "the budget (their levers say what they must be)")
    return Precision(lk.key, layout(lk).output_point, target_mm, lev, need, pm, tol,
                     hole_tol_mm, reach, cte, thermal, dt_k, notes, inputs, payload_g)


def drive_budgets(config: BuildConfig, design, lev: dict[str, float],
                  payload_g: float) -> dict[str, InputBudget]:
    """Each arm input's drive through its lever (:class:`InputBudget`): the design's drive
    groups' error terms; the sag is a payload's moment on the input's lever over the
    drive's stiffness, seen at the output (an upper bound: the whole lever horizontal)."""
    from spiderpig.construction.transmissions import DriveGroup

    if design is None:
        from spiderpig.fabricate import design_side, template_for

        design = design_side(template_for(config), config)
    out = {}
    for g in design.groups:
        if not isinstance(g, DriveGroup):
            continue
        t = g.error_terms(design.ctx)
        lever = lev.get(g.input, 0.0)                    # mm per radian
        torque = payload_g / 1000.0 * G * lever / 1000.0     # N·m
        sag = (torque / t.stiffness_nm_per_rad * lever
               if math.isfinite(t.stiffness_nm_per_rad) and t.stiffness_nm_per_rad > 0
               else 0.0)
        ratio = g.hub(design.ctx).ratio
        out[g.input] = InputBudget(
            g.input, ratio, t.backlash_deg, t.resolution_deg,
            t.stiffness_nm_per_rad, math.radians(t.backlash_deg) * lever,
            math.radians(t.resolution_deg) * lever, sag, t.source,
            math.radians(t.accuracy_deg) * lever, g.motor.holding_nm * ratio, torque)
    return out


def pivot_options(config: BuildConfig) -> list[tuple[str, BuildConfig]]:
    """``config`` with each pivot option: plain holes (``chicago`` / ``standoff``), ball
    pins, ball pins and pillars (``link_radius`` raised to the pillar bearing's seat)."""
    from dataclasses import replace

    from spiderpig import construction

    seat = construction.axle("ball_pillar")
    need = seat.seat.seat_d() / 2 + config.params.min_wall
    wide = replace(config.params, link_radius=max(config.params.link_radius, need))
    return [("chicago pins, standoff pillars", replace(config, pin="chicago", pillar="standoff")),
            ("ball pins, standoff pillars", replace(config, pin="ball", pillar="standoff")),
            (f"ball pins, ball pillars (r {wide.link_radius:g})",
             replace(config, pin="ball", pillar="ball_pillar", params=wide))]


def compare_options(config: BuildConfig) -> list[tuple[str, BuildConfig]]:
    """:func:`pivot_options`, and for an arm each with every transmission."""
    from dataclasses import replace

    from spiderpig import roles as rolemod

    pivots = pivot_options(config)
    if config.lk.kind != "arm":
        return pivots
    out = []
    for label, cfg in pivots:
        for t in rolemod.choices("transmission"):
            roles = tuple(sorted({**dict(cfg.roles), "transmission": t}.items()))
            out.append((f"{label}, {t}", replace(cfg, roles=roles)))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="spiderpig precision",
                                 description="the error budget at a design's output: joint "
                                             "play, the inputs' levers, tolerances, heat")
    add_design_args(ap)
    add_build_args(ap)
    ap.add_argument("--target", type=float, default=DEFAULT_TARGET_MM,
                    help=f"the placement to ask each input's resolution for (mm; default "
                         f"{DEFAULT_TARGET_MM:g})")
    ap.add_argument("--dt", type=float, default=DEFAULT_DT_K,
                    help=f"the temperature swing (K; default {DEFAULT_DT_K:g})")
    ap.add_argument("--payload", type=float, default=DEFAULT_PAYLOAD_G, metavar="G",
                    help=f"the load at the output for the sag row (g; default "
                         f"{DEFAULT_PAYLOAD_G:g})")
    ap.add_argument("--json", action="store_true", help="print the report as JSON")
    ap.add_argument("--compare", action="store_true",
                    help="the same design with each pivot option (plain holes, ball pins, "
                         "ball pins and pillars), and an arm's with each transmission")
    args = ap.parse_args(argv)
    config = config_from_args(args)
    from dataclasses import replace

    if config.robot:
        config = replace(config, robot=False)
    if args.compare:
        print(f"{'pins / pillars / transmission':44} {'repeat mm':>9} {'joints rss':>10} "
              f"{'step mm':>8} {'sag mm':>7}")
        for label, cfg in compare_options(config):
            r = precision(cfg, target_mm=args.target, dt_k=args.dt, payload_g=args.payload)
            step = max((b.resolution_mm for b in r.inputs.values()), default=0.0)
            sag = max((b.sag_mm for b in r.inputs.values()), default=0.0)
            print(f"{label:44} {r.repeatability_mm:9.3f} {r.plays.rss_mm:10.3f} "
                  f"{step:8.4f} {sag:7.3f}")
        return 0
    rep = precision(config, target_mm=args.target, dt_k=args.dt, payload_g=args.payload)
    if args.json:
        import json

        print(json.dumps(rep.as_dict(), indent=1))
    else:
        print("\n".join(rep.lines()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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

**Out of the plane.** Each link tips on its pin by its tilt (the smaller of its bore's free
tilt and what the faces either side allow, :mod:`construction.wobble`) and slides along it
by its column's end float. Each body's small motion out of the plane is a rise and two
tilts; every interface bounds the difference between a body and its axle (the rise at the
joint by half the end float, the tilt in every direction by the link's tilt; a bonded host
and a pillar's frame plates by nothing), so the worst the tool's tip leaves the plane is a
linear programme (:func:`out_of_plane`): the loops restrain what a serial chain would not,
and its duals say which joint's tilt or float holds the answer up. An arm's only (a walker's
foot out of the plane is not a placement).

Every number is the worst over the arm's inputs' box (:meth:`Linkage.box`), or over the
crank's turn (a walker's first foot, a mechanism's output).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from spiderpig import linkage
from spiderpig.actuators import sensors
from spiderpig.config import BuildConfig, add_build_args, add_design_args, config_from_args
from spiderpig.linkage.constraints import gains, layout, sample_inputs
from spiderpig.linkage.stiffness import Compliance, Rod, Section, compliance
from spiderpig.repeatability import Rp, gravity_loads, iso_rp

DEFAULT_TARGET_MM = 0.1     # the placement the report asks each input's resolution for
DEFAULT_DT_K = 10.0         # the temperature swing the thermal row assumes (K)
DEFAULT_HOLE_TOL_MM = 0.13  # a laser-cut hole's position tolerance (SendCutSend, about)
DEFAULT_PAYLOAD_G = 50.0    # the load at the output the sag row assumes (a few glass slides)
SLIDE_G = 5.0               # one glass microscope slide (75 x 25 x 1 mm soda-lime: 4.7 g)
PAIR_TILT_K_PIN = 180e3     # N·mm/rad: a preloaded MR84 pair's moment stiffness, its balls'
#                             Hertz contact at ~30 N (the review's, 180-430: the low one)
PAIR_TILT_K_PILLAR = 800e3  # an MR106 pair's at ~50 N (the review's estimate)
TIP_MASS_KG = 0.1           # what sways at the tool across the plane (tool, payload, links)
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
    homing_mm: float = 0.0         # its homing switch's repeatability (lost on each homing)


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
    tilt_mm: float = 0.0           # a robot's turntable tilting in its bearings' play
    turret_accuracy_mm: float = 0.0   # its drive's eccentricity and its axis out of plumb
    oop: OutOfPlane | None = None  # the tool's tip out of the plane in the links' tilt
    stiffness: Compliance | None = None   # the links bending (linkage.stiffness): an arm's
    rp: Rp | None = None           # ISO 9283 pose repeatability (spiderpig.repeatability)

    @property
    def repeatability_mm(self) -> float:
        """The joints' lost motion at the output, the drives' backlash and homing through
        their levers and the turntable's tilt (worst case: every play the same way), with
        the links' rise out of the plane, at right angles to those, added in quadrature."""
        planar = (self.plays.worst_mm
                  + sum(b.backlash_mm + b.homing_mm for b in self.inputs.values())
                  + self.tilt_mm)
        return math.hypot(planar, self.out_of_plane_mm)     # across the plane: orthogonal

    @property
    def out_of_plane_mm(self) -> float:
        return self.oop.worst_mm if self.oop is not None else 0.0

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
                "payload_g": self.payload_g, "tilt_mm": _num(self.tilt_mm),
                "turret_accuracy_mm": _num(self.turret_accuracy_mm),
                "out_of_plane_mm": _num(self.out_of_plane_mm),
                "iso_rp_session_mm": _num(self.rp.session_mm) if self.rp else None,
                "iso_rp_powerup_mm": _num(self.rp.powerup_mm) if self.rp else None,
                "iso_rp_any_direction_mm": (_num(self.rp.any_direction_mm) if self.rp
                                            else None),
                "payload_sag_mm": (_num(self.stiffness.sag_mm) if self.stiffness else None),
                "across_mm_per_n": (_num(self.stiffness.across_mm_per_n)
                                    if self.stiffness else None),
                "out_of_plane_by_joint": ({j: _rounded(v) for j, v in self.oop.by_joint.items()}
                                          if self.oop is not None else None),
                "inputs": {k: {f: (_num(v) if isinstance(v, float) else v)
                               for f, v in b.__dict__.items()} for k, b in self.inputs.items()},
                "notes": list(self.notes)}

    def lines(self) -> list[str]:
        pm = self.plays
        out = [f"precision of {self.linkage}'s {self.output} (worst over the box):"]
        if self.rp is not None:
            out.append(f"  repeatability, ISO 9283 (30 cycles to a pose, mean + 3 sigma, worst "
                       f"of 5 poses; gravity seats the pins; moves ended one-sidedly, as the "
                       f"host does): {self.rp.session_mm:.3f} mm within a session, "
                       f"{self.rp.powerup_mm:.3f} mm across power-ups; "
                       f"{self.rp.any_direction_mm:.3f} mm from any direction without that"
                       + (f" (re-seating at random: {', '.join(self.rp.unseated)})"
                          if self.rp.unseated else ""))
        out.append(f"  repeatability bound (joint play): {self.repeatability_mm:.3f} mm worst "
                   f"case, {pm.rss_mm:.3f} mm root sum square; at the worst pose, by joint:")
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
                           f"{b.load_nm:.3f}"
                           + (f"; homing {b.homing_mm:.3f} mm" if b.homing_mm else ""))
        if self.oop is not None:
            out.append(f"  out of the plane (the links' tilt and end float, loops counted): "
                       f"{self.oop.worst_mm:.3f} mm worst case; by joint:")
            for j, v in sorted(self.oop.by_joint.items(), key=lambda kv: -sum(kv[1].values())):
                tilt = max((t for (jj, _), t in self.oop.tilt_deg.items() if jj == j),
                           default=0.0)
                out.append(f"    {j:8} {sum(v.values()):.3f} mm (tilt {v['tilt']:.3f}, float "
                           f"{v['float']:.3f}; links tilt up to {tilt:.3f} deg)")
        if self.stiffness is not None:
            c = self.stiffness
            per_g = c.sag_mm / self.payload_g if self.payload_g else 0.0
            k = 1000.0 / c.across_mm_per_n if c.across_mm_per_n > 0 else math.inf  # N/m
            hz = math.sqrt(k / TIP_MASS_KG) / (2 * math.pi)
            out.append(f"  the links bending (a frame of their sections at their layers): "
                       f"{self.payload_g:g} g sags the tool {c.sag_mm:.3f} mm in the plane "
                       f"and {c.sag_across_mm:.3f} across it at worst ({per_g * 1000:.1f} um a "
                       f"gram: a {SLIDE_G:g} g slide picked up moves it "
                       f"{per_g * SLIDE_G:.3f} mm); across the plane {c.across_mm_per_n:.2f} "
                       f"mm/N, swaying at about {hz:.0f} Hz with {TIP_MASS_KG * 1000:g} g at the "
                       "tip (wait it out before a repeatable stop; accuracy at a given load; "
                       "not in the totals)")
        if self.tilt_mm:
            out.append(f"  the turntable's tilt in its bearings: {self.tilt_mm:.3f} mm")
        if self.turret_accuracy_mm:
            out.append(f"  accuracy, the turntable's drive off centre and its axis out of "
                       f"plumb: {self.turret_accuracy_mm:.3f} mm (calibrates out)")
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


@dataclass(frozen=True)
class OutOfPlane:
    """How far the tool's tip leaves the plane in the links' tilt and end float, worst over
    the box (:func:`out_of_plane`): ``worst_mm``; ``by_joint`` joint -> its share at the
    worst pose (``tilt`` and ``float`` rows; they add up to ``worst_mm``); ``tilt_deg``
    interface -> the tilt it was given; ``float_mm`` interface -> its end float."""

    worst_mm: float
    by_joint: dict[str, dict[str, float]]
    tilt_deg: dict[Interface, float]
    float_mm: dict[Interface, float]
    point: tuple[float, float] = (0.0, 0.0)


OOP_DIRECTIONS = 4
"""Directions the tilt bound is taken in (each both ways: an octagon about the circle a
tilt really is, at most 8 % over it)."""


def out_of_plane(lk: linkage.Linkage, values: tuple[float, ...],
                 tilt_deg: dict[Interface, float], float_mm: dict[Interface, float],
                 n: int = 7, drop_mm: float = 0.0,
                 columns: dict[str, tuple[float, dict[Interface, tuple[float, float]]]]
                 | None = None) -> OutOfPlane:
    """The worst rise of the output point (``drop_mm`` below it: the tool's tip) out of the
    plane, over ``n`` poses a side of the box (module docstring). Each interface
    ``(joint, body)`` tilts against its axle by up to ``tilt_deg`` and slides by half
    ``float_mm`` (an interface not named is rigid); or, on a joint in ``columns`` (``joint ->
    (end float g, {interface: (free tilt deg, face radius mm)})``, a column with float),
    its links share the column's float: each tilts by up to its free tilt (its bore's), every
    tilt ``a`` uses ``2 r a`` of the float at its faces, and the column's links slide
    together by what is left, ``2 u + sum 2 r a <= g`` (:mod:`construction.wobble`'s
    supported tilt, the float no longer counted once per link). Unbounded (``inf``) if the
    bodies have a free motion out of the plane."""
    from scipy.optimize import linprog

    lay = layout(lk)
    columns = {j: c for j, c in (columns or {}).items() if c[0] > 0}
    with np.errstate(all="ignore"):
        pts = linkage.LegSolution(1, 0.0, values, lk.key).at(*sample_inputs(lk, n))
    names = [b for b in lay.bodies if b != "torso"] + [f"axle:{j}" for j in lay.axles]
    col = {m: 3 * i for i, m in enumerate(names)}       # (rise, tilt about x, about y)
    shared = [i for j, (_, links) in columns.items() for i in links if i in lay.interfaces]
    s_of = {i: 3 * len(names) + k for k, i in enumerate(shared)}          # each one's tilt
    u_of = {j: 3 * len(names) + len(shared) + k for k, j in enumerate(columns)}  # slide
    nv = 3 * len(names) + len(shared) + len(columns)
    bounds: list[tuple[float | None, float | None]] = [(None, None)] * (3 * len(names))
    bounds += [(0.0, math.radians(columns[i[0]][1][i][0])) for i in shared]
    bounds += [(0.0, None)] * len(columns)
    dirs = [math.pi * k / OOP_DIRECTIONS for k in range(OOP_DIRECTIONS)]
    poses = next(iter(pts.values())).shape[0]
    best, best_rows, at = -1.0, None, (0.0, 0.0)
    for s in range(poses):
        rows: list[np.ndarray] = []
        rhs: list[float] = []
        tags: list[tuple[str, str]] = []

        def add(row, bound, tag, both=True, rows=rows, rhs=rhs, tags=tags):
            for r in ((row, -row) if both else (row,)):
                rows.append(r)
                rhs.append(bound)
                tags.append(tag)

        for j, b in lay.interfaces:
            x, y = (float(v) for v in pts[j][s])
            if not (math.isfinite(x) and math.isfinite(y)):
                break
            rz = np.zeros(nv)
            for who, sign in ((b, 1.0), (f"axle:{j}", -1.0)):
                if who in col:
                    k = col[who]
                    rz[k], rz[k + 1], rz[k + 2] = sign, sign * y, -sign * x
            tilt_rows = []
            for a in dirs:
                rt = np.zeros(nv)
                for who, sign in ((b, 1.0), (f"axle:{j}", -1.0)):
                    if who in col:
                        rt[col[who] + 1] += sign * math.cos(a)
                        rt[col[who] + 2] += sign * math.sin(a)
                tilt_rows.append(rt)
            if (j, b) in s_of:           # shares its column's float
                for r in (rz, -rz):
                    r = r.copy()
                    r[u_of[j]] = -1.0
                    add(r, 0.0, (j, "float"), both=False)
                for rt in tilt_rows:
                    for r in (rt, -rt):
                        r = r.copy()
                        r[s_of[(j, b)]] = -1.0
                        add(r, 0.0, (j, "tilt"), both=False)
            elif j in columns:           # rigid in a column with float (the bonded host)
                add(rz, 0.0, (j, "float"))
                for rt in tilt_rows:
                    add(rt, 0.0, (j, "tilt"))
            else:
                add(rz, float_mm.get((j, b), 0.0) / 2, (j, "float"))
                t = math.radians(tilt_deg.get((j, b), 0.0))
                for rt in tilt_rows:
                    add(rt, t, (j, "tilt"))
        else:
            for j, (g, links) in columns.items():
                r = np.zeros(nv)
                r[u_of[j]] = 2.0
                for i, (_, face_r) in links.items():
                    if i in s_of:
                        r[s_of[i]] = 2.0 * face_r
                add(r, g, (j, "float"), both=False)
            fx, fy = (float(v) for v in pts[lay.output_point][s])
            fy -= drop_mm
            k = col[lay.output_body]
            c = np.zeros(nv)
            c[k], c[k + 1], c[k + 2] = -1.0, -fy, fx           # maximise the rise
            res = linprog(c, A_ub=np.array(rows), b_ub=np.array(rhs), bounds=bounds,
                          method="highs")
            if res.status == 3:
                return OutOfPlane(math.inf, {}, dict(tilt_deg), dict(float_mm))
            if res.status == 0 and -res.fun > best:
                best, at = -res.fun, (fx, fy)
                best_rows = (tags, rhs, np.abs(res.ineqlin.marginals),
                             np.abs(res.upper.marginals))
    by: dict[str, dict[str, float]] = {}
    if best_rows is not None:
        tags, rhs, duals, upper = best_rows
        for (j, kind), d, r in zip(tags, duals, rhs, strict=True):
            if d * r > 0:
                by.setdefault(j, {"tilt": 0.0, "float": 0.0})[kind] += float(d * r)
        for i, k in s_of.items():        # a free tilt's own bound
            hi = bounds[k][1] or 0.0
            if upper[k] * hi > 0:
                by.setdefault(i[0], {"tilt": 0.0, "float": 0.0})["tilt"] += float(upper[k] * hi)
    return OutOfPlane(max(best, 0.0), by, dict(tilt_deg), dict(float_mm), at)


def radial_by_link(config: BuildConfig, wobble: dict) -> dict[Interface, float]:
    """The interfaces whose wobble note gives the link's own radial play (``radial_mm``: a
    ball pair's, a link on one bearing or two)."""
    lay = layout(config.lk)
    out = {}
    for k, note in wobble.items():
        j = k.split(":", 1)[-1]
        for e in note.get("links", []):
            if "radial_mm" in e and (j, e["link"]) in lay.interfaces:
                out[(j, e["link"])] = float(e["radial_mm"])
    return out


def link_tilts(config: BuildConfig, wobble: dict) -> tuple[
        dict[Interface, float], dict[Interface, float],
        dict[str, tuple[float, dict[Interface, tuple[float, float]]]], list[str]]:
    """Each interface's tilt (deg) and end float (mm) from a fabricated side's wobble notes
    (``mech.meta["wobble"]``, keyed ``"<group>:<joint>"``): a link's own tilt, its column's
    float; a body the note doesn't list (a pillar's frame plates) and a link with no tilt
    (the bonded host) rigid. And per joint its column (:func:`out_of_plane`'s ``columns``:
    the float its links share, each link's free tilt and face radius). Notes on the axles
    with no wobble note (taken as rigid)."""
    lay = layout(config.lk)
    tilt: dict[Interface, float] = {}
    flt: dict[Interface, float] = {}
    columns: dict[str, tuple[float, dict[Interface, tuple[float, float]]]] = {}
    notes = []
    by_joint = {k.split(":", 1)[1]: v for k, v in wobble.items() if ":" in k}
    for j in lay.axles:
        note = by_joint.get(j)
        if note is None:
            notes.append(f"out of the plane: {j} has no wobble note (taken as rigid)")
            continue
        links = {}
        for e in note.get("links", []):
            if (j, e["link"]) in lay.interfaces and e.get("tilt_deg", 0.0) > 0:
                tilt[(j, e["link"])] = float(e["tilt_deg"])
                flt[(j, e["link"])] = float(note.get("play_mm", 0.0))
                links[(j, e["link"])] = (float(e.get("free_deg", e["tilt_deg"])),
                                         float(e.get("face_r_mm", 0.0)))
        if links and float(note.get("play_mm", 0.0)) > 0 and all(
                r > 0 for _, r in links.values()):
            columns[j] = (float(note["play_mm"]), links)
    return tilt, flt, columns, notes


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
              n: int = 21, payload_g: float = DEFAULT_PAYLOAD_G,
              wobble: dict | None = None, oop: bool = True) -> Precision:
    """The error budget of ``config``'s side (module docstring). An arm's out-of-plane row
    reads the side's wobble notes (``wobble``: a fabricated side's ``meta["wobble"]``);
    without them and without ``design`` the side is fabricated for them, with ``design``
    and none (or ``oop`` false) the row is left out (said in the notes)."""
    from dataclasses import replace

    from spiderpig.materials import sheet as sheet_record
    from spiderpig.materials import sheet_of

    lk = config.lk
    values = config.values
    turret = config.layout == "turret"
    robot_cfg = config
    if config.module != "single":     # one leg: the linkage's own bodies
        config, design = replace(config.side_config(), module="single", phases=None), None
    elif config.robot:                # (the side's design, given, stays: one module)
        config = config.side_config()
    clearances, notes = joint_clearances(config, design)
    if lk.kind == "arm" and oop and wobble is None and design is None:
        wobble = _wobble(config)
    for (j, body), c in radial_by_link(config, wobble or {}).items():   # a link's own
        if (j, body) in clearances:
            clearances[(j, body)] = c
    pm = play_map(lk, values, clearances, n, hole_tol_mm)
    lev = levers(lk, values)
    need = {k: math.degrees(target_mm / v) if v > 0 else math.inf for k, v in lev.items()}
    tol = tolerances(lk, values)
    with np.errstate(all="ignore"):
        pts = linkage.LegSolution(1, 0.0, values, lk.key).at(*sample_inputs(lk, n))
    f = pts[layout(lk).output_point]
    reach = float(np.nanmax(np.linalg.norm(f - pts["O"], axis=-1)))
    cte = max(sheet_record(sheet_of(config, "link", b)).cte_um_per_m_k   # the worst link's
              for b in lk.links)
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
    tilt = turret_acc = 0.0
    if turret:
        yaw, tilt, turret_acc, yaw_notes = turret_budget(robot_cfg, design, target_mm)
        inputs = {"q1": yaw, **inputs}
        lev = {"q1": _yaw_lever(robot_cfg, design), **lev}
        need = {"q1": math.degrees(target_mm / lev["q1"]) if lev["q1"] > 0 else math.inf,
                **need}
        notes += yaw_notes
    across = None
    if lk.kind == "arm" and not oop:
        notes.append("out of the plane: left out (the links' tilt is not in the budget)")
    elif lk.kind == "arm":
        if wobble is None:
            notes.append("out of the plane: no wobble notes given (the side not fabricated): "
                         "the links' tilt is not in the budget")
        else:
            tilts, floats, columns, oop_notes = link_tilts(config, wobble)
            drop = _tool_drop(robot_cfg, design) if turret else 0.0
            across = out_of_plane(lk, values, tilts, floats, drop_mm=drop, columns=columns)
            notes += oop_notes
            notes.append("out of the plane: the tool's tip as a point below the output (its "
                         "offset across the stack, which turns tilt into a shift in the plane, "
                         "not counted); the turntable's tilt is its own row")
    stiff = None
    if lk.kind == "arm" and oop:
        stiff = link_compliance(config, design, payload_g,
                                _tool_drop(robot_cfg, design) if turret else 0.0)
        notes.append("the links' bending: a 3D frame of each link's section (width "
                     "2 link_radius, its sheet's thickness and modulus) at its layer, the pins "
                     "steel rods between layers, the frame plates and turntable rigid "
                     "(linkage.stiffness); the payload's sag at a given payload calibrates "
                     "out, a change of payload and a load across the plane do not")
    ref = sensors.REFERENCES.get(robot_cfg.role("homing_reference")) if lk.kind == "arm" \
        else None
    if ref is not None and ref.tool_mm is not None:    # every zero from the sheet
        inputs = {k: replace(b, homing_mm=ref.tool_mm) for k, b in inputs.items()}
        notes.append(f"homing: {ref.label} ({ref.source})")
    rp = None
    if lk.kind == "arm" and oop:
        clamped = frozenset(radial_by_link(config, wobble or {}))
        rp = _iso(config, pm, inputs, lev, across, payload_g, clamped,
                  snap_all=ref is not None and ref.tool_mm is not None)
    return Precision(lk.key, layout(lk).output_point, target_mm, lev, need, pm, tol,
                     hole_tol_mm, reach, cte, thermal, dt_k, notes, inputs, payload_g, tilt,
                     turret_acc, across, stiff, rp)


def _iso(config: BuildConfig, pm: PlayMap, inputs: dict[str, InputBudget],
         lev: dict[str, float], across: OutOfPlane | None, payload_g: float,
         clamped: frozenset = frozenset(), snap_all: bool = False) -> Rp:
    """:func:`repeatability.iso_rp` of the budget's terms (module docstring there).
    ``snap_all``: every axis's homing snaps to its driver's phase after the coarse switch
    (with a camera's sighting, even a capstan's: a cord's slip shows up as a full cycle's
    jump the camera sees, :mod:`control.homing`)."""
    from spiderpig.materials import sheet as sheet_record
    from spiderpig.materials import sheet_of

    per_mm = {}                         # g / mm, each link its own sheet's
    for b in config.lk.links:
        rec = sheet_record(sheet_of(config, "link", b))
        per_mm[b] = rec.density * rec.thickness * 2 * config.params.link_radius / 1000
    loads = gravity_loads(config.lk, config.values, link_g_per_mm=per_mm,
                          payload_g=payload_g)
    deg = {k: math.degrees(b.homing_mm / lev[k]) if lev.get(k) else 0.0
           for k, b in inputs.items()}
    nonlin = {k: math.degrees(b.accuracy_mm / lev[k]) if lev.get(k) else 0.0
              for k, b in inputs.items()}
    from spiderpig.repeatability import ACCEL_FACTOR, SLIP_N

    slips = frozenset(i for i in clamped
                      if (loads.force_n.get(i) is not None
                          and float(np.max(loads.force_n[i])) * ACCEL_FACTOR > SLIP_N))
    return iso_rp(config.lk, config.values, pm.clearance_mm, loads,
                  {k: b.backlash_deg for k, b in inputs.items()}, deg,
                  {k: b.resolution_deg for k, b in inputs.items()},
                  {k: snap_all or "capstan" not in b.source for k, b in inputs.items()},
                  across.worst_mm if across is not None else 0.0, lev.get("q1", 0.0),
                  clamped=clamped, slips=slips, nonlin_deg=nonlin)


def link_compliance(config: BuildConfig, design, payload_g: float,
                    drop: float = 0.0) -> Compliance:
    """The links' bending at the tool (:mod:`linkage.stiffness`): each link's section (the
    link sheet's thickness and modulus, ``2 link_radius`` wide; a link cut from another
    sheet, the first such), at its layer's mid-plane in the side's plan, the pins 4 mm and
    the pillars 6 mm steel between the layers and the frame plates."""
    from spiderpig.materials import sheet as sheet_record
    from spiderpig.materials import sheet_of

    d = _side_design(config, design)
    pl = d.plan
    z = {b: (pl.z(k)[0] + pl.z(k)[1]) / 2 for b, k in pl.layers.items()}
    w = 2 * config.params.link_radius
    sections = {}
    for b in config.lk.links:           # each link its own sheet's (link_sheets)
        rec = sheet_record(sheet_of(config, "link", b))
        sections[b] = Section(w, rec.thickness, rec.modulus_gpa * 1000)
    section = next(iter(sections.values()))
    lay = layout(config.lk)
    frame = set(lay.joints["torso"])
    rods = {j: Rod(6.0 if j in frame else 4.0) for j in lay.axles}
    tilt_k = {j: (PAIR_TILT_K_PILLAR if j in frame else PAIR_TILT_K_PIN) for j in lay.axles
              if (config.pillar if j in frame else config.pin).startswith("ball_pair")}
    return compliance(config.lk, config.values, section, payload_n=payload_g / 1000 * G,
                      drop=drop, z=z, rods=rods,
                      frame_z=(pl.z(0)[1], pl.z(pl.top)[0]), tilt_k=tilt_k,
                      sections=sections)


def _wobble(config: BuildConfig) -> dict:
    """The side's wobble notes, from its fabrication at the reference pose (the
    fabrication cache's)."""
    from spiderpig.fabricate import fabricate, template_for

    side = config.side_config() if config.robot else config
    return dict(fabricate(template_for(side), side, 0.0).meta.get("wobble") or {})


def _tool_drop(config: BuildConfig, design) -> float:
    """How far the tool's tip hangs below the output point (mm; 0 without a tool)."""
    from spiderpig.construction.robot import Turret

    _, tool = Turret().plans(_side_design(config, design), config)
    return tool.tool.drop() if tool is not None else 0.0


def _side_design(config: BuildConfig, design):
    if design is not None:
        return design
    from spiderpig.fabricate import design_side, template_for

    side = config.side_config()
    return design_side(template_for(side), side)


def _yaw_lever(config: BuildConfig, design) -> float:
    """How far the output moves per radian of the turntable (mm/rad): its horizontal
    distance from the yaw axis, the most over the box."""
    from spiderpig.construction.robot import Turret

    d = _side_design(config, design)
    tp, _ = Turret().plans(d, config)
    lk = config.lk
    pts = linkage.LegSolution(1, 0.0, config.values, lk.key).at(*lk.box(21))
    f = pts[layout(lk).output_point].reshape(-1, 2)
    return float(np.abs(f[:, 0] - tp.x_a).max())


def turret_budget(config: BuildConfig, design, target_mm: float
                  ) -> tuple[InputBudget, float, float, list[str]]:
    """The turntable's drive at the output (``q1``: its support's, transmission's and
    motor's terms, :meth:`construction.rotary.turret.TurretPlan.error_terms`, and its
    homing switch's repeatability over the flag's radius, through the output's horizontal
    reach), its tilt there (the support's tilt over the farthest the tool's tip gets from
    the bearings' middle), and its systematic error there (the drive's eccentricity over its
    pitch radius, the shaft's tilt in its collars). No sag: gravity has no moment about a
    vertical axis."""
    from spiderpig.construction.robot import Turret
    from spiderpig.construction.rotary import (
        AXIS_RUNOUT_MM,
        ECCENTRICITY_MM,
        SWITCH_REPEAT_MM,
    )
    from spiderpig.hardware.catalog import get

    d = _side_design(config, design)
    tp, tool = Turret().plans(d, config)
    t = tp.error_terms()
    lever = _yaw_lever(config, d)
    lk = config.lk
    pts = linkage.LegSolution(1, 0.0, config.values, lk.key).at(*lk.box(21))
    f = pts[layout(lk).output_point].reshape(-1, 2)
    drop = tool.tool.drop() if tool is not None else 0.0
    mid = tp.y_ground + tp.h["hub_bot"] + (tp.h["hub_top"] - tp.h["hub_bot"]) / 2
    far = float(np.hypot(f[:, 0] - tp.x_a, f[:, 1] - drop - mid).max())
    _, tr, motor, _ = tp.preset.parts
    ratio = tr.ratio(None)
    sensor = tp.preset.parts[3]
    switch = sensor.repeat_mm if sensor is not None else SWITCH_REPEAT_MM
    homing = switch / tp.sensor_r * lever if tp.preset.sensor != "none" else 0.0
    budget = InputBudget("q1", ratio, t.backlash_deg, t.resolution_deg, t.stiffness_nm_per_rad,
                         math.radians(t.backlash_deg) * lever,
                         math.radians(t.resolution_deg) * lever, 0.0, t.source,
                         math.radians(t.accuracy_deg) * lever, motor.holding_nm * ratio, 0.0,
                         homing)
    collar = float(get("collar_8mm").dims["od"])
    systematic = ECCENTRICITY_MM / tp.drive_r() * lever + AXIS_RUNOUT_MM / collar * far
    notes = [f"the turntable ({tp.preset.key}): {n}" for n in t.notes]
    notes.append(f"the turntable's homing: {switch * 1000:g} um at the flag's "
                 f"{tp.sensor_r:.0f} mm radius, {homing:.3f} mm at the output (unverified)")
    notes.append(f"the turntable's drive {ECCENTRICITY_MM:g} mm off centre and its shaft "
                 f"{AXIS_RUNOUT_MM:g} mm out across a collar (estimates): {systematic:.3f} mm "
                 "at the output, once a turn (it calibrates out)")
    notes.append("the turntable's plates' and brackets' stiffness under the arm's moment is "
                 "not modelled")
    return budget, math.radians(t.tilt_deg) * far, systematic, notes


def drive_budgets(config: BuildConfig, design, lev: dict[str, float],
                  payload_g: float) -> dict[str, InputBudget]:
    """Each arm input's drive through its lever (:class:`InputBudget`): the design's drive
    groups' error terms; the sag is a payload's moment on the input's lever over the
    drive's stiffness, seen at the output (an upper bound: the whole lever horizontal); its
    homing, the zero its angle sensor leaves each time (:mod:`actuators.sensors`: a gauge
    by hand, an inclinometer)."""
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
        sensor = sensors.get(config.role("angle_sensor", g.input))   # its zero, each homing
        out[g.input] = InputBudget(
            g.input, ratio, t.backlash_deg, t.resolution_deg,
            t.stiffness_nm_per_rad, math.radians(t.backlash_deg) * lever,
            math.radians(t.resolution_deg) * lever, sag, f"{t.source}; zero: {sensor.key}",
            math.radians(t.accuracy_deg) * lever, g.motor.holding_nm * ratio, torque,
            math.radians(sensor.repeat_deg) * lever)
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
    ap.add_argument("--planar", action="store_true",
                    help="leave the links' tilt out of the plane out (no fabrication: "
                         "seconds, not a minute)")
    ap.add_argument("--json", action="store_true", help="print the report as JSON")
    ap.add_argument("--compare", action="store_true",
                    help="the same design with each pivot option (plain holes, ball pins, "
                         "ball pins and pillars), and an arm's with each transmission")
    args = ap.parse_args(argv)
    config = config_from_args(args)
    from dataclasses import replace

    if config.mirrored:
        config = replace(config, robot=False)
    if args.compare:
        print(f"{'pins / pillars / transmission':44} {'repeat mm':>9} {'joints rss':>10} "
              f"{'step mm':>8} {'sag mm':>7} {'oop mm':>7}")
        for label, cfg in compare_options(config):
            r = precision(cfg, target_mm=args.target, dt_k=args.dt, payload_g=args.payload,
                          oop=not args.planar)
            step = max((b.resolution_mm for b in r.inputs.values()), default=0.0)
            sag = max((b.sag_mm for b in r.inputs.values()), default=0.0)
            print(f"{label:44} {r.repeatability_mm:9.3f} {r.plays.rss_mm:10.3f} "
                  f"{step:8.4f} {sag:7.3f} {r.out_of_plane_mm:7.3f}")
        return 0
    rep = precision(config, target_mm=args.target, dt_k=args.dt, payload_g=args.payload,
                    oop=not args.planar)
    if args.json:
        import json

        print(json.dumps(rep.as_dict(), indent=1))
    else:
        print("\n".join(rep.lines()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

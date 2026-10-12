"""How precisely a design places its output: the error budget of its joints and links.

Precision is kept in two parts, as :mod:`construction.joints` keeps the error terms:

* **repeatability**: what moves the output when the same move is repeated. The joints'
  lost motion: each pin's radial play lets the two links on it shift against each other by
  up to half its diametral clearance (:attr:`ErrorTerms.radial_play_mm`), and each input's
  backlash and resolution (its transmission's, :mod:`construction.joints`; none until a
  design has transmissions);
* **accuracy**: what is the same every time and calibrates out. The links' lengths off
  their drawing (a laser-cut hole's position tolerance), the frame's growth with
  temperature (the link sheet's expansion), and a resolution step.

**Pin play to the output.** The side is a planar linkage of rigid bodies (its links and
the frame) joined at pins. With every input held, a small shift ``e_j`` across pin ``j``
moves the bodies by the linearised constraints (each pin: the two bodies' points at it
move together, but for ``e_j``; each input: its link doesn't turn against the frame), a
square system for a linkage with as many degrees of freedom as inputs, solved at every
pose. ``∂F/∂e_j`` (2 x 2) says how far the output F moves per mm of shift at pin ``j``; the
worst case of independent plays is the sum of its norms times each half clearance
(:func:`play_map`). Near a flattened parallelogram the system goes singular, and the
output's error with it: :attr:`PlayMap.conditioning` says how close the box comes.

**Levers and tolerances.** ``∂F/∂q_i`` (mm per radian) is how far the output moves per
radian of input ``i``: the resolution an input needs to place the output within a target
is the target over that lever (:attr:`Precision.need_deg`). ``∂F/∂p`` for each length
parameter ``p`` is how far it moves per mm the part is off (the tolerance map), and the
frame's growth is the expansion times the output's distance from the shoulder.

Every number is the worst over the arm's inputs' box (an arm's :meth:`Linkage.box`), or
over the crank's turn (a walker's foot, a mechanism's output).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from spiderpig import linkage
from spiderpig.config import BuildConfig, add_build_args, add_design_args, config_from_args

DEFAULT_TARGET_MM = 0.1     # the placement the report asks each input's resolution for
DEFAULT_DT_K = 10.0         # the temperature swing the thermal row assumes (K)
DEFAULT_HOLE_TOL_MM = 0.13  # a laser-cut hole's position tolerance (SendCutSend, about)


@dataclass(frozen=True)
class PlayMap:
    """What each pin's play does to the output over the box: ``per_joint`` joint -> the
    worst ``||∂F/∂e||`` (mm of output per mm of shift across it); ``worst_mm`` the worst
    sum of every pin's half clearance times its gain; ``conditioning`` the worst condition
    number of the constraint system (large: a parallelogram near flat)."""

    per_joint: dict[str, float]
    play_mm: dict[str, float]
    worst_mm: float
    conditioning: float
    rss_mm: float = 0.0             # the plays' root sum square (independent, random)


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
    accuracy_links_mm: float
    reach_mm: float
    cte_um_per_m_k: float
    thermal_mm: float
    dt_k: float
    notes: list[str] = field(default_factory=list)

    @property
    def repeatability_mm(self) -> float:
        """The joints' lost motion at the output (worst case: every play the same way)."""
        return self.plays.worst_mm

    def as_dict(self) -> dict:
        return {"linkage": self.linkage, "output": self.output, "target_mm": self.target_mm,
                "repeatability_mm": round(self.repeatability_mm, 4),
                "repeatability_rss_mm": round(self.plays.rss_mm, 4),
                "lever_mm_per_rad": _rounded(self.lever_mm_per_rad),
                "need_deg": _rounded(self.need_deg, 5),
                "joint_gain": _rounded(self.plays.per_joint),
                "joint_play_mm": _rounded(self.plays.play_mm),
                "conditioning": round(self.plays.conditioning, 1),
                "tolerance_mm_per_mm": _rounded(self.tolerance_mm_per_mm),
                "accuracy_links_mm": round(self.accuracy_links_mm, 4),
                "hole_tol_mm": self.hole_tol_mm, "reach_mm": round(self.reach_mm, 2),
                "thermal_mm": round(self.thermal_mm, 4), "dt_k": self.dt_k,
                "notes": list(self.notes)}

    def lines(self) -> list[str]:
        out = [f"precision of {self.linkage}'s {self.output} (worst over the box):",
               f"  repeatability (joint play): {self.repeatability_mm:.3f} mm worst case, "
               f"{self.plays.rss_mm:.3f} mm root sum square"]
        for j, g in sorted(self.plays.per_joint.items(), key=lambda kv: -kv[1] *
                           self.plays.play_mm.get(kv[0], 0.0)):
            pm = self.plays.play_mm.get(j, 0.0)
            out.append(f"    pin {j:8} play {pm:.3f} mm x gain {g:.2f} = {g * pm / 2:.3f} mm")
        out.append(f"  each input's lever, and its resolution for {self.target_mm:g} mm:")
        for k, lev in self.lever_mm_per_rad.items():
            out.append(f"    {k}: {lev:.1f} mm/rad -> {self.need_deg[k]:.4f} deg "
                       f"({self.need_deg[k] * 60:.2f} arcmin)")
        out.append(f"  accuracy, links off by {self.hole_tol_mm:g} mm each: "
                   f"{self.accuracy_links_mm:.3f} mm (calibrates out)")
        out.append(f"  accuracy, {self.dt_k:g} K at {self.cte_um_per_m_k:g} um/m/K over "
                   f"{self.reach_mm:.0f} mm: {self.thermal_mm:.3f} mm")
        out += [f"  note: {n}" for n in self.notes]
        return out


def _rounded(d: dict, nd: int = 4) -> dict:
    return {k: round(float(v), nd) for k, v in d.items()}


# -- sampling --------------------------------------------------------------------------


def _poses(lk: linkage.Linkage, values: tuple[float, ...], n: int) -> tuple:
    """Every named point at the sampled inputs, and the inputs: an arm's box (``n`` a
    side), else ``4 n`` crank angles."""
    sol = linkage.LegSolution(1, 0.0, values, lk.key)
    if lk.kind == "arm":
        ins = lk.box(n)
    else:
        ins = (np.linspace(0.0, 2 * math.pi, 4 * n, endpoint=False),
               *[np.zeros(4 * n)] * (len(lk.inputs) - 1))
    with np.errstate(all="ignore"):
        return sol.at(*ins), ins


def _output_point(lk: linkage.Linkage) -> str:
    if lk.output is not None:
        return lk.output.point
    return lk.feet[0][1]


def levers(lk: linkage.Linkage, values: tuple[float, ...], n: int = 41,
           h: float = 1e-6) -> dict[str, float]:
    """``max ||∂F/∂q_i||`` over the box (mm per radian of each input), by central
    differences of the compiled program."""
    out = _output_point(lk)
    sol = linkage.LegSolution(1, 0.0, values, lk.key)
    ins = list(lk.box(n)) if lk.kind == "arm" else [
        np.linspace(0.0, 2 * math.pi, 4 * n, endpoint=False),
        *[np.zeros(4 * n)] * (len(lk.inputs) - 1)]
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
    """``max ||∂F/∂p||`` over the box for every length parameter (mm of output per mm the
    part is off; the angles left out)."""
    out = _output_point(lk)
    base = list(values or lk.defaults)
    ins = list(lk.box(n)) if lk.kind == "arm" else [
        np.linspace(0.0, 2 * math.pi, 4 * n, endpoint=False),
        *[np.zeros(4 * n)] * (len(lk.inputs) - 1)]
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
        res[name] = float(np.nanmax(np.linalg.norm((a - b) / (2 * h), axis=-1)))
    return res


# -- pin play -------------------------------------------------------------------------


def _bodies(lk: linkage.Linkage) -> tuple[list[str], dict[str, tuple[str, ...]]]:
    """The rigid bodies (the frame first) and their joints."""
    bodies = {"torso": tuple(lk.frame), **{b: js for b, (js, _) in lk.links.items()}}
    if lk.crank:
        bodies["conn"] = tuple(lk.crank)
    return list(bodies), bodies


def _driven(lk: linkage.Linkage) -> list[str]:
    """The bodies an input turns against the frame."""
    if lk.kind == "arm":
        return [lk.drives[k][0] for k in lk.inputs]
    return ["conn"]


def play_map(lk: linkage.Linkage, values: tuple[float, ...], plays: dict[str, float],
             n: int = 21) -> PlayMap:
    """What each pin's play does to the output (module docstring): ``plays`` joint ->
    diametral clearance (mm; a joint not named has none)."""
    names, joints = _bodies(lk)
    idx = {b: i for i, b in enumerate(names)}
    every = dict.fromkeys(j for js in joints.values() for j in js)
    on = {j: [b for b in names if j in joints[b]] for j in every}
    pins = sorted(j for j, bs in on.items() if len(bs) > 1)    # (a lone point: no pin)
    # a joint on k bodies is k - 1 pins, chained in body order (as the template joins them)
    pairs = [(j, a, b) for j in pins for a, b in zip(on[j], on[j][1:], strict=False)]
    pts, _ = _poses(lk, values, n)
    out_pt = _output_point(lk)
    out_body = (lk.output.link if lk.output is not None else lk.feet[0][0])
    driven = _driven(lk)
    m = len(names)
    nposes = next(iter(pts.values())).shape[0]
    gains = dict.fromkeys(pins, 0.0)
    worst, rss, cond = 0.0, 0.0, 1.0
    scale = max(float(np.nanmax(np.abs(np.concatenate([v.reshape(-1) for v in pts.values()])))),
                1.0)                    # rotations in mm at the linkage's size: a fair condition
    for s in range(nposes):
        rows = []
        a_mat = np.zeros((2 * len(pairs) + 1 + len(driven), 3 * m))
        for r, (j, a, b) in enumerate(pairs):
            p = pts[j][s]
            for body, sign in ((a, 1.0), (b, -1.0)):
                k = 3 * idx[body]
                # point p on body: (dx, dy) + dtheta x (p - 0): body motion about the origin
                a_mat[2 * r, k] += sign
                a_mat[2 * r, k + 2] += -sign * p[1]
                a_mat[2 * r + 1, k + 1] += sign
                a_mat[2 * r + 1, k + 2] += sign * p[0]
            rows.append((j, 2 * r))
        base = 2 * len(pairs)
        a_mat[base, 3 * idx["torso"] + 2] = 1.0         # the frame: fixed (its turn) ...
        ground = np.zeros((2, 3 * m))
        ground[0, 3 * idx["torso"]] = 1.0                # ... and its place
        ground[1, 3 * idx["torso"] + 1] = 1.0
        for i, body in enumerate(driven):               # each input holds its link's turn
            a_mat[base + 1 + i, 3 * idx[body] + 2] = 1.0
            a_mat[base + 1 + i, 3 * idx["torso"] + 2] = -1.0
        full = np.vstack([a_mat, ground])
        if not np.isfinite(full).all():
            continue
        cols = np.ones(3 * m)
        cols[2::3] = 1.0 / scale
        sv = np.linalg.svd(full * cols, compute_uv=False)
        if sv[-1] <= 1e-12 * sv[0]:
            cond = math.inf
            continue
        cond = max(cond, float(sv[0] / sv[-1]))
        pinv = np.linalg.pinv(full)
        k = 3 * idx[out_body]
        f = pts[out_pt][s]
        # the output point's motion from the body's: rows [dx - f_y dth, dy + f_x dth]
        sel = np.zeros((2, 3 * m))
        sel[0, k], sel[0, k + 2] = 1.0, -f[1]
        sel[1, k + 1], sel[1, k + 2] = 1.0, f[0]
        g = sel @ pinv                                  # output per constraint residual
        total = sq = 0.0
        for j, r in rows:
            gj = float(np.linalg.norm(g[:, r:r + 2], 2))
            gains[j] = max(gains[j], gj)
            total += gj * plays.get(j, 0.0) / 2
            sq += (gj * plays.get(j, 0.0) / 2) ** 2
        worst = max(worst, total)
        rss = max(rss, math.sqrt(sq))
    return PlayMap(per_joint=gains, play_mm={j: plays.get(j, 0.0) for j in pins},
                   worst_mm=worst, conditioning=cond, rss_mm=rss)


def joint_plays(config: BuildConfig, design=None) -> tuple[dict[str, float], list[str]]:
    """Each joint's diametral play from the design's axle groups' error terms (the plan's
    pins and pillars), by the joint names the linkage gives them, and notes on what had
    none."""
    from spiderpig.construction.axle import AxleGroup

    if design is None:
        from spiderpig.fabricate import design_side, template_for

        design = design_side(template_for(config), config)
    plays: dict[str, float] = {}
    notes = []
    for g in design.groups:
        if not isinstance(g, AxleGroup):
            continue
        terms = g.error_terms(design.ctx)
        if terms is None:
            notes.append(f"{g.name}: no error terms (taken as rigid)")
            continue
        for _, j in g.axis.joints:
            plays[j] = max(plays.get(j, 0.0), terms.radial_play_mm)
    return plays, notes


def precision(config: BuildConfig, design=None, *, target_mm: float = DEFAULT_TARGET_MM,
              dt_k: float = DEFAULT_DT_K, hole_tol_mm: float = DEFAULT_HOLE_TOL_MM,
              n: int = 21) -> Precision:
    """The error budget of ``config``'s side (module docstring)."""
    from spiderpig.materials import sheet as sheet_record
    from spiderpig.materials import sheet_of

    lk = config.lk
    values = config.values
    plays, notes = joint_plays(config, design)
    pm = play_map(lk, values, plays, n)
    lev = levers(lk, values)
    need = {k: math.degrees(target_mm / v) if v > 0 else math.inf for k, v in lev.items()}
    tol = tolerances(lk, values)
    acc = sum(tol.values()) * hole_tol_mm
    pts, _ = _poses(lk, values, n)
    f = pts[_output_point(lk)]
    o = pts["O"]
    reach = float(np.nanmax(np.linalg.norm(f - o, axis=-1)))
    cte = sheet_record(sheet_of(config, "link")).cte_um_per_m_k
    thermal = cte * 1e-6 * dt_k * reach
    if not math.isfinite(pm.conditioning) or pm.conditioning > 1e4:
        notes.append(f"the joint system is near singular (condition {pm.conditioning:.3g}): "
                     "a parallelogram flattens in the box")
    if lk.kind == "arm":
        notes.append("no transmission yet: the inputs' own backlash and resolution are not "
                     "in the budget (their levers say what they must be)")
    return Precision(lk.key, _output_point(lk), target_mm, lev, need, pm, tol, hole_tol_mm,
                     acc, reach, cte, thermal, dt_k, notes)


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
    ap.add_argument("--json", action="store_true", help="print the report as JSON")
    ap.add_argument("--compare", action="store_true",
                    help="the same design with each pivot option: plain holes, ball pins, "
                         "ball pins and pillars")
    args = ap.parse_args(argv)
    config = config_from_args(args)
    from dataclasses import replace

    if config.robot:
        config = replace(config, robot=False)
    if args.compare:
        print(f"{'pins / pillars':34} {'worst mm':>9} {'rss mm':>8}")
        for label, cfg in pivot_options(config):
            r = precision(cfg, target_mm=args.target, dt_k=args.dt)
            print(f"{label:34} {r.repeatability_mm:9.3f} {r.plays.rss_mm:8.3f}")
        return 0
    rep = precision(config, target_mm=args.target, dt_k=args.dt)
    if args.json:
        import json

        print(json.dumps(rep.as_dict(), indent=1))
    else:
        print("\n".join(rep.lines()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

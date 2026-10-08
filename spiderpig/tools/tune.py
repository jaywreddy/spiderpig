"""Search leg crank phases (and optionally proportions) for a smoother straight walk.

The score is :func:`walk.objective` of :func:`walk.straight_walk_metrics`
(both sides at the same crank angle and rate, one revolution), in
mm-equivalents, lower is better::

    bob_mm + (pitch range + roll range, deg) + 0.5 * slip_rms_mm_per_rev
      + 200 * tipping_fraction + 200 * degenerate_fraction
      + 2 * max(0, -min_margin_mm)
      + 100 * max(0, 0.9 - stride_mm / stride of the default design)

so a candidate that bobs and pitches less and whose planted feet slide less
wins, one that tips, falls back on fewer than three feet or walks less than
90 % of the default stride pays for it.

Search
------
Leg 0 keeps its phase: shifting every leg's phase by the same angle only
moves where the cycle starts. Then

1. a grid over the other legs' phases (``--grid`` degrees), scored on a
   coarse crank grid (``--coarse`` samples per revolution);
2. the ``--top`` best refined by a pattern search (steps 8, 4, 2, 1 deg;
   one leg's phase or two legs' at a time) at the model's full resolution
   (``walk.N_THETA``);
3. with ``--proportions PCT``: the linkage's parameters (all but those
   that only scale the robot, like Klann's ``OA`` or Jansen's ``unit``;
   ``--names`` to choose) join the pattern search, each within +-PCT % of
   its default.

``--linkage`` picks the linkage (Strider by default).

Two legs' phases stay at least ``--min-gap`` degrees apart (default 5)
unless the module's own design has them together (the double's pair):
crankpins that coincide on different cranks leave the layer planner no
layout (the quad at 0,180,180,0 has none within 40 layers; 0,175,180,355
plans like the default).

The model is :mod:`walk`'s without parts: the feet's lateral z from the
default design's layer plan and the nominal centre of mass (the same as
``/api/walk``). ``--plan`` plans the best design (is it buildable?) and
scores it again with its planned foot z; when tuned proportions can't be
planned it falls back on the best phases with the default proportions. (The
planner can take minutes to give up on a design.) Found so far for the
default quad: the phases alone plan like the default; tuning the lower leg
(``--names DF,DE,BE,CD``) plans too, while designs that also move the
crank and frame pivots (``OM``, ``OB``, ``angA``, ...) by as little as 3 %
found no layer plan.

Usage
-----
    spiderpig tune                      # quad, phases only (mise run tune)
    spiderpig tune --module decker --grid 5
    spiderpig tune --linkage jansen --module double --proportions 5
    spiderpig tune --proportions 5 --names DF,DE,BE,CD --plan
    spiderpig tune --proportions 8 --plan --json best.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path

import numpy as np

from spiderpig import linkage as linkage_mod
from spiderpig import walk
from spiderpig.config import BuildConfig, default_module
from spiderpig.linkage import scale_params

PHASE_STEPS = (8.0, 4.0, 2.0, 1.0)          # degrees
MIN_GAP = 5.0                               # degrees between two legs' crank phases
REPORT = ("stride_mm", "bob_mm", "pitch_deg", "roll_deg", "slip_rms_mm_per_rev",
          "min_margin_mm", "tipping_fraction", "degenerate_fraction", "speed_mm_s",
          "yaw_deg_per_rev", "mean_contacts", "walks")


@dataclass(frozen=True)
class Candidate:
    phases: tuple[float, ...]              # degrees, every leg
    proportions: tuple[tuple[str, float], ...] = ()


@dataclass
class Scored:
    candidate: Candidate
    score: float
    metrics: dict | None


def _gap(a: float, b: float) -> float:
    """Angle between two phases (degrees, 0..180)."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


class Tuner:
    """Scores candidates of one module of a linkage (memoized per resolution)."""

    def __init__(self, module: str, stride_ref: float | None = None,
                 min_gap: float = MIN_GAP, linkage: str = linkage_mod.DEFAULT) -> None:
        self.module = module
        self.linkage = linkage_mod.get(linkage)
        self.default = Candidate(tuple(math.degrees(ph) for _, ph in
                                       BuildConfig(linkage=linkage, module=module).legs))
        self.stride_ref = stride_ref
        self.min_gap = min_gap
        self.evaluations = 0
        self.phase_best: Scored | None = None     # best with the default proportions (tune())
        self._memo: dict[tuple, Scored] = {}
        d = self.default.phases
        self._pairs = [(i, j) for i, j in combinations(range(len(d)), 2)
                       if _gap(d[i], d[j]) > 1e-9]        # together by design: exempt
        if self.stride_ref is None:
            m = self.metrics(self.default)
            self.stride_ref = m["stride_mm"] if m else None

    def feasible(self, phases) -> bool:
        """Every two legs' phases ``min_gap`` apart (see the module docstring)."""
        return all(_gap(phases[i], phases[j]) >= self.min_gap - 1e-9 for i, j in self._pairs)

    def config(self, c: Candidate) -> BuildConfig:
        return BuildConfig(linkage=self.linkage.key, module=self.module,
                           phases=tuple(math.radians(p) for p in c.phases),
                           proportions=c.proportions)

    def metrics(self, c: Candidate, n: int = walk.N_THETA, feet_z=None) -> dict | None:
        """Straight-walk metrics of a candidate (``None``: the linkage can't be assembled)."""
        config = self.config(c)
        try:
            legs = walk.side_legs(config, n)
        except walk.LinkageError:
            return None
        self.evaluations += 1
        return walk.straight_walk_metrics(walk.walker(config, legs=legs, feet_z=feet_z))

    def score(self, c: Candidate, n: int = walk.N_THETA) -> Scored:
        key = (tuple(round(p % 360.0, 9) for p in c.phases), c.proportions, n)
        if key not in self._memo:
            m = self.metrics(c, n) if self.feasible(c.phases) else None
            s = math.inf if m is None else walk.objective(m, self.stride_ref)
            self._memo[key] = Scored(c, s, m)
        return self._memo[key]


def grid_search(tuner: Tuner, step: float, n: int, top: int,
                proportions: tuple = ()) -> list[Scored]:
    """Every combination of the free legs' phases at ``step`` degrees; the ``top`` best."""
    base = tuner.default.phases
    free = len(base) - 1
    if free == 0:
        return [tuner.score(Candidate(base, proportions), n)]
    values = np.arange(0.0, 360.0, step)
    grids = np.meshgrid(*([values] * free), indexing="ij")
    combos = np.stack([g.ravel() for g in grids], axis=1)
    out = []
    for i, row in enumerate(combos):
        c = Candidate((base[0], *(float(v) for v in row)), proportions)
        out.append(tuner.score(c, n))
        if (i + 1) % 250 == 0:
            print(f"  grid {i + 1}/{len(combos)}", file=sys.stderr, flush=True)
    out.sort(key=lambda s: s.score)
    return out[:top]


def pattern_search(tuner: Tuner, start: Candidate, names: tuple[str, ...] = (),
                   pct: float = 0.0) -> Scored:
    """Coordinate descent on the free phases (and the named proportions, +-pct %)."""
    defaults = {k: float(v) for k, v in tuner.linkage.params.items()}
    best = tuner.score(start)
    prop_steps = [pct / 2 ** k for k in range(1, 5)] if names and pct > 0 else []
    levels = max(len(PHASE_STEPS), len(prop_steps))
    for level in range(levels):
        dphase = PHASE_STEPS[min(level, len(PHASE_STEPS) - 1)]
        dprop = prop_steps[min(level, len(prop_steps) - 1)] if prop_steps else 0.0
        improved = True
        while improved:
            improved = False
            c = best.candidate
            moves = []
            free = range(1, len(c.phases))
            # one phase at a time, then pairs (the contact pattern couples legs)
            steps = [{i: s} for i in free for s in (-1.0, 1.0)]
            steps += [{i: si, j: sj} for i in free for j in free if i < j
                      for si in (-1.0, 1.0) for sj in (-1.0, 1.0)]
            for step in steps:
                ph = list(c.phases)
                for i, sgn in step.items():
                    ph[i] = (ph[i] + sgn * dphase) % 360.0
                moves.append(Candidate(tuple(ph), c.proportions))
            props = dict(c.proportions)
            for name in names if dprop else ():
                d = defaults[name]
                cur = props.get(name, d)
                for sgn in (-1.0, 1.0):
                    v = cur + sgn * dprop / 100.0 * abs(d)
                    if abs(v - d) > pct / 100.0 * abs(d) + 1e-12:
                        continue
                    trial = Candidate(c.phases, tuple(dict(props, **{name: v}).items()))
                    moves.append(Candidate(c.phases, tuner.config(trial).proportions))
            for m in moves:
                s = tuner.score(m)
                if s.score < best.score - 1e-9:
                    best, improved = s, True
    return best


def _fmt(v) -> str:
    if isinstance(v, list):
        return "[" + ", ".join(f"{x:.2f}" for x in v) + "]"
    if isinstance(v, float):
        return f"{v:.3f}"
    return str(v)


def _phases_arg(phases) -> str:
    return ",".join(f"{p:g}" for p in phases)


def flags(module: str, c: Candidate, linkage_key: str = linkage_mod.DEFAULT) -> dict[str, str]:
    """How to use a candidate: `spiderpig build` / `bake` flags and the viewer's query
    string."""
    cli = f"--module {module} --phases {_phases_arg(c.phases)}"
    cli += "".join(f" --proportion {k}={v:.6g}" for k, v in c.proportions)
    query = f"module={module}&phases={_phases_arg(c.phases)}"
    query += "".join(f"&p.{k}={v:.6g}" for k, v in c.proportions)
    if linkage_key != linkage_mod.DEFAULT:
        cli += f" --linkage {linkage_key}"
        query += f"&linkage={linkage_key}"
    return {"main": f"spiderpig build {cli}",
            "bake": f"spiderpig bake {cli}",
            "query": f"?{query}"}


def tune(module: str, *, grid: float = 30.0, coarse: int = 120, top: int = 6,
         pct: float = 0.0, names: tuple[str, ...] | None = None,
         min_gap: float = MIN_GAP, linkage_key: str = linkage_mod.DEFAULT,
         ) -> tuple[Tuner, Scored, Scored]:
    """``(tuner, default, best)`` for ``module`` (see the module docstring)."""
    tuner = Tuner(module, min_gap=min_gap, linkage=linkage_key)
    default = tuner.score(tuner.default)
    if names is None and pct > 0:
        scale = scale_params(tuner.linkage)
        names = tuple(k for k in tuner.linkage.params if k not in scale)
    names = names or ()
    starts = [s.candidate for s in grid_search(tuner, grid, coarse, top)]
    if tuner.default not in starts:
        starts.append(tuner.default)
    best = tuner.phase_best = default
    for start in starts:
        s = pattern_search(tuner, start)                      # phases first
        if s.score < tuner.phase_best.score:
            tuner.phase_best = s
        if names:
            s = pattern_search(tuner, s.candidate, names, pct)   # then with proportions
        if s.score < best.score:
            best = s
    return tuner, default, best


def plan(tuner: Tuner, c: Candidate) -> dict:
    """Plan a candidate's layers (can it be built?) and score it with its planned foot z."""
    from spiderpig.fabricate import design_side, template_for

    config = tuner.config(c)
    try:
        design = design_side(template_for(config), config)
    except Exception as e:  # noqa: BLE001 - the planner or a construction: report, don't crash
        return {"ok": False, "error": str(e)}
    z = walk.foot_z_planned(config, design)
    m = tuner.metrics(c, feet_z=z)
    assert m is not None  # it planned, so the linkage assembles
    return {"ok": True, "layers": design.plan.top + 1, "foot_z": z,
            "objective": walk.objective(m, tuner.stride_ref), "metrics": m}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    ap.add_argument("--linkage", choices=linkage_mod.available("walker"),
                    default=linkage_mod.DEFAULT)
    ap.add_argument("--module", default=None,
                    help="one of the linkage's modules (default: the linkage's, "
                    "config.default_module)")
    ap.add_argument("--grid", type=float, default=30.0,
                    help="phase grid step in degrees (default 30)")
    ap.add_argument("--coarse", type=int, default=120,
                    help="crank samples per revolution for the grid stage (default 120)")
    ap.add_argument("--top", type=int, default=6, help="grid candidates refined (default 6)")
    ap.add_argument("--proportions", type=float, default=0.0, metavar="PCT",
                    help="also tune the proportions within +-PCT %% of the defaults (default: no)")
    ap.add_argument("--names", default=None,
                    help="comma-separated proportions to tune (default: all but the scale)")
    ap.add_argument("--min-gap", type=float, default=MIN_GAP, metavar="DEG",
                    help=f"least angle between two legs' phases (default {MIN_GAP:g})")
    ap.add_argument("--plan", action="store_true",
                    help="plan the best design's layers (buildable?) and rescore with them")
    ap.add_argument("--json", type=Path, default=None, help="write the result as JSON")
    args = ap.parse_args(argv)
    lk = linkage_mod.get(args.linkage)
    args.module = args.module or default_module(lk.key)
    names = tuple(s.strip() for s in args.names.split(",")) if args.names else None
    if names and (bad := [n for n in names if n not in lk.params]):
        ap.error(f"unknown proportions {bad}; have {list(lk.params)}")
    legs = lk.leg_modules.get(args.module)
    if legs is None:
        ap.error(f"unknown module {args.module!r}; have {list(lk.leg_modules)}")
    if len(legs) == 1 and args.proportions <= 0:
        print(f"{args.module}: one leg per side, so no phases to tune; try --proportions PCT")
        return 0

    t0 = time.perf_counter()
    tuner, default, best = tune(args.module, grid=args.grid, coarse=args.coarse,
                                top=args.top, pct=args.proportions, names=names,
                                min_gap=args.min_gap, linkage_key=lk.key)
    elapsed = time.perf_counter() - t0

    print(f"{lk.key} {args.module}: {tuner.evaluations} evaluations in {elapsed:.1f} s "
          f"(objective: walk.objective, stride reference {tuner.stride_ref:.1f} mm/rev)")
    if not tuner.stride_ref or tuner.stride_ref < 1.0:
        print("  note: the default design doesn't walk in this model (one leg a side can't "
              "stand; with two, the four feet stay coplanar, all on the ground): no stride term")
    for label, scored in (("default", default), ("best", best)):
        if scored.metrics is not None and not scored.metrics.get("walks", True):
            print(f"  WARNING: the {label} design does not walk (stride "
                  f"{scored.metrics['stride_mm']:.1f} mm/rev, on fewer than three feet "
                  f"{scored.metrics['degenerate_fraction'] * 100:.0f} % of the cycle); MuJoCo "
                  "crawls such a design with its body on the floor")
    assert default.metrics is not None  # a registered linkage's default assembles,
    assert best.metrics is not None  # and best scores no worse (no metrics: inf)
    print(f"  {'':22} {'default':>22} {'best':>22}")
    print(f"  {'phases (deg)':22} {_phases_arg(default.candidate.phases):>22} "
          f"{_phases_arg(best.candidate.phases):>22}")
    for k, v in best.candidate.proportions:
        print(f"  {'proportion ' + k:22} {float(lk.params[k]):>22.6g} {v:>22.6g}")
    print(f"  {'objective':22} {default.score:>22.2f} {best.score:>22.2f}")
    for key in REPORT:
        print(f"  {key:22} {_fmt(default.metrics[key]):>22} {_fmt(best.metrics[key]):>22}")

    result = {"linkage": lk.key, "module": args.module,
              "objective_weights": walk.OBJECTIVE_WEIGHTS,
              "stride_ref_mm": tuner.stride_ref,
              "default": {"phases_deg": list(default.candidate.phases),
                          "objective": default.score, "metrics": default.metrics},
              "best": {"phases_deg": list(best.candidate.phases),
                       "proportions": dict(best.candidate.proportions),
                       "objective": best.score, "metrics": best.metrics}}
    chosen = best
    if args.plan:
        p = result["best"]["plan"] = plan(tuner, best.candidate)
        if not p["ok"] and best.candidate.proportions and tuner.phase_best is not best:
            # fall back on the best design with the default proportions
            print(f"  plan: the best design can't be built as is: {p['error']}")
            chosen = tuner.phase_best
            assert chosen is not None  # tune() sets it (to the default at least)
            p = plan(tuner, chosen.candidate)
            result["phases_only"] = {"phases_deg": list(chosen.candidate.phases),
                                     "objective": chosen.score, "metrics": chosen.metrics,
                                     "plan": p}
            print(f"  best with the default proportions: phases "
                  f"{_phases_arg(chosen.candidate.phases)}, objective {chosen.score:.2f}")
        if p["ok"]:
            print(f"  plan: {p['layers']} layers, foot z {[round(v, 1) for v in p['foot_z']]}; "
                  f"objective with the planned z {p['objective']:.2f}")
        else:
            print(f"  plan: can't be built as is: {p['error']}")

    use = flags(args.module, chosen.candidate, lk.key)
    result["use"] = use
    print("use it:")
    print(f"  {use['main']}")
    print(f"  {use['bake']}")
    print(f"  viewer / /api/walk / /api/glb/robot query: {use['query']}")
    if args.json:
        args.json.write_text(json.dumps(walk.jsonable(result), indent=2))
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

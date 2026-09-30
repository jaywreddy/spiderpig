"""Compare every registered linkage: foot paths, layer plans, walking, cost.

    uv run python cli.py report --out build/linkages.json

For each linkage (``linkage.available()``): its parameters and bodies, the
single leg's foot path over one crank revolution (lift; stride, stance
fraction and speed ripple within 2 and 5 mm of the ground), which leg modules the layer
planner can lay out (how thick the side is, whether that is proven the thinnest, the
crank's route, and ``ground_clearance_mm``: how high the body rides over the feet), and,
for the modules that plan, the quasi-static walking metrics (:mod:`walk`) and the robot's
BOM cost. Writes one JSON document; ``--modules`` / ``--linkages`` narrow it.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import time
from pathlib import Path

import numpy as np

import linkage

log = logging.getLogger("linkage_report")
N = 720
MIN_STRIDE = 20.0     # mm per revolution: less isn't walking
STANCE_TOL = (2.0, 5.0)   # mm above the lowest point that still count as stance


def foot_path(lk: linkage.Linkage, n: int = N) -> dict:
    """Leg 0's first foot over one revolution, and what makes it a good foot path."""
    ts = 2.0 * math.pi * np.arange(n) / n
    pts = lk.solve().evaluate(ts)
    _, joint = lk.feet[0]
    f = pts[joint]
    y0 = f[:, 1].min()
    vx = (np.roll(f[:, 0], -1) - np.roll(f[:, 0], 1)) * n / (4 * math.pi)   # mm/rad
    crank_r = float(np.linalg.norm(pts[lk.crank[1]][0]))
    top = max(float(pts[j][:, 1].max()) for j in lk.points)
    out = {
        "xy": np.round(f[:: n // 180], 2).tolist(),
        "crank_radius_mm": round(crank_r, 2),
        "height_mm": round(top - float(y0), 1),              # highest joint to the ground
        "width_mm": round(float(np.ptp(np.concatenate([pts[j][:, 0] for j in lk.points]))), 1),
        "lift_mm": round(float(np.ptp(f[:, 1])), 1),
    }
    for tol in STANCE_TOL:
        stance = f[:, 1] <= y0 + tol
        sv = vx[stance]
        out[f"stance_{tol:g}mm"] = {
            "stride_mm": round(float(np.ptp(f[stance, 0])), 1),
            "fraction": round(float(stance.mean()), 3),
            "speed_ripple": round(float(sv.std() / max(abs(sv.mean()), 1e-9)), 3),
        }
    return out


def plans(key: str, modules) -> dict:
    from config import BuildConfig
    from fabricate import design_side, template_for

    out = {}
    for m in modules:
        cfg = BuildConfig(linkage=key, module=m, robot=False)
        t0 = time.perf_counter()
        try:
            d = design_side(template_for(cfg), cfg)
            gc = d.ground_clearance_mm
            out[m] = {"ok": True, "layers": d.plan.top + 1, "stack_mm": d.plan.height,
                      "optimal": d.plan.optimal, "proof": d.plan.proof,
                      "crank_route": repr(d.plan.choices.get("crank")),
                      "ground_clearance_mm": None if gc is None else round(gc, 1)}
        except ValueError as e:     # a stage said why: AssemblyError, ClearanceError, PlanError
            out[m] = {"ok": False, "stage": type(e).__name__, "error": str(e)}
        out[m]["seconds"] = round(time.perf_counter() - t0, 1)
        log.info("%s %s: %s", key, m, out[m])
    return out


def walking(key: str, module: str) -> dict:
    """Quasi-static straight-walk metrics of the robot (both sides) for a module that plans."""
    import walk
    from config import BuildConfig

    p = walk.api_payload(BuildConfig(linkage=key, module=module))
    if not p["valid"]:
        return {"valid": False, "error": p["error"]}
    m = p["metrics"]
    keep = ("stride_mm", "speed_mm_s", "bob_mm", "pitch_deg", "roll_deg", "slip_rms_mm_per_rev",
            "tipping_fraction", "degenerate_fraction", "mean_contacts", "min_margin_mm")
    return {"valid": True, "mass_g": p["mass_g"], "objective": round(walk.objective(m), 2),
            **{k: m[k] for k in keep}}


def cost(key: str, module: str) -> dict:
    """The robot's purchase total and part counts (builds every part: slow)."""
    from config import BuildConfig
    from fabricate import fabricate, template_for
    from hardware.bom import bom_from_mechanism

    cfg = BuildConfig(linkage=key, module=module)
    mech = fabricate(template_for(cfg), cfg, 0.0)
    bom = bom_from_mechanism(mech)
    return {"module": module, "cost_usd": round(bom.cost_usd, 2), "bodies": len(mech.bodies),
            "laser": sum(b.fab == "laser" for b in mech.bodies),
            "printed": sum(b.fab == "printed" for b in mech.bodies)}


def describe(lk: linkage.Linkage) -> dict:
    return {
        "key": lk.key, "name": lk.name, "family": lk.family or lk.key,
        "source": lk.source, "notes": lk.notes,
        "params": {k: float(v) for k, v in lk.params.items()},
        "angles": sorted(lk.angles),
        "links": {b: {"joints": list(js), "label": lk.labels.get(b, "")}
                  for b, (js, _) in lk.links.items()},
        "feet_per_leg": len(lk.feet),
        "closures": [{"point": c.point, "refs": list(c.refs), "margin_mm": round(c.margin_mm, 2),
                      "transmission_deg": [round(a, 1) for a in c.angle_deg],
                      "toggles": c.toggles, "text": c.describe()}
                     for c in lk.check() if c.kind == "closure"],
        "modules": {m: len(legs) for m, legs in lk.leg_modules.items()},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=Path("build/linkages.json"))
    ap.add_argument("--linkages", nargs="*", default=None)
    ap.add_argument("--modules", nargs="*", default=["single", "double", "decker", "quad"])
    ap.add_argument("--no-plan", action="store_true", help="skip the layer planner")
    ap.add_argument("--no-walk", action="store_true", help="skip the walking model")
    ap.add_argument("--cost", action="store_true",
                    help="build the best-walking module's robot for its BOM total (slow)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(name)s %(message)s")
    keys = args.linkages or linkage.available("walker")
    report = []
    for key in keys:
        lk = linkage.get(key)
        row = describe(lk) | {"foot": foot_path(lk)}
        if not args.no_plan:
            row["plans"] = plans(key, args.modules)
            if not args.no_walk:
                row["walk"] = {m: walking(key, m) for m, r in row["plans"].items() if r["ok"]}
                ok = {m: w for m, w in row["walk"].items()      # it has to go somewhere
                      if w["valid"] and w["stride_mm"] > MIN_STRIDE}
                if ok:
                    row["best"] = min(ok, key=lambda m: ok[m]["objective"])
                    if args.cost:
                        row["cost"] = cost(key, row["best"])
        report.append(row)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=1))
    log.info("wrote %s (%d linkages)", args.out, len(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

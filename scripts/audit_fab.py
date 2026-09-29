"""Fabrication audit: can the robot be built, and does it go together?

For each module (single, double, decker, quad), independent of the unit tests:

1. **plan**     — :func:`stack.verify_plan` re-checks one side's layer plan on
   a fresh, denser sampling of the whole crank cycle.
2. **contract** — :func:`construction.contract.check_side`: every part a
   construction builds stays inside the space its group claimed, at several
   crank angles.
3. **solids**   — every part of the fabricated robot is one valid B-rep solid.
4. **clash**    — pairwise OCCT intersection volume between all parts of the
   robot (both sides and the chassis) at a few crank angles. A screw in the
   part it threads into (``mech.meta["fastened"]``) is not a clash.
5. **dxf**      — every laser-cut part packs onto the sheet stock.
6. **bom**      — every purchased item resolves in the catalog.

Writes ``audit.json`` and ``audit.md`` to ``--out`` and exits non-zero when
any check fails.

Usage::

    uv run python scripts/audit_fab.py                       # all modules
    uv run python scripts/audit_fab.py --modules single --out build/audit
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))

from construction.contract import check_side  # noqa: E402
from fabricate import MODULES, BuildConfig, design_side, fabricate  # noqa: E402
from hardware.bom import BomLine, bom_from_mechanism  # noqa: E402
from layout import pack  # noqa: E402
from main import add_config_args, module_template, sheet_size  # noqa: E402
from stack import verify_plan  # noqa: E402

CLASH_MM3 = 1e-3


def _overlap(a, b, eps: float = 1e-6) -> bool:
    return all(max(getattr(a.min, c), getattr(b.min, c)) < min(getattr(a.max, c),
                                                               getattr(b.max, c)) - eps
               for c in "XYZ")


def clashes(mech) -> list[dict]:
    """Pairs of parts that intersect by more than ``CLASH_MM3`` (fastened pairs excepted)."""
    allowed = {frozenset(p) for p in mech.meta.get("fastened", [])}
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    boxes = {n: p.bounding_box() for n, p in parts.items()}
    out = []
    for a, b in itertools.combinations(parts, 2):
        if frozenset((a, b)) in allowed or not _overlap(boxes[a], boxes[b]):
            continue
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > CLASH_MM3:
            out.append({"a": a, "b": b, "mm3": round(vol, 3)})
    return out


def bad_solids(mech) -> list[dict]:
    out = []
    for b in mech.bodies:
        if b.part is None:
            continue
        n = len(b.part.solids())
        if n != 1 or not b.part.is_valid:
            out.append({"part": b.name, "solids": n, "valid": bool(b.part.is_valid)})
    return out


def audit_module(module: str, config: BuildConfig, ts_contract, ts_clash) -> dict:
    t0 = time.time()
    tmpl = module_template(module)
    design = design_side(tmpl, config)
    rep: dict = {"layers": design.plan.top + 1, "stack_mm": design.plan.height,
                 "plan": design.plan.describe()}
    rep["plan_violations"] = verify_plan(design.plan, tmpl)
    rep["contract"] = {f"t={t:g}": check_side(design, tmpl.freeze_at(t)) for t in ts_contract}
    rep["clash"], rep["solids"] = {}, {}
    mech = None
    for t in ts_clash:
        mech = fabricate(tmpl, config, t)
        rep["clash"][f"t={t:g}"] = clashes(mech)
        rep["solids"][f"t={t:g}"] = bad_solids(mech)
    rep["parts"] = sum(1 for b in mech.bodies if b.part is not None)
    rep["chassis"] = {k: v for k, v in mech.meta.items()
                      if k in ("centre_plates", "rear_screw", "rear_screws_per_servo",
                               "rear_engagement_mm", "ties", "tie_screw", "tie_engagement_mm")}
    try:
        sheets = pack(mech, sheet_size(config))
        rep["dxf_sheets"], rep["dxf_error"] = len(sheets), None
        mech.bom_extras.append(BomLine(config.sheet, len(sheets), "laser-cut parts"))
    except ValueError as e:
        rep["dxf_sheets"], rep["dxf_error"] = 0, str(e)
    try:
        bom = bom_from_mechanism(mech, group=False)
        rep["bom"] = {"items": len(bom.purchased), "cost_usd": round(bom.cost_usd, 2),
                      "unpriced": [r.key for r in bom.unpriced],
                      "unverified_links": [r.key for r in bom.purchased
                                           if not r.verified and not r.same_pack_as]}
        rep["bom_error"] = None
    except KeyError as e:
        rep["bom"], rep["bom_error"] = {}, str(e)
    rep["problems"] = (
        [f"plan: {v}" for v in rep["plan_violations"]]
        + [f"contract {k}: {v}" for k, vs in rep["contract"].items() for v in vs]
        + [f"clash {k}: {c['a']} x {c['b']} {c['mm3']} mm^3"
           for k, cs in rep["clash"].items() for c in cs]
        + [f"solid {k}: {s['part']} ({s['solids']} solids, valid={s['valid']})"
           for k, ss in rep["solids"].items() for s in ss]
        + ([f"dxf: {rep['dxf_error']}"] if rep["dxf_error"] else [])
        + ([f"bom: {rep['bom_error']}"] if rep["bom_error"] else [])
    )
    rep["warnings"] = []
    if config.robot and rep["chassis"].get("rear_screws_per_servo", 0) < 2:
        rep["warnings"].append(f"only {rep['chassis'].get('rear_screws_per_servo', 0)} rear "
                               "screw(s) per servo into the centre plates")
    rep["seconds"] = round(time.time() - t0, 1)
    return rep


def markdown(report: dict) -> str:
    cfg = report["config"]
    lines = ["# Fabrication audit", "",
             "Config: " + ", ".join(f"{k} `{v}`" for k, v in cfg.items()), "",
             "| module | layers | parts | clashes | contract | plan | DXF sheets | BOM items "
             "| est. cost | result |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for module, rep in report["modules"].items():
        n_clash = sum(len(v) for v in rep["clash"].values())
        n_contract = sum(len(v) for v in rep["contract"].values())
        lines.append(
            f"| {module} | {rep['layers']} | {rep['parts']} | {n_clash} | {n_contract} "
            f"| {len(rep['plan_violations'])} | {rep['dxf_sheets']} "
            f"| {rep['bom'].get('items', '-')} | ${rep['bom'].get('cost_usd', 0):.2f} "
            f"| {'OK' if not rep['problems'] else 'FAIL'} |")
    lines.append("")
    for module, rep in report["modules"].items():
        lines += [f"## {module}", "", "```", rep["plan"], "```", "",
                  "Chassis: " + ", ".join(f"{k} {v}" for k, v in rep["chassis"].items()), "",
                  f"Clash check at {', '.join(rep['clash'])}; contract at "
                  f"{', '.join(rep['contract'])}. {rep['seconds']} s.", ""]
        if rep["problems"]:
            lines += ["Problems:", ""] + [f"* {p}" for p in rep["problems"]] + [""]
        if rep["warnings"]:
            lines += ["Warnings:", ""] + [f"* {w}" for w in rep["warnings"]] + [""]
        if rep["bom"].get("unpriced"):
            lines.append("No listed price: " + ", ".join(rep["bom"]["unpriced"]) + ".")
        if rep["bom"].get("unverified_links"):
            lines.append("Preferred offer not verified: "
                         + ", ".join(rep["bom"]["unverified_links"]) + ".")
        lines.append("")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--modules", default=",".join(MODULES))
    ap.add_argument("--ts-contract", default="0,1.6,3.2,4.8",
                    help="crank angles for the contract check")
    ap.add_argument("--ts-clash", default="1,4.38", help="crank angles for the OCCT clash check")
    add_config_args(ap)
    ap.add_argument("--out", type=Path, default=Path("build/audit"))
    args = ap.parse_args(argv)

    base = BuildConfig(sheet=args.sheet, servo=args.servo, pillar=args.pillar, pin=args.pin,
                       crank=args.crank, thickness=args.thickness)
    report: dict = {"config": {"servo": base.servo, "pillar": base.pillar, "pin": base.pin,
                               "crank": base.crank, "sheet": base.sheet,
                               "thickness": base.thickness},
                    "modules": {}}
    ts_contract = [float(x) for x in args.ts_contract.split(",")]
    ts_clash = [float(x) for x in args.ts_clash.split(",")]
    failed = False
    for module in args.modules.split(","):
        config = BuildConfig(**{**base.__dict__, "module": module})
        print(f"== {module}", flush=True)
        rep = report["modules"][module] = audit_module(module, config, ts_contract, ts_clash)
        for p in rep["problems"]:
            print(f"  {p}")
        for w in rep["warnings"]:
            print(f"  warning: {w}")
        print(f"  {rep['parts']} parts, {rep['dxf_sheets']} DXF sheet(s), "
              f"{rep['bom'].get('items', 0)} BOM items: "
              f"{'OK' if not rep['problems'] else 'FAIL'} ({rep['seconds']} s)", flush=True)
        failed |= bool(rep["problems"])
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "audit.json").write_text(json.dumps(report, indent=1))
    (args.out / "audit.md").write_text(markdown(report))
    print(f"wrote {args.out / 'audit.json'} and {args.out / 'audit.md'}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

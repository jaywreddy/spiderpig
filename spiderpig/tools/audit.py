"""Fabrication audit: can the robot be built, and does it go together?

For each module (single, double, decker, quad) of one linkage (``--linkage``,
Klann by default; a mechanism audits as its one module, one side), independent
of the unit tests:

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

    spiderpig audit                       # all modules (mise run audit)
    spiderpig audit --modules single --out build/audit
    spiderpig audit --linkage jansen --modules single,double
    spiderpig audit --linkage parallelogram_lift      # a mechanism: one side
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from spiderpig import linkage
from spiderpig.config import (
    BuildConfig,
    ParamError,
    add_build_args,
    add_design_args,
    config_from_args,
)
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.fabricate import fabricate, template_for
from spiderpig.hardware.bom import BomLine, bom_from_mechanism
from spiderpig.hardware.catalog import sheet_size
from spiderpig.layout import pack
from spiderpig.stack import verify_plan


def audit_module(module: str, config: BuildConfig, ts_contract, ts_clash, store=None) -> dict:
    """One module's audit. ``store``: the design store the config resolves into, whose
    plan is reused when it holds one (:func:`spiderpig.api.plan_config`; the project's by
    default)."""
    from spiderpig import api

    t0 = time.time()
    tmpl = template_for(config)
    design = api.plan_config(config, store if store is not None else api.PROJECT)
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
        sheets = pack(mech, sheet_size(config.sheet))
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
    add_design_args(ap)         # --linkage, --module, --phases, --proportion (as build's)
    ap.add_argument("--modules", default=None,
                    help="comma-separated leg modules (default: --module, else all of the "
                         "linkage's; a mechanism audits as one side)")
    ap.add_argument("--ts-contract", default="0,1.6,3.2,4.8",
                    help="crank angles for the contract check")
    ap.add_argument("--ts-clash", default="1,4.38", help="crank angles for the OCCT clash check")
    add_build_args(ap)
    ap.add_argument("--store", metavar="PATH",
                    help="the design store the options resolve into, whose plans are reused "
                         "(default: $SPIDERPIG_STORE, else ./.spiderpig)")
    ap.add_argument("--out", type=Path, default=Path("build/audit"))
    args = ap.parse_args(argv)
    from spiderpig.store import Store

    store = Store.of(args.store) if args.store else Store.default()
    modules = (args.modules.split(",") if args.modules else [args.module] if args.module
               else list(linkage.get(args.linkage).leg_modules))
    try:        # robot=None: a walker's robot, a mechanism's one side
        configs = [config_from_args(args, module=m, robot=None) for m in modules]
    except ParamError as e:
        ap.error(str(e))
    base = configs[0]
    report: dict = {"config": {"linkage": base.linkage, "servo": base.servo,
                               "pillar": base.pillar, "pin": base.pin, "crank": base.crank,
                               "sheet": base.sheet, "thickness": base.thickness},
                    "modules": {}}
    if base.proportions:
        report["config"]["proportions"] = dict(base.proportions)
    if base.phases is not None:
        report["config"]["phases_deg"] = [round(math.degrees(p), 6) for p in base.phases]
    ts_contract = [float(x) for x in args.ts_contract.split(",")]
    ts_clash = [float(x) for x in args.ts_clash.split(",")]
    failed = False
    for module, config in zip(modules, configs, strict=True):
        print(f"== {module}", flush=True)
        rep = report["modules"][module] = audit_module(module, config, ts_contract, ts_clash,
                                                       store)
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

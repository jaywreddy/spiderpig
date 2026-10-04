"""Fabrication audit: can the robot be built, and does it go together?

For each module (single, double, decker, quad) of one linkage (``--linkage``,
Strider by default; a mechanism audits as its one module, one side), independent
of the unit tests:

1. **plan**     — :func:`stack.verify_plan` re-checks one side's layer plan on
   a fresh, denser sampling of the whole crank cycle.
2. **contract** — :func:`construction.contract.check_side`: every part a
   construction builds stays inside the space its group claimed, at several
   crank angles.
3. **solids**   — every part of the fabricated robot is one valid B-rep solid.
4. **clash**    — pairwise OCCT intersection volume between all parts of the
   robot (both sides and the chassis) at a few crank angles. A screw in the
   part it threads into (``mech.meta["fastened"]``) is not a clash. The electronics
   deck is also checked over the whole cycle (:func:`construction.deck.deck_clearance`:
   no moving part shares its z band but the crank's horn, whose swept disc misses it).
5. **dxf**      — every laser-cut part packs onto the sheet stock.
6. **bom**      — every purchased item resolves in the catalog; the rod pins' cut
   list (identical lengths grouped, the total rod) is reported with it.
7. **snap**     — every printed pin's snap prongs stay under the construction's
   strain limit while snapping (the planner relieves a lip or deepens a slot
   to get there and refuses a pin it can't; this reports what it did).
8. **wobble**   — how far each link can tilt out of the plane on its pivot
   (:mod:`construction.wobble`): free (bore clearance over bearing length) and
   held by the faces beside it (the column's axial play), the worst per pin and
   per pillar. Reported, not failed (a link held to more than 2 deg is a warning).
9. **strength** — :mod:`spiderpig.strength`: every pin's, pillar's and the crank's
   safety factor at the design's own walking and jam loads (MuJoCo, :mod:`sim.loads`,
   cached per design; ``--pin-load WALK,JAM`` overrides, ``--no-sim`` or no MuJoCo
   falls back to the family's measured loads with a note): the bending case from the
   joint's layers (a two-link pin ``F s / 2``, a clevis, a cantilever pillar). A jam
   SF under 1 is a problem (the audit fails), under 2 jammed or 3 walking a warning,
   each naming the joint, its load and SF and fixes recomputed to clear it.
10. **manufacture** — :mod:`spiderpig.manufacture`: every laser-cut part against its
   sheet's service rules (the least hole, hole-to-edge distance, part size, inside corner
   radius); warnings, not failures.

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

from spiderpig import linkage, manufacture, strength
from spiderpig.config import (
    BuildConfig,
    ParamError,
    add_build_args,
    add_design_args,
    config_from_args,
)
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.construction.deck import deck_clearance
from spiderpig.fabricate import fabricate, template_for
from spiderpig.hardware.bom import bom_from_mechanism
from spiderpig.layout import sheet_lines
from spiderpig.stack import verify_plan


def pin_loads_for(linkage: str) -> tuple[float, float] | None:
    """The fallback walking and jam pin loads of ``linkage``'s family
    (:data:`strength.FALLBACK_PIN_LOADS`), used when the design can't be simulated."""
    return strength.family_loads(linkage)


WOBBLE_WARN_DEG = 2.0       # a link held to more tilt than this gets a warning


def audit_module(module: str, config: BuildConfig, ts_contract, ts_clash, store=None,
                 pin_loads: tuple[float, float] | None = None, sim: bool = True) -> dict:
    """One module's audit. ``store``: the design store the config resolves into, whose
    plan is reused when it holds one (:func:`spiderpig.api.plan_config`; the project's by
    default). ``pin_loads``: walking and jam N on every joint instead of the design's
    own (:func:`strength.design_loads`: simulated unless ``sim`` is off, then the
    family's)."""
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
    rep["snap"] = _snap_check(mech.meta.get("snap_strain") or {})
    loads = strength.design_loads(config, store, override=pin_loads, sim=sim)
    notes = mech.meta.get("wobble") or {}
    rep["wobble"] = wobble_check(notes)
    rep["strength"] = strength.check(notes, mech.meta, config, loads)
    if mech.meta.get("chicago"):
        rep["chicago"] = mech.meta["chicago"]
    if mech.meta.get("crank_key"):
        rep["crank_key"] = mech.meta["crank_key"]
    if mech.meta.get("crank_bolt"):
        rep["crank_bolt"] = mech.meta["crank_bolt"]
    rep["chassis"] = {k: v for k, v in mech.meta.items()
                      if k in ("centre_plates", "rear_screw", "rear_screws_per_servo",
                               "rear_engagement_mm", "ties", "tie_screw", "tie_engagement_mm")}
    if config.robot:          # the electronics deck: clear of every moving part, whole cycle
        rep["deck"] = dict(mech.meta.get("deck") or {"fitted": False},
                           clearance=deck_clearance(mech))
    try:
        lines = sheet_lines(mech, config.sheet)
        rep["dxf_sheets"], rep["dxf_error"] = sum(int(x.qty) for x in lines), None
        rep["sheets"] = {x.key: int(x.qty) for x in lines}
        mech.bom_extras.extend(lines)
    except ValueError as e:
        rep["dxf_sheets"], rep["dxf_error"] = 0, str(e)
    rep["manufacture"] = manufacture.check(mech, config.sheet)
    rep["plate_z"] = {"gaps_mm": {str(k): v for k, v in sorted(design.plan.gaps.items())},
                      "heads": design.plan.heads,
                      "thick_mm": {str(k): v for k, v in sorted(design.plan.thick.items())}}
    try:
        bom = bom_from_mechanism(mech, group=False)
        rep["bom"] = {"items": len(bom.purchased), "cost_usd": round(bom.cost_usd, 2),
                      "unpriced": [r.key for r in bom.unpriced],
                      "unverified_links": [r.key for r in bom.purchased
                                           if not r.verified and not r.same_pack_as],
                      "cuts": bom.as_dict()["cuts"]}
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
        + [f"snap: {p}" for p in rep["snap"]["problems"]]
        + [f"deck: {a} sweeps through {b}"
           for a, b in (rep.get("deck") or {}).get("clearance", {}).get("overlapping", [])]
        + strength_messages(rep["strength"], "error")
        + manufacture_messages(rep["manufacture"], "error")
    )
    rep["warnings"] = ([f"wobble: {w}" for w in rep["wobble"]["warnings"]]
                       + strength_messages(rep["strength"], "warning")
                       + manufacture_messages(rep["manufacture"], "warning"))
    sl = rep["strength"]["loads"]
    if sl.get("source") == "sim" and sl.get("jam_stalled", 1.0) < 1.0:
        rep["warnings"].append(f"strength: only {sl['jam_stalled']:.0%} of the jam cases "
                               "stalled: the jam loads are undersampled")
    if config.robot and not rep["deck"].get("fitted"):
        rep["warnings"].append(f"no electronics deck: {rep['deck'].get('why')}")
    if config.robot and rep["chassis"].get("rear_screws_per_servo", 0) < 2:
        rep["warnings"].append(f"only {rep['chassis'].get('rear_screws_per_servo', 0)} rear "
                               "screw(s) per servo into the centre plates")
    rep["seconds"] = round(time.time() - t0, 1)
    return rep


def manufacture_messages(m: dict, lv: str = "warning") -> list[str]:
    """One message per cut rule some parts break at level ``lv`` (:mod:`spiderpig.manufacture`:
    an edge under 1 x the thickness in metal, or a hole the service won't cut, is an error):
    how many parts, and the worst."""
    out = []
    counts: dict[str, int] = {}
    for i in m["issues"]:
        if i.get("level", "warning") == lv:
            counts[i["rule"]] = counts.get(i["rule"], 0) + 1
    for rule, n in sorted(counts.items()):
        worst = next(i for i in m["issues"] if i["rule"] == rule
                     and i.get("level", "warning") == lv)
        out.append(f"manufacture: {n} part(s) break the {rule} rule; worst {worst['part']} "
                   f"({worst['sheet']}): {worst['detail']}")
    return out


def strength_messages(st: dict, lv: str) -> list[str]:
    """The strength findings of level ``lv`` (``error``: the audit's problems, it fails;
    ``warning``) as one line each: the joint, its case, load and SF, and the first fix."""
    return [f"strength: {f['message']} (fix: {f['fixes'][0]})"
            for f in st["findings"] if f["level"] == lv]


def _snap_cell(snap: dict) -> str:
    if snap["worst_pct"] is None:
        return "-"
    return f"{snap['worst_pct']:.1f} % of {snap['max_pct']:.0f} %"


def _snap_check(strains: dict) -> dict:
    """The printed pins' snap strains (``mech.meta["snap_strain"]``, one entry per joint):
    the worst, the joints over the limit (none unless a construction skipped the planner's
    refusal) and the lips the planner relieved."""
    worst = max(strains.values(), key=lambda v: v["strain_pct"], default=None)
    relieved = {k: v["engage_mm"] for k, v in strains.items()
                if any(v["engage_mm"] < w["engage_mm"] for w in strains.values())}
    return {
        "joints": len(strains),
        "worst_pct": worst["strain_pct"] if worst else None,
        "max_pct": worst["max_pct"] if worst else None,
        "relieved": relieved,
        "problems": [f"{k}: prongs strain {v['strain_pct']:.1f} % (max {v['max_pct']:.0f} %)"
                     for k, v in strains.items() if v["strain_pct"] > v["max_pct"] + 1e-9],
    }


def wobble_check(notes: dict, loads: tuple[float, float] | None = None) -> dict:
    """Per kind of pivot (``pin`` / ``pillar``): the worst and mean link tilt held by the
    faces, the worst free tilt, the joint and link behind the worst, and (with ``loads``,
    walking and jam N on every joint) each kind's worst stresses at those loads (the
    design's own loads, per joint: :func:`strength.check`)."""
    out: dict = {"loads_n": list(loads) if loads else None, "warnings": []}
    for kind in ("pin", "pillar"):
        joints = {k: v for k, v in notes.items() if k.startswith(kind + ":")}
        links = [(j, e) for j, v in joints.items() for e in v["links"]]
        if not links:
            continue
        wj, we = max(links, key=lambda je: je[1]["tilt_deg"])
        row = {"joints": len(joints), "links": len(links),
               "worst_deg": we["tilt_deg"], "worst_at": f"{wj} {we['link']}",
               "mean_deg": round(sum(e["tilt_deg"] for _, e in links) / len(links), 3),
               "worst_free_deg": max(e["free_deg"] for _, e in links),
               "play_mm": max(v["play_mm"] for v in joints.values()),
               "play_basis": next(iter(joints.values()))["play_basis"],
               "max_span_mm": max(v["span_mm"] for v in joints.values())}
        if loads:
            for tag, f in zip(("walk", "jam"), loads, strict=True):
                worst = [strength.stresses(v, f) for v in joints.values()]
                row[tag] = {k: max(w[k] for w in worst)
                            for k in ("bending_mpa", "shear_mpa", "bearing_mpa")}
                row[tag]["safety"] = min(w["safety"] for w in worst)
        out[kind] = row
        out["warnings"] += [f"{j} {e['link']}: {e['tilt_deg']:.1f} deg of tilt"
                            for j, e in links if e["tilt_deg"] > WOBBLE_WARN_DEG]
    return out


def _wobble_cell(w: dict, kind: str = "pin") -> str:
    row = w.get(kind)
    if not row:
        return "-"
    return f"{row['worst_deg']:.2f} deg ({row['worst_free_deg']:.1f} free)"


def sf_cell(rep: dict, kind: str) -> str:
    """``jam / walk`` safety factors of the weakest joint of ``kind`` (``-`` without)."""
    w = ((rep.get("strength") or {}).get("worst") or {}).get(kind)
    if not w:
        return "-"
    jam, walk = w.get("jam"), w.get("walk")
    out = f"{jam['safety']:g}" if jam else "-"
    if walk:
        out += f" / {walk['safety']:g}"
    return out


def strength_lines(st: dict) -> list[str]:
    """The strength section of a module: where the loads came from, every joint's case,
    load and safety factors, and the findings with their fixes."""
    loads = st["loads"]
    lines = [f"Joint strength (jam SF under {strength.JAM_ERROR:g} fails, under "
             f"{strength.JAM_WARN:g} jammed or {strength.WALK_WARN:g} walking warns). Loads: "
             f"{loads.get('note', '')}.", "",
             "| joint | links | case | span mm | shaft | walk N | walk SF | jam N | jam SF |",
             "|---|---|---|---:|---|---:|---:|---:|---:|"]
    for r in st["rows"]:
        w, j = r.get("walk") or {}, r.get("jam") or {}
        if r["kind"] == "crank":
            lines.append(
                f"| crank | crankpins ({r['construction']}) | twist x{r['factor']:g} | - "
                f"| {r['weakest']} {min(r['capacity_nm'].values()):g} N·m "
                f"| {w.get('torque_nm', '-')} N·m | {w.get('safety', '-')} "
                f"| {j.get('torque_nm', '-')} N·m | {j.get('safety', '-')} |")
            continue
        if r["kind"] == "link":
            need = r.get("needs")
            lines.append(
                f"| {r['joint']} | {', '.join(r['pins'])} | plate"
                + (" in bending" if r["bending"] else " in tension") + " | - "
                f"| {r['sheet']} {r['thickness_mm']:g} mm"
                + (f" (needs {need['sheet']})" if need else "")
                + f" | {w.get('load_n', '-')} | {w.get('safety', '-')} "
                f"| {j.get('load_n', '-')} | {j.get('safety', '-')} |")
            continue
        lines.append(f"| {r['joint']} | {', '.join(r['links'])} | {r['case']} "
                     f"| {r['span_mm']:g} | {r['section']} | {w.get('load_n', '-')} "
                     f"| {w.get('safety', '-')} | {j.get('load_n', '-')} "
                     f"| {j.get('safety', '-')} |")
    if st["findings"]:
        lines += ["", "Strength findings:", ""]
        for f in st["findings"]:
            lines.append(f"* **{f['level'].upper()}** {f['message']}. Fixes: "
                         + "; ".join(f["fixes"]) + ".")
    return lines


def markdown(report: dict) -> str:
    cfg = report["config"]
    lines = ["# Fabrication audit", "",
             "Config: " + ", ".join(f"{k} `{v}`" for k, v in cfg.items()), "",
             "| module | layers | parts | clashes | contract | plan | DXF sheets | BOM items "
             "| est. cost | snap strain | pin tilt | pillar tilt | pin SF | pillar SF | crank SF "
             "| link SF | result |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for module, rep in report["modules"].items():
        n_clash = sum(len(v) for v in rep["clash"].values())
        n_contract = sum(len(v) for v in rep["contract"].values())
        lines.append(
            f"| {module} | {rep['layers']} | {rep['parts']} | {n_clash} | {n_contract} "
            f"| {len(rep['plan_violations'])} | {rep['dxf_sheets']} "
            f"| {rep['bom'].get('items', '-')} | ${rep['bom'].get('cost_usd', 0):.2f} "
            f"| {_snap_cell(rep['snap'])} "
            f"| {_wobble_cell(rep['wobble'])} | {_wobble_cell(rep['wobble'], 'pillar')} "
            f"| {sf_cell(rep, 'pin')} | {sf_cell(rep, 'pillar')} | {sf_cell(rep, 'crank')} "
            f"| {sf_cell(rep, 'link')} | {'OK' if not rep['problems'] else 'FAIL'} |")
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
        for kind in ("pin", "pillar"):
            row = rep["wobble"].get(kind)
            if not row:
                continue
            line = (f"Wobble, {kind}s: worst {row['worst_deg']:.2f} deg ({row['worst_at']}), "
                    f"mean {row['mean_deg']:.2f}, free {row['worst_free_deg']:.1f} deg; play "
                    f"{row['play_mm']:g} mm ({row['play_basis']}), longest span "
                    f"{row['max_span_mm']:g} mm.")
            lines.append(line)
        if rep.get("strength"):
            lines += [""] + strength_lines(rep["strength"]) + [""]
        if rep.get("chicago"):
            lens: dict = {}
            for v in rep["chicago"].values():
                lens[v["length_mm"]] = lens.get(v["length_mm"], 0) + 1
            lines.append("Chicago screws (per side): " + ", ".join(
                f"{n} x {L:g} mm" for L, n in sorted(lens.items())) + "; play " + ", ".join(
                sorted({f"{v['play_mm']:g}" for v in rep["chicago"].values()})) + " mm.")
        if k := rep.get("crank_key"):
            lines.append(
                f"Crank keys (per side): {k['keys']} x {k['key_af_mm']:g} mm AF, {k['fit']} "
                f"fit in {k['pocket_af_mm']:g} mm pockets: play {k['play_deg']:g} deg per "
                f"interface ({k['play_deg_if_0p05_big']:g} if a pocket prints 0.05 mm over); "
                + ("chain screws threadlocked." if k["threadlocker"] else "chain screws dry."))
        if (k := rep.get("crank_bolt")) and k.get("webs") == "single":
            pins = ", ".join(f"{c['at']} {c['standoff'].rsplit('_', 1)[1]} mm"
                             + (f" + {c['shims_mm']:g} mm shims" if c.get("shims_mm") else "")
                             for c in k["chains"] + k.get("journals", []))
            lines.append(
                f"Crank (per side): {k['plates']} single aluminium plates; crankpins and "
                f"journals {k.get('crankpin', 'round standoffs clamped by M4 screws')}: "
                f"{pins}.")
        elif k := rep.get("crank_bolt"):
            bolts = ", ".join(
                f"{c['at']} {c['bolt'].rsplit('_', 1)[1]} mm"
                + (f" cut to {c['cut_to_mm']:g}" if c.get("cut_to_mm") else "")
                + f" ({c['bare_layers']} bare layer{'s' * (c['bare_layers'] != 1)} under its "
                  "riders)" for c in k["chains"])
            wb = k.get("weakest_bond")
            lines.append(
                f"Crank bolts (per side): M6 x {bolts}; {k['plates']} acrylic plates in "
                f"{len(k['segments'])} cemented stacks, hex pockets {k['pocket_af_mm']:g} mm "
                f"AF: play {k['play_deg']:g} deg per joint ({k['play_deg_worst']:g} on a "
                "minimum-size head)"
                + (f"; weakest bond plates {wb['layers'][0]}/{wb['layers'][1]} "
                   f"{wb['capacity_nm']:g} N·m" if wb else "") + ".")
        if rep["snap"]["relieved"]:
            lines.append("Snap lips relieved (engage mm): " + ", ".join(
                f"{k} {v:.2f}" for k, v in rep["snap"]["relieved"].items()) + ".")
        for c in rep["bom"].get("cuts", []):
            runs = ", ".join(f"{q} x {L:.1f}" for L, q in c["pieces"])
            lines.append(f"Cut to length: {c['count']} pieces of {c['name']} "
                         f"({c['total_mm']:.0f} mm in all, from {c['stock_mm']:g} mm stock): "
                         f"{runs} mm; deburr every cut end.")
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
    ap.add_argument("--pin-load", metavar="WALK,JAM", default=None,
                    help="pin loads (N) on every joint for the pivots' stresses (default: "
                         "the design's own, simulated in MuJoCo and cached per design)")
    ap.add_argument("--no-sim", action="store_true",
                    help="no sim for the pin loads: the family's conservative measured ones")
    ap.add_argument("--out", type=Path, default=Path("build/audit"))
    args = ap.parse_args(argv)
    pin_loads = None
    if args.pin_load:
        try:
            walk, jam = (float(x) for x in args.pin_load.split(","))
        except ValueError:
            ap.error("--pin-load takes WALK,JAM in N")
        pin_loads = (walk, jam)
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
                                                       store, pin_loads, sim=not args.no_sim)
        for p in rep["problems"]:
            print(f"  {p}")
        for w in rep["warnings"]:
            print(f"  warning: {w}")
        for kind in ("pin", "pillar"):
            if rep["wobble"].get(kind):
                print(f"  {kind} tilt {_wobble_cell(rep['wobble'], kind)}")
        print(f"  joint SF jam / walk: pin {sf_cell(rep, 'pin')}, pillar "
              f"{sf_cell(rep, 'pillar')}, crank {sf_cell(rep, 'crank')}, link plate "
              f"{sf_cell(rep, 'link')} ({rep['strength']['loads'].get('source')} loads)")
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

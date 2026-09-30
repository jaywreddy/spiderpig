"""Command-line entry point: build the walking robot and everything needed to make it.

Plans the layer stack of one side, fabricates the robot (two mirror-image
sides, servos back to back in one frame; see :mod:`construction.robot`) and
writes into ``--out``:

* ``<name>.step`` / ``<name>.stl`` — the whole robot;
* ``print/`` — one STL per different printed part, placed on the build
  plate, ``<part>_mirrored.stl`` where the right side needs the mirror image,
  and ``parts.csv`` with how many of each to print;
* ``laser/<name>_sheet_*.dxf`` — every laser-cut part, kerf-compensated and
  packed on the sheet stock's size, with ``<name>_sheet_parts.csv`` saying
  which part is where;
* ``bom.csv`` / ``bom.md`` / ``bom.json`` — what to buy (quantities, packs,
  vendor links, estimated cost), cut and print.

The design parameters: ``--linkage`` (Klann by default; ``--list`` shows
them all), ``--phases`` (every leg's crank phase in degrees, e.g.
``0,175,180,355`` for a quad) and ``--proportion NAME=VALUE`` (repeatable;
overrides one of the linkage's parameters). ``spiderpig tune``
searches for good ones.

Usage
-----
    spiderpig build                           # quad robot, defaults (mise run build)
    spiderpig build --module single --out build/single
    spiderpig build --phases 0,175,180,355       # the tuned quad gait
    spiderpig build --linkage jansen --module double
    spiderpig build --list                    # linkages, modules, servos, constructions
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path

from spiderpig import construction, linkage, servos
from spiderpig.config import ParamError, add_build_args, add_design_args, config_from_args
from spiderpig.fabricate import design_side, fabricate, template_for
from spiderpig.hardware.bom import BomLine, bom_from_mechanism, group_made
from spiderpig.hardware.catalog import CATALOG, _load, sheet_size
from spiderpig.hardware.mass import filament_density
from spiderpig.layout import DEFAULT_KERF, save_sheets


def _parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Walking-robot generator")
    add_design_args(p)
    p.add_argument("--side-only", action="store_true",
                   help="build one side (no second side, no chassis)")
    add_build_args(p)
    p.add_argument("--kerf", type=float, default=DEFAULT_KERF,
                   help=f"laser kerf compensation in mm (default {DEFAULT_KERF})")
    p.add_argument("--sheet-size", type=float, nargs=2, metavar=("W", "H"), default=None,
                   help="usable sheet size in mm (default: the sheet stock's size)")
    p.add_argument("--out", type=Path, default=Path("build"),
                   help="output directory (created if missing). Default: ./build")
    p.add_argument("--name", default=None, help="file-name stem. Default: the linkage (klann)")
    p.add_argument("--no-dxf", action="store_true", help="skip the DXF sheet-packing pass")
    p.add_argument("--list", action="store_true",
                   help="list modules, servos, constructions and sheet stock")
    args = p.parse_args(argv)
    try:            # robot=None: the linkage's kind decides (a mechanism is one side)
        args.config = config_from_args(args, robot=False if args.side_only else None)
    except ParamError as e:
        p.error(str(e))
    args.name = args.name or args.config.linkage
    return args


def _list_options() -> None:
    _load()
    print("linkages (--linkage; --module one of its modules; --proportion NAME=VALUE for its "
          "parameters):")
    for key in linkage.available():
        lk = linkage.get(key)
        params = ", ".join(f"{k}={float(v):g}" for k, v in lk.params.items())
        print(f"  {key:14} {lk.name} ({', '.join(lk.leg_modules)}): {params}")
    print("servos (full rotation):")
    for key in servos.available():
        print(f"  {key:14} {servos.get(key).name}")
    for title, registry in (("pillars / pins (--pillar, --pin)", construction.AXLES),
                            ("cranks (--crank)", construction.CRANKS)):
        print(f"{title}:")
        for key, c in sorted(registry.items()):
            print(f"  {key:14} {getattr(c, 'label', '')}")
    print("sheet stock (--sheet):")
    for key, it in sorted(CATALOG.items()):
        if it.category == "sheet":
            print(f"  {key:14} {it.name}")


def _file_stem(name: str, taken: set[str]) -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]", "_", re.sub(r"^[LR]\.", "", name))
    if stem in taken:
        stem = re.sub(r"[^A-Za-z0-9_-]", "_", name)
    taken.add(stem)
    return stem


def _on_plate(part):
    """``part`` moved so it stands on z = 0, centred on the origin in XY."""
    from build123d import Location

    bb = part.bounding_box()
    return part.moved(Location((-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2,
                                -bb.min.Z)))


def export_prints(groups, out_dir: Path, density: float = 1.24) -> list[dict]:
    """One STL per different printed part (and its mirror image where needed)."""
    from build123d import Plane, export_stl

    out_dir.mkdir(parents=True, exist_ok=True)
    rows, taken = [], set()
    for g in groups:
        stem = _file_stem(g.ref.name, taken)
        part = _on_plate(g.ref.part)
        export_stl(part, str(out_dir / f"{stem}.stl"))
        same = g.qty - len(g.mirrored)
        todo = f"print {same}"
        files = [f"{stem}.stl"]
        if g.mirrored:
            export_stl(_on_plate(g.ref.part.mirror(Plane.YZ)),
                       str(out_dir / f"{stem}_mirrored.stl"))
            files.append(f"{stem}_mirrored.stl")
            todo += f", and {len(g.mirrored)} mirrored ({stem}_mirrored.stl)"
        bb = g.ref.part.bounding_box()
        rows.append({
            "file": files[0], "qty": g.qty, "mirrored": len(g.mirrored), "print": todo,
            "size_mm": f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}",
            "grams_each_100pct": round(g.ref.part.volume / 1000 * density, 1),
            "parts": " ".join(g.names),
        })
    with open(out_dir / "parts.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["file"])
        w.writeheader()
        w.writerows(rows)
    return rows


def main(argv=None) -> int:
    args = _parse_args(argv)
    if args.list:
        _list_options()
        return 0
    config = args.config
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)
    from spiderpig.api import config_warnings

    for w in config_warnings(config):       # what the API's resolve would warn about
        print(f"warning: {w}", file=sys.stderr)

    try:
        tmpl = template_for(config)     # the template stage checks every loop closes
    except linkage.AssemblyError as e:
        print(f"error: the linkage can't be assembled: {e}", file=sys.stderr)
        return 2
    phases = ",".join(f"{math.degrees(p):g}" for _, p in config.legs)
    custom = config.phases is not None or config.proportions
    design_note = (f"{config.linkage} linkage, leg phases {phases} deg, proportions "
                   f"{dict(config.proportions) or 'its defaults'}")
    if custom or config.linkage != linkage.DEFAULT:
        print(f"design: {design_note}")
    design = design_side(tmpl, config)
    plan = design.plan
    print(f"{config.module}: layer plan of one side, {plan.top + 1} layers of "
          f"{config.pitch:g} mm ({plan.height:.1f} mm):")
    print(plan.describe())
    mech = fabricate(tmpl, config, 1.0)
    if config.robot:
        m = mech.meta
        print(f"chassis: {m['centre_plates']} centre plates; rear screws "
              f"{m.get('rear_screws_per_servo', 0)} x {m.get('rear_screw')} per servo; "
              f"{m.get('ties', 0)} frame ties ({m.get('tie_screw')})")

    step_path, stl_path = out / f"{args.name}.step", out / f"{args.name}.stl"
    mech.export_step(step_path)
    mech.export_stl(stl_path)
    print(f"wrote {step_path} and {stl_path}")

    groups = {method: group_made(mech.bodies, method) for method in ("laser", "printed")}
    filament = mech.meta.get("filament", "pla_filament")
    rows = export_prints(groups["printed"], out / "print", density=filament_density(filament))
    n_print = sum(r["qty"] for r in rows)
    print(f"wrote {len(rows)} printed-part STLs for {n_print} parts to {out / 'print'}:")
    for r in rows:
        print(f"  {r['file']:28} {r['print']}")

    if not args.no_dxf:
        size = tuple(args.sheet_size) if args.sheet_size else sheet_size(config.sheet)
        sheets = save_sheets(mech, out / "laser" / f"{args.name}_sheet", sheet_size=size,
                             kerf=args.kerf)
        n_laser = sum(g.qty for g in groups["laser"])
        print(f"wrote {len(sheets)} DXF sheet(s) of {size[0]:.0f} x {size[1]:.0f} mm with "
              f"{n_laser} laser-cut parts ({len(groups['laser'])} different) to {out / 'laser'}")
        mech.bom_extras.append(BomLine(config.sheet, len(sheets), "laser-cut parts"))

    title = (f"{config.module} {'robot' if config.robot else 'side'}, {config.servo}, "
             f"{config.pillar} pillars, {config.pin} pins, {config.crank} crank, {config.sheet}")
    bom = bom_from_mechanism(mech, title=title, filament=filament, groups=groups)
    if args.no_dxf:
        bom.notes.append("Sheet stock not counted (--no-dxf).")
    if custom or config.linkage != linkage.DEFAULT:
        bom.notes.append(f"Design: {design_note}.")
    paths = bom.write(out)
    print(f"wrote {', '.join(str(p) for p in paths)}: {len(bom.purchased)} items to buy, "
          f"est. ${bom.cost_usd:.2f} ({len(bom.unpriced)} without a listed price)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

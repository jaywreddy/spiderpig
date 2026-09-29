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

Usage
-----
    uv run python main.py                           # quad robot, defaults
    uv run python main.py --module single --out build/single
    uv run python main.py --list                    # modules, servos, constructions
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import construction
import servos
from fabricate import MODULES, BuildConfig, design_side, fabricate, sheet_thickness
from hardware.bom import BomLine, bom_from_mechanism, group_made
from layout import DEFAULT_KERF, save_sheets


def module_template(module: str):
    """The kinematic template of one side for ``module`` (see ``fabricate.MODULES``)."""
    from klann import (
        build_double_decker_template,
        build_double_double_decker_template,
        build_double_template,
        build_klann_template,
        create_klann_geometry,
    )

    builders = {
        "single": lambda: build_klann_template(create_klann_geometry()),
        "double": build_double_template,
        "decker": build_double_decker_template,
        "quad": build_double_double_decker_template,
    }
    if module not in builders:
        raise ValueError(f"unknown module {module!r}; have {sorted(builders)}")
    return builders[module]()


def config_from_args(args: argparse.Namespace) -> BuildConfig:
    return BuildConfig(module=args.module, robot=not args.side_only, sheet=args.sheet,
                       servo=args.servo, pillar=args.pillar, pin=args.pin, crank=args.crank,
                       thickness=args.thickness)


def add_config_args(p: argparse.ArgumentParser) -> None:
    """The build options shared by the CLI and the audit."""
    d = BuildConfig()
    p.add_argument("--servo", default=d.servo, help=f"servo model (default {d.servo})")
    p.add_argument("--pillar", default=d.pillar,
                   help=f"construction of the frame pivots (default {d.pillar})")
    p.add_argument("--pin", default=d.pin,
                   help=f"construction of the pivots between links (default {d.pin})")
    p.add_argument("--crank", default=d.crank, help=f"crank construction (default {d.crank})")
    p.add_argument("--sheet", default=d.sheet, help=f"sheet stock catalog item (default {d.sheet})")
    p.add_argument("--thickness", type=float, default=None,
                   help="measured sheet thickness in mm (default: the sheet's nominal)")


def _parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Klann walking-robot generator")
    p.add_argument("--module", choices=MODULES, default="quad",
                   help="legs per side: single, double (mirrored pair), decker (two legs on "
                   "one crankshaft), quad (two mirrored deckers). Default: quad")
    p.add_argument("--side-only", action="store_true",
                   help="build one side (no second side, no chassis)")
    add_config_args(p)
    p.add_argument("--kerf", type=float, default=DEFAULT_KERF,
                   help=f"laser kerf compensation in mm (default {DEFAULT_KERF})")
    p.add_argument("--sheet-size", type=float, nargs=2, metavar=("W", "H"), default=None,
                   help="usable sheet size in mm (default: the sheet stock's size)")
    p.add_argument("--out", type=Path, default=Path("build"),
                   help="output directory (created if missing). Default: ./build")
    p.add_argument("--name", default="klann", help="file-name stem. Default: klann")
    p.add_argument("--no-dxf", action="store_true", help="skip the DXF sheet-packing pass")
    p.add_argument("--list", action="store_true",
                   help="list modules, servos, constructions and sheet stock")
    return p.parse_args(argv)


def _list_options() -> None:
    from hardware.catalog import CATALOG, _load

    _load()
    print("modules: " + ", ".join(MODULES))
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


def sheet_size(config: BuildConfig) -> tuple[float, float]:
    from hardware.catalog import get

    size = get(config.sheet).dims.get("sheet_mm")
    return (float(size[0]), float(size[1])) if size else (200.0, 200.0)


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
    config = config_from_args(args)
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    tmpl = module_template(args.module)
    design = design_side(tmpl, config)
    plan = design.plan
    print(f"{args.module}: layer plan of one side, {plan.top + 1} layers of "
          f"{sheet_thickness(config):g} mm ({plan.height:.1f} mm):")
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
    from hardware.catalog import get

    filament = mech.meta.get("filament", "pla_filament")
    rows = export_prints(groups["printed"], out / "print",
                         density=float(get(filament).dims.get("density", 1.24)))
    n_print = sum(r["qty"] for r in rows)
    print(f"wrote {len(rows)} printed-part STLs for {n_print} parts to {out / 'print'}:")
    for r in rows:
        print(f"  {r['file']:28} {r['print']}")

    if not args.no_dxf:
        size = tuple(args.sheet_size) if args.sheet_size else sheet_size(config)
        sheets = save_sheets(mech, out / "laser" / f"{args.name}_sheet", sheet_size=size,
                             kerf=args.kerf)
        n_laser = sum(g.qty for g in groups["laser"])
        print(f"wrote {len(sheets)} DXF sheet(s) of {size[0]:.0f} x {size[1]:.0f} mm with "
              f"{n_laser} laser-cut parts ({len(groups['laser'])} different) to {out / 'laser'}")
        mech.bom_extras.append(BomLine(config.sheet, len(sheets), "laser-cut parts"))

    title = (f"{args.module} {'robot' if config.robot else 'side'}, {config.servo}, "
             f"{config.pillar} pillars, {config.pin} pins, {config.crank} crank, {config.sheet}")
    bom = bom_from_mechanism(mech, title=title, filament=filament, groups=groups)
    if args.no_dxf:
        bom.notes.append("Sheet stock not counted (--no-dxf).")
    paths = bom.write(out)
    print(f"wrote {', '.join(str(p) for p in paths)}: {len(bom.purchased)} items to buy, "
          f"est. ${bom.cost_usd:.2f} ({len(bom.unpriced)} without a listed price)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

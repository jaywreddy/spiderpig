"""Command-line entry point for the Klann walking-linkage generator.

Builds the chosen assembly, plans its layer stack, fabricates every part
with the chosen joinery and servo, and writes:

* ``<name>.step`` / ``<name>.stl`` — the full assembly (servo included);
* ``<name>_sheet_*.dxf`` — every laser-cut part, kerf-compensated and packed;
* ``bom.csv`` / ``bom.md`` / ``bom.json`` — what to cut, print and buy
  (quantities, packs, vendor links, estimated cost).

Usage
-----
    uv run python main.py                               # single leg, defaults
    uv run python main.py --mode quad --servo xl430_w250 --pin bearing
    uv run python main.py --list                        # available options
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

import joinery
import servos
from fabricate import BuildConfig, fabricate, plan_for
from hardware.bom import BomLine, bom_from_mechanism
from joinery.base import JoineryParams
from klann import (
    build_double_decker_template,
    build_double_double_decker_template,
    build_double_template,
    build_klann_template,
    create_klann_geometry,
)
from layout import DEFAULT_KERF

_MODE_TEMPLATES = {
    "single": lambda: build_klann_template(create_klann_geometry()),
    "double": build_double_template,
    "decker": build_double_decker_template,
    "quad": build_double_double_decker_template,
}


def _parse_args() -> argparse.Namespace:
    d = BuildConfig()
    p = argparse.ArgumentParser(description="Klann linkage generator")
    p.add_argument("--out", type=Path, default=Path("build"),
                   help="Output directory (created if missing). Default: ./build")
    p.add_argument("--name", default="klann", help="File-name stem. Default: klann")
    p.add_argument("--mode", choices=sorted(_MODE_TEMPLATES), default="single",
                   help="single leg, mirrored pair (double), two legs 90° apart on one "
                   "crankshaft (decker), or two mirrored pairs (quad). Default: single")
    p.add_argument("--servo", default=d.servo, help=f"Servo model. Default: {d.servo}")
    p.add_argument("--pin", default=d.pin, help=f"Joinery between links. Default: {d.pin}")
    p.add_argument("--frame-joinery", default=d.frame,
                   help=f"Joinery at the frame pivots. Default: {d.frame}")
    p.add_argument("--crankpin", default=d.crankpin,
                   help=f"Crankpin option. Default: {d.crankpin}")
    p.add_argument("--sheet", default=d.sheet, help=f"Sheet stock. Default: {d.sheet}")
    p.add_argument("--thickness", type=float, default=None,
                   help="Override the sheet thickness (mm)")
    p.add_argument("--axle", type=float, default=d.params.axle_d,
                   help=f"Nominal axle diameter (mm). Default: {d.params.axle_d}")
    p.add_argument("--kerf", type=float, default=DEFAULT_KERF,
                   help=f"Laser kerf compensation (mm). Default: {DEFAULT_KERF}")
    p.add_argument("--no-idler-bracket", action="store_true",
                   help="Don't support the servo's back with a bracket")
    p.add_argument("--no-dxf", action="store_true", help="Skip the DXF sheet-packing pass.")
    p.add_argument("--list", action="store_true", help="List servos and joinery options.")
    return p.parse_args()


def _list_options() -> None:
    print("servos:")
    for key in servos.available():
        s = servos.get(key)
        print(f"  {key:14} {s.name}")
    for kind in ("pin", "frame", "crankpin"):
        print(f"joinery for {kind} pivots:")
        for o in joinery.options(kind):
            print(f"  {o.key:14} {o.label}")


def main() -> None:
    args = _parse_args()
    if args.list:
        _list_options()
        return
    args.out.mkdir(parents=True, exist_ok=True)
    config = BuildConfig(
        sheet=args.sheet, pin=args.pin, frame=args.frame_joinery, crankpin=args.crankpin,
        params=replace(JoineryParams(), axle_d=args.axle), servo=args.servo,
        idler_bracket=not args.no_idler_bracket, thickness=args.thickness,
    )

    tmpl = _MODE_TEMPLATES[args.mode]()
    plan = plan_for(tmpl, config)
    print(f"stack plan ({plan.height:.0f} mm):\n{plan.describe()}")
    mech = fabricate(tmpl.freeze_at(1.0), plan, config).solved()

    step_path = args.out / f"{args.name}.step"
    stl_path = args.out / f"{args.name}.stl"
    mech.export_step(step_path)
    mech.export_stl(stl_path)
    print(f"wrote {step_path} ({step_path.stat().st_size} B)")
    print(f"wrote {stl_path} ({stl_path.stat().st_size} B)")

    if not args.no_dxf:
        sheets = mech.save_layouts(args.out / f"{args.name}_sheet", kerf=args.kerf)
        print(f"wrote {len(sheets)} DXF sheet(s) with every laser-cut part")
        mech.bom_extras.append(BomLine(config.sheet, len(sheets), "laser-cut parts"))

    bom = bom_from_mechanism(mech, title=f"{args.mode}, {config.servo}, {config.pin} joints")
    paths = bom.write(args.out)
    print(f"wrote {', '.join(str(p) for p in paths)} "
          f"({len(bom.purchased)} items to buy, est. ${bom.cost_usd:.2f})")


if __name__ == "__main__":
    main()

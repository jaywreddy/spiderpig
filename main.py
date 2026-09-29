"""Command-line entry point for the Klann walking-linkage generator.

Builds the Klann mechanism, solves its tree-graph pose propagation, and
emits STEP/STL assembly files plus DXF sheets ready for a laser cutter.

Usage
-----
    uv run python main.py                      # writes to build/
    uv run python main.py --out dist/ --no-dxf # STEP + STL only
"""

from __future__ import annotations

import argparse
from pathlib import Path

from fabricate import fabricate, plan_for
from klann import (
    build_double_decker_template,
    build_double_double_decker_template,
    build_double_template,
    build_klann_template,
    create_klann_geometry,
)

_MODE_TEMPLATES = {
    "single": lambda: build_klann_template(create_klann_geometry()),
    "double": build_double_template,
    "decker": build_double_decker_template,
    "quad": build_double_double_decker_template,
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Klann linkage generator")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("build"),
        help="Output directory (created if missing). Default: ./build",
    )
    parser.add_argument(
        "--name",
        default="klann",
        help="File-name stem for the STEP/STL outputs. Default: klann",
    )
    parser.add_argument(
        "--mode",
        choices=sorted(_MODE_TEMPLATES),
        default="single",
        help="Assembly: single leg, mirrored pair (double), two legs 90° apart on "
        "one crankshaft (decker), or two mirrored pairs on one crankshaft (quad). "
        "Default: single",
    )
    parser.add_argument(
        "--no-dxf",
        action="store_true",
        help="Skip the DXF sheet-packing pass.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    tmpl = _MODE_TEMPLATES[args.mode]()
    plan = plan_for(tmpl)
    print(f"stack plan ({plan.height:.0f} mm):\n{plan.describe()}")
    mech = fabricate(tmpl.freeze_at(1.0), plan).solved()

    step_path = args.out / f"{args.name}.step"
    stl_path = args.out / f"{args.name}.stl"
    mech.export_step(step_path)
    mech.export_stl(stl_path)
    print(f"wrote {step_path} ({step_path.stat().st_size} B)")
    print(f"wrote {stl_path} ({stl_path.stat().st_size} B)")

    if not args.no_dxf:
        sheets = mech.save_layouts(args.out / f"{args.name}_sheet")
        print(f"wrote {len(sheets)} DXF sheet(s) with the laser-cut links")


if __name__ == "__main__":
    main()

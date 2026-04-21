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

from klann import KlannLinkage


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
        "--no-dxf",
        action="store_true",
        help="Skip the DXF sheet-packing pass.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    mech = KlannLinkage().solved()

    step_path = args.out / f"{args.name}.step"
    stl_path = args.out / f"{args.name}.stl"
    mech.export_step(step_path)
    mech.export_stl(stl_path)
    print(f"wrote {step_path} ({step_path.stat().st_size} B)")
    print(f"wrote {stl_path} ({stl_path.stat().st_size} B)")

    if not args.no_dxf:
        prefix = args.out / f"{args.name}_sheet"
        mech.save_layouts(prefix)


if __name__ == "__main__":
    main()

"""``spiderpig control``: an arm's kinematics, steps and controller files.

    spiderpig control axes  --linkage palletizer            # each joint's ratio and steps
    spiderpig control fk    --linkage palletizer --q q1=0 q2=70 q3=-10   # degrees -> tip
    spiderpig control ik    --linkage palletizer --at 200 0 120          # tip (mm) -> degrees
    spiderpig control klipper --linkage palletizer --out build/klipper   # printer.cfg
    spiderpig control homing  --linkage palletizer          # the homing sequence

The design options are the build's (``--linkage``, ``--turntable``, ``--tool``, ...); the
tip is in the base frame (:mod:`.arm`).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def _parse(argv):
    from spiderpig.config import add_build_args, add_design_args, config_from_args

    p = argparse.ArgumentParser(prog="spiderpig control", description=(__doc__ or "")
                                .splitlines()[0])
    p.add_argument("action", choices=("axes", "fk", "ik", "klipper", "homing"))
    add_design_args(p)
    add_build_args(p)
    p.add_argument("--side-only", action="store_true", help="the side alone (no turntable)")
    p.add_argument("--q", nargs="*", default=[], metavar="NAME=DEG",
                   help="fk: the joints' angles (degrees; the planar inputs' design angles)")
    p.add_argument("--at", nargs=3, type=float, metavar=("X", "Y", "Z"),
                   help="ik: the tip's target (mm, the base frame)")
    p.add_argument("--out", type=Path, default=Path("build") / "klipper",
                   help="klipper: the folder printer.cfg is written to")
    args = p.parse_args(argv)
    args.config = config_from_args(args, robot=False if args.side_only else None)
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse(sys.argv[1:] if argv is None else argv)
    from spiderpig.control import homing, klipper
    from spiderpig.control.arm import ArmModel, IKError
    from spiderpig.control.mapping import axes as axes_of
    from spiderpig.fabricate import design_side, template_for

    cfg = args.config
    side = cfg.side_config()
    design = design_side(template_for(side), side)
    axes = axes_of(cfg, design)
    if args.action == "axes":
        for n, a in axes.items():
            print(f"{n}: {a.ratio:.6g}:1, {a.motor}, {a.steps_per_rev} x {a.microsteps} steps "
                  f"per turn, {math.degrees(1 / a.per_rad) * 3600:.2f} arcsec per microstep, "
                  f"direction {a.direction:+d}")
        return 0
    if args.action == "homing":
        sensors = homing.sensors_of(cfg, axes)
        for i, s in enumerate(homing.plan(axes, sensors), 1):
            print(f"{i}. [{s.axis}] {s.action}")
        return 0
    if args.action == "klipper":
        args.out.mkdir(parents=True, exist_ok=True)
        path = args.out / "printer.cfg"
        ctl = cfg.role("controller")
        path.write_text(klipper.printer_cfg(axes, cfg.linkage, ctl, homing.sensors_of(cfg, axes)))
        print(f"wrote {path} ({len(axes)} joints; "
              + (f"the {ctl} controller's pins)" if ctl else "fill its <PLACEHOLDER> pins)"))
        return 0
    model = ArmModel.from_config(cfg, design)
    if args.action == "fk":
        q = {k: math.radians(float(v)) for k, v in (a.split("=") for a in args.q)}
        missing = [n for n in model.inputs if n not in q]
        if missing:
            print(f"error: give {', '.join(missing)}", file=sys.stderr)
            return 2
        for n, (lo, hi) in model.ranges.items():
            if not lo - 1e-9 <= q[n] <= hi + 1e-9:
                print(f"error: {n} {math.degrees(q[n]):g} deg is outside "
                      f"{math.degrees(lo):g}..{math.degrees(hi):g}", file=sys.stderr)
                return 2
        x, y, z = model.fk(q)
        print(f"tip: {x:.3f} {y:.3f} {z:.3f} mm")
        return 0
    if args.at is None:
        print("error: ik needs --at X Y Z", file=sys.stderr)
        return 2
    try:
        q = model.ik(args.at)
    except IKError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(" ".join(f"{n}={math.degrees(v):.4f}" for n, v in q.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())

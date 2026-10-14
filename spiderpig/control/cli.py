"""``spiderpig control``: a robot's motors, its kinematics and its controller's files.

    spiderpig control axes  --linkage palletizer            # each joint's ratio and steps
    spiderpig control fk    --linkage palletizer --q q1=0 q2=70 q3=-10   # degrees -> tip
    spiderpig control ik    --linkage palletizer --at 200 0 120          # tip (mm) -> degrees
    spiderpig control klipper --linkage palletizer --out build/klipper   # printer.cfg
    spiderpig control homing  --linkage palletizer          # the homing sequence
    spiderpig control axes                                  # a walker: each crank's servo
    spiderpig control servo --out build/servos              # its servos' setup: servos.json
    spiderpig control homing                                # each crank's one-time zero

What a design's motors are (:func:`actuators.controllers.motors_of`) picks the actions: an
arm's steppers (``axes``, ``fk``, ``ik``, ``klipper``, ``homing``: :mod:`.mapping`,
:mod:`.arm`, :mod:`.klipper`, :mod:`.homing`), a crank's serial-bus servos (``axes``,
``servo``, ``homing``: :mod:`.servos`). The design options are the build's (``--linkage``,
``--controller``, ...); an arm's tip is in the base frame (:mod:`.arm`).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ACTIONS = ("axes", "fk", "ik", "klipper", "homing", "servo")
STEPPER_ONLY = ("fk", "ik", "klipper")
SERVO_ONLY = ("servo",)


def _parse(argv):
    from spiderpig.config import add_build_args, add_design_args, config_from_args

    p = argparse.ArgumentParser(prog="spiderpig control", description=(__doc__ or "")
                                .splitlines()[0])
    p.add_argument("action", choices=ACTIONS)
    add_design_args(p)
    add_build_args(p)
    p.add_argument("--side-only", action="store_true",
                   help="the side alone (an arm's without its turntable, a walker's one crank)")
    p.add_argument("--q", nargs="*", default=[], metavar="NAME=DEG",
                   help="fk: the joints' angles (degrees; the planar inputs' design angles)")
    p.add_argument("--at", nargs=3, type=float, metavar=("X", "Y", "Z"),
                   help="ik: the tip's target (mm, the base frame)")
    p.add_argument("--out", type=Path, default=None,
                   help="klipper: the folder printer.cfg is written to (build/klipper); "
                        "servo: the folder servos.json is written to (build/servos)")
    args = p.parse_args(argv)
    args.config = config_from_args(args, robot=False if args.side_only else None)
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse(sys.argv[1:] if argv is None else argv)
    from spiderpig.actuators.controllers import motors_of

    cfg = args.config
    motors = motors_of(cfg)
    if motors == "servo" and args.action in STEPPER_ONLY:
        print(f"error: {args.action} is a stepper-driven design's (an arm's); "
              f"{cfg.linkage}'s cranks turn on bus servos: axes, servo, homing",
              file=sys.stderr)
        return 2
    if motors == "actuator" and args.action in SERVO_ONLY:
        print(f"error: {args.action} is a bus-servo design's (a crank's); {cfg.linkage}'s "
              "joints turn on steppers: axes, fk, ik, klipper, homing", file=sys.stderr)
        return 2
    if motors == "actuator":
        return _stepper_main(args)
    try:
        return _servo_main(args)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


def _servo_main(args) -> int:
    """A crank's servos (:mod:`.servos`)."""
    from spiderpig.control import servos

    cfg = args.config
    if args.action == "axes":
        for s in servos.bus_servos(cfg):
            where = f", {s.side} side" if s.side else ""
            print(f"{s.name}: bus ID {s.id}{where}, {s.servo}; legs' crank phases "
                  + ", ".join(f"{p:g}" for p in s.phases_deg)
                  + f" deg; forward {s.forward} facing its horn; zero: crankpin {s.zero_pin} "
                    f"at {s.zero_deg:g} deg about O")
        return 0
    if args.action == "homing":
        for i, (name, action) in enumerate(servos.homing_steps(cfg), 1):
            print(f"{i}. [{name}] {action}")
        return 0
    doc = servos.setup(cfg)
    out = args.out or Path("build") / "servos"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "servos.json"
    path.write_text(json.dumps(doc, indent=1) + "\n")
    ctl = doc["controller"]
    print(f"wrote {path} ({len(doc['servos'])} servo{'s' if len(doc['servos']) > 1 else ''} "
          f"on a {doc['bus']['protocol']} bus at {doc['bus']['baud']} bit/s; "
          + (f"the {ctl['key']} controller)" if ctl else "your own controller)"))
    for note in doc["notes"]:
        print(f"note: {note}")
    return 0


def _stepper_main(args) -> int:
    """An arm's steppers (:mod:`.mapping`, :mod:`.arm`, :mod:`.klipper`)."""
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
        out = args.out or Path("build") / "klipper"
        out.mkdir(parents=True, exist_ok=True)
        path = out / "printer.cfg"
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

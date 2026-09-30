"""Simulate the walker in MuJoCo and print how it walks.

    spiderpig sim                                 # quad, both drives 80 %
    spiderpig sim --module single --seconds 6
    spiderpig sim --left 0.4 --right -0.4         # turn in place
    spiderpig sim --left 40rpm --right 40rpm
    spiderpig sim --xml build/quad.xml            # + quad.json

The design and build options are every other tool's (:mod:`config`:
``--linkage``, ``--module``, ``--phases`` in degrees, ``--proportion
NAME=VALUE``, the servo, the constructions, the sheet). Drive speeds are
fractions of the servo's no-load speed (``0.8``, ``-1``), percentages
(``80%``) or crank rpm (``40rpm``); positive walks forward. The drives start
after ``--settle`` seconds at rest; metrics skip the first ``--skip``
seconds.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from spiderpig.config import (
    BuildConfig,
    ParamError,
    add_build_args,
    add_design_args,
    config_from_args,
)
from spiderpig.sim.mjcf import RPM, SimParams, build_mjcf, drive_limits
from spiderpig.sim.run import kinematic_gait, simulate, walk_metrics


def parse_speed(text: str, vmax: float) -> float:
    """Crank speed in rad/s from ``0.8`` (fraction of ``vmax``), ``80%`` or ``40rpm``."""
    t = text.strip().lower()
    if t.endswith("rpm"):
        return float(t[:-3]) * RPM
    if t.endswith("%"):
        return float(t[:-1]) / 100.0 * vmax
    v = float(t)
    if abs(v) > 1.0:
        raise argparse.ArgumentTypeError(
            f"{text!r}: plain numbers are fractions of the no-load speed (-1..1); "
            "use e.g. 40rpm or 80%")
    return v * vmax


def _args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split("\n", 1)[1])
    add_design_args(p)
    add_build_args(p)
    p.add_argument("--seconds", type=float, default=4.0,
                   help="seconds of driving after --settle (4)")
    p.add_argument("--left", default="0.8", help="left drive speed (0.8 of no-load)")
    p.add_argument("--right", default="0.8", help="right drive speed (0.8 of no-load)")
    p.add_argument("--settle", type=float, default=0.5, help="seconds at rest first (0.5)")
    p.add_argument("--skip", type=float, default=1.0, help="metrics skip this long (1.0 s)")
    p.add_argument("--friction", type=float, default=SimParams().friction,
                   help=f"robot/floor friction ({SimParams().friction})")
    p.add_argument("--timestep", type=float, default=SimParams().timestep,
                   help=f"simulation step, s ({SimParams().timestep})")
    p.add_argument("--xml", type=Path, default=None,
                   help="write the MJCF here (and its metadata next to it as .json)")
    p.add_argument("--json", action="store_true", help="print the metrics as JSON")
    args = p.parse_args(argv)
    try:
        args.config = config_from_args(args)
    except ParamError as e:
        p.error(str(e))
    return args


def _report(cfg: BuildConfig, m: dict, kin: dict, left: float, right: float, seconds: float):
    rpm = 1.0 / RPM
    revs_per_s = m["revolutions"] / m["duration"] if m["duration"] else 0.0
    kin_speed = kin["stride"] * revs_per_s
    print(f"{cfg.linkage} {cfg.module} robot, {cfg.servo}, {m['mass'] * 1e3:.0f} g, {seconds:g} s; "
          f"drives L {left * rpm:.1f} rpm, R {right * rpm:.1f} rpm")
    print(f"  walking   speed {m['speed']:.1f} mm/s along the start heading, lateral "
          f"{m['lateral']:.1f} mm, heading drift {m['heading_drift']:.1f} deg "
          f"({m['yaw_rate']:.2f} deg/s)")
    print(f"            {m['revolutions']:.2f} crank revolutions, stride {m['stride']:.1f} "
          f"mm/rev (kinematics, no slip: {kin['stride']:.1f} mm/rev = {kin_speed:.1f} mm/s; "
          f"one foot's stance {kin['stance_length']:.1f} mm)")
    print(f"  body      height {m['height']:.1f} mm, bob {m['bob']:.1f} mm/rev (kinematic "
          f"{kin['bob']:.1f}), pitch range {m['pitch_range']:.1f} deg, roll range "
          f"{m['roll_range']:.1f} deg, max tilt {m['max_tilt']:.1f} deg")
    print(f"            fell over: {'YES' if m['fell'] else 'no'}; something other than a foot "
          f"on the floor {m['body_contact'] * 100:.0f} % of the time")
    for name, d in m["torque"].items():
        print(f"  {name:9s} torque peak {d['peak']:.3f} N·m ({d['peak_fraction'] * 100:.0f} % of "
              f"the {d['limit']:.2f} stall), mean {d['mean']:.3f}, rms {d['rms']:.3f}; "
              f"saturated {d['saturated'] * 100:.1f} %")
        print(f"            speed-torque envelope use p95 {d['envelope_p95']:.2f} (peak "
              f"{d['envelope']:.2f}); under the mean load a real servo turns "
              f"{d['speed_under_load'] * rpm:.1f} rpm at full voltage; power {d['power']:.2f} W")
    print(f"  feet      {m['feet_down']:.2f} down on average; slip in contact mean "
          f"{m['slip']:.1f} mm/s, max {m['slip_max']:.0f} mm/s")
    print(f"  model     loop closure error max {m['loop_error']:.3f} mm, deepest floor "
          f"contact {m['penetration']:.2f} mm")


def main(argv=None) -> int:
    from spiderpig import servos

    args = _args(argv)
    cfg: BuildConfig = args.config
    params = SimParams(friction=args.friction, timestep=args.timestep)
    vmax, _ = drive_limits(servos.get(cfg.servo))
    left, right = parse_speed(args.left, vmax), parse_speed(args.right, vmax)
    if args.xml is not None:
        xml, meta = build_mjcf(cfg, params)
        args.xml.parent.mkdir(parents=True, exist_ok=True)
        args.xml.write_text(xml)
        args.xml.with_suffix(".json").write_text(json.dumps(meta, indent=1))
        print(f"wrote {args.xml} and {args.xml.with_suffix('.json')}", file=sys.stderr)
    result = simulate(cfg, [(0.0, 0.0, 0.0), (args.settle, left, right)],
                      args.settle + args.seconds, params=params)
    m = walk_metrics(result, skip=args.settle + args.skip)
    kin = kinematic_gait(cfg)
    if args.json:
        print(json.dumps({"metrics": m, "kinematic": kin}, indent=1))
    else:
        _report(cfg, m, kin, left, right, args.seconds)
    return 0


if __name__ == "__main__":
    sys.exit(main())

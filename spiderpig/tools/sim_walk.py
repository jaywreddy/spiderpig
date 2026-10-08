"""Simulate the walker in MuJoCo and print how it walks.

    spiderpig sim                                 # quad, both drives 80 %
    spiderpig sim --module single --seconds 6
    spiderpig sim --left 0.4 --right -0.4         # turn in place
    spiderpig sim --left 40rpm --right 40rpm
    spiderpig sim --xml build/quad.xml            # + quad.json
    spiderpig sim 1a2b3c4d5e6f7a8b                # a stored design (its exported MJCF if any)
    spiderpig sim --mjcf out/heel/trotbot_heel.xml --linkage trotbot_heel

The design and build options are every other tool's (:mod:`config`:
``--linkage``, ``--module``, ``--phases`` in degrees, ``--proportion
NAME=VALUE``, the servo, the constructions, the sheet); a stored design's id
(from the API's ``resolve``, ``--store`` picks the store) stands for all of
them, and runs the MJCF its ``export`` wrote when there is one. ``--mjcf FILE``
runs that model (with the ``.json`` written beside it by ``export`` or
``--xml``) for the design the other options describe, instead of building
one. Drive speeds are fractions of the servo's no-load speed (``0.8``,
``-1``), percentages (``80%``) or crank rpm (``40rpm``); positive walks
forward. The drives start after ``--settle`` seconds at rest; metrics skip
the first ``--skip`` seconds.
"""

from __future__ import annotations

import argparse
import json
import math
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
from spiderpig.sim.run import compare_with_walk, kinematic_gait, simulate, walk_metrics


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
    p = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=(__doc__ or "").partition("\n")[2])
    p.add_argument("design", nargs="?", metavar="DESIGN",
                   help="a stored design's id (16 hex digits, from resolve) in place of the "
                        "build options; its exported MJCF is run when the store has one")
    p.add_argument("--store", metavar="PATH",
                   help="the design store (default: $SPIDERPIG_STORE, else ./.spiderpig)")
    p.add_argument("--mjcf", type=Path, default=None, metavar="FILE",
                   help="run this MJCF (its .json beside it) for the design instead of "
                        "building the model")
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
    p.add_argument("--contact-sweep", action="store_true",
                   help="also run at contact solref 5 and 20 ms (the default is 10) and print "
                        "the support metrics' range: they hinge on that unvalidated softness")
    p.add_argument("--xml", type=Path, default=None,
                   help="write the MJCF here (and its metadata next to it as .json)")
    p.add_argument("--json", action="store_true", help="print the metrics as JSON")
    args = p.parse_args(argv)
    args.model = None                # (xml, meta) to run instead of building
    if args.design:
        from spiderpig.store import Store
        from spiderpig.view import load_design

        store = Store.of(args.store) if args.store else Store.default()
        try:
            design = load_design(args.design, store)
        except (KeyError, ValueError) as e:
            p.error(str(e))
        args.config = design.config
        if args.mjcf is None:        # the design's exported MJCF, when the store has one
            assert design.store is not None  # api.load gives the design its store
            rep = design.reports.get("export") or design.store.read_report(design.id, "export")
            if rep is None or isinstance(rep, dict):    # the stored report's JSON, or none
                files = (rep or {}).get("files") or []
            else:                                       # the export's report (api.ExportReport)
                files = getattr(rep, "files", None) or []
            xml = next((Path(f) for f in files if str(f).endswith(".xml")), None)
            if xml is not None and xml.is_file() and xml.with_suffix(".json").is_file():
                args.mjcf = xml
    else:
        try:
            args.config = config_from_args(args)
        except ParamError as e:
            p.error(str(e))
    if args.mjcf is not None:
        meta_path = args.mjcf.with_suffix(".json")
        if not args.mjcf.is_file() or not meta_path.is_file():
            p.error(f"--mjcf needs {args.mjcf} and its metadata {meta_path} beside it (what "
                    f"export(design, ['mjcf']) and spiderpig sim --xml write)")
        args.model = (args.mjcf.read_text(), json.loads(meta_path.read_text()))
        print(f"running {args.mjcf}", file=sys.stderr)
    return args


def _report(cfg: BuildConfig, m: dict, kin: dict, left: float, right: float, seconds: float,
            params: SimParams):
    rpm = 1.0 / RPM
    print(f"{cfg.linkage} {cfg.module} robot, {cfg.servo}, {m['mass'] * 1e3:.0f} g (payload "
          f"{params.payload_g:.0f} g of it), {seconds:g} s; drives L {left * rpm:.1f} rpm, R "
          f"{right * rpm:.1f} rpm; the cranks phase-locked (PI {params.phase_lock_kp:g}/"
          f"{params.phase_lock_ki:g}) against a {params.servo_mismatch * 100:.0f} % slower "
          f"right servo" if params.phase_lock_kp or params.phase_lock_ki else
          f"{cfg.linkage} {cfg.module} robot, {cfg.servo}, {m['mass'] * 1e3:.0f} g (payload "
          f"{params.payload_g:.0f} g of it), {seconds:g} s; drives L {left * rpm:.1f} rpm, R "
          f"{right * rpm:.1f} rpm; open loop, right servo {params.servo_mismatch * 100:.0f} % slow")
    print(f"  walking   speed {m['speed']:.1f} mm/s along the start heading, lateral "
          f"{m['lateral']:.1f} mm, heading drift {m['heading_drift']:.1f} deg "
          f"({m['yaw_rate']:.2f} deg/s)")
    # One stride: MuJoCo's. The kinematic no-slip stride only says how much of it the feet
    # slipped away (a spin has no stride).
    if m["drives_oppose"] or math.isnan(m["stride"]):
        print(f"            {m['revolutions_abs']:.2f} crank revolutions (the drives opposed: "
              f"a turn, no stride)")
    else:
        slip = 1.0 - m["stride"] / kin["stride"] if abs(kin["stride"]) > 1e-9 else float("nan")
        print(f"            {m['revolutions']:.2f} crank revolutions, stride {m['stride']:.1f} "
              f"mm/rev: the feet slip {slip * 100:.0f} % of the {kin['stride']:.0f} mm the "
              f"kinematics would stride without slip")
    print(f"  body      height {m['height']:.1f} mm, bob {m['bob']:.1f} mm/rev (kinematic "
          f"{kin['bob']:.1f}), pitch range {m['pitch_range']:.1f} deg, roll range "
          f"{m['roll_range']:.1f} deg, max tilt {m['max_tilt']:.1f} deg")
    fell = "no"
    if m["fell"]:
        fell = "YES"
        if m.get("fell_at_s") is not None:
            fell += f" ({m.get('fell_axis') or 'tilted'} at {m['fell_at_s']:.1f} s into the run)"
    print(f"            fell over: {fell}; something other than a foot "
          f"on the floor {m['body_contact'] * 100:.0f} % of the time"
          + ("" if m["walks"] else "; DOES NOT WALK (under 5 mm a revolution, or its body down)"))
    print(f"  sides     crank L - R {m['side_phase']:.1f} deg at the end, {m['side_phase_max']:.1f}"
          f" deg apart at most")
    for name, d in m["torque"].items():
        print(f"  {name:9s} torque peak {d['peak']:.3f} N·m ({d['peak_fraction'] * 100:.0f} % of "
              f"the {d['limit']:.2f} stall), mean {d['mean']:.3f}, rms {d['rms']:.3f}; "
              f"saturated {d['saturated'] * 100:.1f} %")
        rated = (f"{d['mean_over_rated'] * 100:.0f} % of the {d['rated']:.2f} N·m rated"
                 if d["rated"] else "no rated torque in the catalog")
        print(f"            mean load {rated}; on the speed-torque line "
              f"{d['at_envelope'] * 100:.0f} % of the time motoring, speed droop "
              f"{d['speed_droop'] * 100:.1f} %; under the mean load a DC servo turns "
              f"{d['speed_under_load'] * rpm:.1f} rpm at full voltage; power {d['power']:.2f} W")
    print(f"  feet      {m['feet_down']:.2f} down on average (none {m['airborne'] * 100:.0f} %, "
          f"a side on fewer than two {m['side_support_low'] * 100:.0f} % of the time); slip in "
          f"contact mean {m['slip']:.1f} mm/s, max {m['slip_max']:.0f} mm/s")
    airborne = ", ".join(f"{a * 100:.0f}" for a in m["airborne_per_rev"][:8])
    print(f"  loads     airborne per revolution {airborne} %; base vertical acceleration peak "
          f"{m['accel_z_peak_g']:.1f} g; a foot's normal force peak {m['foot_force_peak']:.1f} N; "
          f"pin (loop) in-plane force 99.9th pct {m['loop_force_p999']:.0f} N, peak "
          f"{m['loop_force_peak']:.0f} N (the design load of a 3 mm pin and b1)")
    limit = m.get("torque_limit_recommended")
    jam = ("" if not limit else
           f"; set the servo's torque limit to {limit:.2f} N·m "
           f"({limit / m['torque_limit'] * 100:.0f} % of stall) so a jam puts "
           f"{m['joint_moment_at_limit']:.2f} N·m on it")
    cap = ("" if not m.get("crank_capacity_nm") else
           f", its weakest element ({m['crank_weakest']}) {m['crank_capacity_nm']:.2f}")
    print(f"  crank     a crankpin joint carries {m['joint_moment_peak']:.2f} N·m at the torque "
          f"peak (chord / radius {m['joint_moment_factor']:.1f} x){cap}{jam}")
    print(f"  model     loop closure error max {m['loop_error']:.3f} mm, deepest floor "
          f"contact {m['penetration']:.2f} mm")
    print("  contact   the support numbers (feet down, none, a side on fewer than two) and "
          "the torque peaks follow the unvalidated contact softness (SimParams.contact_solref"
          " 10 ms; --contact-sweep shows the 5-20 ms range); speed and mean torque don't; "
          "the torque peak is a range over the servo's unmeasured loop stiffness and rotor "
          "inertia (0.33-0.60 and 0.16-0.89 N·m on the Klann quad)")


SWEEP_SOLREF_MS = (5.0, 20.0)


def _contact_sweep(cfg: BuildConfig, params: SimParams, controls, seconds: float, skip: float,
                   base: dict) -> dict:
    """:func:`walk_metrics` at contact ``solref`` :data:`SWEEP_SOLREF_MS` beside ``base``'s
    (the run just made): per metric its min and max over the three softnesses."""
    runs = [base]
    for ms in SWEEP_SOLREF_MS:
        soft = SimParams(**{**params.__dict__, "contact_solref": (ms * 1e-3, 1.0)})
        runs.append(walk_metrics(simulate(cfg, controls, seconds, params=soft), skip=skip))
    keys = ("speed", "feet_down", "airborne", "side_support_low", "slip", "penetration",
            "torque_peak")
    return {k: [min(r[k] for r in runs), max(r[k] for r in runs)] for k in keys}


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
    xml_in, meta_in = args.model or (None, None)
    controls = [(0.0, 0.0, 0.0), (args.settle, left, right)]
    result = simulate(cfg, controls, args.settle + args.seconds, params=params,
                      model_xml=xml_in, model_meta=meta_in)
    m = walk_metrics(result, skip=args.settle + args.skip)
    kin = kinematic_gait(cfg)
    cmp = compare_with_walk(m, cfg) if xml_in is None else None
    sweep = None
    if args.contact_sweep and xml_in is None:
        sweep = _contact_sweep(cfg, params, controls, args.settle + args.seconds,
                               args.settle + args.skip, m)
    if args.json:
        print(json.dumps({"metrics": m, "kinematic": kin, "comparison": cmp,
                          "contact_sweep": sweep}, indent=1))
    else:
        _report(cfg, m, kin, left, right, args.seconds, params)
        if sweep:
            r = sweep
            print(f"            over solref 5-20 ms: speed {r['speed'][0]:.0f}-{r['speed'][1]:.0f}"
                  f" mm/s, feet down {r['feet_down'][0]:.2f}-{r['feet_down'][1]:.2f}, none "
                  f"{r['airborne'][0] * 100:.0f}-{r['airborne'][1] * 100:.0f} %, a side on "
                  f"fewer than two {r['side_support_low'][0] * 100:.0f}-"
                  f"{r['side_support_low'][1] * 100:.0f} %, penetration "
                  f"{r['penetration'][0]:.1f}-{r['penetration'][1]:.1f} mm, torque peak "
                  f"{r['torque_peak'][0]:.2f}-{r['torque_peak'][1]:.2f} N·m")
        if cmp:
            q, s = cmp["quasi_static"], cmp["mujoco"]
            full = (f"MuJoCo at full speed {s['speed_mm_s']:.0f} mm/s (x{cmp['speed_ratio']:.2f})"
                    if not math.isnan(s["speed_mm_s"]) else "no full-speed scaling for a turn")
            print(f"  vs model  quasi-static {q['speed_mm_s']:.0f} mm/s, {q['stride_mm']:.0f} "
                  f"mm/rev, bob {q['bob_mm']:.1f} mm, {q['mean_contacts']:.1f} feet, margin "
                  f"{q['min_margin_mm']:.0f} mm (no slip assumed); {full}, "
                  f"{s['feet_down']:.1f} feet"
                  + (f"; flags: {', '.join(cmp['flags'])}" if cmp["flags"] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())

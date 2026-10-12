"""A Klipper ``printer.cfg`` for an arm: each joint a ``manual_stepper``.

Klipper has no arm kinematics, so each joint is a ``manual_stepper`` moved in degrees of
the joint (``rotation_distance``: the joint's degrees per turn of its motor, 360 over the
transmission's ratio: :class:`.mapping.Axis`), and the host (:mod:`.host`) does the
kinematics. A TMC2209 per motor (UART), its current a share of the motor's rating; the
turntable's endstop on an endstop input; the vacuum pump and its release valve on two
outputs. A board's pins are its own: every pin is left as a ``<PLACEHOLDER>`` to fill from
the board's pinout (BTT SKR Mini E3, Octopus, ...), and the config won't load until they are.
"""

from __future__ import annotations

import math

from spiderpig.control.mapping import Axis

RUN_CURRENT = 0.7       # a TMC2209's run current as a share of the motor's rated current
HOLD_CURRENT = 0.5
SPEED_RPM = 300.0       # the motors' cruising speed (the joints' velocity: through the ratio)
ACCEL_S = 0.25          # seconds to reach it


def printer_cfg(axes: dict[str, Axis], name: str = "arm") -> str:
    """The config text (module docstring)."""
    from spiderpig import actuators

    lines = [f"# {name}: written by spiderpig control klipper (spiderpig.control.klipper).",
             "# Fill every <PLACEHOLDER> from your board's pinout before loading it.", "",
             "[mcu]", "serial: <MCU_SERIAL>  # ls /dev/serial/by-id/*", "",
             "[printer]", "kinematics: none", "max_velocity: 1", "max_accel: 1", ""]
    for n, ax in axes.items():
        motor = actuators.STEPPERS.get(ax.motor)
        vel = SPEED_RPM / 60.0 * ax.deg_per_motor_rev            # joint deg/s
        up = n.upper()
        lines += [f"[manual_stepper {n}]",
                  f"step_pin: <{up}_STEP_PIN>", f"dir_pin: <{up}_DIR_PIN>"
                  + ("  # flip (!) if a jog turns the joint the wrong way"),
                  f"enable_pin: !<{up}_ENABLE_PIN>",
                  f"microsteps: {ax.microsteps}",
                  f"full_steps_per_rotation: {ax.steps_per_rev}",
                  f"rotation_distance: {ax.deg_per_motor_rev:.6f}"
                  f"  # joint degrees per motor turn: 360 / {ax.ratio:.6g}",
                  f"velocity: {vel:.3f}", f"accel: {vel / ACCEL_S:.3f}"]
        if n == "q1":
            lines.append("endstop_pin: ^<Q1_ENDSTOP_PIN>  # the optical endstop's signal")
        lines.append("")
        if motor is not None:
            lines += [f"[tmc2209 manual_stepper {n}]", f"uart_pin: <{up}_UART_PIN>",
                      f"run_current: {RUN_CURRENT * motor.rated_a:.2f}",
                      f"hold_current: {HOLD_CURRENT * motor.rated_a:.2f}",
                      "stealthchop_threshold: 0  # spreadCycle: holds best under load", ""]
    limits = ", ".join(f"{n} {math.degrees(a.limits[0]):.1f} to {math.degrees(a.limits[1]):.1f}"
                       for n, a in axes.items())
    lines += ["[output_pin vacuum_pump]", "pin: <PUMP_PIN>", "",
              "[output_pin vacuum_release]", "pin: <VALVE_PIN>", "",
              "[idle_timeout]",
              "# never disable the motors when idle (Klipper's default M84 drops the arm and "
              "loses its zeros)",
              "timeout: 3600", "gcode:",
              "  {action_respond_info(\"idle: the motors stay on\")}", "",
              "[gcode_macro ARM_LIMITS]", "description: the joints' ranges (deg)",
              "gcode:", f"  {{action_respond_info(\"{limits}\")}}", ""]
    return "\n".join(lines)

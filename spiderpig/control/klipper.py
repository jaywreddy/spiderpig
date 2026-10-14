"""A Klipper ``printer.cfg`` for an arm: each joint a ``manual_stepper``.

Klipper has no arm kinematics, so each joint is a ``manual_stepper`` moved in degrees of
the joint (``rotation_distance``: the joint's degrees per turn of its motor, 360 over the
transmission's ratio: :class:`.mapping.Axis`), and the host (:mod:`.host`) does the
kinematics. A TMC2209 per motor on the board's UART (its ``uart_address``), its current a
share of the motor's rating; the vacuum pump and its release valve on two outputs.

**Homing** follows each joint's sensor (:func:`.homing.method`): a switch is the joint's
``endstop_pin`` on its port's endstop input; a **stall** (an ``angle_sensor`` ``stall``,
the turntable preset's ``hard_stop``) is Klipper's sensorless homing: ``endstop_pin:
tmc2209_<joint>:virtual_endstop`` (Klipper names the driver's chip ``tmc2209_`` and the
section's last word), the driver's ``diag_pin`` (the port's DIAG jumper fitted),
``driver_SGTHRS`` a starting threshold to tune, and the host lowering the driver's current
(:data:`HOME_CURRENT`) for the move (:func:`.host.home_gcode`). Klipper turns StealthChop
on for a TMC2209's homing move and back after, so the run's spreadCycle stays.

**The pins** are the role ``controller``'s (:mod:`actuators.controllers`: the SKR Pico's,
the Pi on its UART); with no controller (``none``: your own board) every pin is left a
``<PLACEHOLDER>`` to fill from the board's pinout, and the config won't load until they are.
"""

from __future__ import annotations

import math

from spiderpig.control.homing import YAW_SENSOR, method
from spiderpig.control.mapping import Axis

RUN_CURRENT = 0.7       # a TMC2209's run current as a share of the motor's rated current
HOLD_CURRENT = 0.5
HOME_CURRENT = 0.4      # ... while it homes on a stall: a softer hit, a more sensitive stall
SGTHRS = 100            # StallGuard's threshold to start from (0-255, higher stalls sooner)
SPEED_RPM = 300.0       # the motors' cruising speed (the joints' velocity: through the ratio)
ACCEL_S = 0.25          # seconds to reach it


def currents(axis: Axis) -> tuple[float, float, float] | None:
    """``(run, hold, home)`` amps of ``axis``'s motor (``None``: not a catalogued motor)."""
    from spiderpig import actuators

    motor = actuators.STEPPERS.get(axis.motor)
    if motor is None:
        return None
    a = motor.rated_a
    return round(RUN_CURRENT * a, 2), round(HOLD_CURRENT * a, 2), round(HOME_CURRENT * a, 2)


def printer_cfg(axes: dict[str, Axis], name: str = "arm", controller=None,
                sensors: dict[str, str] | None = None) -> str:
    """The config text (module docstring). ``controller``: a key of
    :data:`actuators.controllers.CONTROLLERS`, one, or ``None`` / ``""`` (placeholders);
    ``sensors``: each joint's homing sensor (:func:`.homing.sensors_of`; by default the
    turntable's optical endstop and no other)."""
    from spiderpig.actuators import controllers

    ctl = controllers.get(controller) if isinstance(controller, str) and controller \
        else controller or None
    sensors = {"q1": YAW_SENSOR} if sensors is None else sensors
    lines = [f"# {name}: written by spiderpig control klipper (spiderpig.control.klipper)."]
    if ctl is None:
        lines += ["# Fill every <PLACEHOLDER> from your board's pinout before loading it.", "",
                  "[mcu]", "serial: <MCU_SERIAL>  # ls /dev/serial/by-id/*", ""]
    else:
        lines += [f"# The controller: {ctl.label} ({ctl.key}, spiderpig.actuators.controllers).",
                  "", "[mcu]", f"serial: {ctl.serial}"
                  + ("  # the Pi's UART: enable_uart=1 and dtoverlay=disable-bt in "
                     "/boot/firmware/config.txt" if "ttyAMA" in ctl.serial else "")]
        if ctl.restart_method:
            lines.append(f"restart_method: {ctl.restart_method}")
        lines.append("")
    lines += ["[printer]", "kinematics: none", "max_velocity: 1", "max_accel: 1", ""]
    for n, ax in axes.items():
        lines += _axis(n, ax, ctl, method(sensors.get(n, "")))
    limits = ", ".join(f"{n} {math.degrees(a.limits[0]):.1f} to {math.degrees(a.limits[1]):.1f}"
                       for n, a in axes.items())
    for out, label in (("vacuum_pump", "PUMP"), ("vacuum_release", "VALVE")):
        lines += [f"[output_pin {out}]", f"pin: {ctl.output(out) if ctl else f'<{label}_PIN>'}"
                  + ("  # a 1N5819 across its leads" if ctl else ""), ""]
    if ctl is not None and "driver_fan" in dict(ctl.outputs):
        fan = ctl.output("driver_fan")
        lines += (["[controller_fan driver_fan]", f"pin: {fan}", ""] if fan else
                  ["# driver_fan: on the supply's input, always on (no pin)", ""])
    lines += ["[idle_timeout]",
              "# never disable the motors when idle (Klipper's default M84 drops the arm and "
              "loses its zeros)",
              "timeout: 3600", "gcode:",
              "  {action_respond_info(\"idle: the motors stay on\")}", "",
              "[gcode_macro ARM_LIMITS]", "description: the joints' ranges (deg)",
              "gcode:", f"  {{action_respond_info(\"{limits}\")}}", ""]
    return "\n".join(lines)


def _axis(n: str, ax: Axis, ctl, how: str) -> list[str]:
    """Joint ``n``'s ``manual_stepper`` and its driver's section; ``how`` it homes."""
    vel = SPEED_RPM / 60.0 * ax.deg_per_motor_rev            # joint deg/s
    up = n.upper()
    pins = ctl.pins(n) if ctl is not None else None
    step, dir_, enable, diag = ((pins.step, pins.dir, pins.enable, pins.diag) if pins else
                                (f"<{up}_STEP_PIN>", f"<{up}_DIR_PIN>", f"!<{up}_ENABLE_PIN>",
                                 f"^<{up}_DIAG_PIN>"))
    port = f" ({pins.port})" if pins else ""
    lines = [f"[manual_stepper {n}]" + (f"  # the board's {pins.port} port" if pins else ""),
             f"step_pin: {step}",
             f"dir_pin: {dir_}  # flip (!) if a jog turns the joint the wrong way",
             f"enable_pin: {enable}",
             f"microsteps: {ax.microsteps}",
             f"full_steps_per_rotation: {ax.steps_per_rev}",
             f"rotation_distance: {ax.deg_per_motor_rev:.6f}"
             f"  # joint degrees per motor turn: 360 / {ax.ratio:.6g}",
             f"velocity: {vel:.3f}", f"accel: {vel / ACCEL_S:.3f}"]
    if how == "endstop":
        stop = pins.diag if pins else f"^<{up}_ENDSTOP_PIN>"
        lines.append(f"endstop_pin: {stop}  # the switch's signal on the endstop input{port}"
                     + (" (its DIAG jumper off)" if pins else ""))
    elif how == "stall":
        lines.append(f"endstop_pin: tmc2209_{n}:virtual_endstop  # the driver's stall "
                     "(sensorless homing on the hard stop)")
    lines.append("")
    amps = currents(ax)
    if amps is None:
        return lines
    run, hold, home = amps
    if pins:
        lines += [f"[tmc2209 manual_stepper {n}]", f"uart_pin: {ctl.uart_rx}",
                  f"tx_pin: {ctl.uart_tx}", f"uart_address: {pins.uart_address}"]
    else:
        lines += [f"[tmc2209 manual_stepper {n}]", f"uart_pin: <{up}_UART_PIN>"]
    lines += [f"run_current: {run:.2f}", f"hold_current: {hold:.2f}",
              "stealthchop_threshold: 0  # spreadCycle: holds best under load"]
    if how == "stall":
        lines += [f"diag_pin: {diag}  # the driver's DIAG{port}"
                  + (": its jumper fitted" if pins else ""),
                  f"driver_SGTHRS: {SGTHRS}  # a start: tune it (0-255, higher stalls sooner) "
                  f"with SET_TMC_FIELD STEPPER={n} FIELD=SGTHRS VALUE=..., homing at "
                  f"{home:.2f} A (SET_TMC_CURRENT, spiderpig.control.host)"]
    lines.append("")
    return lines

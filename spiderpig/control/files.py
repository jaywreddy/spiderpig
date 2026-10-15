"""A design's controller files, one dispatch on what drives it.

    from spiderpig.control import files
    files.write(config, Path("build/control"))      # every file its motors need
    files.write(config, out, side=design.side)      # ... with its planned side (no re-plan)

What drives a design (:func:`actuators.controllers.motors_of`: serial-bus servos where the
role ``servo`` is a slot of its kind, else steppers) picks its writer in :data:`WRITERS`:

* ``servo`` (a crank's servos: a walker's, a mechanism's): ``servos.json``, the bus
  setup :func:`.servos.setup` makes (IDs, baud, mode, torque limit, each crank's zero);
* ``actuator`` (an arm's inputs, its steppers): ``printer.cfg``, the Klipper configuration
  :func:`.klipper.printer_cfg` makes of its axes (:func:`.mapping.axes`) and their sensors;

and either one ``HOMING.md``: the numbered steps that zero it (:func:`.servos.homing_steps`,
:func:`.homing.plan`). ``spiderpig control servo|klipper|homing`` print and write the same
(:mod:`.cli`); ``api.export(design, ["control"])`` writes them into ``<out>/control/``.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path

HOMING = "HOMING.md"


def servo_json(config) -> dict:
    """``servos.json``'s document (:func:`.servos.setup`; ``ValueError`` when the servo
    names no bus protocol spiderpig knows)."""
    from spiderpig.control import servos

    return servos.setup(config)


def servo_homing(config) -> list[tuple[str, str]]:
    """``(servo, action)``: each servo's setup and its crank's one-time zero."""
    from spiderpig.control import servos

    return servos.homing_steps(config)


def stepper_axes(config, side=None) -> dict:
    """The arm's motor-driven joints (:func:`.mapping.axes`); ``side``: its planned side
    design when the caller has it (else planned here)."""
    from spiderpig.control.mapping import axes

    return axes(config, side)


def printer_cfg(config, axes: dict) -> str:
    """Klipper's ``printer.cfg`` for the arm's ``axes`` (its controller's pins, or
    ``<PLACEHOLDER>`` ones for your own board)."""
    from spiderpig.control import homing, klipper

    return klipper.printer_cfg(axes, config.linkage, config.role("controller"),
                               homing.sensors_of(config, axes))


def stepper_homing(config, axes: dict) -> list[tuple[str, str]]:
    """``(joint, action)``: the arm's homing sequence (:func:`.homing.plan`)."""
    from spiderpig.control import homing

    return [(s.axis, s.action) for s in homing.plan(axes, homing.sensors_of(config, axes))]


def homing_markdown(config, steps: list[tuple[str, str]]) -> str:
    """The homing steps as ``HOMING.md``."""
    lines = [f"# Homing: {config.linkage} {config.module}", "",
             "Once after assembly (and after a motor or horn comes off), in this order.", ""]
    lines += [f"{i}. **{who}**: {what}" for i, (who, what) in enumerate(steps, 1)]
    return "\n".join(lines) + "\n"


def _servo_files(config, out: Path, side=None) -> list[Path]:
    doc = servo_json(config)
    path = out / "servos.json"
    path.write_text(json.dumps(doc, indent=1) + "\n")
    homing = out / HOMING
    homing.write_text(homing_markdown(config, servo_homing(config)))
    return [path, homing]


def _stepper_files(config, out: Path, side=None) -> list[Path]:
    axes = stepper_axes(config, side)
    path = out / "printer.cfg"
    path.write_text(printer_cfg(config, axes))
    homing = out / HOMING
    homing.write_text(homing_markdown(config, stepper_homing(config, axes)))
    return [path, homing]


WRITERS: dict[str, Callable[..., list[Path]]] = {
    "servo": _servo_files,
    "actuator": _stepper_files,
}
"""What drives a design (:func:`actuators.controllers.motors_of`) -> its files' writer."""

FILES = {"servo": ("servos.json", HOMING), "actuator": ("printer.cfg", HOMING)}
"""The files each writer makes, by what drives the design."""


def motors(config) -> str:
    """What drives ``config``'s design: a key of :data:`WRITERS`."""
    from spiderpig.actuators.controllers import motors_of

    return motors_of(config)


def write(config, out: Path, side=None) -> list[Path]:
    """Every controller file of ``config``'s design into ``out`` (created; module
    docstring), the paths written. ``side``: its planned side design, when the caller has
    it. ``ValueError`` when its motors can't be set up (a servo with no known protocol)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for name in {f for fs in FILES.values() for f in fs}:   # no other design's files left
        (out / name).unlink(missing_ok=True)
    return WRITERS[motors(config)](config, out, side)

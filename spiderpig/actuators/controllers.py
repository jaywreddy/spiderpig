"""Controller boards: what drives an arm's steppers and runs its host (the role
``controller``, :mod:`spiderpig.roles`; 2026-10-13, loop 2's electronics).

A :class:`Controller` is the boards and wiring an arm buys to run (``bom``: catalog keys,
:mod:`hardware.arm_electronics`), and the pins Klipper drives (:mod:`control.klipper`): each
joint's motor port (step, dir, enable, its TMC2209's UART address and DIAG pin, the port's
endstop input: one pin with the DIAG on a jumper), the drivers' shared UART, the outputs
(the vacuum pump, its release valve, the drivers' fan) and how the host reaches the MCU.

The joints take the ports in order (``q1`` the turntable on X, ``q2`` the upper arm on Y,
``q3`` the forearm on Z, ``q4`` a wrist on E), as in the pin map of the notes of 2026-10-13
(BTT's SKR-Pico manual and schematic, Klipper's ``generic-bigtreetech-skr-pico-v1.0.cfg``).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AxisPins:
    """One motor port: Klipper pin names (``!`` inverted, ``^`` pulled up)."""

    port: str               # the board's label ("X")
    step: str
    dir: str
    enable: str
    uart_address: int
    diag: str               # the driver's DIAG (stall) on the port's endstop input


@dataclass(frozen=True)
class Controller:
    """An arm's controller (module docstring). ``axes``: ``(joint, pins)``; ``outputs``:
    ``(name, pin)``, a pin ``""`` wired to the supply (always on); ``uart_rx`` / ``uart_tx``:
    the drivers' shared UART; ``serial`` / ``restart_method``: Klipper's ``[mcu]``."""

    key: str
    label: str
    bom: tuple[tuple[str, float], ...]
    axes: tuple[tuple[str, AxisPins], ...]
    outputs: tuple[tuple[str, str], ...]
    uart_rx: str
    uart_tx: str
    serial: str
    restart_method: str = ""
    driver: str = "tmc2209"
    source: str = ""

    def pins(self, joint: str) -> AxisPins:
        """The port joint ``joint`` drives (:class:`ValueError` past the board's ports)."""
        for n, p in self.axes:
            if n == joint:
                return p
        raise ValueError(f"controller {self.key}: no motor port for joint {joint!r} "
                         f"(it has {', '.join(n for n, _ in self.axes)})")

    def output(self, name: str) -> str:
        return dict(self.outputs).get(name, "")


CONTROLLERS: dict[str, Controller] = {c.key: c for c in (
    Controller(
        "skr_pico_pi_zero", "BTT SKR Pico V1.0 (4x TMC2209 UART) and a Raspberry Pi Zero 2 W "
        "on its UART, a 12 V 5 A brick",
        (("skr_pico_v1", 1.0), ("pi_zero_2w_header", 1.0), ("microsd_32gb", 1.0),
         ("psu_12v_5a_brick", 1.0), ("dc_jack_terminal", 1.0), ("stepper_cable_1m", 3.0),
         ("diode_1n5819", 2.0), ("fan_4010_12v", 1.0)),
        (("q1", AxisPins("X", "gpio11", "!gpio10", "!gpio12", 0, "^gpio4")),
         ("q2", AxisPins("Y", "gpio6", "!gpio5", "!gpio7", 2, "^gpio3")),
         ("q3", AxisPins("Z", "gpio19", "gpio28", "!gpio2", 1, "^gpio25")),
         ("q4", AxisPins("E", "gpio14", "!gpio13", "!gpio15", 3, "^gpio16"))),
        (("vacuum_pump", "gpio23"), ("vacuum_release", "gpio21"), ("driver_fan", "")),
        "gpio9", "gpio8", "/dev/ttyAMA0", "command",
        source="BTT SKR-Pico manual, pin map and schematic (github.com/bigtreetech/SKR-Pico); "
               "Klipper's config/generic-bigtreetech-skr-pico-v1.0.cfg; the pump on HE "
               "(gpio23), the valve on HB (gpio21), a 1N5819 across each; the drivers' fan "
               "on the 12 V input"),
)}
DEFAULT = "skr_pico_pi_zero"


def get(key: str) -> Controller:
    try:
        return CONTROLLERS[key]
    except KeyError:
        raise KeyError(f"unknown controller {key!r}; have {sorted(CONTROLLERS)}") from None

"""Controllers: the electronics that drive a robot's motors (the role ``controller``,
:mod:`spiderpig.roles`; an arm's since 2026-10-13, loop 2's electronics; every robot's
since 2026-10-14).

A :class:`Controller` is the boards and wiring a robot buys to run (``bom``: catalog keys)
and what it drives (``drives``, a capability: :func:`of` matches it to the design's motors,
:func:`motors_of`, never to a kind's name):

* ``"stepper"``: step/dir drivers under Klipper (an arm's, :mod:`hardware.arm_electronics`),
  and the pins Klipper drives (:mod:`control.klipper`): each joint's motor port (step, dir,
  enable, its TMC2209's UART address and DIAG pin, the port's endstop input: one pin with
  the DIAG on a jumper), the drivers' shared UART, the outputs (the vacuum pump, its release
  valve, the drivers' fan) and how the host reaches the MCU. The joints take the ports in
  order (``q1`` the turntable on X, ``q2`` the upper arm on Y, ``q3`` the forearm on Z,
  ``q4`` a wrist on E), as in the pin map of the notes of 2026-10-13 (BTT's SKR-Pico manual
  and schematic, Klipper's ``generic-bigtreetech-skr-pico-v1.0.cfg``). Its parts are BOM
  lines of the robot (``mount=""``).
* ``"bus_servo"``: servos on a half-duplex serial bus (a crank's servo,
  :mod:`hardware.electronics`): the ``protocol`` it speaks, its ``bus_ports`` and ``baud``
  (what :mod:`control.servos` writes into the servos). ``mount="deck"``: its parts are
  bodies of a walker, laid out by the electronics deck (:mod:`construction.deck`, drawn for
  :data:`DECK_KIT`), so the BOM counts them from the bodies, not from ``bom``.

The key :data:`roles.NONE` is your own electronics: nothing bought, a walker's deck bare
(:func:`construction.deck.deck_parts`), an arm's Klipper pins left to fill.
"""

from __future__ import annotations

from dataclasses import dataclass

DRIVES = ("stepper", "bus_servo")
"""What a controller drives: step/dir stepper drivers, or servos on a serial bus."""

PROTOCOLS = ("feetech_sts", "dynamixel_2")
"""The serial-bus servo protocols (a servo's :attr:`servos.spec.ServoSpec.protocol`)."""


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
    """A robot's controller (module docstring). A stepper board's: ``axes``: ``(joint,
    pins)``; ``outputs``: ``(name, pin)``, a pin ``""`` wired to the supply (always on);
    ``uart_rx`` / ``uart_tx``: the drivers' shared UART; ``serial`` / ``restart_method``:
    Klipper's ``[mcu]``. A bus servo board's: ``protocol``, ``bus_ports``, ``baud``."""

    key: str
    label: str
    bom: tuple[tuple[str, float], ...]
    axes: tuple[tuple[str, AxisPins], ...] = ()
    outputs: tuple[tuple[str, str], ...] = ()
    uart_rx: str = ""
    uart_tx: str = ""
    serial: str = ""
    restart_method: str = ""
    driver: str = "tmc2209"
    source: str = ""
    drives: str = "stepper"         # what it drives (DRIVES), matched to the design's motors
    protocol: str = ""              # a bus servo board's protocol (PROTOCOLS)
    bus_ports: int = 0              # its servo bus sockets
    baud: int = 0                   # the bus's speed (bit/s)
    mount: str = ""                 # "deck": its parts are the walker deck's bodies

    def __post_init__(self):
        if self.drives not in DRIVES:
            raise ValueError(f"controller {self.key}: drives is one of {DRIVES}")
        if self.drives == "bus_servo" and self.protocol not in PROTOCOLS:
            raise ValueError(f"controller {self.key}: a bus servo board names its protocol "
                             f"({', '.join(PROTOCOLS)})")

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
    Controller(
        "esp32_bus_servo", "Waveshare Servo Driver with ESP32 (two serial bus servo ports) "
        "on the electronics deck, a 2S 450 mAh HV LiPo with its USB-C charger, protection "
        "board, switch and harness",
        (("esp32_servo_driver", 1.0), ("m25_nylon_standoff_mf_6", 4.0),
         ("m25_nylon_nut", 4.0), ("m25_nylon_screw_6", 4.0), ("lipo_2s_450", 1.0),
         ("lipo_strap_10mm", 1.0), ("ip2326_charger", 1.0), ("bms_hx_2s_jh20", 1.0),
         ("foam_tape", 1.0), ("toggle_mts102", 1.0), ("resistor_100k", 1.0),
         ("resistor_33k", 1.0), ("xt30_pigtail_pair", 1.0), ("dc_plug_5521_pigtail", 1.0)),
        drives="bus_servo", protocol="feetech_sts", bus_ports=2, baud=1_000_000,
        mount="deck",
        source="option 1 of the electronics review (2026-10-03, hardware.electronics); "
               "Waveshare's wiki: two bus servo ports, Feetech SCS/STS servos"),
)}
DEFAULT = "skr_pico_pi_zero"
"""A stepper-driven design's controller when it names none."""
DECK_KIT = "esp32_bus_servo"
"""A bus-servo design's controller when it names none, and the one the electronics deck is
drawn for (:mod:`construction.deck`). The walkers carried it, unnamed, before the role
reached them: it is the role's ``implicit`` key (:class:`roles.Role`): never move it."""
DEFAULTS = {"stepper": DEFAULT, "bus_servo": DECK_KIT}


def get(key: str) -> Controller:
    try:
        return CONTROLLERS[key]
    except KeyError:
        raise KeyError(f"unknown controller {key!r}; have {sorted(CONTROLLERS)}") from None


def motors_of(config) -> str:
    """What drives ``config``'s design (:data:`DRIVES`): servos on a serial bus where the
    role ``servo`` is a slot of it (a crank's drive), else steppers (the role ``actuator``,
    an arm's inputs)."""
    from spiderpig import roles

    return "bus_servo" if roles.applies("servo", config.lk.kind) else "stepper"


def default_key(config) -> str:
    """The role ``controller``'s default for ``config``: the one that drives its motors
    (:data:`DEFAULTS`)."""
    return DEFAULTS[motors_of(config)]


def of(config) -> Controller | None:
    """The controller ``config`` names (``None``: :data:`roles.NONE`, your own), checked
    against what drives its design: :class:`ValueError` when it can't drive them. A design
    the role isn't a slot of (a mechanism: one side, no robot round it) has none."""
    from spiderpig import roles

    if not roles.applies("controller", config.lk.kind):
        return None
    key = config.role("controller")
    if not key:
        return None
    ctl = get(key)
    need = motors_of(config)
    if ctl.drives != need:
        fit = sorted(k for k, c in CONTROLLERS.items() if c.drives == need)
        raise ValueError(f"controller {key!r} drives {ctl.drives.replace('_', ' ')}s and "
                         f"{config.linkage}'s motors are {need.replace('_', ' ')}s: use "
                         f"{' or '.join([*fit, 'none'])}")
    return ctl


def protocol_note(config, ctl: Controller | None) -> str:
    """Why ``ctl`` can't talk to ``config``'s servos (another bus protocol), or ``""``."""
    if ctl is None or not ctl.protocol:
        return ""
    from spiderpig import servos

    spoken = servos.get(config.servo).protocol
    if not spoken or spoken == ctl.protocol:
        return ""
    return (f"the {ctl.key} board speaks {ctl.protocol}, the {config.servo} {spoken}: drive "
            "it from an adapter of its own (the role controller none)")

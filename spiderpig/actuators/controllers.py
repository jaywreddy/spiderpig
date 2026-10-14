"""Controllers: the electronics that drive a design's motors (the role ``controller``,
:mod:`spiderpig.roles`; an arm's since 2026-10-13, loop 2's electronics; a walker's since
2026-10-14; every design with motors, a mechanism's too, since 2026-10-14's second round).

A :class:`Controller` is the boards and wiring a design buys to run (``bom``: catalog keys)
and what it drives (``drives``: the motor role it serves, :data:`DRIVES`, matched by
:func:`of` to the design's own, :func:`motors_of` = :func:`roles.motor_role`, never to a
kind's name):

* ``"actuator"``: steppers on step/dir drivers under Klipper (a ranged input's,
  :mod:`hardware.arm_electronics`), and the pins Klipper drives (:mod:`control.klipper`):
  each joint's motor port (step, dir, enable, its TMC2209's UART address and DIAG pin, the
  port's endstop input: one pin with the DIAG on a jumper), the drivers' shared UART, the
  outputs (the vacuum pump, its release valve, the drivers' fan) and how the host reaches
  the MCU. The joints take the ports in order (``q1`` the turntable on X, ``q2`` the upper
  arm on Y, ``q3`` the forearm on Z, ``q4`` a wrist on E), as in the pin map of the notes
  of 2026-10-13 (BTT's SKR-Pico manual and schematic, Klipper's
  ``generic-bigtreetech-skr-pico-v1.0.cfg``). Its parts are BOM lines (``mount=""``).
* ``"servo"``: servos on a half-duplex serial bus (a crank's servo,
  :mod:`hardware.electronics`): the ``protocol`` it speaks, its ``bus_ports`` and ``baud``
  (what :mod:`control.servos` writes into the servos). ``mount="deck"``: on a walker its
  parts are bodies of the robot, laid out by the electronics deck (:mod:`construction.deck`,
  drawn for :data:`DECK_KIT`), so the BOM counts them from the bodies, not from ``bom``; a
  design with no deck (a mechanism: its side is the machine) buys them as lines
  (:func:`construction.robot.controller_lines`). A design's default is the board that
  speaks its servo's protocol (:data:`BUS_DEFAULTS`): the deck's ESP32 board for Feetech's
  STS servos, ROBOTIS's U2D2 for DYNAMIXEL's (bought as lines, a walker's deck bare).

The key :data:`roles.NONE` is your own electronics: nothing bought, a walker's deck bare
(:func:`construction.deck.deck_parts`), an arm's Klipper pins left to fill.

**Its wiring** is declared too (2026-10-14), so the assembly guide's wiring step and its
diagram (:mod:`guide.wiring`, :func:`construction.assembly.controller_ops`) follow whichever
controller a design names, never a kind: ``nodes`` say which bought part is which box of the
diagram (a pattern over its catalog key, else its name), ``wiring`` the links between them
(:class:`Wire`: the cables bought for each, or what makes it; a sentence for the guide when
both ends are in the robot), ``wiring_text`` the step's own sentences and ``wiring_extras``
the bought lines its parts list takes. What a walker carries of it, modelled in MuJoCo, is
:func:`carried_g` (a bare deck's: the parts the controller declares ``carried``).
"""

from __future__ import annotations

from dataclasses import dataclass

from spiderpig.hardware.electronics import HV_WARNING

DRIVES = ("servo", "actuator")
"""What a controller drives: the motor role it serves (:data:`roles.MOTOR_ROLES`): a crank's
serial-bus servos, or ranged inputs' steppers on step/dir drivers. One vocabulary with the
drive groups (:data:`construction.DRIVES`) and the roles (2026-10-14: ``"bus_servo"`` /
``"stepper"`` before)."""
DRIVE_WORDS = {"servo": "serial-bus servos", "actuator": "steppers"}
"""How a message names each drive's motors."""

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
class Wire:
    """One link of a controller's wiring diagram: from the box ``a`` to ``b`` (roles of
    :attr:`Controller.nodes`), the bought cables that make it (``cables``: a pattern over
    their catalog keys and names) or, none bought, what does (``via``), and the guide's
    sentence for it (``text``: said when both ends are in the robot)."""

    a: str
    b: str
    cables: str = ""
    via: str = ""
    text: str = ""


CABLE = r"cable|lead|pigtail|plug|harness|wire|extension"
"""What a bought part's key or name says when it is a cable, not a box of the diagram."""


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
    drives: str = "actuator"        # the motor role it drives (DRIVES): the design's own
    protocol: str = ""              # a bus servo board's protocol (PROTOCOLS)
    bus_ports: int = 0              # its servo bus sockets
    baud: int = 0                   # the bus's speed (bit/s)
    mount: str = ""                 # "deck": its parts are the walker deck's bodies
    nodes: tuple[tuple[str, str], ...] = ()     # (role, pattern): the diagram's boxes
    wiring: tuple[Wire, ...] = ()               # the links between them
    wiring_title: str = "Wiring"
    wiring_text: tuple[str, ...] = ()           # the guide's wiring step's sentences
    wiring_extras: tuple[str, ...] = ()         # the bought lines its parts list takes
    carried: tuple[str, ...] = ()   # the catalog keys of its parts a walker carries on a
    #                                 bare deck (:func:`carried_g`; "deck" mounts: modelled)

    def __post_init__(self):
        if self.drives not in DRIVES:
            raise ValueError(f"controller {self.key}: drives is one of {DRIVES}")
        if self.drives == "servo" and self.protocol not in PROTOCOLS:
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

    def role_of(self, key: str, name: str = "") -> str | None:
        """The diagram's box a bought part is (:attr:`nodes`, first match: its catalog key,
        else its name), or ``None``."""
        import re

        for text in (key, name):
            for role, pat in self.nodes:
                if text and re.search(pat, text, re.I):
                    return role
        return None

    def port_text(self, joints) -> str:
        """Which motor port each of ``joints`` takes (a stepper board's: its axes, in
        order), or ``""``."""
        ports = [f"{j} on {p.port}" for j, p in self.axes if j in set(joints)]
        return ", ".join(ports)


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
               "on the 12 V input",
        nodes=(("supply", r"^psu_"), ("jack", r"^dc_jack"), ("board", r"^skr_"),
               ("host", r"^pi_zero"), ("motor", r"^stepper_(?!cable)|17hs|nema"),
               ("camera", r"^camera_"), ("pump", r"vacuum_pump"), ("valve", r"valve"),
               ("fan", r"^fan_"), ("endstop", r"endstop|ee_sx|scl3300|inclinometer")),
        wiring=(Wire("supply", "jack", via="the brick's 5.5 x 2.1 mm plug"),
                Wire("jack", "board", via="12 V to the power terminal",
                     text="The 12 V brick's plug into the DC jack adapter, its screw "
                          "terminal to the SKR Pico's power input (+ to +: check before "
                          "plugging the brick in)."),
                Wire("board", "host", via="the Pi header: 5 V and the UART",
                     text="The Pi Zero 2 W onto the SKR Pico's Pi header: its 5 V and the "
                          "drivers' board's UART (Klipper's /dev/ttyAMA0)."),
                Wire("board", "motor", cables=r"stepper_cable",
                     text="Each stepper's cable from its motor to its port ({ports}); check "
                          "each motor's coil pairs against the cable's first."),
                Wire("board", "endstop", cables=r"endstop.*cable",
                     via="its port's endstop input",
                     text="Each joint's endstop on its port's endstop input (the port's DIAG "
                          "jumper off)."),
                Wire("board", "pump", cables=r"diode", via="HE (gpio23)",
                     text="The vacuum pump on HE (gpio23) and the release valve on HB "
                          "(gpio21), a 1N5819 across each (cathode to +12 V)."),
                Wire("board", "valve", cables=r"diode", via="HB (gpio21)"),
                Wire("board", "fan", via="the 12 V input", text="The fan over the drivers on "
                     "the 12 V input (always on)."),
                Wire("host", "camera", cables=r"csi",
                     text="The camera's ribbon (the Pi Zero's 22-pin cable) from the tool to "
                          "the Pi, looped with the motors' cables round the base.")),
        wiring_title="Controller and cables",
        wiring_text=("Flash Klipper onto the SKR Pico (its BOOT jumper, USB-C) and the Pi's "
                     "card (Raspberry Pi OS Lite, Klipper's host, Moonraker) before closing "
                     "anything up.",),
        wiring_extras=("skr_pico_v1", "pi_zero_2w_header", "microsd_32gb",
                       "psu_12v_5a_brick", "dc_jack_terminal", "stepper_cable_1m",
                       "diode_1n5819", "fan_4010_12v")),
    Controller(
        "esp32_bus_servo", "Waveshare Servo Driver with ESP32 (two serial bus servo ports) "
        "on the electronics deck, a 2S 450 mAh HV LiPo with its USB-C charger, protection "
        "board, switch and harness",
        (("esp32_servo_driver", 1.0), ("m25_nylon_standoff_mf_6", 4.0),
         ("m25_nylon_nut", 4.0), ("m25_nylon_screw_6", 4.0), ("lipo_2s_450", 1.0),
         ("lipo_strap_10mm", 1.0), ("ip2326_charger", 1.0), ("bms_hx_2s_jh20", 1.0),
         ("foam_tape", 1.0), ("toggle_mts102", 1.0), ("resistor_100k", 1.0),
         ("resistor_33k", 1.0), ("xt30_pigtail_pair", 1.0), ("dc_plug_5521_pigtail", 1.0)),
        drives="servo", protocol="feetech_sts", bus_ports=2, baud=1_000_000,
        mount="deck",
        nodes=(("servo", r"servo_(?!driver)|^servo$|sts\d|xl\d"), ("battery", r"lipo|battery"),
               ("charger", r"charger|ip2326"), ("protection", r"bms|protection"),
               ("switch", r"switch|toggle"), ("board", r"driver|esp32|board|controller")),
        wiring=(Wire("battery", "protection", cables=r"xt30|xt60|battery"),
                Wire("charger", "protection"),
                Wire("protection", "switch"),
                Wire("switch", "board", cables=r"dc_plug|5521|barrel"),
                Wire("board", "servo", cables=r"bus_cable|y_cable|bus|servo|3.?pin|extension",
                     via="the servo's own bus cable")),
        wiring_title="Bus cables and harness",
        wiring_text=("The harness: the XT30 pigtail from the battery's lead to the protection "
                     "board, the DC plug pigtail from the switch to the board's DC jack, and "
                     "the battery divider (the 100k resistor from the switched battery to an "
                     "ESP32 ADC pin, the 33k from that pin to ground). " + HV_WARNING,
                     "Each servo's bus cable (connector first) to its own one of the board's "
                     "two bus servo ports."),
        wiring_extras=("resistor_100k", "resistor_33k", "xt30_pigtail_pair",
                       "dc_plug_5521_pigtail"),
        source="option 1 of the electronics review (2026-10-03, hardware.electronics); "
               "Waveshare's wiki: two bus servo ports, Feetech SCS/STS servos"),
    Controller(
        "u2d2_dynamixel", "ROBOTIS U2D2 (USB to the DYNAMIXEL bus) on its Power Hub Board: "
        "your computer runs the walk over USB, your supply at the servos' voltage powers "
        "the bus; the electronics deck bare",
        (("u2d2", 1.0), ("u2d2_power_hub_set", 1.0)),
        drives="servo", protocol="dynamixel_2", bus_ports=1, baud=1_000_000,
        nodes=(("servo", r"servo_(?!driver)|^servo$|xl\d|sts\d"),
               ("hub", r"power_hub|hub"), ("board", r"^u2d2$|u2d2 \(|usb")),
        wiring=(Wire("hub", "board", via="the hub's 3-pin cable to the U2D2's TTL port",
                     text="The U2D2 onto its Power Hub Board, the hub's 3-pin cable into the "
                          "U2D2's TTL port."),
                Wire("hub", "servo", cables=r"bus_cable|3.?pin|extension",
                     via="the servos' own cables, chained",
                     text="The servos chained on the hub's TTL bus, each servo's cable into "
                          "the next (the first into the hub).")),
        wiring_title="USB interface and bus",
        wiring_text=("Your supply at the servos' voltage (XL330 5 V, XL430 11.1-12 V) to the "
                     "hub's terminal block, + to +, off until the bus is wired; the U2D2's "
                     "USB-C to the computer that runs the machine (spiderpig control).",),
        wiring_extras=("u2d2", "u2d2_power_hub_set"),
        carried=("u2d2", "u2d2_power_hub_set"),
        source="robotis.us product pages and the e-manual, read 2026-10-14 "
               "(hardware.electronics): the U2D2's TTL port (the XL servos chain from one "
               "another), 9,600 to 6,000,000 bps, no power to the servos; the hub 3.5-24 V"),
)}
DEFAULT = "skr_pico_pi_zero"
"""A stepper-driven design's controller when it names none."""
DECK_KIT = "esp32_bus_servo"
"""A Feetech bus-servo design's controller when it names none (:data:`BUS_DEFAULTS`), and the
one the electronics deck is drawn for (:mod:`construction.deck`). The STS3215 walkers
carried it, unnamed, before the role reached them: it is the role's ``implicit`` key
(:class:`roles.Role`): never move their default. (A DYNAMIXEL walker's moved on 2026-10-14,
the user's call: its resolved spec names ``u2d2_dynamixel``, so its id moved with it.)"""
DEFAULTS = {"actuator": DEFAULT, "servo": DECK_KIT}
BUS_DEFAULTS = {"feetech_sts": DECK_KIT, "dynamixel_2": "u2d2_dynamixel"}
"""A bus-servo design's controller by its servo's protocol (:data:`PROTOCOLS`): one that
speaks it (2026-10-14: an XL330 / XL430 walker had the deck's Feetech board). A servo naming
no protocol gets :data:`DEFAULTS`' ``servo``."""


def get(key: str) -> Controller:
    try:
        return CONTROLLERS[key]
    except KeyError:
        raise KeyError(f"unknown controller {key!r}; have {sorted(CONTROLLERS)}") from None


def motors_of(config) -> str:
    """The motor role of ``config``'s design (:data:`DRIVES`, :func:`roles.motor_role`): a
    crank's servo, or ranged inputs' actuators."""
    from spiderpig import roles

    return roles.motor_role(config.lk)


def default_key(config) -> str:
    """The role ``controller``'s default for ``config``: the one that drives its motors
    (:data:`DEFAULTS`), and on a serial bus the one that speaks its servo's protocol
    (:data:`BUS_DEFAULTS`)."""
    need = motors_of(config)
    if need == "servo":
        from spiderpig import servos

        return BUS_DEFAULTS.get(servos.get(config.servo).protocol, DEFAULTS[need])
    return DEFAULTS[need]


def of(config) -> Controller | None:
    """The controller ``config`` names (``None``: :data:`roles.NONE`, your own), checked
    against what drives its design: :class:`ValueError` when it can't drive them. Every
    design with motors has the role (a mechanism's too: :mod:`spiderpig.roles`)."""
    key = config.role("controller")
    if not key:
        return None
    ctl = get(key)
    need = motors_of(config)
    if ctl.drives != need:
        fit = sorted(k for k, c in CONTROLLERS.items() if c.drives == need)
        raise ValueError(f"controller {key!r} drives {DRIVE_WORDS[ctl.drives]} and "
                         f"{config.linkage}'s motors are {DRIVE_WORDS[need]}: use "
                         f"{' or '.join([*fit, 'none'])}")
    return ctl


def carried_g(ctl: Controller) -> float:
    """What a walker carries of controller ``ctl`` on a bare deck (g): its ``carried``
    parts' catalogued ``mass_g`` (a part with none counts 0: the U2D2's power hub is
    unweighed). A deck-mounted controller's parts are bodies of the robot (modelled, not
    counted here)."""
    from spiderpig.hardware.catalog import get as item

    qty = dict(ctl.bom)
    return sum(qty.get(k, 1.0) * float(item(k).dims.get("mass_g", 0.0)) for k in ctl.carried)


def protocol_note(config, ctl: Controller | None) -> str:
    """Why ``ctl`` can't talk to ``config``'s servos (another bus protocol), or ``""``."""
    if ctl is None or not ctl.protocol:
        return ""
    from spiderpig import servos

    spoken = servos.get(config.servo).protocol
    if not spoken or spoken == ctl.protocol:
        return ""
    fit = BUS_DEFAULTS.get(spoken)
    use = f"the role controller {fit}, or none: your own" if fit else "the role controller none"
    return (f"the {ctl.key} board speaks {ctl.protocol}, the {config.servo} {spoken}: drive "
            f"it from a board that speaks it ({use})")

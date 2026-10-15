"""Driving a crank-driven design's serial-bus servos (2026-10-14): which servo turns which
crank, the file that sets them up, and each crank's one-time zero.

A walker's robot turns each side's crank by one servo (:class:`servos.mount.ServoDrive`:
the left side ``L``, bus ID 1; the right ``R``, ID 2), a mechanism its one crank (``crank``,
ID 1). Everything here comes from the design (its config and its template):

* **The phases.** Each crank carries its side's legs at their crank phases (the module's,
  or the design's ``phases``): the crank sets them, not the firmware. The two sides of a
  walker move identically in their planes (:mod:`construction.robot`), so the cranks are
  held at the same angle (``side_offset_deg`` 0); the host does it with the phase lock
  the sim assumes (:class:`sim.run.PhaseLock`, its gains in the file): two servos told one
  speed open-loop drift apart.
* **The sense.** A servo's output points out of its horn (servo ``+z`` is the side's
  ``-z``), and the right side is the left mirrored, so one crank angle turns the two
  servos opposite ways about their own axes. ``forward`` is the sense, seen facing the
  horn, of the turn that walks the robot forward (:func:`sim.mjcf.crank_sign`; a
  mechanism's: its crank angle growing). Whether the servo's positive speed is that sense
  is the servo's convention: the setup checks it once with a jog and keeps a ``sign``.
* **The zero** (:func:`homing_steps`): the design's crank angle 0 has the first crankpin
  (``zero_pin``) at ``zero_deg`` about the crank axis O from the side's ``+x`` (the robot's
  front). Each servo's 12-bit absolute encoder holds an offset in its EEPROM, set once with
  the crank there so that the pose reads :data:`ZERO_COUNT` (the Feetech STS's own "middle"
  calibration writes it; :func:`zero_offset` computes it for a register write), and
  homing is never needed again: the encoder is absolute over a turn.
* **The servos' settings** (:func:`setup`, written as ``servos.json`` by ``spiderpig
  control servo``): each one's ID, the bus's baud, the continuous (speed) mode, the torque
  limit the joints are rated at (:func:`config.torque_limit_nm`), and the registers of its
  protocol (:data:`REGISTERS`: Feetech's STS memory table, DYNAMIXEL's protocol 2.0 control
  table) to write them with.

The controller (the role ``controller``, :mod:`actuators.controllers`) is the board on the
bus (a walker's deck kit, the ESP32 servo driver), or your own (``none``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STEPS_PER_REV = 4096        # the servos' 12-bit absolute encoders (STS3215, XL330, XL430)
ZERO_COUNT = 2048           # the reading at a crank's zero: mid-scale (Feetech's "middle")

REGISTERS: dict[str, dict[str, tuple[int, int]]] = {
    # Feetech SMS/STS memory table (address, bytes): EEPROM below 40, unlocked by Lock = 0
    "feetech_sts": {"id": (5, 1), "baud": (6, 1), "max_torque": (16, 2),
                    "homing_offset": (31, 2), "mode": (33, 1), "torque_enable": (40, 1),
                    "goal_velocity": (46, 2), "torque_limit": (48, 2), "lock": (55, 1),
                    "present_position": (56, 2)},
    # DYNAMIXEL protocol 2.0, X series (XL330, XL430): EEPROM below 64, written torque off
    "dynamixel_2": {"id": (7, 1), "baud": (8, 1), "mode": (11, 1), "homing_offset": (20, 4),
                    "pwm_limit": (36, 2), "torque_enable": (64, 1), "goal_velocity": (104, 4),
                    "present_position": (132, 4)},
}
"""Each protocol's registers the setup writes (address, size in bytes)."""

BAUD_CODES: dict[str, dict[int, int]] = {
    "feetech_sts": {1_000_000: 0, 500_000: 1, 250_000: 2, 128_000: 3, 115_200: 4,
                    76_800: 5, 57_600: 6, 38_400: 7},
    "dynamixel_2": {9_600: 0, 57_600: 1, 115_200: 2, 1_000_000: 3, 2_000_000: 4,
                    3_000_000: 5, 4_000_000: 6},
}
SPEED_MODE = {"feetech_sts": 1, "dynamixel_2": 1}     # wheel / velocity: a crank turns on
TORQUE_FULL = {"feetech_sts": 1000, "dynamixel_2": 885}   # torque_limit's / pwm_limit's 100 %
DEFAULT_BAUD = 1_000_000                              # with no controller named (yours)


@dataclass(frozen=True)
class BusServo:
    """One crank's servo (module docstring): ``name`` (``L``, ``R``, ``crank``), its bus
    ``id``, ``side``, the crank phases of its legs (degrees, in the module's leg order),
    ``forward`` (``"ccw"`` / ``"cw"`` facing its horn), and its zero's crankpin and angle
    about O (degrees from the side's ``+x``)."""

    name: str
    id: int
    side: str
    phases_deg: tuple[float, ...]
    forward: str
    zero_pin: str
    zero_deg: float
    servo: str = ""


def bus_servos(config) -> list[BusServo]:
    """The design's crank servos (module docstring), from its template: one per side it
    builds (:attr:`config.BuildConfig.sides`). :class:`ValueError` for a design whose
    motors aren't bus servos (an arm's steppers: :mod:`.mapping`)."""
    from spiderpig.actuators.controllers import motors_of
    from spiderpig.fabricate import template_for
    from spiderpig.stack import topology_from_template

    if motors_of(config) != "servo":
        raise ValueError(f"{config.linkage}'s motors are steppers, not bus servos: its axes "
                         "are spiderpig control's arm axes")
    side = config.side_config()
    topo = topology_from_template(template_for(side), samples=360)
    pts = topo.geometry.points
    pins = topo.axes_of("crankpin")
    if topo.center is None or not pins:
        raise ValueError(f"{config.linkage}: no crank on O to drive")
    o = pts["O"][0]

    def angle(name: str, i: int) -> float:
        d = pts[name][i] - o
        return math.atan2(d[1], d[0])

    pin = pins[0].name
    turns_ccw = math.remainder(angle(pin, 1) - angle(pin, 0), 2 * math.pi) > 0
    ahead = _crank_sign(config) > 0          # a growing crank angle is forward
    # facing the horn (servo +z is the side's -z): a ccw turn about the side's +z is cw
    side_ccw = turns_ccw == ahead            # the forward turn, about the side's +z
    phases = tuple(round(math.degrees(ph) % 360.0, 6) for _, ph in config.legs)
    zero = round(math.degrees(angle(pin, 0)), 6)

    def servo(name: str, sid: int, where: str, mirrored: bool) -> BusServo:
        ccw = side_ccw == mirrored               # the mirror flips it, facing the horn
        return BusServo(name, sid, where, phases, "ccw" if ccw else "cw", pin, zero,
                        config.servo)

    if config.sides == 2:
        return [servo("L", 1, "left", False), servo("R", 2, "right", True)]
    return [servo("crank", 1, "", False)]


def _crank_sign(config) -> int:
    """+1 where a growing crank angle walks the robot forward (``+x``), else -1
    (:func:`sim.mjcf.crank_sign`); +1 for a design without feet (its own crank sense)."""
    if not config.lk.walks:
        return 1
    from spiderpig.sim.mjcf import crank_sign

    return crank_sign(config.side_config())


def zero_offset(raw: int, protocol: str, target: int = ZERO_COUNT) -> int:
    """The homing-offset register's value that makes an encoder reading ``raw`` (at the
    crank's zero pose) read ``target``: Feetech's STS reads ``raw - offset``, as a
    sign-magnitude 12-bit number (bit 11 the sign, at most 2047); DYNAMIXEL's reads
    ``raw + offset`` (a signed 32-bit). Values within one turn either way."""
    d = (int(raw) - int(target) + STEPS_PER_REV // 2) % STEPS_PER_REV - STEPS_PER_REV // 2
    if protocol == "feetech_sts":
        d = max(-2047, min(2047, d))
        return (1 << 11) | -d if d < 0 else d
    if protocol == "dynamixel_2":
        return -d
    raise ValueError(f"unknown servo protocol {protocol!r}")


def torque_limit_share(config) -> float | None:
    """The torque limit the joints are rated at, as a share of the servo's stall torque
    (:func:`config.torque_limit_nm`); ``None`` without one."""
    from spiderpig import servos
    from spiderpig.config import torque_limit_nm

    limit, stall = torque_limit_nm(config), servos.get(config.servo).stall_torque_nm
    if limit is None or not stall:
        return None
    return min(1.0, limit / stall)


def setup(config) -> dict:
    """The servos' setup (module docstring) as JSON: the design, the controller and its
    bus, each servo's ID, side, phases, sense and zero, and the register writes of its
    protocol (``writes``: register, address, bytes, value, in order; the zero's offset is
    the one write left to the homing, :func:`homing_steps`)."""
    from spiderpig import servos
    from spiderpig.actuators.controllers import of, protocol_note
    from spiderpig.sim.mjcf import SimParams

    ctl = of(config)
    spec = servos.get(config.servo)
    protocol = spec.protocol or (ctl.protocol if ctl else "")
    if protocol not in REGISTERS:
        raise ValueError(f"the {config.servo} names no serial bus protocol spiderpig knows "
                         f"({', '.join(REGISTERS)})")
    baud = ctl.baud if ctl is not None and ctl.baud else DEFAULT_BAUD
    reg = REGISTERS[protocol]
    share = torque_limit_share(config)
    units = None if share is None else round(share * TORQUE_FULL[protocol])
    p = SimParams()
    out_servos = []
    for s in bus_servos(config):
        values = [("id", s.id), ("baud", BAUD_CODES[protocol][baud]),
                  ("mode", SPEED_MODE[protocol])]
        if units is not None:
            values += [(r, units) for r in ("max_torque", "torque_limit", "pwm_limit")
                       if r in reg]
        writes = [{"register": r, "address": reg[r][0], "bytes": reg[r][1], "value": v}
                  for r, v in values]
        out_servos.append({
            "name": s.name, "id": s.id, "side": s.side, "servo": s.servo,
            "legs_phase_deg": list(s.phases_deg), "forward": s.forward, "sign": 1,
            "zero": {"crankpin": s.zero_pin, "angle_deg": s.zero_deg, "reads": ZERO_COUNT,
                     "homing_offset": None},
            "writes": writes})
    note = protocol_note(config, ctl)
    return {
        "design": {"linkage": config.linkage, "module": config.module,
                   "robot": config.robot, "servo": config.servo},
        "controller": None if ctl is None else {"key": ctl.key, "label": ctl.label},
        "bus": {"protocol": protocol, "baud": baud, "steps_per_rev": STEPS_PER_REV,
                "ports": ctl.bus_ports if ctl is not None else None},
        "torque_limit": None if share is None else {
            "share_of_stall": round(share, 4), "units": units},
        "side_offset_deg": 0.0,
        "phase_lock": {"kp": p.phase_lock_kp, "ki": p.phase_lock_ki,
                       "units": "fractions of the no-load speed per rad, and per rad*s "
                                "(sim.run.PhaseLock)"} if len(out_servos) > 1 else None,
        "servos": out_servos,
        "notes": [note] if note else [],
    }


def homing_steps(config) -> list[tuple[str, str]]:
    """``(servo, action)``: setting each servo up (one on the bus at a time: they all ship
    as ID 1) and zeroing its crank once (module docstring)."""
    from spiderpig import servos

    protocol = servos.get(config.servo).protocol
    sts = protocol == "feetech_sts"
    front = "the robot's front (+x)" if config.lk.walks else "the side's +x"
    out = []
    for s in bus_servos(config):
        where = f" ({s.side} side)" if s.side else ""
        out += [
            (s.name, f"plug the {s.name} servo{where} alone into the bus; write the setup's "
                     f"registers (servos.json): ID {s.id}, the baud, the speed mode, the "
                     "torque limit"),
            (s.name, "torque off; turn its crank by hand until crankpin "
                     f"{s.zero_pin} stands {s.zero_deg:g} deg from {front} about the "
                     "crank axis O: the design's crank angle 0"),
            (s.name, ("write 128 to Torque_Enable (address 40): the STS stores the offset "
                      f"that makes this pose read {ZERO_COUNT} (Feetech's middle "
                      "calibration), or write zero_offset(reading) to Homing_Offset "
                      "(address 31) with the EEPROM unlocked (Lock, 55, = 0, then 1)")
                     if sts else
                     ("read Present_Position, write zero_offset(reading) to Homing_Offset "
                      f"(address 20, torque off): the pose then reads {ZERO_COUNT}")),
            (s.name, f"jog it at a small positive speed: if the crank turns against "
                     f"forward ({s.forward}, facing the horn), set its sign to -1 in "
                     "servos.json"),
        ]
    if len(out) > 4:
        out.append(("all", "plug both servos in; the host holds the cranks at the same "
                           "angle (side_offset_deg 0, the phase lock's gains)"))
    return out

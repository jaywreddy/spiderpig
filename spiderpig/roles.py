"""Construction roles: the kinds of part a design picks a construction for, by key.

A **role** is a slot of a design (the pins between links, the frame pillars, the crank, and
the roles later kinds of robot add) filled by one construction from a registry. Every
place that names a design's constructions (:class:`config.BuildConfig`, the spec's
``constructions`` object, the resolved spec, the CLI's options, the MCP catalog) reads this
table, so a new kind of part is one :class:`Role` here and its registry, not an edit at
each hop.

The first three roles predate the table and keep their own :class:`config.BuildConfig`
fields (``field``): ``pillar``, ``pin``, ``crank``. Every later role lives in
``BuildConfig.roles`` (``(role, key)`` pairs, only those that differ from the role's
default), applies to the linkage kinds it lists (``kinds``: required, so a new role never
reaches the walkers by accident), and is written into a resolved spec for every design it
applies to (a stored design never depends on a default that later moves), but where its
key is the role's ``implicit`` one: what the designs of a kind carried before the role
reached it (the walkers' deck electronics, the role ``controller``), left out so their
ids stay.

A role filled once per design is ``per="design"``. One filled per drive or per joint
(``"drive"``, ``"joint"``: an arm's transmission on each of its inputs) takes one key for
all its slots under its plain name, and a slot's own under ``"<role>@<slot>"``
(``transmission@q2``); the slots are the design's (its linkage's inputs or joints),
checked where it is built. An ``optional`` role may be emptied with the key
:data:`NONE`.

This module is light on purpose (the config imports it): a role names its registry and its
default by ``"module:NAME"`` path, loaded only when asked for (the incremental cache keys
follow those paths: :mod:`spiderpig.keys`).
"""

from __future__ import annotations

import importlib
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

NONE = "none"
"""The key that empties an optional role (a design with no homing sensor)."""

PER = ("design", "drive", "joint")


@dataclass(frozen=True)
class Role:
    """One slot of a design. ``registry``: ``"module:NAME"`` of the mapping key ->
    construction it picks from (a construction that declares ``roles`` is offered only for
    those). ``kinds``: the linkage kinds it applies to. ``field``: the
    :class:`config.BuildConfig` field that holds it (``""``: ``BuildConfig.roles``).
    ``default``: the key a design gets when it names none: a literal key, or
    ``"module:function"`` called with the config (``""``: none). ``per``: one key per
    design, or per drive / joint (:data:`PER`). ``optional``: :data:`NONE` empties it.
    ``robot``: a part of the robot round the side, not of the side (an arm's turntable, its
    tool): the side's design ignores it (:meth:`config.BuildConfig.side_config`), so one
    plan serves every choice of it. ``implicit``: the key designs carried, unnamed, before
    the role reached their kind (a walker's deck electronics, before 2026-10-14): where it
    is a design's default it isn't written into the resolved spec, so the designs stored
    before keep their ids (and read back with it: never move that default)."""

    name: str
    title: str
    registry: str
    kinds: tuple[str, ...]
    field: str = ""
    default: str = ""
    per: str = "design"
    optional: bool = False
    help: str = ""
    robot: bool = False
    implicit: str = ""

    def __post_init__(self):
        if self.per not in PER:
            raise ValueError(f"role {self.name}: per is one of {PER}, got {self.per!r}")
        if "@" in self.name or not self.name.isidentifier():
            raise ValueError(f"role name {self.name!r} must be an identifier")
        if not self.kinds:
            raise ValueError(f"role {self.name}: name the linkage kinds it applies to")

    @property
    def stored_in_roles(self) -> bool:
        return not self.field


ROLES: dict[str, Role] = {}


def register(role: Role) -> Role:
    if role.name in ROLES and ROLES[role.name] != role:
        raise ValueError(f"role {role.name!r} registered twice")
    ROLES[role.name] = role
    return role


def _load(path: str) -> Any:
    mod, _, attr = path.partition(":")
    return getattr(importlib.import_module(mod), attr)


def get(name: str) -> Role:
    try:
        return ROLES[name]
    except KeyError:
        raise KeyError(f"unknown construction role {name!r}; have {sorted(ROLES)}") from None


def split(qualified: str) -> tuple[str, str]:
    """``"transmission@q2"`` -> ``("transmission", "q2")``; a plain name's slot is ``""``."""
    name, _, slot = qualified.partition("@")
    return name, slot


def registry(name: str) -> Mapping[str, Any]:
    """The constructions role ``name`` picks from (imports them: CAD-heavy)."""
    return _load(get(name).registry)


def choices(name: str) -> list[str]:
    """The keys role ``name`` may take: its registry's constructions that serve it (one
    that declares ``roles`` serves only those), and :data:`NONE` for an optional role."""
    out = sorted(k for k, c in registry(name).items()
                 if name in (getattr(c, "roles", None) or (name,)))
    return [*out, NONE] if get(name).optional else out


def default_key(name: str, config) -> str:
    """The key role ``name`` gets in ``config`` when the config names none: its linkage's
    (:data:`config.LINKAGE_ROLES`), else the role's."""
    from spiderpig.config import LINKAGE_ROLES

    own = LINKAGE_ROLES.get(getattr(config, "linkage", "") or "", {}).get(name)
    if own is not None:
        return own
    d = get(name).default
    if ":" in d:
        return str(_load(d)(config))
    return d


def extra() -> list[Role]:
    """The roles kept in ``BuildConfig.roles`` (every one without a field of its own)."""
    return [r for r in ROLES.values() if r.stored_in_roles]


def applies(name: str, kind: str) -> bool:
    """Whether role ``name`` is a slot of a design of linkage kind ``kind``."""
    return kind in get(name).kinds


register(Role("pillar", "frame pivot", "spiderpig.construction:AXLES", ("walker", "mechanism"),
              field="pillar", default="spiderpig.config:config_pillar",
              help="the axles that join links to the frame plates"))
register(Role("pin", "pin between links", "spiderpig.construction:AXLES",
              ("walker", "mechanism"), field="pin", default="spiderpig.config:config_pin",
              help="the axles that join links to links"))
register(Role("servo", "drive servo", "spiderpig.servos:REGISTRY", ("walker", "mechanism"),
              field="servo", default="sts3215",
              help="the serial-bus servo that turns the crank (an arm's motors: actuator)"))
register(Role("crank", "crank", "spiderpig.construction:CRANKS", ("walker", "mechanism"),
              field="crank", default="spiderpig.config:config_crank",
              help="the crankshaft the drive turns: webs and crankpins"))
register(Role("transmission", "transmission", "spiderpig.construction:TRANSMISSIONS",
              ("arm",), default="belt", per="drive",
              help="what turns each input's link from its motor: a GT2 belt (5:1)"))
register(Role("actuator", "actuator", "spiderpig.actuators:STEPPERS", ("arm",),
              default="nema17_17hs4401", per="drive",
              help="each input's stepper motor"))
register(Role("angle_sensor", "joint angle sensor", "spiderpig.actuators.sensors:SENSORS",
              ("arm",), default="gauge", per="drive",
              help="how each planar input finds its zero at homing (the one homing registry, "
                   "the turntable's too): a digital angle gauge by hand, an optical endstop "
                   "and a flag on the link (the MT4's), its driver's stall on a hard stop, or "
                   "an SCL3300 inclinometer on the link"))
register(Role("homing_reference", "homing reference",
              "spiderpig.actuators.sensors:REFERENCES", ("arm",), default="switches",
              robot=True,
              help="what the joints' fine zero comes from: each joint's switch or gauge, or "
                   "a camera on the tool sighting fiducials on a reference sheet"))
register(Role("turntable", "turntable", "spiderpig.construction.rotary:PRESETS", ("arm",),
              default="mt4_compact", robot=True,
              help="the robot's base rotation (yaw): its bearings, transmission, motor and "
                   "homing sensor, as a preset (construction.rotary.PRESETS)"))
register(Role("pin_stack", "pin stacking", "spiderpig.stack.plan:PIN_STACKS", ("arm",),
              default="free",
              help="where a pin's links may sit in the stack: any layers (a spacer tube "
                   "fills each layer between them) or consecutive ones (no spacers)"))
register(Role("tool", "tool", "spiderpig.construction.tools:TOOLS", ("arm",),
              default="suction_cup", optional=True, robot=True,
              help="what the arm holds on its flange: a vacuum cup, fixed or on a wrist"))
register(Role("controller", "controller", "spiderpig.actuators.controllers:CONTROLLERS",
              ("walker", "arm"), default="spiderpig.actuators.controllers:default_key",
              optional=True, robot=True, implicit="esp32_bus_servo",
              help="the electronics that drive the motors: a walker's serial-bus servo "
                   "board, battery and harness on its deck (esp32_bus_servo), an arm's "
                   "stepper board running Klipper with its host computer, supply and cables "
                   "(skr_pico_pi_zero); none: your own (a walker's deck bare, an arm's "
                   "Klipper pins left to fill)"))

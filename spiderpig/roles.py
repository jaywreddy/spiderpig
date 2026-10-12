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
applies to (a stored design never depends on a default that later moves).

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
    design, or per drive / joint (:data:`PER`). ``optional``: :data:`NONE` empties it."""

    name: str
    title: str
    registry: str
    kinds: tuple[str, ...]
    field: str = ""
    default: str = ""
    per: str = "design"
    optional: bool = False
    help: str = ""

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
    """The key role ``name`` gets in ``config`` when the config names none."""
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
              field="pillar", default="standoff",
              help="the axles that join links to the frame plates"))
register(Role("pin", "pin between links", "spiderpig.construction:AXLES",
              ("walker", "mechanism"), field="pin", default="chicago",
              help="the axles that join links to links"))
register(Role("crank", "crank", "spiderpig.construction:CRANKS", ("walker", "mechanism"),
              field="crank", default="spiderpig.config:config_crank",
              help="the crankshaft the drive turns: webs and crankpins"))
register(Role("transmission", "transmission", "spiderpig.construction:TRANSMISSIONS",
              ("arm",), default="belt", per="drive",
              help="what turns each input's link from its motor: a GT2 belt (5:1), a cord "
                   "capstan (about 10:1)"))
register(Role("actuator", "actuator", "spiderpig.actuators:STEPPERS", ("arm",),
              default="nema17_17hs4401", per="drive",
              help="each input's stepper motor"))

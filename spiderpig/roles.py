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
default), applies to the linkage kinds it lists, and is written into a resolved spec for
every design it applies to (a stored design never depends on a default that later moves).

This module is light on purpose (the config imports it): a role names its registry and its
default by import path, loaded only when asked for.
"""

from __future__ import annotations

import importlib
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Role:
    """One slot of a design. ``registry``: ``"module:NAME"`` of the mapping key ->
    construction it picks from. ``field``: the :class:`config.BuildConfig` field that holds
    it (``""``: ``BuildConfig.roles``). ``default``: the key a design gets when it names
    none: a literal key, or ``"module:function"`` called with the config (``""``: the role
    is empty unless named). ``kinds``: the linkage kinds it applies to."""

    name: str
    title: str
    registry: str
    field: str = ""
    default: str = ""
    kinds: tuple[str, ...] = ("walker", "mechanism")
    help: str = ""

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


def registry(name: str) -> Mapping[str, Any]:
    """The constructions role ``name`` picks from (imports them: CAD-heavy)."""
    return _load(get(name).registry)


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


register(Role("pillar", "frame pivots", "spiderpig.construction:AXLES", field="pillar",
              default="standoff", help="the axles that join links to the frame plates"))
register(Role("pin", "pins between links", "spiderpig.construction:AXLES", field="pin",
              default="chicago", help="the axles that join links to links"))
register(Role("crank", "the crank", "spiderpig.construction:CRANKS", field="crank",
              default="spiderpig.config:config_crank",
              help="the crankshaft the drive turns: webs and crankpins"))

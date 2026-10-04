"""Construction groups and the constructions that build them (see :mod:`construction.base`).

Registries map a config key to a construction:

* ``AXLES``: pillars and link pins (:mod:`construction.axle`; the metal-shaft
  ones, ``rod`` / ``bolt`` / ``bearing`` / ``bushing``, in
  :mod:`construction.pivots`)
* ``CRANKS``: the crankshaft (:mod:`construction.crank`)

Add a construction by implementing ``dims`` (validation + the radii its
claims use) and ``realize`` (parts inside those claims), then registering
it here. Add a kind of group (:class:`construction.base.Group`) by
appending its factory to :data:`GROUP_FACTORIES`: :func:`side_groups`
runs them in that order, which is the groups' dependency order.
"""

from __future__ import annotations

from collections.abc import Callable

from spiderpig.construction.axle import AxleGroup as AxleGroup
from spiderpig.construction.axle import PrintedAxle
from spiderpig.construction.base import ConstructionError, Context, Group
from spiderpig.construction.crank import (
    BOLT_ROUND,
    KEYED_FLOAT,
    BoltCrank,
    KeyedCrank,
    PrintedCrank,
)
from spiderpig.construction.crank import CrankGroup as CrankGroup
from spiderpig.construction.pivots import PIVOTS
from spiderpig.construction.plates import FramePlates, LinkPlates
from spiderpig.servos.mount import DriveGroup

AXLES = {c.key: c for c in (PrintedAxle(), *PIVOTS)}
# ``keyed`` is the default (config.BuildConfig.crank, its keys pressed in, the chain screws
# threadlocked); ``keyed_float`` the same with sliding keys and dry screws (6.25 deg of play
# per interface); ``printed`` the crank held by clamp friction alone; both kept to compare
# ``bolt`` (the default): hex standoff crankpins on an aluminium crank sheet; ``bolt_round``
# the same with the friction-clamped round standoff (2026-10-04, kept to compare)
CRANKS = {c.key: c for c in (KeyedCrank(), KEYED_FLOAT, PrintedCrank(), BoltCrank(),
                             BOLT_ROUND)}


def _pick(registry: dict, key: str, what: str):
    try:
        return registry[key]
    except KeyError:
        have = sorted(registry)
    raise ConstructionError(f"no {what} construction {key!r}; have {have}") from None


def axle(key: str) -> PrintedAxle:
    return _pick(AXLES, key, "axle")


def crank(key: str) -> PrintedCrank:
    return _pick(CRANKS, key, "crank")


def _drive_groups(ctx: Context, config) -> list[Group]:
    return [DriveGroup(ctx.servo)]


def _crank_groups(ctx: Context, config) -> list[Group]:
    if ctx.topo.center is None:
        return []
    c = crank(config.crank)
    return [CrankGroup(c.resolve(ctx) if hasattr(c, "resolve") else c)]


def _axle_groups(ctx: Context, config) -> list[Group]:
    """One axle per pillar (``config.pillar``) and per link pin (``config.pin``)."""
    kinds = {"frame": config.pillar, "pin": config.pin}
    return [AxleGroup(ax, axle(kinds[ax.kind])) for ax in ctx.topo.axes if ax.kind in kinds]


def _plate_groups(ctx: Context, config) -> list[Group]:
    return [LinkPlates(), FramePlates()]


# The groups of a side, in dependency order (``config`` is the :class:`config.BuildConfig`).
GROUP_FACTORIES: list[Callable[[Context, object], list[Group]]] = [
    _drive_groups, _crank_groups, _axle_groups, _plate_groups,
]


def side_groups(ctx: Context, config) -> list[Group]:
    """The groups of one side, in dependency order (the plates last)."""
    return [g for make in GROUP_FACTORIES for g in make(ctx, config)]

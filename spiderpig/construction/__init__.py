"""Construction groups and the constructions that build them (see :mod:`construction.base`).

Registries map a config key to a construction:

* ``AXLES``: pillars and link pins (:mod:`construction.pivots`: the ``standoff`` pillar
  and the ``chicago`` pin)
* ``CRANKS``: the crankshaft (:mod:`construction.crank`: ``bolt``, and ``bolt_round`` for
  TrotBot's heel and toe)

The keys removed on 2026-10-07 (the printed, rod, bolt, insert and spliced pivots; the
printed, keyed and acrylic cranks) are :data:`config.REMOVED_CONSTRUCTIONS`, each with its
replacement.

Add a construction by implementing ``dims`` (validation + the radii its
claims use) and ``realize`` (parts inside those claims), then registering
it here. Add a kind of group (:class:`construction.base.Group`) by
appending its factory to :data:`GROUP_FACTORIES`: :func:`side_groups`
runs them in that order, which is the groups' dependency order.
"""

from __future__ import annotations

from collections.abc import Callable

from spiderpig.construction.axle import AxleGroup as AxleGroup
from spiderpig.construction.base import ConstructionError, Context, Group
from spiderpig.construction.crank import BOLT_ROUND, BoltCrank
from spiderpig.construction.crank import CrankGroup as CrankGroup
from spiderpig.construction.handcrank import HAND_CRANK, HandCrank
from spiderpig.construction.pivots import PIVOTS
from spiderpig.construction.plates import FramePlates, LinkPlates
from spiderpig.servos.mount import DriveGroup

AXLES = {c.key: c for c in PIVOTS}
# ``bolt`` (the default, config.DEFAULT_CRANKS): hex standoff crankpins on single aluminium
# web plates; ``bolt_round`` the same with the friction-clamped round standoff (TrotBot's heel
# and toe, config.LINKAGE_CRANKS)
# ``hand``: a decanting cradle's hand-cranked worm drive (construction.handcrank), its only
# crank (config.PRODUCT_CRANKS) and nothing else's
CRANKS = {c.key: c for c in (BoltCrank(), BOLT_ROUND, HAND_CRANK)}
HAND = HAND_CRANK.key


def _pick(registry: dict, key: str, what: str):
    try:
        return registry[key]
    except KeyError:
        have = sorted(registry)
    raise ConstructionError(f"no {what} construction {key!r}; have {have}") from None


def axle(key: str):
    return _pick(AXLES, key, "axle")


def crank(key: str) -> BoltCrank:
    return _pick(CRANKS, key, "crank")


def _drive_groups(ctx: Context, config) -> list[Group]:
    if config.crank == HAND:        # a decanting cradle: the hand crank drives it
        return []
    return [DriveGroup(ctx.servo)]


def _crank_groups(ctx: Context, config) -> list[Group]:
    if ctx.topo.center is None:
        return []
    if config.crank == HAND:
        return [HandCrank()]
    return [CrankGroup(crank(config.crank).resolve(ctx))]


def _axle_groups(ctx: Context, config) -> list[Group]:
    """One axle per pillar (``config.pillar``) and per link pin (``config.pin``)."""
    kinds = {"frame": config.pillar, "pin": config.pin}
    made = {}
    for kind, key in kinds.items():
        a = axle(key)
        made[kind] = a.resolve(ctx) if hasattr(a, "resolve") else a   # per design (a rule)
        if kind == "frame" and _cradle(config) and hasattr(made[kind], "one_piece"):
            # a decanting cradle's pillars carry its bottle: always the steel shaft, never
            # the aluminium goBILDA standoff (cradle_report's strength rows)
            made[kind] = made[kind].one_piece()
    return [AxleGroup(ax, made[ax.kind]) for ax in ctx.topo.axes if ax.kind in kinds]


def _cradle(config) -> bool:
    return getattr(getattr(config, "lk", None), "product", "") == "cradle"


def _cradle_groups(ctx: Context, config) -> list[Group]:
    """A decanting cradle's stand and tie rods in this side (construction.cradle)."""
    if not _cradle(config):
        return []
    from spiderpig.construction.cradle import CradleStand, CradleTies

    return [CradleStand(), CradleTies()]


def _plate_groups(ctx: Context, config) -> list[Group]:
    if _cradle(config):
        from spiderpig.construction.cradle import CradlePlates

        return [LinkPlates(), CradlePlates()]
    return [LinkPlates(), FramePlates()]


# The groups of a side, in dependency order (``config`` is the :class:`config.BuildConfig`).
GROUP_FACTORIES: list[Callable[[Context, object], list[Group]]] = [
    _drive_groups, _crank_groups, _axle_groups, _cradle_groups, _plate_groups,
]


def side_groups(ctx: Context, config) -> list[Group]:
    """The groups of one side, in dependency order (the plates last)."""
    return [g for make in GROUP_FACTORIES for g in make(ctx, config)]

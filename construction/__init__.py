"""Construction groups and the constructions that build them (see :mod:`construction.base`).

Registries map a config key to a construction:

* ``AXLES``: pillars and link pins (:mod:`construction.axle`)
* ``CRANKS``: the crankshaft (:mod:`construction.crank`)

Add a construction by implementing ``dims`` (validation + the radii its
claims use) and ``realize`` (parts inside those claims), then registering
it here.
"""

from __future__ import annotations

from construction.axle import AxleGroup, PrintedAxle
from construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    DriveInterface,
    Group,
    Params,
    Realized,
)
from construction.crank import CrankGroup, PrintedCrank
from construction.plates import FramePlates, LinkPlates

AXLES = {c.key: c for c in (PrintedAxle(),)}
CRANKS = {c.key: c for c in (PrintedCrank(),)}


def _pick(registry: dict, key: str, what: str):
    try:
        return registry[key]
    except KeyError:
        have = sorted(registry)
    raise ConstructionError(f"no {what} construction {key!r}; have {have}") from None


def axle(key: str):
    return _pick(AXLES, key, "axle")


def crank(key: str):
    return _pick(CRANKS, key, "crank")


__all__ = [
    "AXLES", "CRANKS", "FRAME_INNER", "FRAME_OUTER", "AxleGroup", "Build", "ConstructionError",
    "Context", "CrankGroup", "DriveInterface", "FramePlates", "Group", "LinkPlates", "Params",
    "Realized", "axle", "crank",
]

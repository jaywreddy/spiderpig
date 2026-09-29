"""Pivot joinery options (see :mod:`joinery.base` for the contract).

``get(key)`` returns an option; ``options(kind)`` lists the ones that can
serve a site kind. ``DEFAULTS`` is what a build uses unless told otherwise.
"""

from __future__ import annotations

from joinery.base import (
    Envelope,
    Hardware,
    Hole,
    Joinery,
    JoineryParams,
    Member,
    PivotSite,
    SiteKind,
)

REGISTRY: dict[str, Joinery] = {}

# pivots between links / fixed frame pivots / crankpins
DEFAULTS: dict[SiteKind, str] = {"pin": "bolt", "frame": "bolt", "crankpin": "dowel"}


def register(option: Joinery) -> Joinery:
    REGISTRY[option.key] = option
    return option


def get(key: str) -> Joinery:
    _load()
    try:
        return REGISTRY[key]
    except KeyError:
        raise KeyError(f"no joinery option {key!r}; have {sorted(REGISTRY)}") from None


def options(kind: SiteKind | None = None) -> list[Joinery]:
    _load()
    return [o for o in REGISTRY.values() if kind is None or kind in o.kinds]


_LOADED = False


def _load() -> None:
    global _LOADED
    if _LOADED:
        return
    _LOADED = True
    from joinery import bearing, bolt, bushing, crankpins, printed, shoulder  # noqa: F401


__all__ = [
    "DEFAULTS", "Envelope", "Hardware", "Hole", "Joinery", "JoineryParams", "Member",
    "PivotSite", "REGISTRY", "get", "options", "register",
]

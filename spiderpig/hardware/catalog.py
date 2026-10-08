"""Catalog of purchasable items (fasteners, servos, sheet stock, glue, filament).

Every item that ends up in the bill of materials is registered here under a
stable ``key``. Modules that model hardware (:mod:`construction`,
:mod:`servos`) set ``Body.bom_key`` on the bodies they create, or return
:class:`BomLine` extras for things they don't model (glue, sheets); the BOM
resolves those keys against this catalog. The sheet helpers below read what
the build needs from the sheet stock item.

Offers list where to buy, best first: a direct product page for the exact part, from
the makers' own stores, McMaster-Carr, DigiKey, Mouser, Accu or MISUMI first, marketplaces
(Amazon, eBay, AliExpress) last, only where nothing better sells the part
(:mod:`hardware.sources` puts the default build's sourced pages first). ``verified``
records whether the link was checked when it was added; unverified links are still
listed, flagged in the BOM.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from dataclasses import dataclass, field

Category = str  # "fastener" | "nut" | "washer" | "bearing" | "bushing" | "dowel" | "spacer"
#                  | "servo" | "horn" | "sheet" | "adhesive" | "clip" | "electronics" | "misc"


@dataclass(frozen=True)
class Offer:
    """One way to buy an item."""

    vendor: str
    url: str
    sku: str | None = None
    pack_qty: int = 1
    price_usd: float | None = None    # per pack
    verified: bool = False
    note: str = ""
    tiers: tuple[tuple[int, float], ...] = ()
    """Quantity pricing (packs from, USD per pack), ascending; ``price_usd`` is the first."""

    def buy(self, qty: float) -> tuple[int, float | None]:
        """``(packs, usd)``: the cheapest way to have ``qty``, whole packs, buying more
        where a price break makes that cheaper (five of a part at 10.86 cost less than
        four at 14.97)."""
        need = max(1, math.ceil(qty / max(self.pack_qty, 1) - 1e-9))
        if self.price_usd is None:
            return need, None
        if not self.tiers:
            return need, need * self.price_usd

        def cost(n: int) -> float:
            return n * max((t for t in self.tiers if t[0] <= n), default=(1, self.price_usd))[1]

        n = min([need] + [m for m, _ in self.tiers if m > need], key=lambda n: (cost(n), n))
        return n, round(cost(n), 2)


@dataclass(frozen=True)
class Item:
    """A purchasable item. ``dims`` holds modelling dimensions in mm."""

    key: str
    name: str
    category: Category
    offers: tuple[Offer, ...] = ()
    dims: dict = field(default_factory=dict)
    notes: str = ""

    @property
    def offer(self) -> Offer | None:
        return self.offers[0] if self.offers else None


_FACTORIES: list[tuple[str, Callable[[str], Item | None]]] = []
_LOCK = threading.RLock()     # the catalog's writes, and the snapshots it is iterated by


class _Catalog(dict):
    """The registered items by key, plus the families made on demand: a key no item has
    yet goes to the factory of its prefix (:func:`register_factory`), which registers it
    (``CATALOG[key]``, ``CATALOG.get(key)``, ``key in CATALOG``, :func:`get`), its sourced
    offers first (:func:`hardware.sources.sourced`). Iterating it (``for``, ``items()``,
    ``keys()``, ``values()``) walks a snapshot of what is registered so far, so a lookup
    that makes an item, in this thread or another, never changes a dict being iterated."""

    def _make(self, key) -> Item | None:
        if not isinstance(key, str):
            return None
        _load()                 # (outside the lock: loading imports, which register)
        for prefix, make in _FACTORIES:
            if key.startswith(prefix) and (item := make(key)) is not None:
                from spiderpig.hardware.sources import sourced

                item = sourced(item)
                with _LOCK:
                    if dict.__contains__(self, key):    # another thread made it first
                        return dict.__getitem__(self, key)
                    register(item)
                return item
        return None

    def __missing__(self, key):
        item = self._make(key)
        if item is None:
            raise KeyError(key)
        return item

    def get(self, key, default=None):
        if dict.__contains__(self, key):
            return dict.__getitem__(self, key)
        item = self._make(key)
        return default if item is None else item

    def __contains__(self, key) -> bool:
        return dict.__contains__(self, key) or self._make(key) is not None

    def __setitem__(self, key, value) -> None:
        with _LOCK:
            dict.__setitem__(self, key, value)

    def _snapshot(self) -> list[tuple[str, Item]]:
        with _LOCK:
            return list(dict.items(self))

    def __iter__(self):
        return iter([k for k, _ in self._snapshot()])

    def keys(self):
        return [k for k, _ in self._snapshot()]

    def values(self):
        return [v for _, v in self._snapshot()]

    def items(self):
        return self._snapshot()


CATALOG: dict[str, Item] = _Catalog()


def register(*items: Item) -> None:
    with _LOCK:
        for it in items:
            if dict.__contains__(CATALOG, it.key) and dict.__getitem__(CATALOG, it.key) != it:
                raise ValueError(f"catalog key {it.key!r} registered twice with different data")
            CATALOG[it.key] = it


def register_factory(prefix: str, make: Callable[[str], Item | None]) -> None:
    """Items keyed ``<prefix>...`` made when first asked for: ``make(key)`` is the item,
    or ``None`` for a key the family doesn't have (a family too large to register whole,
    the NETRF6 pillar shafts' 2,921 lengths)."""
    _FACTORIES.append((prefix, make))


def get(key: str) -> Item:
    _load()
    try:
        return CATALOG[key]
    except KeyError:
        raise KeyError(f"no catalog item {key!r}") from None


def pick_length(needed: float, lengths: tuple[float, ...]) -> float:
    """The shortest standard length that is at least ``needed``."""
    for length in sorted(lengths):
        if length >= needed - 1e-9:
            return length
    raise ValueError(f"no standard length >= {needed:.1f} mm in {lengths}")


def sheet_thickness(sheet: str, override: float | None = None) -> float:
    """The layer pitch: ``override`` (a measured sheet) or the sheet item's nominal thickness."""
    return override if override is not None else float(get(sheet).dims["thickness"])


def sheet_name(sheet: str) -> str:
    try:
        return get(sheet).name
    except KeyError:
        return sheet


def sheet_size(sheet: str) -> tuple[float, float]:
    """The sheet stock's usable size (mm)."""
    size = get(sheet).dims.get("sheet_mm")
    return (float(size[0]), float(size[1])) if size else (200.0, 200.0)


def adhesive(sheet: str) -> str:
    """The catalog item that laminates plates of this sheet (aluminium: epoxy)."""
    if "plywood" in sheet:
        return "wood_glue"
    try:
        metal = get(sheet).dims.get("material") == "aluminium"
    except KeyError:
        metal = False
    return "epoxy_2part" if metal else "acrylic_cement"


_LOADED = False


def _load() -> None:
    """Import the data modules that register items (lazily, once)."""
    global _LOADED
    if _LOADED:
        return
    _LOADED = True
    from spiderpig.hardware import (
        electronics,  # noqa: F401  (the deck's electronics)
        parts,  # noqa: F401  (registers fasteners, bearings, sheets, ...)
        shims,  # noqa: F401  (each DIN 988 thickness, after parts)
        sources,
    )
    from spiderpig.servos import catalog  # noqa: F401  (registers servos and horns)

    sources.apply()     # each bought item's direct product page first (2026-10-05), last

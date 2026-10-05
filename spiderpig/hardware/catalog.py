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


CATALOG: dict[str, Item] = {}


def register(*items: Item) -> None:
    for it in items:
        if it.key in CATALOG and CATALOG[it.key] != it:
            raise ValueError(f"catalog key {it.key!r} registered twice with different data")
        CATALOG[it.key] = it


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

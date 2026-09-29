"""Catalog of purchasable items (fasteners, bearings, servos, sheet stock, glue).

Every item that ends up in the bill of materials is registered here under a
stable ``key``. Modules that model hardware (``joinery``, ``servos``) set
``Body.bom_key`` on the bodies they create, or return :class:`BomLine`
extras for things they don't model (washers, glue); the BOM resolves those
keys against this catalog.

Offers list where to buy, best first. Prefer large vendors (manufacturer
stores, McMaster-Carr, Amazon, Misumi, igus, RobotShop, Pololu, Adafruit,
DigiKey). ``verified`` records whether the link was checked when it was
added; unverified links are still listed, flagged in the BOM.
"""

from __future__ import annotations

from dataclasses import dataclass, field

Category = str  # "fastener" | "nut" | "washer" | "bearing" | "bushing" | "dowel" | "spacer"
#                  | "servo" | "horn" | "sheet" | "adhesive" | "clip" | "misc"


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


_LOADED = False


def _load() -> None:
    """Import the data modules that register items (lazily, once)."""
    global _LOADED
    if _LOADED:
        return
    _LOADED = True
    from hardware import parts  # noqa: F401  (registers fasteners, bearings, sheets, ...)
    from servos import catalog  # noqa: F401  (registers servos and horns)

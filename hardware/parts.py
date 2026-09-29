"""Catalog data: generic purchasable hardware (fasteners, bearings, sheet stock, glue).

Key naming (other modules build keys with these helpers, so keep them stable):

* socket head cap screws: ``shcs("3", 12) -> "m3_shcs_12"`` (sizes "2", "2p5", "3")
* female-female M3 hex standoffs: ``standoff_ff(20) -> "m3_standoff_ff_20"``

Every key these helpers can produce for the lengths in ``SHCS_LENGTHS`` /
``STANDOFF_FF_LENGTHS`` must be registered below.

Servo-specific items (servos, horns sold separately) live in
:mod:`servos.catalog`.
"""

from __future__ import annotations

from hardware.catalog import Item, Offer, register  # noqa: F401

SHCS_LENGTHS: dict[str, tuple[float, ...]] = {
    "2": (4, 5, 6, 8, 10, 12, 16, 20),
    "2p5": (4, 5, 6, 8, 10, 12, 16, 20),
    "3": (6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 35, 40),
}
STANDOFF_FF_LENGTHS: tuple[float, ...] = (5, 6, 8, 10, 12, 15, 20, 25, 30, 35, 40)


def shcs(size: str, length: float) -> str:
    return f"m{size}_shcs_{length:g}"


def standoff_ff(length: float) -> str:
    return f"m3_standoff_ff_{length:g}"


# ---------------------------------------------------------------------------
# Registrations (filled in from docs/research: vendors, dimensions)
# ---------------------------------------------------------------------------

register(
    Item("acrylic_3mm", "3 mm cast acrylic sheet, 12 x 12 in", "sheet",
         (Offer("Amazon", "https://www.amazon.com/s?k=3mm+cast+acrylic+sheet+12x12"),),
         dims={"thickness": 3.0},
         notes="Nominal 3 mm; real sheets vary by up to about 8 %. "
               "Measure yours and pass --thickness."),
    Item("plywood_3mm", "3 mm Baltic birch plywood, 12 x 12 in", "sheet",
         (Offer("Amazon", "https://www.amazon.com/s?k=3mm+baltic+birch+plywood+12x12"),),
         dims={"thickness": 3.0}),
)

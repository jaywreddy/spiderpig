"""How a body is made: the registry of fabrication methods (:attr:`mechanism.Body.fab`).

Every body names one method by key. The stages that sort bodies by how they are made (the
BOM's made rows, the part labels, the build's per-method output folders, the fabrication
cache) iterate :data:`MADE` and read a method's record instead of spelling the keys, so a
new method is one entry here plus what is particular to it (its cut rules, its files).

==============  =========  ====  ======  =====================================================
key             title      made  flat    what
==============  =========  ====  ======  =====================================================
``laser``       Laser-cut  yes   yes     a 2-D profile cut from a sheet (DXF, kerf, cut rules:
                                         :mod:`manufacture`)
``printed``     3D printed yes   no      FDM printed (STL, filament mass)
``purchased``   Purchased  no    no      bought; its ``bom_key`` names the catalog item
==============  =========  ====  ======  =====================================================

``MADE`` keeps the order the outputs list made parts in (laser first, then printed).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FabMethod:
    """One way a body is made. ``made``: from stock by a process (not bought); ``flat``: a
    2-D profile from a sheet (the DXF, kerf and sheet-label path); ``folder``: the build's
    output folder for its per-part files (``""``: none)."""

    key: str
    title: str
    made: bool
    flat: bool = False
    folder: str = ""


METHODS: dict[str, FabMethod] = {}


def register(m: FabMethod) -> FabMethod:
    if m.key in METHODS and METHODS[m.key] != m:
        raise ValueError(f"fab method {m.key!r} registered twice")
    METHODS[m.key] = m
    return m


LASER = register(FabMethod("laser", "Laser-cut", made=True, flat=True, folder="laser")).key
PRINTED = register(FabMethod("printed", "3D printed", made=True, folder="print")).key
PURCHASED = register(FabMethod("purchased", "Purchased", made=False)).key


def get(key: str) -> FabMethod:
    try:
        return METHODS[key]
    except KeyError:
        raise KeyError(f"unknown fab method {key!r}; have {sorted(METHODS)}") from None


def made() -> tuple[str, ...]:
    """The made methods' keys, in output order (laser, printed, then any added later)."""
    return tuple(k for k, m in METHODS.items() if m.made)


def folders() -> tuple[str, ...]:
    """Every made method's output folder (the build owns and clears them)."""
    return tuple(m.folder for m in METHODS.values() if m.made and m.folder)


MADE = made()
KINDS = (PRINTED, LASER, PURCHASED)
"""The part labels' kinds in their listing order (:mod:`labels`): printed, laser, bought."""

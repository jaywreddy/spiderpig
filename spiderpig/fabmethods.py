"""How a body is made: the registry of fabrication methods (:attr:`mechanism.Body.fab`).

Every body names one method by key (checked when the body is made). The stages that sort
bodies by how they are made (the BOM's made rows, the part labels, the build's per-method
output folders, the fabrication cache, the mass and the bake's materials) read a method's
record through :func:`made`, :func:`kinds` and :func:`get` instead of spelling the keys, so
a new method is one entry here plus what is particular to it (its cut rules, its files).

==============  ==========  ====  =====  ========  ==========================================
key             title       made  flat   stock     what
==============  ==========  ====  =====  ========  ==========================================
``laser``       Laser-cut   yes   yes    sheet     a 2-D profile cut from a sheet (DXF, kerf,
                                                   cut rules: :mod:`manufacture`)
``printed``     3D printed  yes   no     filament  FDM printed (STL, filament mass)
``purchased``   Purchased   no    no               bought; ``bom_key`` names the catalog item
``display``     Display     no    no               drawn only (a decanting cradle's bottle and
                only                               decanter): nothing bought, cut or printed
==============  ==========  ====  =====  ========  ==========================================

``flat``: the DXF and sheet-label path. ``stock``: what a made part's mass and look come
from (``"sheet"``: its ``Body.sheet`` catalog item, else the build's sheet;
``"filament"``: the build's filament). ``order``: where the method's parts come in the
part labels' listing (printed, laser, then bought); :func:`made` lists the made methods in
registration order (laser, then printed: the BOM's and the build's order).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FabMethod:
    """One way a body is made (the module docstring's table)."""

    key: str
    title: str
    made: bool
    order: int
    flat: bool = False
    stock: str = ""
    folder: str = ""


METHODS: dict[str, FabMethod] = {}


def register(m: FabMethod) -> FabMethod:
    if m.key in METHODS and METHODS[m.key] != m:
        raise ValueError(f"fab method {m.key!r} registered twice")
    METHODS[m.key] = m
    return m


LASER = register(FabMethod("laser", "Laser-cut", made=True, order=1, flat=True, stock="sheet",
                           folder="laser")).key
PRINTED = register(FabMethod("printed", "3D printed", made=True, order=0, stock="filament",
                             folder="print")).key
PURCHASED = register(FabMethod("purchased", "Purchased", made=False, order=9)).key
DISPLAY = register(FabMethod("display", "Display only", made=False, order=10)).key
"""Drawn, never bought, cut or printed: a decanting cradle's bottle and decanter
(:mod:`construction.cradle`): no mass, no BOM line, a glass material in the bake."""


def get(key: str | None) -> FabMethod:
    try:
        return METHODS[key]          # pyright: ignore[reportArgumentType]  # None: KeyError
    except KeyError:
        raise KeyError(f"unknown fab method {key!r}; have {sorted(METHODS)}") from None


def known(key: str | None) -> bool:
    return key in METHODS


def made() -> tuple[str, ...]:
    """The made methods' keys, in registration order (laser, printed, then any added)."""
    return tuple(k for k, m in METHODS.items() if m.made)


def kinds() -> tuple[str, ...]:
    """Every method's key in the part labels' listing order (printed, laser, ..., bought)."""
    return tuple(sorted(METHODS, key=lambda k: (METHODS[k].order, k)))


def folders() -> tuple[str, ...]:
    """Every made method's output folder (the build owns and clears them)."""
    return tuple(m.folder for m in METHODS.values() if m.made and m.folder)


def flat(key: str | None) -> bool:
    """Whether a body made by ``key`` is a 2-D profile cut from a sheet."""
    return key in METHODS and METHODS[key].flat     # pyright: ignore[reportArgumentType]


def stock(key: str | None) -> str:
    """What a made body's mass and look come from (``""``: bought, or unknown)."""
    return METHODS[key].stock if key in METHODS else ""   # pyright: ignore[reportArgumentType]

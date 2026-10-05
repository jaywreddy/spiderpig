"""One catalog item per DIN 988 shim ring size and thickness (what is ordered).

The constructions and the planner work with a family (``shim_din988_4x8``: its ``t`` the
thicknesses a stack is made of); the BOM splits a family's lines per thickness
(:func:`hardware.bom.split_shims`) onto these items, ``<family>_t<thickness>``
(``shim_din988_4x8_t0p5``).
"""

from __future__ import annotations

from spiderpig.hardware.bom import SHIM_FAMILIES, shim_key
from spiderpig.hardware.catalog import CATALOG, Item, Offer, register

SHIM_OFFERS: dict[str, tuple[Offer, ...]] = {}
"""Offers per thickness item; a thickness with none takes its family's."""

for _fam in SHIM_FAMILIES:
    _it = CATALOG[_fam]
    for _t in _it.dims["t"]:
        _key = shim_key(_fam, float(_t))
        register(Item(
            _key, f"DIN 988 shim ring {_it.dims['id']:g} x {_it.dims['od']:g} x {float(_t):g} mm",
            "washer", SHIM_OFFERS.get(_key, _it.offers),
            dims={"id": _it.dims["id"], "od": _it.dims["od"], "t": float(_t)},
            notes=f"One thickness of {_it.name}.",
        ))

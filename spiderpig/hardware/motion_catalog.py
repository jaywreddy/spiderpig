"""Catalog data for an arm's drives (:mod:`construction.transmissions`): stepper motors,
GT2 timing pulleys and belts.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only). No price
is quoted from memory: the parts an arm buys have a marketplace listing first
(:data:`AMAZON`, search-result prices of 2026-10-12, ``verified=False``); the rest name the
listings, unpriced and ``verified=False``, until a fetch or an order confirms them (the
McMaster ones through ``spiderpig cad``, which also fetches their models).

**GT2** (2 mm pitch, Gates 2MGT's profile; the 3D printers' belt): a pulley's pitch
diameter is ``teeth x 2 / pi``, its outside diameter 0.508 mm under it (the belt's pitch
line is outside the teeth); a 6 mm belt runs on a 7 mm toothed width between flanges. The
small pulleys have a 16 mm hub 7 mm long with two M3 set screws (one on the shaft's flat);
the large one is a hubless plate pulley bolted to the link it turns: its four M3 holes on
a 30 mm circle are drilled and tapped by hand (a stock 80-tooth plate pulley has a plain
bore; a hubbed one would need the hub cut off). Closed-loop belts come in fixed lengths
(:data:`GT2_LOOPS`). A loop's spans
stretch like springs: :data:`GT2_SPECIFIC_STIFFNESS_N` per unit strain (the glass-fibre
6 mm belt's, a manufacturer's typical figure, unverified: measure the deflection on the
build).
"""

from __future__ import annotations

import math

from spiderpig.hardware.cad import CadRef, register_part_cad
from spiderpig.hardware.catalog import Item, Offer, register
from spiderpig.hardware.catalog import amazon_offer as _amazon
from spiderpig.hardware.catalog import unpriced_offer as _unpriced

_MCM = "McMaster-Carr"
_SO = "StepperOnline"

GT2_PITCH = 2.0
GT2_LOOPS: tuple[int, ...] = (158, 200, 232, 250, 280, 300, 350, 400, 500)
"""The GT2 6 mm closed-loop lengths stocked (mm): the common ones."""
GT2_SPECIFIC_STIFFNESS_N = 24000.0
"""A 6 mm glass-fibre GT2 belt's spring per unit strain (N): a span ``L`` mm long is
``c / L`` N/mm. A manufacturer's typical figure, unverified."""


def pitch_d(teeth: int) -> float:
    return teeth * GT2_PITCH / math.pi


def gt2_loop(length: int) -> str:
    return f"belt_gt2_6mm_{length}"


AMAZON: dict[str, Offer] = {
    "stepper_17hs4401": _amazon("B07LCK19D5", 3, 33.99, "STEPPERONLINE 3-pack, NEMA 17 "
                                "39 mm body, 45 N·cm, 1.5 A, 1 m cable and connector (the "
                                "17HS4401 class)"),
    "pulley_gt2_16t_5mm": _amazon("B077GMR328", 5, 6.99, "WINSINN GT2 16 teeth, 5 mm bore, "
                                  "6 mm belt, 5 pcs"),
    "pulley_gt2_80t_8mm": _amazon("B0GW4H3ML4", 1, 8.59, "GT2 80 teeth, 8 mm bore, 6 mm "
                                  "belt, 1 pc: check its hub and drill and tap the four M3 "
                                  "holes on the 30 mm circle (this item's notes)"),
    "pulley_gt2_60t_8mm": Offer("Amazon", "https://www.amazon.com/s?k=GT2+60+teeth+8mm+bore"
                                "+pulley+6mm", price_usd=7.0,
                                note="search: GT2 60 teeth, 8 mm bore, 6 mm belt; the price "
                                     "an estimate (2026-10-14). A hubbed one: its hub cut off "
                                     "(or a flat 'wheel' type), its four M3 holes drilled on "
                                     "the 30 mm circle as the 80-tooth's"),
    gt2_loop(232): _amazon("B0FRFGBS6J", 5, 8.19, "MECCANIXITY GT2 6 mm closed loop 232 mm, "
                           "5 pcs"),
    gt2_loop(200): _amazon("B07V6N32B1", 10, 7.99, "3Dman GT2 6 mm closed loop 200 mm, "
                           "10 pcs"),
    gt2_loop(158): Offer("Amazon", "https://www.amazon.com/s?k=GT2+closed+loop+158mm+6mm",
                         pack_qty=5, price_usd=8.0,
                         note="search: GT2 6 mm closed loop 158 mm (79 teeth), 5 pcs; the "
                              "price an estimate (2026-10-14)"),
}
"""The first (the BOM's) offer of the drive parts an arm buys, ahead of the item's own."""


def _offers(key: str, *own: Offer) -> tuple[Offer, ...]:
    return (AMAZON[key], *own) if key in AMAZON else own


register(Item(
    "stepper_17hs4401", "NEMA 17 stepper 17HS4401 (40 mm, 0.40 N·m, 1.7 A)", "motor",
    _offers("stepper_17hs4401",
            _unpriced(_SO, "https://www.omc-stepperonline.com/nema-17-bipolar-40mm-64oz-in-"
                           "45ncm-2a-4-lead-3d-printer-motor-17hs16-2004s1", "the 17HS4401 "
                           "family")),
    dims={"mass_g": 280.0, "material": "steel"}))
register(Item(
    "stepper_17hs19", "NEMA 17 stepper 17HS19-2004S1 (48 mm, 0.59 N·m, 2.0 A)", "motor",
    (_unpriced(_SO, "https://www.omc-stepperonline.com/nema-17-bipolar-59ncm-84oz-in-2a-42x48mm-"
                    "4-wires-w-1m-cable-connector-17hs19-2004s1", "17HS19-2004S1"),),
    dims={"mass_g": 390.0, "material": "steel"}))

PULLEYS = {
    # key: (teeth, bore, hub_d, hub_l, flange_d over the pitch diameter)
    "pulley_gt2_16t_5mm": (16, 5.0, 16.0, 7.0),
    "pulley_gt2_20t_5mm": (20, 5.0, 16.0, 7.0),
    "pulley_gt2_80t_8mm": (80, 8.0, 0.0, 0.0),
    "pulley_gt2_60t_8mm": (60, 8.0, 0.0, 0.0),
}
"""The GT2 pulleys the belt drive uses: the motors' (16 and 20 teeth, hubbed, 5 mm bore)
and the joints' (80 teeth, a plate pulley bolted to its link; 60 teeth for the compact
turret's yaw, :data:`construction.rotary.PRESETS` ``mt4_compact``)."""

for _key, (_z, _bore, _hub_d, _hub_l) in PULLEYS.items():
    _pd = pitch_d(_z)
    register(Item(
        _key, f"GT2 timing pulley {_z} teeth, 6 mm belt, {_bore:g} mm bore", "pulley",
        _offers(_key, _unpriced(_MCM, "https://www.mcmaster.com/products/timing-belt-"
                                      "pulleys/", "GT2 (2 mm pitch) pulleys")),
        dims={"teeth": _z, "pitch_d": round(_pd, 3), "od": round(_pd - 0.508, 3),
              "toothed_w": 7.0, "flange_t": 1.0, "flange_d": round(_pd + 4.0, 3),
              "bore": _bore, "hub_d": _hub_d, "hub_l": _hub_l, "material": "aluminium",
              "set_screws": "M3" if _hub_l else "",
              "bolt_pcd": 0.0 if _hub_l else 30.0, "bolts": 0 if _hub_l else 4}))
    register_part_cad(_key, CadRef(url="", sha256="", filename=f"{_key}.step",
                                   source="McMaster-Carr GT2 pulley", vendor="mcmaster"))

for _L in GT2_LOOPS:
    register(Item(
        gt2_loop(_L), f"GT2 closed-loop timing belt, 6 mm wide, {_L} mm", "belt",
        _offers(gt2_loop(_L), _unpriced(_MCM, "https://www.mcmaster.com/products/timing-"
                                              "belts/", "GT2 6 mm loops")),
        dims={"length": float(_L), "width": 6.0, "pitch": GT2_PITCH, "teeth": _L // 2,
              "thickness": 1.38, "material": "nylon"}))


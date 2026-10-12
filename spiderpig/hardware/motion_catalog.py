"""Catalog data for an arm's drives (:mod:`construction.transmissions`): stepper motors,
GT2 timing pulleys and belts, the capstan's cord.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only). No price
is quoted from memory: the offers name the listings, unpriced and ``verified=False``,
until a fetch or an order confirms them (the McMaster ones through ``spiderpig cad``, which
also fetches their models).

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

**The capstan's cord**: 1 mm braided Dyneema (UHMWPE, SK78), nearly no stretch and no
creep; its stiffness is :data:`CORD_EA_N` (an effective modulus of the braid, unverified).
"""

from __future__ import annotations

import math

from spiderpig.hardware.cad import CadRef, register_part_cad
from spiderpig.hardware.catalog import Item, Offer, register

_MCM = "McMaster-Carr"
_SO = "StepperOnline"

GT2_PITCH = 2.0
GT2_LOOPS: tuple[int, ...] = (158, 200, 232, 250, 280, 300, 350, 400, 500)
"""The GT2 6 mm closed-loop lengths stocked (mm): the common ones."""
GT2_SPECIFIC_STIFFNESS_N = 24000.0
"""A 6 mm glass-fibre GT2 belt's spring per unit strain (N): a span ``L`` mm long is
``c / L`` N/mm. A manufacturer's typical figure, unverified."""
CORD_EA_N = 20000.0
"""1 mm braided Dyneema's axial stiffness EA (N): an effective modulus of about 25 GPa on
its 0.785 mm² (the fibre's is about 100 GPa; a braid stretches its weave first).
Unverified."""
CORD_D = 1.0


def pitch_d(teeth: int) -> float:
    return teeth * GT2_PITCH / math.pi


def gt2_loop(length: int) -> str:
    return f"belt_gt2_6mm_{length}"


def _unpriced(vendor: str, url: str, note: str) -> Offer:
    return Offer(vendor, url, None, note=f"{note}; not fetched: unpriced")


register(Item(
    "stepper_17hs4401", "NEMA 17 stepper 17HS4401 (40 mm, 0.40 N·m, 1.7 A)", "motor",
    (_unpriced(_SO, "https://www.omc-stepperonline.com/nema-17-bipolar-40mm-64oz-in-45ncm-"
                    "2a-4-lead-3d-printer-motor-17hs16-2004s1", "the 17HS4401 family"),),
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
}
"""The GT2 pulleys the belt drive uses: the motors' (16 and 20 teeth, hubbed, 5 mm bore)
and the joints' (80 teeth, a plate pulley bolted to its link)."""

for _key, (_z, _bore, _hub_d, _hub_l) in PULLEYS.items():
    _pd = pitch_d(_z)
    register(Item(
        _key, f"GT2 timing pulley {_z} teeth, 6 mm belt, {_bore:g} mm bore", "pulley",
        (_unpriced(_MCM, "https://www.mcmaster.com/products/timing-belt-pulleys/", "GT2 "
                   "(2 mm pitch) pulleys"),),
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
        (_unpriced(_MCM, "https://www.mcmaster.com/products/timing-belts/", "GT2 6 mm "
                   "loops"),),
        dims={"length": float(_L), "width": 6.0, "pitch": GT2_PITCH, "teeth": _L // 2,
              "thickness": 1.38, "material": "nylon"}))

register(Item(
    "tube_al_6x5", "Aluminium tube 6 mm OD x 0.5 mm wall (5 mm ID), per 100 mm", "spacer",
    (_unpriced("Amazon", "https://www.amazon.com/s?k=6mm+od+5mm+id+aluminum+tube",
               "6 x 5 mm round tube"),),
    dims={"od": 6.0, "id": 5.0, "length": 100.0, "material": "aluminium"},
    notes="The capstan's sleeve: cut to the drum's core height and bonded over the motor's "
          "D-flat shaft (retaining compound), so the cord wraps a round surface."))
register(Item(
    "dyneema_1mm", "1 mm braided Dyneema (UHMWPE) cord, per metre", "cord",
    (_unpriced("GigaParts", "https://www.gigaparts.com/", "Mastrant-M 1 mm (100 % DM20 "
               "core, polyester cover), SKU ZCB13112; out of stock 2026-10-12"),
     _unpriced("Amazon", "https://www.amazon.com/s?k=1mm+dyneema+braided+line", "1 mm SK78 "
               "braid: creeps (see notes)")),
    dims={"d": CORD_D, "ea_n": CORD_EA_N, "material": "nylon"},
    notes="The capstan's cord: wraps the motor's capstan and is anchored at both ends of the "
          "drum. Buy DM20 (Dyneema's creep-resistant grade): at 300 MPa and 30 C, SK75 "
          "creeps 0.02 %/day, SK78 0.006, DM20 0.00007 (DSM via riggingdoctor.com); "
          "SK-grade braid creeps enough to need re-tensioning (arXiv 1806.10632). EA "
          "unverified."))

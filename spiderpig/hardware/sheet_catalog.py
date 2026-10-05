"""Sheet stock by material and thickness, and the thin washers and shims that fill a
clearance gap (:mod:`spiderpig.materials` reads them).

Two materials, from two services (the user's direction of 2026-10-04): **cast acrylic** by
default, **aluminium** (5052-H32, or 6061-T6 where the stress needs it) only where acrylic
can't take the load: the frame plates, the crank's plates and the Klann variants' foot
links. Delrin is out. The plates go to SendCutSend (aluminium; acrylic too) or Ponoko
(acrylic: no order minimum, the thin sheets).

Each sheet item's ``dims`` carry what the build and the audit read: ``thickness`` (mm, the
nominal), ``material`` (a :data:`hardware.mass.DENSITY` key), ``density`` (g/cm^3),
``yield_mpa`` (the allowable the strength check uses), ``service``, and the service's cut
rules: ``min_hole`` (mm), ``edge_t`` (least hole-to-edge distance in thicknesses),
``min_part`` (mm, the smaller and the larger side of the smallest part it cuts),
``edge_mm`` (the least web anywhere: Ponoko's minimum feature), ``corner_r`` (mm, how
round it cuts an inside corner), ``kerf_mm`` (the kerf the DXF compensates for:
:func:`layout.sheet_kerf`) and ``sheet_mm`` (the blank the DXF sheets are packed on).
Fetched 2026-10-04:

* SendCutSend 5052-H32 aluminium (sendcutsend.com/materials/5052-aluminum): .040, .063,
  .080, .090, .100, .125 in (and thicker); "Minimum hole size: Equal to material
  thickness"; "leave at least 2x thickness between a hole and an edge"; min part .25 x
  .375 in; max 30 x 44 in. 6061-T6: the same rules (its page lists .125 in for laser).
* SendCutSend acrylic (sendcutsend.com/materials/acrylic): .063, .118 (3.0 mm) in and up;
  min part .187 x .375 in; "at least 1.5x the material thickness between a hole and the
  nearest edge"; cutting tolerance +/- .009 in.
* Ponoko clear acrylic (ponoko.com/materials/clear-acrylic): 1.0, 1.5, 2.0, 3.0 mm and up;
  min part 6.0 mm; min hole / feature 1.0 mm; kerf 0.2 mm ("Kerf width: 0.20mm", one
  figure for every thickness, page re-read 2026-10-04); Ponoko's laser follows the line
  in acrylic (its help page "How much material does the laser burn away?": no offset
  except on metal), so the DXF compensates: ``kerf_mm`` 0.2.
* SendCutSend compensates for the kerf itself (its FAQ "Do I need to compensate for kerf
  in my design?": no, draw the part at size; a compensated file comes back off size), so
  its sheets' ``kerf_mm`` is 0.

The inside-corner radius SendCutSend cuts in aluminium (0.8 mm) is the joinery plan's
figure, not from a page; the laser's in acrylic is about its kerf (0.1 mm). Prices are
estimates (neither service publishes a table: a DXF upload quotes), per 300 x 300 mm of
nested parts.

``shim_din988_3x6`` / ``shim_din988_6x12`` and ``ptfe_washer_3x6x0p5`` join the 4 mm ones
(:mod:`hardware.fastener_catalog`) and the 6 mm PTFE washer (:mod:`hardware.crank_catalog`)
as the washers an axle carries through a clearance gap (:func:`materials.washer_stack`).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

IN = 25.4
SCS_AL = "https://sendcutsend.com/materials/5052-aluminum/"
SCS_6061 = "https://sendcutsend.com/materials/6061-aluminum/"
SCS_ACRYLIC = "https://sendcutsend.com/materials/acrylic/"
PONOKO_ACRYLIC = "https://www.ponoko.com/materials/clear-acrylic"
FETCHED = "service rules fetched 2026-10-04; price an estimate (quote by DXF upload)"

SCS_RULES_AL = {"service": "SendCutSend", "min_part": (0.25 * IN, 0.375 * IN),
                "edge_t": 2.0, "corner_r": 0.8, "metal": True, "kerf_mm": 0.0}
PONOKO_RULES_ACRYLIC = {"service": "Ponoko", "min_hole": 1.0, "min_part": (6.0, 6.0),
                        "edge_t": 0.0, "edge_mm": 1.0, "corner_r": 0.1, "metal": False,
                        "kerf_mm": 0.2}


def _al(key: str, inch: float, alloy: str = "5052", price: float | None = None) -> Item:
    t = round(inch * IN, 3)
    name = f"{alloy} aluminium sheet {inch:.3f} in ({t:g} mm), laser cut"
    url = SCS_AL if alloy == "5052" else SCS_6061
    temper = "H32" if alloy == "5052" else "T6"
    return Item(
        key, name, "sheet",
        (Offer("SendCutSend", url, note=f"{alloy}-{temper} {inch:.3f} in; {FETCHED}",
               price_usd=price),),
        dims={"thickness": t, "material": "aluminium", "alloy": f"{alloy}-{temper}",
              "density": 2.68 if alloy == "5052" else 2.70,
              "yield_mpa": 193.0 if alloy == "5052" else 276.0,
              "min_hole": t, "sheet_mm": (300.0, 300.0), **SCS_RULES_AL},
        notes=f"{alloy}-{temper}: yield {193 if alloy == '5052' else 276} MPa (ASM); "
              "minimum hole = thickness, 2 t hole-to-edge, min part 6.35 x 9.5 mm "
              "(SendCutSend).",
    )


def _acrylic(key: str, t: float, price: float | None = None) -> Item:
    return Item(
        key, f"{t:g} mm clear cast acrylic sheet, laser cut", "sheet",
        (Offer("Ponoko", PONOKO_ACRYLIC, verified=True, price_usd=price,
               note=f"{t:g} mm clear acrylic (1.0, 1.5, 2.0, 3.0 mm listed); {FETCHED}"),),
        dims={"thickness": t, "material": "acrylic", "density": 1.19, "yield_mpa": 50.0,
              "sheet_mm": (300.0, 300.0), **PONOKO_RULES_ACRYLIC},
        notes="Thin acrylic for clearance-gap fillers (a plate stack the gap splits).",
    )


AL_THIN: dict[str, float] = {"al5052_1mm": 0.040, "al5052_1p6mm": 0.063,
                             "al5052_2mm": 0.080, "al5052_2p3mm": 0.090,
                             "al5052_2p5mm": 0.100}
ACRYLIC_THIN: dict[str, float] = {"acrylic_1mm": 1.0, "acrylic_1p5mm": 1.5, "acrylic_2mm": 2.0}
AL6061_THIN: dict[str, float] = {"al6061_1p6mm": 0.063, "al6061_2mm": 0.080,
                                 "al6061_2p5mm": 0.100}
"""SendCutSend's thinner 6061-T6 (its 6061 page lists .040, .063, .080, .100, .125 in and up;
no .090), for the crank's hex-pocket plates (2026-10-04); price an estimate."""

register(
    _al("al5052_3p2mm", 0.125, price=28.0),
    _al("al6061_3p2mm", 0.125, alloy="6061", price=32.0),
    *(_al(k, v, price=18.0) for k, v in AL_THIN.items()),
    *(_al(k, v, alloy="6061", price=21.0) for k, v in AL6061_THIN.items()),
    *(_acrylic(k, v, price=9.0) for k, v in ACRYLIC_THIN.items()),
    Item("epoxy_2part", "Two-part slow-cure structural epoxy (e.g. J-B Weld Original or "
         "Loctite EA E-30CL), 2 x 25 ml", "adhesive",
         (Offer("Amazon", "https://www.amazon.com/s?k=two+part+slow+cure+epoxy",
                note="search; any slow (30 min+) two-part structural epoxy; unverified"),),
         dims={"shear_mpa": 10.0},
         notes="Bonds aluminium to aluminium or acrylic (abrade both faces first); about "
               "10 MPa lap shear on abraded aluminium (estimate; the TDS says 15-25 MPa "
               "on etched steel). Slow cure: 24 h before load."),
    Item("shim_din988_3x6", "DIN 988 shim ring 3 x 6 mm (0.1 / 0.2 / 0.3 / 0.5 / 1.0 mm)",
         "washer",
         (Offer("Accu", "https://accu-components.com/us/shim-washers/",
                note="DIN 988 shim rings 3 x 6; part number per thickness not confirmed"),),
         dims={"id": 3.0, "od": 6.0, "t": (0.1, 0.2, 0.3, 0.5, 1.0)},
         notes="Fills a clearance gap on a 3 mm rod."),
    Item("ptfe_washer_3x6x0p5", "PTFE flat washer 3.2 x 6 x 0.5 mm", "washer",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/products/ptfe-washers/",
                note="PTFE washers for M3, 0.5 mm; part number not confirmed"),),
         dims={"id": 3.2, "od": 6.0, "t": 0.5}),
    Item("shim_din988_6x12", "DIN 988 shim ring 6 x 12 mm (0.1 / 0.2 / 0.3 / 0.5 / 1.0 mm)",
         "washer",
         (Offer("Accu", "https://accu-components.com/us/shim-washers/",
                note="DIN 988 shim rings 6 x 12; part number per thickness not confirmed"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/products/shims/",
                note="ring shims for 6 mm shafts")),
         dims={"id": 6.0, "od": 12.0, "t": (0.1, 0.2, 0.3, 0.5, 1.0)},
         notes="Fills a clearance gap on a 6 mm standoff or crank bolt."),
)

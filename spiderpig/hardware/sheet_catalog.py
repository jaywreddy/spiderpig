"""Sheet stock by material and thickness, and the thin washers and shims that fill a
clearance gap (:mod:`spiderpig.materials` reads them).

Two materials, from two services (the user's direction of 2026-10-04): **cast acrylic** by
default, **aluminium** (5052-H32, or 6061-T6 where the stress needs it) only where acrylic
can't take the load: the frame plates, the crank's plates and the Klann variants' foot
links. Delrin is out. The plates go to SendCutSend: the aluminium, and the 3 mm acrylic
since 2026-10-08 (the user's decision: one service, its rules in
:data:`hardware.parts.SCS_RULES_ACRYLIC`); Ponoko cuts the thin acrylic and, selected,
the 3 mm (``acrylic_3mm_ponoko``).

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
  nearest edge"; cutting tolerance +/- .009 in. Its .118 in spec table (rendered
  2026-10-08): min hole .047 in, min bridge .053 in, min hole to edge .035 in
  (:data:`hardware.parts.SCS_RULES_ACRYLIC`).
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
as the washer families of :data:`materials.WASHERS`: their ODs size what an axle carries
through a clearance gap (a printed ring on the pivots and the hex crank:
:func:`construction.pivots.common.gap_washers`; bought washers and shims only on the round
or M6 crankpins, :func:`materials.washer_stack`) and the clamped shims are bought per
thickness (:mod:`hardware.shims`).
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


SCS_MAX_MM = (44.0 * IN, 30.0 * IN)
"""SendCutSend's largest part (44 x 30 in): a decanting cradle's frame plates and links are
bigger than the 300 x 300 mm the walkers' sheets are drawn on; SendCutSend prices per part,
so the sheet a layout draws is only a page."""


ALLOYS = {
    # alloy: (temper, its page, density g/cm^3, yield MPa, the walkers' cut rate USD/cm^2)
    "5052": ("H32", SCS_AL, 2.68, 193.0, 0.20),
    "6061": ("T6", SCS_6061, 2.70, 276.0, 0.40),
    # SendCutSend's datasheet (materials?sku=ALU7075-190, fetched 2026-10-13): .125, .190,
    # .250 in; yield 69 ksi, 81 ksi ultimate, 7.9 % elongation, 176.26 lb/ft^3; no cut
    # rate quoted (its cutting is left unpriced: quote at upload)
    "7075": ("T6", "https://sendcutsend.com/materials/7075-aluminum/", 2.82, 476.0, None),
}


def _al(key: str, inch: float, alloy: str = "5052", price: float | None = None,
        sheet_mm: tuple[float, float] = (300.0, 300.0)) -> Item:
    t = round(inch * IN, 3)
    name = f"{alloy} aluminium sheet {inch:.3f} in ({t:g} mm), laser cut"
    temper, url, density, yield_mpa, rate = ALLOYS[alloy]
    return Item(
        key, name, "sheet",
        (Offer("SendCutSend", url, note=f"{alloy}-{temper} {inch:.3f} in; {FETCHED}",
               price_usd=price),),
        dims={"thickness": t, "material": "aluminium", "alloy": f"{alloy}-{temper}",
              "density": density, "yield_mpa": yield_mpa,
              "min_hole": t, "sheet_mm": sheet_mm, **SCS_RULES_AL,
              # the cutting, material included (hardware.bom.cut_estimate, calibrated on
              # SendCutSend's live quotes of 2026-10-08): per cm^2 of part, least per part
              **({"cut_usd_cm2": rate, "cut_min_usd": 2.20} if rate is not None else {})},
        notes=f"{alloy}-{temper}: yield {yield_mpa:g} MPa "
              f"({'SendCutSend' if alloy == '7075' else 'ASM'}); minimum hole = thickness, "
              "2 t hole-to-edge, min part 6.35 x 9.5 mm (SendCutSend).",
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


SCS_PLY = "https://sendcutsend.com/materials/baltic-birch-plywood/"


def _plywood(key: str, inch: float, mm: float) -> Item:
    """SendCutSend's Baltic birch plywood, CNC routed (its 9.0 and 12.0 mm; the 3.2 and 6.3
    mm are lasered). Its routed rules from the material's spec table (the page's
    ``materials?sku=`` data, fetched 2026-10-10): tolerance +-.005 in, thickness
    +.016/-.024 in (9 mm), min hole .125 in (the bit), hole to edge .106 in, min bridge .354
    in, min part 1 x 2 in, max 44 x 44 in (the routing guideline says 44 x 30: drawn on
    that); a routed inside corner is the bit's .063 in radius. Grade B/BB, identical faces;
    43.2 lb/ft^3. No price is published (a DXF upload quotes): the BOM leaves its cutting
    unpriced."""
    return Item(
        key, f"Baltic birch plywood {mm:g} mm ({inch:.3f} in), CNC routed", "sheet",
        (Offer("SendCutSend", SCS_PLY,
               note=f"Baltic birch, grade B/BB, {inch:.3f} in routed; rules fetched "
                    "2026-10-10; no published price: quote at upload"),),
        dims={"thickness": mm, "material": "plywood", "density": 0.692,
              # (its strength isn't checked: a base board on the table, no link)
              "yield_mpa": 30.0, "service": "SendCutSend", "process": "CNC router",
              "min_hole": 0.125 * IN, "edge_t": 0.106 * IN / mm, "edge_mm": 0.354 * IN,
              "corner_r": 0.063 * IN, "min_part": (1.0 * IN, 2.0 * IN), "metal": False,
              "kerf_mm": 0.0, "sheet_mm": SCS_MAX_MM},
        notes="Baltic birch plywood (SendCutSend: 'not weather-proof'; routed edges may show "
              "witness marks and light burning: sand and oil it).",
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
    # a decanting cradle's frame and links, and its connecting rod (its buckling): parts
    # up to SendCutSend's largest
    _al("al5052_3p2mm", 0.125, price=28.0, sheet_mm=SCS_MAX_MM),
    _al("al5052_4p8mm", 0.187, price=40.0, sheet_mm=SCS_MAX_MM),
    # its gearbox's riser discs, and its keyed crank arm
    _al("al5052_6p35mm", 0.250, price=55.0, sheet_mm=SCS_MAX_MM),
    _al("al6061_6p35mm", 0.250, alloy="6061", price=60.0, sheet_mm=SCS_MAX_MM),
    # the B cradle's most loaded levers
    _al("al6061_4p8mm", 0.187, alloy="6061", price=45.0, sheet_mm=SCS_MAX_MM),
    _al("al7075_4p8mm", 0.190, alloy="7075", sheet_mm=SCS_MAX_MM),
    _al("al6061_3p2mm", 0.125, alloy="6061", price=32.0),
    # a decanting cradle's base board (2026-10-13, the user's "the bottom board in wood")
    _plywood("birch_9mm", 0.354, 9.0),
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
         notes="Clamped shims: under a horn screw's head, at a one-piece pillar's or a frame "
               "tie's end (ordered per thickness; the 1.0 and 0.5 mm as stock 0.5 mm washers, "
               "bom.SHIM_AS). Its OD also sizes the printed gap rings on a 3 mm shaft."),
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
         notes="Gap washers on a round 6 mm crankpin (--crank bolt_round, the acrylic M6 "
               "crank); the default hex crank and the pillars print theirs (its OD sizes "
               "them, materials.WASHERS)."),
)

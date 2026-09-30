"""Catalog data for the metal-shaft pivot constructions (:mod:`construction.pivots`).

Appended to the catalog from the end of :mod:`hardware.parts` (new items
only; the screw tables there are unchanged). Keys follow the same helpers:
``shcs("3", 45) -> "m3_shcs_45"``.

* ``m3_shcs_45`` / ``m3_shcs_50``: the lengths a bolt pillar needs through a
  quad's 36 mm stack (:data:`BOLT_LENGTHS` is every M3 SHCS length the bolt
  construction may pick).
* ``e_clip_din6799_2p3``: the grooved alternative to a push-on Starlock for
  a 3 mm shaft; listed for reference, not used (a hobbyist won't groove a
  3 mm rod).
* ``m3_nylock_thin``: ISO 10511 low nut (3.9 mm; DIN 985 is 4.0), the
  same claim.

Sources: the 2026-09-29 research notes (``joinery.json``) and the pages
cited per offer. Offers are ``verified=True`` only where the page was
fetched and showed the product; McMaster pages don't render to a fetcher.
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register
from spiderpig.hardware.fasteners import CLEARANCE, screw, shcs

LONG_SHCS_LENGTHS: tuple[float, ...] = (45, 50)
BOLT_LENGTHS: tuple[float, ...] = tuple(
    sorted(set(screw("shcs", "3").lengths) | set(LONG_SHCS_LENGTHS)))

_dk, _k = screw("shcs", "3").head_d, screw("shcs", "3").head_h
for _L in LONG_SHCS_LENGTHS:
    register(Item(
        shcs("3", _L), f"M3 x {_L:g} mm socket head cap screw", "fastener",
        (Offer("McMaster-Carr", "https://www.mcmaster.com/products/socket-head-screws/",
               pack_qty=50, note=f"M3 x {_L:g} mm, pick from the listing (91290A series); part "
               "number not confirmed"),
         Offer("Amazon", f"https://www.amazon.com/s?k=M3+x+{_L:g}+mm+socket+head+cap+screw",
               note="search: M3 ISO 4762, 50-packs")),
        dims={"d": 3.0, "length": float(_L), "head_d": _dk, "head_h": _k,
              "clearance_d": CLEARANCE["3"]},
        notes="ISO 4762 / DIN 912; thread length 18 mm, plain shank above",
    ))

register(
    Item("e_clip_din6799_2p3", "E-clip DIN 6799 size 2.3 (for 3-4 mm shafts)", "clip",
         (Offer("Accu", "https://accu-components.com/us/e-clips/69319-HETC-2-3-A2", "HETC-2-3-A2",
                verified=True, note="A2 stainless; sold singly"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/products/e-style-retaining-rings/",
                note="E-style external retaining ring for 3 mm shafts")),
         dims={"shaft_d": 3.0, "groove_d": 2.3, "od": 6.3, "t": 0.6},
         notes="Needs a 2.3 mm groove in the shaft: not for cut-to-length rod. OD from the "
               "DIN 6799 table for size 2.3 (Accu lists it as 2.3 x 1.94); not fetched."),
    Item("m3_nylock_thin", "M3 low nylon-insert lock nut (ISO 10511)", "nut",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/products/locknuts/specifications-met~"
                "iso-10511/", note="pick M3 from the ISO 10511 listing"),
          Offer("BelMetric", "https://belmetric.com/nylon-locking-nut-low-class-4-steel-iso-10511/",
                note="class 4 steel; size picked on the page")),
         dims={"af": 5.5, "h": 3.9, "d": 3.0},
         notes="Only 0.1 mm lower than DIN 985: it needs the same two layers."),
)

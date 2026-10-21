"""``ORDER.md``: the build's shopping list, one section per place to order from.

:mod:`hardware.bom` lists what the robot needs; this turns it into the orders: one cart per
vendor (each line its direct product page, SKU, packs and price), one upload per cutting
service (the per-part DXFs of :func:`layout.save_parts` with their quantities), the prints
per filament (:func:`build.export_prints`' rows), and what to check before ordering.
"""

from __future__ import annotations

from collections import defaultdict

from spiderpig.hardware.bom import (
    CRADLE_CUT_FITS,
    CUT_SOURCE,
    ON_HAND,
    bought,
    deburr_least,
    file_estimates,
    hand_deburred,
)
from spiderpig.hardware.catalog import get
from spiderpig.hardware.electronics import HV_SHORT, HV_WARNING

LINE_WARNINGS = {"lipo_2s_450": HV_SHORT, "ip2326_charger": HV_SHORT}
"""What a cart line says beside its item (and :data:`LINE_NOTES` under "Before you order")."""
LINE_NOTES = {
    "lipo_2s_450": HV_WARNING,
    "nrv050_30": "**Phidgets' cart** (the NRV050 gearbox and its output shaft, USD 99.35): "
                 "made in China, so US duty is collected at checkout (UPS Global Checkout, "
                 "phidgets.com/docs/U.S._Tariffs): 40 % on the gearbox (HTS 8483.40.50.10) and "
                 "50 % on the shaft (8483.10.50.00), USD 40.43 (tariffs.wove.com, 2026-10-15), "
                 "so USD 139.78 before UPS's shipping, the brokerage and the checkout's own "
                 "fee, which show only at checkout (an account's: no guest checkout). The "
                 "example on that page shows a brokerage of USD 31.99 on an order (its customs "
                 "charges' picture, viewed 2026-10-20): about USD 172 landed before shipping. "
                 "Run the cart before ordering the rest. Not in the total.",
}

SERVICE_ORDER_URL = {
    "SendCutSend": "https://app.sendcutsend.com/customer#/quote",
    "Ponoko": "https://www.ponoko.com/designs",
}
HOME_HEADS = {
    "Carvera": "Your Makera Carvera, at home (not an upload: the panels are in the carts "
               "above, the files in this list and the nests of a panel each)",
}
"""ORDER.md's cut list's heading for the parts a machine at home cuts
(:func:`layout.sheet_route`), in place of a service's upload."""
SERVICE_NOTES = {
    "Carvera": "each panel sawn to about 238 mm wide first (the bed is 360 x 240 mm), taped "
               "down flat on the wasteboard with double-sided tape over its whole face (or "
               "screwed outside the parts); a 1/8 in single-flute end mill for the outlines "
               "(the files are nominal: Makera CAM offsets the bit), tabs left on every part "
               "and sanded flush after; a 90° chamfer bit for each cleat's lead-in; the "
               "nests (`*_sheet_Carvera_*.dxf`) lay out one panel each",
    "SendCutSend": "one part per file, units mm at upload, quantity per file as below",
    "Ponoko": "one part per file, mm, the blue CUT layer mapped to cut; 3 mm sheet arrives "
              "2.49-3.51 mm thick: measure it and rebuild with --thickness if it is off "
              "by more than a few percent",
}
ACRYLIC_NOTES = {
    "SendCutSend": "its 3 mm acrylic arrives 2.24-3.46 mm thick: measure it and rebuild with "
                   "--thickness if it is off by more than a few percent",
}
"""What a service's note adds where the order has acrylic to cut."""
SERVO_BOX = ("m2_self_tap_6", "servo_bus_cable_5264")
"""What comes in the servos' boxes (:data:`hardware.bom.ON_HAND`)."""
COMPENSATES = {"SendCutSend": True, "Ponoko": False, "Carvera": True}
"""Whether a service offsets its cut path for the kerf itself (SendCutSend does; Ponoko
doesn't in acrylic): what the files' own kerf (:func:`layout.sheet_kerf`, ``--kerf``)
must be."""
MEASURED_PINS = ("pivot_pin8_", "rod6061ha_516_", "rod6061ha_38_")
"""Catalog keys (by prefix) of the pins whose length is the stack of cut parts they go
through: MISUMI's CCGH8 (:mod:`construction.pivots.pivot_pin`, made to the 0.1 mm), ordered
last, once the parts are measured; and since simplification round 1 (2026-10-21) the
sticks of hard-anodized 6061 a cradle's pins are cut from at home
(:mod:`construction.pivots.rod_pin`), each to its measured stack (:data:`PIN_TAKE_UP_MM`)."""
PIN_TAKE_UP_MM = 0.3
"""mm over the measured stack a made-to-length pin's length is ordered at (design review
round 2, M5: SendCutSend's sheets run up to +0.027 in thick at 0.250 in, more than the
0.37-0.45 mm the pins' play allows: ``cradle_report.take_up``)."""
MEASURED_PIN_NOTE = (
    "Order the made-to-length pivot pins ({pins}) last, once the cut parts have arrived: "
    "SendCutSend's sheets come up to +0.027 in (0.69 mm) over at 0.250 in and +0.009 in "
    "(0.23 mm) at 0.187 in (its spec tables), more than a pin's play. Caliper each link at "
    "its pin holes, add up each pin's stack (its links and the printed spacers between "
    "them, as the assembly guide stacks them) and order L, the length between the rings' "
    "grooves, at that stack plus {take:g} mm (made to 0.1 mm, +0.1/-0). The clevis pins are "
    "a stock length: where a clip won't seat, sand the printed spacer under the pin's head "
    "by the excess.")
"""ORDER.md's line for a build with :data:`MEASURED_PINS` (a walker's or an older cradle's
CCGH8)."""
ROD_PIN_NOTE = (
    "The pins ({pins}) are cut to length at home, once the cut parts have arrived: "
    "SendCutSend's sheets come up to +0.027 in (0.69 mm) over at 0.250 in and +0.009 in "
    "(0.23 mm) at 0.187 in (its spec tables), more than a pin's play. Caliper each link at "
    "its pin holes, add up each pin's stack (its links and the printed spacers between "
    "them, as the assembly guide stacks them) and cut each pin at that stack plus {take:g} mm "
    "(+0.1/-0, its ends faced square; a clip pin's cross hole at its usable length plus "
    "{take:g}): the assembly guide's \"Pins cut to their stacks\". A 1 ft stick makes ten "
    "pins with its saw cuts. Where a clip won't seat, sand the printed spacer under the "
    "pin's head by the excess.")
"""ORDER.md's line for a cradle's pins cut at home from their sticks (:data:`MEASURED_PINS`'
``rod6061ha_`` keys, simplification round 1)."""


def kerf_note(service: str, kerfs: set[float]) -> str:
    """What the files of ``service`` carry for the kerf (``kerfs``: their rows' ``kerf_mm``),
    and a warning where that is wrong for the service."""
    k = ", ".join(f"{x:g}" for x in sorted(kerfs))
    if COMPENSATES.get(service):
        if any(x > 0 for x in kerfs):
            return (f"**the files carry a {k} mm kerf offset (--kerf), and {service} "
                    "compensates the kerf itself: the parts would come back offset twice. "
                    "Rebuild without --kerf before uploading**")
        return f"{service} compensates the kerf (the files are nominal size)"
    if service in COMPENSATES and not any(x > 0 for x in kerfs):
        return (f"**the files are nominal size and {service} doesn't compensate the kerf: "
                "the parts come back a kerf small. Rebuild without --kerf 0**")
    if service in COMPENSATES:
        return f"the files carry a {k} mm kerf offset ({service} doesn't compensate it)"
    return (f"the files carry a {k} mm kerf offset: leave your laser's own kerf "
            "compensation off, or rebuild with --kerf 0 and use it")




def estimate(row) -> tuple[float, str] | None:
    """An unpriced line's cost from the first priced alternative offer of its item (whole
    packs of that offer): ``(usd, vendor)``, ``None`` when no offer has a price."""

    try:
        offers = get(row.key).offers[1:]
    except KeyError:
        return None
    for o in offers:
        if o.price_usd is not None:
            usd = o.buy(row.qty)[1]
            assert usd is not None  # a priced offer prices whole packs (Offer.buy)
            return usd, o.vendor
    return None


def _money(v: float | None) -> str:
    return "" if v is None else f"${v:.2f}"


def order_markdown(bom, laser_rows: list[dict], print_rows: list[dict], title: str = "",
                   build_dir: str = ".") -> str:
    """The shopping list of a build: carts per vendor, uploads per service, prints."""
    from spiderpig.layout import sheet_service

    # a sheet a service cuts is its upload (the service sells the stock); one no service
    # cuts (plywood: cut it yourself) is bought like any part
    services = {r["sheet"] for r in laser_rows if sheet_service(r["sheet"])}
    carts: dict[str, list] = defaultdict(list)
    shared: dict[str, list] = defaultdict(list)     # a pack's lead row -> the rows it covers
    for r in bom.purchased:
        if r.same_pack_as:
            shared[r.same_pack_as].append(r)
            continue
        if r.key in services or not bought(r.key):     # (the BOM's total leaves out the same)
            continue
        carts[r.vendor or "(no vendor)"].append(r)
    on_hand = [r for r in bom.purchased if r.key in ON_HAND]
    source = getattr(bom, "cut_source", CUT_SOURCE)
    lines = [f"# Order list{': ' + title if title else ''}", "",
             "Everything to buy, cut and print for this build. Each purchase links the "
             "vendor's product page for the exact part (SKU / part number given); quantities "
             "are rounded up to whole packs. Links marked *page not fetched* are part numbers "
             "confirmed from the vendor's own tables, a datasheet or a mirror (McMaster-Carr, "
             "DigiKey and Mouser refuse scripted fetches): check the part on the page as you "
             "add it.", ""]
    total = 0.0
    unpriced = 0
    estimated = 0.0
    lines += ["## Buy (one cart per vendor)", ""]
    for vendor in sorted(carts, key=lambda v: (-len(carts[v]), v)):
        rows = carts[vendor]
        cost = sum(r.cost_usd or 0.0 for r in rows)
        total += cost
        lines += [f"### {vendor} ({len(rows)} line{'s' if len(rows) > 1 else ''}"
                  f"{', ' + _money(cost) if cost else ''})", "",
                  "| buy | item | SKU / part no. | need | est. | link |",
                  "|---:|---|---|---:|---:|---|"]
        for r in sorted(rows, key=lambda r: r.name):
            buy = f"{r.packs} × {r.pack_qty}" if r.pack_qty > 1 else f"{r.packs}"
            need = f"{r.qty:g}" if r.qty >= 1 else f"{r.qty:.3g} of one"
            also = "".join(f"; also {o.name}: need {o.qty:g}" for o in shared.get(r.name, ()))
            name = r.name + (f" (the same pack{also})" if also else "")
            if r.key in LINE_WARNINGS:
                name += f" — **{LINE_WARNINGS[r.key]}**"
            if r.key.startswith(MEASURED_PINS):
                name += (" — **order once the cut parts are measured** (L: the measured "
                         f"stack + {PIN_TAKE_UP_MM:g} mm; see Before you order)"
                         if r.key.startswith("pivot_pin8_") else
                         " — **each pin cut at home to its measured stack** + "
                         f"{PIN_TAKE_UP_MM:g} mm (see Before you order)")
            est = _money(r.cost_usd)
            if r.cost_usd is None:
                unpriced += 1
                if (got := estimate(r)) is not None:
                    estimated += got[0]
                    est = f"≈{_money(got[0])} ({got[1]})"
            link = f"[product page]({r.url})" + ("" if r.verified else " *page not fetched*")
            lines.append(f"| {buy} | {name} | {r.sku} | {need} | {est} | {link} |")
        lines.append("")
    lines += [f"Purchases: **{_money(total)}** at the listed pack prices ({unpriced} line(s) "
              "unpriced: the vendor shows its price only in the cart or to an account"
              + (f"; ≈{_money(estimated)} more estimated from another vendor's price for the "
                 "same part" if estimated else "")
              + "), before shipping and the cut parts.", ""]
    cut = {c.service: 0.0 for c in getattr(bom, "cutting", ())}
    for c in getattr(bom, "cutting", ()):
        cut[c.service] += c.usd or 0.0
    left_out = getattr(bom, "unpriced_cuts", "")     # (a cradle's: what the total leaves out)
    if cut:
        lines += [f"Cutting (the material included): **≈{_money(sum(cut.values()))}** ("
                  + ", ".join(f"{s} ≈{_money(v)}" for s, v in cut.items())
                  + f"; {source}). **Total ≈{_money(total + sum(cut.values()))}**, "
                  "before shipping." + (f" **{left_out}**" if left_out else ""), ""]
    if on_hand:
        boxes = any(r.key in SERVO_BOX for r in on_hand)
        lines += [f"## From the shop{' and the servo boxes' if boxes else ''} (on hand, not "
                  "ordered)", ""] + [
            f"* {r.name}: {r.where[0] if len(r.where) == 1 else f'{len(r.where)} uses'}"
            for r in on_hand] + [""]

    fitted: dict[str, float] = {}       # a cradle's files' estimates, by their first part
    rough: set[str] = set()             # ... and those that arrive as cut
    if getattr(bom, "cut_ranges", None) is not None:
        fitted = file_estimates(getattr(bom, "made", []), CRADLE_CUT_FITS)
        rough = {m.name for m in hand_deburred(getattr(bom, "made", []))}
    if laser_rows:
        lines += ["## Cut (upload per service)", ""]
        by_service: dict[str, list[dict]] = defaultdict(list)
        for r in laser_rows:
            by_service[r["service"]].append(r)
        for service, rows in by_service.items():
            url = SERVICE_ORDER_URL.get(service, "")
            head = HOME_HEADS.get(service) or (service if url or service not in ("", "any") else (
                "your own laser (or any service): the sheet stock is in the carts above"))
            lines += [f"### {head}" + (f": [upload and quote]({url})" if url else ""), ""]
            kerfs = {float(r.get("kerf_mm") or 0.0) for r in rows}
            acrylic = any("acrylic" in str(r.get("sheet", "")) for r in rows)
            note = "; ".join(x for x in (SERVICE_NOTES.get(service),
                                         ACRYLIC_NOTES.get(service) if acrylic else None,
                                         kerf_note(service, kerfs)) if x)
            lines += [note + ".", ""]
            if service in cut:
                unpriced = [c.name for c in getattr(bom, "cutting", ())
                            if c.service == service and c.usd is None] if left_out else []
                lines += [f"Estimate: ≈{_money(cut[service])} for these parts ({source})"
                          + (f", without {', '.join(unpriced)}: unpriced, see the total above"
                             if unpriced else "") + ".", ""]
            if fitted:      # a cradle's: each file's estimate, and whether it comes deburred
                lines += ["| qty | file | material | thickness | size mm | est. | deburred | "
                          "makes |", "|---:|---|---|---:|---|---:|---|---|"]
            else:
                lines += ["| qty | file | material | thickness | size mm | makes |",
                          "|---:|---|---|---:|---|---|"]
            for r in rows:
                head = r["parts"].split()[0] if r["parts"] else ""
                done = ("as cut: by hand" if head in rough else
                        "yes" if deburr_least(r["sheet"]) else "—")   # (wood: not offered)
                more = f" {_money(fitted.get(head))} | {done} |" if fitted else ""
                lines.append(f"| {r['qty']} | `{build_dir}/laser/parts/{r['file']}` | "
                             f"{r['material']} | {r['thickness_mm']:g} mm | {r['size_mm']} |"
                             f"{more} {r['parts']} |")
            lines += ["", f"{sum(r['qty'] for r in rows)} parts in {len(rows)} files.", ""]
            if fitted:      # (a cradle's: design review round 3, laser-cut parts minor 6)
                lines += ["As the parts arrive, write each one's label (its file name's first "
                          "word) on it with a paint pen: some files differ only by a hole or "
                          "a slot (the crank block's keyed and clear plates).", ""]

    if print_rows:
        lines += ["## Print", "",
                  f"STLs in `{build_dir}/print/` (`parts.csv` the same list), each on the plate "
                  "as it should print.", "",
                  "| print | file | filament | size mm | g each at 100 % |",
                  "|---|---|---|---|---:|"]
        for r in print_rows:
            lines.append(f"| {r['print']} | `{r['file']}` | {r.get('filament', '')} | "
                         f"{r['size_mm']} | {r['grams_each_100pct']} |")
        lines.append("")

    notes = [n for r in bom.purchased for n in _item_notes(r.key)]
    notes += [LINE_NOTES[r.key] for r in bom.purchased if r.key in LINE_NOTES]
    measured = [r for r in bom.purchased if r.key.startswith(MEASURED_PINS)]
    made = [r for r in measured if r.key.startswith("pivot_pin8_")]
    sticks = [r for r in measured if r not in made]
    for rows, note in ((made, MEASURED_PIN_NOTE), (sticks, ROD_PIN_NOTE)):
        if rows:
            notes.append(note.format(pins=", ".join(r.sku or r.name for r in rows),
                                     take=PIN_TAKE_UP_MM))
    if notes or bom.notes:
        lines += ["## Before you order", ""] + [f"* {n}" for n in dict.fromkeys(notes)] \
            + [f"* {n}" for n in bom.notes] + [""]
    return "\n".join(lines)


def _item_notes(key: str) -> list[str]:
    """The preferred offer's note where it says something the buyer must act on."""
    try:
        offer = get(key).offer
    except KeyError:
        return []
    note = (offer.note or "") if offer else ""
    cues = ("measure yours", "measure one", "measure it", "check one", "check it", "buy only",
            "don't", "minimum order", "made to order", "check the shipping")
    if offer is not None and any(c in note.lower() for c in cues):   # (no offer: no note)
        return [f"**{get(key).name}** ({offer.vendor}): {note}."]
    return []

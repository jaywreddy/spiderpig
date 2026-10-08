"""``ORDER.md``: the build's shopping list, one section per place to order from.

:mod:`hardware.bom` lists what the robot needs; this turns it into the orders: one cart per
vendor (each line its direct product page, SKU, packs and price), one upload per cutting
service (the per-part DXFs of :func:`layout.save_parts` with their quantities), the prints
per filament (:func:`build.export_prints`' rows), and what to check before ordering.
"""

from __future__ import annotations

from collections import defaultdict

from spiderpig.hardware.bom import ON_HAND, bought  # on hand / a service's sheet: no cart
from spiderpig.hardware.catalog import get

SERVICE_ORDER_URL = {
    "SendCutSend": "https://app.sendcutsend.com/customer#/quote",
    "Ponoko": "https://www.ponoko.com/designs",
}
SERVICE_NOTES = {
    "SendCutSend": "one part per file, units mm at upload, quantity per file as below; its "
                   "3 mm acrylic arrives 2.24-3.46 mm thick: measure it and rebuild with "
                   "--thickness if it is off by more than a few percent",
    "Ponoko": "one part per file, mm, the blue CUT layer mapped to cut; 3 mm sheet arrives "
              "2.49-3.51 mm thick: measure it and rebuild with --thickness if it is off "
              "by more than a few percent",
}
COMPENSATES = {"SendCutSend": True, "Ponoko": False}
"""Whether a service offsets its cut path for the kerf itself (SendCutSend does; Ponoko
doesn't in acrylic): what the files' own kerf (:func:`layout.sheet_kerf`, ``--kerf``)
must be."""


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
    if on_hand:
        lines += ["## From the shop and the servo boxes (on hand, not ordered)", ""] + [
            f"* {r.name}: {r.where[0] if len(r.where) == 1 else f'{len(r.where)} uses'}"
            for r in on_hand] + [""]

    if laser_rows:
        lines += ["## Cut (upload per service)", ""]
        by_service: dict[str, list[dict]] = defaultdict(list)
        for r in laser_rows:
            by_service[r["service"]].append(r)
        for service, rows in by_service.items():
            url = SERVICE_ORDER_URL.get(service, "")
            head = service if url or service not in ("", "any") else (
                "your own laser (or any service): the sheet stock is in the carts above")
            lines += [f"### {head}" + (f": [upload and quote]({url})" if url else ""), ""]
            kerfs = {float(r.get("kerf_mm") or 0.0) for r in rows}
            note = "; ".join(x for x in (SERVICE_NOTES.get(service),
                                         kerf_note(service, kerfs)) if x)
            lines += [note + ".", ""]
            lines += ["| qty | file | material | thickness | size mm | makes |",
                      "|---:|---|---|---:|---|---|"]
            for r in rows:
                lines.append(f"| {r['qty']} | `{build_dir}/laser/parts/{r['file']}` | "
                             f"{r['material']} | {r['thickness_mm']:g} mm | {r['size_mm']} | "
                             f"{r['parts']} |")
            lines += ["", f"{sum(r['qty'] for r in rows)} parts in {len(rows)} files.", ""]

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
            "don't", "minimum order", "made to order")
    if offer is not None and any(c in note.lower() for c in cues):   # (no offer: no note)
        return [f"**{get(key).name}** ({offer.vendor}): {note}."]
    return []

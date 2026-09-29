"""Bill of materials for a fabricated mechanism.

Everything is derived from the mechanism :func:`fabricate.fabricate` returns:

* bodies with ``fab == "purchased"`` count one each of their ``bom_key``;
* bodies with ``fab == "laser"`` are listed as cut parts (with the sheet
  stock they need, from ``mech.meta["sheet"]``);
* bodies with ``fab == "printed"`` are listed with their volume and an
  estimated filament mass;
* ``mech.bom_extras`` adds purchases that aren't modelled as bodies
  (washers, glue, shims, sheet stock).

Purchased lines are grouped by catalog key and rounded up to whole packs of
the first (preferred) offer.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

from hardware.catalog import get

_PLA_G_PER_CM3 = 1.24


@dataclass(frozen=True)
class BomLine:
    """``qty`` of catalog item ``key``, used at ``where``."""

    key: str
    qty: float
    where: str = ""


@dataclass
class PurchaseRow:
    key: str
    name: str
    category: str
    qty: float
    where: list[str]
    vendor: str
    url: str
    sku: str
    pack_qty: int
    packs: int
    pack_price_usd: float | None
    verified: bool
    alternatives: list[str]

    @property
    def cost_usd(self) -> float | None:
        return None if self.pack_price_usd is None else self.packs * self.pack_price_usd


@dataclass
class MadeRow:
    name: str
    method: str          # "laser" | "printed"
    material: str
    size_mm: str         # footprint of a laser part / bbox of a printed part
    volume_cm3: float


@dataclass
class Bom:
    purchased: list[PurchaseRow]
    made: list[MadeRow]
    title: str = ""

    @property
    def cost_usd(self) -> float:
        return sum(r.cost_usd or 0.0 for r in self.purchased)

    # -- writers ----------------------------------------------------------

    def write(self, out_dir: Path, stem: str = "bom") -> list[Path]:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        paths = [out_dir / f"{stem}.csv", out_dir / f"{stem}.md", out_dir / f"{stem}.json"]
        self.write_csv(paths[0])
        paths[1].write_text(self.markdown())
        paths[2].write_text(json.dumps(self.as_dict(), indent=1))
        return paths

    def write_csv(self, path: Path) -> None:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["section", "item", "qty", "packs", "pack_qty", "vendor", "sku",
                        "est_cost_usd", "url", "link_verified", "used_at"])
            for r in self.purchased:
                w.writerow(["buy", r.name, _num(r.qty), r.packs, r.pack_qty, r.vendor, r.sku,
                            "" if r.cost_usd is None else f"{r.cost_usd:.2f}", r.url,
                            "yes" if r.verified else "no", "; ".join(r.where)])
            for m in self.made:
                w.writerow([m.method, m.name, 1, "", "", "", m.material, "", "", "", m.size_mm])

    def markdown(self) -> str:
        lines = [f"# Bill of materials{': ' + self.title if self.title else ''}", ""]
        lines += ["## Buy", "",
                  "| qty | item | buy | packs | est. cost | used at |",
                  "|---:|---|---|---:|---:|---|"]
        for r in self.purchased:
            link = f"[{r.vendor}{' ' + r.sku if r.sku else ''}]({r.url})" if r.url else r.vendor
            if not r.verified and r.url:
                link += " (unverified link)"
            cost = "" if r.cost_usd is None else f"${r.cost_usd:.2f}"
            where = ", ".join(sorted(set(r.where)))[:120]
            lines.append(f"| {_num(r.qty)} | {r.name} | {link} | {r.packs} × {r.pack_qty} "
                         f"| {cost} | {where} |")
        lines += ["", f"Estimated purchase total: **${self.cost_usd:.2f}** "
                  "(pack prices at the listed vendor; excludes shipping).", ""]
        for method, title in (("laser", "Laser-cut"), ("printed", "3D printed")):
            rows = [m for m in self.made if m.method == method]
            if not rows:
                continue
            lines += [f"## {title} ({len(rows)} parts)", "", "| part | material | size |",
                      "|---|---|---|"]
            lines += [f"| {m.name} | {m.material} | {m.size_mm} |" for m in rows]
            if method == "printed":
                grams = sum(m.volume_cm3 for m in rows) * _PLA_G_PER_CM3
                lines += ["", f"About {grams:.0f} g of PLA at 100% infill."]
            lines.append("")
        return "\n".join(lines)

    def as_dict(self) -> dict:
        return {
            "title": self.title,
            "purchased": [dict(r.__dict__, cost_usd=r.cost_usd) for r in self.purchased],
            "made": [m.__dict__ for m in self.made],
            "cost_usd": self.cost_usd,
        }


def _num(q: float) -> str:
    return str(int(q)) if float(q).is_integer() else f"{q:g}"


def _footprint(part) -> str:
    bb = part.bounding_box()
    return f"{bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.1f} mm"


def bom_from_mechanism(mech, title: str = "") -> Bom:
    """Collect purchases and made parts from a fabricated mechanism."""
    lines: list[BomLine] = []
    made: list[MadeRow] = []
    sheet = mech.meta.get("sheet_name", "sheet")
    for body in mech.bodies:
        if body.fab == "purchased" and body.bom_key:
            lines.append(BomLine(body.bom_key, 1, body.name))
        elif body.fab in ("laser", "printed") and body.part is not None:
            made.append(MadeRow(
                name=body.name,
                method=body.fab,
                material=sheet if body.fab == "laser" else "PLA/PETG",
                size_mm=_footprint(body.part),
                volume_cm3=body.part.volume / 1000.0,
            ))
    lines.extend(mech.bom_extras)

    grouped: dict[str, PurchaseRow] = {}
    for line in lines:
        item = get(line.key)
        row = grouped.get(line.key)
        if row is None:
            offer = item.offer
            row = grouped[line.key] = PurchaseRow(
                key=item.key, name=item.name, category=item.category, qty=0.0, where=[],
                vendor=offer.vendor if offer else "", url=offer.url if offer else "",
                sku=(offer.sku or "") if offer else "",
                pack_qty=offer.pack_qty if offer else 1, packs=0,
                pack_price_usd=offer.price_usd if offer else None,
                verified=bool(offer and offer.verified),
                alternatives=[o.url for o in item.offers[1:]],
            )
        row.qty += line.qty
        if line.where:
            row.where.append(line.where)
    for row in grouped.values():
        row.packs = max(1, math.ceil(row.qty / max(row.pack_qty, 1)))
    order = ["servo", "horn", "fastener", "nut", "washer", "bearing", "bushing", "dowel",
             "clip", "spacer", "sheet", "adhesive", "misc"]
    purchased = sorted(
        grouped.values(),
        key=lambda r: (order.index(r.category) if r.category in order else len(order), r.name),
    )
    made.sort(key=lambda m: (m.method, m.name))
    return Bom(purchased=purchased, made=made, title=title)

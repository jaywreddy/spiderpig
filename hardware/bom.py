"""Bill of materials for a fabricated mechanism.

Everything is derived from the mechanism :func:`fabricate.fabricate` returns:

* bodies with ``fab == "purchased"`` count one each of their ``bom_key``;
* bodies with ``fab == "laser"`` are listed as cut parts (with the sheet
  stock they need, from ``mech.meta["sheet"]``);
* bodies with ``fab == "printed"`` are listed with their volume, and add a
  filament line (``mech.meta["filament"]`` or the ``filament`` argument: grams
  at 100 % infill from the catalog item's density, as a fraction of a spool);
* ``mech.bom_extras`` adds purchases that aren't modelled as bodies
  (washers, glue, shims, sheet stock).

Made parts that are the same shape are one row with a quantity
(:func:`group_made`): a laser-cut plate and its mirror image are the same cut
(flip it over), a printed part and its mirror image are not ("print
mirrored").

Purchased lines are grouped by catalog key and rounded up to whole packs of
the first (preferred) offer. Lines whose preferred offer is the same product
(same vendor and SKU, e.g. one screw assortment for several lengths) are
bought once.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from hardware.catalog import get
from hardware.mass import filament_density


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
    same_pack_as: str = ""     # bought with another row's pack (same vendor and SKU)

    @property
    def cost_usd(self) -> float | None:
        if self.same_pack_as:
            return 0.0
        return None if self.pack_price_usd is None else self.packs * self.pack_price_usd


@dataclass
class MadeRow:
    name: str
    method: str          # "laser" | "printed"
    material: str
    size_mm: str         # footprint of a laser part / bbox of a printed part
    volume_cm3: float    # one part
    qty: int = 1
    names: list[str] = field(default_factory=list)
    mirrored: int = 0    # of qty, how many are mirror images (printed parts: "print mirrored")


@dataclass
class Bom:
    purchased: list[PurchaseRow]
    made: list[MadeRow]
    title: str = ""
    notes: list[str] = field(default_factory=list)
    printed_g: float | None = None     # filament for the printed parts at 100 % infill
    filament: str = "PLA"

    @property
    def cost_usd(self) -> float:
        return sum(r.cost_usd or 0.0 for r in self.purchased)

    @property
    def unpriced(self) -> list[PurchaseRow]:
        return [r for r in self.purchased if r.cost_usd is None]

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
                packs = 0 if r.same_pack_as else r.packs
                cost = "" if r.cost_usd is None else f"{r.cost_usd:.2f}"
                where = "; ".join(r.where)
                if r.same_pack_as:
                    where = f"(in the same pack as {r.same_pack_as}) {where}"
                w.writerow(["buy", r.name, _num(r.qty), packs, r.pack_qty, r.vendor, r.sku,
                            cost, r.url, "yes" if r.verified else "no", where])
            for m in self.made:
                note = f"{m.mirrored} mirrored" if m.mirrored else ""
                w.writerow([m.method, m.name, m.qty, "", "", "", m.material, "", "", note,
                            f"{m.size_mm}; {', '.join(m.names or [m.name])}"])

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
            packs = f"{r.packs} × {r.pack_qty}"
            if r.same_pack_as:
                packs, cost = f"with {r.same_pack_as}", "–"
            where = ", ".join(sorted(set(r.where)))[:120]
            lines.append(f"| {_num(r.qty)} | {r.name} | {link} | {packs} | {cost} | {where} |")
        lines += ["", f"Estimated purchase total: **${self.cost_usd:.2f}** "
                  "(pack prices at the listed vendor; excludes shipping)."]
        if self.unpriced:
            lines.append(f"{len(self.unpriced)} item(s) have no listed price and are not in "
                         "the total: " + ", ".join(r.name for r in self.unpriced) + ".")
        lines.append("")
        for method, title in (("laser", "Laser-cut"), ("printed", "3D printed")):
            rows = [m for m in self.made if m.method == method]
            if not rows:
                continue
            count = sum(m.qty for m in rows)
            lines += [f"## {title} ({count} parts, {len(rows)} different)", "",
                      "| qty | part | material | size | same part |", "|---:|---|---|---|---|"]
            for m in rows:
                qty = str(m.qty)
                if m.mirrored:
                    qty = f"{m.qty - m.mirrored} + {m.mirrored} mirrored"
                others = ", ".join(n for n in m.names if n != m.name)
                lines.append(f"| {qty} | {m.name} | {m.material} | {m.size_mm} | {others} |")
            if method == "printed":
                grams = self.printed_g
                if grams is None:
                    grams = sum(m.volume_cm3 * m.qty for m in rows) * filament_density(None)
                lines += ["", f"About {grams:.0f} g of {self.filament} at 100% infill."]
            lines.append("")
        if self.notes:
            lines += ["## Notes", ""] + [f"* {n}" for n in self.notes] + [""]
        return "\n".join(lines)

    def as_dict(self) -> dict:
        return {
            "title": self.title,
            "purchased": [dict(r.__dict__, cost_usd=r.cost_usd) for r in self.purchased],
            "made": [m.__dict__ for m in self.made],
            "cost_usd": self.cost_usd,
            "printed_g": self.printed_g,
            "notes": self.notes,
        }


def _num(q: float) -> str:
    return str(int(q)) if float(q).is_integer() else f"{q:g}"


def _footprint(part) -> str:
    bb = part.bounding_box()
    return f"{bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.1f} mm"


# ---------------------------------------------------------------------------
# Identical made parts
# ---------------------------------------------------------------------------


def _frame(part) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(centroid, principal axes as columns, principal moments), moments ascending."""
    from build123d import CenterOf

    c = part.center(CenterOf.MASS)
    props = sorted(part.principal_properties, key=lambda am: am[1])
    axes = np.array([[a.X, a.Y, a.Z] for a, _ in props]).T
    return np.array([c.X, c.Y, c.Z]), axes, np.array([m for _, m in props])


def _proper_fit(a, fa, b, fb, tol: float) -> bool:
    """Is ``b`` the image of ``a`` under a rotation + translation (principal frames matched)?"""
    from build123d import Location, Plane

    ca, ea, _ = fa
    cb, eb, _ = fb
    for signs in ((1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1),
                  (-1, -1, -1), (-1, 1, 1), (1, -1, 1), (1, 1, -1)):
        r = eb @ np.diag(signs) @ ea.T
        if np.linalg.det(r) < 0:
            continue
        t = cb - r @ ca
        moved = a.moved(Location(Plane(tuple(t), tuple(r[:, 0]), tuple(r[:, 2]))))
        diff = sum(s.volume for s in (moved - b).solids()) + sum(
            s.volume for s in (b - moved).solids())
        if diff < tol:
            return True
    return False


def congruent(a, b, rel: float = 1e-4) -> str | None:
    """``"same"`` if ``b`` is ``a`` moved, ``"mirror"`` if it is ``a``'s mirror image, else None."""
    from build123d import Plane

    va, vb = a.volume, b.volume
    if abs(va - vb) > rel * max(va, vb) or abs(a.area - b.area) > rel * max(a.area, b.area):
        return None
    fa, fb = _frame(a), _frame(b)
    if not np.allclose(fa[2], fb[2], rtol=1e-3, atol=1e-6 * max(fa[2].max(), 1.0)):
        return None
    tol = max(1e-3, 1e-4 * vb)
    if _proper_fit(a, fa, b, fb, tol):
        return "same"
    m = a.mirror(Plane.XY)
    if _proper_fit(m, _frame(m), b, fb, tol):
        return "mirror"
    return None


@dataclass
class MadeGroup:
    """Made parts of one shape: ``ref`` is the body whose part is the pattern."""

    method: str
    ref: object                                  # mechanism.Body
    names: list[str]
    mirrored: list[str] = field(default_factory=list)

    @property
    def qty(self) -> int:
        return len(self.names)


def group_made(bodies, method: str) -> list[MadeGroup]:
    """Group ``method`` bodies by shape. For laser parts a mirror image is the same cut."""
    groups: list[MadeGroup] = []
    for b in bodies:
        if b.fab != method or b.part is None:
            continue
        for g in groups:
            rel = congruent(g.ref.part, b.part)
            if rel is None:
                continue
            g.names.append(b.name)
            if rel == "mirror" and method == "printed":
                g.mirrored.append(b.name)
            break
        else:
            groups.append(MadeGroup(method, b, [b.name]))
    return groups


# ---------------------------------------------------------------------------
# The BOM
# ---------------------------------------------------------------------------


def bom_from_mechanism(mech, title: str = "", filament: str | None = None,
                       group: bool = True,
                       groups: dict[str, list[MadeGroup]] | None = None) -> Bom:
    """Collect purchases and made parts from a fabricated mechanism.

    ``group=False`` lists every made part on its own row (faster: no shape
    comparison); ``groups`` passes :func:`group_made` results already computed
    (method -> groups).
    """
    lines: list[BomLine] = []
    made: list[MadeRow] = []
    notes: list[str] = []
    sheet = mech.meta.get("sheet_name", "sheet")
    filament = filament or mech.meta.get("filament")
    density = filament_density(filament)
    fil_name = get(filament).name.split(",")[0] if filament else "PLA/PETG"
    for body in mech.bodies:
        if body.fab == "purchased" and body.bom_key:
            lines.append(BomLine(body.bom_key, 1, body.name))
    for method in ("laser", "printed"):
        bodies = [b for b in mech.bodies if b.fab == method and b.part is not None]
        if groups is not None and method in groups:
            found = groups[method]
        elif group:
            found = group_made(bodies, method)
        else:
            found = [MadeGroup(method, b, [b.name]) for b in bodies]
        for g in found:
            made.append(MadeRow(
                name=g.ref.name, method=method,
                material=sheet if method == "laser" else fil_name,
                size_mm=_footprint(g.ref.part), volume_cm3=g.ref.part.volume / 1000.0,
                qty=g.qty, names=list(g.names), mirrored=len(g.mirrored),
            ))
    grams = sum(m.volume_cm3 * m.qty for m in made if m.method == "printed") * density
    if filament and grams > 0:
        spool = float(get(filament).dims.get("spool_g", 1000.0))
        lines.append(BomLine(filament, round(grams / spool, 3),
                             f"printed parts: {grams:.0f} g at 100 % infill"))
        notes.append(f"Printed parts need about {grams:.0f} g of filament at 100 % infill "
                     f"({density} g/cm3); less with sparse infill.")
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
        row.qty = round(row.qty, 6)
        row.packs = max(1, math.ceil(row.qty / max(row.pack_qty, 1) - 1e-9))
    order = ["servo", "horn", "fastener", "insert", "nut", "washer", "standoff", "spacer",
             "bearing", "bushing", "dowel", "clip", "sheet", "filament", "adhesive", "misc"]
    purchased = sorted(
        grouped.values(),
        key=lambda r: (order.index(r.category) if r.category in order else len(order), r.name),
    )
    first: dict[tuple[str, str], PurchaseRow] = {}
    for row in purchased:           # one product (vendor + SKU) covering several rows
        if not row.sku:
            continue
        lead = first.setdefault((row.vendor, row.sku), row)
        if lead is not row:
            row.same_pack_as = lead.name
            lead.packs = max(lead.packs, row.packs)
    made.sort(key=lambda m: (m.method, m.name))
    return Bom(purchased=purchased, made=made, title=title, notes=notes,
               printed_g=grams if filament else None,
               filament=fil_name.split()[0] if filament else "PLA")

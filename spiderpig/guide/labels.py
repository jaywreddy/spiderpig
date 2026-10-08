"""Part identifiers for the guide: one label per part type, numbered in assembly order.

``P`` printed (a shape group of :func:`hardware.bom.group_made`, mirror images apart),
``C`` laser-cut (a cut: a mirror image is the same cut, the sheet flipped), ``H`` bought
(a catalog key). A type's number is the order it first appears in the steps, so the
inventory page reads in the order the parts are needed.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

from spiderpig.guide.model import Callout, Step, bare


@dataclass
class PartType:
    label: str
    kind: str            # printed, laser, purchased
    name: str
    ref: str             # a body of the type (its thumbnail)
    names: list[str]

    @property
    def qty(self) -> int:
        return len(self.names)


def part_types(mech, steps: list[Step]) -> tuple[dict[str, PartType], list[PartType]]:
    """Each body's type, and the types in label order."""
    from spiderpig.hardware import catalog
    from spiderpig.hardware.bom import group_made

    keyed: dict[tuple, list[str]] = {}
    ref: dict[tuple, str] = {}
    desc: dict[tuple, str] = {}
    for method in ("printed", "laser"):
        for g in group_made(mech.bodies, method):
            k = (method, g.ref.name)
            keyed[k] = list(g.names)
            ref[k] = g.ref.name
            bb = g.ref.part.bounding_box()  # pyright: ignore[reportAttributeAccessIssue]
            what = _printed_name(g.ref.name) if method == "printed" else "laser-cut part"
            sheet = f", {g.ref.sheet}" if method == "laser" and g.ref.sheet else ""
            dims = sorted((bb.size.X, bb.size.Y, bb.size.Z), reverse=True)
            desc[k] = f"{what} {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm{sheet}"
    for b in mech.bodies:
        if b.fab == "purchased" and b.part is not None:
            k = ("purchased", b.bom_key or b.name)
            keyed.setdefault(k, []).append(b.name)
            ref.setdefault(k, b.name)
            if k not in desc:
                try:
                    desc[k] = catalog.get(b.bom_key).name if b.bom_key else bare(b.name)
                except KeyError:
                    desc[k] = b.bom_key or bare(b.name)
    of = {n: k for k, names in keyed.items() for n in names}
    order: list[tuple] = []
    for st in steps:
        for n in st.adds:
            k = of.get(n)
            if k is not None and k not in order:
                order.append(k)
    order += [k for k in keyed if k not in order]
    prefix = {"printed": "P", "laser": "C", "purchased": "H"}
    count: Counter = Counter()
    types: list[PartType] = []
    by_body: dict[str, PartType] = {}
    for k in order:
        count[k[0]] += 1
        t = PartType(f"{prefix[k[0]]}{count[k[0]]:02d}", k[0], desc[k], ref[k], keyed[k])
        types.append(t)
        by_body.update(dict.fromkeys(t.names, t))
    return by_body, types


def _printed_name(name: str) -> str:
    n = bare(name)
    for pat, what in (("gap", "spacer"), ("ring", "ring"), ("spacer_hi", "top spacer"),
                      ("sock", "socket"), ("collar", "collar"), ("sleeve", "sleeve"),
                      ("thrust", "thrust sleeve"), ("horn_spacer", "horn spacer"),
                      ("deck_rail", "deck rail"), ("cradle", "battery cradle")):
        if pat in n:
            return f"printed {what}"
    return "printed part"


def callouts(by_body: dict[str, PartType], names: list[str]) -> list[Callout]:
    qty: Counter = Counter()
    seen: dict[str, PartType] = {}
    for n in names:
        t = by_body.get(n)
        if t is not None:
            qty[t.label] += 1
            seen[t.label] = t
    return [Callout(lab, qty[lab], seen[lab].name, seen[lab].ref) for lab in sorted(
        seen, key=lambda s: ("PCH".index(s[0]), s))]

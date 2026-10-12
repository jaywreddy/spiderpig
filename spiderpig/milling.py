"""Milled parts (2026-10-12, phase 5): the user's own desktop mill, and what it can make.

The fab method ``milled`` (:mod:`spiderpig.fabmethods`): a part machined from plate stock
on the user's mill (the role ``mill``: :data:`MILLS`), where a laser can't make it: a
bearing seat bored to a slip fit with its shoulder in one piece, concentric with the bolt
holes round it. A construction that wants one describes it as a :class:`MillPart`: its
stock (a plate, :mod:`hardware.plate_catalog`), its outline (a circle) and its operations,
all from one side (one setup), each a circle at a depth (2.5-D):

* ``bore``: a round hole or pocket, helically interpolated by the largest tool that fits
  it (a finish pass to size: the seat's fit is the bore's);
* ``profile``: the outline cut free last, round the outside, holding tabs left.

The part's solid is made from the same description (:meth:`MillPart.solid`), so what is
drawn is what the setup sheet says; :func:`check` holds it to the mill's rules
(:class:`Mill`): it fits the bed with the clamps' margin, every bore takes a tool
(at least the smallest tool's diameter, no deeper than that tool's flutes reach), the
stock is no thicker than the longest tool cuts through, and every web between features
(and to the edge) is at least :attr:`Mill.min_wall`. :func:`minutes` estimates the machine
time (the material removed at the material's removal rate, a setup each); the build writes
each part's STEP and a setup sheet (``mill/SETUP.md``: :func:`setup_sheet`).

**The Carvera** (:data:`CARVERA`, Makera's listing): 360 x 240 x 140 mm, a 1/8 in collet,
a six-tool changer, 15,000 rpm at 200 W. The tools are flat end mills on 1/8 in shanks
(their flute lengths a common catalogue's); the removal rates are conservative figures for a
200 W spindle (estimates: the machine's own feeds and speeds tables are the authority).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field


@dataclass(frozen=True)
class Tool:
    """A flat end mill: diameter and flute length (mm)."""

    key: str
    d: float
    flute: float


@dataclass(frozen=True)
class Mill:
    """A desktop mill (module docstring)."""

    key: str
    label: str
    bed: tuple[float, float]          # the work area (mm)
    z: float
    tools: tuple[Tool, ...]
    clamp: float = 10.0               # the stock past the part on every side for the clamps
    min_wall: float = 1.0             # the thinnest web it leaves (mm)
    mrr: dict[str, float] = field(default_factory=dict)   # mm^3/min by material
    setup_min: float = 10.0           # a setup's minutes (stock, zeroing, the probe)
    source: str = ""

    def tool_for(self, d: float, depth: float) -> Tool | None:
        """The largest tool that bores a ``d`` hole ``depth`` deep (helically: the tool
        under the hole's diameter)."""
        fits = [t for t in self.tools if t.d <= d + 1e-9 and t.flute >= depth - 1e-9]
        return max(fits, key=lambda t: t.d) if fits else None


CARVERA = Mill(
    "carvera", "Makera Carvera", bed=(360.0, 240.0), z=140.0,
    tools=(Tool("em_3p175", 3.175, 12.0), Tool("em_2", 2.0, 6.0), Tool("em_1", 1.0, 3.0)),
    mrr={"aluminium": 300.0, "acrylic": 1500.0, "polycarbonate": 1200.0},
    source="Makera's listing (360 x 240 x 140 mm, 1/8 in collet, 6-tool changer, 200 W "
           "15,000 rpm); the removal rates are estimates")
MILLS: dict[str, Mill] = {CARVERA.key: CARVERA}
"""The mills the role ``mill`` picks from (``none``: no mill: everything is laser-cut)."""


@dataclass(frozen=True)
class Op:
    """One operation from the top: a circle ``d`` at ``(x, y)`` (the part's frame), cut
    ``depth`` down (``None``: through), its ``purpose`` for the setup sheet."""

    kind: str                         # "bore" | "profile"
    d: float
    x: float = 0.0
    y: float = 0.0
    depth: float | None = None
    purpose: str = ""
    fit: str = ""                     # the tolerance a finish pass holds ("+0.01/0")


@dataclass(frozen=True)
class MillPart:
    """A milled part: its stock plate, thickness and operations (module docstring). The
    part's frame: the outline's centre at the origin, its top face (the side machined) at
    ``z = thickness``, its bottom at 0."""

    name: str
    stock: str
    thickness: float
    outline_d: float
    ops: tuple[Op, ...]
    role: str = ""

    def solid(self):
        """The part in its own frame (build123d): the outline less every operation."""
        from build123d import Cylinder, Pos

        t = self.thickness
        part = Cylinder(self.outline_d / 2, t).move(Pos(0, 0, t / 2))
        for op in self.ops:
            if op.kind != "bore":
                continue
            depth = t if op.depth is None else op.depth
            cut = Cylinder(op.d / 2, depth + 0.02).move(Pos(op.x, op.y, t - depth / 2 + 0.01))
            part = part - cut
        return part

    def as_dict(self) -> dict:
        return {"name": self.name, "stock": self.stock, "thickness": self.thickness,
                "outline_d": self.outline_d, "role": self.role,
                "ops": [asdict(o) for o in self.ops]}

    @classmethod
    def from_dict(cls, d: dict) -> MillPart:
        return cls(d["name"], d["stock"], float(d["thickness"]), float(d["outline_d"]),
                   tuple(Op(**o) for o in d["ops"]), d.get("role", ""))

    def blank(self, mill: Mill) -> tuple[float, float]:
        """The stock it is cut from: the outline and the clamps' margin each side (mm)."""
        s = self.outline_d + 2 * mill.clamp
        return (s, s)

    def removed_mm3(self) -> float:
        """The material the operations remove: every bore's, and the profile's slot round
        the outline (the largest tool's width)."""
        t = self.thickness
        v = sum(math.pi * (o.d / 2) ** 2 * (t if o.depth is None else o.depth)
                for o in self.ops if o.kind == "bore")
        prof = next((o for o in self.ops if o.kind == "profile"), None)
        if prof is not None:
            w = 3.175
            v += math.pi * (self.outline_d + w) * w * t
        return v


def check(part: MillPart, mill: Mill) -> list[dict]:
    """``part`` against ``mill``'s rules (module docstring): issues as
    :func:`manufacture.check`'s (``rule``, ``part``, ``value``, ``detail``, ``level``,
    ``why``, ``fix``)."""
    from spiderpig.materials import sheet

    out: list[dict] = []
    t = part.thickness

    def issue(rule, value, detail, why, fix, level="error"):
        out.append({"rule": rule, "part": part.name, "value": value, "detail": detail,
                    "level": level, "why": why, "fix": fix, "sheet": part.stock})

    if abs(sheet(part.stock).thickness - t) > 1e-6:
        issue("mill_stock", t, f"drawn {t:g} mm thick from {part.stock} "
              f"({sheet(part.stock).thickness:g} mm)", "the part is its stock's thickness",
              "draw it at the stock's thickness")
    bx, by = part.blank(mill)
    if bx > mill.bed[0] + 1e-9 or by > mill.bed[1] + 1e-9:
        issue("mill_bed", max(bx, by), f"its blank {bx:.0f} x {by:.0f} mm exceeds the "
              f"{mill.label}'s {mill.bed[0]:g} x {mill.bed[1]:g} mm bed",
              "it has to fit with the clamps", "split it, or cut it by laser")
    longest = max(tl.flute for tl in mill.tools)
    if t > longest + 1e-9:
        issue("mill_depth", t, f"{t:g} mm stock, the longest flutes reach {longest:g} mm",
              "the profile cuts through the stock", "a thinner plate")
    smallest = min(tl.d for tl in mill.tools)
    bores = [o for o in part.ops if o.kind == "bore"]
    for o in bores:
        depth = t if o.depth is None else o.depth
        if mill.tool_for(o.d, depth) is None:
            issue("mill_tool", o.d, f"a {o.d:g} mm bore {depth:g} mm deep ({o.purpose}): no "
                  f"tool bores it (smallest {smallest:g} mm, each to its flutes)",
                  "a bore is helical: the tool under the hole's diameter, the depth within "
                  "its flutes", "a larger hole, or a drill")
    # webs: from each bore to the outline, and between bores side by side (a stepped bore,
    # one circle inside another on one centre, is one feature)
    r_out = part.outline_d / 2
    for i, a in enumerate(bores):
        edge = r_out - (math.hypot(a.x, a.y) + a.d / 2)
        if edge < mill.min_wall - 1e-9:
            issue("mill_wall", round(edge, 3), f"{edge:.2f} mm from a {a.d:g} mm bore "
                  f"({a.purpose}) to the edge", f"the mill leaves no web under "
                  f"{mill.min_wall:g} mm", "move the bore in, or grow the part")
        for b in bores[i + 1:]:
            centres = math.hypot(a.x - b.x, a.y - b.y)
            gap = centres - (a.d + b.d) / 2
            if centres > 1e-9 and gap < mill.min_wall - 1e-9:
                issue("mill_wall", round(gap, 3), f"{gap:.2f} mm between a {a.d:g} and a "
                      f"{b.d:g} mm bore", f"the mill leaves no web under {mill.min_wall:g} mm",
                      "move them apart")
    return out


def minutes(part: MillPart, mill: Mill) -> float:
    """The machine time (min, an estimate): the material removed at the material's rate,
    and a setup."""
    from spiderpig.materials import sheet

    rate = mill.mrr.get(sheet(part.stock).material, min(mill.mrr.values()))
    return mill.setup_min + part.removed_mm3() / rate


def setup_sheet(parts: list[tuple[str, int, MillPart]], mill: Mill) -> list[str]:
    """The setup sheet (markdown lines) of ``(label, qty, part)``: per part its stock and
    blank, the zero (the blank's centre, the top face), each operation with its tool,
    diameter, depth and fit, and the time."""
    from spiderpig.materials import sheet

    lines = [f"# Milled parts: {mill.label}", "",
             f"Machine: {mill.label} ({mill.source}). Zero: the blank's centre, its top "
             "face; every operation from the top, one setup, the profile last on tabs.", ""]
    total = 0.0
    for label, qty, part in parts:
        bx, by = part.blank(mill)
        mins = minutes(part, mill)
        total += mins * qty
        lines += [f"## {label} x{qty}: {part.role or part.name}", "",
                  f"- stock: {sheet(part.stock).name} ({part.stock}), blank {bx:.0f} x "
                  f"{by:.0f} mm, {part.thickness:g} mm",
                  f"- time: about {mins:.0f} min each (estimate)", "",
                  "| # | operation | tool | diameter | at | depth | fit | for |",
                  "|---:|---|---|---:|---|---:|---|---|"]
        for i, o in enumerate(sorted(part.ops, key=lambda o: (o.kind == "profile", -o.d)), 1):
            depth = part.thickness if o.depth is None else o.depth
            tl = (mill.tool_for(o.d, depth) if o.kind == "bore"
                  else max(mill.tools, key=lambda t: t.d))
            how_deep = "through" if o.depth is None else f"{depth:g}"
            lines.append(f"| {i} | {o.kind} | {tl.key if tl else '?'} | {o.d:g} | "
                         f"({o.x:.2f}, {o.y:.2f}) | {how_deep} | {o.fit or ''} | "
                         f"{o.purpose} |")
        lines.append("")
    lines += [f"Total: about {total:.0f} min of machine time (estimates).", ""]
    return lines


def write_outputs(mech, groups, types, out) -> list:
    """The milled parts' files under ``out`` (``mill/``): each distinct part's STEP in its
    own frame (its label's file: :func:`labels.part_types`), ``parts.csv`` and the setup
    sheet ``SETUP.md``. ``groups``: the ``milled`` made groups; ``types``: body -> part
    type."""
    import csv
    from pathlib import Path

    from build123d import export_step

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    mill = MILLS[mech.meta["mill"]]
    specs = mech.meta["milled"]
    rows, sheet_parts, files = [], [], []
    for g in groups:
        t = types[g.ref.name]
        part = MillPart.from_dict(specs[g.ref.name])
        path = out / t.file
        export_step(part.solid(), str(path))
        files.append(path)
        sheet_parts.append((t.label, g.qty, part))
        rows.append({"file": t.file, "label": t.label, "qty": g.qty, "stock": part.stock,
                     "thickness_mm": part.thickness, "minutes_each": round(minutes(part, mill))})
    with (out / "parts.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    (out / "SETUP.md").write_text("\n".join(setup_sheet(sheet_parts, mill)))
    return [*files, out / "parts.csv", out / "SETUP.md"]

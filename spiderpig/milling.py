"""Milled parts (2026-10-12, phase 5): the user's own desktop mill, and what it can make.

The fab method ``milled`` (:mod:`spiderpig.fabmethods`): a part machined from plate stock
on the user's mill (the role ``mill``: :data:`MILLS`), where a laser can't make it: a
bearing seat pocketed to a slip fit with its shoulder in one piece, concentric with the
holes round it. A construction that wants one describes it as a :class:`MillPart`: its
stock (a plate, :mod:`hardware.plate_catalog`), its outline (a circle) and its operations,
all from one side (one setup), each a circle at a depth (2.5-D):

* ``pocket``: a round pocket or through hole cleared by a spiral of the largest tool that
  fits it (the hole at least :data:`HELIX_RATIO` x the tool, so chips get out), roughed
  under size and finished to it by a full-depth pass and a spring pass; a through pocket
  cuts :data:`BREAKTHROUGH` into the spoilboard;
* ``drill``: a small hole spotted on the mill (its place) and drilled through on a drill
  press (no end mill bores a 3.4 mm hole through 3/8 in of aluminium);
* ``profile``: the outline cut free last, round the outside, on holding tabs.

The part's solid is made from the same description (:meth:`MillPart.solid`), so what is
drawn is what the setup sheet says; :func:`check` holds it to the mill's rules
(:class:`Mill`): its blank fits the bed with the clamps' margin; every pocket takes a tool
(its depth within the tool's flutes) and leaves at least :data:`MIN_FLOOR` of floor under
it, or goes through; the profile is the outline and a tool reaches through the stock; every
web between features (and to the edge) is at least :attr:`Mill.min_wall` (a hole inside a
pocket's floor is one feature with it). :func:`minutes` estimates the machine time (the
material removed, measured on the solid and the profile's slot, at the material's removal
rate; each drilled hole's spot; a setup); the build writes each part's STEP and a setup
sheet (``mill/SETUP.md``: :func:`setup_sheet`, the work holding and finishing with it).

**The Carvera** (:data:`CARVERA`, Makera's listing): 360 x 240 x 140 mm, a 1/8 in collet,
a six-tool changer, 15,000 rpm at 200 W. The tools are flat end mills on 1/8 in shanks
(their flute lengths a common catalogue's); the removal rates are cautious figures for a
200 W spindle (estimates: the machine's own feeds and speeds are the authority).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

HELIX_RATIO = 1.4      # a spiral pocket's diameter over its tool's, at least (chip room)
MIN_FLOOR = 0.8        # the thinnest floor a blind pocket leaves (mm)
BREAKTHROUGH = 0.3     # a through cut's depth past the stock, into the spoilboard (mm)
SPOT_MIN = 0.5         # a drilled hole's spot on the mill (minutes, each)


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
    clamp: float = 18.0               # the stock past the part each side: screws through
    #                                   the waste, clear of the collet nut on a through cut
    min_wall: float = 1.0             # the thinnest web it leaves (mm)
    mrr: dict[str, float] = field(default_factory=dict)   # mm^3/min by material
    setup_min: float = 15.0           # a setup's minutes (stock, zeroing, the probe)
    source: str = ""

    def tool_for(self, d: float, depth: float) -> Tool | None:
        """The largest tool that spirals a ``d`` pocket ``depth`` deep (the pocket at least
        :data:`HELIX_RATIO` x the tool, the depth within its flutes)."""
        fits = [t for t in self.tools if HELIX_RATIO * t.d <= d + 1e-9
                and t.flute >= depth - 1e-9]
        return max(fits, key=lambda t: t.d) if fits else None

    @property
    def profile_tool(self) -> Tool:
        return max(self.tools, key=lambda t: t.d)


CARVERA = Mill(
    "carvera", "Makera Carvera", bed=(360.0, 240.0), z=140.0,
    tools=(Tool("em_3p175", 3.175, 12.0), Tool("em_2", 2.0, 6.0), Tool("em_1", 1.0, 3.0)),
    mrr={"aluminium": 200.0, "acrylic": 1200.0, "polycarbonate": 1000.0},
    source="Makera's listing (360 x 240 x 140 mm, 1/8 in collet, 6-tool changer, 200 W "
           "15,000 rpm); the removal rates are estimates")
MILLS: dict[str, Mill] = {CARVERA.key: CARVERA}
"""The mills the role ``mill`` picks from (``none``: no mill: everything is laser-cut)."""


@dataclass(frozen=True)
class Op:
    """One operation from the top: a circle ``d`` at ``(x, y)`` (the part's frame), cut
    ``depth`` down (``None``: through), its ``purpose`` for the setup sheet, the ``fit`` a
    finish pass holds (as drawn: the solid is cut at ``d``)."""

    kind: str                         # "pocket" | "drill" | "profile"
    d: float
    x: float = 0.0
    y: float = 0.0
    depth: float | None = None
    purpose: str = ""
    fit: str = ""


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
        """The part in its own frame (build123d): the outline less every hole."""
        from build123d import Cylinder, Pos

        t = self.thickness
        part = Cylinder(self.outline_d / 2, t).move(Pos(0, 0, t / 2))
        for op in self.ops:
            if op.kind == "profile":
                continue
            depth = min(t, t if op.depth is None else op.depth)
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

    def removed_mm3(self, mill: Mill) -> float:
        """The material the mill removes: the pockets' (measured on the solid: what the
        outline loses, less the drilled holes'), and the profile's slot round the outline
        (its tool's width)."""
        t = self.thickness
        outline = math.pi * (self.outline_d / 2) ** 2 * t
        drilled = sum(math.pi * (o.d / 2) ** 2 * t for o in self.ops if o.kind == "drill")
        v = outline - float(self.solid().volume) - drilled
        if any(o.kind == "profile" for o in self.ops):
            w = mill.profile_tool.d
            v += math.pi * (self.outline_d + w) * w * (t + BREAKTHROUGH)
        return max(v, 0.0)


def _contained(a: Op, b: Op) -> bool:
    """Whether one of two circles lies inside the other (a hole in a pocket's floor)."""
    small, big = sorted((a, b), key=lambda o: o.d)
    return math.hypot(a.x - b.x, a.y - b.y) + small.d / 2 <= big.d / 2 + 1e-9


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
    profiles = [o for o in part.ops if o.kind == "profile"]
    if len(profiles) != 1 or abs(profiles[0].d - part.outline_d) > 1e-6:
        issue("mill_profile", len(profiles), f"{len(profiles)} profile(s), not one round "
              f"the {part.outline_d:g} mm outline", "the part is cut free by its outline",
              "one profile at the outline's diameter")
    reach = mill.profile_tool.flute
    if t + BREAKTHROUGH > reach + 1e-9:
        issue("mill_depth", t, f"{t:g} mm stock and {BREAKTHROUGH:g} mm past it; the "
              f"longest flutes reach {reach:g} mm", "the profile cuts through the stock",
              "a thinner plate")
    smallest = min(tl.d for tl in mill.tools)
    holes = [o for o in part.ops if o.kind in ("pocket", "drill")]
    for o in holes:
        if o.kind == "drill":
            continue
        depth = t + BREAKTHROUGH if o.depth is None else float(o.depth)
        if o.depth is not None and depth > t - MIN_FLOOR + 1e-9:
            issue("mill_floor", round(t - depth, 3), f"a {o.d:g} mm pocket {depth:g} "
                  f"mm deep ({o.purpose}) leaves {t - depth:.2f} mm of floor",
                  f"a blind pocket leaves at least {MIN_FLOOR:g} mm", "make it through, or "
                  "shallower")
        if mill.tool_for(o.d, depth) is None:
            issue("mill_tool", o.d, f"a {o.d:g} mm pocket {depth:g} mm deep ({o.purpose}): "
                  f"no tool spirals it (at least {HELIX_RATIO:g} x the tool, smallest "
                  f"{smallest:g} mm, each to its flutes)", "a pocket is spiralled: the tool "
                  "well under the hole, the depth within its flutes",
                  "a larger hole, or a drilled one")
    r_out = part.outline_d / 2
    for i, a in enumerate(holes):
        edge = r_out - (math.hypot(a.x, a.y) + a.d / 2)
        if edge < mill.min_wall - 1e-9:
            issue("mill_wall", round(edge, 3), f"{edge:.2f} mm from a {a.d:g} mm hole "
                  f"({a.purpose}) to the edge", f"the mill leaves no web under "
                  f"{mill.min_wall:g} mm", "move the hole in, or grow the part")
        for b in holes[i + 1:]:
            if _contained(a, b):
                continue                  # a hole in a pocket's floor: one feature
            gap = math.hypot(a.x - b.x, a.y - b.y) - (a.d + b.d) / 2
            if gap < mill.min_wall - 1e-9:
                issue("mill_wall", round(gap, 3), f"{gap:.2f} mm between a {a.d:g} and a "
                      f"{b.d:g} mm hole", f"the mill leaves no web under {mill.min_wall:g} mm",
                      "move them apart")
    return out


def minutes(part: MillPart, mill: Mill) -> float:
    """The machine time (min, an estimate): the material removed at the material's rate,
    each drilled hole's spot, and a setup."""
    from spiderpig.materials import sheet

    rate = mill.mrr.get(sheet(part.stock).material, min(mill.mrr.values()))
    spots = sum(1 for o in part.ops if o.kind == "drill")
    return mill.setup_min + part.removed_mm3(mill) / rate + SPOT_MIN * spots


def setup_sheet(parts: list[tuple[str, int, MillPart]], mill: Mill) -> list[str]:
    """The setup sheet (markdown lines) of ``(label, qty, part)``: the work holding and the
    finishing, then per part its stock and blank, each operation with its tool, diameter,
    depth and fit, and the time."""
    from spiderpig.materials import sheet

    tl = mill.profile_tool
    lines = [
        f"# Milled parts: {mill.label}", "",
        f"Machine: {mill.label} ({mill.source}).", "",
        "## Holding and finishing", "",
        f"- Each blank is the part and {mill.clamp:g} mm round it. Screw the plate to a "
        "sacrificial spoilboard through the waste outside each blank (or tape and CA glue), "
        "clear of the profile's path and the collet nut. Several parts share a piece: lay "
        "their blanks side by side.",
        "- Zero on the blank's centre and its top face (probe it); measure the plate's "
        f"thickness first (a plate is made to about +-0.3 mm) and set the through cuts "
        f"{BREAKTHROUGH:g} mm past it.",
        "- Pockets: rough 0.1-0.2 mm under size, then a full-depth finish pass and a spring "
        "pass. Before the profile, try the part that fits it (a bearing in its seat): once "
        "the part is cut free it can't be zeroed again.",
        "- Drilled holes: spot them on the mill, drill them through on a drill press.",
        f"- The profile last ({tl.key}), on three or four tabs; cut the tabs, deburr every "
        "edge and chamfer each seat's mouth.", ""]
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
        order = sorted(part.ops, key=lambda o: ({"drill": 0, "pocket": 1}.get(o.kind, 2), -o.d))
        for i, o in enumerate(order, 1):
            through = o.depth is None
            depth = part.thickness + BREAKTHROUGH if o.depth is None else float(o.depth)
            if o.kind == "pocket":
                tool = mill.tool_for(o.d, depth)
                what = "pocket (spiral)"
            elif o.kind == "drill":
                tool, what = None, "spot, then drill (press)"
            else:
                tool, what = tl, "profile (tabs)"
            how_deep = "through" if through else f"{depth:g}"
            lines.append(f"| {i} | {what} | {tool.key if tool else '-'} | {o.d:g} | "
                         f"({o.x:.2f}, {o.y:.2f}) | {how_deep} | {o.fit or ''} | "
                         f"{o.purpose} |")
        lines.append("")
    lines += [f"Total: about {total:.0f} min of machine time (estimates).", ""]
    return lines


def write_outputs(mech, groups, types, out) -> list:
    """The milled parts' files under ``out`` (``mill/``): each distinct part's STEP in its
    own frame (its label's file: :func:`labels.part_types`), ``parts.csv`` and the setup
    sheet ``SETUP.md``. ``groups``: the ``milled`` made groups; ``types``: body -> part
    type. A milled body without its spec (``meta["milled"]``) is refused."""
    import csv
    from pathlib import Path

    from build123d import export_step

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    mill = MILLS[mech.meta["mill"]]
    specs = mech.meta.get("milled") or {}
    rows, sheet_parts, files = [], [], []
    for g in groups:
        if g.ref.name not in specs:
            raise ValueError(f"{g.ref.name} is milled but its construction gave no milling "
                             "spec (meta['milled'])")
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

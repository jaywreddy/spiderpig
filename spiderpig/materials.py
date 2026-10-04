"""What each laser-cut part is cut from: its sheet (material, thickness, service and the
service's cut rules), and the stock that fills a clearance gap.

Two materials (the user's direction of 2026-10-04): **acrylic** by default
(:attr:`config.BuildConfig.sheet`: the links, the spacer rings, the deck), **aluminium**
only where acrylic can't take the load: the frame plates and the robot's centre plates
(:attr:`config.BuildConfig.frame_sheet`, 5052, 0.125 in), the crank's plates
(:attr:`config.BuildConfig.crank_sheet`, 5052) and, on the Klann variants, the foot
links (6061-T6: 125-158 MPa at a jam, which no plastic survives;
:func:`default_link_sheets`). A layer is as thick as the thickest plate in it
(:func:`stack.finalize`), so a 3.175 mm aluminium plate and 3.0 mm acrylic stack at
their own z.

A **clearance gap** (:attr:`stack.Placed.gap`) is one of the thin sheets' thicknesses
(:func:`gap_options`): a plate stack it splits (a crank stack) gets a filler plate cut
from that sheet (:func:`filler_sheet`), and an axle that crosses it a stack of washers
and shims to that thickness (:func:`washer_stack`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import cache

from spiderpig.hardware.catalog import get

DEFAULT_SHEET = "acrylic_3mm"
FRAME_SHEET = "al5052_3p2mm"
CRANK_SHEET = "al5052_3p2mm"
FOOT_SHEET = "al6061_3p2mm"
THIN_SHEETS = ("acrylic_1mm", "acrylic_1p5mm", "acrylic_2mm", "al5052_1mm", "al5052_1p6mm",
               "al5052_2mm", "al5052_2p3mm", "al5052_2p5mm")


@dataclass(frozen=True)
class Sheet:
    """A sheet catalog item, as the build and the audit read it."""

    key: str
    name: str
    material: str          # "acrylic" | "aluminium" | "plywood"
    thickness: float
    density: float         # g/cm^3
    yield_mpa: float       # the allowable the strength check uses
    service: str
    min_hole: float        # mm
    edge_t: float          # least hole-to-edge distance, in thicknesses
    edge_mm: float         # ... and in mm (the least web the service cuts)
    min_part: tuple[float, float]   # the smallest part's (smaller, larger) side, mm
    corner_r: float        # an inside corner comes out this round (mm)
    metal: bool
    sheet_mm: tuple[float, float]

    @property
    def min_edge(self) -> float:
        """The least distance from a hole to an edge (or another hole) the service takes."""
        return max(self.edge_t * self.thickness, self.edge_mm)

    @property
    def label(self) -> str:
        """``"5052-H32 aluminium 3.175 mm (SendCutSend)"``."""
        alloy = get(self.key).dims.get("alloy")
        what = f"{alloy} aluminium" if alloy else self.material
        return f"{what} {self.thickness:g} mm ({self.service})"


@cache
def sheet(key: str) -> Sheet:
    """The sheet ``key`` (a catalog ``sheet`` item; rules a plain item lacks: none)."""
    it = get(key)
    d = it.dims
    t = float(d["thickness"])
    material = d.get("material") or ("plywood" if "plywood" in key else "acrylic")
    from spiderpig.hardware.mass import DENSITY

    return Sheet(
        key=key, name=it.name, material=material, thickness=t,
        density=float(d.get("density", DENSITY.get(material, DENSITY["acrylic"]))),
        yield_mpa=float(d.get("yield_mpa", 50.0)), service=str(d.get("service", "")),
        min_hole=float(d.get("min_hole", 0.0)), edge_t=float(d.get("edge_t", 0.0)),
        edge_mm=float(d.get("edge_mm", 0.0)),
        min_part=tuple(float(v) for v in d.get("min_part", (0.0, 0.0))),
        corner_r=float(d.get("corner_r", 0.0)), metal=bool(d.get("metal", False)),
        sheet_mm=tuple(float(v) for v in d.get("sheet_mm", (300.0, 300.0))))


def default_link_sheets(lk) -> dict[str, str]:
    """A linkage's links that aren't cut from the default sheet: a Klann variant's foot
    links (aluminium 6061-T6, the joinery plan of 2026-10-03)."""
    if lk.family == "klann":
        return {link: FOOT_SHEET for link, _ in lk.feet}
    return {}


def link_sheets(config) -> dict[str, str]:
    """Link class (``b4``) -> its sheet, for the links not cut from ``config.sheet``."""
    out = default_link_sheets(config.lk)
    if config.link_sheets is not None:
        out = dict(config.link_sheets)
    return {k: v for k, v in out.items() if v != config.sheet}


def sheet_of(config, role: str, link: str | None = None) -> str:
    """The sheet a part is cut from: ``role`` "frame" (frame and centre plates), "crank",
    "link" (``link``: its class or name), else the default sheet (rings, the deck)."""
    if role == "frame":
        return config.frame_sheet
    if role == "crank":
        return config.crank_sheet
    if role == "link" and link is not None:
        from spiderpig.stack import body_class

        return link_sheets(config).get(body_class(link), config.sheet)
    return config.sheet


def thickness(config, key: str) -> float:
    """A sheet's thickness in this build: the default sheet's may be measured
    (``config.thickness``)."""
    if key == config.sheet and config.thickness is not None:
        return float(config.thickness)
    return sheet(key).thickness


def gap_options() -> tuple[float, ...]:
    """The thicknesses a clearance gap may have: the thin sheets', thinnest first."""
    return tuple(sorted({sheet(k).thickness for k in THIN_SHEETS}))


def filler_sheet(material: str, t: float) -> str:
    """The thin sheet a ``t`` mm filler plate is cut from: of ``material`` when one is
    that thick, else any (acrylic first)."""
    near = [k for k in THIN_SHEETS if abs(sheet(k).thickness - t) < 1e-6]
    if not near:
        raise ValueError(f"no {t:g} mm thin sheet")
    same = [k for k in near if sheet(k).material == material]
    return (same or near)[0]


WASHERS: dict[float, tuple[str, str]] = {
    3.0: ("ptfe_washer_3x6x0p5", "shim_din988_3x6"),
    4.0: ("ptfe_washer_4x8x0p5", "shim_din988_4x8"),
    6.0: ("ptfe_washer_6x12x0p5", "shim_din988_6x12"),
}


def washer_family(shaft_d: float) -> tuple[str, str]:
    """(PTFE washer, DIN 988 shims) for a shaft of ``shaft_d`` mm (the nearest size not
    under it)."""
    for d in sorted(WASHERS):
        if d >= shaft_d - 1e-6:
            return WASHERS[d]
    raise ValueError(f"no washers for a {shaft_d:g} mm shaft")


def washer_od(shaft_d: float) -> float:
    ptfe, shim = washer_family(shaft_d)
    return max(float(get(ptfe).dims["od"]), float(get(shim).dims["od"]))


def washer_stack(shaft_d: float, t: float) -> tuple[list[tuple[str, float]], float]:
    """The washers that fill a ``t`` mm gap on a ``shaft_d`` shaft: one PTFE washer (the
    face a link turns on) when it fits, then DIN 988 shims thickest first; and the
    axial play left (under the smallest shim)."""
    ptfe, shim = washer_family(shaft_d)
    out: list[tuple[str, float]] = []
    left = round(t, 3)
    pt = float(get(ptfe).dims["t"])
    if left >= pt - 1e-6:
        out.append((ptfe, pt))
        left = round(left - pt, 3)
    for s in sorted((float(v) for v in get(shim).dims["t"]), reverse=True):
        k = int(math.floor(left / s + 1e-6))
        out += [(shim, s)] * k
        left = round(left - k * s, 3)
    return out, max(left, 0.0)

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
FRAME_SHEET = "al5052_3p2mm"      # (stage 1's; the defaults are now thinnest_sheet's)
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


# -- the thinnest sheet each part may be cut from ----------------------------------------
#
# The user's rule of 2026-10-04: no weight budget, but every plate as thin as strength and
# the service's cut rules allow, per part, aluminium above all. Each role's checks below
# are simple, conservative and named; :func:`thinnest_sheet` takes the thinnest stock sheet
# of the material that passes them all (and the cut rules the role's holes need).

ROLE_LOAD_N = 155.0     # the most loaded family's jam pin load (strength.GENERIC_PIN_LOADS)
FRAME_SPAN = 84.0       # the longest pillar bay between the frame plates (the Strider quad)
FRAME_WIDTH = 27.0      # the plate width a pillar's clamped end bends (3 x its 9 mm washer)
WEB_SPAN = 24.0         # the longest crankpin between its webs
WEB_WIDTH = 14.0        # a crank web's width
ROLE_SF = 2.0           # against the sheet's yield
CRANK_TWIST_NM = 2.0 * 0.85   # a crankpin's jam twist: chord / radius 2 x the torque limit


@dataclass(frozen=True)
class RoleCheck:
    """One role's checks: the out-of-plane bending a part takes at a pin's clamped end
    (a fixed-fixed shaft's end moment ``F L / 8`` over the plate's ``b t^2 / 6``), and the
    holes it must hold (SendCutSend: a hole at least the thickness)."""

    name: str
    load_n: float
    span: float
    width: float
    smallest_hole: float      # the smallest hole the part has (mm)
    twist_nm: float = 0.0     # (the crank) the jam twist a hex crankpin's pocket must carry
    hex_af: float = 5.5       # ... the hex standoff's across-flats (M3)
    hex_recess: float = 0.3   # ... the most its end may stand inside the pocket
    #                           (construction.crank.BoltCrank.recess_max)

    def hex_nm(self, sh: Sheet) -> float:
        """The twist a hex in this sheet's pocket carries over the sheet's thickness less the
        recess: :func:`construction.crank.hex_bearing_nm` at the sheet's yield (the steel
        standoff, 300 MPa, is stronger), each flat short by 0.3 mm for the standoff's
        rounded corners."""
        a = self.hex_af / math.sqrt(3) - 0.3
        return 0.75 * min(sh.yield_mpa, 300.0) * a * a * (sh.thickness - self.hex_recess) / 1e3

    def why_not(self, sh: Sheet) -> str | None:
        t = sh.thickness
        if self.twist_nm > 0 and self.hex_nm(sh) / self.twist_nm < ROLE_SF:
            return (f"{self.name}: a {self.hex_af:g} AF hex pocket holds "
                    f"{self.hex_nm(sh):.2f} N·m against a {self.twist_nm:g} N·m jam twist, "
                    f"SF {self.hex_nm(sh) / self.twist_nm:.2f} under {ROLE_SF:g}")
        m = self.load_n * self.span / 8
        sigma = 6 * m / (self.width * t * t)
        if sh.yield_mpa / sigma < ROLE_SF:
            return (f"{self.name}: {sigma:.0f} MPa at a {self.load_n:g} N jam, SF "
                    f"{sh.yield_mpa / sigma:.2f} under {ROLE_SF:g}")
        if sh.min_hole > self.smallest_hole + 1e-6:
            return (f"{self.name}: its {self.smallest_hole:g} mm holes under {sh.service}'s "
                    f"{sh.min_hole:g} mm minimum")
        return None


ROLES = {
    # the servo's M2 screw holes (2.4 mm) are the frame plate's smallest
    "frame": RoleCheck("frame plate (a pillar's clamped end)", ROLE_LOAD_N, FRAME_SPAN,
                       FRAME_WIDTH, 2.4),
    # the horn screws' 3.4 mm holes and the stub screw's are the crank plates' smallest
    # and since 2026-10-04 (the hex standoff crankpin) the hex pockets: the jam twist of
    # two crankpins 180 deg apart (chord / radius 2, the Strider double and every quad) at
    # the STS3215's 0.85 N·m torque limit
    "crank": RoleCheck("crank web (a crankpin's clamped end)", ROLE_LOAD_N, WEB_SPAN,
                       WEB_WIDTH, 3.4, twist_nm=CRANK_TWIST_NM),
}


def aluminium_sheets(alloy: str = "5052") -> list[str]:
    """Every stock sheet of ``alloy`` in the catalog, thinnest first."""
    from spiderpig.hardware.catalog import CATALOG

    keys = [k for k, it in CATALOG.items() if it.category == "sheet"
            and it.dims.get("alloy") and str(it.dims["alloy"]).startswith(alloy)]
    return sorted(keys, key=lambda k: (sheet(k).thickness, str(get(k).dims.get("alloy"))))


ROLE_ALLOYS = {"crank": ""}
"""A role whose thinnest sheet may be of any aluminium (``""``: 5052 or 6061, the thinner;
5052 on a tie): the crank's hex pockets (2026-10-04), where 6061-T6's 276 MPa lets a plate
that fits its 3 mm layer (0.100 in) hold the jam twist 5052 needs 0.125 in for, which is
thicker than the layer and moves every pillar's column off the stock lengths."""


@cache
def thinnest_sheet(role: str, alloy: str | None = None) -> str:
    """The thinnest stock ``alloy`` sheet that passes ``role``'s checks (:data:`ROLES`;
    ``alloy`` None: the role's, :data:`ROLE_ALLOYS`, else 5052)."""
    if alloy is None:
        alloy = ROLE_ALLOYS.get(role, "5052")
    check = ROLES[role]
    for key in aluminium_sheets(alloy):
        if check.why_not(sheet(key)) is None:
            return key
    raise ValueError(f"no {alloy} sheet passes the {role} checks")


def role_report(role: str, alloy: str | None = None) -> list[dict]:
    """Each stock sheet of ``alloy`` against ``role``'s checks (for the docs and the audit)."""
    if alloy is None:
        alloy = ROLE_ALLOYS.get(role, "5052")
    return [{"sheet": k, "thickness_mm": sheet(k).thickness,
             "why_not": ROLES[role].why_not(sheet(k))} for k in aluminium_sheets(alloy)]

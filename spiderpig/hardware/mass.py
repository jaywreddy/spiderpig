"""Materials and exact mass properties of fabricated parts: every density in one place.

What a body is made of follows ``Body.fab`` and its catalog item
(:func:`material_of`): laser-cut sheet at the sheet's density, printed parts
at the filament's (100 % infill), the servo at its datasheet mass, an item whose catalog
entry lists ``mass_g`` (the deck's electronics and nylon hardware) at that mass, a horn
aluminium or a plastic, heat-set inserts brass, other purchased parts steel.
:func:`part_props` reads a part's exact B-rep invariants once; the walking
model, the MuJoCo model and the bake share them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig.hardware.catalog import get
from spiderpig.stack import body_class

# g/cm^3
DENSITY = {
    "acrylic": 1.19,      # cast PMMA (ISO 1183; Röhm PLEXIGLAS GS: 1.19)
    "plywood": 0.68,      # Baltic birch plywood (typical 650-720 kg/m^3)
    "pla": 1.24,          # solid PLA, when no filament item says otherwise
    "steel": 7.85,        # carbon / alloy steel fasteners (EN 10025 / ISO 898)
    "brass": 8.5,         # heat-set inserts (CuZn39Pb3 free-cutting brass: 8.47)
    "aluminium": 2.70,    # 6061-T6 (ASM handbook)
    "plastic": 1.41,      # acetal, a plastic horn
    "bushing": 1.4,       # igus iglide
    "ptfe": 2.2,          # PTFE washers and tube liners
    "nylon": 1.14,        # PA66 standoffs, screws and nuts
}
SERVO_BOX_DENSITY = 1.5   # g/cm^3 of a servo's body box, when its spec has no weight


def sheet_density(sheet: str) -> float:
    """g/cm^3 of a sheet catalog item (its ``density``, else by name; acrylic by default)."""
    try:
        dens = get(sheet).dims.get("density")
    except KeyError:
        dens = None
    if dens:
        return float(dens)
    return next((DENSITY[k] for k in ("acrylic", "plywood") if k in sheet), DENSITY["acrylic"])


def filament_density(filament: str | None) -> float:
    """g/cm^3 of a filament catalog item (PLA when ``None`` or unknown)."""
    try:
        return float(get(filament or "pla_filament").dims.get("density", DENSITY["pla"]))
    except KeyError:
        return DENSITY["pla"]


def servo_mass_g(spec) -> float:
    """A servo's mass: its spec's ``weight_g``, else its body box at :data:`SERVO_BOX_DENSITY`."""
    if spec.weight_g:
        return float(spec.weight_g)
    return SERVO_BOX_DENSITY * math.prod(spec.body) / 1000.0


def _category(bom_key: str | None) -> str | None:
    if not bom_key:
        return None
    try:
        return get(bom_key).category
    except KeyError:
        return None


def _fixed_mass(bom_key: str | None) -> float | None:
    """A catalog item's ``mass_g`` (grams per piece), if it lists one."""
    if not bom_key:
        return None
    try:
        m = get(bom_key).dims.get("mass_g")
    except KeyError:
        return None
    return float(m) if m else None


def item_material(bom_key: str | None) -> str | None:
    """The material of a purchased item when it isn't steel: its ``dims["material"]``,
    else what its name says (aluminium or brass standoffs, PTFE washers, nylon hardware;
    a nylon-*insert* lock nut is steel); ``None``: the default (steel)."""
    if not bom_key:
        return None
    try:
        item = get(bom_key)
    except KeyError:
        return None
    if item.dims.get("material"):
        return str(item.dims["material"])
    name = item.name.lower()
    for word, material in (("alumin", "aluminium"), ("brass", "brass"), ("ptfe", "ptfe")):
        if word in name:
            return material
    if "nylon" in name.replace("nylon-insert", ""):
        return "nylon"
    return None


def material_of(body, sheet: str, filament: str | None, servo) -> tuple[str, float, float | None]:
    """``(material, density g/cm^3, fixed mass g)`` of a fabricated body.

    A fixed mass (the servo's) replaces volume x density. ``sheet`` and
    ``filament`` are the build's catalog items; ``servo`` its spec. A laser-cut body
    cut from another sheet than the build's default names it (``Body.sheet``: the frame
    plates' aluminium).
    """
    cls = body_class(body.name)
    category = _category(body.bom_key)
    if body.fab == "laser":
        return "sheet", sheet_density(getattr(body, "sheet", None) or sheet), None
    if body.fab == "printed":
        return "printed", filament_density(filament), None
    if body.fab != "purchased":
        raise ValueError(f"body {body.name!r} has a part but no known fabrication ({body.fab!r})")
    if category == "servo" or cls == "servo":
        return "servo", 0.0, servo_mass_g(servo)
    fixed = _fixed_mass(body.bom_key)
    if fixed is not None:       # a part catalogued by its mass (the deck's electronics)
        return ("electronics" if category == "electronics" else "nylon"), 0.0, fixed
    if category == "horn" or cls.startswith("servo_horn"):
        alu = "alumin" in servo.horn.name.lower()
        return ("aluminium", DENSITY["aluminium"], None) if alu else ("plastic", DENSITY["plastic"],
                                                                     None)
    material = item_material(body.bom_key)        # what the item says it is, first (a
    if material in DENSITY:                        # PTFE liner is a "bushing" too)
        return material, DENSITY[material], None
    if category in ("insert", "bushing"):
        return ("brass", DENSITY["brass"], None) if category == "insert" else (
            "bushing", DENSITY["bushing"], None)
    return "steel", DENSITY["steel"], None


@dataclass(frozen=True)
class PartProps:
    """Exact B-rep invariants of a part (mm): volume, surface area, the volume and surface
    centroids, the inertia tensor about the volume centroid at unit density, its z extent."""

    volume: float
    area: float
    com: np.ndarray
    surf_com: np.ndarray
    inertia: np.ndarray
    z_range: tuple[float, float]


def part_props(part) -> PartProps:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    vol, surf = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(part.wrapped, vol)
    BRepGProp.SurfaceProperties_s(part.wrapped, surf)
    c, sc = vol.CentreOfMass(), surf.CentreOfMass()
    m = vol.MatrixOfInertia()
    bb = part.bounding_box()
    return PartProps(
        volume=vol.Mass(), area=surf.Mass(),
        com=np.array([c.X(), c.Y(), c.Z()]), surf_com=np.array([sc.X(), sc.Y(), sc.Z()]),
        inertia=np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]),
        z_range=(bb.min.Z, bb.max.Z),
    )

"""Contract for pivot joinery options.

A *pivot site* is one physical axis through a stack of plates: a pin joining
links (``kind="pin"``), a fixed pivot joining links to the frame plate
(``kind="frame"``), or a crankpin joining b1 links to the crank plates
(``kind="crankpin"``). A joinery option turns a site into:

* the finished hole each plate needs at that axis (clearance, press fit,
  bushing or bearing bore, D-shape for a D-shaft);
* hardware bodies modelled in world coordinates (bolt, nut, bearing, dowel,
  printed pin, spacer ring...), each riding a host body;
* extra purchases that aren't modelled (shim washers, glue).

Before any of that, :meth:`Joinery.envelope` returns a
:class:`stack.Envelope`: how much room the option needs below, above and
between the plates, so the plan can guarantee nothing collides over the
whole crank cycle. Crankpins must be *flush* (``head_slots == tail_slots ==
0``): anything sticking out would sweep into a neighbouring b1.

Geometry conventions: Z is up, the stack is built from ``z = 0``; plates are
``pitch`` thick. The axis is vertical at ``site.xy``. Heads/tails must fit in
the slots the envelope reserved: ``head_slots`` slots of ``pitch`` directly
below the lowest member, ``tail_slots`` directly above the highest member
(for frame sites the highest member is the frame plate, and there is no
limit above it).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Protocol

from hardware.bom import BomLine
from mechanism import Body
from stack import Envelope  # noqa: F401  (the planner owns it; options return it)

SiteKind = Literal["pin", "frame", "crankpin"]


@dataclass(frozen=True)
class JoineryParams:
    """User-facing knobs shared by every option (mm)."""

    axle_d: float = 3.0              # nominal axle / bolt / dowel diameter
    running_clearance: float = 0.25  # diametral clearance: plate turning on an axle
    press_interference: float = 0.05  # diametral undersize for parts pressed into a plate
    spacers: bool = True             # add laser-cut spacer rings in gap slots that fit


@dataclass(frozen=True)
class Member:
    """A plate on the axis: its body name, Z range and whether the axle is fixed in it.

    ``fixed`` members hold the axle (the frame plate, crank plates); the
    others turn on it (leg links, b1 on a crankpin).
    """

    name: str
    z0: float
    z1: float
    fixed: bool = False


@dataclass(frozen=True)
class PivotSite:
    name: str                                  # e.g. "C_leg0", "A_leg1", "M_leg2"
    kind: SiteKind
    xy: tuple[float, float]                    # axis position in world XY at build time
    members: tuple[Member, ...]                # bottom to top
    host: str                                  # body the hardware rides with
    gap_slots: tuple[tuple[float, float], ...]  # z ranges where a spacer ring fits (checked)
    pitch: float
    params: JoineryParams = field(default_factory=JoineryParams)

    @property
    def bottom(self) -> float:
        return self.members[0].z0

    @property
    def top(self) -> float:
        return self.members[-1].z1


@dataclass(frozen=True)
class Hole:
    """A finished hole in a plate. ``flat`` > 0 makes it a D-hole (flat depth, mm)."""

    d: float
    flat: float = 0.0


@dataclass
class Hardware:
    holes: dict[str, Hole] = field(default_factory=dict)   # member name -> hole at this axis
    bodies: list[Body] = field(default_factory=list)
    extras: list[BomLine] = field(default_factory=list)


class Joinery(Protocol):
    """A joinery option. Implementations are small frozen dataclasses."""

    key: str          # registry key, e.g. "bolt"
    label: str        # human name, e.g. "M3 bolt + nylon lock nut"
    kinds: tuple[SiteKind, ...]   # site kinds this option can serve

    def envelope(self, params: JoineryParams, pitch: float) -> Envelope: ...

    def build(self, site: PivotSite) -> Hardware: ...


def hardware_body(name: str, part, host: str, *, fab: str, bom_key: str | None = None,
                  color: str = "gray") -> Body:
    """A hardware body riding ``host`` (world-coordinate part, no joints)."""
    return Body(name=name, part=part, color=color, rigid_with=host, fab=fab, bom_key=bom_key)

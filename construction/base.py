"""Contract for construction groups: how each functional part of a side gets built.

The design (the sympy kinematics, :mod:`klann`) says where joints are over
the crank cycle and nothing about material. A **side** of the walker is then
rationalized by *groups*, one per functional part:

=================  =========================================================
group              what it is
=================  =========================================================
``drive``          the servo on the inner frame plate, and its horn
``crank``          the crankshaft: couples to the horn, carries the crankpins
``pillar:<axis>``  a frame pivot: an axle held by both frame plates
``pin:<axis>``     a pivot between links
``links``          the laser-cut leg links
``frame``          the laser-cut inner and outer frame plates
=================  =========================================================

Each group is built by a *construction* chosen in the config (a printed
crank, a printed stepped axle, ...), and is rationalized in two passes:

1. :meth:`Group.claims` states the space the group needs, per layer and
   relative to the link layers it depends on (:class:`stack.Claim`). The
   planner (:mod:`stack`) finds link layers where all claims clear each
   other over the whole crank cycle; it never sees how a group is built.
2. :meth:`Group.realize` builds the group's parts for the solved plan: bodies
   (printed, purchased, laser), holes it needs in the plates
   (:attr:`Realized.cuts`), outline it needs added to the frame plates
   (:attr:`Realized.pads`) and unmodelled purchases (:attr:`Realized.extras`).

**Contract:** every part a group builds lies inside the shapes it claimed
(:mod:`construction.contract` checks it). Together with the planner's
guarantee, nothing can collide. A construction that can't be built with the
given parameters raises :class:`ConstructionError` before planning.

Groups run in dependency order (drive, crank, axles, links, frame;
:data:`construction.GROUP_FACTORIES` makes them): a later group may read an
earlier group's interface from :attr:`Context.interfaces`, and a group that
``cuts`` (the plates) realizes after every other, with what they asked for.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from hardware.bom import BomLine
from mechanism import Body
from shapes import Cut
from stack import Claim, Keepout, Layout, Placed, StackPlan, Topology

if TYPE_CHECKING:
    from mechanism import Mechanism
    from servos.spec import ServoSpec

XY = tuple[float, float]

# Plate keys for Realized.cuts / .pads besides link names.
FRAME_INNER = "frame:inner"
FRAME_OUTER = "frame:outer"


class ConstructionError(ValueError):
    """The chosen construction can't be built with these parameters."""


@dataclass(frozen=True)
class Params:
    """Dimensions every construction shares (mm). Defaults suit FDM + 3 mm sheet."""

    margin: float = 1.0            # clearance between parts that move relative to each other
    # laser-cut plates
    link_radius: float = 6.0       # half-width of a leg link (pill radius)
    frame_radius: float = 7.0      # half-width of a frame plate arm
    min_wall: float = 1.5          # thinnest laser-cut ring around a hole
    # fits (diametral clearances)
    running_fit: float = 0.35      # a part that turns in a laser-cut hole
    glue_fit: float = 0.15         # a part glued into a laser-cut hole
    print_fit: float = 0.3         # two printed parts that slide together
    # printed axles (pillars and link pins)
    axle_d: float = 6.0            # the diameter plates turn on
    spacer_d: float = 8.5          # shoulder beside a link (built-in spacer)
    neck_d: float = 4.0            # thinnest an axle may neck down where a link passes
    head_d: float = 8.5            # head / cap outside the plates it retains
    # printed crank
    crankpin_d: float = 6.0        # post b1 turns on
    web_radius: float = 6.0        # half-width of a crank web (O to crankpin)
    journal_d: float = 12.0        # crank body on the axis O
    stub_d: float = 8.0            # journal stub turning in the outer frame plate
    hub_thickness: float = 5.0     # coupling disc under the servo horn

    def hole(self, d: float, fit: str = "running") -> float:
        """Finished hole diameter for a part of diameter ``d``."""
        return d + {"running": self.running_fit, "glue": self.glue_fit}[fit]


@dataclass(frozen=True)
class DriveInterface:
    """What the crank couples to, in side coordinates.

    The servo sits on the inner frame plate's top face, output face down,
    output axis on O. Depths are measured down from that face.
    """

    horn_face_depth: float    # horn's outer face (where the crank bolts on)
    horn_radius: float
    horn_thickness: float
    screw_pcd: float          # horn screw circle
    screw_count: int
    screw_clearance_d: float  # clearance hole for a horn screw
    screw_head_d: float       # counterbore for its head
    screw_key: str | None     # catalog item for the horn screws
    center_head_d: float      # pocket for the horn's centre screw head
    center_head_h: float
    pattern_angle: float = 0.0  # first horn hole, radians from the crank's first crankpin


@dataclass
class Context:
    """Everything a group may read while stating claims or building parts."""

    topo: Topology
    params: Params
    pitch: float
    servo: ServoSpec
    config: object                              # fabricate.BuildConfig
    interfaces: dict[str, object] = field(default_factory=dict)

    def layout(self, layers, top: int) -> Layout:
        return Layout(layers, top, self.pitch)


@dataclass
class Build:
    """What :meth:`Group.realize` sees: the plan and the mechanism frozen at ``t``."""

    ctx: Context
    plan: StackPlan
    mech: Mechanism

    @property
    def layers(self) -> dict[str, int]:
        return self.plan.layers

    @property
    def top(self) -> int:
        return self.plan.top

    def z(self, layer: int) -> tuple[float, float]:
        return self.plan.z(layer)

    def xy(self, point: str) -> np.ndarray:
        """World XY of a topology point at this ``t``.

        A point a group added to the geometry (not a joint) is where the
        geometry says if it's fixed (e.g. a servo mounting screw); if it moves,
        it's fixed to the crank (:meth:`stack.Topology.add_crank_point`)
        and has turned with it about O.
        """
        node = self._node(point)
        if node is not None:
            body, joint = node
            b = self.mech.body(body)
            return (b.pose @ b.joint(joint).pose).matrix[:2, 3].copy()
        g = self.plan.topo.geometry.points
        p = np.asarray(g[point][0], dtype=float)
        if not np.ptp(g[point], axis=0).any():
            return p.copy()
        pin = self.plan.topo.axes_of("crankpin")[0].name
        v, w = p - g["O"][0], g[pin][0] - g["O"][0]
        turn = self.angle("O", pin) - math.atan2(w[1], w[0])
        c, s = math.cos(turn), math.sin(turn)
        return self.xy("O") + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

    def angle(self, a: str, b: str) -> float:
        d = self.xy(b) - self.xy(a)
        return math.atan2(d[1], d[0])

    def _node(self, point: str) -> tuple[str, str] | None:
        if not hasattr(self, "_nodes"):
            self._nodes: dict[str, tuple[str, str]] = {}
            for node, p in sorted(self.plan.topo.point_of.items()):
                self._nodes.setdefault(p, node)
        return self._nodes.get(point)

    def shapes(self, group: str) -> list[Placed]:
        return self.plan.shapes(group)


@dataclass
class Realized:
    """A group's contribution to the fabricated side."""

    bodies: list[Body] = field(default_factory=list)
    cuts: dict[str, list[Cut]] = field(default_factory=dict)       # plate -> holes
    # frame plate -> pills to add to its outline
    pads: dict[str, list[tuple[XY, XY, float]]] = field(default_factory=dict)
    extras: list[BomLine] = field(default_factory=list)

    def cut(self, plate: str, cut: Cut) -> None:
        self.cuts.setdefault(plate, []).append(cut)

    def pad(self, plate: str, p: XY, q: XY, r: float) -> None:
        self.pads.setdefault(plate, []).append((tuple(p), tuple(q), r))

    def merge(self, other: Realized) -> None:
        self.bodies.extend(other.bodies)
        for k, v in other.cuts.items():
            self.cuts.setdefault(k, []).extend(v)
        for k, v in other.pads.items():
            self.pads.setdefault(k, []).extend(v)
        self.extras.extend(other.extras)


class Group:
    """A functional part of a side, built by one construction (the contract above).

    A group implements :meth:`claims` and :meth:`realize`; :meth:`keepouts`
    and :meth:`interface` are optional. ``cuts``: the group cuts what the
    others asked for (holes, pads), so it realizes after them.
    """

    name: str
    cuts: bool = False

    def keepouts(self, ctx: Context) -> list[Keepout]:
        """Space the group needs whatever the layout (:class:`stack.Keepout`); the static
        stage checks every link against it."""
        return []

    def interface(self, ctx: Context) -> object | None:
        """What later groups may read as ``ctx.interfaces[name]`` (``None``: nothing)."""
        return None

    def claims(self, ctx: Context) -> list[Claim]:
        raise NotImplementedError

    def realize(self, build: Build, done: Realized) -> Realized:
        """The group's parts for the solved plan; ``done`` is what the groups before it
        built (a plate cuts the holes and adds the pads they asked for)."""
        raise NotImplementedError


def hardware(name: str, part, host: str, *, fab: str, bom_key: str | None = None,
             color: str | None = None) -> Body:
    """A non-kinematic body riding ``host`` (world-coordinate part)."""
    return Body(name=name, part=part, color=color, rigid_with=host, fab=fab, bom_key=bom_key)

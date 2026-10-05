"""The electronics deck: a laser-cut plate between the two inner frame plates, above the
servos, carrying the robot's electronics (:mod:`hardware.electronics`, option 1).

Where it is and why
-------------------
Between the two inner frame plates nothing moves: the legs, the crank and the pillars are
all outboard of them (the planner puts every link in a layer below an inner plate; a
pillar ends at the inner plate's face), and the kinematics is planar, so no moving part
ever changes its ``z``. The space over the servos, between the plates (``|z| <`` the
inner faces, 73 mm on the default designs) and from the chassis' top (the centre plates,
tie columns and servos, :func:`deck_floor`) up to the plates' top edges, is therefore
free over the whole crank cycle, and walled in by the plates against a side knock.
:func:`deck_clearance` proves it for a fabricated robot from the ``z`` extents (exact
over the cycle) and the audit's OCCT clash check confirms it at its crank angles.

Construction
------------
* **Rails** (printed, one per side, ``L.deck_rail`` / ``R.deck_rail``): a 60 x 11 x 8 mm
  bar on the inner plate's servo-side face, screwed to it (no glue since 2026-10-04) by two
  M3 button heads up through the plate from the leg side into M3 nuts dropped into traps
  in the rail (their heads in the clearance gap under the plate, the drive group's claim;
  :class:`robot.FrameTies` cuts the holes: :func:`rail_screw_points`). Two M3 heat-set inserts
  per rail, vertical, take the deck's screws. The rails sit 1 mm above the chassis' top.
* **Deck plate** (laser-cut, the build's sheet): 136 x 71 mm, 1 mm from each inner
  plate, centred over the servos, screwed to the rails with four M3 SHCS into the
  inserts. Cut-outs: the screws' clearance holes, the board's four M2.5 holes, two strap
  slots beside the battery, a 6.4 mm hole for the switch's bushing and two 12 x 6 mm
  wire slots, one each side of the board over a servo: each takes that servo's bus
  cable (connector first) up to the board and one pair of the power harness (the
  battery's XT30 mates on top with an XT30 pigtail whose wires go down to the protection
  board; the switched supply comes back up to the board's DC jack).
* **Driver board** on top, the front half (+x), on four M2.5 x 6 nylon standoffs (male
  end through the deck, a nylon nut under it), 2 mm in from the front edge with its DC jack
  at the front, so the switched supply's right-angle plug goes in from the open front of
  the bay (2026-10-05: the jack facing the battery left 4-5 mm for a plug that needs 12;
  Waveshare's STEP model puts the jack on a short end and the USB-C on a long edge, 47-57
  mm from the jack end, so the USB-C faces an inner plate about 21 mm away: flash the board
  before the deck goes in, then over the air, or with a right-angle USB-C cable).
  **Battery** on top, the rear half, in a printed cradle (a 5 mm rim, 1.6 mm
  walls, open at the inner end for the leads; screwed to the deck through two ears by M3
  button heads from above into M3 nuts under the deck, no glue: the user's decision of
  2026-10-05) and held by a 10 mm hook-and-loop strap through the two slots.
  **Charger** (IP2326) under the deck at the front, its USB-C end at the deck's front edge
  (0.8 mm back on the Strider, where a pillar's head stands in its way down: below);
  **protection board** under the
  deck behind the servos; both on foam tape. **Toggle switch** through the deck at the
  rear, lever up, its body hanging below the deck behind the chassis.
* Every port is reachable with the robot assembled: the board's DC jack and the charger's
  USB-C face forward out of the open front of the bay between the plates, the switch lever
  points up out of its open top; the board's USB-C faces an inner plate (above). The
  divider (100k / 33k, battery + to an ADC pin) is
  wired in the harness: BOM only.
* Length: 136 mm is the board (65) and the battery's cradle (65.7) end to end. It sits
  inside the Strider's inner plates (x +-78.5) but overhangs the Klann quad's (+-61.9)
  by 6 mm at each end, where nothing moves (every leg is outboard of the plates).
* UNVERIFIED: the board's connector edges (from the product photo), every component
  height (the barrel jack taken as 11 mm), the charger's and the protection board's
  masses; the MTS-102's DC rating (3 A at 250 V AC; two stalled STS3215 draw ~5 A at 2S:
  switch the supply only with the servos idle, or fit a 6 A DC switch).

* **Lowering it in** (2026-10-05): the pillars' inner M4 heads and washers stand 3 mm
  into the bay from each inner plate's face, where the deck's edges pass. The deck plate is
  notched round every static part in its path (:func:`path_notches`, from the robot's real
  parts: on the Strider double the four corners, round J2's and J6's heads) and the charger
  steps back from the front edge as far as a head in its band needs, so the whole deck,
  electronics on, goes straight down onto the rails; :func:`deck_clearance` proves it from
  the parts' geometry (``blocked``: each lowered part swept straight up against everything
  else; an audit problem).

Assembly (the last step of :data:`construction.robot.ASSEMBLY`): screw the rails to the
inner plates with the sides (before the legs), fit the electronics to the deck (the cradle
screwed on, wires tied down through the cable-tie slots beside each wire slot), join the
sides, lower the deck between the plates onto the rails and screw it down.

Everything here is a body of the robot (mass, centre of mass, BOM, DXF, bake): the
sim's and the walking model's electronics are these parts, not an allowance.
"""

from __future__ import annotations

import math
import re
from collections.abc import Sequence
from dataclasses import dataclass

from build123d import Axis, Box, Cylinder, Pos, scale

from spiderpig.construction.base import Build, ConstructionError, Context
from spiderpig.construction.chassis import (
    BRASS,
    MODEL_GAP,
    STEEL,
    seat_keepouts,
    servo_frame_ctx,
    tie_dims,
    tie_pad_r,
    tie_points_ctx,
)
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get, pick_length
from spiderpig.hardware.fasteners import CLEARANCE, screw
from spiderpig.mechanism import Body
from spiderpig.shapes import union
from spiderpig.stack import body_class

DECK_SCREW = screw("shcs", "3")
DECK_GAP = 1.0           # deck plate edge to an inner plate's face (mm): frame tolerance
FLOOR_MARGIN = 1.0       # a rail's underside above the chassis' top
HALF_LEN = 68.0          # the deck plate's half length along x (mm)
RAIL_HALF = 30.0         # a rail's half length
RAIL_T = 11.0            # a rail's thickness (z): 3.5 mm insert walls, and the deck's
#                          screw holes 3.3 mm from the plate's edge
RAIL_H = 8.0             # a rail's height (y): the insert's 5.7 mm and a 1.3 mm floor
INSERT_X = 24.0          # the rails' inserts at x_c +- this
SPIGOT_XS = (12.0, 16.0, 8.0, 20.0)      # rail screw offsets tried, nearest-first preference
RAIL_SCREW = screw("bhcs", "3")          # up through the inner plate from the leg side
RAIL_HOLE = 3.4          # its hole in the inner plate (ISO 273 medium; over the 5052's 3.175)
RAIL_SCREW_R = 5.7 / 2 + 0.3             # its head's clearance shape under the plate
RAIL_NUT_AF, RAIL_NUT_H = 5.5, 2.4       # an M3 hex nut in a trap in the rail
NUT_DEPTH = 3.0          # the trap's floor over the rail's plate face
CABLE_TIE_SLOT = (4.0, 2.0)              # beside each wire slot, for a 2.5 mm cable tie
RAIL_SCREW_L = 10.0      # through the 3.175 mm plate, past the nut trap
STANDOFF_AF = 5.0
BOARD_X0 = 1.0           # the board's inner end, from x_c
JACK_PROUD = 0.9         # the board's DC jack past its front end (Waveshare's STEP model)
BATTERY_X1 = -4.0        # the cradle's inner end (inside), from x_c
BATTERY_FIT = 0.3        # cradle clearance round the battery (each way)
CRADLE_WALL, CRADLE_H, CRADLE_GAP = 1.6, 5.0, 10.0
STRAP_SLOT = (12.0, 3.0)  # along x, across (z)
WIRE_SLOT = (12.0, 6.0)
WIRE_SLOT_Z = 21.0
TAPE_GAP = 0.3           # modelled gap for the foam tape under the deck
SWITCH_X, SWITCH_Z = -55.0, 21.0     # +z: balances the charger (-z); 21 (was 20): 1 mm
#                                      clear of the cradle's outer screw's nut under the deck
CRADLE_SCREW = screw("bhcs", "3")    # the cradle's two ears to the deck, nuts under it
CRADLE_EARS = ((3.5, 1.0), (19.5, -1.0))   # (x from the cradle's outer inside end, z side):
#                                      clear of the switch's body and the strap's run
#                                      under the deck, and of the BMS (behind the servos)
EAR_R, EAR_H = 3.5, 3.0              # an ear's radius round its screw, its height
EAR_HEAD_GAP = 0.25                  # the screw head's edge to the cradle wall
NOTCH_CLEAR = 0.5                    # a path notch's clearance round what it passes
SETBACK_MAX = 3.0                    # the charger's USB-C end back from the front edge, at most
PAD_STRIP = 3.0                      # the width of the charger pad's two strips (z)
EPS_D = 1e-6
DECK_COLOR = "#eb6834"
RAIL_COLOR = "#2a7ab0"
PCB_COLOR = "#1f6b3a"
BATTERY_COLOR = "#3b3f46"
NYLON = "#e8e4d8"


def _box(x0, x1, y0, y1, z0, z1):
    return Box(x1 - x0, y1 - y0, z1 - z0).move(Pos((x0 + x1) / 2, (y0 + y1) / 2,
                                                  (z0 + z1) / 2))


def _cyl_y(x, z, r, y0, y1):
    """A cylinder along y (vertical in the mech frame)."""
    return Cylinder(radius=r, height=y1 - y0).rotate(Axis.X, 90).move(
        Pos(x, (y0 + y1) / 2, z))


def _cyl_z(x, y, r, z0, z1):
    return Cylinder(radius=r, height=z1 - z0).move(Pos(x, y, (z0 + z1) / 2))


# ---------------------------------------------------------------------------
# Where the deck goes (side level: the rails' spigots are cut in the inner plate)
# ---------------------------------------------------------------------------


def _ctx(build_or_ctx) -> Context:
    return getattr(build_or_ctx, "ctx", build_or_ctx)


def deck_floor(build: Build | Context, drive=None) -> float:
    """World y of the chassis' top between the plates: the servo body's corners, the tie
    columns and the centre plates grown round them (:mod:`construction.chassis`)."""
    ctx = _ctx(build)
    spec = ctx.servo
    frame = servo_frame_ctx(ctx)
    L, W, _ = spec.body
    x0, x1 = spec.axis_offset - L / 2, spec.axis_offset + L / 2
    ys = [frame.xy(x, y)[1] for x in (x0, x1) for y in (-W / 2, W / 2)]
    ys += [xy[1] + tie_pad_r(ctx) for xy in tie_points_ctx(ctx)]   # centre plate corner
    return max(ys)


def deck_centre_x(build: Build | Context, drive=None) -> float:
    """World x the deck is centred on: the servo body's centre."""
    ctx = _ctx(build)
    return servo_frame_ctx(ctx).xy(ctx.servo.axis_offset, 0.0)[0]


@dataclass(frozen=True)
class DeckPlace:
    """The deck's place (mech mm): centre x, the rails' underside, the deck's underside, and
    the rail screws' x offsets from the centre."""

    x_c: float
    rail_y0: float
    deck_y: float
    spigot_x: float

    @property
    def spigot_y(self) -> float:
        return self.rail_y0 + RAIL_H / 2


def deck_place(build: Build | Context, drive=None) -> DeckPlace:
    """Where the deck goes on this side's frame (known before a plan: the rail screws'
    heads under the inner plate are the drive group's claims), or
    :class:`ConstructionError` when the rail screws find no free spot in the inner plate
    (a pillar's end, the horn's hole or a tie in the way)."""
    ctx = _ctx(build)
    p = ctx.params
    x_c = deck_centre_x(ctx)
    rail_y0 = deck_floor(ctx) + FLOOR_MARGIN
    keep_out = seat_keepouts(ctx)
    keep_out += [(xy, tie_dims(ctx).head_r) for xy in tie_points_ctx(ctx)]
    y = rail_y0 + RAIL_H / 2
    for sx in SPIGOT_XS:
        pts = [(x_c - sx, y), (x_c + sx, y)]
        if all(math.dist(q, c) >= RAIL_SCREW_R + r + p.min_wall for q in pts
               for c, r in keep_out):
            return DeckPlace(x_c, rail_y0, rail_y0 + RAIL_H, sx)
    raise ConstructionError("the deck rails' screws find no free spot in the inner frame "
                            f"plate at y = {y:.1f} mm (tried x offsets {SPIGOT_XS})")


def rail_screw_points(build: Build | Context, drive=None) -> list[tuple[float, float]]:
    """World XY of the rails' screws through this side's inner plate."""
    d = deck_place(build)
    return [(d.x_c - d.spigot_x, d.spigot_y), (d.x_c + d.spigot_x, d.spigot_y)]


spigot_points = rail_screw_points        # (the rails were glued on spigots until 2026-10-04)


# ---------------------------------------------------------------------------
# The deck's parts (robot level, world coordinates, mid-plane z = 0)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DeckLayout:
    """Every placement the parts and the checks share (mech mm)."""

    x_c: float
    rail_y0: float
    deck_y: float        # deck plate underside
    pitch: float         # deck plate thickness
    z_in: float          # the left inner plate's servo-side face (negative)
    z_leg: float         # the left inner plate's leg-side face
    spigot_x: float

    @property
    def deck_top(self) -> float:
        return self.deck_y + self.pitch

    @property
    def half_w(self) -> float:
        return -self.z_in - DECK_GAP

    def board(self) -> dict:
        b = get("esp32_servo_driver").dims
        x0 = self.x_c + BOARD_X0
        y0 = self.deck_top + get("m25_nylon_standoff_mf_6").dims["length"]
        px, pz = b["hole_pitch"]
        inset = (b["length"] - px) / 2
        holes = [(x0 + inset + i * px, s * pz / 2) for i in (0, 1) for s in (-1, 1)]
        return {"x0": x0, "x1": x0 + b["length"], "y0": y0, "y1": y0 + b["pcb"],
                "half_w": b["width"] / 2, "holes": holes, "dims": b}

    def battery(self) -> dict:
        b = get("lipo_2s_450").dims
        ix1 = self.x_c + BATTERY_X1
        ix0 = ix1 - b["length"] - 2 * BATTERY_FIT
        hw = b["width"] / 2
        return {"x0": ix0 + BATTERY_FIT, "x1": ix1 - BATTERY_FIT, "half_w": hw,
                "y0": self.deck_top, "y1": self.deck_top + b["height"],
                "inner": (ix0, ix1, hw + BATTERY_FIT), "mid": (ix0 + ix1) / 2}

    def strap_slots(self) -> list[tuple[float, float]]:
        _, _, ihw = self.battery()["inner"]
        z = ihw + CRADLE_WALL + 0.5 + STRAP_SLOT[1] / 2
        return [(self.battery()["mid"], s * z) for s in (-1, 1)]

    def wire_slots(self) -> list[tuple[float, float]]:
        return [(self.x_c, s * WIRE_SLOT_Z) for s in (-1, 1)]

    def screws(self) -> list[tuple[float, float]]:
        zc = self.z_in + RAIL_T / 2
        return [(self.x_c + s * INSERT_X, side * -zc) for side in (-1, 1) for s in (-1, 1)]

    def switch(self) -> tuple[float, float]:
        return self.x_c + SWITCH_X, SWITCH_Z

    def cradle_ears(self) -> list[tuple[float, float]]:
        """XZ of the cradle's two screws (through its ears and the deck)."""
        ix0, _, ihw = self.battery()["inner"]
        z = ihw + CRADLE_WALL + CRADLE_SCREW.head_d / 2 + EAR_HEAD_GAP
        return [(ix0 + dx, side * z) for dx, side in CRADLE_EARS]


def layout(design, z_mid: float, place: DeckPlace) -> DeckLayout:
    plan, pitch = design.plan, design.ctx.pitch
    z_top = plan.z(plan.top)[1] - z_mid
    return DeckLayout(x_c=place.x_c, rail_y0=place.rail_y0, deck_y=place.deck_y, pitch=pitch,
                      z_in=z_top, z_leg=z_top - plan.t(plan.top), spigot_x=place.spigot_x)


def _rail(lay: DeckLayout, side: str):
    """A side's rail, its deck inserts' pockets and its two screws' bores and nut traps cut
    (world coordinates): the nut drops into its trap from the rail's top face before the
    deck goes on; the screw comes up through the inner plate from the leg side."""
    ins = get("m3_heat_set_insert").dims
    sign = 1.0 if side == "L" else -1.0           # the right rail is the left one mirrored
    z_face, z_far = lay.z_in, lay.z_in + RAIL_T
    zs = sorted((sign * z_face, sign * z_far))
    y0, y1 = lay.rail_y0, lay.deck_y
    part = _box(lay.x_c - RAIL_HALF, lay.x_c + RAIL_HALF, y0, y1, *zs)
    pocket = max(ins["length"], 5.0) + 1.0
    zc = sign * (lay.z_in + RAIL_T / 2)
    for s in (-1, 1):
        part = part - _cyl_y(lay.x_c + s * INSERT_X, zc, ins["hole_d"] / 2, y1 - pocket, y1 + 1)
    ym = (y0 + y1) / 2
    for s in (-1, 1):
        x = lay.x_c + s * lay.spigot_x
        bore = _cyl_z(x, ym, 1.7, *sorted((sign * (z_face - 1), sign * (z_face + RAIL_SCREW_L))))
        n0 = z_face + NUT_DEPTH
        trap = _box(x - RAIL_NUT_AF / 2 - 0.15, x + RAIL_NUT_AF / 2 + 0.15, ym - 3.3, y1 + 1,
                    *sorted((sign * n0, sign * (n0 + RAIL_NUT_H + 0.3))))
        part = part - bore - trap
    return part


Box6 = tuple[float, float, float, float, float, float]     # x0, x1, y0, y1, z0, z1


def lowered(name: str) -> bool:
    """A body that goes in with the deck, lowered onto the rails as one unit (the plate and
    everything fitted to it on the bench); the rails, their screws, nuts and inserts are on
    the inner plates already, the deck's four screws go in after."""
    if "deck" not in name:
        return False
    return not ("deck_rail" in name or "deck_insert" in name
                or re.fullmatch(r"deck_screw\d+", name) is not None)


def _overlap(a0: float, a1: float, b0: float, b1: float) -> bool:
    return a0 < b1 - EPS_D and b0 < a1 - EPS_D


def path_notches(lay: DeckLayout, obstacles: Sequence[Box6]) -> list[tuple[float, ...]]:
    """XZ rectangles ``(x0, x1, z0, z1)`` cut from the deck plate so it lowers straight down
    past ``obstacles`` (static parts' boxes over its underside: the pillars' inner M4
    heads): each grown by :data:`NOTCH_CLEAR`, opened to the nearer side edge, and to the
    end when it comes within a thickness of it (no sliver left)."""
    x0, x1, hw = lay.x_c - HALF_LEN, lay.x_c + HALF_LEN, lay.half_w
    out = []
    for ox0, ox1, _, oy1, oz0, oz1 in obstacles:
        nx0, nx1 = ox0 - NOTCH_CLEAR, ox1 + NOTCH_CLEAR
        nz0, nz1 = oz0 - NOTCH_CLEAR, oz1 + NOTCH_CLEAR
        if oy1 <= lay.deck_y + EPS_D or not (_overlap(nx0, nx1, x0, x1)
                                             and _overlap(nz0, nz1, -hw, hw)):
            continue
        if nz0 + nz1 < 0:
            nz0 = -hw - 1.0
        else:
            nz1 = hw + 1.0
        if nx1 > x1 - lay.pitch:
            nx1 = x1 + 1.0
        if nx0 < x0 + lay.pitch:
            nx0 = x0 - 1.0
        out.append((nx0, nx1, nz0, nz1))
    return out


def _clear_x1(x1: float, length: float, y0: float, z0: float, z1: float,
              obstacles: Sequence[Box6]) -> float:
    """The farthest ``+x`` end, at most ``x1``, a part ``length`` long over ``y0`` in
    ``z0..z1`` can have and still lower past ``obstacles``."""
    for _ in range(len(obstacles) + 1):
        hit = [ox0 for ox0, ox1, _, oy1, oz0, oz1 in obstacles
               if oy1 > y0 + EPS_D and _overlap(oz0, oz1, z0 - NOTCH_CLEAR, z1 + NOTCH_CLEAR)
               and _overlap(ox0, ox1, x1 - length - NOTCH_CLEAR, x1 + NOTCH_CLEAR)]
        if not hit:
            return x1
        x1 = min(hit) - NOTCH_CLEAR
    raise ConstructionError("the deck's charger finds no place to lower past the parts "
                            "between the inner plates")


def _charger_place(lay: DeckLayout, ch: dict, yu: float, obstacles: Sequence[Box6],
                   nuts: Sequence[tuple[float, float, float]], nut_h: float
                   ) -> tuple[float, float, float]:
    """``(x1, z0, pad)`` of the charger under the deck: its USB-C end ``x1`` at the front
    edge, 1 mm in from the left inner plate (``z0``), where nothing stands in its way down;
    else stepped back up to :data:`SETBACK_MAX` from the edge (the Strider's corner heads);
    else at the edge, moved in past what stands there (a pillar's head 3 mm into the bay),
    on a printed pad (``pad`` mm) under the board's nuts when that brings it over them."""
    front = lay.x_c + HALF_LEN
    L, w, h = ch["length"], ch["width"], ch["height"]
    z0 = -(lay.half_w - 1.0)
    try:
        x1 = _clear_x1(front, L, yu - h, z0, z0 + w, obstacles)
    except ConstructionError:
        x1 = -math.inf
    if front - x1 <= SETBACK_MAX + EPS_D:
        return x1, z0, 0.0
    for _ in range(len(obstacles) + 1):
        hit = [oz1 for ox0, ox1, _, oy1, oz0, oz1 in obstacles
               if oy1 > yu - h - nut_h - 1.0 and oz1 < 0
               and _overlap(oz0, oz1, z0 - NOTCH_CLEAR, z0 + w + NOTCH_CLEAR)
               and _overlap(ox0, ox1, front - L - NOTCH_CLEAR, front + NOTCH_CLEAR)]
        if not hit:
            break
        z0 = max(hit) + NOTCH_CLEAR
    else:
        raise ConstructionError("the deck's charger finds no place to lower past the parts "
                                "between the inner plates")
    over = any(_overlap(x - r, x + r, front - L, front) and _overlap(z - r, z + r, z0, z0 + w)
               for x, z, r in nuts)
    return front, z0, (nut_h + 0.3 if over else 0.0)


def deck_parts(design, z_mid: float, place: DeckPlace, host: dict[str, str],
               obstacles: Sequence[Box6] = ()
               ) -> tuple[list[Body], list[BomLine], dict, list[tuple[str, str]]]:
    """The rails, inserts, screws, deck plate and electronics (world coordinates): bodies,
    the purchases they don't model, what ``mech.meta["deck"]`` says, and the fastened
    pairs. ``obstacles``: the boxes of the static parts between the inner plates (the
    chassis', the pillars' inner heads), which the deck must lower past: the plate is
    notched round them (:func:`path_notches`) and the charger steps back from the front
    edge; another lowered part in one's way is a :class:`ConstructionError`."""
    lay = layout(design, z_mid, place)
    ins = get("m3_heat_set_insert").dims
    bodies: list[Body] = []
    fastened: list[tuple[str, str]] = []
    extras: list[BomLine] = []
    yd, yt, hw = lay.deck_y, lay.deck_top, lay.half_w

    # rails, screwed to the inner plates (no glue: the deck and its rails come off), and
    # their inserts
    for s in ("L", "R"):
        sign = 1.0 if s == "L" else -1.0
        bodies.append(Body(name=f"{s}.deck_rail", part=_rail(lay, s), rigid_with=host[s],
                           fab="printed", color=RAIL_COLOR))
        ym = (lay.rail_y0 + lay.deck_y) / 2
        for i, sx in enumerate((-1, 1)):
            x = lay.x_c + sx * lay.spigot_x
            leg = lay.z_leg
            head = _cyl_z(x, ym, RAIL_SCREW.head_d / 2,
                          *sorted((sign * leg, sign * (leg - RAIL_SCREW.head_h))))
            shank = _cyl_z(x, ym, 1.45, *sorted((sign * leg, sign * (leg + RAIL_SCREW_L))))
            n0 = lay.z_in + NUT_DEPTH
            nut = (_cyl_z(x, ym, RAIL_NUT_AF / 2, *sorted((sign * n0, sign * (n0 + RAIL_NUT_H))))
                   - _cyl_z(x, ym, 1.5, *sorted((sign * (n0 - 1), sign * (n0 + 4)))))
            bodies += [Body(name=f"{s}.deck_rail_screw{i}", part=union([head, shank]),
                            rigid_with=host[s], fab="purchased",
                            bom_key=RAIL_SCREW.key(RAIL_SCREW_L), color=STEEL),
                       Body(name=f"{s}.deck_rail_nut{i}", part=nut, rigid_with=host[s],
                            fab="purchased", bom_key="m3_nut", color=STEEL)]
            fastened.append((f"{s}.deck_rail_screw{i}", f"{s}.deck_rail_nut{i}"))
    length = pick_length(lay.pitch + min(5.0, ins["length"]), DECK_SCREW.lengths)
    key = DECK_SCREW.key(length)
    per_side = {"L": 0, "R": 0}         # each side's inserts numbered from 0: the right
    for i, (x, z) in enumerate(lay.screws()):     # side's are the left's mirrored, by name
        s = "L" if z < 0 else "R"
        k = per_side[s]
        per_side[s] += 1
        insert = (_cyl_y(x, z, ins["hole_d"] / 2 - MODEL_GAP, yd - ins["length"], yd)
                  - _cyl_y(x, z, DECK_SCREW.d / 2, yd - ins["length"] - 1, yd + 1))
        scr = union([_cyl_y(x, z, DECK_SCREW.head_d / 2, yt, yt + DECK_SCREW.head_h),
                     _cyl_y(x, z, DECK_SCREW.d / 2 - 0.05, yt - length, yt)])
        bodies += [Body(name=f"{s}.deck_insert{k}", part=insert, rigid_with=host[s],
                        fab="purchased", bom_key="m3_heat_set_insert", color=BRASS),
                   Body(name=f"deck_screw{i}", part=scr, rigid_with=host["L"],
                        fab="purchased", bom_key=key, color=STEEL)]
        fastened.append((f"deck_screw{i}", f"{s}.deck_insert{k}"))

    # the deck plate, notched where it passes the pillars' inner heads on its way down
    plate = _box(lay.x_c - HALF_LEN, lay.x_c + HALF_LEN, yd, yt, -hw, hw)
    notches = path_notches(lay, obstacles)
    cuts = [_cyl_y(x, z, CLEARANCE["3"] / 2, yd - 1, yt + 1) for x, z in lay.screws()]
    board = lay.board()
    cuts += [_cyl_y(x, z, CLEARANCE["2p5"] / 2, yd - 1, yt + 1) for x, z in board["holes"]]
    ties = [((x + s * (WIRE_SLOT[0] / 2 + 3.0), z), CABLE_TIE_SLOT)
            for x, z in lay.wire_slots() for s in (-1, 1)]
    for (x, z), (a, b) in ([(c, STRAP_SLOT) for c in lay.strap_slots()]
                           + [(c, WIRE_SLOT) for c in lay.wire_slots()] + ties):
        cuts.append(_box(x - a / 2, x + a / 2, yd - 1, yt + 1, z - b / 2, z + b / 2))
    sx, sz = lay.switch()
    sw = get("toggle_mts102").dims
    cuts.append(_cyl_y(sx, sz, sw["hole_d"] / 2, yd - 1, yt + 1))
    cuts += [_cyl_y(x, z, CLEARANCE["3"] / 2, yd - 1, yt + 1) for x, z in lay.cradle_ears()]
    cuts += [_box(nx0, nx1, yd - 1, yt + 1, nz0, nz1) for nx0, nx1, nz0, nz1 in notches]
    plate = plate - union(cuts)
    bodies.append(Body(name="deck_plate", part=plate, rigid_with=host["L"], fab="laser",
                       color=DECK_COLOR))

    # the driver board on its standoffs
    so, nut, bs = (get(k).dims for k in ("m25_nylon_standoff_mf_6", "m25_nylon_nut",
                                          "m25_nylon_screw_5"))
    r_af = STANDOFF_AF / 2
    for i, (x, z) in enumerate(board["holes"]):
        standoff = union([_cyl_y(x, z, r_af, yt, yt + so["length"]),
                          _cyl_y(x, z, 1.2, yt - so["thread"], yt)])
        standoff = standoff - _cyl_y(x, z, 1.25, yt + so["length"] - 5.0, yt + so["length"] + 1)
        nut_part = _cyl_y(x, z, r_af, yd - nut["h"], yd) - _cyl_y(x, z, 1.25, yd - 5, yd + 1)
        y_b = board["y1"]
        scr = union([_cyl_y(x, z, bs["head_d"] / 2, y_b, y_b + bs["head_h"]),
                     _cyl_y(x, z, 1.2, y_b - bs["length"], y_b)])
        bodies += [Body(name=f"deck_standoff{i}", part=standoff, rigid_with=host["L"],
                        fab="purchased", bom_key="m25_nylon_standoff_mf_6", color=NYLON),
                   Body(name=f"deck_nut{i}", part=nut_part, rigid_with=host["L"],
                        fab="purchased", bom_key="m25_nylon_nut", color=NYLON),
                   Body(name=f"deck_board_screw{i}", part=scr, rigid_with=host["L"],
                        fab="purchased", bom_key="m25_nylon_screw_5", color=NYLON)]
        fastened += [(f"deck_board_screw{i}", f"deck_standoff{i}"),
                     (f"deck_standoff{i}", f"deck_nut{i}")]
    bd = board["dims"]
    x0, x1, y0, y1, bhw = board["x0"], board["x1"], board["y0"], board["y1"], board["half_w"]
    pcb = _box(x0, x1, y0, y1, -bhw, bhw) - union(
        [_cyl_y(x, z, bd["hole_d"] / 2, y0 - 1, y1 + 1) for x, z in board["holes"]])
    # its DC jack at the front end (0.9 mm past the board, a right-angle plug out of the open
    # front of the bay), the headers and the rest behind it (Waveshare's STEP model)
    jack = _box(x1 - 14.0, x1 + JACK_PROUD, y1, y1 + bd["jack_h"], -4.5, 4.5)
    parts = _box(x0 + 0.5, x1 - 14.0, y1, y1 + bd["parts_h"], -9.0, 9.0)   # clear of the holes
    bodies.append(Body(name="deck_board", part=union([pcb, jack, parts]), rigid_with=host["L"],
                       fab="purchased", bom_key="esp32_servo_driver", color=PCB_COLOR))

    # the battery in its cradle
    bat = lay.battery()
    bodies.append(Body(name="deck_battery",
                       part=_box(bat["x0"], bat["x1"], bat["y0"], bat["y1"], -bat["half_w"],
                                 bat["half_w"]),
                       rigid_with=host["L"], fab="purchased", bom_key="lipo_2s_450",
                       color=BATTERY_COLOR))
    ix0, ix1, ihw = bat["inner"]
    w = CRADLE_WALL
    rim = (_box(ix0 - w, ix1 + w, yt, yt + CRADLE_H, -ihw - w, ihw + w)
           - _box(ix0, ix1, yt - 1, yt + CRADLE_H + 1, -ihw, ihw)
           - _box(ix1 - 1, ix1 + w + 1, yt - 1, yt + CRADLE_H + 1, -CRADLE_GAP / 2,
                  CRADLE_GAP / 2))
    # its two ears, screwed to the deck (no glue: the user's decision of 2026-10-05): an M3
    # button head from above through each ear and the deck, an M3 nut under the deck
    nut_d = get("m3_nut").dims
    nut_af, nut_h = float(nut_d.get("af", RAIL_NUT_AF)), float(nut_d.get("h", RAIL_NUT_H))
    ear_len = pick_length(EAR_H + lay.pitch + nut_h + 0.5, CRADLE_SCREW.lengths)
    ye = yt + EAR_H
    for i, (ex, ez) in enumerate(lay.cradle_ears()):
        side = 1.0 if ez > 0 else -1.0
        za, zb = sorted((side * (ihw + w / 2), ez))
        ear = union([_box(ex - EAR_R, ex + EAR_R, yt, ye, za, zb),
                     _cyl_y(ex, ez, EAR_R, yt, ye)])
        rim = union([rim, ear]) - _cyl_y(ex, ez, CLEARANCE["3"] / 2, yt - 1, ye + 1)
        scr = union([_cyl_y(ex, ez, CRADLE_SCREW.head_d / 2, ye, ye + CRADLE_SCREW.head_h),
                     _cyl_y(ex, ez, CRADLE_SCREW.d / 2 - 0.05, ye - ear_len, ye)])
        nut = (_cyl_y(ex, ez, nut_af / 2, yd - nut_h, yd)
               - _cyl_y(ex, ez, CRADLE_SCREW.d / 2, yd - nut_h - 1, yd + 1))
        bodies += [Body(name=f"deck_cradle_screw{i}", part=scr, rigid_with=host["L"],
                        fab="purchased", bom_key=CRADLE_SCREW.key(ear_len), color=STEEL),
                   Body(name=f"deck_cradle_nut{i}", part=nut, rigid_with=host["L"],
                        fab="purchased", bom_key="m3_nut", color=STEEL)]
        fastened.append((f"deck_cradle_screw{i}", f"deck_cradle_nut{i}"))
    bodies.append(Body(name="deck_cradle", part=rim, rigid_with=host["L"], fab="printed",
                       color=RAIL_COLOR))
    extras.append(BomLine("lipo_strap_10mm", 1, "battery strap through the deck's slots"))

    # under the deck: charger at the front, protection board behind the servos
    ch = get("ip2326_charger").dims
    yu = yd - TAPE_GAP
    nut_r = STANDOFF_AF / 2 + 0.3                      # the board's nuts under the deck
    cx1, cz0, pad = _charger_place(lay, ch, yu, obstacles,
                                   [(x, z, nut_r) for x, z in board["holes"]],
                                   get("m25_nylon_nut").dims["h"])
    cz1, cy1 = cz0 + ch["width"], yu - pad
    charger = _box(cx1 - ch["length"], cx1, cy1 - ch["height"], cy1, cz0, cz1)
    if pad:
        # two printed strips the charger is taped to, between it and the deck, clear of the
        # board's nuts it now passes under
        nuts = union([_cyl_y(x, z, nut_r, yd - 10, yd + 1) for x, z in board["holes"]])
        for i, z0 in enumerate((cz0, cz1 - PAD_STRIP)):
            strip = _box(cx1 - ch["length"], cx1, cy1, yu, z0, z0 + PAD_STRIP) - nuts
            if len(strip.solids()) != 1:
                raise ConstructionError("the charger's pad strip is cut in two by the "
                                        "board's nuts")
            bodies.append(Body(name=f"deck_charger_pad{i}", part=strip, rigid_with=host["L"],
                               fab="printed", color=RAIL_COLOR))
    bms = get("bms_hx_2s_jh20").dims
    bx1 = lay.x_c - WIRE_SLOT[0] / 2 - 0.5
    bodies += [
        Body(name="deck_charger", part=charger, rigid_with=host["L"], fab="purchased",
             bom_key="ip2326_charger", color=PCB_COLOR),
        Body(name="deck_bms", part=_box(bx1 - bms["width"], bx1, yu - bms["height"], yu,
                                        -bms["length"] / 2, bms["length"] / 2),
             rigid_with=host["L"], fab="purchased", bom_key="bms_hx_2s_jh20",
             color=PCB_COLOR),
    ]
    extras.append(BomLine("foam_tape", 1, "charger and protection board under the deck"))

    # the switch: bushing and lever above, body and lugs below
    bx, by, bh = sw["body"]
    switch = union([_box(sx - bx / 2, sx + bx / 2, yd - bh - sw["lugs"], yd, sz - by / 2,
                         sz + by / 2),
                    _cyl_y(sx, sz, sw["bushing_d"] / 2 - MODEL_GAP, yd, yt + sw["bushing_h"]),
                    _cyl_y(sx, sz, 1.5, yt + sw["bushing_h"],
                           yt + sw["bushing_h"] + sw["lever_h"])])
    bodies.append(Body(name="deck_switch", part=switch, rigid_with=host["L"],
                       fab="purchased", bom_key="toggle_mts102", color=STEEL))

    extras += [BomLine("resistor_100k", 1, "battery divider (top) to an ESP32 ADC pin"),
               BomLine("resistor_33k", 1, "battery divider (bottom)"),
               BomLine("xt30_pigtail_pair", 1, "battery lead to the protection board"),
               BomLine("dc_plug_5521_pigtail", 1, "switched battery to the board's DC jack")]
    blocked = _blocked_boxes(bodies, obstacles)
    if blocked:
        a, b = blocked[0]
        raise ConstructionError(f"the deck can't be lowered onto its rails: {a} meets "
                                f"{b} on the way down")
    info = {"fitted": True, "x_c": round(lay.x_c, 2), "deck_y": round(yd, 2),
            "rail_y": round(lay.rail_y0, 2), "plate_mm": [2 * HALF_LEN, round(2 * hw, 2)],
            "screw": key, "spigot_x": place.spigot_x,
            "notches": [[round(v, 2) for v in n] for n in notches],
            "charger_setback_mm": round(lay.x_c + HALF_LEN - cx1, 2),
            "charger_z_mm": [round(cz0, 2), round(cz1, 2)], "charger_pad_mm": round(pad, 2),
            "top_y": round(max(b.part.bounding_box().max.Y for b in bodies), 2)}
    return bodies, extras, info, fastened


def _rect_dist(p, x0, x1, y0, y1) -> float:
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


def _swept_radius(part, n: int = 24) -> float:
    """The farthest a part reaches from the crank axis (x = y = 0): every edge sampled
    (a solid's farthest point from an axis lies on an edge or a cylindrical face's edge)."""
    r = max(math.hypot(p.X, p.Y) for e in part.edges()
            for p in (e.position_at(u) for u in [k / n for k in range(n + 1)]))
    return r / math.cos(math.pi / n)          # a sampled arc's chord, covered


def _sweep_up(part, prism: bool, top: float):
    """``part`` swept straight up to ``top``: a part that is a prism along y (the deck
    plate: every cut goes through) stretched exactly, any other its bounding box."""
    bb = part.bounding_box()
    if prism:
        h = bb.max.Y - bb.min.Y
        k = (top - bb.min.Y) / h
        return scale(part.moved(Pos(0, -bb.min.Y, 0)), by=(1.0, k, 1.0)).moved(
            Pos(0, bb.min.Y, 0))
    return _box(bb.min.X, bb.max.X, bb.min.Y, top, bb.min.Z, bb.max.Z)


def _box6(bb) -> Box6:
    return (bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z)


def _blocked_boxes(bodies, obstacles: Sequence[Box6]) -> list[tuple[str, str]]:
    """Lowered parts (but the plate, which :func:`path_notches` cut) whose box, swept up,
    meets an obstacle's box (the build-time check; :func:`deck_path` is the exact one)."""
    out = []
    for b in bodies:
        if b.part is None or not lowered(b.name) or b.name == "deck_plate":
            continue
        x0, x1, y0, _, z0, z1 = _box6(b.part.bounding_box())
        for i, (ox0, ox1, _, oy1, oz0, oz1) in enumerate(obstacles):
            if oy1 > y0 + 1e-3 and _overlap(x0, x1, ox0, ox1) and _overlap(z0, z1, oz0, oz1):
                out.append((b.name, f"obstacle {i}"))
    return out


def deck_path(mech, clash_mm3: float = 0.01) -> list[tuple[str, str, float]]:
    """``(deck part, other part, mm^3)`` for every part that would stop the deck going
    straight down onto its rails: each lowered part (:func:`lowered`: the plate and what
    is fitted to it) swept up from where it sits, against every other part of the robot
    (the deck plate's sweep is exact, the rest their boxes). Empty: it goes in."""
    deck = [b for b in mech.bodies if b.part is not None and lowered(b.name)]
    if not deck:
        return []
    top = max(b.part.bounding_box().max.Y for b in mech.bodies if b.part is not None) + 1.0
    others = [(b.name, b.part, b.part.bounding_box()) for b in mech.bodies
              if b.part is not None and not lowered(b.name)
              and re.fullmatch(r"deck_screw\d+", b.name) is None]
    out = []
    for b in deck:
        bb = b.part.bounding_box()
        sweep = None
        for name, part, ob in others:
            if not (ob.max.Y > bb.min.Y + 1e-3 and _overlap(bb.min.X, bb.max.X, ob.min.X, ob.max.X)
                    and _overlap(bb.min.Z, bb.max.Z, ob.min.Z, ob.max.Z)):
                continue
            if sweep is None:
                sweep = _sweep_up(b.part, b.name == "deck_plate", top)
            inter = sweep & part
            vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
            if vol > clash_mm3:
                out.append((b.name, name, round(vol, 3)))
    return out


def deck_clearance(mech) -> dict:
    """How far the deck's parts are from every moving part over the whole crank cycle.

    The kinematics is planar, so a moving part's ``z`` band never changes: a moving part
    whose band misses every deck part's band can never touch it (``z_gap_mm``, the least
    such gap). A part that turns with the crank (the horn, which pokes 1.3 mm past the
    inner plate's face into the servo bay) sweeps a disc about the crank axis O: it
    clears a deck part whose band it shares when that disc misses the part's XY box
    (``sweep_gap_mm``). Any other moving part sharing a band is a failure
    (``overlapping``). ``blocked``: what stops the deck going in (:func:`deck_path`, the
    assembly's way down onto the rails). ``ok`` is all of it. The rails' screws are left
    out of the moving-part check: they come up through the inner plate from the leg side,
    and their heads are the drive group's claims (:meth:`servos.mount.DriveGroup.claims`),
    which the planner keeps clear of every moving part in XY over the cycle wherever they
    sit, in the gap under the plate or sunk into the layer there (the plan's ``heads``)."""
    by_name = {b.name: b for b in mech.bodies}
    deck = [b for b in mech.bodies if b.part is not None and "deck" in b.name
            and "deck_rail_screw" not in b.name]
    if not deck:
        return {"fitted": False, "ok": True}

    def root(b):
        while b.rigid_with is not None:
            b = by_name[b.rigid_with]
        return b

    boxes = {b.name: b.part.bounding_box() for b in deck}
    z_gap, sweep_gap, overlapping = math.inf, math.inf, []
    nearest = None
    for b in mech.bodies:
        if b.part is None or body_class(root(b).name) == "torso":
            continue                                   # static: the OCCT clash check's job
        bb = b.part.bounding_box()
        crank = body_class(root(b).name).startswith(("conn", "coupler"))
        for name, db in boxes.items():
            gap = max(db.min.Z - bb.max.Z, bb.min.Z - db.max.Z)
            if gap > 0:
                if gap < z_gap:
                    z_gap, nearest = gap, (b.name, name)
                continue
            if crank:
                r = _swept_radius(b.part)
                d = _rect_dist((0.0, 0.0), db.min.X, db.max.X, db.min.Y, db.max.Y) - r
                sweep_gap = min(sweep_gap, d)
                if d <= 0:
                    overlapping.append((b.name, name))
            else:
                overlapping.append((b.name, name))
    blocked = deck_path(mech)
    return {"fitted": True, "z_gap_mm": round(z_gap, 3), "nearest": nearest,
            "sweep_gap_mm": round(sweep_gap, 3), "overlapping": overlapping,
            "blocked": blocked, "ok": not overlapping and not blocked}

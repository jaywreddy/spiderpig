"""The build: what to make and how, as one validated, hashable record.

:class:`BuildConfig` is the key every cache uses (side designs, baked
``.glb`` files, the MuJoCo model) and the argument every stage takes. It
validates itself on construction: the linkage exists (and walks, when a
robot is asked for), the module is one of the linkage's, the leg phases
(radians, one per leg) fit the module and are ``None`` when they are the
module's own, the proportions name the linkage's parameters, lengths are
positive, and only the overrides that differ from the defaults are kept,
in the linkage's order. So a design has exactly one config however it was
asked for, and a bad one fails where it is named (:class:`ParamError`, a
``ValueError``).

The CLIs share ``--linkage`` / ``--phases`` (degrees) / ``--proportion
NAME=VALUE`` and the build options (:func:`add_design_args`,
:func:`add_build_args`, :func:`config_from_args`); the server takes
``linkage=``, ``module=``, ``phases=`` and ``p.NAME=``
(:func:`design_from_query`).
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, replace

from spiderpig import linkage, servos
from spiderpig.fit import Params
from spiderpig.hardware.catalog import sheet_thickness


class ParamError(ValueError):
    """Bad design parameters (unknown linkage, module or proportion, wrong phase count, ...)."""


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


DEFAULT_CRANKS = {"walker": "bolt", "mechanism": "bolt"}
"""The crank a design gets when it names none, per kind (:func:`default_crank`): the bolt
crank for both (the crank study of 2026-10-03; mechanisms since 2026-10-04), which is the
single-plate web crank with hex-standoff crankpins in hex pockets (the user's decision of
2026-10-04 (1), ``BoltCrank.pin="hex"``; jam SF >= 2.25 on 0.100 in 6061-T6). The round
friction-clamped standoff it replaced is ``bolt_round``. Which construction a design gets is
data here, not code."""

LINKAGE_CRANKS: dict[str, str] = {
    # hoecken_pantograph and dwell_rocker plan with the hex crank since it merged
    # (2026-10-04; 9 layers each): the hub plate caps a crankpin under the horn spacer
    # (BoltCrank.hub_capped).
    #
    # TrotBot's heel and toe: b7 passes crankpin J1 at 10.2 mm; the hex crankpin's 8.5 mm
    # sleeve needs 11.2 (the static stage stops). The round 6 mm standoff clears it and plans
    # (its friction clamp: jam SF 3.57, UNVERIFIED); the hex crank at unit 12 rescales it.
    "trotbot_heel": "bolt_round",
    "trotbot_toe": "bolt_round",
}
"""Per linkage: the crank it gets when it names none, where :data:`DEFAULT_CRANKS`'
doesn't plan (each with why)."""


MODULE_CRANKS: dict[tuple[str, str], str] = {
    # empty since BoltCrank.hex_gap_fit (2026-10-05) put the Strider decker and quad on the
    # hex crank (git log has the debug)
}
"""Per (linkage, module): the crank it gets when it names none, ahead of
:data:`LINKAGE_CRANKS` (each with why)."""


CRANK_SHEET = "al6061_2p5mm"
"""The crank's plates when nothing names a sheet: 0.100 in 6061-T6, the thinnest whose hex
pockets hold the walkers' jam twist (2 x 0.85 N·m) at SF 2 (``materials.thinnest_sheet
("crank")``, 2026-10-04)."""

LINKAGE_CRANK_SHEETS: dict[str, str] = {
    # hoecken_pantograph (r4, 2026-10-05): its 12 mm crank puts crankpin M's hex pocket in
    # the hub plate 2.16 mm from the horn's screw holes (r 7 mm), under 1 x the 2.54 mm
    # sheet (a cut-rule error). On 0.080 in 6061 (2.03 mm) that web is over 1 x t (a
    # warning) and the hex holds the drive's torque at SF 3.89 (factor 1, a mechanism). The
    # round standoff plans but screws over the hub plate (no assembly order). The 0.100 in
    # sheet stays selectable (--crank-sheet).
    "hoecken_pantograph": "al6061_2mm",
}
"""Per linkage: the crank sheet it gets when the config names none, where :data:`CRANK_SHEET`
breaks a rule (each with why)."""


REMOVED = "2026-10-07"
_PIVOTS_GONE = {            # a removed pin or pillar key -> what it was
    "printed": "the printed snap-together axle",
    "rod": "the 3 mm rod with push-on clips",
    "bolt": "the M3 socket-head screw and nylock",
    "bearing": "the MF63ZZ flanged bearings on a 3 mm rod",
    "bushing": "the igus GFM-0304 bushings on a 3 mm rod",
    "ptfe": "the PTFE-lined 3 mm rod",
    "chicago_bushing": "the Chicago screw in igus GFM-0405 bushings",
    "standoff_hand": "the standoff column spliced in the stack by hand",
    "standoff_bench": "the standoff column spliced on the bench",
    "standoff_m3": "the spliced column of uxcell M3 standoffs",
}
REMOVED_CONSTRUCTIONS: dict[str, dict[str, tuple[str, str, str]]] = {
    "crank": {
        "printed": ("bolt", REMOVED, "the printed crankshaft held by clamp friction alone"),
        "keyed": ("bolt", REMOVED, "the printed crankshaft keyed by brass hex standoffs"),
        "keyed_float": ("bolt", REMOVED, "the keyed printed crankshaft with sliding keys"),
        "bolt_hub_screw": ("bolt", REMOVED, "the hex crank screwed over its hub plate"),
        "bolt_unretained": ("bolt", REMOVED, "the hex crank without its thrust sleeves"),
    },
    "pin": {k: ("chicago", REMOVED, w) for k, w in _PIVOTS_GONE.items() if "standoff" not in k},
    "pillar": {k: ("standoff", REMOVED, w) for k, w in _PIVOTS_GONE.items()
               if k not in ("ptfe", "chicago_bushing")},          # (those two: pins only)
}
"""Construction keys removed from :mod:`construction` (D1, W2), per :class:`BuildConfig`
field: ``key -> (replacement, date removed, what it was)``; a config, spec, stored design or
CLI option naming one fails with its replacement (:func:`removed_construction`)."""


REMOVED_PARAMS: dict[str, tuple[str, float, str]] = {
    "glue_fit": (REMOVED, 0.15, "a part glued into a laser-cut hole"),
    "neck_d": (REMOVED, 4.0, "the printed axle's neck where a link passes"),
    "head_d": (REMOVED, 8.5, "the printed axle's head outside the plates"),
    "journal_d": (REMOVED, 12.0, "the printed crank's body on O"),
    "stub_d": (REMOVED, 8.0, "the printed crank's journal stub"),
    "hub_thickness": (REMOVED, 5.0, "the printed crank's hub under the horn"),
}
"""Fields of :class:`construction.base.Params` (a spec's ``fit``) removed because nothing read
them once the printed axle and crank were gone (W5): ``name -> (date removed, its last
default, what it sized)``. A spec naming one is invalid with that message
(:func:`removed_param`); a stored design's resolved record, which writes every field in,
loads when it holds the old default and fails with the message otherwise."""


def removed_param(name: str, value=None) -> str | None:
    """Why ``fit.<name>`` can't be given any more, or ``None`` (not removed; or, with
    ``value``, the removed field's last default, which built the same parts)."""
    gone = REMOVED_PARAMS.get(name)
    if gone is None or (value is not None and value == gone[1]):
        return None
    when, default, what = gone
    return (f"fit.{name} ({what}) was removed on {when}: no construction reads it since the "
            f"printed axle and crank went; drop it (it built as {default:g}; "
            "config.REMOVED_PARAMS)")


def removed_construction(field: str, key) -> tuple[str, str] | None:
    """(why ``field`` can't be ``key`` any more, its replacement), or ``None``."""
    gone = REMOVED_CONSTRUCTIONS.get(field, {}).get(key) if isinstance(key, str) else None
    if gone is None:
        return None
    replacement, when, what = gone
    return (f"{field} {key!r} ({what}) was removed on {when}; use {field}={replacement!r} "
            "(config.REMOVED_CONSTRUCTIONS)"), replacement


def default_crank_sheet(lk: linkage.Linkage) -> str:
    """The crank sheet ``lk`` gets when the config names none (a decanting cradle's: its
    keyed arm's two 0.125 in 6061-T6 plates; :data:`LINKAGE_CRANK_SHEETS`, else
    :data:`CRANK_SHEET`)."""
    if lk.product == CRADLE:
        return "al6061_3p2mm"
    return LINKAGE_CRANK_SHEETS.get(lk.key, CRANK_SHEET)


LINKAGE_TORQUE_LIMITS: dict[str, float] = {
    # klann_lego (2026-10-05): its 6061 leg b4 bends at D under the foot's 80 mm lever; at
    # the servo's 0.85 N·m limit the jam rates it SF 1.58 (strength.link_rows), and no
    # thicker 6061 fits its layer. The quad walks at 0.13 N·m (p99; 0.14 peak, 146 mm/s in
    # the sim), so 0.60 N·m keeps 4.2 x of headroom over walking (the Strider double's 0.85
    # over its 0.18 is 4.7 x) and puts every joint and link at jam SF >= 2 with no part
    # added or changed.
    "klann_lego": 0.60,
}
"""Per linkage: a servo torque limit (N·m) under the servo's own
(:attr:`servos.spec.ServoSpec.torque_limit_nm`), where a joint or link holds a jam at SF 2
only below it (each with why): :func:`torque_limit_nm`."""


def torque_limit_nm(config: BuildConfig) -> float | None:
    """The torque limit to set in the robot's servo firmware (N·m), the one the jam loads
    and the joints' ratings assume: the servo's (``TORQUE_LIMIT_FRACTION`` of stall, at
    most ``JAM_TORQUE_NM``), lowered to the linkage's :data:`LINKAGE_TORQUE_LIMITS` entry;
    ``None`` for a servo with no stall torque."""
    if config.lk.product and config.linkage in PRODUCT_JAM_NM:   # no servo: the hand
        return PRODUCT_JAM_NM[config.linkage]                   # through the gearbox
    limit = servos.get(config.servo).torque_limit_nm
    own = LINKAGE_TORQUE_LIMITS.get(config.linkage)
    if limit is None or own is None:
        return limit
    return min(limit, own)


def torque_limit_note(config: BuildConfig) -> str | None:
    """What the builder must set for :func:`torque_limit_nm` (a BOM / ORDER.md note), or
    ``None`` when the servo has no limit."""
    limit = torque_limit_nm(config)
    spec = servos.get(config.servo)
    stall = spec.stall_torque_nm
    if limit is None or not stall or config.lk.product:
        return None
    why = (" (lowered for this linkage: config.LINKAGE_TORQUE_LIMITS)"
           if config.linkage in LINKAGE_TORQUE_LIMITS else "")
    return (f"Servo firmware: before the first run, set the torque limit of every servo "
            f"({spec.name.split(' (')[0]}) to {limit:g} N·m, {limit / stall * 100:.0f} % of "
            f"its {stall:.2f} N·m stall{why}: every joint and link is rated (spiderpig audit, "
            "strength) at a jammed foot holding the servo at that limit.")


def default_crank(lk: linkage.Linkage, module: str = "") -> str:
    """The crank construction ``lk`` (in ``module``) gets when the config names none: a
    product's (:data:`PRODUCT_CRANKS`: a decanting cradle's hand crank), its module's
    (:data:`MODULE_CRANKS`), else its own (:data:`LINKAGE_CRANKS`), else its kind's
    (:data:`DEFAULT_CRANKS`)."""
    if lk.product in PRODUCT_CRANKS:
        return PRODUCT_CRANKS[lk.product]
    return MODULE_CRANKS.get((lk.key, module),
                             LINKAGE_CRANKS.get(lk.key, DEFAULT_CRANKS[lk.kind]))


CRADLE = "cradle"
SHEET = "acrylic_3mm"
PRODUCT_CRANKS: dict[str, str] = {CRADLE: "hand"}
"""Per product (:attr:`linkage.Linkage.product`): the crank it is built with, the only one
(a decanting cradle: the hand-cranked worm drive, :mod:`construction.handcrank`)."""

FRAME_SHEET = "al5052_2mm"
PRODUCT_FRAME_SHEETS: dict[str, str] = {CRADLE: "al5052_4p8mm"}
"""Per product: the frame plates' sheet when the config names the default
(:data:`FRAME_SHEET`). A decanting cradle's frame plates are its stand (2026-10-10): 0.187 in
5052, 3.4 x as stiff across as 0.125 in (the side's sway under the hand and a bumped bottle)
and still cut with the M5 screws' 5.5 mm holes (SendCutSend: a hole at least the
thickness; 0.250 in would need 6.35)."""

PRODUCT_PARAMS: dict[str, dict[str, float]] = {
    CRADLE: {"link_radius": 12.0, "frame_radius": 12.0, "spacer_d": 12.5},
}
"""Per product: the shared dimensions (:class:`fit.Params`) a design gets when it names
the defaults. A decanting cradle's links are 22 mm wide (two plate thicknesses of web round
its pins' 8.2 mm holes, the 8 mm pivot pin's or the M6 Chicago barrel's: SendCutSend's rule)
and its frame arms 24 mm; its pins' spacer rings 12.5 mm (the 8 mm pin and a wall)."""

PRODUCT_PINS: dict[str, str] = {CRADLE: "pivot8"}
"""Per product: the link pin a design gets when it names the default (``chicago``): a
decanting cradle's 8 mm hardened steel pivot pins (:data:`construction.pivots.pivot_pin.PIVOT8`,
since 2026-10-13: a 6 L bottle's jam bent the M6 Chicago screw's stainless barrel;
``chicago6`` is still there, ``--pin chicago6``), but at :data:`PRODUCT_PIN_JOINTS`."""

PRODUCT_PIN_JOINTS: dict[str, dict[str, str]] = {
    "decanter_rcm": dict.fromkeys(("K", "J2", "X", "Y"), "clevis8"),
}
"""Per product linkage, with its product's own pin (:data:`PRODUCT_PINS`): the joints that take
another pin instead. B's X and Y, where its cradle link meets each side, take a
quick-release clevis pin (``clevis8``, 2026-10-13: the passive side panel slides off its
cradle link once their two clips are out, ``cradle_pack``); so do its K and J2 (design
review round 1, 2026-10-14: the 18-8 pin holds their jam at SF 1.4 or more, where J2's M6
Chicago screw had held 1.40 on an unstated alloy, with threadlocker and pliers). Its rod's
pin P keeps the hardened steel (as a clevis pin it would hold 1.1), as A's one pin E does,
and so does J: a clevis pin's clip and end stand in the cradle link's layer, and b6's rear
arm X-Tr1 passes right over J (the planner finds no plan). Two pin types, no Chicago
screw, nothing threadlocked."""

PRODUCT_CHORD_R: dict[str, dict[str, dict[tuple[str, str], float]]] = {
    "decanter_rcm": {"b6": {("X", "Tr1"): 20.0, ("X", "Y"): 16.0}},
    "decanter_hinge": {"b2": {("E", "Tr1"): 16.0}},
}
"""Per product linkage, per link: a chord (a pair of its joints, either way round) cut wider
than ``Params.link_radius``, its own half-width (mm), each from the cradle's links rated
in bending (design review round 1, 2026-10-14; ``cradle_report.structure``):
- B's cradle link b6: its rear arm X-Tr1, a cantilever off X carrying the rear saddle, 40
  mm wide (24 held the jam's couple at SF 0.3 in 0.125 in); its chord X-Y, 284 mm in
  compression under the jam, 32 mm wide against buckling out of its plane.
- A's cradle link b2: its chord E-Tr1, in compression from the rod's pin to the rear tie
  under the jam, 32 mm wide against buckling.
The planner claims each at its width (:class:`construction.plates.LinkPlates`)."""


def chord_r(config, link: str, a: str, b: str) -> float:
    """The half-width (mm) ``config`` cuts link ``link``'s chord ``a``-``b`` (joint or
    topology point names: a lone point is ``link.joint``): :data:`PRODUCT_CHORD_R`'s, else
    ``Params.link_radius``."""
    from spiderpig.stack import body_class

    own = PRODUCT_CHORD_R.get(config.linkage, {}).get(body_class(link), {})
    if own:
        a, b = a.rsplit(".", 1)[-1], b.rsplit(".", 1)[-1]
        r = own.get((a, b), own.get((b, a)))
        if r is not None:
            return r
    return config.params.link_radius

PRODUCT_SHEETS: dict[str, str] = {CRADLE: "al5052_3p2mm"}
"""Per product: the default sheet (the layer pitch, the printed rings' and the order's
material) when the config names the default (:data:`SHEET`): a decanting cradle is all
aluminium, its links' 0.125 in 5052 (:data:`materials.CRADLE_LINK_SHEET`)."""

PRODUCT_JAM_NM: dict[str, float] = {"decanter_hinge": 61.3, "decanter_rcm": 61.3}
"""Per product linkage: the crank torque its joints are rated at in a jam, in place of a
servo's firmware limit (:func:`torque_limit_nm`): a decanting cradle's, the most its hand
crank's slip clutch lets through, however hard the push (:data:`PRODUCT_CLUTCH_NM`, since
design review round 1; a firm 40 N push at the grip had been the case:
:func:`spiderpig.cradle_report.jam_nm`, which a test holds these to)."""

PRODUCT_CLUTCH_NM: dict[str, float] = {"decanter_hinge": 61.3, "decanter_rcm": 61.3}
"""Per product linkage: the most its hand crank's slip clutch lets onto the crank, however
hard the push (design review round 1, 2026-10-14): set to slip at
``construction.handcrank.CLUTCH_SLIP_NM`` at the input, at the top of its
``CLUTCH_SCATTER``, through the NRV040's 30:1 worm at a hand's speed (``handcrank.clutch``,
which a test holds these to). The crank torque to re-run a strength check at: a bound, unlike
the push (``cradle_report``'s ``jam_capped`` rates it over the whole turn, the toggles
included)."""


def product_defaults(product: str) -> dict:
    """The fields a product sets where the config names the default (the sheets, the pin,
    the shared dimensions): what :class:`BuildConfig` fills in, and what a design moved to
    another linkage drops (:func:`own_constructions`)."""
    out: dict = {}
    if product in PRODUCT_SHEETS:
        out["sheet"] = PRODUCT_SHEETS[product]
    if product in PRODUCT_FRAME_SHEETS:
        out["frame_sheet"] = PRODUCT_FRAME_SHEETS[product]
    if product in PRODUCT_PINS:
        out["pin"] = PRODUCT_PINS[product]
    return out


def merged_params(params: Params, product: str) -> Params:
    """``params`` with a product's dimensions (:data:`PRODUCT_PARAMS`) in every field it
    leaves at the default: one fit override keeps the rest of the product's."""
    mine = PRODUCT_PARAMS.get(product, {})
    base = Params()
    return replace(params, **{k: v for k, v in mine.items()
                              if getattr(params, k) == getattr(base, k)})


def is_cradle(config) -> bool:
    """Whether ``config`` builds a decanting cradle (:mod:`construction.cradle`)."""
    return config.lk.product == CRADLE


@dataclass(frozen=True)
class BuildConfig:
    """What to build and how. Construction keys refer to :mod:`construction` registries."""

    linkage: str = linkage.DEFAULT    # see linkage.available()
    module: str = ""                  # legs per side: one of the linkage's modules ("": its
    #                                   default, config.default_module: Strider's double)
    robot: bool = True                # two mirrored sides, servos back to back in one frame
    phases: tuple[float, ...] | None = None          # crank phase per leg (rad); None = module's
    proportions: tuple[tuple[str, float], ...] = ()  # overrides of the linkage's params
    sheet: str = SHEET                # the default sheet (the links, rings, deck): it sets
    #                                   the layer pitch (spiderpig.materials)
    thickness: float | None = None    # override the sheet's nominal thickness
    frame_sheet: str = FRAME_SHEET      # the frame and centre plates (aluminium: acrylic
    #                                     can't take their load; the user's call 2026-10-04):
    #                                     0.080 in 5052, the thinnest stock that passes
    #                                     (materials.thinnest_sheet("frame"); tests pin it)
    crank_sheet: str = ""               # the crank's laser-cut plates ("": default_crank_sheet:
    #                                     the linkage's, LINKAGE_CRANK_SHEETS, else CRANK_SHEET,
    #                                     0.100 in 6061-T6, the thinnest whose hex pockets hold
    #                                     the jam twist at SF 2: materials.thinnest_sheet
    #                                     ("crank"), 2026-10-04)
    heads: str = "best"               # fasteners' heads: "sink" into the layer beside their
    #                                   link, "gap" (a thin clearance gap where a link passes),
    #                                   "best" (sunk, else in gaps; stack.StackSpec)
    link_sheets: tuple[tuple[str, str], ...] | None = None   # link class -> sheet; None: the
    #                                   linkage's (materials.default_link_sheets: a Klann
    #                                   variant's foot links in 6061)
    servo: str = servos.DEFAULT
    pillar: str = "standoff"          # frame pivots: a 6 mm round standoff column, a stock
    #                                   goBILDA one or else one steel standoff made to length,
    #                                   never spliced (construction.pivots.standoff)
    pin: str = "chicago"              # pivots between links: an M3 Chicago screw (4 mm barrel),
    #                                   printed rings and head spacers (construction.pivots.
    #                                   chicago)
    crank: str = ""                   # the crankshaft ("": default_crank: the linkage's own,
    #                                   LINKAGE_CRANKS, else its kind's, DEFAULT_CRANKS: "bolt",
    #                                   construction.crank.BoltCrank, single aluminium web plates
    #                                   on the crank sheet, hex-standoff crankpins in hex
    #                                   pockets; "bolt_round": the round friction-clamped
    #                                   standoff). The keys removed on 2026-10-07:
    #                                   REMOVED_CONSTRUCTIONS
    params: Params = field(default_factory=Params)

    def __post_init__(self) -> None:
        try:
            lk = linkage.get(self.linkage)
        except KeyError as e:
            raise ParamError(e.args[0]) from None
        if self.robot and lk.kind != "walker":
            # a cradle is one side too (robot=False): construction.cradle builds both from it
            raise ParamError(f"{lk.key} is a mechanism, not a walker: it has no feet to walk on, "
                             f"so it builds one side (robot=False; --side-only on the command "
                             f"line) (walkers: {', '.join(linkage.available('walker'))})")
        if not self.module:
            object.__setattr__(self, "module", default_module(lk.key))
        if not self.crank:
            object.__setattr__(self, "crank", default_crank(lk, self.module))
        if not self.crank_sheet:
            object.__setattr__(self, "crank_sheet", default_crank_sheet(lk))
        if lk.product:          # a product's defaults where the config names the default's
            plain = {"sheet": SHEET, "frame_sheet": FRAME_SHEET, "pin": "chicago"}
            for what, value in product_defaults(lk.product).items():
                if getattr(self, what) == plain[what]:
                    object.__setattr__(self, what, value)
            object.__setattr__(self, "params", merged_params(self.params, lk.product))
        hand = PRODUCT_CRANKS.get(lk.product)
        if (self.crank == "hand") != (hand == "hand"):
            cradles = [k for k in linkage.available() if linkage.get(k).product]
            raise ParamError(f"crank {self.crank!r}: the hand crank builds a decanting cradle "
                             f"and a cradle only that ({lk.key} is "
                             f"{'a cradle' if hand else 'not'}; cradles: {', '.join(cradles)})")
        for what in ("crank", "pin", "pillar"):
            if (gone := removed_construction(what, getattr(self, what))) is not None:
                raise ParamError(gone[0])
        if self.module not in lk.leg_modules:
            raise ParamError(f"unknown module {self.module!r}; have {list(lk.leg_modules)}")
        if self.servo not in servos.available():
            raise ParamError(f"unknown servo {self.servo!r}; have {servos.available()}")
        object.__setattr__(self, "phases", self._phases(lk))
        object.__setattr__(self, "proportions", self._proportions(lk))
        object.__setattr__(self, "link_sheets", self._link_sheets(lk))
        if self.heads not in ("best", "sink", "gap"):
            raise ParamError(f"heads must be best, sink or gap, got {self.heads!r}")
        from spiderpig.hardware.catalog import get

        for what in ("sheet", "frame_sheet", "crank_sheet"):
            key = getattr(self, what)
            try:
                if get(key).category != "sheet":
                    raise KeyError(key)
            except KeyError:
                raise ParamError(f"{what} {key!r} is not a sheet in the catalog") from None
        from spiderpig.materials import sheet

        if not sheet(self.crank_sheet).metal:
            raise ParamError(f"crank_sheet {self.crank_sheet!r} is not metal (the acrylic crank "
                             f"was removed on {REMOVED}): an aluminium sheet, e.g. {CRANK_SHEET!r}")

    def _link_sheets(self, lk: linkage.Linkage) -> tuple[tuple[str, str], ...] | None:
        """Link class -> sheet, validated; ``None`` when it is the linkage's own."""
        if self.link_sheets is None:
            return None
        from spiderpig.hardware.catalog import get
        from spiderpig.materials import default_link_sheets

        out = {}
        for link, key in dict(self.link_sheets).items():
            if link not in lk.links:
                raise ParamError(f"link_sheets: {lk.key} has no link {link!r} "
                                 f"(have {list(lk.links)})")
            try:
                ok = get(key).category == "sheet"
            except KeyError:
                ok = False
            if not ok:
                raise ParamError(f"link_sheets: {key!r} is not a sheet in the catalog")
            out[link] = key
        if out == default_link_sheets(lk):
            return None
        return tuple(sorted(out.items()))

    def _phases(self, lk: linkage.Linkage) -> tuple[float, ...] | None:
        """Radians per leg; ``None`` for the module's own (mod 2 pi), however given."""
        if self.phases is None:
            return None
        legs = lk.leg_modules[self.module]
        try:
            values = tuple(float(p) for p in self.phases)
        except (TypeError, ValueError):
            raise ParamError(f"phases must be numbers, got {list(self.phases)!r}") from None
        if len(values) != len(legs):
            raise ParamError(f"{self.module} has {len(legs)} legs per side, got {len(values)} "
                             "phases")
        if not all(math.isfinite(p) for p in values):
            raise ParamError(f"phases must be finite numbers, got {list(values)}")
        if all(abs(_wrap(a - ph)) < 1e-9 for a, (_, ph) in zip(values, legs, strict=True)):
            return None
        return values

    def _proportions(self, lk: linkage.Linkage) -> tuple[tuple[str, float], ...]:
        """Overrides that differ from the defaults, validated, in the linkage's order."""
        overrides = dict(self.proportions)
        unknown = sorted(set(overrides) - set(lk.params))
        if unknown:
            raise ParamError(f"unknown {lk.key} proportions {unknown}; have {list(lk.params)}")
        out = []
        for name, default in lk.params.items():
            if name not in overrides:
                continue
            try:
                v = float(overrides[name])
            except (TypeError, ValueError):
                raise ParamError(f"proportion {name} must be a number, "
                                 f"got {overrides[name]!r}") from None
            if not math.isfinite(v):
                raise ParamError(f"proportion {name} must be a finite number, "
                                 f"got {overrides[name]!r}")
            if name not in lk.signed and v <= 0:       # angles and coordinates may be <= 0
                raise ParamError(f"proportion {name} is a length and must be > 0, got {v:g}")
            if abs(v - float(default)) > 1e-12:
                out.append((name, v))
        return tuple(out)

    # -- what the stages read ---------------------------------------------------

    @property
    def lk(self) -> linkage.Linkage:
        return linkage.get(self.linkage)

    @property
    def legs(self) -> linkage.LegList:
        """``(orientation, crank phase in rad)`` per leg of one side."""
        legs = self.lk.leg_modules[self.module]
        if self.phases is None:
            return legs
        return tuple((o, ph) for (o, _), ph in zip(legs, self.phases, strict=True))

    @property
    def pitch(self) -> float:
        """The layer pitch: the sheet's thickness."""
        return sheet_thickness(self.sheet, self.thickness)

    @property
    def values(self) -> tuple[float, ...]:
        """Every parameter of the linkage, overrides applied."""
        return self.lk.values(dict(self.proportions))

    @property
    def is_default(self) -> bool:
        """The linkage's default design (the module's phases, its proportions), built the
        default way."""
        return self == BuildConfig(linkage=self.linkage, module=self.module, robot=self.robot)

    @property
    def key(self) -> str:
        """A file-name stem: ``strider_double_robot``, plus a hash for anything but a default."""
        stem = f"{self.linkage}_{self.module}_{'robot' if self.robot else 'side'}"
        if self.is_default:
            return stem
        return f"{stem}_{hashlib.sha1(repr(self).encode()).hexdigest()[:12]}"

    def design_json(self) -> dict:
        """The design parameters as JSON: linkage, module, phases (deg) and all proportions."""
        return {
            "linkage": self.linkage,
            "module": self.module,
            "phases_deg": [round(math.degrees(ph), 6) for _, ph in self.legs],
            "proportions": dict(zip(self.lk.params, self.values, strict=True)),
        }


# ---------------------------------------------------------------------------
# From the command line and the query string
# ---------------------------------------------------------------------------


def parse_phases(text: str) -> tuple[float, ...]:
    """``"0,180,90,270"`` -> radians (degrees in; the count is checked by the config)."""
    try:
        values = [float(s) for s in str(text).split(",")]
    except ValueError:
        raise ParamError(f"phases must be comma-separated numbers (degrees), got {text!r}") \
            from None
    if not all(math.isfinite(v) for v in values):
        raise ParamError(f"phases must be finite numbers, got {text!r}")
    return tuple(math.radians(v) for v in values)


def parse_proportion(item: str) -> tuple[str, float]:
    """``"DF=2.6"`` -> ``("DF", 2.6)`` (the config checks the name)."""
    name, sep, value = str(item).partition("=")
    name = name.strip()
    if not sep or not name:
        raise ParamError(f"expected NAME=VALUE, got {item!r}")
    try:
        return name, float(value)
    except ValueError:
        raise ParamError(f"proportion {name} must be a number, got {value!r}") from None


DEFAULT_MODULE = "quad"           # a walker's, when neither it nor the linkage says


def default_module(key: str) -> str:
    """The module a design gets when none is asked for: the linkage's own
    (``Linkage.default_module``: Strider's ``double``), else a walker's ``quad`` (its
    first module if it has no quad), a mechanism's one module (:class:`ParamError` for
    an unknown linkage)."""
    try:
        lk = linkage.get(key)
    except KeyError as e:
        raise ParamError(e.args[0]) from None
    if lk.default_module:
        return lk.default_module
    if DEFAULT_MODULE in lk.leg_modules:
        return DEFAULT_MODULE
    return next(iter(lk.leg_modules))


def default_robot(key: str) -> bool:
    """Whether a design builds the two-sided robot when nothing says: a walker does, a
    mechanism is one side (it has no feet to walk on)."""
    try:
        return linkage.get(key).kind == "walker"
    except KeyError as e:
        raise ParamError(e.args[0]) from None


def add_design_args(p) -> None:
    """``--linkage``, ``--module``, ``--phases`` and ``--proportion`` on an argparse parser.
    ``--module`` defaults to ``None``: :func:`config_from_args` fills in the linkage's
    (:func:`default_module`), so a mechanism needs no ``--module single``."""
    import argparse

    def arg(fn):
        def convert(text):
            try:
                return fn(text)
            except ParamError as e:
                raise argparse.ArgumentTypeError(str(e)) from None
        convert.__name__ = fn.__name__
        return convert

    p.add_argument("--linkage", choices=linkage.available(), default=linkage.DEFAULT,
                   help=f"the linkage (default {linkage.DEFAULT})")
    p.add_argument("--module", default=None,
                   help="legs per side (the linkage's modules): single, double (mirrored "
                   "pair), decker (two legs on one crankshaft), quad (two mirrored deckers). "
                   f"Default: the linkage's ({linkage.DEFAULT}'s "
                   f"{default_module(linkage.DEFAULT)}; a walker's {DEFAULT_MODULE} otherwise, "
                   "a mechanism's single)")
    p.add_argument("--phases", type=arg(parse_phases), default=None, metavar="DEG,...",
                   help="crank phase of every leg of a side, in degrees (default: the "
                   "module's, e.g. quad 0,180,90,270)")
    names = "; ".join(f"{k}: {', '.join(linkage.get(k).params)}" for k in linkage.available())
    p.add_argument("--proportion", type=arg(parse_proportion), action="append", default=None,
                   metavar="NAME=VALUE",
                   help=f"override one of the linkage's parameters (repeatable; lengths in mm "
                   f"or the linkage's unit, angles in degrees): {names}")


def add_build_args(p) -> None:
    """The build options: ``--servo``, the constructions, the sheet."""
    import argparse

    from spiderpig import construction  # (the registries: not at import, CAD-heavy)

    d = BuildConfig()
    axles, cranks = sorted(construction.AXLES), sorted(construction.CRANKS)

    def kept(field: str):       # a removed key's error names its replacement (argparse: exit 2)
        def check(text: str) -> str:
            if gone := removed_construction(field, text):
                raise argparse.ArgumentTypeError(gone[0])
            return text
        return check
    p.add_argument("--servo", default=d.servo, choices=servos.available(),
                   help=f"servo model (default {d.servo})")
    p.add_argument("--pillar", default=d.pillar, choices=axles, type=kept("pillar"),
                   help=f"construction of the frame pivots (default {d.pillar})")
    p.add_argument("--pin", default=d.pin, choices=axles, type=kept("pin"),
                   help=f"construction of the pivots between links (default {d.pin})")
    p.add_argument("--crank", default=None, choices=cranks, type=kept("crank"),
                   help="crank construction (default: "
                        + ", ".join(f"{v} for a {k}" for k, v in DEFAULT_CRANKS.items())
                        + "; " + ", ".join(f"{v} for {k}" for k, v in LINKAGE_CRANKS.items())
                        + ")")
    p.add_argument("--sheet", default=d.sheet, help=f"sheet stock catalog item (default {d.sheet})")
    p.add_argument("--thickness", type=float, default=None,
                   help="measured sheet thickness in mm (default: the sheet's nominal)")
    p.add_argument("--frame-sheet", dest="frame_sheet", default=d.frame_sheet,
                   help=f"sheet of the frame and centre plates (default {d.frame_sheet})")
    p.add_argument("--crank-sheet", dest="crank_sheet", default=None,
                   help=f"sheet of the crank's plates (default {CRANK_SHEET}; "
                        + ", ".join(f"{v} for {k}" for k, v in LINKAGE_CRANK_SHEETS.items())
                        + ")")
    p.add_argument("--heads", default=d.heads, choices=("best", "sink", "gap"),
                   help="fasteners' heads: sunk into a layer, in thin clearance gaps, or "
                        "best: sunk, and in gaps only when no sunk plan exists (the plan's "
                        f"proof says which was searched; default {d.heads})")
    p.add_argument("--link-sheet", dest="link_sheet", action="append", default=None,
                   metavar="LINK=SHEET",
                   help="cut a link class from another sheet, e.g. b4=al6061_3p2mm "
                        "(repeatable; default: the linkage's, a Klann variant's foot links "
                        "in 6061 aluminium)")


def config_from_args(args, **fixed) -> BuildConfig:
    """The config the parsed arguments ask for (:class:`ParamError` if they don't fit).

    ``fixed`` overrides fields (``robot=False``); build options the parser lacks keep
    their defaults. A module not given (``None``) is the linkage's
    (:func:`default_module`); ``robot`` not given, or given as ``None``, is the
    linkage's kind's (:func:`default_robot`: a mechanism builds one side).
    """
    d = BuildConfig()
    fields = {k: getattr(args, k, getattr(d, k)) for k in ("linkage", "sheet", "thickness",
                                                            "servo", "pillar", "pin",
                                                            "frame_sheet", "crank_sheet",
                                                            "heads")}
    if getattr(args, "link_sheet", None):
        pairs = []
        for item in args.link_sheet:
            link, sep, key = str(item).partition("=")
            if not sep:
                raise ParamError(f"--link-sheet wants LINK=SHEET, got {item!r}")
            pairs.append((link.strip(), key.strip()))
        fields["link_sheets"] = tuple(pairs)
    fields["crank"] = getattr(args, "crank", None) or ""        # the kind's default
    fields["crank_sheet"] = getattr(args, "crank_sheet", None) or ""   # the linkage's
    fields.update(phases=getattr(args, "phases", None),
                  proportions=tuple(getattr(args, "proportion", None) or ()))
    fields.update(fixed)
    fields["module"] = (fixed.get("module") or getattr(args, "module", None)
                        or default_module(fields["linkage"]))
    if fields.get("robot") is None:
        fields["robot"] = default_robot(fields["linkage"])
    return BuildConfig(**fields)


def query_proportions(query: Mapping) -> list[tuple[str, float]]:
    """The ``p.<NAME>=<value>`` items of a query string, in order (a multi-dict's every
    item)."""
    multi = getattr(query, "multi_items", None)       # (starlette's QueryParams)
    items = multi() if multi is not None else query.items()
    return [parse_proportion(f"{k[2:]}={v}") for k, v in items if k.startswith("p.")]


def own_constructions(base: BuildConfig) -> dict:
    """``crank`` and ``crank_sheet`` as ``base`` *chose* them: each one its linkage and
    module would get anyway (:func:`default_crank`, :func:`default_crank_sheet`, filled in
    by the config) is ``""``, so a design moved to another linkage or module gets that
    one's own default instead of carrying the old one's (TrotBot heel's ``bolt_round``,
    the Hoecken pantograph's 0.080 in crank sheet); a product's (a decanting cradle's)
    sheets, pin and dimensions, where they are its defaults, the plain ones."""
    lk = linkage.get(base.linkage)
    out: dict = {"crank": "" if base.crank == default_crank(lk, base.module) else base.crank,
                 "crank_sheet": ("" if base.crank_sheet == default_crank_sheet(lk)
                                 else base.crank_sheet)}
    if lk.product:      # a product's own sheets, pin and dimensions: back to the plain ones
        plain = {"sheet": SHEET, "frame_sheet": FRAME_SHEET, "pin": "chicago"}
        for what, value in product_defaults(lk.product).items():
            if getattr(base, what) == value:
                out[what] = plain[what]
        mine = PRODUCT_PARAMS.get(lk.product, {})
        out["params"] = replace(base.params, **{
            k: getattr(Params(), k) for k, v in mine.items() if getattr(base.params, k) == v})
    return out


def design_from_query(query: Mapping, base: BuildConfig | None = None,
                      **fixed) -> BuildConfig:
    """The config a query string asks for: ``linkage``, ``module`` (default: the linkage's,
    :func:`default_module`), ``phases`` (degrees, comma-separated) and ``p.<NAME>=<value>``;
    other keys are ignored. ``fixed`` (``robot``, a mode's ``module``) wins.

    With ``base`` (a stored design's config), the query applies on top of it: its servo,
    sheets, thickness, heads, constructions and fit, which no query string expresses,
    stay; ``module``, ``phases`` and ``p.NAME`` change. Another ``linkage`` starts from
    that linkage's defaults (its own parameters, phases and link sheets) and keeps the
    materials and constructions; a crank or crank sheet that was only the old linkage's
    (or module's) default is not kept (:func:`own_constructions`)."""
    proportions = query_proportions(query)
    if base is None:
        key = query.get("linkage") or linkage.DEFAULT
        kw = {"linkage": key,
              "module": query.get("module") or default_module(key),
              "phases": parse_phases(query["phases"]) if query.get("phases") else None,
              "proportions": tuple(proportions)}
        return BuildConfig(**{**kw, **fixed})
    own = own_constructions(base)
    if (query.get("linkage") or base.linkage) != base.linkage:
        # (link_sheets name a linkage's own links: kept only with its linkage)
        materials = {"sheet": base.sheet, "thickness": base.thickness, "servo": base.servo,
                     "frame_sheet": base.frame_sheet, "pillar": base.pillar, "pin": base.pin,
                     "heads": base.heads, "params": base.params, **own}
        return design_from_query(query, **{**materials, **fixed})
    props = dict(base.proportions)
    props.update(proportions)
    module = fixed.get("module") or query.get("module") or base.module
    if query.get("phases"):
        phases = parse_phases(query["phases"])
    else:
        phases = base.phases if module == base.module else None     # the module's own
    rest = {k: v for k, v in fixed.items() if k != "module"}
    rest.setdefault("crank", base.crank if module == base.module else own["crank"])
    return replace(base, module=module, phases=phases,
                   proportions=tuple(sorted(props.items())), **rest)

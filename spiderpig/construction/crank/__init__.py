"""The crank: the bolt crank's single aluminium web plates on stock standoff crankpins.

The crank turns about O, driven by the servo horn, and carries one crankpin
per leg (point M), which the leg's riders (Klann's b1) turn on. Every rider
sweeps over O, and the crank turns fully relative to it, so the crank can
cross a rider's layer only along that rider's own crankpin: it is a
**built-up crankshaft**. Its shape is a :class:`CrankRoute`, which the
planner chooses (else :func:`default_route`): **runs**, where the shaft
leaves O along a post (at a crankpin, or at a detour point fixed to the
crank) over some layers, each between two **webs** (arms from O out to the
post) in the layers either side; the hub under the servo horn; and, with the
bottom bearing, a journal stub turning in the outer frame plate (without it
the crank hangs from the servo side). That shape is :meth:`CrankGroup.claims`;
the construction, :class:`BoltCrank`, decides radii and how the pieces are made
and joined (its docstring; the planner's rules for it are in
:mod:`construction.route`).

``bolt`` (the default) builds every web as one laser-cut aluminium plate and every
crankpin and journal as a stock steel hex standoff keyed in hex pockets of its two webs;
``bolt_round`` (TrotBot's heel and toe, :data:`config.LINKAGE_CRANKS`) as a round goBILDA
standoff clamped between them by friction. The printed cranks, the acrylic two-plate
M6-bolt crank and the hex crank's variants were removed on 2026-10-07
(:data:`config.REMOVED_CONSTRUCTIONS`).

The package (a pure move of the former ``construction/crank.py``, W5): :mod:`.base` the
routes, claims and :class:`CrankGroup`; :mod:`.bolt` :class:`BoltCrank` (its fits in
:mod:`.hex`, :mod:`.web` and :mod:`.capacity`, mixed in); :mod:`.plates` its parts
(:class:`_WebPlates`). Every name keeps its old import path here, and a write to one
reaches the submodule that reads it (:mod:`spiderpig.reexport`).
"""

from spiderpig.construction.crank.base import (
    BOLT_COLOR,
    EPS,
    GROUP,
    PRESS_DRAWN,
    SEGMENT_COLOR,
    STEEL,
    CrankDims,
    CrankGroup,
    CrankRoute,
    Run,
    _hex,
    chains_of,
    default_route,
    hex_play,
    hub_layers,
    route_of,
)
from spiderpig.construction.crank.bolt import BOLT_ROUND, BoltCrank
from spiderpig.construction.crank.capacity import CapacityMixin, hex_bearing_nm
from spiderpig.construction.crank.hex import HexFitMixin, HexJoint, _fit_hex, _hex_gap_fit
from spiderpig.construction.crank.plates import _WebPlates
from spiderpig.construction.crank.web import (
    HORN_TIP_CLEAR,
    SHIM_KEY,
    WebFitMixin,
    WebJoint,
    _pin_lengths,
    shim_stack,
)
from spiderpig.reexport import forward_writes

__all__ = [
    "BOLT_COLOR", "BOLT_ROUND", "EPS", "GROUP", "HORN_TIP_CLEAR", "PRESS_DRAWN",
    "SEGMENT_COLOR", "SHIM_KEY", "STEEL", "BoltCrank", "CapacityMixin", "CrankDims",
    "CrankGroup", "CrankRoute", "HexFitMixin", "HexJoint", "Run", "WebFitMixin", "WebJoint",
    "_WebPlates", "_fit_hex", "_hex", "_hex_gap_fit", "_pin_lengths", "chains_of",
    "default_route", "hex_bearing_nm", "hex_play", "hub_layers", "route_of", "shim_stack",
]

forward_writes(__name__)

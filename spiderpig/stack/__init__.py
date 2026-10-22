"""Layer planning: which layer every plate sits in, so that nothing ever collides.

The Klann mechanism is planar; Z only matters for fabrication, and this module
decides it. One side of the walker is a stack of equal-height **layers**
(``pitch`` = sheet thickness):

* layer ``0`` holds the **outer frame plate**, layer ``top`` the **inner frame
  plate** (the one the servo bolts to); nothing else may sit in them except
  parts seated in their holes;
* every leg link (b1..b4) sits in one layer in between;
* layers below 0 and above ``top`` are outside the stack (pillar heads, the
  servo).

Everything that isn't a link plate (axles and their built-in spacers, pin
heads, the crank, the servo horn) is described to the planner by the
construction groups as **claims** (:class:`Claim`): the shapes a group will
occupy in each layer, stated relative to the layers of the links it depends
on. A shape is a disc around a point or a pill (capsule) between two points,
and points move over the crank cycle. The planner does not know how anything
is built. It guarantees that no two shapes of different groups in one layer
ever come closer than ``margin`` over the **whole** cycle: distances are
lower bounds that also cover the motion between samples.

One group, the crank, has a shape to choose: which layers its shaft runs
along a crankpin (or a detour point), where its webs go, whether it keeps
its bottom bearing. It is a :class:`Router`: for any layering it finds the
cheapest route whose pieces clear everything else in their layers.

:meth:`StackProblem.solve` finds the lowest stack (:meth:`StackProblem.key`: its
height in mm, then the crank route's cost, the stock parts' cost and the parts printed;
a walker's objective "layers": the fewest layers, then the route; an arm's
"layers_height": the fewest layers, then the lowest in them, then the axles' spans), as
jobs of one stack
size and one head placement each (:mod:`.drive`), here or in forked workers
(:mod:`stack_pool`), with the same answer. A job searches link layers (the assembly
tree from the crank outwards, fewest open layers first) with forward checking: every
placed shape removes the layers it rules out for unplaced links. Every node asks the
router for a route through what is placed so far; a dead end, a collision or a claim
that can't be built is explained by the links whose layers caused it, the search jumps
back to the latest of them and remembers the combination (a nogood); a node whose least
height is over the best plan's is cut (branch and bound). The search's z is a bound of
every plan its layering can become (:class:`Layout`): a claim refuses a layout only on
a bound valid for every completion. :attr:`StackPlan.optimal` says whether every job
the objective needs ran to the end, :attr:`StackPlan.proof` how far they went.
:func:`verify_plan` re-checks a plan exhaustively on a fresh, denser sampling.


The package (a pure move of the former ``stack.py``, W5): :mod:`.geometry` (distances,
shapes, claims, :class:`Layout`), :mod:`.topology` (:class:`Topology`, static clearances,
:class:`PlanError`, the :class:`Router` protocol), :mod:`.plan` (:class:`StackSpec`,
:class:`StackPlan`), :mod:`.search` (:class:`StackProblem`), :mod:`.plan_z` (the plan's z,
:func:`finalize`; not ``finalize.py``, which the function's re-export would shadow) and
:mod:`.verify` (:func:`verify_plan`), and :mod:`.drive` (the jobs, 2026-10-14). Every name
keeps its old import path here, for
reading; a patch goes on the submodule that reads the name (``stack.plan.MAX_SECONDS``,
``stack.plan_z._make_all``).
"""

from spiderpig.mechanism import body_class, is_crank, is_frame, is_link
from spiderpig.stack.geometry import (
    _BETWEEN,
    Claim,
    Core,
    Disc,
    Geometry,
    Layout,
    Pill,
    Placed,
    Shape,
    Unbuildable,
    _cross,
    _point_seg,
    log,
    made,
    seg_seg,
)
from spiderpig.stack.plan import (
    MAX_SECONDS,
    Deadline,
    StackPlan,
    StackSpec,
    thread_time,
)
from spiderpig.stack.plan_z import (
    EPS_Z,
    GAP_GROUPS,
    GAP_MAX,
    GAP_MORE,
    GAP_STEP,
    GAP_TRIES,
    GIVE_UP,
    HEADS_ORDER,
    MEMO_MAKES,
    PlanReject,
    SunkKey,
    _gap_options,
    _gap_sizes,
    _hit,
    _make_all,
    _ReadGaps,
    _sinkable,
    _thicker_gaps,
    _thicknesses,
    bound_layout,
    bridges,
    finalize,
    gap_most,
    heads_claims,
    plate_bridged,
    settle,
    sunk_key,
)
from spiderpig.stack.search import (
    HEIGHT_TOL,
    StackProblem,
    _Budget,
    _Done,
    _Search,
)
from spiderpig.stack.topology import (
    _SAMPLED,
    SAMPLED_MAX,
    Axis,
    AxisKind,
    Clearance,
    ClearanceError,
    Keepout,
    PlanError,
    Recommendation,
    Route,
    RouteConflict,
    Router,
    RouteView,
    Topology,
    _axis_name,
    _copy_of,
    _sample_topology,
    group_axes,
    static_clearances,
    topology_from_template,
)
from spiderpig.stack.verify import (
    verify_plan,
)

__all__ = [
    "EPS_Z",
    "GAP_GROUPS",
    "GAP_MAX",
    "GAP_MORE",
    "GAP_STEP",
    "GAP_TRIES",
    "GIVE_UP",
    "HEADS_ORDER",
    "HEIGHT_TOL",
    "MAX_SECONDS",
    "MEMO_MAKES",
    "SAMPLED_MAX",
    "_BETWEEN",
    "_SAMPLED",
    "Axis",
    "AxisKind",
    "Claim",
    "Clearance",
    "ClearanceError",
    "Core",
    "Deadline",
    "Disc",
    "Geometry",
    "Keepout",
    "Layout",
    "Pill",
    "Placed",
    "PlanError",
    "PlanReject",
    "Recommendation",
    "Route",
    "RouteConflict",
    "RouteView",
    "Router",
    "Shape",
    "StackPlan",
    "StackProblem",
    "StackSpec",
    "SunkKey",
    "Topology",
    "Unbuildable",
    "_Budget",
    "_Done",
    "_ReadGaps",
    "_Search",
    "_axis_name",
    "_copy_of",
    "_cross",
    "_gap_options",
    "_gap_sizes",
    "_hit",
    "_make_all",
    "_point_seg",
    "_sample_topology",
    "_sinkable",
    "_thicker_gaps",
    "_thicknesses",
    "body_class",
    "bound_layout",
    "bridges",
    "finalize",
    "gap_most",
    "group_axes",
    "heads_claims",
    "is_crank",
    "is_frame",
    "is_link",
    "log",
    "made",
    "plate_bridged",
    "seg_seg",
    "settle",
    "static_clearances",
    "sunk_key",
    "thread_time",
    "topology_from_template",
    "verify_plan",
]

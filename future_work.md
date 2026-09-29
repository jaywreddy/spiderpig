# future_work.md

Open follow-ups after the rationalization rework ([stack.py](stack.py),
[construction/](construction/), [fabricate.py](fabricate.py)). Resolved by
it: tolerances are parameters (`construction.base.Params`, kerf in
`layout.py`), the frame is two plates with pillars anchored in both, the
crankpin diameter is a parameter, the servo is modelled and planned around.

## More constructions

Only printed constructions exist so far (stepped axles, printed crank). The
registries in `construction/__init__.py` are ready for others; each needs
`dims()` (validation and the radii its claims use) and `realize()` inside
those claims:

- rod axles: steel dowel / cut rod, links running on it, retained by
  push-on washers, laser spacer rings in the gaps (a spacer-free variant
  packs each pillar's links against a frame plate);
- inserts in the turning plates: igus sleeve bushings, MR63ZZ bearings;
- a laser-cut crank (glued plate stack, rod crankpins).

A claim can express these: shoulders become ring or washer discs, necks the
bare rod. The catalog already lists the parts (research notes in the
commit history).

## Stiffer necks

Where a link passes close to a pillar (every b1 passes about 9.7 mm from
pivot B), the printed pillar necks down to `Params.neck_d` (4 mm). A steel
core (a 3 mm rod through a printed sleeve) would keep the neck stiff.

## Rigid link frames

Parts are still modelled in world coordinates at the build angle `t`, so the
glTF bake recovers each body's motion by a planar rigid fit of its joints.
Giving each link a local frame (joint *i* at `(dᵢ, 0)`, lengths from
`klann.PROPORTIONS`) and a closed-form pose `(x, y, θ)(t)` would drop the fit
and make every leg's link literally the same part.

## Search

`StackProblem.solve()` is a depth-first search, most-constrained link first,
with a node budget per stack size (a plan it finds is always valid; only
minimality depends on the budget). Quad per side plans in about a second.
Much larger stacks, or a secondary objective (shortest axles, fewest
shoulders), may want CP-SAT over precomputed pairwise tables.

## Walking

Both servos turn at the same speed today (the render). Steering is the two
sides at different speeds; a gait controller for the STS3215 bus (speed
mode) is outside this repo.

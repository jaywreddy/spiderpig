# future_work.md

Open follow-ups after the plan-driven fabrication rework ([stack.py](stack.py),
[fabricate.py](fabricate.py)). The old joinery migration items are resolved:
pins go on after composition, holes are drilled only at real pivots, the
frame holds every deck's pivots directly, and meshes are keyed per body.

## Rigid link frames

Parts are still modelled in world coordinates at the build angle `t`, so the
glTF bake recovers each body's motion by a planar rigid fit of its joints.
Giving each link a local frame (joint *i* at `(dᵢ, 0)`, lengths from
`klann.PROPORTIONS`) and a closed-form pose `(x, y, θ)(t)` would drop the fit,
make every leg's link literally the same part, and let the kinematic tree in
`mechanism.py` carry real rotations instead of identity poses.

## Tolerances as parameters

`shapes.HOLE_R` (2.0) / `PIN_R` (1.9) and the press-fit bores (exactly
`PIN_R`) are nominal. Real printers and lasers need calibration: move them
into `StackSpec`, add kerf compensation for laser-cut holes, and make the
drive key match the actual servo horn.

## Stiffer frame for tall stacks

In the decker and quad the A/B frame pins span 27–33 mm, from a head under
the lower deck to a cap above the plate, and between the two decks' links
they run bare wherever a sleeve would hit a b1 (b1 passes 9.9 mm from B). A
second frame plate at the bottom of the stack would support each pin at both
ends; the planner would need a second plate slot and posts rising from it.

## Crankpin strength

In multi-deck modes torque passes from web to web through the crankpins
(3.8 mm printed pins in shear). Consider metal pins (M4 rod) or two pins per
throw; the pin diameter should be a `StackSpec` parameter.

## Servo body

The frame plate carries a servo pad, screw holes and a clearance hole for
the hub, and the crank's top segment ends in a 4 × 4 mm key above the
plate. The servo body itself isn't modelled; nothing else sits above the
plate today, but a model would let the audit check it.

## Search scaling

`StackProblem.solve()` is a depth-first search (b1s first, then most
conflicted links) with incremental checks. It solves the quad (16 links) in
~0.2 s and three legs on one crankshaft in ~0.4 s; much larger `multi`
stacks may need a smarter search (e.g. CP-SAT).

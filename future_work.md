# future_work.md

Deferred follow-ups from the joinery refactor (see [joinery.py](joinery.py)).

## Migrate `_jp` / `_jpt` Z-stagger onto `LayerStaggerPin`

**Why deferred.** The original plan's step 2 was to refactor `_jp` /
`_jpt` in [klann.py](klann.py) to emit z=0 joints and apply
`LayerStaggerPin(spacing=thickness)` per connection. Tracing the
composition primitives (`combine_connectors`, `fuse_torsos`,
`_add_standoffs` in [klann.py](klann.py)) revealed a sharp interaction:
when `fuse_torsos` drops a connection (e.g., the `(torso_leg1, A) →
(b3_leg1, A)` edge in `decker` mode), it would orphan the spacer body
that joinery had inserted between them. The orphaned spacer becomes a
new BFS root with default `base_pose = identity`, which places its
downstream subtree (the upper-deck leg) at the wrong world position. The
subsequent `_add_standoffs` then introduces a second parent for
`b3_leg1.A`, and BFS resolves only one of the two constraints.

**Fix sketch.** Apply joinery *after* all composition primitives have
run, not inside `build_klann_mechanism`. The pipeline becomes:

1. `build_klann_mechanism` builds per-leg mechanisms with the current
   `_jp` pos/neg layered approach (no joinery yet).
2. `_merge_mechanisms` + `combine_connectors` + `fuse_couplers` +
   `fuse_torsos` + `_add_standoffs` operate on the layered graph
   exactly as today.
3. *New step*: walk the final `mech.connections`, classify each by
   joinery type (lookup table or per-connection metadata), apply the
   joinery. At this stage the original layered Z stagger is already
   present, so joinery uses `spacing=0` and only contributes geometry.

Or — more invasive but cleaner — replace the per-body layered Z
math with explicit per-connection spacings stored on `_CONN_TEMPLATE`,
and have joinery own all of it. That requires per-connection spacings
and signed direction (parent above vs below child); see the trace in
the conversation that produced this commit for the eight per-connection
spacings under each orientation.

## Migrate `rationalize_segment` clevis holes onto `ClevisHole.host_negative`

**Why deferred.** Today [shapes.py:114](shapes.py:114)
`rationalize_segment` blindly cuts a 4 mm hole at every joint on every
link via `clevis_neg`. The plan's step 2b was to remove that and have
`ClevisHole` cut the hole only at connections that actually have a
joinery applied. This is blocked on the `_jp` migration above — both
need to know which connections are joinery'd, and that signal lives in
the same place.

When the migration above lands: drop the per-joint `clevis_neg` loop in
`rationalize_segment`, apply `ClevisHole` (instead of `LayerStaggerPin`)
to every layered connection. Net rendered output should be identical
(same holes, same spacings, just declared explicitly).

## Convert `_add_standoffs` to use the `Standoff` joinery factory

**Why deferred.** Same root cause: `_add_standoffs` runs after
`fuse_torsos` and adds new connections to ground orphaned upper-deck
legs. Replacing it with the `Standoff` joinery requires the
"joinery-after-composition" flow above to be in place first.

When migrated, the call sites in `build_double_decker_klann` and
`build_double_double_decker_klann` shrink to one `Standoff().apply(mech,
parent=..., child=("ground", "anchor"))` per orphaned pivot.

## (Optional) Auto-sized joinery from host introspection

The plan mentioned the `Joinery.sized_for(host_thickness=...)` factory
pattern for joinery that adapts to its host's geometry. The
infrastructure is in place (joinery is constructed before `apply`, so
callers can read host parts and pass dimensions in), but no built-in
factory uses it yet. Add `ClevisPin.sized_for(host_link)` once a real
need shows up.

## Performance: dedup joinery meshes across instances

`viewer/bake_gltf.py` already collapses joinery body names to a class
via `_class_of` (e.g., `clevis_pin0_pin` → `clevis_pin_pin`), so all
`ClevisPin` instances share one mesh in the glTF. But the bake's
tessellation step builds *one* part per class from the first body it
sees in `ref_mech.bodies`. If a future user creates two `ClevisPin`s
with different `shaft_radius` values, both will tessellate against the
first one's mesh (wrong geometry). Fix: include geometry-affecting
parameters in the class key, or compare part bounding boxes for
mismatch warnings.

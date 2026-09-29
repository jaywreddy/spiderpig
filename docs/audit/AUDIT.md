# Spiderpig audit — 2026-09-29

Scope: performance, elegance of the symbolic core, and whether the
fabrication build (STEP / STL / DXF + joinery) produces parts that actually
go together. Audited at `6ef75ea` (master == `claude/great-carson-6nw2bz`).

## Status after the rework (same day)

Every finding below is fixed, or has moved to [future_work.md](../../future_work.md).
`mise run audit` now passes in every mode (single, double, decker, quad): no
part intersects any other at five crank angles, every part is one valid
solid, the stack plan re-checks clean over 1440 crank samples, and every
link reaches the DXF.

| finding | fix |
|---|---|
| F1 b1/b2 share a layer | `stack.py` assigns slots explicitly, against full-cycle clearance tables |
| F2 frame holds nothing | the frame is one solid: plate in the top slot, posts, servo pad; pins through every pivot |
| F3 coupler vs frame/links | no separate coupler disc. The crank's top segment is the hub: journal, then a stub through the plate and a key for the servo |
| F4 mirrored leg at z < 0 | z lives only in the plan, never in joint offsets |
| F5 standoffs through the lower deck | standoffs removed. Every deck's pivots are frame axes |
| F6 ClevisPin misfit / fused cap | pins are sized from the plan: head below the lowest link, a separate press-on cap above the highest, sleeves where they fit |
| F7 DXF drops a foot link | links are laid flat along their long axis (diagonal if needed); a part that doesn't fit is an error |
| F8 pins missing from STEP | `main.py` fabricates with hardware |
| F9 e2e regressions | no empty spacer nodes; `__viewer.mode` reports the GLB on screen. 9/9 e2e pass |
| decker / quad on one shaft | infeasible as the 2016 design had it: every b1 sweeps over O. They now use a **built-up crankshaft**, with webs either side of each b1 and torque carried through the crankpins |
| sympy as float calculator | `klann.py` is the straight-line program (≤126 ops per step, exact rational proportions), compiled once; phase is a time shift |
| twin builders | single-t mechanisms are `template.freeze_at(t)`; `klann.py` is 508 lines (was 1,171) |
| performance | unit suite 40 s for 77 tests (was 3 m 25 s for 75); bake single 1.7 s (was 3.1 s), quad 4.7 s (was 9.5 s) with 71 bodies |
| lint | `ruff check .` clean (was 17) |

The rest of this document is the original audit, kept as the record of
what was found.

Reproduce the original findings by checking out `6ef75ea` and running:

```bash
mise run audit                                   # fabrication checks (exit 1 there)
uv run python viewer/bake_gltf.py --mode quad    # bake profile
```

## TL;DR

* **Green tests, broken hardware.** 75/75 unit tests pass, and `main.py` writes
  STEP/STL/DXF for every mode, but no mode produces an assembly that can be
  built. The tests only check that connected joints coincide in space. They
  never check that the solids fit.
* **The kinematics are numerically right.** Link lengths are constant to
  1e-13 over the cycle, with no NaNs, for both chiralities. The problems are
  in layering, the frame, and the joinery.
* **Sympy is used as an expensive float calculator.** Expressions swell to
  about 34k ops, design parameters are hard-coded floats, and the same
  solve runs once per leg per call. That redundancy is about 38% of a quad
  bake, and memoizing it takes the unit suite from 3 m 25 s to 30 s.
* **The architecture duplicates itself.** Every builder has a single-t twin
  and a template twin, and bodies have no rigid local frames, so motion has
  to be recovered after the fact by Procrustes fitting.
* A ~200-line sketch (since folded into `klann.py`) showed the target
  design: a symbolic straight-line program with symbolic design parameters,
  compiled once. It matched the old `klann.py` to about 3e-13.

## 1. Repo state

| item | state |
|---|---|
| branches | `master` = `6ef75ea`. Five other `claude/*` branches, **all fully merged** (0 commits ahead) and safe to delete. |
| PRs / issues | none open, none closed |
| unit tests | **75 passed** in 3 m 25 s (`test_template_matches_scalar` alone takes 131 s) |
| e2e (Playwright) | 8 tests, deselected by default and not run anywhere. **5 pass, 3 fail** (see F9). Run here with the sandbox's preinstalled Chromium mapped to Playwright 1.58's expected path. |
| lint | `ruff check .` reports **17 findings** (6 `zip()` without `strict=`, 3 unsorted import blocks, 3 long lines, E741 `O`, SIM105, an unused import, 2 pytest-style) |
| docs drift | README says "20 smoke tests", lists `test_poses.py` (deleted), and claims the STEP output is colour-tagged. It isn't: build123d warns `Unknown Compound type, color not set`. |
| dead code | `shapes.joint_from_point`, `shapes.connector`, `bake_gltf._rigid_planar`, `_quat_xyzw_from_z_rot`, `KlannSolution.segments`. `LayerStaggerPin`, `ClevisHole`, `Standoff` and `MechanismTemplate.freeze_at` are used only by tests. |

## 2. Fabrication: what is broken

Evidence comes from `mise run audit`: OCCT intersection volumes at t ∈ {0, 1, 2.5, 4} plus a 720-sample full-cycle sweep.

| # | finding | where | measured |
|---|---|---|---|
| F1 | **b1 and b2 share a layer and grind through each other.** No code assigns layers explicitly. They fall out of which `_jp` pos/neg joint copy each body holds, and both links land on z 6–9. | `klann.build_klann_mechanism` | collision on **13.6% of every crank turn, up to 2.15 mm**, for every leg in every mode |
| F2 | **The frame holds nothing.** `create_support_bar` points its pin *down*, so it ends exactly at the link's bottom face (0 mm engagement at A, B, O). The torso is a `Compound` of 4–6 **disconnected solids**. The servo mount floats about 18 mm below the bars, with no plate joining them. | `shapes.create_torso`, `create_support_bar` | torso: 4 solids (single/decker), 6 (double/quad) |
| F3 | **The coupler collides with the frame and the links.** The r = 10 disc sits on the torso's O bar. The 4×4×20 key rises through b1's layer, and b1 passes within 0.43 mm of O. The conn key slot is exactly 4×4, so the fit has zero clearance. | `shapes.create_shaft_connector` | 150.8 mm³ (every mode, every t); 47.9 mm³ vs b1 |
| F4 | **The mirrored leg sits at z −3..3**, inside the coupler disc and the torso bars. | `fuse_torsos(..., a_dz=-6)` | coupler × b1_leg1 up to **674 mm³** |
| F5 | **Standoffs are solid 15 mm columns bored straight through the lower deck's b3/b2 pivots.** Kinematically they are leaves (connected only to the upper link), so the upper deck's A/B pivots are not attached to the frame at all. | `_add_standoffs`, `shapes.standoff` | 113.1 mm³ per standoff |
| F6 | **The joinery doesn't fit.** `ClevisPin` goes on every link↔link edge, but only in the bake. See the next table. | `joinery.ClevisPin`, `bake_gltf._apply_clevis_pins` | 16–20 / 38–44 / 26–30 / 57–67 clashing pairs (single / double / decker / quad) |
| F7 | **The DXF silently drops a foot link.** Parts are packed in their world orientation at t = 1 with `rotation=False`, and anything over 200 mm is skipped without a warning. | `layout.save_sheets` | `b4_leg1` missing (double, decker); `b4_leg2` missing (quad) |
| F8 | **Pins never reach fabrication output.** `main.py` never applies joinery, so STEP/STL/DXF have no pins, and the bake shows hardware the build doesn't make. | `main.py` | — |
| F9 | **Joinery broke two e2e tests unnoticed.** Spacer bodies become mesh-less, static, animated glTF nodes (8 of 23 nodes in single mode), which fails `test_mesh_nodes_actually_move`. Separately, `window.__viewer.mode` flips *before* the new GLB loads, so `test_mode_toggle_swaps_assembly` reads the old clip. | `joinery._apply_to_*`, `viewer/src/main.ts:44` | 3/8 e2e failing |

The joinery details behind F6:

| defect | consequence |
|---|---|
| `pin_anchor_z = base_height + shaft_height/2` (8) centres the mate on the shaft, not on the cap gap (7.2) | the cap overlaps the upper link by 0.6 mm (22.6 mm³ at every pin) |
| `part = bottom + (cap - bottom)` is just `bottom ∪ cap` | one solid spool with two flanges, which cannot be inserted through a hole |
| pins go on torso edges too | each coincides with the torso's own integral pin (62.4 mm³) |
| O appears in two edges (torso↔conn, conn↔coupler) | two identical pins at O (474.7 mm³ overlap), both inside the drive key (125 mm³) |

### A layer plan that works (single leg)

The swept-area overlap graph has edges b1–b2 (14% of the cycle), b1–b3,
b1–b4, b2–b4 and conn–b1. The b1–b2–b4 triangle means **a leg needs 3 link
layers**; the code uses 2. One plan that passes every pin pass-through
check (≥ 12 mm clearance through intermediate layers over the full cycle):

```
L0: b1          (the drive shaft must stop below this: b1 passes 0.43 mm from O)
L1: conn, b2, b3
L2: b4
frame + drive:  beyond L2 (pins at A, B, O cross L2 with ≥ 12 mm clearance)
```

## 3. Performance

Bake profile, `--frames 120`, 4-core container:

| mode | bake total | `create_klann_geometry` calls | sympy (solve + lambdify) | OCCT parts + tessellation |
|---|---|---|---|---|
| single | 3.1 s | 3 | 1.1 s (36%) | 1.7 s |
| double | 5.2 s | 5 | 1.8 s (35%) | 3.2 s |
| quad | 9.5 s | 9 | 3.6 s (38%) | 5.5 s |

(The profiler's `1_reference_build` stage, 1.7 / 3.3 / 6.5 s, also contains
the sympy work for the reference legs. The OCCT column subtracts that and
adds tessellation.)

* **The same symbolic solve runs repeatedly.** Each call costs about 0.28 s
  to solve plus 0.12 s to lambdify, and it runs once per leg for the
  reference build, again for the template, and once more for the foot path.
  **Phase is only a time shift** (`M(t + φ)`), so one solve per chirality
  would do. Memoizing on `orientation` and evaluating at `t + phase` removes
  about 3 s from a quad bake. Even a plain `functools.cache` on
  `create_klann_geometry` takes the unit suite from **3 m 25 s to 30 s**
  (verified, 75/75 still pass), because `test_template_matches_scalar`
  re-solves sympy for each of its 16 t values.
* **Expression swell.** `Point` auto-rationalizes floats
  (`549313338788749/10000000000000`), and every step substitutes the full
  previous tree. The ops counts are C 211, D 844, E 10,250, **F 34,156**.
  Only `lambdify(cse=True)` keeps evaluation fast (45 ms per 10⁵ samples).
* **Fabrication build:** quad takes 9.4 s = 2.3 s `import build123d` + 1.1 s
  sympy + ~3 s OCCT + < 1 s export. Per-leg torsos and couplers are built,
  then discarded by `fuse_*`. Holes are cut one boolean at a time.
* **The numeric fast path is not the bottleneck.** The batched BFS takes
  11–54 ms and Procrustes 7–13 ms.

## 4. Elegance and architecture

1. **Two parallel worlds.** `Mechanism` (single-t) and `MechanismTemplate`
   (batched) each have their own builders, composition primitives
   (`combine_connectors[_template]`, `fuse_*[_template]`,
   `_add_standoffs[_template]`) and `Joinery.apply` / `apply_template`.
   About 400 of `klann.py`'s 1,171 lines are template twins, and a 131-second
   test exists only to keep the twins in lockstep.
2. **Bodies have no rigid local frames.** Joint poses are *world* positions at
   time t. After `solved()`, every body in every mode has **rotation = I**, so
   BFS only propagates Z offsets. The bake then fits the motion back with
   Procrustes against a t = 0 reference. That needs `_witness` joints for
   pins, regex class dedup (`_class_of`) and identity fallbacks, and parts are
   built in world coordinates at an arbitrary t, which causes F7.
3. **Sympy is used as a float calculator.** Constants like `66.28 * np.pi /
   180`, `0.412 * mOA` and `** 0.5` are baked in. None of the Klann
   proportions is a symbol, so changing a ratio means a full re-solve, and
   sensitivity analysis or gait optimisation is impossible.
4. **Layering is implicit** in `_jp` pos/neg joint copies and ad-hoc `a_dz=-6`
   offsets. Nobody decides which link is on which layer, which is how F1
   and F4 happened.
5. **Composition is stringly typed.** Suffix conventions (`_leg0`),
   `startswith("torso")`, regex classifiers, and a module-global
   `_BAKE_PROFILER` in `klann.py`.

## 5. Proposed direction: a sympy core with compiled fast paths

The sketch (now the core of `klann.py`):

| | current `klann.py` | sketch |
|---|---|---|
| step size | F = 34k ops | each step ≤ 126 ops, printable |
| design params | hard-coded floats | sympy symbols with exact rational defaults |
| chirality / phase | new solve per leg | symbol `s` / evaluate at `t + φ` |
| compile | 0.4 s **per leg per call** | **0.34 s once**, for all legs and designs |
| eval, 10⁵ samples | 45 ms | 27 ms |
| link motion | Procrustes after the fact | closed-form SE(2) `(x, y, θ)(t)` per link |
| parameter sweep | re-run sympy | 200 designs in 53 ms |
| parity | — | foot path within **3e-13** of `klann.py` (both chiralities, two phases) |

Suggested order of work:

1. **Core.** Replace `create_klann_geometry` with the straight-line program
   and memoize the compiled program.
2. **Rigid links.** Each link gets a fixed local frame (joint i at
   `(dᵢ, 0)`, with lengths taken from the symbols). Parts are built once, in
   that frame. This fixes F7 for free, since links become axis-aligned for
   packing.
3. **One assembly model.** A leg is `(chirality, phase, layer plan)`, and
   `sample(ts)` returns link poses; the single-t case is `sample([t])`.
   Delete the `*_template` twins, `freeze_at`, Procrustes and witness
   joints.
4. **Explicit layers, joinery after composition.** Each pin spans the
   layers of its two links. The frame is one solid plate with pins pointing
   *toward* the links. The drive stops at the conn layer. Standoffs connect
   the upper decks to the frame. Run joinery in `main.py`, not just in the
   bake.
5. **Gates.** Run `mise run audit` and the e2e suite in CI. Fix the 17 lint
   findings and the README drift.

# CLAUDE.md — pointers for future agents

This file captures non-obvious things you'll want to know before diving into
the code. Keep it terse.

## How to run things

Tasks live in `mise.toml`:

```bash
mise run view       # FastAPI + Vite (HMR); URL printed in startup banner
mise run bake       # bake viewer/data/*.glb
mise run test       # pytest (runs viewer-build first; -m e2e for browser tests)
mise run build      # STEP/STL/DXF -> build/
mise run lint       # ruff check
mise run audit      # do the parts physically fit? (see docs/audit/AUDIT.md)
mise run kill       # stop dev servers spawned from THIS worktree
mise run kill-port -- --port 5173   # force-stop whoever is on a port (orphan recovery)
mise run clean
```

Direct invocation of the bake script (more flags than `mise run bake`):

```bash
uv run python viewer/bake_gltf.py --mode robot --module quad --frames 120
uv run python viewer/bake_gltf.py --mode single          # one side only
uv run python viewer/bake_gltf.py --linkage jansen --module double   # another linkage
uv run python viewer/bake_gltf.py --mode single --linkage hoecken    # a mechanism: one side
```

`--linkage` / `--phases` / `--proportion NAME=VALUE` are shared by `main.py`,
`bake_gltf.py` and `scripts/tune_gait.py` (`walk.add_design_args`); the server
takes `linkage=`, `module=`, `phases=`, `p.NAME=`. Not every linkage's module
has a layer plan (Jansen decker/quad, Strider double/quad don't): its bake is
a 422, its `/api/walk` still works.

The viewer is a Vite + TypeScript app under `viewer/src/`. In dev, Vite
serves on a port derived from a CRC32 hash of the worktree path
(range 5500-5999) and proxies `/api` + `/ws` to FastAPI on a similarly
hash-derived port (8500-8999). This means parallel worktrees get unique
stable ports with **zero manual config** — just `mise run view` in each
and bookmark the URL printed in the banner. Override via env vars in
`mise.local.toml` (gitignored) when the auto-picked port collides:

```toml
# mise.local.toml — per-worktree, not committed
[env]
VITE_PORT = "5173"   # pin the main checkout to the canonical port
API_PORT  = "8000"
```

For single-port runs (e2e tests, prod-like), build first with
`mise run viewer-build`; `server/app.py` auto-mounts `viewer/dist` when it
exists (override via `SPIDERPIG_VIEWER_DIST`).

## Baking the glTF — performance profiler

`viewer/bake_gltf.py` has a built-in stage-level profiler. It is **on by
default** and prints a summary table via `logging` at the end of every bake.

Flags:

| flag | default | purpose |
|---|---|---|
| `--profile / --no-profile` | on | stage timings + metrics summary |
| `--cprofile PATH` | off | also dump `cProfile` `.prof` file + `<PATH>.txt` top-30 cumulative functions |
| `--log-level LEVEL` | `INFO` | `DEBUG` for per-class tessellation and frame-sampling chatter |

Instrumented stages (keys in the summary table):

1. `1_reference_build` — freeze the template at `t=0`, rationalize (or
   reuse) the side design and its layer plan, fabricate every part
2. `2_mesh_share` — find bodies whose parts are exact translates of another
   (mirrored right-side plates, the legs' links) so they share one mesh
3. `2_tessellate_total` + `2_tessellate.<kind>` — OCCT tessellation, one
   mesh per shared shape
3. `3_gltf_pack_geometry` — accessor/bufferview/material packing
4. `4_animation_sample_total` with three sub-timers (run once per bake now,
   not once per frame):
   - `4.1_template_build` — one-shot `MechanismTemplate` assembly
     (includes all `klann.create_geometry` / `lambdify` work)
   - `4.2_template_sample` — vectorized batched-BFS pose propagation
     over the whole `ts` array
   - `4.3_trs_batch` — batched planar rigid fit + quaternion hemisphere
     fix per body; hardware (`Body.rigid_with`) copies its host's motion
5. `5_gltf_nodes_channels` — glTF node + animation sampler/channel assembly
6. `6_foot_path_extra` — 64-sample foot path written to scene extras;
   `6b_drive_extra` — the drive data (feet over the crank cycle, COM, mass,
   servo rpm) on the root node `walker` for the viewer's drive mode
7. `7_serialize` — `pygltflib.GLTF2.save_binary`

Plus `bake_total` wrapping everything. The inner `klann.*` sub-timers
(`4.1a_klann.create_geometry`, `4.1b_klann.lambdify`,
`4.1c_klann.joints_at_eval`, `4.1d_klann.assemble_leg`) fire inside
`1_reference_build` + `4.1_template_build`. `4.1b` fires once per process:
the symbolic program is compiled once and cached.

Metrics the summary reports: `n_frames`, `n_legs`, `n_bodies`, per-class
`verts.*` / `tris.*`, `blob_bytes`, `gltf_bytes`, `animation_channels`,
`accessors`, `n_meshes`, `peak_rss_mb`. Counters: `body_extract.calls`,
`body_extract.static` (bodies with no joints and no host), `mesh_shared`.

### Known hot stage

`1_reference_build` (OCCT parts, ~70%) and `2_tessellate_total` (~20%)
dominate; the frame loop is ~1%. Robot quad (two sides, 91 bodies, 33
meshes) ≈ 6.6 s.

Historical: the symbolic solve used to run per leg per call (per frame,
before `19e020e`), and substituted expressions grew to ~34k ops. The
straight-line program of each linkage (`linkage.py`) is compiled once per
process; a leg's phase is a time shift. Don't reintroduce per-leg or
per-frame solves. The `*_klann.*` labels are kept for every linkage.

### How to extend

The profiler lives in `viewer/bake_gltf.py` as `_Profiler`. To add a new
bracket:

```python
with prof.timed("label"):
    ...
prof.bump("counter_name")
prof.set_metric("metric_key", value)
```

All output goes through `logging.getLogger("bake_gltf")` — do not revert to
`print`.

## Repository map

| file | role |
|---|---|
| `linkage.py` | the symbolic engine: compass-and-ruler helpers (`crank`, `circle_x_circle`, `extend`, `offset`), `Linkage` (a straight-line program over exact `params`, compiled once per linkage), `LegSolution` (mirror = reflect x at crank angle π − t), the generic leg template (bodies `coupler`, `b<k>` links, `conn`, `torso`; connections from shared joint names), composition (`combine_connectors`, `fuse_*`) and `build_module_template(module, phases, params, linkage)`. A walker has `feet`; a mechanism an `Output` (`output_check()`, promises enforced as `OutputError`) and maybe a second input (`inputs`, `crank_at`). Registry: `get` / `available(kind)`. |
| `linkages/` | one module per linkage family (Klann, Strider, Jansen, ...); each registers its `Linkage` (and variants). Auto-imported; Klann first (the default). `mechanisms.py`: building blocks (straight lines, lifts, xy, rockers), one side only; `tests/test_mechanisms.py`. |
| `explain.py` | prints each pipeline stage's verdict for a design (program checks, static clearances, plan or `PlanError`) |
| `klann.py` | the Klann-named API kept for callers: `PROPORTIONS`, `STEPS`, `KlannSolution` (= `LegSolution`), `build_*_template`, single-t `build_*_mechanism`. |
| `mechanism.py` | `Body` / `Joint` / `Pose` / `Mechanism`; `MechanismTemplate` / `SampledPoses` for batched sampling. All joints sit at z = 0: kinematics is planar. `Body.fab` / `bom_key` / `rigid_with`. |
| `stack.py` | the layer planner. Knows only **claims** (`Claim` -> `Placed` discs/pills per layer, relative to link layers), a `Topology` (links, axles as named points) and sampled `Geometry` (distances are lower bounds that cover motion between samples). `StackProblem.solve()`; `verify_plan()` re-checks exhaustively on fresh sampling. |
| `construction/` | the rationalization: one **group** per functional part (`base.py` is the contract). `axle.py` (pillars + link pins), `crank.py`, `plates.py` (laser links + frame plates), `robot.py` (two mirrored sides + chassis), `contract.py` (parts inside claims), `envelope.py`. Registries in `__init__.py`. |
| `servos/` | `ServoSpec` data (continuous-rotation servos only), the drive group (`mount.py`: servo on the inner frame plate, `DriveInterface` for the crank), models and CAD cache. |
| `hardware/` | purchasable-item catalog (`catalog.py`, data in `parts.py` and `servos/catalog.py`) and the BOM (`bom.py`). |
| `fabricate.py` | orchestration: `BuildConfig`, `design_side()` (groups -> claims -> plan, cached), `fabricate_side()`, `fabricate()` (the robot unless `robot=False`). |
| `shapes.py` | build123d primitives (disc, pill, plate, link plate, cuts incl. D-holes and rectangles) |
| `layout.py` | DXF sheets of every laser-cut body, kerf-compensated; errors instead of dropping parts |
| `scripts/audit_fab.py` | `mise run audit`: plan re-check, contract, OCCT clashes, DXF, BOM |
| `walk.py` | quasi-static walking model (support plane, no-slip velocity, per-revolution metrics); feeds `/api/walk`, the bake's drive data and `scripts/tune_gait.py`. The viewer's `viewer/src/drive/model.ts` implements the same model. |
| `viewer/bake_gltf.py` | end-to-end `.glb` bake for the three.js viewer |
| `server/app.py` | dev server: `/api/glb/{mode}?linkage=&module=&phases=&p.NAME=` bakes on demand (cached per design), `/api/walk` (same params) answers walk metrics for a design without building parts, `/api/linkages` lists the linkages (`kind`, `output`) and their params/modules for the viewer's tune panel and mechanism picker |

### Pipeline contract

1. **Symbolic** — a `Linkage`'s steps (`linkages/*.py`): each point is a
   small sympy expression over earlier points' symbols, `t` and the params.
   `O` is the crank centre at the origin, y up, feet lowest.
2. **Compiled** — `Linkage.compiled` lambdifies it once;
   `LegSolution(orientation, phase).evaluate(ts)` runs it at `ts + phase`
   (a mirrored leg: reflected, at `π − (ts + phase)`).
3. **Template** — `MechanismTemplate`: topology, per-body `outline`, and
   per-joint `pose_at` closures over the compiled program. One template is
   one *side* of the robot (a leg module: single, double, decker, quad).
4. **Rationalized** — `fabricate.design_side(tmpl, config)`: groups in
   dependency order (drive, crank, axles, links, frame), each with the
   construction the config picks; their claims; the layer plan.
5. **Fabricated** — `fabricate(tmpl, config, t)`: every group realizes its
   parts at `t` inside its claims; plates are cut last with every hole the
   other groups asked for; the robot mirrors the side and adds the chassis.
6. **Serialized** — STEP/STL/DXF/BOM (`main.py`) or `.glb` (`bake_gltf`).

Every stage says what fails, so no follow-up digging is needed
(`python explain.py --linkage K --module M` prints all three):

- **template**: `Linkage.assert_assembles` raises `AssemblyError` naming the
  step whose bars can't meet, by how much and at which crank angles.
  `Linkage.check()` gives every loop's margin and transmission angle (over
  the torus of both inputs for a two-input mechanism). A mechanism's
  `output_check()` measures its output; `assert_output` raises `OutputError`
  when it breaks a promise (a platform that turns, a line not straight to
  its tolerance, a dwell too short).
- **drive**: one servo turns `t`; a second input stops at
  `ConstructionError` (`servos/mount.py`).
- **static clearance**: each group declares `keepouts(ctx)` (an axle's neck
  over its span, the crank at O). `side_clearances` lists every link that
  can never share their layers. `stack.impossible` raises `ClearanceError`
  when no layer can hold a link at all.
- **plan**: the planner tallies what blocked it, and claims raise
  `Unbuildable(reason)` rather than returning `None`. If the search fails,
  `stacked_plan` composes the side from a verified smaller module's plan.
  Otherwise `PlanError` lists the blockers with distances, the stacking
  result and the static clearances involved.

A new construction or claim must keep this up: raise with a reason, and
declare its keep-outs.

Correct by construction: the planner guarantees claims of different groups
never meet over the whole crank cycle, and `construction.contract` checks
that every part lies inside its own group's claims. A construction that
can't be built with the given parameters raises `ConstructionError` before
planning; a layout it can't be built in makes its claim return `None`.

To add a construction: implement `dims(ctx)` (validation, the radii its
claims use) and `realize(group, build)` (parts inside those claims), register
it in `construction/__init__.py`, run the contract tests. To add a leg
module: add it to `linkage.MODULE_LEGS` (or a linkage's own `modules`). To
add a linkage: a module in `linkages/` with its params, program, links
(`b<k>` -> joints, outline), frame, crank and feet (a mechanism: its
`output`); `tests/test_linkage.py` checks it assembles, stays rigid and
plans (`tests/test_mechanisms.py`: outputs against the research's numbers).

### Stacking (future)

Not built. A stage would mount on its parent's output body (`Output.frame`:
origin joint, x-axis joint). The engine would need: the child's fixed pivots
placed as `offset(J1, J2, along, across)` on that body instead of `xy`; its
point names prefixed per stage so the programs concatenate into one; its
input added to `inputs` (its crank turns relative to the parent body, so its
angle is its input plus that body's rotation); one drive per input; and the
planner's clearances between bodies in relative motion across stages (the
child's frame is a moving body, not the frame plates).

Physical rules the claims encode:

- The links riding a crankpin (Klann's b1; Jansen's j and k; Strider's
  bars) sweep over the crank axis O, and the crank turns fully relative to
  them, so the crank crosses a rider's layer only along its crankpin: a
  built-up crankshaft with webs either side of each rider.
- Layers 0 (outer frame plate) and `top` (inner frame plate) hold nothing
  but the plates and parts seated in their holes.
- Pillars (frame pivots) are anchored in both frame plates whenever the
  mechanism lets them reach both; every link on an axle is held in its
  layer by a shoulder, head, cap, plate or neighbouring link on each side.

## House rules

- Don't add `print` statements to the bake path — use the `bake_gltf` logger.
- Don't regress the profiler (keep the stage keys stable; downstream scripts
  may parse them).
- Don't put Z into joint poses. Z is the stack plan's job.
- A group builds only inside its own claims; keep `check_side` at `[]`.
- A change that alters parts should leave `mise run audit` green.
- `verbose=True` on `bake_gltf()` is back-compat only: it forces the logger
  to DEBUG. Prefer `--log-level DEBUG` from the CLI.

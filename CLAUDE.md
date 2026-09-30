# CLAUDE.md — pointers for future agents

This file captures non-obvious things you'll want to know before diving into
the code. Keep it terse.

## How to run things

Tasks live in `mise.toml`:

```bash
mise run view       # FastAPI + Vite (HMR); URL printed in startup banner
mise run bake       # bake <store>/bakes/<design>.glb (the project store, .spiderpig/)
mise run test       # pytest (runs viewer-build first; -m e2e for browser tests)
mise run build      # STEP/STL/DXF -> build/
mise run lint       # ruff check
mise run audit      # do the parts physically fit? (see docs/audit/AUDIT.md)
mise run explain    # each pipeline stage's verdict on a design
mise run tune       # search crank phases for a smoother walk
mise run sim        # MuJoCo
mise run report     # compare every linkage -> build/linkages.json
mise run kill       # stop dev servers spawned from THIS worktree
mise run kill-port -- --port 5173   # force-stop whoever is on a port (orphan recovery)
mise run clean
```

`build`, `bake`, `audit`, `explain`, `tune`, `sim`, `report`, `mcp` and `view` are
the subcommands of the `spiderpig` console script (`spiderpig/cli.py`; `spiderpig
<command> --help`; from a checkout `uv run python -m spiderpig.cli <command>`, or
`mise run <command> -- <options>`, which passes options through); the tools' own
modules (`spiderpig/build.py`, `spiderpig/bake.py`, `spiderpig/explain.py`,
`spiderpig/tools/*.py`, `spiderpig/mcp/`, `spiderpig/view.py`) are what it runs. For
example:

```bash
spiderpig bake --module quad --frames 120
spiderpig bake --module single --side             # one side only
spiderpig bake --linkage jansen --module double   # another linkage
spiderpig bake --side --linkage hoecken           # a mechanism: one side
```

What every tool builds is a `spiderpig.config.BuildConfig` (linkage, module,
robot or side, phases, proportions, servo, constructions, sheet), which
validates itself; `--linkage` / `--module` / `--phases` / `--proportion
NAME=VALUE` and the build options are shared by `build.py`, `bake.py`,
`explain.py` and the tools (`config.add_design_args` / `add_build_args` /
`config_from_args`); the server takes `linkage=`, `module=`, `phases=`,
`p.NAME=` (`config.design_from_query`). A bake is cached as
`<store>/bakes/<config.key>.glb` (`bake.default_bake_dir()`: the project
store, `$SPIDERPIG_STORE` else `./.spiderpig`; `klann_quad_robot.glb`, a hash
suffix for a non-default design). A design with no layer plan (the planner says why, e.g.
TrotBot's heel scaled back to its drawing's 7 mm unit, `p.unit=7`) bakes a
422; its `/api/walk` still works.

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
`mise run viewer-build`: Vite's `outDir` is `spiderpig/viewer/dist` (git-ignored
package data, what a release wheel ships); `spiderpig/server/app.py` mounts it
when it exists (override via `SPIDERPIG_VIEWER_DIST`; without a build `/`
answers 503 and the API still works). `spiderpig view <design>` serves that
app for a stored design with no Node on the machine (the wheel carries the
dist; `mise run release` builds both): see `docs/agentlib/API.md`, "View".

## Baking the glTF — performance profiler

`spiderpig/bake.py` (the viewer's bake) has a built-in stage-level profiler. It is **on by
default** and prints a summary table via `logging` at the end of every bake.

Flags:

| flag | default | purpose |
|---|---|---|
| `--profile / --no-profile` | on | stage timings + metrics summary |
| `--log-level LEVEL` | `INFO` | `DEBUG` for per-class tessellation and frame-sampling chatter |

(For a function-level profile run the script under `python -m cProfile`.)

Instrumented stages (keys in the summary table):

1. `1_reference_build` — freeze the template at `t=0`, rationalize (or
   reuse) the side design and its layer plan, fabricate every part
2. `2_mesh_share` — find bodies whose parts are exact translates of another
   (mirrored right-side plates, the legs' links) so they share one mesh
3. `2_tessellate_total` + `2_tessellate.<kind>` — OCCT tessellation, one
   mesh per shared shape (positions and indices only: the viewer shades flat)
3. `3_gltf_pack_geometry` — accessor/bufferview/material packing
4. `4_animation_sample_total` with three sub-timers (run once per bake now,
   not once per frame):
   - `4.1_template_build` — one-shot `MechanismTemplate` assembly (the
     linkage's program was compiled once per process by then)
   - `4.2_template_sample` — vectorized batched-BFS pose propagation
     over the whole `ts` array
   - `4.3_trs_batch` — batched planar rigid fit + quaternion hemisphere
     fix per body; hardware (`Body.rigid_with`) copies its host's motion
5. `5_gltf_nodes_channels` — glTF node + animation sampler/channel assembly
6. `6_foot_path_extra` — 64-sample foot path written to scene extras;
   `6b_drive_extra` — the drive data (feet over the crank cycle, COM, mass,
   servo rpm) on the root node `walker` for the viewer's drive mode
7. `7_serialize` — `pygltflib.GLTF2.save_binary`

Plus `bake_total` wrapping everything.

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
straight-line program of each linkage (`spiderpig/linkage/engine.py`) is compiled once per
process; a leg's phase is a time shift. Don't reintroduce per-leg or
per-frame solves.

### How to extend

The profiler lives in `spiderpig/bake.py` as `_Profiler`. To add a new
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

Everything Python is the `spiderpig` package (installable: `pyproject.toml`,
hatchling; `uv sync` installs it editable, `spiderpig` is its console script).
`tests/` and the viewer's TypeScript sources (`viewer/`) stay beside it.

| file | role |
|---|---|
| `spiderpig/linkage/` | the symbolic side, one package re-exporting everything. `engine.py`: compass-and-ruler helpers (`crank`, `circle_x_circle`, `extend`, `offset`), `Linkage` (a straight-line program over exact `params`, compiled once per linkage), the registry (`get` / `available(kind)`), `LegSolution` (mirror = reflect x at crank angle π − t), `scale_params`. `checks.py`: the stage checks (`check_steps`: every loop's margin and transmission angle; `check_output`: a mechanism's output against its promises). `assembly.py`: the generic leg template (bodies `coupler`, `b<k>` links, `conn`, `torso`; connections from shared joint names), composition (`combine_connectors`, `fuse_*`), `build_module_template(module, phases, params, linkage)` and `feet_of`. A walker has `feet`; a mechanism an `Output` (`output_check()`, promises enforced as `OutputError`) and maybe a second input (`inputs`, `crank_at`). |
| `spiderpig/linkages/` | one module per linkage family (Klann, Strider, Jansen, ...); each registers its `Linkage` (and variants). Auto-imported; Klann first (the default). `mechanisms.py`: building blocks (straight lines, lifts, xy, rockers), one side only; `tests/test_mechanisms.py`. |
| `spiderpig/explain.py` | prints each pipeline stage's verdict for a design (program checks, static facts, plan with its crank route and proof, or the stage's error and what would clear it) |
| `spiderpig/recommend.py` | what would clear a static or plan failure, checked by re-running the stage: the least practical scale of the linkage (`linkage.scale_params`), thinner `Params` parts within every construction's `dims()`, the default scale after a scale-down, printed pillars for a plan a bolt pillar's stock screw bounds; and, for a design that plans but misses a target that scales with the linkage (a mechanism's stroke or straightness, a walker's lift), `target_scale`: the least practical scale that meets it, measured again and planned (`api.advise`) |
| `spiderpig/mechanism.py` | `Body` / `Joint` / `Pose` / `Mechanism`; `MechanismTemplate` / `SampledPoses` for batched sampling (numpy 4x4s). All joints sit at z = 0: kinematics is planar. `Body.fab` / `bom_key` / `rigid_with`. |
| `spiderpig/stack.py` | the layer planner. Knows only **claims** (`Claim` -> `Placed` discs/pills per layer, relative to link layers; an `early` part checked as soon as a group's own links are placed), a `Router` (a group whose shape it chooses per layering: the crank), a `Topology` (links, axles as named points, points fixed to the crank) and sampled `Geometry` (distances are lower bounds that cover motion between samples). `StackProblem.solve()` (see "The planner" below); `verify_plan()` re-checks exhaustively on fresh sampling. |
| `spiderpig/construction/` | the rationalization: one **group** per functional part (`base.py` is the contract). `axle.py` (pillars + link pins: the claims, `AxleDims`, the `printed` snap axle), `crank.py` (routes, claims, the printed crankshaft), `route.py` (the crank's router: static facts, detours, the exact route per layering), `underside.py` (the body's underside: the envelope, ground clearance), `plates.py` (laser links + frame plates), `robot.py` (two mirrored sides, the frame ties' holes, the assembly), `chassis.py` (the servo frames in the plate plane, centre plates, rear screws, tie columns), `contract.py` (parts inside claims), `envelope.py` (solids of claims). Registries in `__init__.py`. |
| `spiderpig/construction/pivots/` | metal-shaft pivots (`--pin` / `--pillar` keys; its docstring holds the hardware research): `rod` (3 mm rod, laser-cut spacer rings, Starlock clips, glued into the frame plates; `rod.py`), `bolt` (M3 SHCS axle, rings, washer + nylock; a pillar clamps both plates, the nut end claims 2-3 layers; `bolt.py`), `bearing` (MF63ZZ flanged bearing glued in each link, rod, printed sleeves; `insert.py`), `bushing` (igus GFM-0304-03 pressed in each link, same; `insert.py`). Their claims fill every layer (`AxleDims.fill`: a rod can't neck, so `neck` is the narrowest ring or sleeve), flanges need a free face (`AxleDims.flange`, `flange_sides`), retainers come from the construction's `ends` hook. Catalog additions in `hardware/fastener_catalog.py`. |
| `spiderpig/servos/` | `ServoSpec` data (continuous-rotation servos only), the drive group (`mount.py`: servo on the inner frame plate, `DriveInterface` for the crank), models and CAD cache. |
| `spiderpig/hardware/` | purchasable-item catalog (`catalog.py`, data in `parts.py` and `servos/catalog.py`; the sheet helpers), the screw families (`fasteners.py`: heads, stock lengths, keys, solids), materials and exact mass properties (`mass.py`: the one density table, `material_of`, `part_props`) and the BOM (`bom.py`). |
| `spiderpig/config.py` | `BuildConfig`: what to build and how, validated on construction (the linkage's module, one phase per leg, the linkage's proportions; defaults dropped so a design has one config and one `key`), the shared CLI arguments and the server's query parsing. |
| `spiderpig/fabricate.py` | orchestration: `design_side()` (groups -> claims -> plan, cached; the robot's side is the side's design), `fabricate_side()`, `fabricate()` (the robot unless `robot=False`: the frame ties join at build time). |
| `spiderpig/shapes.py` | build123d primitives (disc, pill, plate, cuts incl. D-holes and rectangles) |
| `spiderpig/layout.py` | DXF sheets of every laser-cut body, kerf-compensated; errors instead of dropping parts |
| `spiderpig/cli.py` | the one entry point, the `spiderpig` console script (`python -m spiderpig.cli` from a checkout): `build` (`spiderpig/build.py`: STEP/STL/DXF/BOM), `bake` (`spiderpig/bake.py`), `audit`, `tune`, `sim`, `report` (`spiderpig/tools/`), `explain`, `mcp`, `view` (each a module's `main(argv)`); the `mise` tasks run it. It imports nothing of the engine until a command runs |
| `spiderpig/view.py` | `spiderpig view <design> [--store] [--port] [--open]`, or `spiderpig view --linkage ... --pin bolt` (the build options: `resolve_args` turns them into a design in the store through `api.spec_of`): the store as the MCP picks it, `api.export(design, ["glb"])` (cached), the server below on a free port, the URL `/?design=<id>`; `start_background()` runs it as a child process (`--serve-only`) for the MCP `view` tool |
| `spiderpig/tools/` | `audit.py` (`mise run audit`: plan re-check, contract, OCCT clashes, DXF, BOM; `construction.contract` has the checks), `tune.py` (crank phases and proportions for a smoother walk), `sim_walk.py` (the MuJoCo CLI), `report.py` (every linkage compared), `dev.py` / `kill_dev.py` (`mise run view` / `kill`: the dev servers, a checkout only) |
| `spiderpig/walk.py` | quasi-static walking model (support plane, no-slip velocity, per-revolution metrics); feeds `/api/walk`, the bake's drive data and `spiderpig/tools/tune.py`. The viewer's `viewer/src/drive/model.ts` implements the same model. |
| `spiderpig/sim/` | MuJoCo: `mjcf.py` builds the MJCF of the fabricated robot (exact masses, loop equalities, velocity drives) and its viewer metadata; `run.py` steps it (`simulate`, `walk_metrics`, kinematic playback). `spiderpig/tools/sim_walk.py` is the CLI. |
| `spiderpig/bake.py` | end-to-end `.glb` bake for the three.js viewer (`spiderpig bake`; cached in the store's `bakes/`) |
| `viewer/` | the Vite + TypeScript three.js client (`src/`), built by `mise run viewer-build` into `spiderpig/viewer/dist` (package data); its `node_modules` never ship |
| `spiderpig/server/app.py` | the viewer's FastAPI app (the dev server and `spiderpig view`, serving `spiderpig/viewer/dist`; `configure(store, prebake_default)`): `/api/glb/{mode}?linkage=&module=&phases=&p.NAME=` bakes on demand (cached per design; `mode` is `robot`, `side`, or one of the side-only ids old URLs use, `MODES`), `/api/walk` (same params) answers walk metrics for a design without building parts, `/api/linkages` lists the linkages (`kind`, `output`) and their params/modules for the viewer's tune panel and mechanism picker, `/api/modes` the dropdown's ids and labels; `?design=<id>` on `/api/glb` and `/api/walk` starts from a stored design's `BuildConfig` (`resolved.json`) and applies the other parameters on top (the glb from the store's export when it matches: `X-Spiderpig-Glb`), `/api/design/{id}` is its card for the page |
| `spiderpig/` | the agent-facing surface (`docs/agentlib/API.md`): `spec.py` (Spec v1: a validated, JSON-schema'd document; every metric a `Target`, hard or soft per `TARGET_FIELDS`, unknown fields and wildcards rejected with `SpecError[]`), `api.py` (`resolve` -> `Design` with a content-addressed id; `check`, `plan`, `explain`, `recommend`, `walk`, `build`, `recheck`, `verify`, `export` as pure functions of the handle, each mapping onto one engine pass and returning a report), `failure.py` (every engine exception as a `Failure`: stage, code, culprits, numbers, blockers, recommendations with spec patches), `verify.py` (rows with a `proven` / `measured` / `estimated` tier at `quick` / `standard` / `full`), `design.py` (the handle, `Part` with the live build123d `solid`; an edited solid is outside the guarantee until `recheck` passes), `store.py` (the per-project store, `$SPIDERPIG_STORE` else `./.spiderpig`, git-ignored: `designs/<id>/` with the spec, the resolved record, one JSON per stage, the build's STEP parts, exports and a log; every op reads its stage when valid for the engine version, a plan is re-made and `verify_plan`ed on reload, `load` / `derive` / `compare` / `list_designs` / `gc`; `store=None` for memory). It only calls the engine; CLI and MCP come after it. |
| `spiderpig/mcp/` | the MCP server over that API (`spiderpig mcp --store PATH`, `mise run mcp`, `python -m spiderpig.mcp`; stdio; the official `mcp` SDK 2.x, `MCPServer`): one tool per operation, files and numbers only (a design is its id, a part the path of its STEP file in the store, a failure the `Failure` document under `failures` with `ok: false`; a misused tool sets `isError` with the same envelope). `outputs.py` holds the TypedDict output schemas, `jobs.py` the process pool per store (`build`, `verify` standard/full and `export` come back as jobs after `wait_seconds`; `get_job` / `wait_job`; `view(design)` starts or reuses a `spiderpig view --serve-only` child process and returns the URL), `guide.md` the `spiderpig://guide` resource (the vocabulary tables are generated from `TARGET_FIELDS` and the registries). Engine calls run in a worker thread one at a time; `tune` / `search` are not in v1. `tests/test_spiderpig_mcp.py` drives it through the SDK's in-memory client. |

### Pipeline contract

1. **Symbolic** — a `Linkage`'s steps (`spiderpig/linkages/*.py`): each point is a
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
6. **Serialized** — STEP/STL/DXF/BOM (`spiderpig/build.py`) or `.glb` (`spiderpig/bake.py`).

Every stage says what fails, so no follow-up digging is needed
(`spiderpig explain --linkage K --module M` prints all three):

- **template**: `Linkage.assert_assembles` raises `AssemblyError` naming the
  step whose bars can't meet, by how much and at which crank angles.
  `Linkage.check()` gives every loop's margin and transmission angle (over
  the torus of both inputs for a two-input mechanism). A mechanism's
  `output_check()` measures its output; `assert_output` raises `OutputError`
  when it breaks a promise (a platform that turns, a line not straight to
  its tolerance, a dwell too short).
- **drive**: one servo turns `t`; a second input stops at
  `ConstructionError` (`spiderpig/servos/mount.py`).
- **static facts**: each group declares `keepouts(ctx)` (an axle's neck over
  its span, a pillar's to a plate, the crank's journal at O).
  `side_clearances` lists every link that can never share their layers. The
  crank's router (`construction.route.crank_facts`) knows which links sweep O
  (their layer needs the crank off its axis) and which crank points each
  clears; `fabricate.static_stage` raises `ClearanceError` for a link no
  crankpin and no detour inside the body's underside clears, with the
  distances (`NoCrankPoint`).
- **plan**: the planner tallies what blocked it, and claims raise
  `Unbuildable(reason)` rather than returning `None`. `PlanError` lists the
  blockers with distances and the static clearances involved. A plan says
  whether it is proven the thinnest (`StackPlan.optimal`, `proof`: nodes per
  size ruled out, or which sizes a budget left open, and when the crank's
  joints forced a taller stack).
- Both `ClearanceError` and `PlanError` carry `recommendations`
  (`stack.Recommendation`: what to change, from, to, why, side effects, and
  what re-running showed), printed under "what would clear it:". A
  recommendation is only given once the stage passes with it
  (`recommend.py`); what can't help goes in the notes.

A new construction or claim must keep this up: raise with a reason, and
declare its keep-outs.

Correct by construction: the planner guarantees claims of different groups
never meet over the whole crank cycle, and `construction.contract` checks
that every part lies inside its own group's claims. A construction that
can't be built with the given parameters raises `ConstructionError` before
planning; a layout it can't be built in makes its claim return `None`.

To add a construction: implement `dims(ctx)` (validation, the radii its
claims use) and `realize(group, build)` (parts inside those claims), register
it in `spiderpig/construction/__init__.py`, run the contract tests. To add a kind of
group (a second drive, spacer rings): subclass `construction.base.Group`
(`claims`, `realize(build, done)`; `keepouts` / `interface` if it has any;
`cuts = True` if it cuts what the others asked for) and append its factory
to `construction.GROUP_FACTORIES`, in dependency order. To add a leg
module: a `linkage.Module` (its legs, and which of them share one crank
body) in `linkage.MODULES` or a linkage's own `modules`. To
add a linkage: a module in `spiderpig/linkages/` with its params, program, links
(`b<k>` -> joints, outline), frame, crank and feet (a mechanism: its
`output`); `tests/test_linkage.py` checks it assembles, stays rigid and
plans (`tests/test_mechanisms.py`: outputs against the research's numbers).

### The planner

`stack.StackProblem.solve()` finds the thinnest stack, and the cheapest
crank route in it:

1. **Static facts** (before any layer): the keep-outs above; each link's own
   shapes per layer; the crank's facts. A link with no crank point stops here.
2. **Search per stack size** (`_Search`), fewest layers first: link layers by
   fewest open layers, then the assembly tree from the crank out. Forward
   checking (every placed shape removes the layers it rules out; an axle's
   links bound where links that can't pass it may go; a pillar must reach a
   plate), the router as a sub-check at every node (`check`: bit-mask
   reachability over layers x crank states, and which states each layer still
   has on a route, which prunes more), conflicts as the links behind a
   failure, backjumping to the latest of them, learned nogoods (watched), and
   branch and bound on the route's cost.
3. **The route** for a complete layering (`CrankRouter.route`): exact, a
   shortest path over layers and **chains** (runs along one point whose webs
   meet: one screw). Buildable only: a stock screw per chain
   (`JointRules.spans`, from `PrintedCrank.post_joint`, end-play faces
   included), one chain per point, pockets of consecutive chains (and the
   last one and the horn screws) apart, a chain ending set back in the hub's
   lowest layer only if the horn screws still fit the shortened hub
   (`JointRules.hub_play`). Cost, in order: added features (run
   layers no rider needs, detour runs), detour sweep, a dropped bearing
   (`StackSpec.drop_bearing`, off by default), then fewer runs.
4. **Verification**: `problem.plan(layers, top, choices)` and `verify_plan`;
   a failure there is a bug (it raises).

Effort: a short search per size until one finds a plan (sizes in turn while
they are ruled out; once one exhausts its short budget, in doubling steps,
then back down the skipped sizes; for a multi-leg module, a second
strategy: a leg at a time at the single module's layers, `hint`; when no
size found one, the sizes left open get the full budget, thinnest first),
then the thinner sizes it didn't rule out with the full budget (the next
thinner first), then a cheaper route. A group may bound the sizes searched
(`Group.max_top(ctx)`: a bolt pillar's longest stock screw clamps at most
15 layers of 3 mm); `side_problem` caps `StackSpec.max_top` and the
`PlanError` names the bound (`StackProblem.notes`). Budgets (`StackSpec.quick_nodes`, `max_nodes`,
`max_total_nodes`) and the wall-clock deadline (`StackSpec.max_seconds`,
60 s: a node's cost grows with the stack size) bound all of it, never
validity: a plan found is returned with `optimal=False` and a `proof`
naming the sizes left open and what ran out; none found raises `PlanError`
with the tally (what blocked it, and per size: ruled out, left open at its
budget with nodes and seconds, or not tried). The checks `recommend.py`
re-runs share one more such deadline, so `design_side` returns within a few
minutes at worst. `tests/brute.py` is an independent brute force (every
layering, every route) the tests compare the planner's optimum with.

**Envelope** (`spiderpig/construction/underside.py`): the body (frame plates, the
crank's own sweep, servo, centre plates) has an underside profile; what the
planner adds to the crank turns with it, so a detour sweeps a full circle
about O, which must stay `margin` above the profile and within the body's
x-extent (`Underside.allows`). `SideDesign.ground_clearance_mm`: the body's
lowest point above the lowest foot point.

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
  built-up crankshaft with webs either side of each rider. A link pinned to
  a rider inside the crank circle (TrotBot's B8, the 6-bar's B6) sweeps O
  too: the crank runs along a post (the crankpin, or a detour point fixed to
  the crank) through its layer, a post it must clear.
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
- Tests build each design, side and robot once per session (the `design` /
  `side` / `robot` factories in `tests/conftest.py`): read them, never
  mutate them. `tests/test_contract.py` is the contract and clash check for
  every module and servo; bakes, the CLI build, MuJoCo and the tuner are
  marked `slow` (in the default run; `-m 'not slow'` skips them).
- One density table, one OCCT mass query: `spiderpig/hardware/mass.py`. One
  screw table: `spiderpig/hardware/fasteners.py` (`spiderpig/construction/crank.py` still carries its
  own until its rewrite lands).

# CLAUDE.md — pointers for future agents

Non-obvious things to know before diving into the code. Keep it terse: a pointer file.
Where the rest lives:

- how it works: `docs/ARCHITECTURE.md`, and each module's docstring
- the design numbers (layers, heights, parts, costs): `docs/agentlib/DESIGNS.md`, generated
  from the gate's snapshot; never quote them here
- why each hardware choice was made, dated: `docs/agentlib/DECISIONS.md`
- testing: `docs/agentlib/TESTING.md`; the plan and what's left: `docs/agentlib/ROADMAP.md`

## How to run things

Tasks live in `mise.toml` (AGENTS.md: use them, not raw `uv` / `npm`):

```bash
mise run view       # FastAPI + Vite (HMR); URL printed in startup banner (npm install: network)
mise run bake       # bake <store>/bakes/<design>.glb (the project store, .spiderpig/)
mise run guide      # ASSEMBLY.pdf + ASSEMBLY.md -> build/ (cached: an unchanged design ~5 s)
mise run build      # STEP/STL/DXF/BOM -> build/ (-- --profile: stage timings; a current
                    # --out is skipped, -- --force rebuilds)
mise run test-<module>  # one module's fast tier, seconds: linkage, planner, construction,
                        # hardware, strength, api, sim, server. Iterate with these
mise run test-viewer    # the viewer's tsc + vitest
mise run test-quick # the quick tier: -m 'not slow and not e2e', xdist -n 4
mise run test-fixtures  # rewrite the recorded fixtures (after an engine edit: they warn stale)
mise run test       # pytest, serial (runs viewer-build first; -m e2e for browser tests)
mise run gate -- compare ~/.cache/spiderpig/gate/next-ba41c10   # parts/plans/BOM/DXF identity
mise run gate -- doc ~/.cache/spiderpig/gate/<baseline>        # regenerate DESIGNS.md
mise run remote-test               # the full suite on the remote runner (may be down: AGENTS.md)
mise run remote-audit              # the Strider's modules audited at once there
mise run remote -- uv run python -m spiderpig.cli sim   # sims/bakes/anything there
mise run scorecard  # every ROADMAP number -> build/scorecard.json (~10 min; --gate/--full/
                    # --audit opt-in; -- --compare A B); docs/agentlib/scorecard-baseline.json
mise run doc-check -- --strict   # the docs' backticked names vs the code (CI blocks on it)
mise run lint       # ruff check, then lint-imports (the layers: pyproject.toml [tool.importlinter])
mise run pyright    # basic mode ([tool.pyright]); CI holds its error count
mise run audit      # do the parts fit and hold? (spiderpig/tools/audit.py; 75-120 s)
mise run explain    # each pipeline stage's verdict on a design
mise run tune       # search crank phases for a smoother walk
mise run sim        # MuJoCo
mise run report     # compare every linkage -> build/linkages.json
mise run kill       # stop dev servers spawned from THIS worktree
mise run kill-port -- --port 5173   # force-stop whoever is on a port (orphan recovery)
mise run clean
```

`build`, `bake`, `guide`, `audit`, `explain`, `tune`, `sim`, `export`, `report`, `mcp` and `view` are
the subcommands of the `spiderpig` console script (`spiderpig/cli.py`; `spiderpig <command>
--help`; from a checkout `uv run python -m spiderpig.cli <command>`, or `mise run <command>
-- <options>`; `spiderpig mcp --store PATH` for an agent). E.g. `spiderpig bake --module
single --side`, `spiderpig bake --linkage
jansen --module double`. What every tool builds is a `spiderpig.config.BuildConfig`, which
validates itself; the build options are shared (`config.add_design_args` /
`add_build_args` / `config_from_args`), the server takes `linkage=`, `module=`, `phases=`,
`p.NAME=` (`config.design_from_query`). `--module` defaults to the linkage's own
(`config.default_module` / `default_robot`: a walker's `quad` or its `default_module`, the
robot; a mechanism's `single`, one side).

## Defaults today

- `BuildConfig()` is `linkage.DEFAULT` (Strider) in its `default_module`, the double robot,
  on the STS3215 servo.
- Constructions: `--pin chicago --pillar standoff --crank bolt`. The `bolt` crank is single
  aluminium web plates on steel hex-standoff crankpins (`construction/crank/`); `bolt_round`
  is the round friction-clamped standoff. Nothing else exists (`config.REMOVED_CONSTRUCTIONS`,
  `config.REMOVED_PARAMS`: a removed key fails with its replacement).
- Sheets: links, rings and deck acrylic 3 mm (`BuildConfig.sheet`); frame 0.080 in 5052
  (`frame_sheet`); crank 0.100 in 6061-T6 (`crank_sheet`, `materials.thinnest_sheet`);
  centre plates by `chassis.centre_sheet`; aluminium links per `materials.LINK_SHEETS`.
- Sheets go to SendCutSend (the acrylic too since 2026-10-08, `--sheet acrylic_3mm_ponoko`
  for Ponoko); a service's sheet is a cutting line (`bom.CutRow`, `bom.cut_estimate`), in
  the BOM's and ORDER.md's totals.
- Per-linkage overrides are data in `spiderpig/config.py`: `LINKAGE_CRANKS` (TrotBot's heel
  and toe on `bolt_round`), `MODULE_CRANKS` (empty), `LINKAGE_CRANK_SHEETS`
  (`hoecken_pantograph` on 0.080 in 6061), `LINKAGE_TORQUE_LIMITS`; Klann variants' quads at
  `linkage.KLANN_QUAD`. `config.default_crank` / `default_crank_sheet` read them. The
  Chicago barrels a linkage stocks on a crank: `chicago.BARRELS` (the Strider's bolt crank),
  beside `MAX_BARREL`.
- Heads: `StackSpec.heads` "best" (sunk first); a single-plate crank plans "gap_sink".
- Numbers (layers, height, parts, cost, audit verdict): `docs/agentlib/DESIGNS.md`.

## Environment

- `SPIDERPIG_STORE`: the project store (default `./.spiderpig`).
- `SPIDERPIG_PLAN_SECONDS`: the planner's CPU budget (60; outside `engine_version`'s hash).
- `SPIDERPIG_WORKERS=0`: no worker processes (`spiderpig/workers.py`).
- `SPIDERPIG_FAB_CACHE=off`: no product fabrication cache (`spiderpig/fabcache.py`,
  `<store>/fab/`, keyed by `spiderpig/keys.py`'s code closure, the plan and the servo model).
- `SPIDERPIG_OCCT_THREADS`: OCCT's pool per process (`workers.occt_threads`; unset: every
  core). Reported numbers are tie-stable across it (`spiderpig.rounding`).
- `SPIDERPIG_OFFLINE=1` / `SPIDERPIG_SERVO_CAD=0` / `SPIDERPIG_CAD_CACHE`: servo CAD downloads.
- `SPIDERPIG_VIEWER_DIST`: the built viewer the server mounts.
- `SPIDERPIG_DEV_ORIGIN_PORT`: set by `mise run view` for the API (the Vite port whose
  loopback pages may open its WebSockets).
- `SPIDERPIG_DIGEST_CACHE`: where `engine_version()` and `spiderpig/keys.py` keep their
  digests and each source's index (`off` recomputes).
- Tests: `SPIDERPIG_TEST_CACHE` (`~/.cache/spiderpig/test-cache/`; `off` builds afresh),
  `SPIDERPIG_TEST_CACHE_DAYS`, `SPIDERPIG_TIER_WORKERS` (a tier's `-n`),
  `SPIDERPIG_TEST_OCCT_THREADS`, `SPIDERPIG_SLOW_WARN_S`, `SPIDERPIG_REGEN`,
  `SPIDERPIG_REQUIRE_VIEWER_TESTS=1` (the walk-model parity test fails, not skips, without
  Node), `SPIDERPIG_SHOTS` (e2e screenshots).
- Remote: `SPIDERPIG_REMOTE`, `SPIDERPIG_REMOTE_DIR`, `SPIDERPIG_REMOTE_WORKERS`.
- Dev servers: `VITE_PORT`, `API_PORT`, `VITE_ALLOWED_HOSTS`, `SPIDERPIG_NO_BROWSER`.

Nothing in the environment changes a design's parts: a design's id holds everything that
shapes it. In a sandbox, set TMPDIR to a roomy disk (pytest, OCCT and the gate write there).

## Ports and the viewer

The viewer is a Vite + TypeScript app under `viewer/src/`. In dev, Vite serves on a port
derived from a CRC32 hash of the worktree path (5500-5999) and proxies `/api` + `/ws` to
FastAPI on a similarly derived port (8500-8999), so parallel worktrees get stable ports
with no config: `mise run view` in each, bookmark the banner's URL. Override in
`mise.local.toml` (gitignored) when the auto-picked port collides:

```toml
[env]
VITE_PORT = "5173"   # pin the main checkout to the canonical port
API_PORT  = "8000"
VITE_ALLOWED_HOSTS = ".ts.net"   # extra Host names Vite and the API answer (`tailscale serve`)
```

The server refuses any other Host header with a 400 and a WebSocket from a foreign origin
(`spiderpig/server/app.py`). For single-port runs (e2e, prod-like) `mise run viewer-build`
first: its `outDir` is `spiderpig/viewer/dist` (package data, what a wheel ships; without
it `/` answers 503 and the API still works). `spiderpig view <design> [--store] [--port]
[--open]` serves that app with no Node on the machine (`docs/agentlib/API.md`, "View"; the
build options instead of an id go through `view.resolve_args`). Routes:
`/api/glb/{mode}?linkage=&module=&phases=&p.NAME=` bakes on demand into the store's
`bakes/` (`strider_double_robot.glb`; a design with no plan, e.g. TrotBot's heel at
`p.unit=7`, answers 422), `/api/walk`, `/api/linkages`, `/api/modes`, `/api/design/{id}`,
and `/ws/sim?linkage=&module=` (the design live in MuJoCo, `spiderpig/sim/live.py`).

## Baking the glTF — performance profiler

`spiderpig/bake.py` has a stage profiler, **on by default** (`--profile / --no-profile`;
`--log-level LEVEL`, `DEBUG` for per-class chatter), printing a summary through
`logging.getLogger("bake_gltf")` (a function-level profile: `python -m cProfile`). Stage keys: `1_reference_build` (template at `t=0`, the side design and plan,
every part fabricated), `2_mesh_share` (one mesh per congruence group, mass properties as
they are measured), `2_tessellate_total` + `2_tessellate.<kind>` (`mesh.mesh_part` per
shape, then `mesh.read_meshes` reads them all), `3_gltf_pack_geometry`,
`4_animation_sample_total` (`4.1_template_build`, `4.2_template_sample`: batched pose
propagation over the whole `ts` array, `4.3_trs_batch`: planar rigid fit per body;
hardware copies its host's motion, `Body.rigid_with`), `5_gltf_nodes_channels`,
`6_foot_path_extra`, `6b_drive_extra` (the drive data on the root node `walker`),
`7_serialize` (`pygltflib.GLTF2.save_binary`), and `bake_total`. Metrics: `n_frames`,
`n_legs`, `n_bodies`, `verts.*` / `tris.*`, `blob_bytes`, `gltf_bytes`,
`animation_channels`, `accessors`, `n_meshes`, `peak_rss_mb`; counters
`body_extract.calls`, `body_extract.static`, `mesh_shared`.

**Where the time goes** (measured 2026-10-08 at 8bfc039, `spiderpig bake --profile` of the
default Strider double robot, 3 runs, 20-core box at load ~4-6): `bake_total` 18.3-20.8 s;
`1_reference_build` 55-62 % (OCCT parts), `2_tessellate_total` 28-33 %, `2_mesh_share` ~9 %;
the frame loop (`4_*`) under 1 %. 391 bodies share 246 meshes. Re-measure before quoting.

The straight-line program of each linkage (`spiderpig/linkage/engine.py`) is compiled once
per process; a leg's phase is a time shift (before `19e020e` the symbolic solve ran per leg
per frame). Don't reintroduce per-leg or per-frame solves.
The profiler class is `_Profiler` in `spiderpig/bake.py`; `spiderpig/tools/profiler.py` is
its generalised copy behind `spiderpig build --profile` (`build_profile.STAGES`). Add a
bracket with `prof.timed("label")`, `prof.bump(...)`, `prof.set_metric(...)`.

## Repository map

Everything Python is the `spiderpig` package (`pyproject.toml`, hatchling; `uv sync`
installs it editable). Engine layers, enforced by `lint-imports` (no exceptions): linkage
→ stack → construction → fabricate → (sim | stages) → (bake | build | strength) → api →
(mcp | server | cli).

| path | role |
|---|---|
| `spiderpig/linkage/` | symbolic core: `engine.py` (`Linkage`, compiled once; registry `get`; `LegSolution`), `checks.py`, `assembly.py` (leg template, `build_module_template`) |
| `spiderpig/linkages/` | one module per linkage family, auto-registered; `mechanisms.py` the one-sided mechanisms |
| `spiderpig/mechanism.py` | `Body` / `Joint` / `Mechanism`; `MechanismTemplate` for batched sampling. Planar: z = 0 |
| `spiderpig/stack/` | the layer planner: `geometry.py`, `topology.py`, `plan.py` (`StackSpec`, `StackPlan`), `search.py` (`StackProblem`), `plan_z.py` (`finalize`), `verify.py` |
| `spiderpig/stack_pool.py`, `stack_symmetry.py` | the planner's opt-in workers and mirrored-leg symmetry |
| `spiderpig/construction/` | one group per functional part (`base.py` the contract): `axle.py`, `crank/` (`BoltCrank`: `bolt.py`, `hex.py`, `web.py`, `capacity.py`, `plates.py`), `route.py` (the crank's router), `pivots/` (`standoff.py`, `chicago.py`), `plates.py`, `chassis.py`, `deck.py`, `robot.py`, `assembly.py` (`ROBOT_ORDER`, the hooks' records), `underside.py`, `wobble.py`, `contract.py`, `envelope.py` |
| `spiderpig/servos/` | `ServoSpec`, the drive group (`mount.py`), models and CAD cache |
| `spiderpig/hardware/` | catalog, sources, the one screw table (`fasteners.py`), mass (`mass.py`), BOM (`bom.py`, `shims.py`), `order.py` (`ORDER.md`) |
| `spiderpig/config.py` | `BuildConfig`, defaults per linkage, removed keys, CLI args, query parsing |
| `spiderpig/materials.py` | sheets, thinnest sheet per role, gap options |
| `spiderpig/manufacture.py` | the cut rules (SendCutSend / Ponoko) |
| `spiderpig/fabricate.py` | `design_side()`, `fabricate_side()`, `fabricate()` |
| `spiderpig/fabcache.py`, `keys.py`, `uptodate.py` | the fabrication cache, incremental cache keys, the build skip |
| `spiderpig/shapes.py`, `rounding.py` | build123d primitives; tie-stable reported numbers |
| `spiderpig/mesh.py` | `mesh_part` + `read_meshes` (the bake), `tessellate` / `tessellate_many` (the MJCF's hulls), `export_stl` |
| `spiderpig/layout.py` | DXF sheets and per-part DXFs, kerf per sheet, `fidelity` |
| `spiderpig/workers.py` | `submit`: package functions in fresh processes (OCP holds the GIL) |
| `spiderpig/strength.py` | joint and link safety factors at the sim's loads |
| `spiderpig/walk.py` | quasi-static walk model (`viewer/src/drive/model.ts` is its twin) |
| `spiderpig/sim/` | MuJoCo: `mjcf.py`, `run.py`, `live.py` (`/ws/sim`), `loads.py` |
| `spiderpig/bake.py`, `build.py` | the `.glb` bake; STEP/STL/DXF/BOM/ORDER.md |
| `spiderpig/guide/`, `labels.py` | the assembly guide (`docs/agentlib/GUIDE.md`); the part labels (SP8-0.7, LK75x27, M3-BH-8) the print and cut files share |
| `spiderpig/explain.py`, `recommend.py` | stage verdicts; fixes checked by re-running the stage |
| `spiderpig/spec.py`, `api/`, `failure.py`, `verify.py`, `design.py`, `store.py` | the agent surface (`docs/agentlib/API.md`) |
| `spiderpig/stages/` | under bake/build and the API (which re-exports them): `resolve.py` (`resolve`, `spec_of`), `records.py`, `reports.py`, `planning.py` (`check`, `plan`, `plan_config`) |
| `spiderpig/mcp/` | the MCP server over the API (`jobs.py`, `outputs.py`, `guide.md`) |
| `spiderpig/server/app.py`, `view.py` | the viewer's FastAPI app; `spiderpig view` |
| `spiderpig/cli.py`, `tools/` | the console script; audit, tune, sim, export, report, dev, remote |
| `viewer/` | the Vite + TypeScript three.js client (`src/drive/`: drive mode) |
| `tests/` | `conftest.py` (factories), `cache.py`, `_ctx.py` (seams), `tiers.py`, `gate/`, `doc_check.py`, `scorecard.py` |

## Pipeline contract

1. **Symbolic**: a `Linkage`'s steps (`spiderpig/linkages/*.py`), sympy over earlier points,
   `t` and the params; `O` the crank centre, y up, feet lowest.
2. **Compiled**: `Linkage.compiled` once; `LegSolution(orientation, phase).evaluate(ts)`.
3. **Template**: `MechanismTemplate`, one *side* (a leg module: single, double, decker, quad).
4. **Rationalized**: `fabricate.design_side(tmpl, config)`: groups in dependency order
   (drive, crank, axles, links, frame), their claims, the layer plan.
5. **Fabricated**: `fabricate(tmpl, config, t)`: each group builds inside its claims;
   plates are cut last; the robot mirrors the side and adds the chassis.
6. **Serialized**: `spiderpig/build.py` or `spiderpig/bake.py`.

Every stage says what fails (`spiderpig explain` prints them): `AssemblyError`,
`OutputError`, `ConstructionError`, `ClearanceError` (`NoCrankPoint`), `PlanError` with
blockers, and `recommendations` (`stack.Recommendation`) only once re-running passes. A new
construction or claim raises with a reason (`Unbuildable`) and declares its keep-outs.
Correct by construction: claims of different groups never meet over the cycle, and
`construction.contract` checks each part lies inside its own claims.

To add: a **construction**, `dims(ctx)` (validation, its claims' radii) and
`realize(group, build)`, registered in `spiderpig/construction/__init__.py`, then the
contract tests; a **group**, a `construction.base.Group` subclass (`claims`, `realize`,
`keepouts` / `interface`, `cuts`) appended to `construction.GROUP_FACTORIES` in dependency
order; a **leg module**, a `linkage.Module` in `linkage.MODULES` or a linkage's `modules`;
a **linkage**, a module in `spiderpig/linkages/` (`tests/test_linkage.py` checks it).

## The planner

`stack.StackProblem.solve()` finds the thinnest stack and the cheapest crank route in it:
static facts first (keep-outs, the crank's facts); then a search per stack size, fewest
layers first (`_Search`: forward checking, the router as a sub-check at every node,
backjumping, learned nogoods, branch and bound); the exact route of a complete layering
(`CrankRouter.route`: chains along a point, a stock standoff each, `JointRules`); then
`verify_plan`. Effort: a short search per size until one plans; if none did, the sizes
left open get `max_nodes`; then each thinner size gets `max_nodes // 2` to prove it out,
then a cheaper route. Budgets (`StackSpec.quick_nodes`, `max_nodes`, `max_total_nodes`,
`max_seconds`) bound effort, never validity: a plan found unproven says so
(`StackPlan.optimal`, `proof`). Sizes up to `StackSpec.max_top`. `tests/brute.py` is the
independent brute force. Details: `docs/ARCHITECTURE.md` §5, `spiderpig/stack/search.py`.

Physical rules the claims encode: a crank crosses a rider's layer only along its crankpin
(a built-up crankshaft, webs either side of each rider); layers 0 and `top` hold only the
frame plates and what sits in their holes; every link on an axle is held in its layer on
both sides.

## House rules

- Don't add `print` statements to the bake path — use the `bake_gltf` logger. Keep the
  profiler's stage keys stable (scripts parse them).
- Don't put Z into joint poses. Z is the stack plan's job.
- How a part goes on is its construction's `assembly` hook (`construction/assembly.py`);
  a new construction adds one or gets generic steps. The hooks stay out of the fab key.
- A group builds only inside its own claims; keep `check_side` at `[]`.
- A change that alters parts: run the identity gate before and after (baseline
  `next-ba41c10`, `docs/agentlib/TESTING.md`), list each intended diff, leave `mise run
  audit` green, and regenerate `docs/agentlib/DESIGNS.md` with a new baseline.
- Docs: no layer counts or heights outside DESIGNS.md; a dated decision goes to
  DECISIONS.md, not here; `mise run doc-check -- --strict` must pass.
- One density table, one OCCT mass query: `spiderpig/hardware/mass.py`. One screw table:
  `spiderpig/hardware/fasteners.py`. One shim loop: `bom.stack`.
- Caches are keyed, never trusted blindly: the fabrication cache and the test cache follow
  `spiderpig/keys.py`; a plan from the store is re-made and `verify_plan`ed. Never key a
  cache on less than what shapes its output.
- Tests: read the session factories (`design` / `side` / `robot` in `tests/conftest.py`),
  never mutate them. Unit tests go through seams (`tests/_ctx.py`); a test marked
  `no_fabricate` fails if it fabricates. Every test over ~5 s is `slow`; a heavy
  parametrized check keeps one cheap case quick (`tests/tiers.py` `quick()`). xdist-safe:
  write under `tmp_path`, free ports only.
- Iterate with the module tiers; before handing back run the gate on product edits and the
  full suite (`mise run remote-test`, or locally `pytest -m 'not e2e' -n 12`).

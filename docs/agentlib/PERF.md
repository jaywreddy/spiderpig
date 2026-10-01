# Where the time went, and what was stupid — 2026-10-01

A follow-up to [TIMING.md](TIMING.md): every slow step of the known path profiled, what it
does, the stupid thing (or "nothing stupid"), the fix where the result stays identical, the
saving measured after it, and the risk. A change that would alter an output, a plan or a
proof, or trade optimality for speed, is a design decision and was not made: it is listed
with its number. Two halves, done apart and side by side on one 4-core machine:
rationalization and start-up (the planner, the warm path, the cards, imports, the CLI by
options), then the fabrication side (`export`, `verify`, `build`, the bake, the stage cache).

## Rationalization and start-up

**How it was measured.** *Base* is `38158e6` (the timing study), *final* is `389ab8f`
(this section's last code commit), each a `git archive` run with its own `PYTHONPATH`.
The machine is the study's (4 cores, Python 3.12); the fabrication side's agent was
running beside it (load 3-4), so every planner row below ran base and final **side by
side** (two processes at once, equal contention) and every start-up row ran one process
at a time. *Node-exact* means the planner with its node budgets and no wall-clock
deadline (`StackSpec.max_seconds = inf`): both trees explore exactly the same nodes and
differ only in seconds. The scripts and logs are in the session's scratchpad (`rs/`:
`gate.py`, `api_gate.py`, `cli_gate.sh`, `measure.sh`, `measure2.sh`, `phases.py`).

### The commits

| commit | what | verdict |
|---|---|---|
| `0749384` | planner: the router's cheap relaxation before the exact route, the route memoised, the route DP on bitmasks | kept (the gate below) |
| `93bc2be` | planner: the axes' connected components by union-find, not scipy.sparse | kept (same partition, same order) |
| `8ba20bf` | warm path: one sampling per module template, no leg hint where nothing is searched | kept |
| `6f9edc1` | cards: `describe()` and the guide's scale parameters cached in the store per source version | kept; its `Store` methods moved out of store.py (the fabrication side's file) by `1b57ed9` |
| `33f141a` | cli: explain and audit by options go through the store and reuse its plan; audit takes `--proportion` | kept; `389ab8f` fixes which design they resolve |
| `1b57ed9` | cards: the per-code cache's file helpers in api.py, store.py as it was | new, a fix of `6f9edc1` |
| `389ab8f` | cli: explain and audit by options resolve the linkage's own design, so they share its plan | new, a fix of `33f141a` |

Nothing was reverted. A first identity gate imported the worktree whatever `PYTHONPATH`
said (a `sys.path.insert` in the script), so its "before" was the new code; the gate
below was run again from scratch.

### 1. Rationalizing a known design: `fabricate.design_side`

**What it does.** `side_problem` (the topology sampled at 1440 crank angles, the groups,
their claims and interfaces, the body's underside, the crank router with its static
facts; and, for a multi-leg module, the *leg hint*: the single module's whole
`design_side`), then `static_stage`, then `StackProblem.solve`: a quick search per stack
size until one finds a plan, then the proof (every thinner size with the full budget),
then the cheapest crank route in the size found (branch and bound). Each node is a layer
assignment with forward checking plus the crank router's sub-check (`CrankRouter.check`:
a bit-mask reachability relaxation, then the exact route DP); `verify_plan` checks every
plan found.

**Where the time went (base).** Strider double, shin 16, unit 6.3, XL330, plywood (the
study's final design), 27.4 s, most of it proof: 19 and 18 layers ruled out (15453
nodes) and no cheaper crank route in 20 layers (18222 nodes of branch and bound). Per
node (≈ 0.8 ms): the router 0.4-0.5 ms (`check` 47179 calls; the exact route DP's
`chains` 765k calls, 21 s under cProfile), the layer assignment 0.3 ms (`assign`,
`add`, `spans`, `undo`, `cut`). TrotBot quad and the sixbar quad run into the 60 s deadline with the
router at 70-75 % of their time (TrotBot's DP is larger: up to 40 layers below the hub
and its detour points, 90 chains per route).

**The stupid things, and the fixes** (`0749384`, `93bc2be`):

1. `check` ran the exact route DP first and the reachability relaxation (`_forward`, a
   hundred times cheaper) second, although a dead relaxation decides the node alone and
   a route found always passes it (every DP transition is one of the relaxation's).
   Now the relaxation first. It is the same answer: `check` always gets a partial view
   (`open` a dict), so the DP never words a dead end there, and the bound needs a route.
   Exact routes skipped: Strider 7.5 %, sixbar quad 42 %, TrotBot quad 14.5 %.
2. A backtracking search asks for the same layering again and again (TrotBot quad: 44 %
   of route calls repeat). `route` remembers the DP's answer per everything it reads
   (the hub's bottom, the riders' layers, the pieces blocked per layer, the run layers
   unplaced riders may still take, which joint rule `_unbuildable` has relaxed) and
   applies the branch-and-bound cut afterwards; plain tuples only, so the collector has
   nothing to walk (with `Run` objects in it the planner's full collections of the
   sympy/OCP heap went from 2 to 15).
3. `hub_bottom` recomputed `hub_layers` on every `_prepare` (90k times a plan): one per
   stack size now.
4. The DP read posts, webs and riders through closures and dict lookups per layer, built
   a `Run` for every candidate chain end and scanned every state once per layer: now
   bitmasks per point, a run tuple only where a web ends it, states kept per layer in
   the order first reached (ties broken as before, so the route chosen among equals is
   the same).
5. `group_axes` built a scipy sparse matrix for ~50 joints and compared every pair over
   all 1440 samples: union-find, and a pair apart at the first sample is apart (46 ms →
   3 ms per topology; the planner no longer imports scipy.sparse, though build123d
   still pulls scipy in).

**Measured, final against base, side by side:**

| design | base | final | |
|---|---|---|---|
| Strider double, round 4 (shin 16, unit 6.3, XL330, plywood) | 27.4 s | **21.4 s** | −22 %, same plan, proof (15453 + 18222 nodes) |
| Strider double, candidate 2 (shin 16, plywood) | 42.8 s | **30.6 s** | −29 %, same plan and proof |
| TrotBot quad, node-exact | 121.5 s | **86.6 s** | −29 %, same plan and proof |
| sixbar quad, node-exact | 106.5 s | **78.0 s** | −27 %, same plan and proof |
| TrotBot quad / sixbar quad, as shipped (60 s deadline) | 61.2 / 60.7 s | 61.1 / 60.5 s | the same stack and route cost; the faster search gets further before the deadline, so the proofs' node counts and tallies differ (sixbar: 10001 route nodes instead of 8781) |
| Strider double, defaults | 1.85 s | 1.66 s | |
| Klann quad | 0.89 s | 0.90 s | a 66-node search: nothing to win |

The router is still 40 % (Strider) to 50 % (TrotBot) of a node. **Tried and dropped:**
a cross-call memo of the DP's chains (53 % hits on Strider) saves 1.8 s of the 21 s
but adds 1.25 s of full collections (6 instead of 3); `gc.freeze()` around the search
makes the collector run *more* often (frozen objects leave the long-lived count it
paces itself by: 8-22 full collections instead of 3-6). Neither kept.

**Where the time is now** (`phases.py`, final): the round-4 Strider has a 21-layer plan at
4.4 s and the 20-layer one it returns (7 features) at 6.4 s; proving 19 and 18 layers
hold none and no cheaper route exists in 20 takes the other 14.5 s. Candidate 2: a
21-layer plan at 4.3 s, the one it returns at 7.0 s, then 23.6 s failing to rule out 20
layers and a cheaper route within the node budgets. TrotBot quad: 41 layers at 22 s, the
36 it returns at 38.6 s, then the deadline. Sixbar quad: 26 layers at 15 s, the 24 it
returns at 34 s, then the deadline. See "not done".

**Risk.** Low: the gate (below) gives the same layers, route, cost, `optimal` and proof
text for every case; `tests/test_stack.py` compares the planner's optimum with
`tests/brute.py` and passes. The memo is bounded (100k routes, then dropped) and dies
with the problem.

### 2. The warm path: a stored plan re-made (`api.plan` with a store)

**What it does.** `_reuse_plan` → `_remake_plan`: the template (the program compiled,
the loops checked), `side_problem` (topology, groups, claims, router), `static_stage`,
`StackProblem.plan` of the stored layers and route, `verify_plan` on the plan's own
sampling.

**The stupid things.** `side_problem`'s default `hint=True` planned the *single* module
(a full `design_side`) for a leg hint only the search reads, in `api.check` and in the
re-make, neither of which searches; and `check`, `plan` and the re-make each sampled the
same template's topology (1440 angles) and measured the same link distances again.

**Fix** (`8ba20bf`). `hint=False` where nothing is searched; `topology_from_template`
samples a module template once per process (16 kept, keyed by the `meta` that made it:
linkage, module, phases, proportions) and every caller gets its own `Topology` over its
own `Geometry` (a servo's screws and a plan's detours go into its own points and table)
sharing one distance table for the template's own points.

**Measured** (final against base):

| | base | final |
|---|---|---|
| `api.check`, a fresh process, Klann quad / round-4 Strider | 0.62 / 0.86 s | 0.41 / 0.46 s |
| `api.plan` re-made from the store, a fresh process, Klann quad | 0.56-0.66 s | 0.45 s |
| the same, round-4 Strider | 0.85-0.93 s | 0.49-0.55 s |
| the same, the second and later designs of one template in one process (the MCP server) | 0.28-0.41 s | **0.023-0.028 s** |

In a fresh process what is left is the program's compile (sympy steps + lambdify, ≈ 0.3
s for Strider, once per linkage per process), the topology (0.15 s, once per template)
and `verify_plan`; in a running server it is the claims and `verify_plan`: 25 ms.

**Risk.** Low: the hint only orders the search, and neither caller searches; the shared
table holds the very floats the per-design one did (shared only for the identical
point arrays); `servos/mount.py` adds its screws to a private dict. A template without
`meta` is never cached. The API gate's `check` reports (warnings included) are the same.

### 3. The cards: `describe` (17 walker cards, 13.4-13.7 s every session)

**What it does per card.** `lk.check()` (every loop over 720 samples), the foot path,
the sensitivity table (one re-solve per parameter), and for every module the walk model
at the default phases: `support()` over every triple of feet per sample (a 16-foot quad:
C(16,3) = 560 triangles × 361 samples, 78 MB tensors) and `_margin` over every pair
(`trotbot_toe` alone 6-7 s).

**The stupid thing.** Nothing inside (the quasi-static model is what it is); the stupid
part is computing every session an answer that depends on the code alone; and the
guide's linkages table compiled every linkage's program for `scale_params` (2.5 s of the
2.7 s guide read) on every server start.

**Fix** (`6f9edc1`, `1b57ed9`). With a store (the project's by default) a card is kept in
`<store>/cache/<source version>/cards/<key>.json` and the scale parameters in
`.../scale_params.json`; `store=None` computes them. The key, `design.source_version()`,
is the package version plus a hash of every Python source of the package (24 ms once per
process), so any edit of the checkout starts the cache afresh. `describe` returns JSON
values on both paths (lists for tuples, `None` for non-finite floats: what the MCP
already sent).

**Measured** (an MCP session: connect, the guide, `list_tools`, `describe` × 17):

| | base | final, first session (fresh store) | final, every later session |
|---|---|---|---|
| guide | 2.45-2.52 s | 2.42 s | **0.021 s** |
| `describe` × 17 | 13.4-13.7 s | 13.4 s | **0.10 s** |
| the session | 21.5-21.9 s | 21.6 s | **6.1 s** (4.1-4.6 s of it the connect) |

The guide, the tools and the 17 cards are identical across base, final cold and final
warm (digests; the guide differs only in the store path it names).

### 4. Imports and start-up

**`import spiderpig.api`: 3.0-3.3 s, base and final alike.** `-X importtime`: build123d
2.6-2.9 s (OCP 1.25-1.5 s, IPython 0.42-0.53 s through `build123d.topology.shape_core`'s
unconditional `from IPython.lib.pretty import ...`, scipy.optimize 0.46 s, ezdxf 0.19
s), sympy 0.36 s, spiderpig's own modules 0.18 s. build123d is imported at module level
by `shapes.py`, `construction/{crank,printed,robot,contract}.py`,
`construction/pivots/common.py`, `servos/{model,mount}.py` and `layout.py`, and
`spiderpig.config` imports `construction` (`base.py` → `shapes.py`), so every stage pays
it, `--help` included, although `check`, `plan`, `walk` and the cards never touch a
solid; the rest of the engine imports in 0.56-0.59 s. **MCP connect 4.1-4.6 s**: that import plus the MCP SDK's own (0.8-1.0
s) and `make_server` (0.14 s).

**Nothing changed here, and why.** Every remaining start-up cost sits in files this
half does not own or in third-party code: making build123d lazy means moving its imports
inside the functions that build (`shapes.py`, the constructions, `servos/*`, `layout.py`)
and making the registries importable without them; the MCP server could answer
`initialize` before the engine is imported only if `spiderpig.spec` (which imports
`config`, hence build123d) and the guide's tables (`Params`, the materials) were light.
IPython is build123d's own import; keeping it out would take a stub package in
`sys.modules` while build123d imports, which leaves `IPython.lib` half-initialised for
anyone who imports IPython afterwards: not done. The program compile (sympy steps +
lambdify: 0.1-0.35 s per linkage, once per process) was measured and left (below).
What changed for start-up is what each session pays *after* the import: the guide and
the cards (section 3).

### 5. The CLI by options (`explain`, `audit`; the plan solved per command)

**The stupid thing.** `spiderpig explain/audit --linkage ... --pin ...` called
`design_side` in every process (27 s for the round-4 design) while the store held the
plan; `audit` could not take `--proportion`, so the study's final design was audited at
the default proportions.

**Fix** (`33f141a`, `389ab8f`). `api.plan_config(config, store)`: the options resolved as
a design, its stored plan re-made and verified when there is one, else solved and
recorded; the engine's cache told (`fabricate.remember`) so the build that follows plans
nothing again. `explain` and `audit` take `--store`; `audit` takes `--module`,
`--phases`, `--proportion`; `explain_config` of a designed side rebuilds nothing
(`SideDesign.facts`). `33f141a` resolved the options as given, so `explain` (one side,
`sides: 1`) and `audit` / `export` / `view` (the robot, `sides: 2`) were two designs and
each solved the plan once; `389ab8f` resolves the design the linkage's kind builds (the
one `export` / `view --linkage` make), so they share it.

**Measured** (round-4 Strider, a fresh process each):

| | base | final, no stored plan | final, stored |
|---|---|---|---|
| `spiderpig explain` | 32.8 s | 25.2 s | **4.8 s** (3.1-3.4 s of it the import) |
| the robot's plan for the same options after `explain` (what `audit` / `export` / `view` then use) | a 27 s solve | | **0.48 s** |

`explain`'s text is byte-identical base vs final cold vs final warm for seven designs
(Klann single and quad, the round-4 Strider, TrotBot heel at unit 7 (a static failure
with checked recommendations), Hoecken, Jansen double on bolts, the five-bar (a drive
failure)). `audit` of Klann single and double: the same output and `audit.json` (times
aside) as base; the full `spiderpig audit` (four Klann modules) is OK. The round-4 design,
refused by `audit` in the study, now audits as built (`--proportion shin=16 --proportion
unit=6.3 --servo xl330_m288 --sheet plywood_3mm`): 161 parts, 2 DXF sheets, 12 BOM items,
OK, 103 s with the plan from the store.
`spiderpig build` (build.py, the fabrication side's file) still calls `design_side`:
`design = api.plan_config(config, store)` in its place gives it the same reuse.

### Before / after

| step | base | final |
|---|---|---|
| `design_side`, round-4 Strider (the study's final) | 27.4 s | 21.4 s |
| `design_side`, candidate 2 | 42.8 s | 30.6 s |
| `design_side`, TrotBot quad, node-exact | 121.5 s | 86.6 s |
| `design_side`, sixbar quad, node-exact | 106.5 s | 78.0 s |
| `api.check`, a fresh process (Klann quad / round 4) | 0.62 / 0.86 s | 0.41 / 0.46 s |
| a stored plan re-made, a fresh process (Klann quad / round 4) | 0.56-0.66 / 0.85-0.93 s | 0.45 / 0.49-0.55 s |
| a stored plan re-made in a running server | 0.28-0.41 s | 0.023-0.028 s |
| `describe` × 17 | 13.4-13.7 s | 13.4 s once per code version, then 0.10 s |
| the guide | 2.45-2.52 s | 2.42 s once per code version, then 0.021 s |
| `import spiderpig.api` | 3.0-3.3 s | 3.1-3.2 s |
| MCP connect | 4.1-4.2 s | 4.1-4.6 s |
| MCP session (connect, guide, tools, 17 cards) | 21.5-21.9 s | 21.6 s first, 6.1 s after |
| `spiderpig explain` by options (round 4) | 32.8 s | 25.2 s cold, 4.8 s with the stored plan |
| `audit` / `export` / `view` by options after `explain` (round 4) | + 27 s each | + 0.5 s |

### Identity gate

- **Plans.** Every linkage × module (79 cases) and eight variants (the round-4 design,
  candidate 2, Klann quad on bolts and on rods, Klann single on bearings, Strider double
  on bolts, TrotBot heel at unit 7, an unknown servo), node-exact, base against final:
  layers, top, route, cost, `optimal`, proof text, `describe()` of the plan, clearances,
  ground clearance, crank points, and every error's text and recommendations. **83 of 87
  identical in every field**; the four `PlanError` cases (Klann quad and Strider double on
  bolts, TrotBot heel and toe double) differ only in the seconds their message prints
  ("after 60001 search steps in 163 s" against "359 s"), and the two TrotBot ones also in
  how far the recommendation checks got in their 60 s of wall clock ("a scale of the
  linkage from unit 18 up" against "17.5 up": the faster planner checked one more scale
  before that deadline). `tests/test_stack.py`'s brute-force comparisons pass.
- **API.** Every linkage's card (34, computed), and `check`, `plan` and `explain` of
  five designs (Klann quad, Strider double, TrotBot heel at unit 7, Klann quad on printed
  pins, Jansen double on bolts): identical base against final.
- **MCP.** The guide, the tool list and the 17 cards: identical base, final cold, final
  warm.
- **CLI.** `explain` by options, seven designs: byte-identical base, final cold, final
  warm; `audit` of Klann single and double: the same output and `audit.json`.

### Not done, with the numbers

- **The plan first, the proof optional.** The round-4 Strider has the plan it returns at
  6.4 s of 21.4 s; the other 14.5 s prove that 18 and 19 layers hold none and that no
  cheaper crank route exists in 20 (the route proof's bound is weak: a run layer an
  unplaced rider may still take costs nothing, so few nodes are cut). Candidate 2: 7.0 s
  of 30.6 s. An agent comparing candidates would get each plan in a third of the time
  and the proof as a second call, or never. That changes what `plan` returns (`optimal`,
  `proof`) and the API: a design decision. A tighter bound for the route proof would
  change which nodes are explored, hence the proof's node counts and possibly the route
  chosen among equals.
- **The route DP is still 40-50 % of a node.** A cross-call memo of its chains and
  `gc.freeze()` were measured and dropped (section 1). What is left is constant-factor
  work in pure Python (≈ 0.4 ms a node); halving it means a different representation
  (the DP in numpy or a compiled extension), not a fix.
- **`_unbuildable`'s three relaxed DPs** word every dead end the joint rules close (the
  proof's "forced it taller" sentence and the `PlanError` tally depend on the wording, so
  it cannot be skipped); the route memo now serves the relaxed runs on repeats.
- **Lazy build123d** (`shapes.py`, the constructions, `servos/*`, `layout.py`: the
  fabrication side's files): `import spiderpig.api` 3.1 s → ≈ 0.6 s (numpy, sympy, the
  linkages, spiderpig's own 0.18 s), every CLI process and MCP worker −2.5 s, `spiderpig
  explain --help` 4.6 s → ≈ 1 s; `check`, `plan`, `walk` and the cards never need a solid.
  The MCP connect would then be ≈ 1.5 s too (the SDK 0.8-1.0 s + the linkages), the
  engine's import overlapping the agent's first think.
- **IPython** 0.42-0.53 s per process: build123d's unconditional import; the fix belongs
  upstream (import it inside `_repr_pretty_`). A `sys.modules` stub was not made (above).
- **The program compile** (sympy steps + lambdify, `cse=True`): Strider 0.33 s, TrotBot
  toe 0.26 s, Klann 0.1 s, once per linkage per process: half of a warm re-make in a
  fresh process. A disk cache of the generated source (keyed by the source version and
  sympy's) would remove it; `check_steps` also reads each step's free symbols, so those
  would be cached too.
- **The cards' first session** (13.4 s per code version): a convex-hull `support()`
  would make `trotbot_toe` well under a second (and `walk` on every 16-foot design), but
  the hull's lower facets are the lowest supporting plane found differently, so the cards'
  numbers could move in the last digit at exact ties; or the cards could ship with the
  wheel, computed at release.
- **`verify_plan` on reload** (0.03-0.13 s): the independent check of a stored plan;
  kept, it is the guarantee.
- **Planning candidates in parallel** (TIMING.md's third recommendation: `plan` /
  `verify quick` as pool jobs, three candidates on three cores): scheduling, not a
  per-call change; it belongs to the MCP's job design.

## Fabrication side

**Designs.** The round-4 Strider `double` of TESTDRIVE.md (`shin 16`, `unit 6.3`, XL330,
plywood: 20 layers, 161 parts) and the default Klann quad (173 parts).

**Method.** Every number is a fresh process (`import spiderpig.api`, `resolve`, `plan`
from the store, then the one step timed), the base commit `38158e6` and this branch back
to back on the same 4-core VM, with the plan already in the store; an export reloads the
build from its STEP files first (as the MCP worker does: 7 s for round 4, 5.5 s for
Klann, timed apart and not in the export's number). Another agent's runs shared the
machine (load 3-6), so wall times carry a few seconds of noise; CPU seconds are given
beside them. The profiles are a wall-clock stack sampler on the main thread (2 ms).

**The identity gate.** The seven formats exported by the base and by this branch (the
four changes below), both designs, a fresh build in one process: `bom.json` and `bom.csv` (text),
`print/parts.csv` and the laser `parts.csv`, every DXF's `ENTITIES`, the glb's BIN chunk
sha256 and its JSON chunk, the MJCF and its metadata, the whole-robot STL bytes and the
STEP's per-solid volumes. **All identical, both designs**, with the plan re-made from the
store on this branch (the base's bake solved it in-process). Also on the reload path
(the build from STEP, the MCP's): every format identical except the round-4 grouping,
where the base was wrong (section 1). The print STLs are the one thing that moved, and
section 1 shows why that is the fix, not a regression.

### Before / after

Fresh process, the plan in the store, seconds wall (CPU).

| step | round 4 before | round 4 after | Klann before | Klann after | what changed |
|---|---|---|---|---|---|
| `build`, a fresh fabrication | 23.7 (24.2) | **17.1** (18.9) | 14.8 (18.7) | 13.2 (19.3) | the servo record (3) |
| `build` reloaded from STEP | 7.9 | 7.0 | 5.6 | 5.5 | – (noise) |
| `verify standard`, cold | 64.2 (81.3) | **50.7** (74.0) | 53.3 (63.3) | 47.6 (72.3) | the servo record; the rest is what the checks cost (4) |
| `verify standard` after a `quick` | 48.7 (74.3) | **0.0** | 40.9 (52.8) | **0.0** | one report per level (4) |
| `export` print | 116.6 (160.5) | **26.4** (40.5) | 78.8 (137.7) | **27.2** (49.2) | the grouping (1) |
| `export` bom | 117.1 (158.4) | **25.7** (38.9) | 80.4 (140.2) | **25.8** (47.0) | the grouping (1) |
| `export` glb | 59.0 (71.7) | **24.4** (34.0) | 19.5 (30.3) | 18.8 (30.2) | no plan, no second fabrication (2) |
| `export` mjcf | 49.6 (58.0) | **17.4** (25.4) | 13.1 (19.9) | 13.0 (20.0) | the same (2) |
| `export` step / stl / dxf | 3.9 / 14.2 / 2.1 | 3.9 / 11.4 / 1.5 | 3.3 / 4.9 / 1.6 | 2.9 / 5.2 / 1.5 | – (noise) |
| **`export`, the seven formats in one process** | **185.2** (313.2) | **75.2** (117.5) | **121.8** (205.9) | **60.1** (105.4) | all of it |
| `spiderpig bake` (no design: plans from its options) | 60.3 (64.8) | 50.3 (64.1) | 19.4 (29.7) | 24.6 (28.3) | the servo record (5) |

The MCP's `export` of the seven formats, TIMING.md's 184.5 s, is the 185.2 s row plus the
reload: **192 → 82 s** for round 4, **128 → 66 s** for Klann. Klann's glb and mjcf barely
move: its plan takes 1.4 s and its servo model (STS3215, one solid) strips in 0.1 s, so
what each did twice was cheap; they share one fabrication when asked together.

### 1. `export` print and bom: the grouping — 117 → 26 s each

**What it did.** Both formats group the made parts by shape (`hardware/bom.py`
`group_made`: one row and one STL per different part). Every body is compared with every
group's reference by `congruent(a, b)`: volume, area, principal moments, then a proof that
`b` is `a` moved (or mirrored).

**Profile top (base, print, 100.6 s).** `congruent` 99.9 s = `_proper_fit` 88.2 s, of
which `Shape.__sub__` (OCCT cuts) 84.5 s; `volume` 10.3 s, `area` 1.9 s, `_frame` 2.0 s.

**The stupid things.**

1. Every call re-measured both parts: a `Part`'s `volume` and `area` are fresh `GProp`
   integrations, so the made parts' invariants were integrated again for every pair
   (14 s).
2. The proof tried up to eight matchings of the two principal frames in order and proved
   each with **two** cuts (`moved − b`, `b − moved`), before looking at anything cheap:
   a wrong sign flip costs two booleans of near-coincident solids, the slowest case for
   OCCT. For round 4's 125 made parts that is 750 comparisons and 368 cuts at 0.25 s
   (390 on the STEP-reloaded parts). The mirror image of the reference was rebuilt and
   re-measured for every candidate.
3. The cuts ran on the design's live parts, and OCCT widens its arguments' tolerances in
   place. So what a part was later meshed as depended on how often it had been compared:
   the base's print STLs of the parts its cuts touched (3 of 14 for round 4, 4 of 17 for
   Klann) are not the STLs of the parts as built, and a second `group_made` in the same
   process changes them again. On parts reloaded from STEP (the MCP's path) the cuts came
   out wrong: the base listed the round-4 pillar segment `pillar_J2_leg0_seg1` as three
   rows (2 + 1 + 1) where a fresh build has one row of 4: the right-side twins' proof
   came back with ~1 270 mm³ of difference for the right matching (the frames matched
   to 1e-8 mm).

**The fix** (`bom: measure each part once, ...`). Each part is measured once (`_Sig`:
volume, area, principal frame, surface centroid); a group's reference is mirrored once,
lazily; a frame matching is tried only when it carries the surface centroid onto the
other part's within 1e-3 mm (a wrong matching misses by 0.72 mm on the pillar, a right one
by 1e-8 after a STEP round trip); the proof is one `BRepAlgoAPI_Common` with
`SetNonDestructive`: `va + vb − 2·shared < tol`, the same tolerance, "same" before
"mirror" as before. The same 750 comparisons now run 103 booleans, and the parts are
left as they were (`test_bom`: their tolerances after two groupings equal those before;
it fails on the base).

**Measured.** print 116.6 → 26.4 s, bom 117.1 → 25.7 s (round 4); 78.8 → 27.2 and
80.4 → 25.8 (Klann).

**What remains** (after, print, 26.2 s): the proofs, 21.7 s (103 booleans, one per
match, ~0.2 s each); the invariants once per part, 3.1 s; the STLs, 0.8 s.

**Outputs.** Fresh build: the groups, their order, references and mirror counts, and
every BOM and CSV byte identical on both designs. The print STLs: exporting the groups
taken from `parts.csv` with no comparison at all gives the same STL bytes on the base and
on this branch for every file, and this branch's grouped export equals them for every
file; the base's grouped export differs exactly for the parts its cuts touched. Reload
path: this branch's grouping equals the fresh build's (22 rows); the base's did not
(24).

**Risk.** Low. The proof is still a boolean, on the same tolerance. The centroid filter
only skips a matching whose proof would fail: a motion that maps one part onto the other
within `tol` maps the surface centroid along with it; it could only differ for two parts
that differ by a feature smaller than `tol` (1e-4 of the volume) and still shift the
surface centroid by more than 1e-3 mm, which no construction here makes.

### 2. `export` glb and mjcf: a plan and a fabrication per format — 59 → 24 s, 50 → 17 s

**What it did.** `_export_files` called `bake_gltf(cfg)` and `build_mjcf(cfg)`. Each went
back to the *config*: `fabricate(template_for(cfg), cfg, 0)` → `design_side`, which in a
fresh process (the MCP worker, the CLI) solved the plan again, then fabricated the robot,
once per format.

**Profile top (base, glb, 56.9 s).** `_reference` 43.9 s = `design_side` (the plan
solve) 29.2 s + `fabricate_side` 13.0 s (the servo strip 4.6 of it); tessellation 9.5 s.
The mjcf: `fabricated` 45.1 s (the plan 29.9) of 52.2 s.

**The stupid thing.** The design already held its plan (re-made from the store in 1 s);
the two formats asked the planner for it again (27 s for round 4) and fabricated the same
robot at the same angle twice.

**The fix** (`export: fabricate the walker at the bake's angle once, ...`). The export
fabricates the walker at `T_REF` once from the design's own side (`fabricate_at`) and
hands it to both: `bake_gltf(fabricated=, side=)` (the side's layer plan for the drive
extras) and `mjcf.set_fabricated`. `mesh.tessellate` now cleans a shape's earlier
triangulation first: on the shared parts the hulls' 0.5 mm mesh would otherwise reuse
the bake's finer 0.1 mm one (the mesh is the parameters' alone, whoever meshed the shape
before). `bake_gltf(cfg)` with no walker given (`spiderpig bake`, the dev server) is
unchanged.

**Measured.** glb 59.0 → 24.4 s, mjcf 49.6 → 17.4 s (round 4, each ~4 s of it the servo
record of section 3). Klann: 19.5 → 18.8, 13.1 → 13.0 (its plan is 1.4 s); both formats
in one export share the fabrication. What remains of the glb (after, 25.1 s): the
fabrication at t = 0, 11.9 s; the bake 13.1 s (tessellation 9.7, the mesh sharing's mass
properties 2.2).

**Outputs.** The glb's BIN and JSON chunks, the MJCF and its metadata: identical to the
base's on both designs and both paths, now from the stored plan.

**Risk.** Low: the same fabrication from the same plan, re-made and `verify_plan`ed. A
design whose stored plan differs from what a fresh solve in a new process would find (a
solve cut by its deadline) now bakes its own plan, where the base baked whatever the
second solve found.

### 3. `build` / `fabricate`: the servo model's bounding boxes, every process — 23.7 → 17.1 s

**What it did** (round 4, 22.0 s in the base's profile). `fabricate_side` 14.6 s: OCCT
booleans 9.1 s (the links' `plate()` 3.2 s, the printed axles 3.1 s, the crank 1.9 s,
the chassis 1.1 s) and the drive group 6.3 s, **5.4 s of it the servo's manufacturer
model**: `cad_servo` → `strip_horn` → `_matches` takes an optimal bounding box of every
one of the XL330 model's 15 solids (4.65 s; build123d's `bounding_box()` is `Clean` +
`AddOptimal`) to find which solid is the horn, after a 0.7 s import. `attach_build`
5.3 s (`part_props` × 161 2.2 s for the masses; the STEP files 1.4 s);
`assemble_robot` 1.7 s (the right side is the left mirrored: one transform per part,
nothing rebuilt).

**The stupid thing.** Which solid of a hash-pinned file is the horn was decided again,
by the most expensive box there is, in every process that fabricates: the build, the
verify (once for its three fabrications), each export's bake and MJCF, the CLI.

**The fix** (`servos: record which solids the horn strip keeps, ...`). The kept indices
are recorded as JSON beside the download, keyed by the model's sha256, its transform,
scale, the strip boxes, `BBOX_TOL` and a format version; a record for another solid count,
or unreadable, is ignored and rewritten. The shape is built from the same import and the
same solids (the import carries no triangulation, so the boxes' `Clean` changed nothing),
and the strip-count warning still fires. A binary BREP of the stripped shape was tried
first and dropped: its meshes are not the same (the Klann quad's whole-robot STL
changed).

**Measured.** `cad_servo` for the XL330 in a fresh process: strip 4.17 → 0.01 s (the
0.7 s import stays); the STS3215's one-solid model 0.13 → 0.10 s. The round-4 build
23.7 → 17.1 s.

**What else is in a fabrication, and why it stays.** One boolean per feature: a link is
two pills (two fuses each), one union and one cut of all its holes; a crank segment a
union of discs and posts minus a union of bores and pockets; an axle segment a revolve
and one cut with all its slot cutters. No fillets, no repeated `Part` operations. The
identical links of the two legs are built twice (not fixed: see Not done).

### 4. `verify standard`: 64 s cold (now 51), 49 s "warm" (now 0)

**What it did** (round 4, cold, the base's profile at 53.8 s wall / 86.5 CPU-s):
`api.build` 19.5 s (fabrication 14.3, `attach_build` 5.2), `check_side` at t = 0 and
t = 3.2: 22.9 s (every group realized again at each angle, the claims' envelope solids
`claimed_solid` 5.4 s, `part − envelope` per part 4.4 s), `clashes` 7.3 s (pairwise `&`
after a bounding-box prefilter), `bad_solids` 1.8 s (`is_valid` × 161), the DXF pack and
the ungrouped BOM ≈ 1.5 s, `verify_plan` on fresh samples ≈ 0.5 s.

**Redundant?** Little. The contract's two angles are not the build's, and its side is
not the robot's (no frame ties), so neither the build nor the export's t = 0 fabrication
can stand in for it; the clash check runs on the build's own parts. The redundancy was
inside the fabrications: the servo strip of section 3, once per process (4.2 s). Cold:
64.2 → 50.7 s. After (a run at 56.6 s): `check_side` 27.2 s, `build` 16.6 s, `clashes`
8.5 s, `bad_solids` 2.0 s; OCCT booleans are 42 s of it.

**The 45-49 s warm miss** (`store: keep one verify report per level ...`). `verify.json`
held one level and `_cached` served it for that level only: a `quick` after a `standard`
overwrote it, and the next `standard` ran again (TIMING.md's warm pass: 45 s). The store
now also writes `verify.<level>.json` and reads the level's copy first (then
`verify.json`, so a store written before this still serves its own level); `verify.json`
stays the latest, for the cards and `stages()`. Validity is unchanged: the running
engine version. Measured: 48.7 → 0.0 s (round 4), 40.9 → 0.0 s (Klann).

### 5. The bake (`spiderpig/bake.py`)

**What it did** (`spiderpig bake`, a fresh process, round 4: 58.8 s in the base's
profile). `1_reference_build` 41 s = the plan solved (27 s: the CLI has no design, only
its options) + the fabrication 14 s; `2_tessellate_total` 8 s (printed parts 3.3, the two
servos 3.2: the right servo is a mirror, not a translate, so it is meshed on its own);
`2_mesh_share` 2 s; nodes and channels 1 s. Inside an export it was the same, the plan
included (section 2).

**Fixed.** Inside an export: no plan, no second fabrication (section 2). Everywhere: the
servo record. `spiderpig bake` alone still plans from its options (that solve is the
planning side's): 60.3 → 50.3 s round 4. The profiler's stage keys are unchanged; the
export's bake still runs with `profile=False`.

### 6. The rest: `stl`, `step`, `dxf`, the STEP reload

Nothing stupid found. `stl` is one `BRepMesh` of the whole robot at 1e-3 mm on three
cores (11-14 s round 4, 5 s Klann); `step` the XDE writer (3-4 s); `dxf` sections, kerf
offsets and the packing (1.5-2 s). The reload (7 s for round 4's 161 parts from 90
files): `import_step` 3.0 s (an XCAF reader per file), `part_props` 2.3 s,
`placed_part`'s deep copies 1.4 s; see Not done.

### Not done (would change outputs, or needs a decision), with its number

| what | saving | why not |
|---|---|---|
| bake at the build's crank angle (t = 1) and share `design.mech` | the one fabrication left in a glb/mjcf export: 11.9 s round 4 | the glb's frame 0 and the MJCF's `t_ref` (qpos0) change: a design decision |
| reuse `design.mech` when the build is at t = 0 | the same, for such builds | an edited (rechecked) part would then show in the glb and the MJCF, where the base fabricated afresh: a semantics decision |
| keep the grouping with the build (the store) | 25 s per later `print` or `bom` export, and for `spiderpig build` | print and bom asked in two exports group twice; a store-layout decision |
| keep the baked glb per design across export folders | 24 s round 4, 19 s Klann per export of `glb` into a new folder | a store-layout decision, and the bake's code is not in the engine version (a cached glb could outlive a bake change) |
| a binary BREP beside each STEP in the store | most of the reload's 3 s of `import_step` | a reloaded build's parts would become the fresh build's (its STL and DXF change, towards the fresh ones); the MCP hands out the STEP paths |
| build each identical link once and move it to the other legs | up to half of `plates.realize`'s 3.2 s per fabrication (× 3 in a verify) | a moved copy is a new B-rep; needs the gate on every output, not attempted |
| one 2D profile extruded per plate instead of pill fuses + a cut | part of the 9 s of booleans per fabrication | different faces and edge order: DXF polylines and glb vertex order change |
| skip build123d's deep copy in `placed_part()` for an identity pose | 1.4 s per reload, 2.2 s per verify | `to_compound` sets the label and colour on the copy; sharing the object would carry them onto the stored parts |
| the clash check in parallel | most of its 7-8.5 s per verify on 4 cores (an estimate) | needs processes (pickling 161 solids); not attempted |
| plain (not optimal) bounding boxes in `attach_build` and the contract's prefilter | ≈ 2.3 s per verify | the build report's `envelope_mm` (and the manifest's) would change |

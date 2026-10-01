# Where the time went, and what was done about it — 2026-10-01

A follow-up to [TIMING.md](TIMING.md): for every slow step of the known path, what it
does, the stupid thing (or "nothing stupid"), the fix, the saving measured after it, and
the risk. The gate for every fix: the same result as before (the same plan, proof and
route for every linkage x module the planner's tests cover, the same cards, the same CLI
text). A change that would alter a plan or a proof, or trade optimality for speed, is a
design decision and was not made: it is listed with its number.

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

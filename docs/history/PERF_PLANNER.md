# The layer planner: the shape of its search, and what would make it fast — 2026-10-01

> **History** (2026-10-01; historical: the planner's search measured. The opt-in `StackSpec` flags it introduced still exist; the planner moved to `spiderpig/stack/` (W5).)

The question (the user's words): *"For the planning, that's insane. It's a DFS on not that
many nodes, so we shouldn't have bad fan-out. Are we pruning the search effectively (e.g.
once we're beyond the global min number of layers so far, we don't need to keep exploring
that branch)? Are we using parallelism? Is what's running in the loop fast enough?"*

**Short answers**, each measured below:

- **Fan-out**: small. 2.0-3.9 layers tried per node, depth 16-24, a few hundred to 23 000
  nodes per stack size. What is expensive is the node (0.5-0.75 ms on the Strider, 1-2 ms
  on TrotBot and the sixbar, up to 5 ms in a 41-layer stack) times the number of stack
  sizes that get a real search (4-6 of the 17-26 tried).
- **Pruning**: the incumbent bound is structural (a size at or above a plan found is never
  searched again), and forward checking, backjumping, nogoods and the route's branch and
  bound all work. Three things leak: **the search explores every mirror image of every
  layering** (the legs of a module, and the Strider's two halves, are symmetric: breaking
  that halves round 4's nodes and lets candidate 2 finish its proof); **a short search
  keeps branching-and-bounding a size after its first plan** although a thinner size is
  tried next; **the leg-at-a-time second strategy found a plan in 2 of the 53 multi-leg
  designs** and spends 23 % of all the planning time across them.
- **Parallelism**: none (CPU = wall everywhere). A portfolio of stack sizes in forked
  workers returns the serial planner's answer node for node, in 33-50 % less wall time
  for 1.05-1.9× the CPU (the budget-bound quads waste the most on wrong guesses).
- **The loop**: pure Python doing ~550-650 µs of dict and set work per node, 40 % of it the
  crank route's exact DP, 40 % forward checking. Rewriting the hot paths for speed (same
  search, node for node) gives 15-19 %; Cython on the files as they are, 6-8 %; a compiled
  core would give 20-50× on the search and is a project.

## How it was measured

- **Machine**: the 4-core VM of TIMING.md, Python 3.12, `PYTHONPATH=` the tree under test.
  The exporters' agent ran beside this one throughout (load 2-10), so **every before/after
  ran side by side** (both at once, equal contention), CPU seconds are given beside wall,
  and the parallel prototype was run on its own.
- **Trees**: *base* `5f426b1`; *C1* `42bc4e6` (the serial speedups); *C2* `8089444` +
  `179cb76` (the prototypes, all off by default); each an archive of its commit in the
  scratchpad.
- **Node-exact**: `StackSpec.max_seconds = inf` with every node budget kept, so two trees
  that make the same search explore exactly the same nodes and differ only in time. The
  60 s deadline would let a faster planner search further, which hides what it saved.
  TrotBot quad and the sixbar quad then stop at the 60 000-node total (in normal runs the
  60 s deadline stops them first, unproven either way), candidate 2 at its per-size budgets.
- **Designs**: *round 4* (Strider `double`, `shin 16`, `unit 6.3`, `xl330_m288`,
  `plywood_3mm`: TESTDRIVE.md's final), *candidate 2* (the same at the default `unit`),
  *TrotBot quad*, *sixbar quad*, *Klann quad*.
- **Instruments** (scratchpad `pp/`): `shape.py` (an instrumented copy of `_Search.dfs`:
  nodes, depth, layers tried, open layers, assigns and their failures, router checks,
  backjumps, nogood hits, per stack size and per phase), `prof.py` (cProfile of the outer
  solve), `bound_expl.py` (conflict sizes against what the router reads), `witness.py` and
  `memo_keys.py` (how often the route DP's answer repeats), `capture.py` + `bench_solve.py`
  (the DP offline on 5 272 captured inputs, every output compared), `rand_relax.py`,
  `static_bound.py`, `symmetry.py` / `symcheck.py` / `symbreak.py` / `syms_list.py`,
  `solo_hits.py`, `legs_scan.py`, `pickle_test.py`, `td.py` / `pair.sh` / `pair2.sh` (wall
  and CPU per design, side by side), `gate.py` / `gate_cmp.py` (the identity gate: every
  linkage × module, 79, and seven variants: 86 designs, node-exact).

## 1. The tree, per design

Base, node-exact. Phases as `StackProblem.solve` runs them: *find* (`_first`: a short
search of 1 500 nodes per size, upward while sizes are ruled out, doubling once they stop
being ruled out, then back down), *prove* (every thinner size again, with half the full
budget), *route* (branch and bound on the crank route's cost in the size found).

| design | links | sizes searched | find | prove | route | total | result |
|---|---|---|---|---|---|---|---|
| round 4 | 16 | 17 (3-21 layers) | 11 821 nodes, 7.2 s | 6 634, 5.0 s | 16 721, 11.4 s | 35 176 nodes, 23.7 s | 20 layers, cost 7 (features), proven; first plan at 6.3 s |
| candidate 2 | 16 | 17 | 13 130, 7.0 s | 25 878, 13.4 s | 20 001, 13.3 s | 59 009, 33.9 s | 21 layers, cost 5, **unproven**: 20 layers and the route left open at their budgets |
| TrotBot quad | 24 | 26 (to 41) | 28 934, 39.0 s | 31 067, 51.1 s | – (total spent) | 60 001, 90.7 s | 36 layers, cost 16, unproven (19-35 open) |
| sixbar quad | 16 | 21 | 15 624, 21.6 s | 44 377, 62.4 s | – (total spent) | 60 001, 84.6 s | 24 layers, cost 12, unproven (19-23 open) |
| Klann quad | 16 | 8 | 66, 0.5 s | 0 | 0 | 66, 0.6 s | 12 layers, cost 0, proven |

(The instrumented run is ~5 % slower than a plain one; the multi-leg designs first plan the
single module for the leg hint, 0.04-0.43 s.)

**Inside a size** (`shape_*.txt`): depth = the number of links (16 or 24); 2.0-2.7 layers
tried per node on the Strider, 2.3-3.9 on TrotBot and the sixbar (12-13 in TrotBot's sizes
36-41, below); after forward checking the link chosen has 2-4 open layers (8-30 at depth
0-1). Most of the work is at inner nodes: 2.6 assigns per node, 60 % of them failing at
once (a collision, a wiped domain, a dead crank route), 1.3 router checks per node, 20-30 %
of them dead. Leaves are rare (2 in round 4's 18 222-node route phase): the router cuts
nearly every branch before its last link. The variable order (fewest open layers, then the
assembly tree from the crank) does its job: 1-3 open layers below depth 3.

- **Round 4**: 3-17 layers are ruled out by the short search (2.8 k nodes, 2.3 s); 18 and
  19 exhaust it (1 501 nodes and 1 501 a leg at a time each); the doubling jumps to 21 (a
  plan of cost 5), back to 20 (cost 8 at node 29, cost 7 at node 445); the
  proof searches 19 layers again from its root (5 296 more nodes, 4.0 s) and 18 (1 338,
  1.0 s); then **11.4 s of branch and bound find no route cheaper than 7 in 20 layers**.
  Most of its dead ends are the crank's joint rules (5 607 of 6 390 router conflicts: the
  relaxation lets a route through, the exact DP finds no screw that fits).
- **Candidate 2**: the same until 21 layers (cost 5 at 5.9 s); 20 layers then gets 1 501 +
  1 501 + 10 001 + 10 001 nodes and stays open; the route phase's 20 001 don't finish.
- **TrotBot quad**: 19, 20, 22, 26 and 34 layers each exhaust the short search and a leg at
  a time (3 002 nodes, 2.5-5 s each); 41 (the cap) has a plan at node 105; the descent
  41 → 36 finds one in each size **at node ~105, then spends its other ~1 400 nodes
  enumerating leaves** (1 337-1 346 per size: the last two links have ~25 open layers each,
  and a leaf's bound conflict is explained by every link, so nothing is learned): 6 × 1 501
  nodes, 13.8 s, in sizes all thicker than the one kept. 35 and 34 layers take the rest of
  the 60 000 (conflict sets of 9-20 links, above the 8 a nogood is kept for).
- **Sixbar quad**: a plan at 26 layers at 18.5 s; 25 and 24 find plans only with the
  proof's budget; 23 is left open after 20 002 nodes, 22 after 4 373.
- **Klann quad**: a 66-node search.

## 2. Where the time goes

**Per node** (round 4, cProfile of the outer solve, base: 48.2 s profiled, 35 176 nodes;
0.65 ms a node unprofiled):

| part | share | what |
|---|---|---|
| the crank route's exact DP (`CrankRouter._solve`) | 41 % | 36 773 calls (one a node; 25 % memo hits), 210-260 µs each: a shortest path over layers and chains under the joint rules; `chains()` 624 k calls |
| the rest of the router's sub-check | 16 % | the reachability relaxation forward and back, `_prepare`, the memo key, wording a rules dead end (`_unbuildable`, 5 %) |
| forward checking | 39 % | `add` (322 k shapes placed, 9 a node, 0.5 shapes scanned each: the cost is the call and its effects, not the scan), `spans` (1.5 M `close` calls, almost all cutting nothing), the router's state cut, `undo` (670 k cuts undone) |
| the DFS itself | 4 % | `dfs`, `values`, the variable choice |

The DP's input changes at nearly every node: the parent's answer still holds (same cost)
in 20 % of calls (`witness.py`), and leaving the unplaced riders out of the memo key adds
2 % hits (`memo_keys.py`). In TrotBot's 36-41-layer sizes a check costs 3-5 ms: the DP
grows with the layers below the hub.

**Per phase**: round 4 31 % find / 21 % prove / 48 % route; candidate 2 21 / 40 / 39;
TrotBot 43 / 57 / –; sixbar 26 / 74 / –. The plan returned is in hand at 6.3 s (round 4),
5.9 s (candidate 2), 34.8 s (TrotBot), 18.5 s (sixbar: 26 layers; the proof's budget finds
the 24 it returns).

## 3. The four questions

### Fan-out

Not the problem: 2-4 layers tried per node, depth 16-24, a few thousand to 23 000 nodes
per size. The exception, TrotBot's descent (12-13 layers tried at the last two depths), is
a pruning leak (below). The cost is (a) the node, 0.5-5 ms, and (b) the sizes that get a
real search: 4-6 per design near the optimum, thousands of nodes each.

### Pruning

**(a) Sizes and the proof.** Each stack size is its own search. The short search (1 500
nodes) runs per size; the proof later re-runs every size it didn't rule out **from its
root** with half the full budget, keeping that size's learned nogoods (round 4's 19 layers:
3 002 short nodes, then 5 296 more). That is round 4's 5.0 s of proof; its other 11.4 s
are the route's branch and bound (c).

**(b) Static lower bounds.** The conflict graph's clique (links whose own shapes collide
in every layer both may take, `static_bound.py`) gives ≥ 10 layers for round 4 and
candidate 2 (they need 20, 21), ≥ 8 for Klann (12), ≥ 14 for TrotBot (36), ≥ 10 for the
sixbar (24). The sizes it would skip are ruled out by the short search in a few dozen nodes
each (every size up to 13 layers: < 0.4 s a design). What makes 18-19 layers impossible for
round 4 is the crank's joint rules (a stock screw per chain: "the crank's own rules forced
it taller", 1 202 layouts), which no pairwise bound sees. Not worth building.

**(c) Branch and bound on the route.** The bound is applied at every node, before the
leaf, through the router's sub-check (its DP is a lower bound: a run layer an unplaced
rider may take costs nothing). Two weaknesses: the bound is loose (a layer is free if *any*
unplaced rider of its point may take it), and a bound or joint-rules conflict is explained
by every placed link, so the search backtracks chronologically there and keeps no nogood
above 8 links. Explaining a router conflict by what the router actually reads (the riders,
the pieces blocked below the hub, the unplaced riders' removed layers: `tight.patch`)
**changes nothing** on round 4, candidate 2, the sixbar and Klann (the very same nodes:
there every placed link is a rider or blocks a crank piece) and on TrotBot finds a cheaper
route (cost 14 against 16) in the same budget at +43 % time. Dropped.

**(d) Symmetry: the biggest leak.** Re-timing the crank cycle (a shift by half or a
quarter turn, run backwards, mirrored about O) maps every point's path onto another's for
all five designs: round 4 and candidate 2 have three such maps (the two legs swapped; each
Strider leg's two halves b1-b4 ↔ b5-b8; both), the quads one (leg 0 ↔ leg 3 mirrored).
`symcheck.py` / `syms_list.py`: every point, link, rider, router table and axle fact maps
onto itself, and every claim gives the mapped shapes for the mapped layering (on sampled
layerings, radii to 1e-9 mm: the legs' paths are computed apart and agree to ~1e-13). The
planner explores every layering and its mirror images. Keeping `layer(x) <= layer(g(x))`
for the first link `x` it places and every map `g` (prototype `symmetry`, below): round 4
35 176 → 17 712 nodes with the same plan; candidate 2 **proven** (it was not), the same
plan, in 34 122 nodes against 59 009 unproven; the sixbar's 24 layers proven thinnest
(only its route stays open).

**(e) The incumbent.** A size at or above a plan found is never searched again: sizes are
tried fewest layers first, the doubling stops at the first plan, the descent stops at the
first size without one, the proof only goes down. Within one size, "a partial layering
already needing ≥ K layers" can't arise: the size is fixed. Two leaks:
1. A short search that finds a plan **keeps searching that size for a cheaper route** for
   the rest of its 1 500 nodes, although the next, thinner size is tried next and, if it has
   a plan, this one is thrown away: TrotBot 6 sizes × ~1 400 nodes = 13.8 s of 90.7 (its
   descent 41 → 36), round 4 21 layers' 1 501 nodes (1.3 s). Prototype `quick_first`.
2. The doubling overshoots and walks back one size at a time (TrotBot 34 → 41 → 36: 7
   short searches above the plan kept); with leak 1 closed each costs ~105 nodes.

**(f) The leg-at-a-time strategy** (a second short search per size, each leg at the single
module's layers, whenever the first found nothing) found a plan in **2 of the 53 multi-leg
designs** (`legs_scan.py`: the sixbar_v3 quad and the TrotBot toe decker) and took 201 s of
the 855 s all of them plan in, node-exact: 0.9 s on round 4, 5.1 s on candidate 2, 14 s on
TrotBot quad, 25-28 s on each sixbar quad, 12 s on the Strider quad, 19-25 s on the TrotBot
heel and toe doubles. Without it (side by side with C1): round 4 19.0 → 17.5 CPU-s (same
plan; 19 layers ruled out in 12 457 nodes instead of 15 453), candidate 2 27.5 → 23.7 (same
plan and proof), the sixbar 66.8 → 65.9 (same plan; 19-21 layers now ruled out), **TrotBot
76.9 → 105.5** (same plan and proof: node-exact, the cheap leg-at-a-time nodes, 0.6 ms, are
replaced by 1.5-2 ms ones in the same budget; under the deadline it simply searches other
nodes for the same 60 s). Not a clear win: it does find the plan in two designs.

### Parallelism

None today: CPU = wall in every run (TIMING.md). Three kinds of independent work exist:
the stack sizes (separate searches), the phases after the first plan (each thinner size's
proof and the route of the size found), and the recommendation re-runs. The problem can't
be pickled (claims are closures: `DriveGroup.claims.<locals>.make`), so a worker must be
forked (35 ms with a reply) or rebuild the problem from the config (`side_problem`
0.25-0.47 s plus 3 s of imports). Splitting one size's tree across workers loses the
shared nogoods and the order the bound tightens in, so it can't promise the serial answer;
a portfolio of whole sizes can, and that is prototype `workers` (below): with four
workers, the same plans and proofs, node for node, in 12.5 s instead of 18.7 (round 4),
14.2 instead of 26.7 (candidate 2), 34 instead of 68 (sixbar), 45 instead of 74 (TrotBot).

### The inner loop

0.55-0.65 ms a node on the Strider, 1-2 ms on TrotBot and the sixbar (up to 5 ms at 41
layers), pure Python: the DP builds dicts and tuples per chain, forward checking makes ~19
domain cuts a node with a trail entry each, `close()` ran 1.5 M times to cut nothing.

- **Rewritten for speed, the same search** (C1, below): −15-19 %, node for node.
- **Cython on the files as they are** (`cythonize -3 -X infer_types=False -X
  annotation_typing=False` of `stack.py` and `route.py` in a scratch venv; one line changed:
  `INF` an int, which Cython's comparisons need): round 4 21.5 → 19.9 CPU-s, candidate 2
  31.6 → 29.8 (6-8 %). Untyped Python compiled stays dict-bound.
- **Bitsets over layers** (not built): domains as int masks, precomputed per-link conflict
  masks, a cut one AND and an undo one restore per link, the router's state cut ~10 ANDs
  over state planes. Estimated 1.3-1.5× on the forward-checking 40 %; it rewrites `_Search`,
  and the conflict explanations per (link, layer) still need their sets.
- **A compiled core** (not built): Rust or C++ behind a binding, or typed Cython, with the
  claims tabulated per layering of their 1-10 links (lazily) and the DP on arrays: ~10-30 µs
  a node, 20-50× on the search; a new build dependency and a second implementation of the
  search and the router to keep in step (`tests/brute.py` as its oracle).
- **CP-SAT** (not prototyped): the layer assignment is a natural CP model, but the crank
  route (a shortest path over chains with screw lengths, pockets and end play) would need an
  automaton per point and every claim tabulated, and a model without it finds "plans" in the
  18-19 layers the joint rules forbid, so it can't prove what the planner proves. Weeks of
  work and a large binary dependency for a search whose size is not the problem.

## 4. Prototypes, before and after

Branch `worktree-agent-a45e5907b3a75b1b0`.

### C1 `42bc4e6`: a faster route DP and sub-check — identical, merge

The DP keeps its transitions and the order ties are broken in, but carries a chain's runs
as a linked list until the route is read back and inlines its steps (offline on 5 272
captured inputs: 216 → 156 µs a call, every output identical); the relaxation passes inline
`_valid` / `_webs` (400 000 random inputs, identical); the sub-check builds no `Route` it
throws away; `close()` takes an axle's layers by a set intersection and `spans()` skips the
domains they are already gone from (an empty one still answers its conflict); the router's
state cut and `undo` bind their tables once.

| design (node-exact, side by side) | base wall / CPU | C1 wall / CPU | |
|---|---|---|---|
| round 4 | 24.9 / 23.0 s | 21.1 / 19.4 s | −16 % CPU |
| candidate 2 | 33.9 / 32.4 s | 27.7 / 26.5 s | −18 % |
| TrotBot quad | 89.3 / 87.5 s | 72.4 / 71.1 s | −19 % |
| sixbar quad | 82.9 / 79.5 s | 68.9 / 65.9 s | −17 % |
| Klann quad | 1.0 / 0.9 s | 0.9 / 0.9 s | |

**Identity gate: 86 of 86 designs identical** in layers, top, route, cost, `optimal`, the
proof text (node counts included) and `describe()`, and every `PlanError`'s summary,
blockers and size tally (gate seconds 2 036 → 1 686 under the same load). The planner
tests (`test_route`, `test_crank`, `test_planner_bounds`, `test_stack`, `test_recommend`:
the brute-force optimality comparisons included) pass: 104.

### C2 `8089444` + `179cb76`: four prototypes, each off by default

The default path is C1's: with every flag off, 86 of 86 gate designs are identical to the
base, the proof text included; the five planner test files pass (104), and so does the
suite without its `slow` tests (`-m 'not slow'`; `test_view.py` needs a viewer build, which
this worktree hasn't: pointed at one by `SPIDERPIG_VIEWER_DIST`, it passes). The `slow`
tests (bakes, MuJoCo, the tuner) were not run.

**`StackSpec.symmetry`** (`stack_symmetry.py`, ~200 lines). Finds the re-timings that map
the problem onto itself (geometry, links, riders, router tables, axle facts: 0.02-0.1 s a
design), checks the claims commute on 8 sampled layerings per claim when a size gets more
than its short search (~0.15 s a map and size), and keeps `layer(x) <= layer(g(x))` for the
size's first link `x` and every map `g`. Side by side with C1 (this run checked 24 sampled
layerings per claim; the commit checks 8):

| design | C1 wall / CPU | symmetry wall / CPU | plan | proof |
|---|---|---|---|---|
| round 4 | 22.2 / 18.1 s | 13.9 / 10.4 s (−43 % CPU) | identical | proven; 19 layers ruled out in 9 884 nodes (was 15 453), the route in 7 828 (was 18 222) |
| candidate 2 | 45.3 / 27.6 s | 36.3 / 20.5 s (−26 %) | identical layers and route | **now proven optimal** (20 layers ruled out in 18 339 nodes; route searched to the end) |
| sixbar quad | 70.5 / 65.7 s | 73.4 / 68.3 s | identical | **24 layers now proven thinnest**; its route still open at the budget |
| TrotBot quad | 92.5 / 75.0 s | 93.0 / 74.0 s | identical | 19 layers now ruled out; 20-35 open |
| Klann quad | 1.4 / 0.9 s | 1.9 / 1.0 s | identical | identical (+0.1 s finding the map) |

**Gate with `symmetry`: every one of the 86 designs returns the same plan** (layers, top,
route, cost); candidate 2's `optimal` turns true; the three `PlanError`s (Klann quad on
bolts, TrotBot heel and toe double) say the same with fewer nodes (Klann: 15 layers ruled
out in 2 922 search steps instead of 4 610; TrotBot: 23 layers ruled out too); the gate's
2 036 s took 1 336. What it trades: the claims' check is a sample (they are closures), so a
construction that broke the symmetry only in layerings the sample missed would make a proof
wrong (never a plan: every plan is still `verify_plan`ned). To merge it, the constructions
should state their symmetry (or the check be made exhaustive per size, which the claims'
1-10 links make too costly as is), and the tolerance (1e-6 mm on paths, 1e-9 on radii)
reviewed.

**`StackSpec.workers`** (`stack_pool.py`, ~300 lines). Every stack size searched in a forked
worker (the problem comes over by `fork`, nothing pickled but the answers); the coordinator
runs the serial algorithm and asks each run of each size of its worker, while idle workers
run what it will likely ask next (the next sizes up once they get dear, the sizes below a
plan, and, once a plan is found, every thinner size's proof and the route of the size found
at once). A run made with a larger budget than the serial one is cut at the serial budget
from per-node records (plans found, the tally, the crank's rules); a wrong guess is replayed
in a fresh worker; a size the speculation gave a short search the serial search never asked
for is respawned when its proof is expected (without that, the sixbar's proofs replayed one
after another on the critical path: 113 s; `179cb76`). Measured alone, one design after
another (the other agent at load 4-5), against C1 serial in the same minutes:

| design | C1 serial wall / CPU | `workers=4` wall / CPU | answer |
|---|---|---|---|
| round 4 | 18.7 / 18.4 s | **12.5** / 19.7 s (−33 %) | identical plan and proof text, node counts included |
| candidate 2 | 26.7 / 26.4 s | **14.2** / 27.8 s (−47 %) | identical |
| sixbar quad | 67.7 / 66.7 s | **34.1** / 119.4 s (−50 %) | identical |
| TrotBot quad | 73.7 / 72.7 s | **44.9** / 137.9 s (−39 %) | identical |
| Klann quad | 0.88 / 0.87 s | 0.99 / 0.99 s | identical (no worker started: every size is cheap) |

With the 60 s deadline as shipped, the sixbar and TrotBot then spend their 60 000-node
budget in 34-45 s and return the node-exact answer, where today the deadline stops them
first. **Gate with `workers=4`: 86 of 86 designs identical** to the base, the proof text
(node counts) and every `PlanError`'s blockers and size tally included (the gate's 2 036 s
in 1 273 s of wall, beside the other agent). What it trades: CPU (speculation that guesses
wrong is thrown away), `fork` (Linux; Python 3.12 warns when forking a threaded process
such as the MCP server's engine thread), a worker per size alive for the solve (memory is
copy-on-write), and it wants the cores to itself: four workers beside a busy machine
contend with each other.

**`StackSpec.quick_first`**: a short search stops at its first plan. Side by side with C1:
round 4 20.2 → 18.6 CPU-s (−8 %), candidate 2 28.4 → 27.0 (−5 %), plans identical (the route
phase's node count moves: 18 222 → 18 225, 21 502 → 20 029); sixbar and Klann unchanged;
TrotBot's plan is found 13.8 s sooner but, node-exact, the nodes saved go to its proof,
which still runs out (CPU unchanged, plan identical).

**`StackSpec.prove` off** (plan first, the proof not run): the plan the find phase returns,
unproven. Side by side with C1: round 4 19.3 → 6.5 CPU-s (same layers and route, `optimal`
false), candidate 2 27.4 → 6.8 (same plan; unproven either way), TrotBot 76.0 → 34.8 (same
plan), Klann unchanged (proven anyway: every thinner size fell to the short search), **the
sixbar 67.5 → 18.4 but 26 layers instead of 24** (its thinner plans only appear with the
proof's budget). A design decision: it changes what `plan` returns.

### Tried and dropped

- **Tight conflict explanations** (pruning (c)): no change on four designs, +43 % on TrotBot.
- **A witness route** (reuse the parent's DP answer while it holds): 20 % of calls at best.
- **The DP's memo keyed without the unplaced riders' layers**: +2 % hits.
- **Skipping `add()`'s collision scan for a link's own shapes** (forward checking already
  cleared them: 0 collisions in 87 746 such calls): 0.5 shapes scanned a call, nothing to win.
- **A static clique bound** (pruning (b)): < 0.4 s a design to win.

## 5. What to merge, in order

Expected times for the five designs, node-exact, each measured (base and C1 alone; the
combination alone, in the same minutes as the C1 column):

| design | base | 1. C1 | 1 + 2 (`workers=4`) | 1 + 2 + 3 + 4 (+ `symmetry`, `quick_first`) |
|---|---|---|---|---|
| round 4 | 22.9 s, proven | 18.7 s | 12.5 s | **7.9 s** (10.7 CPU-s), the same plan, proven |
| candidate 2 | 32.2 s, unproven | 26.7 s | 14.2 s | **11.5 s, proven optimal**, the same plan |
| sixbar quad | 79.7 s (60 s deadline: cut short) | 67.7 s | **34.1 s** | 44.0 s, 24 layers proven thinnest, the same plan |
| TrotBot quad | 90.8 s (60 s deadline: cut short) | 73.7 s | **44.9 s** | 66.1 s, the same plan |
| Klann quad | 0.9 s | 0.9 s | 1.0 s | 1.0 s |

(The two budget-bound quads spend the same 60 000 nodes in every column; which nodes, and so
what a node costs, differs with the pruning, which is why 3 + 4 is slower than 2 alone for
them.)

1. **C1 (`42bc4e6`), now.** Identical node for node (86 of 86 gate designs, the proof text
   included), −16 to −19 % CPU on every design, no new concept. Risk: low (the DP's output
   checked offline on 5 272 captured inputs, the relaxation on 400 000 random ones, the
   brute-force optimality tests pass).
2. **`workers` (`stack_pool.py`), next, as an opt-in the host turns on when it has idle
   cores.** The serial answer node for node, −33 to −50 % wall on the four slow designs, and
   the two quads come in under their deadline. Before making it a default: decide who owns
   the cores (TIMING.md's third recommendation runs plans as pool jobs, several at once:
   then each wants one core, not four), keep the serial path for non-Linux hosts and
   threaded callers, and run the full test suite with it on.
3. **`symmetry` (`stack_symmetry.py`), after an exact certificate.** The largest cut in
   nodes measured (round 4 −50 % nodes, −43 % CPU; candidate 2 and the sixbar's thinnest
   size proven where they weren't), the same plan on all 86 gate designs. The geometry
   (to 1e-6 mm), link, router and axle checks are complete; the claims' check is a sample,
   so either the constructions declare their symmetry (an axle's claim is built from its
   axle and links only, by name-agnostic code) or the check is made exhaustive in a
   cheaper form (per claim, by the layers of the links it actually reads). Until then it
   may not back an `optimal` (a plan is still verified either way).
4. **`quick_first`**, small: −5 to −8 % on the proven Strider designs, the same plans; the
   budget-bound ones find their plan sooner (TrotBot 13.8 s), not their proof.
5. **`prove` off as an explicit second mode, not the default.** "Plan first, proof as an
   optional second call": round 4 6.5 s instead of 19.3 (the same plan, unproven), candidate
   2 6.8 s, TrotBot 34.8 s instead of 76; but the sixbar then returns 26 layers instead of
   24 (its thinner plans only appear with the proof's budget). It changes what `plan`
   returns; an agent comparing candidates could plan each this way and prove only the one
   it keeps.

## 6. What not to do

- **Don't split one stack size's tree across processes.** The route phase, the longest
  single piece (round 4: 11.4 of 23.7 s), is one size; its workers would lose the shared
  nogoods and the order the bound tightens in, so they can't promise the serial plan (the
  first of equal-cost routes found) or its proof text. The portfolio of sizes already takes
  the parallel work there is.
- **Don't drop the leg-at-a-time strategy wholesale.** It found the plan in 2 of 53
  multi-leg designs; node-exact, the budget-bound designs get *slower* without it (TrotBot
  +37 %: its cheap nodes are replaced by dear ones). If anything, give it a smaller budget.
- **Don't build a static clique bound**: it rules out < 14 layers, which the short search
  already does in < 0.4 s a design.
- **Don't tighten the router's conflict explanations as prototyped**: no change on four
  designs, +43 % on TrotBot.
- **Don't reach for CP-SAT or a compiled core first.** The tree is small; the waste is
  symmetry and wasted runs, which 3-4 remove in the planner as it is, and 2 overlaps what
  is left. A compiled core
  (20-50× a node) is the step after, if planning is still the bottleneck: it is a second
  implementation of the search and the router to keep in step. Cython on the files as they
  are buys 6-8 %, not worth a build step.
- **Don't make `prove` off the default**: the sixbar comes back two layers thicker.
- **Don't raise the node budgets to "finish" the quads**: TrotBot leaves 17 sizes open at
  60 000 nodes; with symmetry the sixbar's thinnest size is proven within the same budget.

# Test drives: an outside agent against the public surface

> **History** (2026-09-30; historical: the five test-drive rounds against the public surface. Every entry is fixed or listed as open; the tests cite it by round.)

Each round: a fresh agent with only `README.md`, `docs/agentlib/API.md` and what
the MCP server says (tools, `spiderpig://guide`, cards, catalog) tries a goal end
to end, logs every point of friction, then switches hats and fixes what it found.
Severity: **blocker** (could not proceed without a workaround or the source),
**annoying** (proceeded, but lost time or guessed), **nit**. "Recoverable" says
whether a naive agent could have recovered from the docs and messages alone.

## Round 1 — 2026-09-30

Goal: *"Design me a four-legged walking robot that fits in a 300 × 200 × 150 mm
box, costs at most $100 in purchased parts, keeps at least 40 mm of ground
clearance, and walks as fast as you can get it. Give me the laser-cut sheets, the
print files, the BOM, and let me look at it."*

Driver: the MCP server as a subprocess (`python -m spiderpig.cli mcp --store
<tmp>`), the official `mcp` 2.x Python client; then the same job through
`spiderpig.api` in a Python session.

### Wall clock, first run

| step | wall | note |
|---|---|---|
| connect + list tools/resources/prompts + read guide | 6.5 s (3.5 s connect) | |
| prompt, schema, catalog, 6 `describe` cards | 5.5 s | |
| spec 1: resolve, check, plan, walk, verify quick (Klann double) | 5.9 s | 1.0 s of engine |
| survey: 30 (linkage, module) specs, resolve + verify quick each | 246 s | four of them hit the planner's 60 s deadline |

### Entries

#### 1. "Four-legged" has no module, and `double` / `decker` don't walk — **blocker**

Tried: the goal says four legs; the guide's module table says "modules (legs per
side)", so `double` (2 per side, 2 sides). Klann `double` resolved, checked, planned
(8 layers) and walked: `stride_mm: 2.2e-13`, `speed_mm_s: 1.9e-13`, `bob_mm: 82.7`,
`duty: [1, 1, 1, 1]`, every row `pass` except the soft speed target; `verify quick`
`ok: false, score: 0`. With `phases_deg: [0, 180]`: the same. `decker`: stride 0,
`bob_mm: 460`, `tipping_fraction: 0.9`. The same for every mirrored-pair linkage
(Klann and its variants, Jansen, TrotBot, the 6-bars, the 4-bars). Only the
Strider's `double` walks (172.8 mm/rev).
Expected: the walk report (or the guide's module table) to say that a mirrored pair
per side has no net travel in the quasi-static model and that a walking module of
this linkage is `quad`; or `resolve` to warn. What happened: 30 resolve + verify
calls (246 s) to find out by survey, and no message anywhere saying why the stride
is zero. Nothing says what `double` and `decker` *are* (a mirrored pair; two legs
one way, 90° apart) either.
Recoverable from docs and messages alone: no (a stride of 2e-13 with every row
green reads as a bug, not a design choice).

#### 2. The guide's costs hold for the Klann quad only; the planner's 60 s deadline is undocumented — **annoying**

Tried: `verify quick` on every walker's `quad`. The guide says plan = 0.4 s, quick
verify ~1 s. TrotBot quad 60.9 s, sixbar quad 60.9 s, Strider quad 40.9 s, sixbar_v3
quad 48.6 s, and the Klann quad at OA 34 / the Jansen quad at unit 1.05 and 1.1 each
63 s ending in `plan/no_plan: ... after 48089 search steps in 60 s; the 60 s deadline
ran out`. `recommend` on such a design then re-runs the stage (another minute or
more): the two Strider variants that failed to plan took 184 s each through
`verify quick`. A `bolt` construction failed differently, "the 60000 search-step
budget ran out" after 35 s (`no_plan_in_budget`). Expected: the guide to say a plan search is bounded by a 60 s deadline per
design (and the checked recommendations by another), so a quick verify of a
non-default design can take a minute; and the `plan` tool description to say so.
Recoverable: yes, once you have waited.

#### 3. Scaling a linkage down to fit a box loses the plan, and the message stops at the tally — **annoying**

Tried: `derive` the Jansen quad with `unit: 1.6 -> 1.1` (to fit 150 mm of height),
and the Klann quad with `OA: 60 -> 34`. Both: `plan/no_plan`, "no layer plan found
with up to 41 layers after 48313 search steps in 60 s; the 60 s deadline ran out;
what blocked it (count x shape vs shape):" followed by the tally. (Details in entry
9 once `explain` / `recommend` returned.)
Expected: a hint in the failure that the links have become small against the fit
defaults (link_radius 6, axle 6, spacer 8.5) so thinner pins or a bigger unit are
the levers, and `recommend` to say the same with patches.

#### 4. The envelope estimate under-reads z, so `quick` passes and `standard` fails — **annoying**

The rows do say what they measure (`detail`: "joints' sweep + plates; measured
after a build" / "at t = 1"). But for the final design `size.envelope_z_mm` read
199.0 (estimated, PASS, `<= 200`) at `quick` and 205.0 (measured, FAIL) at
`standard`: the estimate misses 6 mm of the chassis (the servos back to back on
the centre plates), so I chose a design on a number that then failed after a 40 s
job. In x and y the estimate is the sweep (220 x 148) and the build's single pose
is smaller (208 x 145), which is the right direction. Expected: the z estimate to
include the chassis (it is a function of the stack and the servo, known before a
build), and the row to say it is a lower bound when it is one.

#### 5. The module names — **nit**

`quad` is four legs *per side*: an eight-legged robot. The guide's table header says
"(legs per side)", which is enough once read carefully; the `design_walker` prompt
just lists "a module (single/double/decker/quad)".

#### 6. `describe` nests its card — **nit**

`describe(key)` returns `{ok, failures, card: {...}}`, `list_linkages` returns
`{linkages: [...]}`; API.md's table says "the linkage cards". Found from the output
schema; cost one wasted call reading `params` at the top level.

#### 7. Nothing says which body part sets the ground clearance — **annoying**

The Strider's `ground_clearance_mm` is 22.6 (the goal needs 40). The check report
gives the number only; `crank_facts.underside_lowest_mm: -40.85` hints at the
underside, and swapping the servo changes the clearance (sts3215 32.4 -> xl330
37.2 at unit 7.5), so the servo body is the lowest point, but I found that by
probing 10 parameters (25 s) and two servos. Expected: `check` (and the verify row's
`detail`) to name the lowest body point (servo, frame plate, crank sweep, a link)
so the agent knows which lever to move.

#### 8. Loop-closure messages are good — no entry

`program/loop_cannot_close: strider: joint J4 can't be placed for 14% of the cycle
(crank angles 51°..129°): bars J3-J4 117.0 mm and J11-J4 65.0 mm miss each other by
up to 0.54 mm` told me exactly how far shin=18 overshoots. Kept as the model for the
other messages.

#### 9. `recommend` answers `[]` in 0.0 s on a plan that ran out of time, with no note — **blocker**

Tried: `recommend(8673cf9c4d18fcae)` (the Jansen quad at unit 1.1: `plan/no_plan`,
"the 60 s deadline ran out", sizes 16-29 "left open at their budget"). Result:
`{"ok": true, "stage": "plan", "recommendations": []}`, instantly. The failure's
own `recommendations` is `[]` and its `notes` hold only the size tally and one
static clearance. So the engine says the search was inconclusive and offers nothing,
and there is no knob in the Spec or the tools for more search time. The same for the
Klann quad at OA 34 and the Jansen quad at 1.25 and 1.4 (every unit below the
default 1.6 fails the same way; the default plans in 3 s).
Expected: either a recommendation ("unit 1.1 -> 1.6 plans in 16 layers", the
scale the engine knows) or a note saying why none is given (the search never
completed, so nothing could be checked), and a way to ask for a longer search.
Recoverable: no.

#### 10. `explain` re-solves a failed plan (61 s) though the failure is in the store — **annoying**

`plan(design)` on the failed Jansen returned its cached failure in 0.1 s; `explain`
on the same design took 61 s (the planner ran again to its deadline) before
printing the prose. API.md says "ms after plan".

#### 11. What the plan proof says is good — no entry

"no plan in 19 layers or fewer (15453 nodes); the crank's own rules forced it
taller: in 19 layers the search met 1202 layouts that fit everything else but its
crank routes need a joint no stock screw fits ...; 20 layers searched to the end for
the cheapest crank route" answered the question I would have asked next.

#### 12. Spec errors are good — no entry

Twelve deliberate mistakes (a bare number target, an unknown field, a typo in the
linkage key, a mechanism as a walker, a wildcard, a parameter of another linkage,
three phases for two legs, an unknown servo, `size.box_mm`, a string number, an
unknown output, no `kind`): every one came back `ok: false` with the path, the
message, the allowed values and the nearest key ("did you mean 'klann'?",
"`size.box_mm`: unknown metric (did you mean 'envelope_x_mm'?)", "a target is an
object {min, max, ...}, not a bare number (40: use {"max": 40} ...)").

#### 13. Two misuses fall through to the SDK's text error — **nit**

`export(formats=["svg"])` and `get_design(stage="bom")` answer `isError` with the
pydantic text "Input should be 'step', 'stl', ..." instead of the `{ok, failures}`
envelope; API.md documents `export / unknown_format` and `store / no_such_stage`
codes that the `Literal` schema makes unreachable. The text does list the allowed
values, so it costs nothing but consistency.

#### 14. The XL330 servo's solid is bad: `verify standard` fails on it and `export(glb)` crashes — **blocker**

Tried: the Strider double at shin 16, unit 6.3 with `materials.servo:
xl330_m288` (the fastest servo, 103 rpm, the guide's obvious choice for "as fast as
you can"). `verify standard` (44 s): every row green except `solids@t=1: 2 vs 0
[FAIL] L.servo; R.servo`, failure `clash / bad_solid: L.servo; R.servo`. Nothing
says what a bad servo solid means for the design (the servo is a purchased part:
its solid is the catalog's model, not anything the spec chose) or what to do.
`export(["step","dxf","print","bom","glb"])` then wrote the STEP, DXF, print and
BOM files and died in `glb` with an uncaught `AttributeError: 'NoneType' object has
no attribute 'NbNodes'` from build123d's `tessellate` (the same solid), no
`ExportReport`, no `manifest.json`; through the MCP this would have crossed as
stage `engine`, code `AttributeError`. With `sts3215` the same design passes.
Expected: the catalog model to be sound (an engine bug), and until then the
failure to say "the servo's CAD model has a bad solid; the servo is purchased, so
this doesn't affect your parts; the STL/GLB of the servo may be missing" and the
bake to skip a part it can't tessellate instead of crashing the whole export.
Recoverable: only by trial (swap the servo).

#### 15. The cost row hides what costs; packs and consumables dominate — **annoying**

`budget.cost_usd: 115.7 USD vs <= 100 [FAIL, estimated] 12 items; 5 unpriced; 3
unverified links`. I had to `export(bom)` (a job, after a build) and read `bom.md`
to learn the total is 2 servos $54.98 + a 1 kg spool of PLA $25.49 (for 83 g) +
4 oz of acrylic cement $12.84 + a pack of 100 heat-set inserts $11.37 (for 4) +
acrylic $10.99. Nothing in the spec can lower the last four, so the budget the
user gave is met by no servo in the catalog ($100.69 with the sts3215). Expected:
the row's `detail` (or the walk/quick level, since the catalog prices are known
before a build) to list the top items with their prices so the agent can answer
"why" and "what would help" without a build; and `verify quick` to price the design
from its catalog choices (servo x 2, sheet, spool, cement) as an estimate, which
it already does for the mass.

#### 16. An unpriced item counts as $0 — **nit**

"5 unpriced" items (the M3 screws, nuts, CA glue) are not in the total, and the
plywood sheet has `price_usd: null`, so switching to `plywood_3mm` lowers the
estimate by $10.99 for free. The hard budget row passes or fails on a lower bound
without saying so. Expected: the row to say "at least" when items are unpriced.

#### 17. The job record's id field is not named anywhere — **nit**

`verify(level="standard")` came back after 15 s with `job: {"job":
"0f4b2ae14b34", "op": "verify", "state": "running", ...}`. API.md and the guide say
"returns the running `job`" and "`wait_job(job, seconds)`", never that the record's
id is under `job`; I guessed `id`, crashed my client and lost the server (and its
running job, which has no record in the store). Expected: one line in the guide,
"a job record is `{job, op, design, args, state, started_at, finished_at, seconds,
result?, error?}`; pass its `job` to `wait_job`".

#### 18. Engine warnings go to stderr, where no MCP client sees them — **annoying**

While the Strider planned, the server printed `pin:J11_leg0 seg0: its snap prongs
(5.1 mm) strain 6.0 % while snapping (want at most 4.0 %)` (four such lines) to
stderr. A printed pin whose prongs overstrain is something the person building the
robot must know, and nothing in `check`, `plan`, `verify` or the BOM carries it.
Expected: such warnings in the plan report (`warnings`) and as informational
verify rows.

#### 19. `spiderpig view` re-exports a GLB the MCP `export` just wrote — **annoying**

`export(formats=[step, dxf, print, bom, glb])` wrote `exports/strider.glb`; the
MCP `view` served it (`X-Spiderpig-Glb: export`, 5.6 s to the URL). From a shell,
`spiderpig view 1239e7052a4af68d --store ...` (the README's way to look at a stored
design) printed "exporting the glb ... (built and baked once, then cached)" and
baked again: 59 s to the URL, though the file was there. The export cache is keyed
on the exact format list, so `["glb"]` misses a record of five formats.
Expected: an export to answer any subset of the formats it wrote.

#### 20. The default phases of the Strider's `double` are the walking ones, Klann's aren't — **nit**

Strider `double` defaults to `[0, 180]` (orientations `[1, 1]`) and walks; Klann
`double` defaults to `[0, 0]` (mirrored) and doesn't. Both are called `double`.
Part of entry 1.

#### 21. Cost of the search an agent must do by hand — **annoying**

v1 has no search, so the goal's four numbers took: 30 survey specs, 14 scale /
parameter derivations, 10 Strider probes, a 16-point grid, 4 construction
variants, 2 servos: 76 designs, 63 of them `verify quick`, about 18 minutes of
engine time, before one candidate met everything but the two marginal misses
below. Every number I moved (`shin`, `rocker`, `unit`, the servo) I chose by
reading the card and guessing; the linkage card says which parameters scale the
linkage but not which raise the body or lengthen the stride. Expected (v2): the
card to carry one sensitivity line per parameter (foot-path height, stride, lift,
clearance), or `tune` to be exposed.

### The robot I ended up with

Strider (coupled pair) `double`, two sides: a true four-legged walker. Spec
patches from the defaults: `unit 6.5 -> 6.3`, `shin 13 -> 16`, servo `sts3215`,
printed pivots and crank, 3 mm acrylic. Design `1239e7052a4af68d`.

| requirement | value | tier |
|---|---|---|
| plan | 21 layers, 63 mm a side (20 layers left open at the budget) | proven |
| ground clearance | 41.0 mm (>= 40) | measured |
| stride / speed | 169 mm per rev; 146.5 mm/s at 52 rpm | measured / estimated |
| envelope, quick (sweep) | 220 x 148 x 199 mm | estimated |
| envelope, standard (t = 1) | 208 x 145 x **205** mm (z misses 200 by 5) | measured |
| cost | **$100.69** (misses 100 by 0.69; 6 items unpriced) | estimated |
| mass, print, sheets | 397 g nominal; 83 g PLA; 2 sheets | measured |
| contract, clashes, solids, plan re-check | all clean | proven / measured |

With the XL330 (103 rpm): 20 layers, 60 mm, 290 mm/s, 208 x 145 x 181 mm, $115.67,
but its solid is bad (entry 14). Files (the store's `exports/`): `strider.step`,
`laser/strider_sheet_0.dxf`, `laser/strider_sheet_1.dxf`, `laser/strider_sheet_parts.csv`,
`print/*.stl` (21 distinct parts) + `print/parts.csv`, `bom.csv/.md/.json`,
`strider.glb`, `manifest.json`. The viewer (`view`) rendered it headless: the
animated robot, the drive and tune panels seeded from the design, no console
errors; with drive mode on, the HUD answered (height 77.7 mm, pitch -3.16°,
contacts 4 / 8, margin 63.8 mm, model "169.07 mm/rev · 146.53 mm/s", the walk
report's numbers).

### Wall clock, first run (continued)

| step | wall | note |
|---|---|---|
| deckers, scaled Klann / Jansen, Strider probes (14 designs) | 209 s | |
| diagnose the scaled Jansen: plan (cached) + explain + recommend | 66 s | explain re-solved (61 s); recommend empty in 0 s |
| Strider parameter probes (10) | 25 s | |
| Strider grid (10 of 16 before the 15 min task limit) | 15 min | two hit the deadline at 184 s each |
| spec mistakes, misuse, hard target, compare, list, summary | 26 s | |
| final design, MCP: derive + quick (30.7 s plan) + standard (38.5 s job) + export (75 s job) + view (5.6 s) | 150 s | plus one lost run (my client's job-record guess) |
| browser: page + GLB + panels | 13 s | |
| `spiderpig view` from a shell + play + drive | 111 s | 59 s re-bake |
| constructions and plywood (4) | 183 s | |
| the same job through `spiderpig.api` (cold store): import 2.7, resolve 0, check 0.4, plan 18.3, walk 0.1, quick 0, standard 44.3, build 0 (cached), export: crashed in `glb` | 70 s | XL330 variant |
| **total, goal to files** | **~55 min** of wall clock, ~35 of it engine time | |

Through the Python API against the MCP: the same numbers at every stage (the
store is shared by content); the difference is that a failure that is a report
over MCP is an exception in Python (`export` raised), and `VerifyReport.describe()`
prints what I had to format myself from the MCP's rows.

### Phase 2 — what was fixed, and the second run

Commits: `0e7c352` (engine: purchased compounds, the bake's mesh, advice for a
plan without a gap, explain from the recorded plan, the underside's lowest
shape), `3939382` (the reports: the walk note, the card's module strides and
sensitivities, the lowest body part, the z estimate, the cost detail, the
constructions' warnings, the export cache), `2b9190a` (MCP: `wait_job`'s shape,
`recommend`'s notes, the descriptions, the guide, API.md). Tests for each in
`tests/test_spiderpig_api.py` and `tests/test_spiderpig_mcp.py` (the section
"Test drive, round 1").

| entry | severity | fixed in | how |
|---|---|---|---|
| 1 `double` / `decker` don't walk | blocker | `3939382`, `2b9190a` | `walk` says "no net travel: ... every foot stays on the ground ... of this linkage's modules, quad (102 mm/rev) walk" on the report (`notes`) and on the stride and speed rows; the card gives every module's `stride_mm` and `walks`; the guide's Modules paragraph says what each module is and that the Strider's `double` is the four-legged walker |
| 2 the 60 s deadline undocumented | annoying | `2b9190a` | the guide's "The planner's clock", the pass table's costs, the `plan` / `verify` tool descriptions, API.md |
| 3 scaling down loses the plan | annoying | `0e7c352` | with the fix of entry 9 the failure carries "unit 1.1 -> 1.6: back to the linkage's default scale ... checked: plans in 16 layers" |
| 4 the z estimate under-read | annoying | `3939382` | the estimate counts the axle heads outside the outer plates (199 -> 205, the measured value); every envelope row's detail says what it measures |
| 5 module names | nit | `2b9190a` | the guide and the `design_walker` prompt |
| 6 `describe` nests `card` | nit | `2b9190a` | API.md's table |
| 7 what sets the ground clearance | annoying | `0e7c352`, `3939382` | `check.lowest_body_part` ("the servo's pad on the inner frame plate") and the clearance row's detail |
| 9 `recommend` answers `[]` in 0 s | blocker | `0e7c352`, `2b9190a` | `design_side` asks `recommend` even without an involved clearance; `recommend` checks the linkage's default scale for a plan that ran into the stack's own room, else notes why nothing is recommended and what levers are left; the MCP tool returns the notes |
| 10 `explain` re-solves | annoying | `0e7c352`, `3939382` | `explain_config` on the design's full config from its own plan; a recorded failure is printed (61 s -> 0.5 s) |
| 13 misuse text errors | nit | `2b9190a` | not changed: the input schema's refusal names the allowed values; API.md now says so instead of listing unreachable codes |
| 14 XL330 bad solid, `export(glb)` crash | blocker | `0e7c352` | a purchased part's compound is sound when every solid is valid; the bake meshes face by face and skips what the mesher leaves out (3 of 511 faces) |
| 15 what costs | annoying | `3939382`, `2b9190a` | the cost row lists the largest items with pack sizes; the guide says packs are bought whole |
| 16 unpriced counts as 0 | nit | `3939382` | the row says "unpriced, so the total is a lower bound" and names them |
| 17 the job record | nit | `2b9190a` | `wait_job` / `get_job` answer as the tool itself does; the record's fields are in the guide, API.md and the descriptions |
| 18 warnings on stderr | annoying | `3939382` | `plan.warnings` / `build.warnings` on the reports and as informational verify rows |
| 19 `spiderpig view` re-bakes | annoying | `3939382` | an export that wrote these formats or more is served as is |
| 20 the Strider's `double` phases | nit | `2b9190a` | the guide |
| 21 the search by hand | annoying | `3939382`, `2b9190a` (partly) | the card's `sensitivity` says which parameter moves lift, stride, height and width; the modules' strides say which walk; no `search` in v1 (the guide says so) |

Not fixed, and why:

- A `walk` (and so a `verify quick`) of a design that hasn't planned searches
  for the plan first, because the model wants the feet at their planned z; a
  design the planner can't finish in its deadline makes its first quick verify
  a minute long. That is the engine's contract (the walk of a planned design
  is the truthful one); the guide now says it. The card's strides use a nominal
  spacing instead, so `describe` stays at 0.2-1.4 s.
- The deadline has no knob in the Spec. Documented as a limit of v1.
- The goal's $100 is not reachable with this catalog whatever the linkage (two
  servos, a spool, a can of cement and a pack of inserts are $100.69 with the
  cheapest servo); the row now says what costs, which is the honest answer.
- The final design's z-extent, 205 mm against 200, is a fact of the stack (21
  layers a side plus the chassis), not a reporting problem; the quick estimate
  now says 205 too, so the agent sees it before the standard verify.
- The my-client mistakes of the first run (the SDK's snake_case fields, the job
  record's key) are the client's; the second one is now documented.

Honesty note: the first run's Python-API step ran against the main checkout's
package (the venv's editable install; the worktree was at the same commit, so the
numbers stand), because `python script.py` puts the script's folder, not the cwd,
first on `sys.path`; the MCP runs used `python -m spiderpig.cli` from the worktree
and so the worktree's code. The second run sets `PYTHONPATH`.

### Wall clock, second run

| step | wall | note |
|---|---|---|
| 1 discover: tools, resources, prompts, guide | 1.9 s | |
| 2 cards (5 describes) + catalog | 2.2 s | |
| 3 spec 1 (Klann double): resolve, walk, verify quick | 0.4 s | |
| 4 Strider double default: resolve + verify quick | 1.8 s | |
| 5 two derives + quick verifies (shin, unit) | 62.0 s | two Strider plans (32 s and 29 s), each with the same strain warnings the old run only printed |
| 6 verify standard (job) | 41.9 s | |
| 7 export step/dxf/print/bom/glb (job) | 131.4 s | the BOM's grouping and the bake |
| 8 view | 3.0 s | |
| 9 browser check | 7.2 s | |
| 10 scaled Jansen quad: resolve + plan (deadline + recommendation) | 63.7 s | the failure now carries `unit 1.1 -> 1.6 ... plans in 16 layers` |
| 11 explain + recommend on the failed plan | 0.3 s | was 66 s |
| 12 XL330 variant: standard verify + glb export | 81.8 s | `solids@t=1: 0`, the GLB written (3 faces skipped); was a hard fail and a traceback |
| **total, goal to files and the failure path** | **404 s** | no workaround; 55 min in the first run, with three dead ends |

Read as the agent: the guide's Modules paragraph and the cards' `walks` sent it to
the Strider's `double` in two calls instead of thirty; `shin`'s sensitivity line
(+10 %: lift +36 %, height +8 %) named the parameter that raises the body; the
clearance row named the servo's pad; the z row read 205 before the build; the cost
row named the servos, the spool and the cement. What it still had to do by hand:
pick `shin 16` and `unit 6.3` by two derives (the sensitivity table says which way,
not how far), and accept the two marginal misses (z 205 / 200, $100.69 / 100) as
facts of the catalog and the stack.

## Round 2 — 2026-09-30

Goal: *"I want a small, quiet six-legged walker on 2 mm acrylic with metal pivots
(bearings or bushings, not printed snap pins) and the smaller XL330 servo, under
$120 in parts, that can step over a 25 mm obstacle. Give me the files to cut and
print, tell me what it will cost and how fast it walks, and show it to me in the
viewer. Also: I already have a TrotBot design in mind at its published 7 mm scale —
tell me whether it builds and, if not, what to change."*

Driver: `spiderpig.api` in a Python session and the `spiderpig` CLI (`explain`,
`build`, `audit`, `view`), the worktree's package (`PYTHONPATH=.`), a scratch
store; the viewer checked with headless Chromium. Allowed reading: `README.md`,
`docs/agentlib/API.md`, this file's round 1, `--help`, the modules' `pydoc`.

### Wall clock, first run

| step | wall | note |
|---|---|---|
| import, `list_linkages`, `describe(klann)` | 3.8 s | 2.8 s of imports |
| 15 spec probes (six legs, 2 mm, servo names, targets) | 3.4 s | all ms; every miss a `SpecError` |
| goal spec (Klann quad, bearings, XL330, 2 mm): check, plan, walk, quick | 3.8 s | `construction / unbuildable` at 2 mm |
| four probes around the 2 mm wall (printed pins, 2.5 mm, `fit.hub_thickness`, recommend + explain) and two CLI `explain`s | 26 s | dead end: nothing names a lever |
| the same at 3 mm: quick 4.3 s, standard 45.7 s | 50 s | |
| Strider double (four legs): quick 4.9 s, standard 43.4 s; bushing variant standard 43.5 s | 92 s | |
| TrotBot heel at unit 7: check 7 s, recommend 0 s, derive 0 s, plan of the derived quad 61.5 s, quick 1.1 s, explain 3.9 s | 77 s | |
| build (cached from the standard verify) 4.8 s, edit + recheck 6.1 s, export step/dxf/print/bom/glb 43.3 s | 58 s | plus one crash on API.md's worked example (entry 6) |
| CLI `build` (Strider double, bearings, XL330) 35.6 s; CLI `audit` 66 s | 102 s | |
| export of the final design 37.7 s; `spiderpig view` to the URL 8 s (the export served as is); browser: page + GLB 3.6 s, drive mode + HUD 31 s | 80 s | |
| **total, goal to files, the viewer and the TrotBot answer** | **~8.5 min** of wall clock, ~6 of it engine time | three dead ends |

### Entries

#### 1. Six legs has no module, and the error doesn't say what the modules are — **annoying**

Tried: `legs.module: "hex"`, `"triple"`, `legs.count: 6`, and three phases on
`decker`. Every one a clean `SpecError`: `legs.module: unknown value 'hex'`,
allowed `('single', 'double', 'decker', 'quad')`; `legs.phases_deg: decker has 2
legs per side, got 3 phases`. The cards say `modules: {single: 1, double: 2,
decker: 2, quad: 4}` (legs per side), so six legs (three a side) exists for no
linkage, and of the modules that exist only `quad` (eight legs) walks for Klann and
`double` (four) for the Strider (the cards' `walks`, the round 1 fix). Expected: the
module error to say "legs per side: single 1, double 2, decker 2, quad 4 (a robot
has two sides); no linkage has a three-leg module", so the agent learns in one call
that the count is out of the vocabulary rather than mis-spelt, and API.md's
`legs.module` row to say the same. I told the user: six is not on offer; four
(Strider double) or eight (Klann quad).
Recoverable from docs and messages alone: yes (the cards), after two calls.

#### 2. 2 mm acrylic: `thickness_mm: 2` resolves silently, then every stage stops at "no M3 screw and nut fit a crankpin joint in 2 mm layers", with no lever — **blocker**

Tried: `materials.sheet: "acrylic_2mm"` — rejected, allowed `acrylic_3mm`,
`plywood_3mm` (fine: the catalog has no 2 mm sheet). Then the documented way, the
3 mm sheet with a measured `thickness_mm: 2`: `resolve` OK, `warnings: []`. `check`:
`ok: False`, failure `construction / unbuildable: no M3 screw and nut fit a crankpin
joint in 2 mm layers`, `culprits: []`, `numbers: {}`, `recommendations: []`, `notes:
[]`. `plan` the same; `walk` ran (102 mm/rev, as if built); `verify quick` FAIL with
the failure on a row named `drive.one_servo: unbuildable [FAIL, proven]` (it is the
crank, not the drive); `recommend` -> `[]` in 0.0 s with no note; `explain` prints
the program and then `STOP: no M3 screw and nut fit a crankpin joint in 2 mm
layers`. The same with printed pins, at 2.5 mm, and with `fit.hub_thickness: 8`. The
README says "measure your sheet: acrylic varies by up to 8 %", which 2 on 3 is not,
but nothing says the constructions have a thickness range. Expected: `resolve` to
warn (or refuse) when the measured thickness is far from the sheet's nominal;
the failure to name the lever (the layer pitch is the sheet: the printed crank's
joints take a stock M3 screw across `n` layers, and at 2 mm no stock length lands on
a layer boundary — say which thickness works, e.g. "3 mm plans", as a checked
recommendation with the patch `{"materials": {"thickness_mm": 3}}` or the sheet
choice), and the verify row to be named for the failing stage (`construction`).
Recoverable: no. I found the answer by trying 3 mm because round 1 used it.

#### 3. The hard budget row passes on a lower bound that leaves out the bearings (and names 4 of 8 unpriced items) — **blocker**

Tried: `budget.cost_usd: {max: 120, hard: true}` on the Klann quad and the Strider
double, bearings and bushings, XL330. Every `verify standard`: `budget.cost_usd:
115.7 USD vs <= 120 [ok, estimated] 15 items, the largest ... XL330-M288-T x 2
$54.98; PLA ... $25.49; ... cement $12.84; ... insert x 4 $11.37; 8 unpriced, so the
total is a lower bound (m2_self_tap_10, m3_shcs_18, m3_bhcs_6, m3_nut); 3 unverified
links`. The BOM (`export(bom)`, `bom.md`) says which 8: the four screws **and the 64
MF63ZZ bearings (7 packs of 10), the 3 mm rod, the 56 push-on clips and the CA
glue** — the whole pivot construction the user asked for. Bearings and bushings
price the same, $115.67, because neither is priced. So "under $120" reads as met and
is almost certainly not (seven packs of bearings alone are $40-60 at the linked
vendor). The row names only four of the eight (the screws, the cheap ones) and
hides the four that matter. Expected: a hard `max` target with unpriced items to
be `unverified` (or FAIL with "at least"), never `ok`; the row to name every
unpriced item, the biggest quantities first; and the catalog's bearing, bushing, rod
and clip items to carry a price (they have vendor links).
Recoverable: no (the row says ok; only the BOM file shows what is missing).

#### 4. `verify quick` doesn't price the design — **annoying**

`not verified at this level: budget.cost_usd` on every quick verify. The catalog
prices are known from the spec (servo x 2, sheet, spool, cement, the constructions'
hardware per joint), as round 1 asked (entry 15, "verify quick to price the design
from its catalog choices"); the round 1 fix added the detail to the standard row
only. Each answer to "what will it cost" costs a 40 s build.
Recoverable: yes (wait for standard).

#### 5. The TrotBot recommendation is checked with the `single` module, the design is the `quad` — **annoying**

`trotbot_heel` at `unit: 7`, the robot (the default `quad`, two sides): `check` ->
`static / link_no_layer`, the good message from round 1 (b7 passes crankpin J1 at
6.8 mm, needs 10.0), recommendation `unit 7 -> 10.5 ... checked: the static stage
passes, and it plans (its single module) in 12 layers (36 mm)`. `derive` with the
patch, then `plan`: 61.5 s (the 60 s deadline), 37 layers, 111 mm, `optimal:
False`, sizes 20-36 left open. So the checked claim answers a question the design
didn't ask (one leg a side), and the design's own plan is three times taller and
unproven. Expected: the recommendation to be checked with the design's module
(and to say the deadline ran out if it does), or to say plainly "checked for the
single module; the quad may take longer or more layers".
Recoverable: yes, after the minute.

#### 6. API.md's worked example doesn't transfer to the robot: `design.mech.body("b7")` and `outline` — **annoying**

Tried the lightening-hole edit from API.md on the robot (two sides). Parts are
`L.b1_leg0`, `R.b1_leg0`, ...; `design.mech.body("L.b1_leg0")` returns a body with
`outline: ()` and no joints (the robot's mechanism carries the placed bodies, not the
side's template), so `body.outline[0]` raises `IndexError`. `design.side` has
`checks, clearances, config, ctx, drive, ground_clearance_mm, groups, plan` and no
mechanism to ask. I cut at the solid's centre of mass instead. Expected: the example
(and `Part`) to say how to get a link's joints on the robot (`design.side`'s template,
or a `Part.joints_xy()` / `Part.frame` helper), and the example to state it is for
`sides: 1`.
Recoverable: yes, by fallback (the bounding box), not from the docs.

#### 7. `Part.mass_g` / `volume_mm3` don't follow an edited solid — **annoying**

After `part.solid = part.solid - Cylinder(...)` the solid's `volume` went from
1895.9 to 1886.5 mm3, `part.edited` read `True`, `recheck` passed and listed the
part as `edited` and `checked`, but `part.mass_g` (2.2562) and `part.volume_mm3`
(1895.9) stayed at the build's values; the export's BOM and the mass rows keep the
old mass. Expected: the properties to be computed from the live solid (or refreshed
by `recheck`), and the docs to say so.
Recoverable: yes (compute from `solid.volume` and the density yourself), not
obviously.

#### 8. CLI `explain` takes no servo, pins, sheet or thickness — **annoying**

`spiderpig explain --linkage klann --module quad --thickness 2` -> `error:
unrecognized arguments: --thickness 2`. `explain` accepts only `--linkage`,
`--module`, `--phases`, `--proportion`; `build` and `audit` take `--servo`,
`--pillar`, `--pin`, `--sheet`, `--thickness`. So the CLI can't explain the design
I'm building (bearings, XL330, 2 mm): it printed the printed-pin, STS3215 plan
(ground clearance 64.2 mm) while the API's `check` of my design said 73.9 mm. The
Python `explain(design)` does take the full config.
Recoverable: yes (use the API).

#### 9. `list_designs` cards don't say which design is which — **nit**

The two Strider doubles (bearings, bushings) have identical cards: id, kind,
linkage, module, sides, engine version, times, stages. Expected: servo, sheet,
constructions and the targets on the card, or a `label`.

#### 10. The bake prints to stdout while exporting — **nit**

`export` (Python) prints `tessellate: 3 of 511 faces have no triangulation;
skipped` twice (the XL330 model, round 1's fix) and build123d's `UserWarning:
Unknown Compound type, color not set`; neither reaches `ExportReport` or
`BuildReport.warnings`.

#### 11. "Quiet" has no vocabulary — **nit**

Nothing in the Spec speaks to noise; the closest levers are the pivots (rolling
bearings against printed snaps) and the servo. Not expected in v1; the guide could
say what the vocabulary doesn't cover.

#### 12. Good: the spec errors, the TrotBot message, the export cache, the view — no entry

Fifteen deliberate mistakes came back with the path, the allowed values and the
nearest key (`materials.servo: unknown value 'XL330-M288'` -> nearest
`xl330_m288`). The TrotBot's static failure said which link, which pin, the
distance, the need and its three terms, why no detour works, the least scale that
clears it, the torque cost, and that it was checked. `spiderpig view` served the
export in 8 s (`X-Spiderpig-Glb: export`); the page rendered the robot, the drive
panel seeded from the design, and with drive mode on the HUD answered (height 63.3
mm, contacts 4 / 8, margin 53.9 mm, model "172.82 mm/rev · 296.68 mm/s"). `recheck`
after an edit ran in 6 s and named the part. CLI `build` wrote everything in 36 s and
CLI `audit` came back OK in 66 s.

### The robot I ended up with

Strider (coupled pair) `double`, two sides: four legs (six is not a module of any
linkage; eight, the Klann quad, is 444 mm long). Bearings (`MF63ZZ` glued in each
link on 3 mm rod, printed sleeves, clips) for both pins and pillars, XL330-M288, 3 mm
acrylic (2 mm can't be built: entry 2). Design `6a1eee81af8cf837`.

| requirement | value | tier |
|---|---|---|
| plan | 16 layers, 48 mm a side | proven optimal |
| ground clearance (the 25 mm obstacle) | 31.5 mm (>= 25); lift 32.8 mm | measured |
| stride / speed | 172.8 mm per rev; 296.7 mm/s at the XL330's no-load 103 rpm | measured / estimated |
| envelope (built, t = 1) | 228 x 128 x 155 mm | measured |
| cost | "$115.67" (<= 120, ok) — **a lower bound without the 64 bearings, the rod, 56 clips and the glue** | estimated |
| mass, print, sheets | 395 g; 71 g PLA; 2 sheets of 300 x 300 | measured |
| contract, clashes, solids, plan re-check | clean | proven / measured |

Files (the store's `exports/`): `strider.step`, `laser/strider_sheet_0.dxf`,
`laser/strider_sheet_1.dxf` + `parts.csv`, `print/*.stl` (12 distinct, 38 parts) +
`parts.csv`, `bom.csv/.md/.json`, `strider.glb`, `manifest.json`. The bushing
variant (`885574195baab37d`) is the same stack and "cost", 23 g lighter.

The TrotBot answer: at its drawing's 7 mm unit it does not build — b7 passes the
crankpin at 6.8 mm where a post needs 10.0, no detour clears it, and no thinner
parts help; scaled x1.5 to `unit: 10.5` (the engine's checked patch) it plans, in
12 layers as one leg a side, and as the quad robot in 37 layers (111 mm a side)
within the deadline, unproven.

### Phase 2 — what was fixed, and the second run

Commits: `4696993` (the engine's reports and the handles: the thin sheet's lever,
the budget row, the checked module, the reloaded build's joints, the live mass, the
cards, the export's warnings, the CLI's explain), `96ba6b8` (API.md, the MCP guide,
the README, the `recommend` tool's description). Tests for each in
`tests/test_spiderpig_api.py`, `tests/test_spiderpig_mcp.py` and `tests/test_view.py`
(the section "Test drive, round 2").

| entry | severity | fixed in | how |
|---|---|---|---|
| 1 six legs has no module | annoying | `4696993`, `96ba6b8` | the `legs.module` error says a module is the legs per side and the robot has two sides, with every module's count ("quad 4 a side (8 on the robot)"), and that no linkage has a three-leg module; API.md's row and the guide say the same |
| 2 the 2 mm sheet | blocker | `4696993` | `resolve` warns when a measured thickness is more than 12 % off the sheet's nominal (the layer pitch follows it); the printed crank's `dims()` says why no crankpin joint fits (one-layer webs, the flattest head 1.65 mm and the nut 2.4 mm under their floors), that the least pitch is 2.9 mm, and hands the API a lever (`ConstructionError.changes`); `check` re-runs the static stage and the design's own plan at 3 mm (`recommend.construction_fix`) and hands out `{"materials": {"thickness_mm": 3.0}}` as a checked recommendation, which `recommend`, `explain` (the CLI's too) and the MCP tool carry; the verify row is `construction.buildable`, the drive's stays green |
| 3 the budget passes on a lower bound | blocker | `4696993` | the cost row names every unpriced item, largest quantities first, with its packs ("64 x MF63ZZ ... (7 packs of 10 at Amazon)"); a hard `max` / `value` target can't pass on a lower bound: the row fails "at least; the target can't be verified while items are unpriced" until they are priced in the catalog or accepted by hand (a soft target keeps the priced part's verdict with the same note; a `min` is confirmed by a lower bound). The catalog's bearings, bushings, rod, clips and glue stay unpriced: their vendors' pages don't give a price to a fetch, and nothing was invented |
| 4 `quick` doesn't price | annoying | `4696993` | `budget.cost_floor_usd` at `quick`: the servos, a spool, a sheet, the robot's cement and inserts from the catalog ($115.70 for this goal, every one a whole pack), and a floor already over a `max` fails `budget.cost_usd` before any build |
| 5 checked with the `single` module | annoying | `4696993` | the `verified` text says "its single module plans in 12 layers (36 mm); the quad module's own plan is not checked here (plan the derived design: the planner's deadline is 60 s, and a bigger module stacks taller)". The design's own plan is not attempted in the check: the TrotBot quad's takes the whole deadline, which would double every failed check |
| 6 the worked example on the robot | annoying | `4696993`, `96ba6b8` | the real cause: a build reloaded from the store's STEP files had no joints or outlines (a fresh build has them); they now come from the side's template at the build's angle, so `design.mech.body("L.b2_leg0").outline` works after a reload as after a build; API.md says the example is one side and the robot's bodies are `L.b7` / `R.b7` |
| 7 `mass_g` / `volume_mm3` stale | annoying | `4696993` | measured on the live solid (`Part.density`, `fixed_mass_g` for the servo), so they follow an edit at once; an accepted `recheck` refreshes the build report's masses |
| 8 CLI `explain` without build options | annoying | `4696993` | `spiderpig explain` takes `--servo`, `--pillar`, `--pin`, `--crank`, `--sheet`, `--thickness` (the README says so) |
| 9 cards alike | nit | `4696993` | `list_designs` cards carry `params` (the spec's overrides), `servo`, `sheet`, `thickness_mm`, `constructions` and `targets` |
| 10 the bake prints | nit | `4696993` | `ExportReport.warnings` (and the manifest) carry the bake's and the constructions' warnings; build123d's "Unknown Compound type" warning is silenced |
| 11 "quiet" | nit | not fixed | out of the vocabulary; the guide's limits say what v1 doesn't cover |

Not fixed, and why:

- The pivot hardware's prices. The row is now honest instead: with bearings or
  bushings the goal's "under $120" is **not verifiable** in this catalog, and the
  priced part alone is $115.67 (two XL330s, a spool, the cement, a pack of inserts),
  so the 64 bearings, the rod, the clips and the glue put it over. That is the answer
  the user gets now, at `verify standard`, in one row.
- Six legs. A true limit of the modules (1, 2, 2, 4 a side); the error says so.
- A parameter search (`sweep`). Not the top friction this round: both blockers were
  reports that hid a fact (a lever, a lower bound), and the numbers the goal asked
  for (clearance, lift, speed) were met by the first Strider without a search.
- The design's own plan in a recommendation's check (entry 5). Said plainly rather
  than attempted, to keep a failed check inside one deadline.

### Wall clock, second run

Against the fixed tree, a fresh store, the same scripts and the same order; the
full unit suite (and, for the first minutes, the Klann audit) ran alongside on the
same 4 cores, so the engine steps read a little slower than alone.

| step | wall | note |
|---|---|---|
| 1 import, `list_linkages`, `describe(klann)` | 3.8 s | |
| 2 the 15 spec probes | 3.4 s | the module error now says legs per side |
| 3 goal spec at 2 mm: check, plan, walk, quick | 4.2 s | `construction / unbuildable` with the least pitch (2.9 mm), the lever and a checked patch; `resolve` warned |
| 4 recommend, derive with its patch, check, plan, quick | 4.4 s | 12 layers, optimal; was four probes and a guess |
| 5 the derived Klann quad: verify standard | 54.0 s | `budget.cost_usd` FAIL: at least $115.67, the 64 bearings, 56 clips, rod and glue named |
| 6 Strider double (four legs): quick 4.9 s, standard 48.5 s | 53.4 s | `budget.cost_floor_usd` $115.70 at quick; the same honest FAIL at standard |
| 7 export step/dxf/print/bom/glb | 44.4 s | the bake's warning on the report |
| 8 the example edit on the loaded robot: build (reloaded) 4.7 s, edit, recheck 5.9 s | 14.2 s | `outline` and `joints` present after the reload; mass 5.907 -> 5.882 g at once; the build report follows |
| 9 TrotBot heel at unit 7: check 7.2 s, recommend, derive, plan 61.5 s, quick, explain | 76.8 s | the recommendation says "its single module plans in 12 layers ... the quad module's own plan is not checked here" |
| 10 CLI `explain` with the build options 4.7 s; CLI `build` 38.7 s; CLI `audit` 73.9 s | 117 s | |
| 11 `spiderpig view` to the URL 3.0 s; browser: page + GLB 9.5 s, drive mode + HUD 33 s | 45.5 s | the export served as is; HUD height 63.3 mm, contacts 4 / 8, margin 53.9 mm, model 172.82 mm/rev · 296.68 mm/s |
| **total, goal to files, the viewer and the TrotBot answer** | **422 s** (7.0 min) | no workaround, no dead end; ~8.5 min with three in the first run |

Entry 12, found in this run and fixed in `c2a8838`: the design derived with the
checked patch (`thickness_mm: 3.0`) had another id than the one resolved with
`thickness_mm: 3`, and both another than the plain default, though all three are one
config; `resolve` now drops a thickness at the sheet's nominal and keeps a measured
one as a float.

Read as the agent: the 2 mm wall is now two calls (`check` says 2.9 mm and hands
out 3, `derive` takes it) instead of four probes and a guess; the cost row answers
the budget question honestly (FAIL, at least $115.67, the bearings named) where it
used to say ok; the `explain` CLI shows the design being built; the example edit
works on the robot straight from the store, and the part's mass moves with it. What
it still has to do by hand: accept that six legs and a verified $120 with metal
pivots are not on offer, and choose four (Strider) or eight (Klann) legs.

## Round 3 — 2026-09-30

Three goals, three surfaces. *Goal 1 (MCP)*: "a single-servo straight-line mechanism
for a small pick-and-place: a point that travels at least 50 mm along a line straight
to 0.5 mm, laser-cut, with a printed crank; which building block fits, what stroke and
straightness does it really give, build it and export STEP and DXF." *Goal 2 (CLI)*:
"the default Klann quad with M3 bolt pivots instead of printed snap pins, on the
thicker 5 mm acrylic if the catalog has it, audited; the stack thickness and part
count changes versus the default." *Goal 3 (Python API)*: "a Jansen quad that must fit
a side stack of 30 mm and weigh under 400 g, ground clearance at least 80 mm" —
follow the failures and recommendations the way the reports suggest, `derive` with
their patches, say what an agent would conclude.

Driver: the MCP server as a subprocess (`python -m spiderpig.cli mcp --store <tmp>`
from the worktree, the official `mcp` 2.x client); `PYTHONPATH=. python -m
spiderpig.cli build|audit|explain|view`; `spiderpig.api` in a Python session with a
scratch store; headless Chromium for the viewer. Allowed reading: `README.md`,
`docs/agentlib/API.md`, this file's rounds 1 and 2, `--help`, the guide, cards and
catalog. Several engine runs overlapped on the 4 cores, so a few wall clocks below
read slower than alone (noted where it matters).

### Wall clock, first run

| step | wall | note |
|---|---|---|
| G1 connect, tools, resources, prompts, guide | 7.4 s (2.1 s in session) | the server's imports |
| G1 `list_linkages(mechanism)`, `catalog`, 11 `describe` cards | 0.2 s | the cards answer the stroke question |
| G1 Hoecken: resolve, check, plan, quick, explain, standard (job), export step + dxf, view | 16.9 s | standard 11.9 s (the worker's start), export 1.3 s, view 3.0 s |
| G1 `spiderpig view` from a shell: to the URL 6.5 s, page + GLB 5.8 s | 12.3 s | no console errors |
| G1 probes: Peaucellier / Watt at a stroke of 50, stroke 80, dwell, sides 2, misuse, prompts, cards; then the same at defaults | 12 s | entries 1, 2, 3 |
| G2 `build --list`, `explain` default quad | 7.4 s | no 5 mm sheet |
| G2 `explain` bolt pillars + pins; the same at `--thickness 5`; bolt pillars only | 108 s, 116 s, 45 s | all three: no plan in the budget (entry 7) |
| G2 `explain` bolt single 3.5 s; bolt pins only, quad 4.5 s | 8 s | 8 layers / 24 mm; 13 layers / 39 mm |
| G2 `build` default 107 s, `build --pin bolt` 83 s (both contended) | 190 s | |
| G2 `audit --pin bolt --modules quad` | 65 s | OK, 197 parts |
| G2 view the bolt-pin quad: API resolve 2.5 s, `spiderpig view` to the URL 27 s, page 5 s | 35 s | a rebuild of what `build` had built (entry 10) |
| G3 import 2.7, resolve, check 0.5, plan 2.0, walk, quick, recommend, explain | 6.4 s | stack and mass fail; `recommend` `[]` |
| G3 `describe(jansen)` + derives: XL330 2.7 s, decker 0.9 s, double 0.4 s, unit 1.3 65.6 s (deadline) + recommend + explain | 73 s | |
| G3 plywood 3.0 s, plywood + XL330 2.8 s, standard verify 65 s, mass by group | 74 s | |
| G3 two more standard verifies for the mass entry | 71 s each | |
| **total, three goals** | **~15 min** of wall clock | G1 ~1 min, G2 ~10 min (4.5 of it the three bolt-pillar plans that fail), G3 ~5 min |

### Entries

#### 1. Two of the three straight-line building blocks fail every tool after `resolve`, even at their defaults, as `store / bad_design_id` — **blocker**

Tried: `resolve({"kind": "mechanism", "linkage": {"key": "peaucellier_crank", "params":
{"unit": 18}}})` (the card says 45.9 mm at unit 16, so 18 for 50 mm): `ok: true`, an id,
no warning. Then `verify(quick)`, `check`, `explain`: every one `isError` with
`{"stage": "store", "code": "bad_design_id", "message": "proportion yy is a length and
must be > 0, got -1.75", "notes": ["a design id is the 16 hex digits resolve
returned"]}`. `yy: -1.75` is the linkage's own default (the card lists it; `yx: 3, yy:
-1.75` place the fixed pivot Y). The Watt crank the same: `proportion ax is a length
and must be > 0, got -4`. Both fail **at their defaults too** (no `params` at all);
`get_design(summary)` reads fine, so the record is there and the id is right. Giving
`yy: -1.75` explicitly is refused by `resolve` ("must be > 0"), so the negative default
is written into `resolved.json` on the way in and rejected on the way out. Only the
Hoecken (all lengths positive) works; the mechanism cards' `output_check` (computed
from the same defaults) is fine, so the engine can evaluate them.
Expected: the three line mechanisms of the catalog to be usable; a signed coordinate
to be a coordinate, not a length; and whatever failed to be reported as its own stage,
not as a malformed id. What happened: 6 dead tools, a message that points at my id,
and no way through for the exact-line mechanism the goal might have preferred.
Recoverable from docs and messages alone: no.

#### 2. A mechanism that misses its stroke target gets nothing from `recommend` or `explain`, and its card has no sensitivity table — **annoying**

Tried: the Hoecken with `motion.stroke_mm: {min: 80, hard: true}`. `verify(quick)`:
`ok: false`, the row `motion.stroke_mm: 66.89 >= 80 FAIL`, `score: 1.0`. `recommend`:
`{"stage": null, "recommendations": [], "notes": []}` in 0.0 s. `explain`: the
program, the static facts, the plan; not a word about the target. `describe(hoecken)`
has `scale_params: ["unit"]` and the `output_check` at the defaults, but no
`sensitivity` (the walker cards have one), so it doesn't say the stroke is linear in
`unit`. I scaled by hand: `unit: 16 x 80 / 66.89 = 19.2` -> stroke 80.27 mm, straight
to 0.077 mm, 6 layers, in 0.5 s. Right, but a guess.
Expected: a missed target that a scale parameter meets to come back as a checked
recommendation (`unit 16 -> 19.2: the stroke scales with unit; checked: 80.27 mm`), or
at least a note ("the target is missed, not a stage: stroke scales with `unit`"), and
the mechanism cards to carry a sensitivity line (stroke, straightness, extent per +10 %
of each parameter) as the walkers do.
Recoverable: yes (the card names the scale parameter).

#### 3. A `dwell_deg` target on a line output resolves, then reads `pass: false` under `ok: true` — **nit**

`motion.dwell_deg: {min: 90}` on the Hoecken: `resolve` ok; `verify(quick)` `ok:
true`, `unverified: ["motion.dwell_deg"]`, and the row `motion.dwell_deg: null >= 90
pass: false, "not measured by this design"`. The card says the output has `dwell:
null`, so the target could have been refused at `resolve` ("hoecken's line output has
no dwell; its metrics: stroke_mm, straightness_mm, on_line_fraction"), and a row that
is unverified shouldn't also say it failed.

#### 4. A mechanism's `check` names "the centre plates (the chassis between the servos)" as its lowest body part — **nit**

`check` on the one-sided Hoecken: `ground_clearance_mm: null`, `lowest_body_part: "the
centre plates (the chassis between the servos)"`. A mechanism has no feet, one side
and no chassis; the field should be null with the clearance, or name the real lowest
shape.

#### 5. The viewer shows a mechanism in the walker's chrome — **nit**

`view` (and `spiderpig view`) rendered the Hoecken (mode `side`, the output point's
path as the red curve, no console errors), inside the Drive panel (tank / arcade,
speed, R − L phase), the walker's HUD (yaw rate, contacts, slip, margin) and a mode
dropdown offering `double`, `decker`, `double double`. Nothing shows the output's
line, stroke or straightness. Cosmetic, but the page doesn't know what it is showing.

#### 6. `export` answers `warnings: null` — **nit**

The Hoecken's `export(["step", "dxf"])` result has `warnings: null`; API.md and the
output schema say a list. One `or []` in the client.

#### 7. Bolt pillars on the Klann quad: no plan within the budget, three times, 45-116 s each, and the levers "not checked, the 60 s ... ran out" — **blocker**

Tried: `spiderpig explain --module quad --pillar bolt --pin bolt` (the literal goal:
bolts for pins and pillars). 108 s: `STOP: klann_quad: no layer plan found with up to
41 layers after 60001 search steps in 44 s; the 60000 search-step budget ran out;
what blocked it: 9800 x pin:E_leg1 head vs a frame plate; 5126 x pillar:A_leg0: no
standard M3 screw for its 99 mm stack (needs 104.0 to 108 mm); 4529 x ... 96 mm
stack; 3991 x pillar:B_leg1 head vs b1_leg1: 0.9 mm apart in one layer, need 1.0;
...; sizes: 3-14 layers ruled out (1 s in all); 15-33 layers left open at their
budget (38 s in all); 34-40 not tried; 41 left open`, then `not checked, the 60 s for
checking what would clear it ran out: a scale of the linkage from OA 61.5 up and
thinner parts`. The same with `--thickness 5` (116 s; "no standard M3 screw for its
165 mm stack") and with bolt pillars alone (45 s; `no_plan_in_budget`, "levers left: a
bigger scale of the linkage, another module, another pillar or pin construction").
Bolt pins alone plan in 4.5 s (13 layers, 39 mm); the bolt single in 3.5 s (8 layers,
24 mm, the nuts 3 layers outside the outer plate). So the quad with bolt pillars is
either impossible or beyond the budget, and nothing says which.
What the tally shows: a bolt pillar clamps both frame plates, so its screw spans the
whole stack, and the longest stock M3 bounds the stack; yet the search spent its
budget on sizes 15-33 (45-99 mm, 165 mm at 5 mm), where every layout dies on "no
standard M3 screw". Expected: the pillar's screw bound to rule those sizes out before
the search ("a bolt pillar's stock M3 (up to xx mm) allows at most N layers"), so the
budget goes to the sizes that could plan and the verdict is proven ("no plan in N
layers or fewer") instead of a timeout; then a checked recommendation ("bolt pins with
printed pillars: 13 layers, 39 mm"), which the message half-names as "another pillar
or pin construction".
Recoverable: yes, by trying the constructions one at a time (the message's hint),
after 4.5 minutes of failed plans.

#### 8. No 5 mm sheet; `--thickness 5` on the 3 mm sheet is taken without a word — **nit**

`build --list`: `acrylic_3mm`, `plywood_3mm` only, so "the thicker 5 mm acrylic if
the catalog has it" is answered (it doesn't). `explain --thickness 5 --sheet
acrylic_3mm` ran without the warning the API's `resolve` gives beyond 12 % off the
nominal (round 2, entry 2's fix); the CLI should say the same.

#### 9. The bolt variant "costs the same": every M3 item is unpriced — **annoying**

`build --pin bolt`: `14 items to buy, est. $100.69 (9 without a listed price)`; the
default: `11 items, est. $100.69 (6 without a listed price)`. The 24 M3 x 12 SHCS, 24
nylock nuts and 24 washers the change adds are all unpriced (McMaster links,
"unverified"), so the honest answer to "what does the change cost" is "at least $0
more". The standing item from rounds 1 and 2; it bites again on the one comparison the
goal asked for.
Recoverable: the BOM names the unpriced items; the number is not there.

#### 10. A CLI build can't be viewed — **annoying**

`spiderpig build --pin bolt --out ...` wrote STEP, STL, DXF and the BOM (83 s); to
look at it, `spiderpig view` wants a stored design id, `mise run view`'s query string
carries no constructions, and `build` records nothing in the store. I resolved the
same config through the API (2.5 s) and `spiderpig view`ed that id: 27 s to build and
bake again what `build` had just built. Expected: `spiderpig view` to take the build
options (`--linkage --module --pin ...`, the same parser as `build` and `explain`)
and resolve them into the store, or `build --view`.

#### 11. The Jansen quad's 30 mm stack: the rows say FAIL and proven optimal, nothing says the target is out of reach for any walking module — **annoying**

Tried: the goal spec at the defaults. `verify(quick)` in 6 s: `size.stack_mm: 48 mm
vs <= 30 [FAIL, proven]`, `plan.optimal: True ... no plan in 15 layers or fewer`,
`recommend -> []` (no note), `explain` silent on targets. To learn whether 30 mm is
possible I derived the other modules: `double` 27 mm (fits) and `decker` 39 mm, both
`stride_mm ~ 1e-12` with the walk note ("every foot stays on the ground"); the card
says only `quad` walks. So a walking Jansen is 48 mm a side and proven so (and no
walker in the catalog is under 36 mm, the Klann quad). That took three derives (1.3 s)
and the reasoning was mine. Expected: a hard row that fails on a *proven* value to say
so in its detail ("proven the thinnest for this module and sheet; a `double` plans in
27 mm but does not walk"), or `recommend` to note it.
Recoverable: yes.

#### 12. The quick mass estimate reads 9-24 % high, and neither tier says what the mass is made of — **annoying**

`size.mass_g` at `quick`, "the walk model's nominal mass": 598.4 g at the defaults,
524.4 with the XL330, 471.9 on plywood, 397.9 on plywood + XL330. Measured at
`standard`: 550.3, —, 417.0, 319.9. The bias is one way (high) and up to 24 %, so a
`max: 400` target read on the quick row rejects designs that pass built (plywood +
XL330 passes on both only because the estimate lands 2 g under). Nothing before or
after a build says what weighs: I summed `design.parts[*].mass_g` by group myself
(links 258 g of acrylic, servos 115, chassis 77, crank 40, frame 31, pins 22,
pillars 8; on plywood the links are 147 g). Expected: an estimate that follows the
plan (every part's dims and layers are known after `plan`) or a row that says its
bias, and a `detail` that lists the groups as the cost row lists items, so the agent
knows the lever (the sheet's density and the servo, not the pins).
Recoverable: yes, by building.

#### 13. `recommend`'s checked text no longer says which module it checked — **nit**

`unit 1.3`: `plan / no_plan` after 65.6 s, the recommendation `unit 1.3 -> 1.6 ...
checked: the static stage passes, and it plans in 16 layers (48 mm)`. 16 layers is the
quad's own plan (cached from the parent design), which is what round 2's item asked
for; but the text no longer says whether it is the quad's or the single's.

#### 14. The constructions' warnings still print to stderr in a Python session — **nit**

`verify(standard)` printed 16 lines of `pin:B_leg0 seg0: its snap prongs (5.1 mm)
strain 6.0 % while snapping (want at most 4.0 %)` (each twice) to the terminal while
the same eight sit on `build.warnings`. Round 1's entry 18 put them on the report; the
print stayed.

#### 15. Good — no entry

The spec errors (a mechanism with `sides: 2` -> "one side only (it has no feet to
walk on)"; a walker metric on a mechanism -> the mechanism's six; `crank: bolt` ->
allowed `printed`); the card answering the stroke question before any design; the
whole Hoecken loop in 17 s; the walk note on `double` / `decker`; the checked scale
recommendation on `unit 1.3` (the quad's own plan this time); `compare`; the audit
(197 parts, OK, 65 s); the plan tables, which made the stack delta a one-line read
(36 -> 39 mm: the nut end claims two layers); the viewer on both designs.

### What I ended up with

**Goal 1.** The Hoecken straight-line four-bar (`hoecken`, design `5fd91d2581c6b4c7`):
at its default `unit 16` the point P travels **66.9 mm** along its line, straight to
**0.064 mm** over crank 90°..270° (on the line for 51 % of the turn; the return is a
14 mm-high arc), printed pillars, pins and crank on 3 mm acrylic, 6 layers (18 mm),
20 parts, 80.7 g, one sheet, $56.48 estimated. `verify standard` all green in 12 s;
`hoecken.step`, `laser/hoecken_sheet_0.dxf` + parts.csv exported in 1.3 s; viewed.
Scaled to `unit 19.2` it gives 80.3 mm at 0.077 mm. The Peaucellier (exact line, 45.9
mm at unit 16, 51.6 at 18) and the Watt crank (32 mm) could not be checked (entry 1).

**Goal 2.** No 5 mm sheet in the catalog, so 3 mm acrylic. Bolt *pins* with printed
pillars (`--pin bolt`): 13 layers / **39 mm** a side against 12 / 36 mm (+1 layer: a
nylock claims two layers where a printed cap claimed one); printed parts **90 -> 42**
(16 -> 15 distinct: the 48 pin segments become 24 M3 x 12 screws, 24 nylocks, 24
washers), laser-cut parts **39 -> 39** (8 different, 2 sheets, no spacer rings in this
layout), PLA 95 -> 84 g, items to buy 11 -> 14, estimate $100.69 -> $100.69 (9
unpriced: entry 9). `audit --pin bolt --modules quad`: OK, 197 parts, 0 clashes, 0
contract, 65 s. Bolt *pillars* too: no plan in the budget (entry 7).

**Goal 3.** At the defaults the Jansen quad clears 116.9 mm (the crank's sweep is the
lowest point; 80 needed), stacks **48 mm** a side (proven optimal; 30 asked) and
weighs 550 g built (400 asked). The stack can't be met by any walking module (entry
11); the mass can: plywood + XL330 gives **319.9 g** measured (design
`8d79c9a1329d33a3`: 16 layers, clearance 116.9 mm, stride 150 mm/rev, 258 mm/s,
$91.84 with 7 unpriced, 3 sheets, all checks green but the stack row). Scaling down
(`unit 1.3`) loses the plan and the engine sends it back to 1.6. What an agent would
conclude: drop the 30 mm stack (48 mm is the floor for a Jansen that walks; 36 mm for
the Klann quad), take plywood and the XL330 for the mass, keep the clearance for free.

### Phase 2 — what was fixed, and the second run

Commits: `6682a4c` (engine: signed coordinates, the bolt pillars' stack bound and a
full-effort pass on the sizes a quick search leaves open, printed pillars and a target's
scale as checked levers, the nominal mass by what it is made of, a target on an output
metric the mechanism lacks refused), `583f359` (the API and reports: `advise` on the
targets, the mass rows' composition, the stack's floor, `spec_of` and `spiderpig view`
by build options, the CLIs' thickness warning, the export's warnings over MCP, the
misuse code of a stored design that doesn't validate), `55e4614` (the catalog's
prices, with sources), `f445482` (API.md, the guide, the README, CLAUDE.md), `91eced9`
(`explain` warns for the robot's spec). Tests for each in `tests/test_spiderpig_api.py`,
`tests/test_spiderpig_mcp.py`, `tests/test_view.py` and `tests/test_bom.py` (the section
"Test drive, round 3").

| entry | severity | fixed in | how |
|---|---|---|---|
| 1 Peaucellier / Watt die after `resolve` as `bad_design_id` | blocker | `6682a4c`, `583f359` | the engine already made a parameter with a non-positive default a *real* symbol (a coordinate); `Linkage.signed` names them, and `config` and `spec` validate them as such, so `yy: -1.75` and `ax: -4` resolve, check, plan and reload; the card marks `signed`; a stored design whose values don't validate is `spec / bad_parameter`, not a bad id |
| 2 no recommendation for a missed mechanism target; no sensitivity on the card | annoying | `6682a4c`, `583f359` | `api.advise` (the MCP `recommend`): when every stage passes, stage `target`: a missed stroke, straightness or lift is met by `recommend.target_scale`, the least practical scale, measured again and planned before it is offered (`unit 16 -> 19.5: checked: stroke_mm 81.53; it plans in 6 layers`); `explain` ends with `4. targets`; a mechanism's card carries a sensitivity table of its output's numbers |
| 3 a `dwell_deg` target on a line output | nit | `6682a4c` | refused at `resolve`: "hoecken's line output has no dwell_deg; its metrics: stroke_mm, straightness_mm, on_line_fraction" |
| 4 a mechanism's lowest body part | nit | `583f359` | empty with the clearance |
| 5 the viewer's chrome for a mechanism | nit | not fixed | cosmetic, a viewer change; the page renders the mechanism and its output path |
| 6 `export`'s `warnings: null` | nit | `583f359` | `ExportOut` declared no `warnings`, so the SDK stripped them; declared, `[]` |
| 7 bolt pillars on the quad: 45-116 s to "budget ran out", levers unchecked | blocker | `6682a4c` | a group may bound the stack (`Group.max_top`): a bolt pillar's longest stock M3 (50 mm) clamps 15 layers of 3 mm, `side_problem` caps the sizes searched and the failure names the bound; the sizes a quick pass leaves open get the full effort, so the verdict is proven: 108 s and "left open" -> 2 s and "3-15 layers ruled out"; `recommend` checks printed pillars ("plans (quad module, the design's own) in 13 layers (39 mm)") and a construction change maps to a `constructions.*` patch |
| 8 `--thickness 5` taken without a word | nit | `583f359`, `91eced9` | `build`, `explain` and `view` print `resolve`'s warnings on stderr |
| 9 the bolt variant "costs the same": M3 hardware unpriced | annoying | `55e4614` | Bolt Depot's M3 socket caps (6, 8, 12, 16, 20, 30, 45), button heads (6, 10), hex nut, nylock and washer; TME's bushing; an Amazon MF63ZZ 10-pack; Woodcraft's Starbond CA and Woodpeckers' plywood per sheet (both pages fetched, `verified`); Home Depot's Titebond. Bolt Depot, TME, Amazon and Home Depot refuse a fetch, so those prices are what a web search quoted from the page on 2026-09-30, and the offer's note says so. The 3 mm rod, the starlock kit, the M2 tapping kit and M3 x 18 / x 50 got no price from any page and stay unpriced. The pin-bolt quad now costs $10.85 more than the default (24 screws, nylocks and washers) with 2 unpriced items instead of 9 |
| 10 a CLI build can't be viewed | annoying | `583f359` | `spiderpig view --linkage klann --module quad --pin bolt` (the build options, the same parser as `build`) resolves them into the store through `api.spec_of(config)` and serves the design: 25 s to the URL |
| 11 a proven stack miss without a word | annoying | `583f359` | the row's detail: "48 mm is proven the thinnest for jansen's quad module on 3 mm layers (16 layers ...); a thinner stack needs fewer legs a side, and no module of jansen with fewer walks (single, double, decker stand still ...); the sheet sets the layer pitch"; `advise` and `explain` carry the same |
| 12 the quick mass 9-24 % high, no breakdown | annoying | `6682a4c`, `583f359` | `walk.nominal_mass_breakdown`: links as pills with the joint discs counted once and the holes taken out (within 1 % of the built links), the plates at the sheet's density and the printed parts by pivot and pin counts, refitted to the four default Klann robots (within 1 %; the crank by its distinct crankpins, the pillars by pivot and leg); the Jansen quad reads 552.8 vs 550.3 g built (was 598), on plywood 413.0 vs 417.0 (was 471.9), with the XL330 339.0 vs 319.9 (was 397.9); the quick row's detail says what it counts, the measured row lists the mass by group |
| 13 which module the recommendation checked | nit | `6682a4c` | "plans (quad module, the design's own) in 16 layers (48 mm)" |
| 14 the constructions' warnings on stderr | nit | `583f359` | `capture_warnings` stops their propagation while they go on the report |

Not fixed, and why:

- The viewer's chrome (entry 5): the page shows a mechanism inside the walker's
  panels; a viewer change, out of this round's scope.
- The planner's 60 s deadline on a scaled-down multi-leg design (goal 3's `unit 1.3`:
  65.8 s to `no_plan`, unchanged): the search space is real and there is no knob; the
  recommendation it comes back with is now checked with the design's own module.
- A `sweep` operation (the standing item b): not the top friction this round either.
  The blockers were an engine bug (the signed coordinates) and a search that spent its
  budget on stacks no screw could span; the one search a goal needed by hand, the
  stroke's scale, is now `advise`'s. Goal 3's five derives (servo, modules, unit, sheet)
  stay by hand: the reports now say why each lever moves what it moves.
- Two facts the prices made visible rather than caused: the engine's CA glue estimate
  (0.02 bottle per glued anchor: 1.16 bottles on the Jansen quad, so two $13.99
  bottles) and the M3 hardware lift every total (the default Klann quad's estimate is
  $138.86, not $100.69). They are the honest numbers of the catalog as it stands.

### Wall clock, second run

Against the fixed tree, fresh stores, the same scripts in the same order; goals 1 and
3 ran side by side, goal 2 after them, nothing else on the cores.

| step | wall | note |
|---|---|---|
| G1 connect, tools, resources, prompts, guide, 11 cards, catalog | 8.4 s (2.6 s in session) | |
| G1 Hoecken: resolve, check, plan, quick, explain (with `4. targets`), standard (job 9.7 s), export step + dxf (1.1 s, `warnings: []`), view | 22.3 s (14.6 s in session) | |
| G1 `spiderpig view` from a shell: to the URL 7.0 s, page + GLB 5.2 s | 12.2 s | no console errors |
| G1 probes: Peaucellier at unit 18 and Watt at 25 verify quick `ok` in 0.2 s each; stroke 80 -> `recommend` hands out `unit 19.5` in 0.1 s; dwell refused at resolve; the three line mechanisms check and plan at their defaults | 12.2 s | entries 1, 2, 3 gone |
| G2 `build --list`, `explain` default quad | 8.2 s | |
| G2 `explain` bolt pillars + pins 6.5 s; at `--thickness 5` 5.2 s (the warning, the bound at 9 layers of 5 mm); bolt pillars only 7.0 s | 18.7 s | all three: a proven verdict and the checked lever "pillar bolt -> printed"; was 108 s, 116 s and 45 s of "budget ran out" |
| G2 `explain` bolt pins only, quad | 4.7 s | 13 layers / 39 mm |
| G2 `build` default 84.4 s, `build --pin bolt` 59.6 s | 144 s | $138.86 -> $149.71, 2 unpriced each |
| G2 `audit --pin bolt --modules quad` | 66.5 s | OK, 197 parts |
| G2 `spiderpig view --linkage klann --module quad --pin bolt`: to the URL 25 s, page 4.7 s | 30 s | resolved into the store as `1a89d6bb0b7de797` (the same id the API gives that spec) |
| G3 resolve, check, plan, walk, quick, recommend, explain | 6.9 s | the stack row and `explain` say the floor; the mass estimate 548.5 g (552.8 with the crank and pillar refit that followed) |
| G3 `describe` + derives: XL330 2.7 s, decker 0.9 s, double 0.4 s, unit 1.3 65.8 s (deadline) + recommend + explain | 74 s | the recommendation names the quad's own plan |
| G3 plywood 3.1 s, plywood + XL330 2.7 s, standard verify 55 s, mass by group | 70 s | estimate 334.6 vs 319.9 g built (339.0 after the refit) |
| G3 two more standard verifies | 57 s, 59 s | measured rows "by group: links 258 g, drive 115 g, ..." |
| **total, three goals** | **~10 min** of wall clock | G1 ~55 s, G2 ~4.6 min, G3 ~4.5 min; no workaround, no dead end; ~15 min with two dead ends in the first run |

Read as the agent: the three line mechanisms are all on offer now, and the goal's
"which fits" is a `describe` and three quick verifies (the Peaucellier at unit 18 is
the exact line at 51.6 mm; the Hoecken the fewest bars at 66.9 mm); a missed stroke
comes back with its scale, checked. The bolt quad's answer is a proven "no plan in 15
layers with bolt pillars" and "bolt pins with printed pillars: 13 layers, 39 mm" in
seven seconds, and the cost of the change is a number ($10.85, two items unpriced).
The Jansen's 30 mm is refused with the reason (the quad's floor is 48 mm, and nothing
with fewer legs walks); its mass estimate is within 1 % of the build, so the plywood +
XL330 choice is made at `quick`. What it still does by hand: the five derives of goal
3 (the reports say which way each lever moves, not how far) and the wait for the
deadline on a scaled-down quad.

Verdict on the loop: the surfaces are good enough to stop test-driving them. Every
entry of this round was a fact the reports hid or a bug, not a missing operation, and
the second run went through all three goals without a workaround. The single biggest
remaining problem is the engine's, not the surface's: a scaled-down multi-leg design
costs the planner's whole 60 s deadline per try with no knob and no proof, so the
iterate loop's one slow step is the one an agent must repeat.

## Round 4 — 2026-09-30 (the confirmation round)

Four goals, to check round 3's verdict with fresh eyes. *Goal 1 (MCP, the regression
of round 1)*: "a four-legged walker in a 300 × 200 × 150 mm box, at most $100 in
purchased parts, ground clearance ≥ 40 mm, as fast as possible; sheets, print files,
BOM, view". *Goal 2 (MCP)*: "the cheapest robot that actually walks (stride over 50 mm
per turn), any linkage, any module; tell me the bill and what drives the cost". *Goal 3
(Python API)*: take goal 2's design; `derive` it to the XL330 and to `rod` pins;
`compare` the designs; `recheck` after editing one link's solid; `verify("full")`
once and say what the MuJoCo rows add. *Goal 4 (CLI)*: "a single-servo lift whose
platform rises at least 30 mm and stays level to 1°": find the block from the cards,
build it, audit it, view it.

Driver: the MCP server as a subprocess (`python -m spiderpig.cli mcp --store <tmp>`
from the worktree, the official `mcp` 2.2 client, a fresh store per goal); `PYTHONPATH=.
python -m spiderpig.cli explain|report|build|audit|view`; `spiderpig.api` in a Python
session on goal 2's store; headless Chromium for the viewer. Allowed reading:
`README.md`, `docs/agentlib/API.md`, this file's rounds 1 to 3, `--help`, the guide,
the cards and the catalog.

### Wall clock, first run

| step | wall | note |
|---|---|---|
| G1 connect, tools, resources, prompts, guide | 6.3 s (3.4 s connect) | |
| G1 `describe(strider)`, `catalog`, resolve the goal spec (Strider double), quick | 7.7 s | clearance 22.6 (the servo's pad), cost floor $100.69 "already over" |
| G1 four derives + quick verifies (plywood; + shin 16; + unit 6.3; XL330 + shin 16) | 87 s | three Strider plans of 20-31 s each |
| G1 two more derives + quick (XL330 + shin 16 + unit 6.3; shin 15), a standard verify of the sts3215 candidate as a job | 58 s | z 205 and $122.75 on the sts3215 candidate |
| G1 final (XL330): standard verify 63 s, export step/dxf/print/bom/glb 135 s (jobs in parallel), view 4.6 s, browser 15 s | 163 s | one lost session on my client's key for `get_design` |
| **G1 total, goal to files and the viewer** | **5.4 min** of session wall (8.8 min with my pauses) | no workaround; two facts decided by hand (entries 1, 2) |
| G2 `list_linkages(walker)` + 17 `describe` cards | 15 s | the cards' `walks` and `stride_mm` name the candidates in one pass |
| G2 seven candidates: resolve + quick each, standard verifies as jobs | 193 s | sixbar quad's plan 60.6 s (the deadline); the standards 50-116 s |
| G2 `export(bom)` of the Strider double + `get_design(summary)` | 81 s | two totals (entry 3) |
| G2 pillar variants: `bolt` (fails to plan, 4 s, checked patch back to printed), `rod`, `rod` + `rod` pins: quick + standard | 66 s | the same $122.75 |
| **G2 total** | **5.9 min** | no workaround; the answer needed six builds (entry 1) |
| G3 import 2.4 s, load, three derives, three standard verifies (46, 30, 33 s), three compares | 111 s | |
| G3 build (reloaded, 4 s), the API.md edit, recheck 5.4 s | 9 s | the edit missed the solid (entry 5) |
| G3 `verify("full")` on the XL330 + rod design | 54 s | `sim / sim_failed` (entry 6) |
| G3 a second `verify("full")` on the sts3215 + rod design (for what the sim adds) | 65 s | four sim rows |
| **G3 total** | **4.0 min** | one dead end (the XL330 sim), one silent no-op (the edit) |
| G4 `explain` of the three lift blocks | ~25 s | the output line answers the rise and the level |
| G4 `report --linkages parallelogram_lift watt_table_lift` | 3 s | crash (entry 8) |
| G4 `build`: default (refused), `--module single` (refused), `--side-only` alone (refused), `--module single --side-only` (33 s) | ~50 s | three tries (entry 9) |
| G4 `audit --linkage parallelogram_lift` | 2 s | refused (entry 10) |
| G4 MCP cards of the three lifts 5 s; `spiderpig view --linkage ... --side-only` to the URL 25 s; browser 14 s; MCP `verify(standard)` as the audit 8 s + `explain` | ~60 s | |
| **G4 total** | **4.4 min** | one workaround (the audit through the API) |

### Entries

#### 1. The quick cost floor is $37 under the built cost: the constructions' glue, screws and the second sheet are known before a build but not counted — **annoying**

Tried (G1): the goal's Strider double at the defaults, `verify(quick)`:
`budget.cost_usd: 100.69 USD vs <= 100 [FAIL, estimated] a lower bound already over
the target: ... Feetech STS3215 x 2 $40.00; PLA ... 1 kg spool $25.49; 3 mm cast acrylic
$10.99 (a pack of 2); SCIGRIP ... cement $12.84; M3 x 5.7 brass heat-set insert x 4
$11.37 (a pack of 100); the pivots' hardware, screws, rod and clips are counted after
a build (verify standard)`. Derived to `plywood_3mm`: `budget.cost_floor_usd: 85.45
[ok] ... Baltic birch plywood $3.10; Titebond II ... $5.49; ...` and `unverified:
['budget.cost_usd']`. So I chose plywood on that number. `verify(standard)` of the
same design: `budget.cost_usd: 122.75 ... 11 items, the largest Feetech STS3215 x 2
$40.00; Medium CA (cyanoacrylate) glue, 2 oz x 1.16 $27.98; PLA ... $25.49; ... insert
x 4 $11.37; 3 unpriced ...`. The XL330 variant: `100.43 [FAIL] a lower bound already
over the target` at quick (I weighed 43 cents), `133.90` at standard. In G2 every one
of seven walkers read `cost_floor_usd: 85.45` at quick and `122.75`-`126.72` at
standard. What the floor leaves out is not the pivots' hardware (that is what its
text says it leaves out) but the CA glue of the printed pillars' anchors and the
robot's tie spigots ($27.98: two bottles), the crank's horn screws and nuts ($6.22)
and the second sheet ($3.10): all of it a function of the plan (pillars, ties, the
crank) that `quick` already has.
Expected: the floor to count what the constructions of the plan buy (a glue item per
anchor and spigot, the crank's screws and nuts), so that a quick verdict on the budget
is within a few dollars of the built one; and its text to name what it still leaves
out (the sheets' count, the pivots' hardware).
Recoverable from docs and messages alone: yes, after a 50-100 s standard verify per
candidate.

#### 2. Two bottles of CA glue for four tie spigots: the robot's glue estimate buys a second $13.99 bottle on every walker — **annoying** (an engine number)

The BOM's glue line (G2, `bom.md`): `1.16 | Medium CA (cyanoacrylate) glue, 2 oz |
... | 2 × 1 | $27.98 | pillar:J2_leg0 anchors, pillar:J6_leg0 anchors, tie spigots
into the inner frame plates`. The one-sided lift of G4: `0.08 | ... | 1 × 1 | $13.99 |
pillar:G1 anchors, pillar:G2 anchors` (two pillars, four anchors: 0.02 a piece, as
round 3 noted). The robot's four pillars are 0.16; the four tie spigots the other
1.00: a quarter of a 2 oz bottle (about 14 g of cyanoacrylate) each. That is the
second-largest item of every walker's bill (23 % of it) and no spec field moves it.
Expected: a spigot's glue on the same footing as an anchor's, so a robot needs one
bottle.
Recoverable: n/a (nothing to do; the number is the engine's).

#### 3. Two totals for one bill: the verify row says $122.75, `bom.md` says $116.55 — the plywood sheets are missing from the BOM — **annoying**

G2, the Strider double on plywood: `verify(standard)`'s `budget.cost_usd: 122.75`
(`budget.sheets: 2`); `export(bom)`'s `bom.md`: `Estimated purchase total: **$116.55**`,
and its Buy table has no sheet line at all, though its Laser-cut table lists 39 plywood
parts; `bom.json`'s `purchased` has no sheet item either. The acrylic lift's BOM (G4)
does list `1 | 3 mm (1/8 in) cast acrylic sheet, 12 x 12 in | ... | 1 × 2 | $10.99 |
laser-cut parts`. The difference is exactly two plywood sheets at $3.10.
Expected: one total; the BOM to carry the sheets whatever the stock.
Recoverable: yes (the row is right; the file is short).

#### 4. `get_design(design, stage)` answers under `report`, not under the stage's name — **nit**

API.md's table: "`get_design(design, stage?)` | `summary`, `spec`, `resolved`,
`check`, `plan`, ...". The result is `{ok, failures, design, stage, report}`. Cost one
crashed client session (`KeyError: 'verify'`) and the running job it was waiting on.

#### 5. On a build reloaded from the store, a part's `solid` sits in the world frame while `pose` reads the identity, so API.md's edit misses the link and `recheck` passes a no-op — **blocker**

Tried (G3): API.md's lightening hole on goal 2's Strider double, whose build the MCP's
standard verify had written (so `api.build(base)` reloaded it: "157 parts" in 4 s).
`design.mech.body("L.b2_leg0")`: `outline (('J3', 'J9'), ('J9', 'J10'))`, joints
`J3 [-76.9, 20.0, 0]`, `J9 [53.0, 22.7, 0]` (z = 0: the side's frame, as documented);
`design.side.plan.z(link.layers[0])` = `(6.0, 9.0)`. `link.solid.bounding_box()`:
`min (-82.9, 14.0, -78.5) max (59.0, 35.2, -75.5)`; `link.pose` = the identity;
`link.placed()` the same box. So the STEP solid carries the left side's world
placement (z −78.5..−75.5, the robot's chassis offset) and the example's cylinder at
z 7.5 cut nothing: `volume_mm3 4945.09` before and after, `mass_g 3.3627` both, yet
`edited: True` and `recheck: ok=True edited=['L.b2_leg0'] checked=['L.b2_leg0']`. The
docstring of `Part` says "``solid`` is the live build123d solid in the body's own frame
(for every part of a side that is the side's frame; ``pose`` places it in the world)":
true of a fresh build (round 2 tested it), not of a reloaded one, and a reloaded build
is what an agent gets after any standard verify or export in another process.
Expected: the reloaded part in the same frame as a fresh one (solid in the side's
frame, `pose` the side's placement), or the docstring and the example to say which
frame a reloaded solid is in and how to move the cut; and `recheck` to say when an
"edited" solid has the same volume as the build's.
Recoverable: no (nothing said the cut missed).

#### 6. `verify("full")` with the XL330 fails in the sim's mesher: `sim / sim_failed: 'NoneType' object has no attribute 'NbNodes'` — **blocker**

G3's XL330 + rod design (`65e97dc5f64f2b2e`): every row green through `budget`, then
`sim.run: failed [FAIL, measured] 'NoneType' object has no attribute 'NbNodes'`,
`failures: sim (sim_failed) ...`, `ok: False`. Round 1's entry 14 (the XL330's model
has three faces the mesher can't triangulate) was fixed for the bake ("meshes face by
face and skips what the mesher leaves out"); the MJCF's meshing of the same servo
still dies on it. With the sts3215 the sim runs (entry 7). Nothing in the row says it
is the servo's purchased model, or that swapping the servo would let the sim run.
Expected: the MJCF to mesh the purchased model the way the bake does (skip the faces
it can't, or use the servo's box), and a sim failure's message to name the part.
Recoverable: only by trial (the sts3215 works), as in round 1.

#### 7. What the sim adds, and two rows with one name — **nit**

`verify("full")` on the sts3215 + rod design (65 s): `motion.speed_mm_s = 164.9 mm/s
[measured] 4 s at the drives' full speed; the walk model's row above is the spec's`
(the walk model: 149.8 estimated), `motion.stride_mm = 190.4 mm/rev [measured]
forward travel per crank revolution in the sim` (the walk model: 172.8),
`sim.stays_up = True max tilt 4.2 deg`, `sim.torque = 0.212 N·m <= 1.9123 peak 0.212
N·m of 1.9123 stall`. Good numbers; but two rows carry `motion.speed_mm_s` and two
`motion.stride_mm`, told apart only by `source`, and `compare` keys rows by name.

#### 8. `spiderpig report` crashes on a mechanism — **annoying**

`spiderpig report --linkages parallelogram_lift watt_table_lift --no-walk`:
`IndexError: tuple index out of range` at `spiderpig/tools/report.py:37, foot_path:
_, joint = lk.feet[0]`. The CLI's way to compare the blocks compares walkers only;
`explain` and the MCP cards answered instead.

#### 9. `spiderpig build` on a mechanism: "is a mechanism, not a walker" and "unknown module 'quad'", three tries to `--module single --side-only` — **annoying**

`build --linkage parallelogram_lift --out ...`: `error: parallelogram_lift is a
mechanism, not a walker: it has no feet to walk on (walkers: klann, ...)`. With
`--module single`: the same. With `--side-only` alone: `error: unknown module 'quad';
have ['single']`. With both: the build (33 s: STEP, STL, 7 print STLs, one DXF sheet,
the BOM at $80.66). The API's `resolve` infers `single` and `sides: 1` for a mechanism;
the CLI's defaults are the walker's (`quad`, the robot) and its refusal names the
linkage's kind instead of the flags. The same defaults sit on `view` (`--module single
--side-only` needed there too).
Expected: the CLI to default a mechanism to its one module and one side, as the API
does, or the message to say "a mechanism is one side: pass --side-only (and --module
single)".
Recoverable: yes, by trial (the second message names the module).

#### 10. `spiderpig audit` refuses every mechanism — **blocker** (for "audit it" through the CLI)

`audit --linkage parallelogram_lift`: `error: argument --linkage: invalid choice:
'parallelogram_lift' (choose from 'klann', 'fourbar', ... 'trotbot_toe')`: the choices
are the walkers. The README says `mise run audit` "checks every module end to end";
nothing says a mechanism can't be audited. I ran the MCP's `verify(standard)` on the
design `view` had resolved into the store instead (26 parts, contract at two angles,
clashes, solids, plan re-verified: all green, 8 s).
Expected: `audit` to take a mechanism (one side, its `single` module) as `build`,
`explain` and `view` do.
Recoverable: yes, through the API or the MCP, not through the CLI.

#### 11. The output line doesn't name its axes — **nit**

`explain`: `output: parallelogram_lift: b4 (translation_platform) covers 1.62 x
32.00 mm, stroke 32.00 mm, turns 8.53e-14°`; the card: `extent_mm: [1.616, 32.0]`. That
the 32 mm is the vertical (the rise) and the 1.62 mm the sway had to be assumed (the
`hoecken_table`'s `76.48 x 13.99 mm` is the other way round). "covers 1.62 mm across x
and 32.00 mm along y" would settle it.

#### 12. The constructions' warnings still print to stderr, in the MCP server and in a Python session — **nit**

`pin:J11_leg0 seg0: its snap prongs (5.1 mm) strain 6.0 % while snapping (want at
most 4.0 %)` (and `seg1`, 6.9 %) printed by the MCP server process on every Strider
plan (they reach the client's terminal through the subprocess's stderr) and five times
in the G3 session's output, while the same lines sit on `plan.warnings` /
`build.warnings`. Round 3's entry 14 said `capture_warnings` stops the propagation; it
does on the report's path, not on these.

#### 13. The viewer's chrome for a mechanism — **nit**, standing

The lift renders (the platform, its two arms, the crank, the output path as the red
curve, no console errors) inside the walker's Drive panel, HUD and mode dropdown
(`double`, `decker`, `double double`); round 3's entry 5.

#### 14. `walks: true` at a 4 mm stride — **nit**

`describe(trotbot_toe).modules.single`: `stride_mm: 4.0, walks: true`; its `double`
29 mm, `walks: true`. The flag says "not zero", which is not what "walks" reads as; the
goal's "over 50 mm" filtered on the number, so no time lost.

#### 15. Good — no entry

The sensitivity table (`shin +10 %: lift +36 %, height +8 %`) named the parameter that
raises the body; the clearance row named the servo's pad, then the crank's sweep once
the XL330 lifted it; the z row read 205 before any build; the plywood floor answered
the sheet choice; the bolt-pillar variant came back in 4 s with the proven bound and
the checked patch back to printed; `derive` + `compare` traced every changed number
(a servo swap: mass 335.8 -> 238.5 g, speed 149.8 -> 296.7 mm/s, clearance 22.6 ->
31.5 mm, cost 122.75 -> 133.90); the three lift cards answered the rise and the level
before any design; `spiderpig view --linkage ... --side-only` resolved the CLI build
into the store and served it in 25 s; both viewer pages rendered without a console
error; 17 walker cards in 15 s.

### What I ended up with

**Goal 1.** Strider (coupled pair) `double`, two sides, XL330-M288, plywood, `shin
13 -> 16`, `unit 6.5 -> 6.3` (design `e5e84260d275aa3e`): four legs, ground clearance
**50.6 mm** (the crank's sweep is the lowest point), **169 mm/rev, 290 mm/s** at the
XL330's 103 rpm, built envelope **208 x 145 x 181 mm** (in the 300 x 200 x 150 box
with 150 up), 20 layers / 60 mm a side (proven), 241 g, 83 g of PLA, 2 sheets;
`budget.cost_usd` **$133.90 at least** (2 XL330 $54.98, CA glue 2 bottles $27.98, a
spool $25.49, 100 inserts $11.37, 3 unpriced packs of screws). The $100 is not
reachable by any servo, sheet or linkage in the catalog: the sts3215 on plywood is
$122.75 and its body sits 5 mm too wide (z 205) once `shin` lifts the clearance to 41
mm. The XL330 buys the clearance, the speed and the box for $11.15 more. Files in the
store's `exports/`: `strider.step`, `laser/strider_sheet_{0,1}.dxf` + `parts.csv`,
`print/*.stl` (21 distinct) + `parts.csv`, `bom.{csv,md,json}`, `strider.glb`,
`manifest.json`; viewed (`view`: 166 nodes, 330 tracks, the drive panel seeded from
the design).

**Goal 2.** Every walking design in the catalog costs the same, because the bill is
whole packs. Seven candidates (the four two-legs-a-side walkers of the cards: Strider
`double` 173 mm and `decker` 122 mm, TrotBot-toe `decker` 130 mm, TrotBot-heel
`decker` 69 mm; and three quads: Klann 102 mm, sixbar 66 mm, the LEGO Spot Micro v2
4-bar 61 mm) on plywood with the sts3215: `budget.cost_usd` **$122.75** (lower bound)
for five of them and $126.72 for the two whose extra screw is priced. The cheapest
that walks is therefore the leanest of the tie: the **Strider `double`** (design
`7ecb570b728dc83c`): 173 mm/rev, 150 mm/s, 16 layers / 48 mm, 336 g, 97 g of PLA, 2
sheets. Its bill (`bom.md`, $116.55 without the two sheets the row counts): 2 STS3215
**$40.00** (33 %), CA glue 2 x 2 oz **$27.98** (23 %, for "1.16 bottles": the tie
spigots, entry 2), a 1 kg spool of PLA **$25.49** (21 %, for 97 g), 100 heat-set
inserts **$11.37** (9 %, for 4), 2 plywood sheets $6.20, Titebond $5.49, 8 M3 x 6 SHCS
$3.83, 4 M3 nuts $2.39, and three unpriced packs of 100 (8 M2 x 6 self-tappers, 4 M3 x
16 BHCS, 4 M3 x 18 SHCS). What drives the cost: the packs (a spool, a pack of inserts,
two bottles of glue for a few grams each: $65 of the $123 buys stock that outlives ten
robots); the linkage moves only the spool's fraction. `rod` pillars or pins change
nothing priced; `bolt` pillars don't plan (15 layers is the bound; the engine hands
back printed).

**Goal 3.** From goal 2's design: `derive` to the XL330 (`7b604e4f099c991f`), to `rod`
pins (`9b92a91c4ab6bce1`), to both (`65e97dc5f64f2b2e`); `verify("standard")` on each
(46, 30, 33 s); `compare`: the XL330 is 97 g lighter (335.8 -> 238.5 g: drive 115 ->
39 g), twice as fast (149.8 -> 296.7 mm/s), clears 31.5 mm instead of 22.6 (the crank's
sweep replaces the servo's pad as the lowest point) and costs $11.15 more (122.75 ->
133.90); `rod` pins are 37 g heavier (the pin group 14 -> 48 g: steel rod and 48 clips),
print 14 g less, and cost "the same" (the rod and clips unpriced: 5 unpriced items
instead of 3); the stack is 48 mm in all four. The edit + `recheck`: 5.4 s, `ok`, but
a no-op (entry 5). `verify("full")` on the XL330 + rod design: `sim_failed` (entry 6);
on the sts3215 + rod design, 65 s: the sim adds a measured speed (164.9 mm/s against
the 149.8 estimated at the no-load rpm: 10 % faster), a measured stride (190.4 against
172.8 mm/rev), that it stays up (4.2° of tilt) and the peak torque (0.21 of 1.91 N·m
stall), and two more contract angles and a second clash angle, all green.

**Goal 4.** From the cards: `parallelogram_lift` (`b4 (translation_platform) covers
1.62 x 32.00 mm, stroke 32.00 mm, turns 8.53e-14°`), `watt_table_lift` (`covers 0.04 x
32.04 mm, straight to 0.0167 mm, turns 1.42e-13°`, 8 layers), `hoecken_table` (a
lift-and-carry: 66.9 mm along, 14 mm up). The parallelogram lift: **rises 32.0 mm**
(1.6 mm of sway) and **turns 8.5e-14°** (level to 1° by fourteen orders), 4 links, 7
layers / 21 mm, one servo. Built with `spiderpig build --linkage parallelogram_lift
--module single --side-only`: `parallelogram_lift.step/.stl`, 7 print STLs (10 parts),
one DXF sheet (6 parts), the BOM ($80.66: the servo $20, the spool, a bottle of CA, a
sheet, 8 items). Audited through the MCP's `verify(standard)` (the CLI refuses a
mechanism): 26 parts, contract, clashes, solids, plan re-check all green, 102 g, 121 x
132 x 59 mm. Viewed with `spiderpig view --linkage parallelogram_lift --module single
--side-only` (resolved into the store as `af8b33c63bb7b4b0`, 25 s to the URL; the page
rendered the lift and its output path, no console errors).

### Phase 2 — what was fixed, and the second run

Commits: `20dd70d` (engine: a tie spigot's glue, the shared face-by-face mesher
for the sim), `53259c0` (the API and reports: the quick cost floor, the BOM's sheets,
a robot part's frame and the recheck note, the sim's rows, the walks flag, the warnings,
the CLIs' mechanism defaults, audit and report on a mechanism, the output's axes; API.md,
the guide, the README, CLAUDE.md), and this round's log in the commit after them. Tests
for each in
`tests/test_spiderpig_api.py`, `tests/test_spiderpig_mcp.py`, `tests/test_view.py` and
`tests/test_robot.py` (the section "Test drive, round 4").

| entry | severity | fixed in | how |
|---|---|---|---|
| 1 the quick floor $37 under the built cost | annoying | `53259c0` | `verify.cost_floor` counts what the constructions buy whatever the parts' sizes: a bottle of CA glue with every pivot construction but `bolt` (the pillars' anchors, the inserts) and for the robot's tie spigots, and the printed crank's crankpin nuts (a pack); its detail says what a build adds (`FLOOR_LEAVES_OUT`: the sheets' count, the crank's screws, the pivots' hardware, rod and clips). The plywood Strider double's floor reads $101.83 against $108.76 built (was $85.45 against $122.75); the lift's $72.86 against $80.66 |
| 2 two bottles of CA for four tie spigots | annoying (engine) | `20dd70d` | `chassis.GLUE_PER_SPIGOT` 0.02 a spigot, as a pillar's anchor, instead of one bottle a robot: every robot's glue line drops from 1.16 to 0.24 bottles, one bottle, $13.99 less on every walker's bill |
| 3 the BOM without the plywood sheets | annoying | `53259c0` | `api.export` packed the sheets only when writing the DXF, and the sheet line with them; a `bom` export without `dxf` now packs to count them (no files written), so the BOM's total is the verify row's |
| 4 `get_design` under `report` | nit | `53259c0` | API.md's table says `{ok, failures, design, stage, report}` |
| 5 a robot part's solid in the world frame, the edit a no-op, `recheck` silent | blocker | `53259c0` | the real cause: a robot's parts (fresh or reloaded) sit in the world with the side's placement baked in (`assemble_robot` moves the solids, the joints stay the side's), and the docstring and example claimed the side's frame. `Part` carries `z_mid` (the robot's mid-plane) and `z_side` (its z range in the side's frame) and `Part.locate(xy, z=None)` gives the `Location` of a tool at the side's coordinates in the solid's frame, whichever side; `recheck.notes` names an edited part whose volume is still the build's; the docstring, API.md's frames paragraph and the worked example (`link.locate((a + b) / 2)`) say so |
| 6 `verify("full")` with the XL330 dies in the sim's mesher | blocker | `20dd70d` | `spiderpig/mesh.py`: the bake's face-by-face tessellation as one function; the MJCF's hulls (`mjcf._hull`) use it instead of build123d's `tessellate`, so the XL330's three untriangulated faces are skipped there as in the bake: the Strider double with the XL330 loads (36 bodies, 6 meshes) and runs (320 mm/s in the sim) |
| 7 two rows of one name from the sim | nit | `53259c0` | the sim's rows are `sim.speed_mm_s`, `sim.stride_mm` (still against the spec's target, informational), `sim.stays_up`, `sim.torque`; the guide and API.md name them |
| 8 `spiderpig report` crashes on a mechanism | annoying | `53259c0` | a mechanism's row carries `output` (the output check's numbers and text) instead of `foot`, plans its own modules only, and skips the walk |
| 9 `build` on a mechanism: three tries | annoying | `53259c0` | `--module` defaults to `None` and `config_from_args(robot=None)` fills both from the linkage's kind (`config.default_module` / `default_robot`): `spiderpig build --linkage parallelogram_lift` builds the one side; `bake`, `explain`, `view` and the server's query the same; the refusal of a robot of a mechanism says "it builds one side (robot=False; --side-only on the command line)" |
| 10 `audit` refuses a mechanism | blocker | `53259c0` | `--linkage` takes every linkage; a mechanism audits as its one module, one side (26 parts, OK, 17 s) |
| 11 the output's axes | nit | `53259c0` | "covers 1.62 mm in x by 32.00 mm in y (up)" |
| 12 the warnings on stderr | nit | `53259c0` | the leak was the standard verify's contract and clash angles (the parts realized again at each) in the job worker and in a session: wrapped in `capture_warnings` (they are the build's, already on `build.warnings`); `check` captures the static stage's onto `CheckReport.warnings` |
| 13 the viewer's chrome for a mechanism | nit | not fixed | round 3's entry 5: a viewer change |
| 14 `walks` at a 4 mm stride | nit | `53259c0` | `api.WALKS_MM` 20: the card's `walks` needs a stride of 20 mm a turn; the walk note says "a shuffle, not a walk: ... (a module walks from 20 mm a turn)" for a stride between 1 and 20 mm, and still lists the modules that walk |

Not fixed, and why:

- The viewer's chrome around a mechanism (entry 13), as in round 3.
- The bill itself. With one bottle of glue the cheapest walker is $108.76 at least
  (three packs of screws unpriced), and $65 of it is stock that outlives the robot
  (a spool for 97 g, 100 inserts for 4, a bottle for a few grams): the catalog's
  honest packs, not a reporting problem. The floor now says so before any build.
- The sheets in the quick floor: one sheet is counted; the count needs the layout
  (a build). The floor's text says so.

### Wall clock, second run

Against the fixed tree, fresh stores, the same scripts in the same order (the first
run's, unchanged, so the same ids); goals 1 and 2 ran side by side, then goals 3 and 4,
nothing else on the cores.

| step | wall | note |
|---|---|---|
| G1 discover; `describe`, `catalog`, resolve, quick | 5.9 s + 7.1 s | the floor reads $117.07 (acrylic): the glue and the nuts counted |
| G1 four derives + quick verifies | 88.0 s | plywood $101.83, XL330 $116.81 at quick |
| G1 two derives + quick, the sts3215 candidate's standard verify (job) | 57.5 s | built $108.76 (was $122.75): within $7 of its floor |
| G1 final (XL330): standard 63 s, export 126 s (jobs), view, browser | 152.3 s | built $119.91 (was $133.90) against a $116.81 floor: the second sheet |
| **G1 total** | **315 s** (5.3 min) | no workaround; 5.4 min in the first run |
| G2 17 cards | 14.5 s | |
| G2 seven candidates: quick + standard (jobs) | 193.6 s | every one $108.76 or $112.73 (was $122.75 / $126.72); the floor $101.83 for all |
| G2 `export(bom)` + summary | 74.3 s | `bom.md` **$108.76**, the same as the row (the sheets in) |
| G2 pillar variants (bolt fails with the checked patch; rod, rod + rod) | 51.9 s | |
| **G2 total** | **338 s** (5.6 min) | 5.9 min in the first run; the bill is one number now |
| G3 load, three derives, three standard verifies, three compares | 106 s | cost 108.76 -> 119.91 with the XL330 |
| G3 build (reloaded) 4 s, the first run's edit + recheck 5.6 s | 10 s | the first run's script, unchanged: its cut still misses, and `recheck.notes` now says so |
| G3 `verify("full")` on the XL330 + rod design | 54.4 s | **ok**: `sim.speed_mm_s 326.3 mm/s`, `sim.stride_mm 190.7`, stays up (4.2°), torque 0.12 of 0.52 N·m; was `sim_failed` |
| G3 `verify("full")` on the sts3215 + rod design | 52.5 s | `sim.*` rows beside the walk model's `motion.*` |
| G3 the edit through `Part.locate` on both sides + a wrong-frame edit + recheck | 14 s | 21.2 mm3 off each link; the note names the missed one |
| **G3 total** | **244 s** (4.1 min) | no dead end, no silent no-op; 4.0 min with one of each |
| G4 three `explain`s, `report` | 12 s | the output line names its axes; `report` covers the mechanisms (was a crash) |
| G4 `build --linkage parallelogram_lift` (no other option) | 9 s | was three tries |
| G4 `audit --linkage parallelogram_lift` | 19 s | OK, 26 parts (was refused) |
| G4 `view --linkage parallelogram_lift` to the URL, browser | 25 s + 14 s | the same design id as the first run; no console errors |
| G4 cards, MCP `verify(standard)` | 5 s + 16 s | |
| **G4 total** | **91 s** (1.5 min) | no workaround; 4.4 min with one in the first run |
| **all four** | **~16.5 min** of session wall | ~20 min in the first run, with two dead ends, one silent no-op and one workaround |

Read as the agent: the budget question is answered at `quick` now (the floor within
$7 of the built total, $3.10 of it the second sheet), and the bill has one number; the
cheapest walker costs $108.76 at least instead of $122.75 because the glue is counted
by the drop; a lift is `spiderpig build --linkage parallelogram_lift`, audited and
viewed by the same option; the full verify of the fastest servo runs and says what the
sim adds (10 % more speed than the no-load estimate, the torque margin, that it stays
up); and an edit that misses its part is named. What it still does by hand: choose the
XL330 over the $100 (nothing in the catalog reaches $100 with a spool, a pack of
inserts and two servos), and accept that a `double` Strider at 41 mm of clearance is
205 mm wide with the sts3215.

Verdict on the loop: converged. Round 3's verdict held for the surfaces it drove
(the spec, the reports, the cards, the MCP's failures) and this round's blockers were
all in the corners it hadn't reached: the Python handle on a *reloaded robot* build
(the frame), the *full* verify on the *XL330* (the sim's mesher), and the *CLI* on a
*mechanism* (audit, report, the defaults). Every one was a fact hidden or a path
untested, not a missing operation; the second run went through all four goals without a
workaround, and what remains is nits (the viewer's chrome around a mechanism) and the
catalog's honest packs. The single biggest remaining problem is the same as round 3's
and the engine's: the planner's 60 s deadline on a scaled-down multi-leg design (the
sixbar quad took its whole minute here too), with no knob and no proof.

## Round 5 — 2026-09-30 (the final confirmation)

Two jobs. *The regression*: rounds 2, 3 and 4's goals again, on the same paths their
logs describe (round 2's six-legged walker on 2 mm acrylic with bearings and the XL330
through the Python API and the CLI, with the TrotBot unit-7 failure; round 3's Hoecken
over MCP, the bolt Klann over the CLI and the Jansen quad over the API; round 4's
box-and-budget walker and cheapest walker over MCP, the derive / compare / recheck /
full verify over the API, the lift over the CLI), expected to go through clean. *Five
corners nobody had driven*: (1) a rotation mechanism over MCP, "a single-servo rocker
that swings at least 120° with a transmission angle that never drops below 40°"; (2)
every export format at once for a TrotBot heel quad, then `spiderpig sim` on that MJCF
and the sim's speed against the walk model's; (3) `spiderpig report` over every linkage
and `explain` on a mechanism and a walker; (4) the two-input five-bar over MCP; (5) a
Spec written wrong three ways (a misspelled key, a bare-number target, `legs.module:
"quad"` on a mechanism).

Driver: the MCP server as a subprocess (`python -m spiderpig.cli mcp --store <tmp>`
from the worktree, the official `mcp` 2.x client, a fresh store per goal);
`PYTHONPATH=. python -m spiderpig.cli ...`; `spiderpig.api` in a Python session on a
scratch store; headless Chromium for the viewer. Allowed reading: `README.md`,
`docs/agentlib/API.md`, this file's rounds 1 to 4, `--help`, `help()`, the guide, the
cards and the catalog. Goals ran two at a time on the 4 cores (as the logs' second
runs did); the browser checks ran alone at the end. One honesty note: my first
browser numbers read 128 s to the GLB on every page because my Playwright client
polled with `time.sleep` (the sync API dispatches events only inside its own calls);
the numbers below are from the fixed client, on an idle machine.

### The regression

| goal (path) | clean? | this run | the log's second run |
|---|---|---|---|
| R2 API: import + cards; 10 spec probes | yes | 3.0 s; ms | 3.8; 3.4 |
| R2 API: the goal at 2 mm: check, plan, walk, quick | yes: `construction / unbuildable`, least pitch 2.9 mm, the checked patch `thickness_mm: 3`, `resolve` warned | 1.1 s | 4.2 |
| R2 API: recommend, derive, check, plan, quick | yes: 12 layers, optimal | 0.8 s | 4.4 |
| R2 API: the derived Klann quad, standard | yes: `budget.cost_usd` **$259.71** FAIL, 4 unpriced (the 64 bearings are priced now, $123.69: round 3's catalog, not "$115.67 at least") | 44.7 s | 54.0 |
| R2 API: Strider double, quick + standard | yes ($255.74 at least; 16 layers, 31.5 mm clearance, 172.8 mm/rev, 296.7 mm/s) | 38.3 s | 53.4 |
| R2 API: export step/dxf/print/bom/glb | yes, 22 files, the XL330's 3 faces on `warnings` | 33.7 s | 44.4 |
| R2 API: the edit on the loaded robot + recheck | yes (`L.b2_leg0` 4963.8 -> 4942.6 mm3, mass 5.907 -> 5.882 g, no note) | 9.7 s | 14.2 |
| R2 API: TrotBot heel at unit 7: check, recommend, derive, plan, quick, explain | yes (the same message; the quad's own plan 37 layers in the deadline, unproven) | 69.7 s | 76.8 |
| R2 CLI: explain (2 mm; the Strider), build, audit | yes (the 2 mm warning on stderr, `STOP: no M3 screw ...`; 257 parts OK) | 2.9 + 4.1; 32.7; 61.8 s | 4.7; 38.7; 73.9 |
| R2 CLI: `spiderpig view` to the URL; page + GLB; drive mode + HUD | yes: the export served as is (`X-Spiderpig-Glb: export`, 10.5 MB), no console errors; the HUD's numbers not read (my client's toggle) | 4.6; 2.7 + 6.2; — | 3.0; 9.5; 33 |
| R3 G1 MCP: discover, 11 cards, catalog; Hoecken resolve → view; probes | yes (66.9 mm at 0.064; standard 12.4 s job; export 1.2 s; Peaucellier 51.6 / Watt 50.1 mm; stroke 80 → `unit 19.5`, checked 81.53; dwell refused; the three line mechanisms plan) | 2.2; 17.0; 0.9 s (27 s in session) | 8.4 (2.6); 22.3 (14.6); 12.2 |
| R3 G1 CLI: `spiderpig view` from a shell; page + GLB | yes: the export served, no console errors | 9.6; 2.9 + 6.4 | 7.0; 5.2 |
| R3 G2 CLI: `build --list`, explain default | yes | 2.9; 3.8 s | 8.2 |
| R3 G2 CLI: explain bolt both; at 5 mm; bolt pillars; bolt pins | yes (proven "3-15 layers ruled out", `pillar bolt -> printed ... 13 layers (39 mm)`; the 5 mm warning; 13 / 39 mm) | 6.4; 4.5; 6.2; 4.1 s | 6.5; 5.2; 7.0; 4.7 |
| R3 G2 CLI: build default; build --pin bolt; audit --pin bolt | yes (197 parts OK) | 86.7; 55.2; 62.9 s | 84.4; 59.6; 66.5 |
| R3 G2 CLI: `view --linkage klann --module quad --pin bolt` | yes: resolved into the store as `1a89d6bb0b7de797` (the log's id), the export served | 32.0; 2.7 + 6.2 | 25 + 4.7 |
| R3 G3 API: resolve → explain | yes (48 mm proven, the floor note; 552.8 g estimated) | 3.3 s | 6.9 |
| R3 G3 API: describe + derives (XL330, decker, double, unit 1.3) | yes (unit 1.3: the deadline, then `unit 1.3 -> 1.6 ... plans (quad module, the design's own) in 16 layers`) | 65.3 s | 74 |
| R3 G3 API: plywood, + XL330, standard; two more standards | yes (319.87 g built vs 339.0 estimated; by group) | 75.1; 112.5 s | 70; 57 + 59 |
| R4 G1 MCP: discover; describe, catalog, resolve, quick | yes (floor $117.07 "already over") | 2.1; 3.1 s | 5.9; 7.1 |
| R4 G1 MCP: four derives + quick; two derives + quick + the sts3215 standard | yes ($101.83 plywood, $116.81 XL330; z 205 on the sts3215 candidate) | 78.2; 70.5 s | 88.0; 57.5 |
| R4 G1 MCP: final standard, export, view, browser | yes: 50.63 mm, 290 mm/s, 208 x 145 x 181, $119.91, 20 layers proven; the page: 166 nodes, 330 tracks, the export served, no console errors (from a shell: 35 s cold after a restart, 4.1 s warm to the URL) | 52.3; 142.0; 3.1; 2.9 + 6.4 | 152.3 in all |
| R4 G2 MCP: 17 cards; seven candidates quick + standard | yes ($108.76 / $112.73 at least; the sixbar quad's quick 60.5 s) | 10.9; 285.4 s (standards two at a time beside G1) | 14.5; 193.6 |
| R4 G2 MCP: export(bom) + summary; pillar variants | yes (`bom.md` $108.76 = the row; bolt fails in 1 s with the checked patch back to printed) | 73.2; 68.3 s | 74.3; 51.9 |
| R4 G3 API: load, three derives, standards, compares | yes (335.8 -> 238.5 g, 149.8 -> 296.7 mm/s, 22.6 -> 31.5 mm, 108.76 -> 119.91) | 97.4 s | 106 |
| R4 G3 API: build (reloaded), the `Part.locate` edit on both sides + a wrong-frame edit, recheck | yes (21.2 mm3 off each; the note names the missed one) | 9.3 s | 10 + 14 |
| R4 G3 API: verify full, XL330 + rod; sts3215 + rod | rows yes (326.3 mm/s, 190.7 mm/rev, stays up 4.2°, 0.12 of 0.52 N·m; 164.9 mm/s, 0.21 of 1.91), but **`ok: false`** (entry 1) | 65.6; 57.7 s | 54.4; 52.5 |
| R4 G4 CLI: three explains, report; build; audit | yes (the axes named; 26 parts OK) | 9.2 + 3.3; 8.1; 20.2 s | 12; 9; 19 |
| R4 G4 CLI: `view --linkage parallelogram_lift`; browser | yes: resolved as `af8b33c63bb7b4b0` (the log's id), the export served, no console errors | 11.7; 2.8 + 6.3 | 25; 14 |

Every regression goal went through on the logged path with no workaround; the
numbers are the logs' (the engine's, to the digit) and the wall clocks at or under the
logs' second runs but for two: G2's seven standard verifies (285 s: I ran them two at a
time beside goal 1's plans) and G1's export (142 s beside G2's). One verdict changed,
by round 3's catalog: with the bearings priced, round 2's "$115.67 at least" is now
"$259.71 at least" (the honest number). One verdict is new and wrong for the goal:
every `verify` of round 4's goal 2 and 3 designs reads `ok: false` on the cost row alone
(entry 1), because I gave the "cheapest walker" goal a budget (`max: 150`) where round 4
gave none.

### The corners

| corner | wall | outcome |
|---|---|---|
| 1 rocker over MCP (3 cards, amplifier resolve → standard → export → view → browser; crank-rocker probes) | 45 s | the cards answer both numbers before any design (entry 2); one engine bug (entry 3) |
| 2 export of all seven formats (API), the sim | 209 s export, 107 s full verify, 79 s CLI sim | no `spiderpig export`; `spiderpig sim` takes no MJCF (entry 6); the robot falls over in the sim (entry 5) |
| 3 `spiderpig report`; `explain` on `crank_rocker` and the Strider double | 693.6 s; 3.2 s; 4.3 s | the report covers the walkers only, 9 of its 68 plans at the deadline (entry 7); both explains complete |
| 4 the five-bar over MCP | 15 s | says what, not what to do (entry 4) |
| 5 nine wrong specs over MCP and the API | 7 s | every one back on track in one round from `nearest` / `allowed` (entries 9, 10) |

### Entries

#### 1. A hard `budget.cost_usd` target can never pass on a walker: three packs of screws are unpriced on every design, and "accept them by hand" has no lever — **annoying** (a blocker for any goal that names a budget and reads `ok`)

Tried (R4 G2, the cheapest walker): the goal spec with `budget: {cost_usd: {max: 150}}`
on seven candidates. Every `verify(standard)`: `budget.cost_usd: 108.76 USD vs <= 150
[FAIL, estimated] at least; the target can't be verified while items are unpriced (price
them in the catalog, or accept them by hand): 11 items, the largest Feetech STS3215
servo ...; 3 unpriced, so the total is a lower bound: 8 x M2 x 6 mm pan-head
self-tapping screw (for plastic) (1 pack of 100), 4 x M3 x 16 mm button head socket
screw (1 pack of 100), 4 x M3 x 18 mm socket head cap screw (1 pack of 100)`, and
`ok: false`. The same three items (the servo horn's self-tappers, the crank's button
heads, the frame ties' screws) are on every walker in the catalog, whatever the linkage,
sheet, servo or pivots, so no walker with a budget target ever verifies, at any level:
goal 3's `verify("full")` with every sim row green is `ok: false` for this row alone,
and the `design_walker` prompt's loop ("stop at the first ok: false") stops here. The
row's own remedy, "accept them by hand", names nothing an agent can do: no spec field
takes an allowance or an acceptance, and the catalog is not the agent's to edit.
Round 2's entry 3 asked for this honesty on a lower bound that hid $50 of bearings; here
the bound hides three packs of screws at a few dollars each under a $41 margin.
Expected: a way to accept unpriced items in the spec (an allowance per unpriced pack,
or a list of accepted keys) that the row reads ("$108.76 + 3 unpriced packs at up to $5
each = $123.76 <= 150: ok, the packs named"), and the row's remedy to name it.
Recoverable from docs and messages alone: no (the number is right; the verdict is
wrong for the goal, and nothing says how to change it).

#### 2. The goal's second number, the transmission angle, is not a target; the spec error's "nearest" is `rotation_deg` — **annoying**

Tried (corner 1): `motion.transmission_angle_deg: {min: 40}` on the rocker amplifier.
`resolve`: `motion.transmission_angle_deg: unknown metric (did you mean 'rotation_deg'?);
allowed: stroke_mm, straightness_mm, on_line_fraction, rotation_deg, swing_deg,
dwell_deg`. The number exists: the card's `closures` (`E: ... transmission angle
55°..125°`, `K: ... 43°..136°`) and `check.steps[].transmission_deg`, so I read it off
the card (the amplifier's 43° meets the 40°; the crank-rocker's 55°; the dwell rocker's
48°) and dropped the target. But nothing verifies it, no row carries it (the
`program.loops_close` row gives the least *margin*, in mm), and a derive that lowers it
under 40° would pass every row. Expected: `motion.transmission_angle_deg` (the least
over the closures, measured by `check`) as a target and an informational row, for
walkers too; and the nearest-key hint not to point at a rotation.
Recoverable: yes, by reading the card.

#### 3. `crank_rocker` with `crank: 1.6, rocker: 1.2`: the fixed pivot goes NaN, the program stage passes, and `check` fails at `static` with "passes crankpin M at nan mm" — **annoying** (an engine bug)

Tried (corner 1, chasing 120° on the crank-rocker after the card's sensitivity: `crank
+10 %: swing +11.2 %`, `rocker +10 %: -9.9 %`): `crank 1.5` -> 97.2°, `rocker 1.2`
-> 112.9°, then `crank 1.6, rocker 1.2`. `derive` ok; `check`: `ok: false`, `static /
link_no_layer: crank_rocker: b2 sweeps right across the crank at O, so its layer needs
the crank off its axis, and no crank point clears it: it passes crankpin M at nan mm,
under the 10.0 mm a post there needs ...; no point within 150 mm of O clears it \n no
part sizes at this scale clear it`, `culprits[0].dist_mm: null`, `output.text:
"crank_rocker: b2 (rotation) covers nan mm in x by nan mm in y (up), swings nan° about
G"`, `steps: [... E: derived (G, M)]` (the closure line is gone), and on the server's
stderr `RuntimeWarning: invalid value encountered in sqrt ... unit*sqrt(-crank**2 +
rocker**2)`. The linkage places G at `unit * sqrt(rocker² - crank²)`, imaginary once the
crank is longer than the rocker; the program stage saw a NaN and called it a pass.
Expected: `resolve` or `check` to refuse the parameters at stage `program` ("crank_rocker:
the crank (1.6) must be shorter than the rocker (1.2): the fixed pivot G at sqrt(rocker²
- crank²) is imaginary"), never NaN geometry with a static-stage message about it.
Recoverable: only by trying other values (the message says nothing true).

#### 4. The five-bar: the drive failure says what is missing, not what to do; `recommend` answers `[]` with no note; `resolve` is silent — **annoying**

Tried (corner 4): `describe(five_bar)` (`inputs: ["t", "t2"]`, `notes: "Two cranks place
P anywhere in its workspace. Needs a second drive."`), `resolve` (`ok: true`, no
warning), then `check`, `plan`, `verify(quick)`, `build`, `export`: every one `drive /
second_input_no_drive: five_bar: the drive turns one input, the crank at O (one servo);
five_bar has 2 (t, t2), and a drive for t2 isn't built`; `recommend`: `{"stage":
"drive", "recommendations": [], "notes": []}`; `explain`: the program (its closure over
both inputs, `P ... transmission angle 44°..122°`) then `STOP:` the same line. The guide's
failure table says "the mechanism has a second input; v1 drives one" and its limits say
nothing of it. So the agent learns, after a check, that the one two-input linkage in the
catalog can't be built, and not that this is a limit of v1 rather than of the spec, nor
what to pick instead. `view` then answers `ok: true` with a URL for the design (the page
would bake it and fail), where `spiderpig view` from a shell says `error: 315ae7254b632dba
can't be built: drive (second_input_no_drive): ...`.
Expected: the message (and a `resolve` warning, from the card's `inputs`) to say "v1
builds one drive: a two-input mechanism can't be built; the one-input mechanisms are
..." and `recommend`'s notes to carry it; the MCP `view` to refuse as the CLI does.
Recoverable: yes (the guide's table), after the dead calls.

#### 5. The TrotBot heel quad falls over in MuJoCo (`sim.stays_up: False, max tilt 92.2 deg`) while the walk model reads `tipping_fraction: 0.0`; the sim's speed is 108 mm/s against the model's 165 — **annoying** (an engine finding the row doesn't explain)

Tried (corner 2): `verify("full")` on the TrotBot heel quad at its defaults (unit 10.5,
37 layers / 111 mm a side, 614 g). Rows: `motion.stride_mm 190.3`, `motion.speed_mm_s
165.0 [estimated]`, `sim.speed_mm_s: 108.4 mm/s [measured] 4 s at the drives' full
speed`, `sim.stride_mm: 125.4`, `sim.stays_up: False [FAIL] max tilt 92.2 deg`,
`sim.torque: 0.96 N·m ... of 1.9123 stall`, `ok: false`, `failures: []`. `spiderpig sim
--linkage trotbot_heel --module quad --json` says the same (`fell: true`, `body_contact:
0.92`, `penetration: 3.2`, `feet_down: 0.61`, `slip: 49`). The quasi-static model
(`tipping_fraction: 0.0`, `min_margin_mm: 45`) and the sim disagree by a fall, and the
row gives the tilt only: not when it fell (at the start, settling? under drive?), on
which side, or what to move. Round 4's Strider double stayed up in both; nothing in
the earlier rounds ran the sim on an eight-legged, 111-mm-a-side robot. Expected: the
row to say when and how it fell (the time, the axis, whether it was upright after the
settle), and the walk note or the row to name the levers (a lower stack, the phases).
Recoverable: the fact is there; the why is not.

#### 6. No `spiderpig export`; `spiderpig sim` takes no MJCF — **annoying**

Tried (corner 2): `spiderpig export --help`: `unknown command 'export'`. The seven
formats at once are the Python API's (`api.export(design, [step, stl, print, dxf, bom,
glb, mjcf], out_dir)`: 34 files in 209 s, `manifest.json` with the plan, the BOM total
$131.89 and 3 unpriced, the snap-prong warnings on `warnings`) or the MCP's; the CLI
writes them in three commands (`build` for five, `bake` for the glb, `sim --xml` for the
MJCF). Then "`spiderpig sim` on that MJCF": `sim --help` has `--xml XML  write the MJCF
here` and no way to read one, so it rebuilt the robot (79 s) to write a model byte for
byte the size of the exported one (144223 bytes both), and the exported `trotbot_heel.xml`
loads in MuJoCo directly (68 bodies, 6 meshes, 2 actuators `L.drive` / `R.drive`, 32
equalities). Expected: `spiderpig export <design or build options> --formats ...` (the
API's `export` on the CLI, as `view` took the build options in round 3), and `spiderpig
sim --xml-in FILE` (or `sim <design>`) to run a stored model.
Recoverable: yes (the API).

#### 7. `spiderpig report` with no `--linkages` reports the 17 walkers only, in 11.5 minutes, with no progress line — **annoying**

Tried (corner 3): `spiderpig report --out linkages.json` ("Compare every registered
linkage"). 693.6 s; `wrote linkages.json (17 linkages)`: the walkers; the 11 mechanisms
only with `--linkages` (round 4's fix, which works: the lifts' report ran in 3.3 s). Nine
of the 68 module plans ran to the 60 s deadline or the node budget (the sixbar, sixbar_v1,
sixbar_v2, sixbar_v3, Strider, TrotBot and TrotBot-heel / toe quads, unproven, and the
two TrotBot-heel / toe `double`s, `PlanError` after 60 s), each a full minute with no
line before it; `--cost` is rightly opt-in. Expected: every linkage by default (the
mechanisms with their output rows), a line per (linkage, module) as it starts, and a
`--modules` hint in the help ("the quads and the TrotBot doubles take the planner's
minute each").
Recoverable: yes (wait, or name the linkages).

#### 8. A target on the five-bar's xy output: "its metrics: " and nothing — **nit**

`motion.stroke_mm: {min: 50}` on `five_bar`: `motion.stroke_mm: five_bar's xy output has
no stroke_mm; its metrics: ` (empty), `allowed: null`. The card's `extent_mm` (64 x 53
mm) is the output's one number and no metric names it. Expected: "its metrics: none (an
xy output is measured by its extent, on the card)" or `extent_x_mm` / `extent_y_mm`
targets.

#### 9. `legs.module: "quad"` on a mechanism gets the walker's wording — **nit**

`{"kind": "mechanism", "linkage": {"key": "hoecken"}, "legs": {"module": "quad"}}`:
`legs.module: unknown value 'quad'; a module is the legs per side, and the robot has two
sides: single 1 a side (2 on the robot); no linkage has a three-leg module (which modules
walk is on the linkage's card, api.describe); allowed: single`. `allowed` is right and
got me back in one round; the sentence is a walker's (a mechanism has no legs, one side
and no robot). Expected: "hoecken is a mechanism: its one module is `single` (one side, no
legs)".

#### 10. A misspelled `linkage.key` lists all 28 linkages whatever the `kind` — **nit**

`klan` under `kind: walker`: `nearest: klann` (good) and `allowed` = all 28 keys, the 11
mechanisms included; a right key of the wrong kind lists the right kind's ("klann is a
walker, not a mechanism; allowed: hoecken, ..."). Expected: the kind's keys.

#### 11. The guide's cost paragraph is stale — **nit**

`spiderpig://guide`: "the catalog's bearings, bushings, rod, clips, CA glue and most
screws have vendor links but no price, so a metal-pivot design's cost stays a lower
bound until they are priced or accepted by hand". Round 3 priced the bearings, the
bushing, the glues and most M3 screws; this run's rows price 64 MF63ZZ at $123.69 and
name 4 unpriced. Expected: the sentence to name what is still unpriced (the rod, the
clips, the M2 tapping screws, M3 x 16 / 18 / 50).

#### 12. A one-sided design's quick `size.envelope_z_mm` detail: "the stacks + the chassis" — **nit**

The Hoecken (one side, no chassis): `size.envelope_z_mm: 57.5 mm [estimated] the stacks
+ the chassis + 3 mm of axle heads outside each outer plate; measured after a build`
(built: 56.3). The number is fine; the words are the robot's.

#### 13. `spiderpig build` prints build123d's "Unknown Compound type, color not set" — **nit**

On every CLI build (`exporters3d.py:295: UserWarning`); round 2's entry 10 silenced it
on the API's export, not on `spiderpig build`.

#### 14. `spiderpig sim --json`'s `kinematic.stride` reads -297 for a walk model stride of +190 — **nit**

The TrotBot heel quad: the walk model 190.3 mm/rev (+x), the sim 125.4, the JSON's
`kinematic: {stride: -297.3, stance_length: 117.6, foot_lift: 54.3}`: another number
with the other sign, and nothing says what it measures.

#### 15. Good — no entry

The regression: every goal on its logged path, with the logs' numbers. The spec errors
(corner 5): `klan -> klann`, `stryder -> strider`, `hoeken -> hoecken` under `nearest`;
the bare number's three forms (`{"max": 40}, {"min": 40} or {"value": 40}`); a
mechanism's `sides: 2`; two errors at once both listed (the module error waits for a
valid key, rightly: its vocabulary is the linkage's). The rocker cards answered the goal
before any design (the amplifier: `swings 153.26° about H`, closures `55°..125°` and
`43°..136°`; the crank-rocker's `60°`; the dwell rocker's `43.9°, 125° dwell`), the
amplifier resolved, planned (7 layers), verified standard (17 s, 29 parts), exported and
viewed (the page baked it in 11 s, no console errors) in 45 s of session; `recommend` on
the crank-rocker's 60° said the honest "missed; no lever the engine can compute for it"
and the card's sensitivity named the two levers. The bolt pillar's proven verdict and
patch in 1 s; the sim rows beside the walk model's; the exported MJCF loading in MuJoCo
as is; the reloaded robot's `Part.locate` edit on both sides and the note on the
wrong-frame one.

### What I ended up with

**Corner 1.** The rocker amplifier (`rocker_amplifier`, design `b56f19944c764ee0`) at
its defaults: **153.3°** of swing about H, transmission angles **43°..136°** and
55°..125° on its two closures, 7 layers / 21 mm, 29 parts, 96.6 g estimated, one servo,
$72.86 floor; standard verify green but two snap-prong warnings (6.0 %, 6.9 %); STEP,
DXF, print STLs and the BOM exported; viewed. The crank-rocker's 60° reaches 113° with
`rocker 1.2` and 97° with `crank 1.5`; `crank 1.6, rocker 1.2` is entry 3.

**Corner 2.** The TrotBot heel quad (`726f2ecf20866afb`, unit 10.5): 37 layers / 111 mm
a side in the deadline (unproven), 614 g, 335 x 202 x 301 mm, $131.89 at least; 34
files in the seven formats; the walk model 190 mm/rev and 165 mm/s at 52 rpm, the sim
**108 mm/s and 125 mm/rev, and it falls** (entry 5); the MJCF loads in MuJoCo.

**Corner 3.** `report`: 17 walkers, 68 module plans, 2 `PlanError`s (the TrotBot-heel /
toe doubles), 7 quads unproven at the deadline, 693 s (entry 7). `explain crank_rocker`:
the program, the output line, 6 layers; `explain --linkage strider --module double`:
the 56 static clearances, 16 layers, 48 mm.

**Corner 4.** The five-bar resolves and stops at `drive` (entry 4); one input is v1.

**Corner 5.** Nine specs, each back on track from the message alone (entries 9, 10).

### Phase 2 — what was fixed, and the second run

Commits: `4e5dc9e` (engine: a point that isn't a number fails the program stage
and names its square root, the kinematic stride's sign, when and how a fall happened, an
exported MJCF runs as is), `03aa2d2` (the API, reports and CLIs: the budget's
allowance, the transmission angle as a target and a row, the two-input mechanism's words
at `resolve`, `check`, `recommend` and the MCP `view`, `spiderpig export`, `spiderpig
sim <design>` / `--mjcf`, `report` over every linkage with a line per plan, the spec
messages per kind, the one-sided envelope's words, the build's warning; API.md, the
guide, the README) and this round's log in the commit after them. Tests for each in
`tests/test_spiderpig_api.py`, `tests/test_spiderpig_mcp.py`, `tests/test_view.py` and
`tests/test_sim.py` (the section "Test drive, round 5", `test_r5_*`).

| entry | severity | fixed in | how |
|---|---|---|---|
| 1 a hard budget never verifies on a walker | annoying | `03aa2d2` | `budget.allowance_usd`: one plain number under `budget` (validated ≥ 0, in the schema, in the resolved record and so in the id), USD for all the unpriced items; `cost_row` adds it and the row reads "$108.76 priced + $15.00 allowed (budget.allowance_usd) for the 3 unpriced items: ..." and verifies; without it the failing row's remedy names the field. The guide's cost paragraph says what is priced and what is not, and that three unpriced packs are on every walker |
| 2 the transmission angle is not a target | annoying | `03aa2d2` | `motion.transmission_angle_deg` (both kinds, soft, measured by `check`): the least over the closures folded about 90° (`verify.least_transmission_angle`), a row on every quick verify ("the least over the closures is at K"), a target when asked; the "unknown metric" hint no longer offers a metric of another name (`rotation_deg` for a transmission angle) |
| 3 a NaN point passes the program stage | annoying (engine) | `4e5dc9e` | `check_steps` flags a fixed or derived point whose coordinates aren't finite (`StepCheck.invalid`, failing the whole cycle) and says why: the square root whose argument went negative, with its value ("sqrt(-crank**2 + rocker**2) with -crank**2 + rocker**2 = -1.12 at these parameters; the linkage needs it positive"); `assert_assembles` raises on it, `api.check` reports `program / point_undefined` with a note, `explain` prints it as the STOP, and the static stage never sees a nan |
| 4 the five-bar says what, not what to do | annoying | `03aa2d2` | `resolve` warns (`api.second_input_note`: v1 builds one drive, `check` reads the program and output, the rest stops at `drive`, a limit of v1 not of the spec, the one-input mechanisms named); the drive's `ConstructionError` says the same; `advise` / `recommend` add the note ("no fix: ..."); the MCP `view` runs `check` and `plan` first and answers `ok: false` with the failure for a design the page could not bake, as `spiderpig view` does; the guide's limits carry it |
| 5 the TrotBot heel quad falls in the sim | annoying (engine) | `4e5dc9e`, `03aa2d2` (the row) | `walk_metrics` carries `fell_at_s` and `fell_axis` (`sim.run._fall`: the first sample past 45°, rolling or pitching); the `sim.stays_up` row says "fell over rolling at 0.9 s into the 4 s run" and what the quasi-static model saw ("tipping fraction 0.00: it saw no tipping: the fall is dynamic, or the sim's contacts; a lower stack, a slower drive or other phases are the levers"); `spiderpig sim` prints it too. The fall itself is not fixed: it reproduces at 0.4 of the no-load speed and at half the timestep, and the robot stands while settling (tilt 4.2° over 3 s), with its heel links on the floor by design (the sim counts a heel as body contact); a real dynamic finding of the sim on a 111 mm stack, left to the sim's owner with the row now saying when and how |
| 6 no `spiderpig export`; `sim` takes no MJCF | annoying | `03aa2d2` | `spiderpig export <design or build options> --formats ... --out --store --force` (`spiderpig/tools/export.py`: `api.export` on the command line, the build options resolved into the store as `view` does, warnings on stderr); `spiderpig sim <design> [--store]` runs a stored design with its exported MJCF when the store has one, `spiderpig sim --mjcf FILE` runs a given model with its `.json` (`simulate(model_xml=, model_meta=)`); README and API.md |
| 7 `report` covers the walkers only | annoying | `03aa2d2` | every registered linkage by default (the mechanisms with their output rows), a log line as each (linkage, module) starts planning and one naming the linkages, the help saying which modules take the planner's minute |
| 8 "its metrics: " and nothing | nit | `03aa2d2` | "an xy output is measured by its extent alone (extent_mm on the card), which is no target" |
| 9 the walker's words on a mechanism's module | nit | `03aa2d2` | "hoecken is a mechanism: its one module is single (one side, no legs; the leg modules double, decker and quad are a walker's)" |
| 10 all 28 keys whatever the kind | nit | `03aa2d2` | `allowed` is the kind's keys; the nearest key of the other kind is still named with its kind ("'hoecken' is a mechanism, not a walker") |
| 11 the guide's stale cost paragraph | nit | `03aa2d2` | rewritten: what is priced, what is not, the allowance |
| 12 "the stacks + the chassis" on one side | nit | `03aa2d2` | "the stack + the servo on the inner plate + 3 mm of axle heads outside the outer plate" for one side; "the two stacks + the chassis + ..." for the robot |
| 13 `spiderpig build` prints build123d's warning | nit | `03aa2d2` | silenced around the STEP export, as the API's export does |
| 14 `kinematic.stride` reads negative | nit (engine) | `4e5dc9e` | `kinematic_gait` multiplies by the drive's `crank_sign`, so the kinematic stride reads forward for every walker (a test over the Klann, the TrotBot heel and the Strider) |

Not fixed, and why:

- The fall (entry 5): reported now, not cured; the sim's contact model on the TrotBot
  heel (a heel that touches the floor by design) is the engine's question, not the
  surface's, and a settle-and-drive it survives for 0.5 s is not a proof either way.
- The HUD's numbers in headless Chromium: my client never read them (the page's drive
  mode toggled, the canvas rendered, no console error); rounds 1-4's clients did. Not a
  product finding.
- The report's 12 minutes: `--modules` and the log line say where the time goes; the
  planner's deadline is the engine's, as in every round.

### Wall clock, second run

Against the fixed tree, fresh stores, the corners again (the round 5 tests, the full
suite and the audit ran alongside on the same 4 cores, so the longer steps read slower
than alone).

| step | wall | note |
|---|---|---|
| C1 rocker over MCP: the goal as a spec (`swing_deg >= 120`, `transmission_angle_deg >= 40`, both hard), resolve, quick, explain | 1.0 s | `motion.transmission_angle_deg 42.6 [ok] the least over the closures is at K, folded about 90°`; `explain`'s `4. targets` reads both; was "unknown metric (did you mean 'rotation_deg'?)" |
| C1 the NaN derive (`crank 1.6, rocker 1.2`): check, recommend | in the 1.0 s | `program / point_undefined: crank_rocker: G (fixed) can't be placed at these parameters: its coordinates are not a number (sqrt(-crank**2 + rocker**2) with -crank**2 + rocker**2 = -1.12 at these parameters; the linkage needs it positive)`, with the note; was "passes crankpin M at nan mm" |
| C4 the five-bar: resolve, check, recommend, view, a stroke target | 0.2 s | `resolve` warns ("v1 builds one drive ... a limit of v1, not of the spec; the one-input mechanisms are hoecken, ..."), `recommend` notes "no fix: ...", `view` answers `ok: false` with the drive failure, the xy target's refusal says what an xy output is measured by |
| C5 the wrong specs | ms | the kind's keys under `allowed`, "'hoecken' is a mechanism, not a walker" for a misspelling of the other kind, the mechanism's own words on `legs.module` |
| R4 G2's verdict: the Strider double with `budget.allowance_usd: 15`, standard | 50.2 s | `budget.cost_usd: 123.76 USD vs <= 150 [ok] $108.76 priced + $15.00 allowed (budget.allowance_usd) for the 3 unpriced items: ...`; `ok: true`; was FAIL "at least" on every walker |
| C2 `spiderpig export --linkage trotbot_heel --formats step stl print dxf bom glb mjcf` | 348.5 s (beside the suite and the audit) | 34 files into `--out`, resolved into the store as `840981a0b319eb3b`; was the API only |
| C2 `spiderpig sim 840981a0b319eb3b --store ...` (its exported MJCF) | 7.6 s | "running .../trotbot_heel.xml"; no rebuild (was 79 s); `fell over: YES (pitching at 2.0 s into the run)`; the kinematic stride reads +297.3 mm/rev (was -297.3) |
| C2 `spiderpig sim --mjcf out/heel/trotbot_heel.xml --linkage trotbot_heel --module quad` | 7.2 s | the same model, the same numbers |
| C3 `spiderpig report --linkages crank_rocker hoecken klann --modules single` | 4.8 s | "3 linkages: crank_rocker, hoecken, klann", a "planning" line per module, the mechanisms' output rows; the default is every linkage now |
| `spiderpig build --module single` | 34.2 s | no build123d warning |
| the new tests (`-k r5`: 15) | 36 s + 35 s | green; the full suite and the audit: see the report |

Read as the agent: the rocker goal is a spec now (both numbers as targets, verified
at `quick`, explained under `4. targets`) and a parameter set that breaks the geometry
is refused with the expression that broke; the five-bar says what it is (a limit of v1)
at `resolve`, and `view` no longer hands out a URL for it; a budget verifies once the
three unpriced packs are accepted with one number; every format is one CLI command and
the sim runs the exported model in seconds. What it still does by hand: nothing in the
corners; the fall of the TrotBot heel quad is reported (when, how, and that the
quasi-static model saw no tipping), not explained.

Verdict on the loop: converged, still. The regression of rounds 2, 3 and 4 went
through clean on every path (the numbers to the digit, the wall clocks at or under the
logs'), and the corners' entries were a missing spec lever (the allowance), a missing
metric (the transmission angle), an engine bug (the NaN point), an under-explained limit
(the second input), two missing CLI commands and a report's default: none a workaround,
each a few lines. Only nits and one engine finding remain: the viewer's chrome around a
mechanism (rounds 3 and 4), and the sim's fall on the tallest walker in the catalog, now
reported honestly. The single biggest remaining problem is that finding: the sim and the
quasi-static model disagree on whether the TrotBot heel quad stands, and no row can yet
say which of them is right about the physical robot.

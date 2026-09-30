# Test drives: an outside agent against the public surface

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

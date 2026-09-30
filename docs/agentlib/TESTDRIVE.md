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

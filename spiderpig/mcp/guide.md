# Designing with spiderpig

spiderpig is a compiler from a **Spec** (what you want: a walking linkage, a cranked
mechanism or an arm, its module, materials, constructions and the numbers it must meet)
to **verified geometry** (a layer plan proven clash-free over the whole cycle (a crank's
turn, an arm's tour of its inputs' box), every part built inside its planned claims,
STEP/STL/DXF/BOM files). You never draw: you name a linkage, set targets, and ask the
engine to resolve, check, plan, walk, build, verify and export. Every stage says exactly
what fails and, where it can, what would clear it.

You may start before any linkage is named, from a **task**: what the machine must do
(walk, place or trace, and its numbers). `candidates(task)` answers with the registered
linkages that can do it, ranked, each with the numbers behind its verdict and a Spec that
asks for the task's numbers as targets (below, "Tasks").

Across MCP everything is **files and numbers**: a design is its 16-hex-digit id, a
report is JSON, a part is the path of a STEP file inside the store, a failure is a
document. Solids are not exposed here (they are in the Python API,
`docs/agentlib/API.md`).

## The store

Every call reads and writes one folder, the **store** (`--store PATH`, else
`$SPIDERPIG_STORE`, else `./.spiderpig` next to the project). A design id is the
sha256 of its resolved spec plus the engine version, so the same spec is the same
design across sessions, and every stage's report is cached there: a second `plan` of
the same design costs a re-check, not a search; a second `build` reloads its STEP
files. `list_designs` lists what the store holds, `get_design` reads any stage, `gc`
removes designs (only with `keep` and/or `older_than_seconds`; it refuses to run bare).

Current store: `<<STORE>>`. Engine: `<<ENGINE>>`.

## The passes, and what each proves

| pass | tool | proves / measures | cost |
|---|---|---|---|
| candidates | `candidates(task, measure?, limit?)` | which registered linkages can do a task, by what they declare (feet, an output and its motion, ranged inputs, a turntable's yaw): each with `checks` (value, need, ok, tier, source), a `spec` to resolve, the `scale` that would meet a missed length; `excluded` says what each other lacks | ms to seconds (the cards, kept in the store); `measure=N` plans the best N (an arm's plan: minutes cold) |
| resolve | `resolve(spec)` | the spec validates; every omitted value is inferred and written into `resolved`; the id | ms |
| check | `check(design)` | **program**: every loop closes over the cycle (margins, transmission angles); **output**: a mechanism keeps its promises; **drive**: one servo turns the crank; **static**: every link has a crank point (a crankpin or a detour the body's underside allows); the ground clearance | 0.5 s |
| plan | `plan(design)` | a layer plan of one side exists: the planner guarantees the claims of different groups never meet over the whole cycle; `optimal` says whether it is proven the thinnest (`proof` says what a budget left open); `warnings` is what the constructions warned about (a printed snap that overstrains) | 0.4 s on the Klann quad; up to the 60 s deadline on a large or scaled-down design (cached: a re-check) |
| walk | `walk(design)` | the quasi-static walk model's metrics (stride, bob, slip, tipping, speed at the servo's no-load rpm) with the feet at their planned layers; a stride near zero comes with a `note` saying why and which module walks | 0.1 s |
| precision | `precision(design)` | the error budget at the output (a mechanism's point, an arm's tool; a walker is skipped): `metrics` are the `precision.*` targets measured (the worst-case `repeatability_mm` bound, the `accuracy_mm` that calibrates out; an arm's ISO 9283 `rp_mm` / `rp_powerup_mm`, `out_of_plane_mm`, `payload_sag_mm`), `rows` their verdicts, `budget` every term (each joint's share, each input's lever and the resolution it needs, the drives), `text` the budget in prose | a mechanism's: 1 s; an arm's fabricates its side for the links' tilt (a job, ~40 s cold) |
| build | `build(design)` | every part fabricated at crank angle `t`, inside its claims; masses, the envelope, one STEP per distinct part in the store | 8 s (a job) |
| verify | `verify(design, level)` | pass/fail per requirement with an evidence tier: `quick` = check + plan + walk; `standard` adds the build, the contract at two crank angles, OCCT clashes and solids, the plan re-verified on fresh sampling, the DXF pack and the BOM; `full` adds two more contract angles, a second clash angle and a MuJoCo run; every kind's output precision (`quick`: what needs no fabrication, an arm's tilt-dependent rows at `standard`) and, after the build, its joints' strength (a walker's at its sim or family loads, an arm's at its static loads: operating and a crash at the drives' holding torque) | 1 s / 30-45 s / 85 s on the Klann quad; `quick` plans first, so its first run on another design can take a minute |
| export | `export(design, formats)` | the files: `step`, `stl`, `print` (one STL per printed part), `dxf` (kerf-compensated sheets), `bom` (csv/md/json), `glb` (the viewer's animated bake), `mjcf`; always `manifest.json` (not the shopping list `ORDER.md` or the per-part DXFs `laser/parts/`: `spiderpig build` writes those); an export that already wrote these formats (or more) into the folder is returned as is | 5-60 s (a job) |

**The planner's clock.** A plan search is bounded by a 60 s wall-clock deadline and node
budgets, never validity: a plan found late is returned with `optimal: false` and a
`proof` naming the sizes left open; none found is `plan / no_plan` with the tally of
what blocked it, and the checks behind its `recommendations` get one more such deadline.
So a design that fails to plan costs one to three minutes, once (the failure is
recorded), and `verify("quick")` on a design that hasn't planned inherits that. There is
no knob for a longer search in v1.

Evidence tiers on every verify row: **proven** (the engine's guarantees: loops close,
a plan exists and re-verifies, parts stay inside their claims, the stack height),
**measured** (numbers read off models or solids: walk metrics, foot path, ground
clearance, clashes, masses, sheets), **estimated** (nominal inputs: the mass before a
build, the envelope before a build, `speed_mm_s` at the servo's no-load rpm, prices).

## The loops

**From a task**: `candidates(task) → resolve(candidate.spec)`, then the design loop. The
`design(task)` prompt walks it for any kind of machine.

**Design**: `resolve → check → plan → verify("quick") → verify("standard") → export`.
Read `ok` at every step. `resolve` returns the id every other tool takes. Stop at the
first `ok: false` and read `failures` (below). `verify` scores the soft targets and
fails on any hard miss; `export` writes the files you asked for into the store's
`exports/` (or `out_dir`) and returns their paths.

**Diagnose**: `explain → recommend → derive`. `explain` is every stage's verdict in
prose (a recorded failure is printed, not searched for again). `recommend` is the
engine's checked fixes for the failing stage, each with a spec **patch** (a JSON merge
patch: `{"linkage": {"params": {"unit": 10.5}}}` or `{"fit": {"link_radius": 5.5}}`),
given only after the stage passes with it, plus the failure's `notes` on what can't
help or wasn't checked. The engine can compute a fix from a *gap* (a link passing an
axle too closely: scale the linkage, or thinner parts) and, for a plan that ran into
the stack's own room (the crank's route, pin heads against the frame plates) after the
linkage was scaled down, it checks the linkage's default scale. When every stage passes, `recommend` answers for the **targets** (stage
`target`): a missed stroke, straightness or lift is met by a scale of the linkage (these
scale with `unit` exactly; the least practical scale is measured again and planned
before it is offered, as `{"linkage": {"params": {"unit": 19.5}}}`); a missed stack
that is proven the thinnest gets a note saying what could be thinner (fewer legs a
side, and whether those modules walk; the sheet's pitch), a missed clearance a note
naming its lowest part. Anything else (a different module, construction or servo) is
yours to try with `derive`. `derive(design,
patch)` resolves the patched spec as a new design that records its parent, so
`compare(parent, child)` shows exactly what moved. Then loop back to `plan`/`verify`.

A failure is always the same document under `failures`:

```
{stage, code, message, culprits: [{body?, joint?, point?, ...}],
 numbers: {margin_mm?, dist_mm?, need_mm?, fails_deg?, ...},
 blockers: [{count, a, b, gap_mm, need_mm}], recommendations: [{changes, why, effects,
 verified, patch}], notes: [..]}
```

| stage | code | means |
|---|---|---|
| spec | invalid_spec | the spec doesn't validate: `errors` lists each path, message, allowed values and the nearest key |
| program | loop_cannot_close | a loop's bars can't meet at some crank angles (culprit joint, margin, angles) |
| output | promise_broken | a mechanism's output misses its promise (straightness, dwell, rotation) |
| drive | second_input_no_drive | the mechanism has a second input; v1 drives one |
| static | link_no_layer | a link passes too close to every crank point: the crank can't cross its layer (distances, need; recommendations) |
| plan | no_plan / no_plan_in_budget / no_plan_in_time | no layer plan (the blockers with gaps and needs; recommendations); `no_plan_in_time`: the planner's CPU budget ran out, so `plan` searches again next time |
| fabricate / contract / clash | unbuildable / part_outside_claim / parts_clash | a part can't be made or leaves its claims or intersects another |
| layout / bom | part_exceeds_sheet / unknown_catalog_key | a part is larger than the sheet; a hardware key is unknown |
| walk | linkage_invalid | the walk model can't use the linkage |
| store | no_such_design, bad_design_id, no_such_stage, gc_needs_arguments | a tool was misused (`isError` is set) |

## Tasks

A task is a JSON object (`spiderpig://schema/task`): `do` (`walk`, `place` or `trace`)
and its numbers. A bare number is the bound the task means (a stride's least, a
repeatability's most); a target object `{min, max, value, tol, hard}` says it exactly. The
numbers that are Spec targets become the candidate's spec targets (an arm's
`repeatability_mm` is ISO 9283's `precision.rp_mm`, what a maker quotes; a cranked
mechanism's the worst-case `precision.repeatability_mm`); the task's own geometry
(`area_mm`, `points_mm`, `height_mm`, `path_mm`, `obstacle_mm`) is read by the checks.

<<TASKS>>

`candidates` reads the linkages by what they declare, never their kind's name, cheapest
evidence first: (1) the **screen**, every linkage: a walker's card (each module's stride
in the walk model at nominal feet, a foot's lift; the modules that don't walk are left
out), a cranked mechanism's output check (stroke, straightness, swing, dwell), an arm's
output over its inputs' box about its yaw axis (its turntable's range: the role
`turntable`). A place task needs three axes (two across the bench, one to lift): one input
traces a path, two in a plane sweep it, an arm's two and its yaw place. The screen's
reach of an arm is a **bound** (tier `bound`: it refutes, never confirms): where its tool
gets down to the bench is its turret's plan's. (2) the **store**: a design of the
candidate's spec (whatever its targets) lends its stored reports from this engine: the
walk with the feet at their layers, the check's ground clearance, the precision budget
(an arm's ISO RP), a verify's cost and mass. (3) **measure** (`measure=N`, opt-in): the
best N that pass are resolved into the store, checked and planned (walked; a cranked
mechanism's precision budgeted; an arm's bench reach read off its turret's plan). Nothing
is fabricated: an arm's RP needs `precision(design)` (a job).

A check's `ok` is null where this tier didn't measure it; its `source` says what does. A
candidate's `ok` is false when any check fails, null when none fails but some are
unmeasured. They rank by verdict, then a missed length that a `scale` of the proportions
would meet, then score (the mean over the measured checks of `max(0, 1 - miss / need)`),
then the task's merit (the longest stride, the best repeatability, the straightest line).
The `scale` is unchecked (the stride, lift, stroke and straightness scale with the
linkage; an arm's reach band too, its base's keep-out doesn't): `recommend` plans a scale
before it offers one.

## The Spec

A JSON object; `spiderpig://schema/spec` is its JSON Schema and `resolve` validates
it exactly (unknown fields, wildcards such as `any` or `*`, and bare numbers where a
target is expected are errors with the nearest allowed key). One spec compiles one
design: there is no search in v1.

| field | values | default |
|---|---|---|
| `version` | `"1"` | `"1"` |
| `kind` | `walker` (feet), `mechanism` (a crank and an output) or `arm` (inputs over ranges, no crank: the palletizer) | required |
| `linkage.key` | a registered linkage (below) whose kind matches | required |
| `linkage.params` | `{name: number}` overrides of that linkage's parameters (`describe` lists them; lengths in mm and > 0, angles in degrees; a coordinate such as a fixed pivot's x or y, marked `signed` on the card, may be zero or negative) | the linkage's defaults |
| `legs.module` | one of the linkage's modules (legs **per side**, the robot has two; below): `single` (2 legs), `double` (4), `decker` (4), `quad` (8) or its own; there is no three-leg module | the linkage's own default (Strider: `double`), else `quad` (walker), `single` (mechanism, arm: one side, no legs) |
| `legs.phases_deg` | one crank phase per leg of the module | the module's |
| `legs.sides` | `2` (the robot: two mirrored sides and the chassis) or `1` (one side) | 2 (walker), 1 (mechanism, arm) |
| `legs.robot` | an arm's whole robot (`true`: its side on the turntable, its tool, its controller) or its side alone (`false`); an arm's only (a walker's robot is `sides: 2`) | `true` |
| `materials.sheet` | a sheet item (sets the layer pitch) | `acrylic_3mm` |
| `materials.thickness_mm` | a measured sheet thickness | the sheet's nominal |
| `materials.servo` | the continuous-rotation servo that turns a crank (a walker's, a mechanism's; an arm's motors are its `actuator` role) | `sts3215` |
| `constructions.pillar`, `.pin` | the pivots (the catalog's `pillar / pin` rows). A walker's and a mechanism's: pillar `standoff` (a 6 mm round standoff column screwed through both frame plates, printed rings: a stock goBILDA M4 standoff where one length fills it, else one MISUMI steel standoff made to its length, M3 ends, never spliced), pin `chicago` (an M3 Chicago screw, printed rings and head spacers). An arm's: ball bearings at every pivot, `ball` / `ball_pillar` on 6.6 mm links, or the preloaded pairs `ball_pair` / `ball_pair_pillar` (the approved `palletizer_reach`'s). The walkers' other pivots were removed on 2026-10-07: naming one is a `bad_parameter` failure that names its replacement | per kind (`config.DEFAULT_PIVOTS`), per linkage where its own differ (`config.LINKAGE_PIVOTS`) |
| `constructions.<role>` | every later role a design of its kind fills (`spiderpig.roles`; the catalog's rows by role): an arm's `transmission` (per drive: `transmission@q2`), `actuator`, `angle_sensor`, `homing_reference`, `turntable`, `pin_stack`, `tool`; every robot's `controller` (a walker's `esp32_bus_servo` on its deck, an arm's `skr_pico_pi_zero`; `none`: your own). A role of another kind is refused with the kind it belongs to | the role's default for the design (its linkage's first: `config.LINKAGE_ROLES`) |
| `constructions.crank` | `bolt` (laser-cut aluminium, one plate per web, each crankpin a steel M3 hex standoff in hex pockets, M3 screws and wide washers retaining it, the riders on a printed sleeve), `bolt_round` (the same on round standoffs clamped by friction; TrotBot's heel and toe default to it). The printed, keyed and acrylic cranks and the hex crank's variants were removed on 2026-10-07 (a `bad_parameter` failure naming `bolt`); `materials.crank_sheet` must be aluminium | the linkage's (`config.default_crank`): `bolt`, `bolt_round` for TrotBot's heel and toe |
| `fit.*` | part sizes and fits in mm (below), plus `kerf_mm` and `sheet_size_mm: [w, h]` | the engine's defaults |
| `outputs` | a list of `step`, `stl`, `print`, `dxf`, `bom`, `glb`, `mjcf` | `[step, stl, print, dxf, bom]` |
| `motion.*`, `size.*`, `budget.*`, `precision.*` | **targets** (below) | none |
| `precision.payload_g` | an arm's load at the tool in grams (a plain number, not a target): what the payload sag, the ISO RP's gravity seating and the strength check's static loads assume | 50 |

**Modules** are legs per side, and the robot has two sides: `single` is one leg (a
two-legged machine that doesn't walk in the model), `double` two legs as a mirrored
pair (one facing forward, one back, at the same phase), `decker` two legs one way,
90° apart (a double-decker), `quad` two mirrored pairs 90° apart (eight legs in all).
A side's feet must take turns on the ground for the quasi-static model to walk: for
most linkages only `quad` does, and a `double` or `decker` stands on all its feet and
bobs (`walk` says so, with `stride_mm` near 0 and a note). The Strider's `double` is a
coupled pair at 0° and 180° and walks: the four-legged walker of the catalog. Every
linkage's card (`describe`) carries each module's `stride_mm` and `walks` (a stride of
at least 20 mm a turn; a few mm is a shuffle, not a walk), and a
`sensitivity` table: what +10 % of each parameter does to the foot path's lift,
stride, height and width, so you know which parameter to move before trying it.

A **target** is `{min?, max?, value?, tol?, weight: 1, hard?}` (never a bare number).
`value` is met within `tol` (default 5 % of the value). A **hard** miss fails
`verify`; a **soft** miss lowers its score (the weighted mean of `max(0, 1 - miss /
bound)` over the soft targets). Physical limits are hard by default, gait quality
soft; `hard: true|false` on any target flips it.

<<TARGETS>>

A metric's capability, in brackets beside its kinds, is what the linkage must have, and a
spec asking a linkage without it is refused with the reason: `(output)` an output point
(a walker's foot is no placement), `(ranged)` inputs that turn over ranges (an arm's
drives: a cranked mechanism has no ISO pose, no payload on its drives).

What the rows measure, where it isn't obvious: `size.mass_g` before a build is the
nominal model's total for what the design builds, and its detail says what it is made
of (links as pills at the sheet's density with the holes taken out, the servos, the
plates, the printed parts: within a few percent of the built mass); the measured row
after a build lists the mass by group (links, drive, chassis, crank, ...), so the lever
(the sheet, the servo) is plain. A `size.stack_mm` row that fails on a plan that is
proven the thinnest says so and what could be thinner (fewer legs a side, and whether
those modules walk; the sheet's pitch). `size.envelope_*` before a build is
the joints' sweep over the whole cycle plus the plates (x, y) and the stacks plus the
chassis and the axle heads outside the outer plates (z); after a build it is the
extent at the build's crank angle, so x and y read smaller and z the same.
`motion.ground_clearance_mm`'s row names the body part that sets it (the servo's body,
the frame plates, the crank's sweep, the centre plates): that is what to move.
`budget.cost_usd` is the purchase total at the catalog's pack prices: two servos, a
spool of filament, a can of solvent cement and a pack of inserts are bought whole, so
the total rarely goes under about $100 whatever the linkage; the row's detail lists
the largest items. An item with no listed price is not in the total, so a total with
unpriced items is a lower bound: it can refute a `max` but not confirm it, and a hard
`max` target then fails ("at least ...") with every unpriced item named by quantity.
Most hardware is priced (the servos, sheets, glues, inserts, the M3 socket caps, button
heads, nuts and washers); what is not is the M2 tapping screws and the M3 x 16 / x 18
screws among others, and three of those packs are on every walker (the servo horn's
self-tappers, the crank's button heads, the frame ties' screws), so a hard budget
never verifies on its own. **`budget.allowance_usd`** accepts them: a plain number
under `budget` (not a target), USD for all the unpriced items in all, added to the
priced total; the row then reads "$108.76 priced + $15.00 allowed for the 3 unpriced
items" and verifies against the target. At `quick`, `budget.cost_floor_usd` prices what the design
buys whatever its parts (servos, spool, a blank of each sheet, the Chicago pins' epoxy, a
bottle of each threadlocker) from the catalog, and a floor already over the `max` fails
the target before any build; its detail says what a build adds (the sheets' count, the
crank's screws, the pivots' hardware).

`materials.thickness_mm` is the layer pitch: every construction sizes its parts by it,
and a value more than 12 % off the sheet's nominal is a warning on `resolve`. Too thin a
layer fails at `check` (stage `construction`) with the thickness that works, where the part
that doesn't fit says it: a standoff pillar's end screw (an M4 button head and washer, one
layer outside the frame plate) names the least pitch that holds it as a checked
recommendation.

**Precision** (`precision.*`, from `spiderpig precision`'s model): `repeatability_mm`
is the worst-case bound (every pin's clearance the same way at the worst pose, the
drives' backlash and homing, the links' rise out of the plane); a maker's figure is
ISO 9283's `rp_mm` (30 cycles to each of 5 poses, mean + 3 sigma, gravity seating the
pins), much smaller. `accuracy_mm` is what calibrates out (holes off their drawing, a
10 K swing, the turntable's and the drives' systematic errors). All are tier
`estimated` (a model of nominal clearances). `verify("quick")` reports what needs no
fabrication (a mechanism's whole budget, an arm's `accuracy_mm`); an arm's
`repeatability_mm`, RP, out-of-plane and sag stay `unverified` until `standard` (or a
`precision` run, whose stored report `quick` then reads).

**Strength** rows (`strength.joints`, after a build) rate every pin, pillar, link (and a
walker's crank): a jam safety factor under 1 at the design's own loads fails `verify`
(`strength / joint_overload`, with fixes). An arm's loads are its static model's:
operating (its weight and the payload over the box x 1.5 for the moves) and jammed (a
crash: the tool blocked while the drives push at their holding torque); a fix may be a
lower run current. A cranked mechanism's loads are what it drives: not rated.

`fit` defaults (mm): <<FIT>>

### Linkages

<<LINKAGES>>

`describe(key)` gives a linkage's card: parameters (which only scale it), links, the
closures at the defaults, one foot's path numbers (lift, stance stride) or the output
check, and its modules with their default phases.

### Materials and constructions (`catalog`)

<<MATERIALS>>

## Seeing a design

`view(design)` returns the URL of the viewer for a design: the animated robot (or
one side), drive mode, and the tune panel seeded with the design's linkage, module,
phases and proportions; an arm's joint sliders pose it anywhere in its box. The page's
view picker lists the modes the design's linkage declares (`/api/modes?linkage=`: a
walker's robot and each module's side, an arm's robot on its turntable and its side, a
mechanism's one side), and its machine picker every arm and mechanism. The server starts once per MCP server on a free port and is
reused; the first load of a design bakes it (seconds) unless `export` wrote its
`glb`. The same page is `spiderpig view <design>` from a shell, and
`spiderpig view --linkage klann --module quad --pin bolt` (the build options, as for
`spiderpig build`) resolves that design into the store and shows it, so a CLI build
needs no spec to be looked at.

## Long operations

`build`, `verify` at `standard`/`full`, `export` and `precision` run in worker
processes. Each
waits `wait_seconds` (default 15) and returns the finished result when the job is
done; otherwise it returns `{ok: true, failures: [], job: {...}}` with the job's record
alone, and you follow it with `wait_job(job, seconds)` or `get_job(job)`, passing the
record's `job` field (its id). A record is `{job, op, design, args, state:
queued|running|done|failed, started_at, finished_at, seconds}`. `wait_job` and
`get_job` answer in the same shape as the tool itself: once done, the tool's own result
(a build's manifest, a verify's rows, an export's files) flat beside `job`; if it
failed, `ok: false` with the failure. Reports land in the store either way, so
`get_design(design, "build")` reads the manifest afterwards. Job records live in the
server process; the store keeps what they produced.

## Limits of v1

- Leg modules are named presets (`single`, `double`, `decker`, `quad`, a linkage's own)
  with phases per leg; no explicit leg lists.
- One machine per spec: no compound machines, no stacking of mechanisms.
- No `tune` and no `search`: one spec compiles one design; move parameters
  yourself with `derive` and read `compare` (the card's `sensitivity` says which way each
  parameter pushes the foot path, or a mechanism's output). `candidates` ranks the
  registered linkages for a task at their own proportions (it suggests a `scale`, never
  checks one); the one search the engine does is `recommend`'s scale for a missed stroke,
  straightness or lift target.
- The planner's 60 s deadline is fixed; a design it can't plan in that time is
  reported as such, with what blocked it, not searched longer.
- `speed_mm_s` is the stride at the servo's no-load rpm, tier `estimated`, until a
  MuJoCo run (`verify("full")`) reports beside it as its own rows, `sim.speed_mm_s`
  and `sim.stride_mm` (measured, 4 s at the drives' full speed), `sim.stays_up` and
  `sim.torque`.
- Prices are the catalog's preferred offers, unverified.
- A walker with `sides: 1` builds one side; its walk metrics still model the two-sided
  robot.
- One drive per machine: a two-input mechanism (`five_bar`, `inputs: [t, t2]` on its
  card) resolves, and `check` reads its program and output, but `plan`, `build`,
  `verify` and `export` stop at `drive / second_input_no_drive`; `resolve` warns, and
  no spec change helps (the one-input mechanisms build).
- The transmission angle is a target (`motion.transmission_angle_deg`, the least over
  the closures, folded about 90°) and a row; the per-closure ranges are on the card.
  A parameter set that puts a length under a square root below zero (a crank-rocker
  whose crank outgrows its rocker) fails `check` at `program / point_undefined`,
  naming the expression.

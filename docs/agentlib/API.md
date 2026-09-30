# spiderpig as a compiler: the Python API (harness v1, steps 1-4)

An agent writes a **Spec** and uses the engine as a compiler to verified geometry.
Everything lives in the `spiderpig/` package: `spec.py` (the vocabulary), `api.py`
(the operations), `failure.py` (every engine exception as data), `verify.py` (the
harness), `design.py` (the handle), `store.py` (the per-project store, below), `mcp/`
(the MCP server over all of it), `cli.py` (the `spiderpig` command) and `view.py`
(`spiderpig view`, at the end). This document is the surface.

```python
from spiderpig import api
design = api.resolve({"kind": "walker", "linkage": {"key": "klann"}})
report = api.verify(design, "quick")
```

## The Spec (v1)

A JSON object (`Spec.from_dict` validates it; `spec_schema()` is its JSON Schema).
Only fields the engine can verify today exist. Unknown fields, wildcards (`any`,
`*`) and out-of-vocabulary keys are rejected with a list of `SpecError{path, message,
allowed, nearest}` (raised as `SpecErrors`; `validate(doc)` returns the list).

| field | values | default (inferred by `resolve`, written to `design.resolved`) |
|---|---|---|
| `version` | `"1"` | `"1"` |
| `kind` | `walker` \| `mechanism` | required |
| `linkage.key` | a registered linkage (`api.list_linkages()`) | required; its kind must match |
| `linkage.params` | `{name: number}` overrides of that linkage's parameters (`api.describe(key)` lists them) | the linkage's defaults |
| `legs.module` | one of the linkage's modules: `single`, `double`, `decker`, `quad`, or its own | `quad` (walker), `single` (mechanism) |
| `legs.phases_deg` | one crank phase per leg of the module | the module's |
| `legs.sides` | `2` (the robot: two mirrored sides and the chassis) or `1` (one side) | 2 (walker), 1 (mechanism) |
| `materials.sheet` | a catalog sheet item: `acrylic_3mm`, `plywood_3mm` (sets the layer pitch) | `acrylic_3mm` |
| `materials.thickness_mm` | a measured sheet thickness | the sheet's nominal |
| `materials.servo` | `sts3215`, `xl330_m288`, `xl430_w250` | `sts3215` |
| `constructions.pillar`, `.pin` | `printed`, `rod`, `bolt`, `bearing`, `bushing` | `printed` |
| `constructions.crank` | `printed` | `printed` |
| `fit.*` | every `construction.Params` field (`margin`, `link_radius`, `frame_radius`, `min_wall`, `running_fit`, `glue_fit`, `print_fit`, `axle_d`, `spacer_d`, `neck_d`, `head_d`, `crankpin_d`, `web_radius`, `journal_d`, `stub_d`, `hub_thickness`), plus `kerf_mm` and `sheet_size_mm: [w, h]` | the engine's defaults; kerf 0.15; the sheet stock's size |
| `outputs` | a list of `step`, `stl`, `print`, `dxf`, `bom`, `glb`, `mjcf` | `[step, stl, print, dxf, bom]` |

Every metric under `motion`, `size` and `budget` is a **Target**
`{min?, max?, value?, tol?, weight: 1, hard?}` (a bare number is rejected). `value` is
met within `tol` (default 5 % of the value). A **hard** miss fails `verify`; a **soft**
miss lowers its score. Defaults (`TARGET_FIELDS`), overridden per target with `hard`:

| metric | kind | unit | hard | measured by | tier | meaning |
|---|---|---|---|---|---|---|
| `motion.stride_mm` | walker | mm/rev | soft | walk | measured | body travel per crank revolution (quasi-static walk model, both sides at one rate) |
| `motion.lift_mm` | walker | mm | soft | foot_path | measured | one foot's vertical travel over a revolution (kinematic) |
| `motion.speed_mm_s` | walker | mm/s | soft | walk | **estimated** | stride x the servo's no-load rpm / 60 |
| `motion.bob_mm` | walker | mm | soft | walk | measured | range of the body height over a revolution |
| `motion.slip_mm_per_rev` | walker | mm/rev | soft | walk | measured | RMS foot slip per revolution |
| `motion.tipping_fraction` | walker | fraction | soft | walk | measured | fraction of the cycle the CoM leaves the support polygon |
| `motion.ground_clearance_mm` | walker | mm | **hard** | static | measured | the body's lowest point above the lowest foot point |
| `motion.stroke_mm`, `straightness_mm`, `on_line_fraction`, `rotation_deg`, `swing_deg`, `dwell_deg` | mechanism | | soft | output | measured | the `OutputCheck` numbers (only those the output has) |
| `size.stack_mm` | both | mm | **hard** | plan | proven | one side's stack, frame plates included |
| `size.mass_g` | both | g | **hard** | build (walk before a build) | measured (estimated) | mass of every part |
| `size.envelope_x_mm`, `_y_mm`, `_z_mm` | both | mm | **hard** | build (joint sweep before) | measured (estimated) | extent at the build's crank angle: x along the walk, y up, z across the sides |
| `budget.cost_usd` | both | USD | **hard** | bom | estimated | purchase total at the preferred offers |
| `budget.print_g` | both | g | **hard** | bom | measured | filament at 100 % infill |
| `budget.sheets` | both | sheets | **hard** | layout | measured | sheets the laser parts pack onto |

Semantics are pinned once, in `TARGET_FIELDS`: robot metrics come from `spiderpig/walk.py`;
per-leg foot-path numbers (stance stride, ripple) appear only on the linkage card
(`api.describe`), except `lift_mm`, which is a target.

## Operations (`spiderpig.api`)

Pure functions of the `Design` handle, synchronous, each storing its report on the
handle (`design.reports[stage]`) and returning it. A design that merely fails a stage
gets a report with `ok = False` and `failures: [Failure]`; operations raise only for
programming errors (and `resolve` raises `SpecErrors` for an invalid spec).

| op | returns | what it runs | cost (Klann quad) |
|---|---|---|---|
| `resolve(spec, store=PROJECT)` | `Design` (`id`, `resolved`, `config`, `engine_version`, `warnings`, `store`) | validation, inference, `BuildConfig`; `id = sha256(canonical resolved spec + engine version)[:16]`, engine version = package version + hash of `spiderpig/linkages/*.py` + `StackSpec` defaults; records the design in the store | ms |
| `list_linkages(kind?)`, `describe(key)` | linkage cards; a walker's card carries each module's `stride_mm` / `walks` (the walk model at the defaults) and a `sensitivity` table (what +10 % of each parameter, +5° of an angle, does to the foot path's lift, stride, height and width) | registry, `Linkage.check`, foot path / output check | ms (`describe`: ~1 s, the walk per module) |
| `check(design)` | `CheckReport`: `steps`, `output`, `foot_path`, `drive`, `clearances`, `crank_facts`, `ground_clearance_mm`, `lowest_body_part` (which body shape sets the clearance) | `Linkage.check`, `output_check`, `side_problem`, `static_stage` | 0.5 s |
| `plan(design)` | `PlanReport`: `layers`, `n_layers`, `height_mm`, `route`, `optimal`, `proof`, `table`, `warnings` (what the constructions warned about: a printed snap that overstrains) | `fabricate.design_side` (cached by the engine); a search is bounded by `StackSpec.max_seconds` (60 s) and the recommendation checks by one more | 0.4 s (Klann quad); up to a minute on a large or scaled-down design, one to three on one that fails |
| `explain(design)` | text | `explain.explain_config` on the design's full config, from its own plan (a recorded failure is printed, not re-solved) | ms after plan |
| `recommend(design)` | `[Recommendation]` with `patch` (the failure's `notes` say what can't help or wasn't checked) | the failing stage's checked recommendations: a scale or thinner parts for a link-to-axle gap; the linkage's default scale for a plan that ran into the stack's own room after a scale-down | 1-60 s |
| `walk(design)` | `WalkReport`: `metrics`, `mass_g`, `rows`, `notes` (a stride near zero: why, and which module of the linkage walks) | `walk.api_payload` (feet at their planned layers) | 0.1 s |
| `build(design, t=1.0)` | `BuildReport`: `parts` manifest, `mass_g`, `envelope_mm`, `counts`, `warnings`; `design.parts[name]` | `fabricate.fabricate` | 8 s |
| `attach_build(design, mech, t)` | `BuildReport` | adopt a fabricated mechanism (a store, a test fixture) | 2 s |
| `recheck(design, all_parts=False)` | `RecheckReport`: `edited`, `checked`, `contract`, `clashes`, `bad_solids` | `contract.bad_solids`, `clashes`, edited parts inside their claims | 1-5 s |
| `verify(design, level)` | `VerifyReport` (below) | quick: check + plan + walk; standard: + build, contract at t = 0 and 3.2, clash and solids at the build's t, `verify_plan`, DXF pack, BOM; full: the audit's four contract angles, clashes at 1 and 4.38, and MuJoCo when it imports | 1 s / 30-45 s / 85 s |
| `export(design, formats?, out_dir?)` | `ExportReport`: `files`, `manifest` | what `spiderpig build` writes (`step`, `stl`, `print`, `dxf`, `bom`) plus `glb` (the viewer bake) and `mjcf`; always `manifest.json`; into the design's `exports/` in its store unless `out_dir` says where; a recorded export of these formats or more into the same folder, its files all still there, is returned as is | 5-60 s (the BOM's grouping dominates) |
| `load(id, store=PROJECT)` | `Design` | the recorded design (the id must hash to its record); reports load as the operations ask | ms |
| `derive(design, patch)` | `Design` | `resolve(apply_patch(spec, patch))` with `derived_from` and the patch recorded | ms |
| `compare(a, b)` | dict | the merge patch between two specs (and resolved specs), whether one derives from the other, every differing value per stage report | ms |
| `list_designs()`, `gc(keep, older_than)` | cards / removed ids | the store's folders | ms |

### Solids (decision 3)

After `build`, `design.parts[name]` is a `Part`: `solid` (the live build123d solid the
export and the clash check use, in the side's frame; `placed()` applies the pose),
`group` (`links`, `frame`, `drive`, `crank`, `pillar:<axis>`, `pin:<axis>`, `chassis`),
`side` (`L`/`R`/`None`), `fab`, `material`, `mass_g`, `volume_mm3`, `dims_mm`, `layers`,
`bom_key`, `rigid_with`. Replace `solid` to edit a part (`edited` then reads true).
**An edited solid is outside the correct-by-construction guarantee until `recheck`
passes**: it re-runs the solid and clash checks over every part and, for edited parts
of the claim-bound groups (links, crank, pillars, pins), checks they lie inside their
group's claims at the build's crank angle; a passing recheck accepts the edits.

### Failure

```
Failure {stage, code, message, culprits: [{body?, group?, joint?, point?, ...}],
         numbers: {margin_mm?, dist_mm?, need_mm?, fails_deg?, search_steps?, mm3?, ...},
         blockers: [{count, a, b, gap_mm, need_mm, text}],
         recommendations: [Recommendation{changes: [{name, before, after}], why, effects,
                                          verified, patch}],
         notes: [str]}
```

| stage | code | from |
|---|---|---|
| spec | invalid_spec, bad_parameter | `SpecErrors`, `ParamError` |
| program | loop_cannot_close | `AssemblyError` (the failing `StepCheck`'s numbers) |
| output | promise_broken | `OutputError` (the `OutputCheck`) |
| drive | second_input_no_drive | `ConstructionError` from the drive |
| construction | unbuildable | any other `ConstructionError` before planning |
| static | link_no_layer | `ClearanceError` (`NoCrankPoint` per culprit: dist, need, post, detour) + recommendations |
| plan | no_plan, no_plan_in_budget | `PlanError` (blockers parsed, recommendations) |
| fabricate | unbuildable | `ConstructionError` while building |
| contract / clash | part_outside_claim / parts_clash, bad_solid | `check_side`, `clashes`, `bad_solids` |
| layout / bom | part_exceeds_sheet / unknown_catalog_key | `layout.pack`, the BOM |
| walk / sim | linkage_invalid / sim_failed | `walk.api_payload`, MuJoCo |

A recommendation's `patch` is a spec patch (`{"linkage": {"params": {"unit": 10.5}}}` or
`{"fit": {"link_radius": 5.5}}`): a JSON merge patch (RFC 7386, `null` removes a key);
`apply_patch(spec_doc, patch)` merges it, `merge_patch(a, b)` is the smallest patch from
one document to another, `derive(design, patch)` resolves the patched spec.

## Store (step 2, decision 4)

Designs, their reports, parts and the plan cache live in a git-ignored folder next to
the spec: `$SPIDERPIG_STORE`, else `./.spiderpig`, created on the first write. Every
operation takes `store=`: the project store by default (`spiderpig.store.PROJECT`), a
path or a `Store`, or `None` to keep everything in memory. The store is a cache and a
record, never a second source of truth: the id is the design, and everything under it
recomputes from `resolved.json`. Several users share one folder (writes are atomic).

```
.spiderpig/designs/<id>/
  spec.json            the spec as given
  resolved.json        id, engine_version, spec_hash, created_at, derived_from, patch,
                       warnings, and `resolved` (every inferred value written in)
  check.json, plan.json, walk.json, recheck.json, verify.json, export.json
                       one file per stage: the report's JSON plus stage, design,
                       engine_version, written_at
  build/manifest.json  the build report, plus per part its pose, colour and file
  build/parts/*.step   one STEP per distinct part at the manifest's t (a right-side part
                       references its left twin: same_as + mirror, nothing duplicated)
  exports/             export()'s default out_dir
  log.jsonl            one line per operation: op, engine_version, seconds, ok, cached, at
```

What is cached, and when it is stale:

- A stage file is served as is when its `engine_version` is the running engine's; else
  it is recomputed and rewritten (a `load`ed design says so in `warnings`). `verify.json`
  holds the latest level; it answers a call for that level only. `export.json` answers
  the same formats into the same folder while every file is still there.
- A **plan** is never trusted blindly: `plan(design)` re-makes the stored layout through
  `stack.StackProblem.plan(layers, top, choices)` (the route rebuilt as
  `construction.crank.CrankRoute`) and checks it with `stack.verify_plan`, exactly as
  `fabricate._reuse` does (0.2 s on the quad against a 0.5 s solve; `PlanReport.reused`
  = `"store"`). From another engine version it is verified on fresh sampling and adopted
  without its optimality proof (`optimal = False`, the proof says why); when the check
  fails it is solved again. A new design of the same resolved spec under a new engine
  (a different id) seeds its plan from the old record the same way (`reused` = that id).
- A **build** reloads its parts from STEP (`api.attach_build`: masses, layers, groups
  and the envelope recomputed from the solids; `Part.pose` restored; the quad's 173 parts
  from 98 files in ~4 s against an 8 s build) when the manifest's `t` and engine match;
  otherwise it is rebuilt at the requested `t` and the files replaced. The files are what
  the engine built: an edited `Part.solid` lives in the session only.
- `check`, `walk`, `recheck`, `verify` are read back through `Report.from_dict`
  (`Failure.from_dict`, `Row.from_dict`); a failing stage is cached as its failure.

Warm against cold on the Klann quad: `verify("standard")` 40 s cold, ~0 s warm
(`verify.json`); a second process's `build` 4 s, `plan` 0.5 s, `check`/`walk` ms.

`list_designs()` returns one card per design (id, kind, linkage, module, sides, engine
version, created/last-used times, `derived_from`, the stages held with their `ok`, the
latest verify verdict). `gc(keep=[ids or handles])` removes every other design;
`gc(older_than=timedelta | datetime | seconds)` those last used before then; both
together remove only what is neither kept nor recent. `derive(design, patch)` records
the parent and patch on the child; `compare(a, b)` (handles or ids) returns
`{"spec_patch", "resolved_patch", "derived", "engine_version", "reports": {stage:
{"dotted.path": {"a", "b"}}}, "only_in"}`, comparing rows and steps by name and
skipping timings, tables and texts. `Store(root)` itself exposes the files
(`read_report`, `read_log`, `summary`, `load_mechanism`) for the MCP layer.

### VerifyReport

```
VerifyReport {level, ok, score, rows: [Row], failures: [Failure], unverified: [str], seconds}
Row {requirement, source, value, target, pass, tier: proven|measured|estimated, hard,
     detail, score?, weight, unit}
```

`ok` is false when any hard row fails or any stage failed; `score` is the weighted mean
over the soft targets of `max(0, 1 - miss / |bound|)`. Rows without a target are
informational (every metric the level measured is reported); `unverified` lists the
spec's targets the level didn't measure (budget at `quick`). What a row's `detail`
says: the ground clearance names the body part that sets it; the envelope at `quick`
is the joints' sweep plus the plates (x, y) and the stacks, the chassis and the axle
heads outside the outer plates (z), at `standard` the built extent at the build's
crank angle; the cost lists the largest items (packs are bought whole) and says when
unpriced items make the total a lower bound; `plan.warnings` (informational, soft)
carries the constructions' warnings; a walker whose stride reads near zero has the
walk note on its stride and speed rows.

## Worked example: spec to STEP

One side of the TrotBot heel at its drawing's 7 mm unit (35 s in all; the `single`
module because a one-leg-per-side robot doesn't walk in the quasi-static model, so
its stride would read 0):

```python
from build123d import Cylinder, Location
from spiderpig import api, apply_patch

spec = {"kind": "walker", "linkage": {"key": "trotbot_heel", "params": {"unit": 7}},
        "legs": {"module": "single", "sides": 1},
        "size": {"stack_mm": {"max": 45}}, "motion": {"ground_clearance_mm": {"min": 30}}}
design = api.resolve(spec)
report = api.check(design)
if not report.ok:                          # static: b7 passes crankpin J1 at 6.8 mm, needs 10
    failure = report.failures[0]           # stage "static", code "link_no_layer"
    rec = failure.recommendations[0]       # unit 7 -> 10.5, "checked: ... plans in 12 layers"
    design = api.resolve(apply_patch(spec, rec.patch))
plan = api.plan(design)                    # 12 layers, 36 mm, optimal
result = api.verify(design, "standard")    # rows with tiers; ok and score
print(result.describe())
api.build(design)                          # design.parts["b7"].solid: a build123d Part
link, body = design.parts["b7"], design.mech.body("b7")
a, b = (body.joint(j).pose.matrix[:2, 3] for j in body.outline[0])   # one outline segment
z = design.side.plan.z(link.layers[0])
link.solid = link.solid - Cylinder(1.5, 10).moved(Location((*((a + b) / 2), sum(z) / 2)))
assert api.recheck(design).ok              # the lightening hole: inside its claims, no clashes
files = api.export(design, ["step", "dxf", "bom"], "out/heel").files
```

## MCP (step 3)

`spiderpig/mcp/` serves the same operations over the Model Context Protocol (the
official `mcp` SDK, 2.x: `MCPServer`, the class FastMCP became). Across this boundary
everything is **files and numbers** (decision 3): a design is its id, a report is JSON,
a part is the path of its STEP file inside the store, and a failure is the `Failure`
document under `failures` of a result whose `ok` is false, never an exception's text.
The store (decision 4) is the state shared between calls; the server keeps nothing
else but its job pool.

```bash
spiderpig mcp --store .spiderpig      # stdio; also: mise run mcp, python -m spiderpig.mcp
```

`--store PATH` picks the store (else `$SPIDERPIG_STORE`, else `./.spiderpig`),
`--workers N` the processes for the long operations (default 2), `--log-level` the
server's logging (stderr; while serving, the SDK points fd 1 at stderr so nothing the
engine prints can reach the wire). A Claude Code / Claude Desktop entry:

```json
{
  "mcpServers": {
    "spiderpig": {
      "command": "uv",
      "args": ["run", "--directory", "/path/to/spiderpig", "spiderpig", "mcp",
               "--store", "/path/to/project/.spiderpig"]
    }
  }
}
```

### Tools

Every tool maps one-to-one onto `spiderpig.api`; each has an input schema (`resolve`'s
`spec` argument is the Spec's own JSON Schema, `spec_schema()`), a typed output schema
(the `TypedDict`s of `spiderpig/mcp/outputs.py`, every result an `{ok, failures, ...}`
envelope) and `readOnlyHint` / `idempotentHint` on everything but `export` and `gc`.
A design argument is the id `resolve` returned.

| tool | returns |
|---|---|
| `list_linkages(kind?)`, `describe(key)` | `linkages` (the list) / `card` (one linkage's card) |
| `catalog(category?)` | servos, sheets, constructions with prices, dims, rpm, torque, mass, hardware |
| `resolve(spec)` | `design` (the id), `resolved`, `engine_version`, `warnings`; an invalid spec: `ok: false`, `failures[0].code = invalid_spec`, `errors: [{path, message, allowed, nearest}]` |
| `check(design)`, `plan(design)`, `walk(design)` | the `CheckReport` / `PlanReport` / `WalkReport` as JSON (`walk` plans first so the feet sit at their layers) |
| `explain(design)` | `text` |
| `recommend(design)` | `stage` (the failing one), `recommendations` with their `patch`, and the failure's `notes` (what can't help, what wasn't checked) |
| `build(design, t?, wait_seconds?)` | the manifest: every part with `path` (its STEP in the store's `build/parts/`), `dir`, masses, envelope; **a job** |
| `verify(design, level?, wait_seconds?)` | the `VerifyReport` (rows with `pass`, `tier`, `hard`); `quick` inline, `standard` / `full` **jobs** |
| `export(design, formats?, out_dir?, wait_seconds?)` | `files` (paths) and `manifest`; **a job** |
| `get_job(job)`, `wait_job(job, seconds?)` | the same shape as the long tool itself: while it runs, `job` alone (`{job, op, design, args, state, started_at, finished_at, seconds}`; its `job` field is the id these take); once done, the tool's own result flat beside `job`; failed, `ok: false` with the failure |
| `compare(a, b)`, `derive(design, patch)` | as the Python API |
| `get_design(design, stage?)` | `summary`, `spec`, `resolved`, `check`, `plan`, `walk`, `build` (the manifest with paths), `recheck`, `verify`, `export`, `log` |
| `list_designs()` | the store's cards |
| `gc(keep?, older_than_seconds?)` | `removed`; refuses to run without either argument |
| `view(design)` | `url` of the viewer for the design (`?design=<id>`), `server` (its base URL) and `mode`: a `spiderpig view --serve-only` child process over the store, started on a free port on the first call and reused (stopped with the server); the page's first load bakes the design unless it was exported (`ok: false`, code `viewer_not_built`, when the package has no built viewer) |

`tune` and `search` are not in v1 (the guide says so); `recheck` needs solids and stays
in the Python API. `view` is the one tool that isn't an operation: it serves the
viewer (below).

**Failures.** A stage that fails is an ordinary result: `ok: false` and `failures` as
data (stage, code, message, culprits, numbers, blockers, recommendations with patches,
notes). Only a *misuse* of a tool (an unknown design, a malformed id, an unknown
linkage, `gc` without arguments) sets `isError`, and its payload is the same envelope
(`store` / `no_such_design`, `bad_design_id`, `no_such_stage` (a stage the design
doesn't hold yet), `gc_needs_arguments`; `spec` / `unknown_linkage`; `job` /
`no_such_job`). An unknown export format or stage name never reaches a tool: the
input schema (`Literal`) refuses it, and the SDK answers `isError` with its own text
naming the allowed values. A programming error in the engine crosses the same way
(stage `engine`, code = the exception's class) and is logged with its traceback on the
server.

**Long operations.** The installed SDK carries the wire types of task-augmented
execution but its server doesn't run tools as tasks, so `build`, `verify` at
`standard` / `full` and `export` run in a process pool (one per store, workers
spawned fresh: a clean `fabricate._DESIGNS` per process, no fork of the threaded
server). Each waits `wait_seconds` (default 15) and returns the finished result with
its `job` record, or the running `job` alone (`{ok: true, failures: [], job: {job: <id>,
op, design, args, state, started_at, finished_at, seconds}}`) for `wait_job` /
`get_job`, which answer in that same shape (the finished result flat beside `job`). A
worker loads the design from the store, runs the Python operation (which writes its
report, parts or files into the store) and returns the report's JSON; the reports are
in the store either way (`get_design`). Job records live in the server process. Short
operations run in a worker thread, one at a time, so the loop keeps answering.

### Resources and prompts

| resource | content |
|---|---|
| `spiderpig://guide` | how to design with spiderpig (markdown): the passes and what each proves, the Spec vocabulary with defaults and hard/soft (generated from `TARGET_FIELDS`), the linkages, the catalog, the two loops, jobs, the limits of v1 |
| `spiderpig://schema/spec` | the Spec's JSON Schema |
| `spiderpig://linkages/{key}` | a linkage's card (also listed per linkage) |
| `spiderpig://catalog/{servos\|sheets\|constructions}` | the catalog |
| `spiderpig://designs/{id}/{stage}` | a stored stage as JSON (`build` is the manifest with part paths) |

Prompts: `design_walker(goal)` (resolve → check → plan → verify → export),
`diagnose(design)` (explain → recommend → derive), `iterate(design, metric)` (a
derive / compare loop on one metric).

`tests/test_spiderpig_mcp.py` drives the server through the SDK's in-memory client
(`mcp.Client(server)`, no subprocess). A cold `resolve → verify("quick")` on the Klann
single takes ~0.7 s through the client once the engine is imported (~3 s of imports
before that); `build` of the single as a job ~10 s including the worker's start.

## View (step 4, decision 6)

The Python package ships the built viewer as package data (`spiderpig/viewer/dist`,
Vite's output; `mise run viewer-build` makes it, `mise run release` makes it and
then the sdist and wheel, and a wheel built without it fails with a message saying
so), so `spiderpig view` needs no Node on the user's machine:

```bash
spiderpig view <design> [--store PATH] [--port N] [--open] [--no-export]
```

It picks the store as the MCP server does (`--store`, else `$SPIDERPIG_STORE`, else
`./.spiderpig`), loads the design, runs `api.export(design, ["glb"])` (built and baked
once, cached in the store's `exports/`), starts the FastAPI app of
`spiderpig/server/app.py` on a free port serving the built viewer, prints the URL,
`http://127.0.0.1:<port>/?design=<id>`, and serves until Ctrl-C (`--open` opens the
browser). What the page does with `?design=<id>`:

- `GET /api/design/{id}` is the design's card from its `resolved.json`: `kind`,
  `linkage`, `module`, `sides`, `mode` (`robot`, or `side` for a one-sided design),
  `phases_deg`, `params` (the linkage's proportions), `servo`, and `glb`, the URL of
  its bake. The viewer seeds its mode and the tune panel's state (linkage, module,
  phases, parameter sliders) from it.
- Every `/api/glb/{mode}` and `/api/walk` query the page makes then carries
  `design=<id>` first: the server resolves the design's `BuildConfig` from the store
  (servo, sheet, thickness, constructions and fit included, which no query string
  expresses) and applies the tune panel's own `module`, `phases` and `p.NAME` on top
  of it, so the sliders edit the viewed design; a different `linkage` starts from
  that linkage's defaults but keeps the design's materials and constructions.
- The glb of the design itself is served from the store's export when it exists and
  is newer than the package's sources (response header `X-Spiderpig-Glb: export`);
  an edited design bakes into the store's `bakes/` under its config key (`bake`),
  as every dev-server bake does.

`python -m spiderpig.view --serve-only --store PATH --port N` serves a store without
a design: the MCP `view` tool starts one such child process per server (reused
across calls, stopped with the server) and returns the design's URL.

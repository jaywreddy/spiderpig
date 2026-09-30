# Designing with spiderpig

spiderpig is a compiler from a **Spec** (what you want: a walking linkage or a
mechanism, its module, materials, constructions and the numbers it must meet) to
**verified geometry** (a layer plan proven clash-free over the whole crank cycle,
every part built inside its planned claims, STEP/STL/DXF/BOM files). You never draw:
you name a linkage, set targets, and ask the engine to resolve, check, plan, walk,
build, verify and export. Every stage says exactly what fails and, where it can, what
would clear it.

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
| resolve | `resolve(spec)` | the spec validates; every omitted value is inferred and written into `resolved`; the id | ms |
| check | `check(design)` | **program**: every loop closes over the cycle (margins, transmission angles); **output**: a mechanism keeps its promises; **drive**: one servo turns the crank; **static**: every link has a crank point (a crankpin or a detour the body's underside allows); the ground clearance | 0.5 s |
| plan | `plan(design)` | a layer plan of one side exists: the planner guarantees the claims of different groups never meet over the whole cycle; `optimal` says whether it is proven the thinnest (`proof` says what a budget left open) | 0.4 s (cached: a re-check) |
| walk | `walk(design)` | the quasi-static walk model's metrics (stride, bob, slip, tipping, speed at the servo's no-load rpm) with the feet at their planned layers | 0.1 s |
| build | `build(design)` | every part fabricated at crank angle `t`, inside its claims; masses, the envelope, one STEP per distinct part in the store | 8 s (a job) |
| verify | `verify(design, level)` | pass/fail per requirement with an evidence tier: `quick` = check + plan + walk; `standard` adds the build, the contract at two crank angles, OCCT clashes and solids, the plan re-verified on fresh sampling, the DXF pack and the BOM; `full` adds two more contract angles, a second clash angle and a MuJoCo run | 1 s / 30-45 s / 85 s |
| export | `export(design, formats)` | the files: `step`, `stl`, `print` (one STL per printed part), `dxf` (kerf-compensated sheets), `bom` (csv/md/json), `glb` (the viewer's animated bake), `mjcf`; always `manifest.json` | 5-60 s (a job) |

Evidence tiers on every verify row: **proven** (the engine's guarantees: loops close,
a plan exists and re-verifies, parts stay inside their claims, the stack height),
**measured** (numbers read off models or solids: walk metrics, foot path, ground
clearance, clashes, masses, sheets), **estimated** (nominal inputs: the mass before a
build, the envelope before a build, `speed_mm_s` at the servo's no-load rpm, prices).

## The two loops

**Design**: `resolve → check → plan → verify("quick") → verify("standard") → export`.
Read `ok` at every step. `resolve` returns the id every other tool takes. Stop at the
first `ok: false` and read `failures` (below). `verify` scores the soft targets and
fails on any hard miss; `export` writes the files you asked for into the store's
`exports/` (or `out_dir`) and returns their paths.

**Diagnose**: `explain → recommend → derive`. `explain` is every stage's verdict in
prose. `recommend` is the engine's checked fixes for the failing stage, each with a
spec **patch** (a JSON merge patch: `{"linkage": {"params": {"unit": 10.5}}}` or
`{"fit": {"link_radius": 5.5}}`), given only after the stage passes with it. `derive(
design, patch)` resolves the patched spec as a new design that records its parent, so
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
| plan | no_plan / no_plan_in_budget | no layer plan (the blockers with gaps and needs; recommendations) |
| fabricate / contract / clash | unbuildable / part_outside_claim / parts_clash | a part can't be made or leaves its claims or intersects another |
| layout / bom | part_exceeds_sheet / unknown_catalog_key | a part is larger than the sheet; a hardware key is unknown |
| walk | linkage_invalid | the walk model can't use the linkage |
| store | no_such_design, bad_design_id, no_such_stage, gc_needs_arguments | a tool was misused (`isError` is set) |

## The Spec

A JSON object; `spiderpig://schema/spec` is its JSON Schema and `resolve` validates
it exactly (unknown fields, wildcards such as `any` or `*`, and bare numbers where a
target is expected are errors with the nearest allowed key). One spec compiles one
design: there is no search in v1.

| field | values | default |
|---|---|---|
| `version` | `"1"` | `"1"` |
| `kind` | `walker` or `mechanism` | required |
| `linkage.key` | a registered linkage (below) whose kind matches | required |
| `linkage.params` | `{name: number}` overrides of that linkage's parameters (`describe` lists them; lengths in mm, angles in degrees) | the linkage's defaults |
| `legs.module` | one of the linkage's modules: `single`, `double`, `decker`, `quad` or its own | `quad` (walker), `single` (mechanism) |
| `legs.phases_deg` | one crank phase per leg of the module | the module's |
| `legs.sides` | `2` (the robot: two mirrored sides and the chassis) or `1` (one side) | 2 (walker), 1 (mechanism) |
| `materials.sheet` | a sheet item (sets the layer pitch) | `acrylic_3mm` |
| `materials.thickness_mm` | a measured sheet thickness | the sheet's nominal |
| `materials.servo` | a continuous-rotation servo | `sts3215` |
| `constructions.pillar`, `.pin` | `printed`, `rod`, `bolt`, `bearing`, `bushing` | `printed` |
| `constructions.crank` | `printed` | `printed` |
| `fit.*` | part sizes and fits in mm (below), plus `kerf_mm` and `sheet_size_mm: [w, h]` | the engine's defaults |
| `outputs` | a list of `step`, `stl`, `print`, `dxf`, `bom`, `glb`, `mjcf` | `[step, stl, print, dxf, bom]` |
| `motion.*`, `size.*`, `budget.*` | **targets** (below) | none |

A **target** is `{min?, max?, value?, tol?, weight: 1, hard?}` (never a bare number).
`value` is met within `tol` (default 5 % of the value). A **hard** miss fails
`verify`; a **soft** miss lowers its score (the weighted mean of `max(0, 1 - miss /
bound)` over the soft targets). Physical limits are hard by default, gait quality
soft; `hard: true|false` on any target flips it.

<<TARGETS>>

`fit` defaults (mm): <<FIT>>

### Linkages

<<LINKAGES>>

`describe(key)` gives a linkage's card: parameters (which only scale it), links, the
closures at the defaults, one foot's path numbers (lift, stance stride) or the output
check, and its modules with their default phases.

### Materials and constructions (`catalog`)

<<MATERIALS>>

## Long operations

`build`, `verify` at `standard`/`full` and `export` run in worker processes. Each
waits `wait_seconds` (default 15) and returns the finished result when the job is
done; otherwise it returns `job` with `state: running` and you follow it with
`wait_job(job, seconds)` or `get_job(job)`. Reports land in the store either way, so
`get_design(design, "build")` reads the manifest afterwards. Job records live in the
server process; the store keeps what they produced.

## Limits of v1

- Leg modules are named presets (`single`, `double`, `decker`, `quad`, a linkage's own)
  with phases per leg; no explicit leg lists.
- One machine per spec: no compound machines, no stacking of mechanisms.
- No `tune` and no `search`: one spec compiles one design; move parameters yourself
  with `derive` and read `compare`.
- `speed_mm_s` is the stride at the servo's no-load rpm, tier `estimated`, until a
  MuJoCo run (`verify("full")`) reports beside it.
- Prices are the catalog's preferred offers, unverified.
- A walker with `sides: 1` builds one side; its walk metrics still model the two-sided
  robot.

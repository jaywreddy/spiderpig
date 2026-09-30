# spiderpig as a compiler: the Python API (harness v1, step 1)

An agent writes a **Spec** and uses the engine as a compiler to verified geometry.
Everything lives in the `spiderpig/` package: `spec.py` (the vocabulary), `api.py`
(the operations), `failure.py` (every engine exception as data), `verify.py` (the
harness), `design.py` (the handle). CLI, MCP and the per-project store come in later
steps; this document is the surface they will wrap.

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

Semantics are pinned once, in `TARGET_FIELDS`: robot metrics come from `walk.py`;
per-leg foot-path numbers (stance stride, ripple) appear only on the linkage card
(`api.describe`), except `lift_mm`, which is a target.

## Operations (`spiderpig.api`)

Pure functions of the `Design` handle, synchronous, each storing its report on the
handle (`design.reports[stage]`) and returning it. A design that merely fails a stage
gets a report with `ok = False` and `failures: [Failure]`; operations raise only for
programming errors (and `resolve` raises `SpecErrors` for an invalid spec).

| op | returns | what it runs | cost (Klann quad) |
|---|---|---|---|
| `resolve(spec)` | `Design` (`id`, `resolved`, `config`, `engine_version`, `warnings`) | validation, inference, `BuildConfig`; `id = sha256(canonical resolved spec + engine version)[:16]`, engine version = package version + hash of `linkages/*.py` + `StackSpec` defaults | ms |
| `list_linkages(kind?)`, `describe(key)` | linkage cards | registry, `Linkage.check`, foot path / output check | ms |
| `check(design)` | `CheckReport`: `steps`, `output`, `foot_path`, `drive`, `clearances`, `crank_facts`, `ground_clearance_mm` | `Linkage.check`, `output_check`, `side_problem`, `static_stage` | 0.5 s |
| `plan(design)` | `PlanReport`: `layers`, `n_layers`, `height_mm`, `route`, `optimal`, `proof`, `table` | `fabricate.design_side` (cached by the engine) | 0.4 s |
| `explain(design)` | text | `explain.explain` | ms after plan |
| `recommend(design)` | `[Recommendation]` with `patch` | the failing stage's checked recommendations | 1-30 s |
| `walk(design)` | `WalkReport`: `metrics`, `mass_g`, `rows` | `walk.api_payload` (feet at their planned layers) | 0.1 s |
| `build(design, t=1.0)` | `BuildReport`: `parts` manifest, `mass_g`, `envelope_mm`, `counts`; `design.parts[name]` | `fabricate.fabricate` | 8 s |
| `attach_build(design, mech, t)` | `BuildReport` | adopt a fabricated mechanism (a store, a test fixture) | 2 s |
| `recheck(design, all_parts=False)` | `RecheckReport`: `edited`, `checked`, `contract`, `clashes`, `bad_solids` | `contract.bad_solids`, `clashes`, edited parts inside their claims | 1-5 s |
| `verify(design, level)` | `VerifyReport` (below) | quick: check + plan + walk; standard: + build, contract at t = 0 and 3.2, clash and solids at the build's t, `verify_plan`, DXF pack, BOM; full: the audit's four contract angles, clashes at 1 and 4.38, and MuJoCo when it imports | 1 s / 30-45 s / 85 s |
| `export(design, formats?, out_dir)` | `ExportReport`: `files`, `manifest` | what `main.py` writes (`step`, `stl`, `print`, `dxf`, `bom`) plus `glb` (the viewer bake) and `mjcf`; always `manifest.json` | 5-60 s (the BOM's grouping dominates) |

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
`{"fit": {"link_radius": 5.5}}`); `apply_patch(spec_doc, patch)` merges it.

### VerifyReport

```
VerifyReport {level, ok, score, rows: [Row], failures: [Failure], unverified: [str], seconds}
Row {requirement, source, value, target, pass, tier: proven|measured|estimated, hard,
     detail, score?, weight, unit}
```

`ok` is false when any hard row fails or any stage failed; `score` is the weighted mean
over the soft targets of `max(0, 1 - miss / |bound|)`. Rows without a target are
informational (every metric the level measured is reported); `unverified` lists the
spec's targets the level didn't measure (budget at `quick`).

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

# spiderpig as a compiler for linkage machines that agents drive — scope

> **Status: historical.** The agent surface's original proposal (2026-09-30). What was built is
> documented in [API.md](API.md); the decisions it raised are in [DECISIONS.md](DECISIONS.md).
> Names below that were renamed since point at what was built; names in plain text are the
> proposal's and were never built.

Repo `/home/user/spiderpig` at `ac60cfa` (branch claude/great-carson-6nw2bz), read-only study. Assumes the code audit's plan lands (validating `BuildConfig`, `spiderpig` CLI, `linkage/` split into engine/checks/assembly, one design cache, `Group` protocol + registry) and that the planner rewrite in worktree `agent-ad3e24b76a3850113` merges (route search over whole stackups, `stack.Recommendation`, `fabricate.ground_clearance`, `StackPlan.optimal/proof/cost`, `ClearanceError(PlanError)`). Where this scope overlaps either, it says so.

## 0. The proposal in one paragraph

Keep the engine (symbolic program → stage checks → planner → constructions → outputs) and put three thin things around it. (1) A **Spec**: what the agent wants, as a JSON-schema'd dataclass, and a **resolver** that maps it onto a linkage + params + `BuildConfig`, infers the rest, and says precisely when it can't (unknown value → nearest; conflicting targets → the arithmetic). (2) A **Design store**: one content-addressed directory per resolved spec where every pass writes its result as JSON (+ part files), so every operation is load-or-compute, resumable, diffable and referenceable from another design. (3) An **operations façade** (`spiderpig.api`: ~16 pure functions returning dataclasses), with the CLI, an MCP server and HTTP as thin views, exposing the same operations as JSON-schema'd tools whose failures are **data** (`stage`, culprits, numbers, verified recommendations) and whose slow calls are MCP Tasks. The one engine change that matters most is finishing what the rewrite started: every stage result and failure as a value, not formatted text. The planner rewrite already does it for `PlanError.recommendations`; `blockers`, `ClearanceError`, `AssemblyError`, `ConstructionError`, `verify_plan`, `check_side` and `layout.pack` still format strings from numbers they hold in hand.

## 1. The repo's shape: passes and artefacts

| # | pass | function today | input → output | cached | raises (text) | serialisable today? |
|---|---|---|---|---|---|---|
| 1 | resolve | four parsers (walk's, bake's, the old main.py's and `explain`'s; since merged into `config.config_from_args`) | CLI/query args → `BuildConfig` | – | `ParamError` | yes (frozen dataclass); not validating (audit #2) |
| 2 | program check | `Linkage.check(params)`, `output_check` | (key, values) → `tuple[StepCheck]`, `OutputCheck` | `@cache` per process | – (data) | yes (frozen dataclasses); emitted only by `linkage_report` and `/api/linkages` |
| 3 | template | `build_module_template` | (linkage, module, phases, params) → `MechanismTemplate` (+`meta`) | no | `AssemblyError`, `OutputError` | no (closures); recomputable from `meta` in ~0 s |
| 4 | static facts | `side_problem`, `side_clearances`; rewrite: `route.crank_facts`, `ground_clearance` | template + config → `Context`, groups, `StackProblem`, `[Clearance]`, ground clearance | no | `ConstructionError` (a second input has no drive), `ClearanceError` | `Clearance`/`Keepout`/`DriveInterface`: yes; groups: rebuilt from config |
| 5 | plan | `StackProblem.solve()` | claims → `StackPlan{layers, top, placed, choices{crank: CrankRoute}, optimal, proof, cost}` | `_DESIGNS`/`_LAYOUTS` per process | `PlanError{summary, blockers: list[str], notes, recommendations}` | layers/top/choices/placed/optimal/proof: yes (`Placed` is data; `CrankRoute` is `Run`s); `claims` are closures — `fabricate._reuse` already reconstructs a plan from `(layers, top, choices)` via `problem.plan()` + `verify_plan` |
| 6 | design | `SideDesign{config, ctx, groups, plan, clearances, ground_clearance_mm}` | – | process | – | **implicit**: never written anywhere |
| 7 | fabricate | `fabricate(tmpl, config, t)` | design + t → `Mechanism{Body{part, fab, bom_key, rigid_with}, meta, bom_extras}` | no | `ConstructionError` | parts as STEP/STL: yes; `meta`: untyped dict; `Realized{cuts, pads, extras}`: transient |
| 8 | verify | `verify_plan`, `contract.check_side`, `contract.clashes` / `bad_solids`, `layout.pack`, BOM resolve | design (+t) → `list[str]` / dicts | no | `ValueError`, `KeyError` | text lists |
| 9 | analyses | `walk.api_payload`, `sim.run.simulate/walk_metrics`, `tune_gait`, `linkage_report` | config → JSON dicts | `lru_cache`, `@cache` MjModel | `LinkageError` | yes (already JSON) |
| 10 | outputs | `build.py` (then main.py), `layout.save_sheets`, `Bom.write`, `bake_gltf`, `build_mjcf` | mech → files | server `.glb` cache keyed by config hash | – | files |

Measured here (Klann quad robot unless noted): check 0.17 s (0 cached) · template ≈ 0 · **plan 0.6 s** (single 6.1 s; Jansen double 0.2 s; Strider/sixbar quad ≈ 22 s per the linkage report; trotbot fails in 0.1–16 s) · walk 0.06 s · **fabricate 8.4 s** (one side 2.0 s) · verify_plan 0.3 s · **check_side 9.7 s per crank angle** · layout.pack 0.8 s · **BOM 61 s** (the `congruent` pairwise grouping the audit flagged: slower than fabrication) · MuJoCo: model build + compile 11.4 s (includes a fabrication), then 3 s of simulation in 0.9 s · glb bake ≈ 6.6 s. A full `export` today is ~90 s, of which two thirds is the BOM grouping.

Already first-class data: checks (2), the plan's core (5), the analyses (9). Implicit: the design key itself (split between `tmpl.meta` and `BuildConfig`), `SideDesign`, `StackPlan.choices` (never serialised), `Realized`, `Mechanism.meta`, and every failure's numbers (`StackProblem._blocked_by` holds the `Placed` pairs and gaps that `blockers()` renders; `Clearance` has `dist`/`need`; `StepCheck` has margins and angles).

Determinism: the program is compiled once from exact rationals; the search is depth-first over a sorted order with fixed node budgets; OCCT, ezdxf and rectpack are deterministic per version. A design is therefore a pure function of (spec, engine version, `StackSpec` budgets) — sound for content addressing if version and budgets are in the key. Caveat the rewrite already names: a plan found under a budget is *a valid plan*, and `optimal`/`proof` say whether it is the thinnest.

## 2. Patterns from agent-facing tool libraries

| pattern | seen in | what it means here |
|---|---|---|
| JSON-schema'd tools, structured results, errors *inside* results | MCP 2025-11-25: inputSchema, outputSchema, structuredContent; execution errors returned with `isError` and actionable text so the model can self-correct, protocol errors kept apart | every op returns a typed report; a failure is a row with recommendations, never a traceback |
| long-running calls as tasks | MCP Tasks (taskId, working/completed/failed, pollInterval, ttl, cancel; per-tool execution.taskSupport); Zoo text-to-CAD: async operation id, cache hit returns at once | `tune`, `search`, `simulate`, `export`, `verify(full)` are task-capable; a store hit returns immediately |
| check-as-you-go with evidence tiers | build123d-mcp: measure/validate/verify_spec with tiers measured/structural/recognised/unverified; `clearance` status; "don't proceed until measure passes"; a failed step doesn't advance state | `verify` rows carry a tier: proven (planner + contract + `verify_plan`), measured (OCCT, walk, sim), estimated (nominal mass, unverified prices) |
| snapshots and diffs | build123d-mcp save/restore/diff snapshot by metrics; Zoo's agent inspects and snapshots geometry | design ids with `derived_from` + patch; `compare` diffs specs and reports |
| determinism, value semantics | Onshape FeatureScript: regenerates identically everywhere, no hidden state, no time/randomness | content addressing is sound only while the pipeline is pure; keep wall clock and globals out of results |
| objectives vs limits vs manufacturing constraints; several outcomes | Fusion generative design: objectives and limits, manufacturing constraints, an outcome per method | the Spec separates hard limits from weighted objectives; `search` returns a ranked set |
| repair hints attached to the error | build123d-mcp's repair_hints; the rewrite's `Recommendation` (checked by re-running the stage) | the universal failure payload |
| intent declared, then verified | build123d-mcp's verify_spec; FeatureScript "design intent as inputs" | the same Spec drives `resolve` and `verify` |

Sources: [MCP tools](https://modelcontextprotocol.io/specification/2025-11-25/server/tools), [MCP tasks](https://modelcontextprotocol.io/specification/2025-11-25/basic/utilities/tasks), [Zoo text-to-CAD](https://zoo.dev/docs/developer-tools/api/ml/generate-a-cad-model-from-text), [build123d-mcp](https://github.com/pzfreo/build123d-mcp/blob/main/llms.md), [mcp-build123d](https://github.com/Casys-AI/mcp-build123d), [FeatureScript](https://cad.onshape.com/FsDoc/intro.html), [Fusion generative design](https://www.autodesk.com/autodesk-university/article/Fusion-360-Introduction-Generative-Design).

## 3. The agent-facing surface

### 3.1 Spec

A frozen dataclass `Spec` in `spiderpig/spec.py`, version-stamped; JSON Schema generated from it at the tool layer (a pydantic TypeAdapter or msgspec there only; as built, `spec.spec_schema()` writes the schema by hand — the audit's "no pydantic" verdict was about `BuildConfig`'s cross-field validation, which stays hand-written in the resolver).

```
Spec v1
  version: "1"
  kind: "walker" | "mechanism"                          # required ("compound" reserved, §4 #16)
  linkage: {key | family | "any" = "any", params: {name: value}, lock: [name...]}
  legs: {per_side: 1|2|4 | module: str | explicit: [{orientation: ±1, phase_deg}], sides: 1|2 = 2}
  motion:
    walker:    stride_mm, lift_mm, speed_mm_s, bob_mm, slip_mm_per_rev, stability_margin_mm,
               tipping_fraction, ground_clearance_mm, yaw_deg_per_rev
    mechanism: motion: line|rotation|translation_platform|xy|path, stroke_mm, straightness_mm,
               swing_deg, dwell_deg, on_line_fraction
  size: {envelope_mm: [x, y, z], stack_mm, mass_g}
  materials: {sheet: key | {thickness_mm, kind}, filament: key,
              servo: key | {min_rpm, min_torque_kgcm, max_price_usd}}
  constructions: {pillar, pin, crank: key | "any", bearing: bool}   # "keep bearing"
  fit: subset of construction.Params (margin, link_radius, fits...) + kerf_mm + sheet_size_mm
  budget: {cost_usd, print_g, sheets}
  outputs: [step, stl, print, dxf, bom, glb, mjcf, report]
```

Every metric is a `Target = {min?, max?, value?, weight: 1.0, hard: bool}`. Hard targets are constraints (`verify` fails on them; `resolve` checks them for mutual consistency); soft ones are objective terms (`search`/`tune` weigh them; `verify` only reports them). Required: `kind`, plus either `linkage.key` or at least one motion target. Everything else is inferred and the inferred values are written to `resolved.json`, so a stored design never depends on a default that later moves.

Mapping onto today's values: `linkage.key/params`, `legs` → `build_module_template(module, phases, proportions, linkage)`; `materials`, `constructions`, `fit` → `BuildConfig(sheet, thickness, servo, pillar, pin, crank, params)`; `sides` → `robot`; `fit.kerf/sheet_size` → export options; `motion`, `size`, `budget` → nothing in the engine today: they live in the Spec and are consumed by `verify`, `search`, `tune` and `recommend`. Metric definitions must be named once (three "stride" definitions exist: walk model, `sim.run.kinematic_gait`, `linkage_report.foot_path`); proposal: robot metrics from the walk model, per-leg foot-path numbers on the linkage card only.

### 3.2 Operations

Python: `spiderpig.api`, synchronous, pure functions of a `Design` handle; CLI, MCP and HTTP call these and add nothing.

| op | in → out | cost | cache | failures |
|---|---|---|---|---|
| `list_linkages(kind?, filters?)` | → `[LinkageCard]` | ms | pure | – |
| `describe(linkage)` | key → card: params (+angle flag, scale params), links + labels, feet/output, closures at defaults (margins, transmission, toggles), foot-path numbers (lift, stride, stance, ripple), modules and which plan (golden), sources | ms (foot path 50 ms) | package golden data | an unknown key + the nearest (built: a `KeyError` naming it) |
| `resolve(spec)` | Spec → `Design` (id, resolved `BuildConfig`, inferred fields, warnings) | ms | pure | `SpecError[]`: path, message, allowed, nearest, conflicts with arithmetic |
| `check(design)` | → `CheckReport`: step checks, output check, drive, static facts (clearances, crank facts), ground clearance | < 1 s | store | rows of `Failure` |
| `plan(design)` | → `PlanReport`: layers, top, height, route, `optimal`/`proof`, ground clearance, blockers, recommendations | 0.1–25 s | store | `Failure(stage=plan)` |
| `explain(design)` | → today's `explain.py` text + the two reports | ms after plan | store | – |
| `recommend(design)` | → `[Recommendation + spec patch]`, each verified by re-running the failing stage | 1–30 s | store | – |
| `walk(design)` | → `WalkReport` (`walk.api_payload` metrics, each against its Target) | 0.1 s | store | – |
| `tune(design, free=[phases, params...], budget)` | → ranked `[Candidate{patch, metrics, objective, plans?}]` | 10 s – minutes | store; task | – |
| `build(design, t?)` | → `BuildReport` (manifest, masses, meta); parts into the store | 2–10 s | store | `Failure(stage=fabricate)` |
| `verify(design, level)` | → `VerifyReport` (§3.4) | quick 1 s · standard ~30–90 s · full minutes | store; task | rows |
| `simulate(design, controls, seconds)` | → a SimReport (`sim.run.walk_metrics`; never built as an op: `verify` full runs the sim) | ~12 s + 0.3 s per simulated second | store per controls hash; task | `Failure(stage=sim)` |
| `export(design, formats, out)` | → `[files]` + manifest | 5–90 s (61 s of it BOM today) | store; task | pack error → `Failure(stage=layout)` |
| `compare(ids)` | → spec diff (JSON patch) + report diff | ms | pure | – |
| `search(spec, candidates?, objective?)` | → ranked `[Design]` over linkage × module (+ `tune` on the top few) | minutes | task; one store entry per candidate | – |
| `compose(stages)` | reserved | – | – | – |

Structured failure, the boundary type (exceptions stay as they are inside the engine; `Failure.from_exception` maps them):

```
Failure {
  stage: spec|program|output|drive|static|plan|fabricate|contract|clash|layout|bom|walk|sim
  code:  loop_cannot_close | promise_broken | second_input_no_drive | link_no_layer | no_plan |
         no_plan_in_budget | no_plan_in_time | screw_no_stock_length | part_outside_claim | parts_clash |
         part_exceeds_sheet | unknown_catalog_key | ...
  message: str                                  # today's text, unchanged
  culprits: [{body?, group?, joint?, layer?, side?, point?}]
  numbers:  {margin_mm?, fails_deg?, dist_mm?, need_mm?, gap_mm?, count?, layers?, mm3?, ...}
  recommendations: [Recommendation{change, before, after, why, effects, verified, patch}]
  notes: [str]
}
```

| exception today | carries after |
|---|---|
| `AssemblyError` (text) | the failing `StepCheck` (point, refs, radii, margin, fails_deg, worst_t2) |
| `OutputError` (text) | the `OutputCheck` + the promise it breaks |
| `ClearanceError` (text) | `[Clearance]` (link, keepout owner, dist, need) + rewrite's crank-facts failures + recommendations |
| `PlanError` (`blockers: list[str]`) | `blockers: [Blocker{a: Placed, b: Placed|None, count, gap_mm, need_mm}]` — the data `_blocked_by` already holds — plus `notes`, `recommendations` |
| `ConstructionError` (text) | `{group, construction, what, value, need}` |
| `verify_plan`, `check_side`, `clashes`, `bad_solids` (`list[str]`/dicts) | row objects with `.describe()` |
| `layout.pack` `ValueError` | `{part, size_mm, sheet_mm}` |
| BOM `KeyError` | `{bom_key, body}` |

### 3.3 Design handle and checkpoints

`Design.id = sha256(canonical_json(resolved spec) + engine_version)[:16]`; `engine_version` = package version + hash of `linkages/*.py` + `StackSpec` defaults. Store root `$SPIDERPIG_STORE`, default `./.spiderpig/designs/<id>/`:

| file | written by | content |
|---|---|---|
| `spec.json`, `resolved.json` | resolve | the spec; `BuildConfig` + inferred values + engine version + `derived_from` + JSON patch |
| `check.json` | check | step checks, output check, clearances, crank facts, ground clearance, failures |
| `plan.json` | plan | layers, top, pitch, `choices` (`{crank: {runs: [{at, lo, hi}], bearing}}`), placed, optimal, proof, cost, failures, recommendations |
| `walk.json`, tune.json, sim/<controls-hash>.json | analyses | as today's JSON |
| `build/manifest.json`, `build/parts/<part>.step` | build | one STEP per distinct part (t recorded), §3.5 |
| `verify.json` | verify | rows |
| `exports/` | export | STEP/STL/print/laser/bom/glb/mjcf/report |
| `log.jsonl` | all | op, engine version, seconds |

Loading a design rebuilds the engine objects: the template from `resolved.json` (free), the plan from `plan.json` via `StackProblem.plan(layers, top, choices)` + `verify_plan` (this is exactly `fabricate._reuse`; ~0.5 s), so nothing with closures is pickled and a plan from another engine version is re-verified before use. `derived_from` + patch give the iteration history and make `compare` cheap. Referencing another design (stacking) is `stages: [{design: id, mount: output.frame}]` in a compound spec, reserved. The viewer attaches by id: `spiderpig view <id>` serves `exports/<id>.glb` (the server's two query-string caches become the store — the audit's "one cache").

### 3.4 Harness: `verify(design, level) -> VerifyReport`

```
Row {requirement, source, value, target, pass, tier: proven|measured|estimated, detail}
```

- **proven**: every loop closes (`StepCheck.margin ≥ 0`), output promises hold, a plan exists and `verify_plan` re-checks it on fresh 2880-sample geometry, `check_side` at N crank angles, `optimal` (else `proof` says why not).
- **measured**: OCCT pairwise clashes and one-valid-solid (today's audit), DXF packs on the sheet, BOM keys resolve, walk metrics vs targets, sim metrics vs targets (full level), ground clearance, stack height, mass from parts.
- **estimated**: nominal mass (walk without parts), prices with unverified links, sim assumptions (`SimParams.crank_armature` is marked unverified in the source).

Levels: quick (check + plan + walk, ~1 s), standard (+ build + contract at 2 angles + clash at 1 + DXF + BOM; ~90 s today, ~30 s after the BOM fix), full (audit's 4 + 2 angles + sim). A **conformance suite** ships in the package: conformance/*.json golden designs — the 17 walkers × 4 modules the linkage report already covers plus the 11 mechanisms — with expected layers, stack, cost, metrics ± tolerance and, for sixbar/trotbot, the expected failing stage and code. A spiderpig conformance command would run it (never built: the identity gate, `tests/gate/identity_gate.py`, and the scorecard, `tests/scorecard.py`, took its place); it is also the regression suite for planner changes.

### 3.5 Outputs and manifests

Files as today (`<name>.step/.stl`, `print/*.stl` + `parts.csv`, `laser/*_sheet_*.dxf` + csv, `bom.{csv,md,json}`), plus `.glb` (foot path + drive extras), MJCF `.xml`, report.md (explain + plan table + verify rows + BOM). New `manifest.json`:

```
Manifest {
  design: id, engine_version, t_ref,
  parts: [{name, side, group, fab: laser|printed|purchased, file, qty, mirrored, material,
           dims_mm, volume_mm3, mass_g, layers: [k...], bom_key?}],
  assembly: [{step, do, parts}],
  bom: Bom.as_dict(), cost_usd, printed_g, sheets,
  plan: {layers, top, height_mm, route}, metrics: {walk, sim?}, verify: {pass, failed: [...]}
}
```

`assembly` order is derivable from the plan and the crank/axle docstrings (outer plate → pillars and pins bottom-up → crank segments with riders threaded on → inner plate → servo → chassis); today it exists as prose in `construction/crank/bolt.py`.

### 3.6 MCP tools, resources, prompts

Tools (each with inputSchema/outputSchema; `readOnlyHint`/`idempotentHint` set on all but `export`; execution.taskSupport: optional on the starred):

| tool | one line |
|---|---|
| `list_linkages` | registered linkages, filtered by kind/family/feet/output motion |
| `describe` | one linkage's card: parameters, links, closures, foot path, which modules plan |
| `catalog` | servos, sheets, filaments, constructions, with prices and dims |
| `resolve` | validate a Spec, infer the rest, return a design id or precise SpecErrors |
| `check` | program, output, drive and static-clearance stages; no parts built |
| `plan` | the layer plan and crank route, optimality proof, ground clearance, or blockers + recommendations |
| `explain` | all stages in prose, for a design or a failure |
| `recommend` | verified changes that clear a failure or move a metric, as spec patches |
| `walk` | quasi-static walking metrics against the spec's targets |
| `tune*` | search phases/params for the objective; ranked candidates |
| `build*` | fabricate every part; manifest with masses |
| `verify*` | pass/fail per requirement with evidence tier |
| `simulate*` | MuJoCo run under crank-speed controls; metrics |
| `export*` | write the chosen formats; returns file list + manifest |
| `compare` | diff two designs' specs and reports |
| `search*` | pick linkage + module + params to meet the spec; ranked designs |
| `get_design` | any stored stage of a design |

Resources: `spiderpig://guide` (how to design with spiderpig: the passes, what each proves, the spec vocabulary, the two standard loops), `spiderpig://schema/spec`, `spiderpig://linkages/{key}` (the card), `spiderpig://catalog/{servos|sheets|constructions}`, `spiderpig://designs/{id}/{stage}`, `spiderpig://designs/{id}/report.md`. Prompts: `design_walker(goal)` (resolve → search → verify → export), `diagnose(design)` (explain + recommend), `iterate(design, metric)` (tune/recommend loop). HTTP mirrors it: `POST /api/{op}`, `GET /api/designs/{id}/{stage}`, `GET /api/jobs/{id}`; the FastAPI app already exists.

## 4. Gap analysis

| # | change | where | size | depends on / overlaps |
|---|---|---|---|---|
| 1 | **Stage results as data** (§3.2): Blocker objects from `_blocked_by` (built: `PlanError.blockers`); `ClearanceError` carries `[Clearance]`; `AssemblyError`/`OutputError` carry their check; `ConstructionError(group, what, value, need)`; `verify_plan`/`check_side`/`clashes`/`pack` return rows with `.describe()`; `Failure.from_exception()` at the boundary | `stack/`, `linkage/checks.py`, `construction/base.py`, `contract.py`, `servos/mount.py`, `layout.py`, `hardware/bom.py`, the audit script (now `spiderpig/tools/audit.py`) | M | planner rewrite (extend its `Recommendation`/`PlanError`); audit's error-consistency item |
| 2 | **Spec + resolver + validator**: nearest-value messages, conflict detection (stride × rpm vs speed; legs vs modules; second input vs drive), inferred defaults recorded | new `spiderpig/spec.py` (the resolver became `api.resolve`) | M | audit #2 (validating `BuildConfig` is the inner half; the four parsers go) |
| 3 | **Design store + serialisation**: a plan's JSON form (route as data; built in `api/planning.py`), design reconstruction via `problem.plan()`; replace `_DESIGNS`/`_LAYOUTS` and the server caches with the store | new `spiderpig/design.py`; `fabricate.py`, `server/app.py` | M | rewrite (`choices`, `optimal`, `proof`); audit #8 |
| 4 | **`api.py` façade + CLI subcommands + MCP server** (`mcp` SDK, FastMCP); a process pool for jobs — OCCT work can't share the event loop's thread, and today's `_BAKE_LOCK` serialises everything | new `spiderpig/api/`, `cli.py`, an MCP server (built as `spiderpig/mcp/`); `server/app.py` | M | audit's `cli.py` |
| 5 | **`verify` + conformance suite**: fold the audit script, `verify_plan`, `check_side`, walk/sim targets into one report with tiers; golden designs from the linkage report | new `spiderpig/verify.py`, a conformance/ folder (never built) | M | 1, 3 |
| 6 | **Manifest + assembly order + report.md** | `hardware/mass.py` (audit #3), `hardware/bom.py`, new `report.py` | S–M | audit #3 |
| 7 | **BOM grouping cost** (61 s): invariant-only `congruent` behind a flag | `hardware/bom.py` | S | audit's bom item; export drops from ~90 s to ~30 s |
| 8 | **`search`**: `linkage_report` + `tune_gait` as one op, candidates pruned by the golden "which modules plan", scored against Spec targets (`Target` → objective weights) | a new search module (never built); the tuner and the linkage report (now `spiderpig/tools/tune.py`, `spiderpig/tools/report.py`) become callers | M | 2, 3 |
| 9 | **Spec-level `recommend`**: scale (`unit`/`OA`) for stride/lift/envelope, module for leg count, servo by rpm/torque/price, sheet by thickness; each verified by re-running check/plan | `recommend.py` | M | rewrite's pattern; 2 |
| 10 | **Custom modules from the spec** (`legs.explicit`, 3 legs per side): `Module(legs, cranks)` as data; the old stacked_plan generalised | `linkage/assembly.py`, `fabricate.py` | M | audit's `build_module_template` fix |
| 11 | **Determinism/caching**: engine version + `StackSpec` budgets in the key; golden `plan.json` hashes in conformance; no wall clock in results | `design.py`, `stack/` | S | – |
| 12 | **Packaging**: `package = true`, `spiderpig/` layout (audit's map), console script, `importlib.resources` for linkages/catalog/viewer dist; extras `[viewer]`, `[sim]` (mujoco), `[mcp]`; servo CAD sha-pinned download on demand into `SPIDERPIG_CAD_CACHE` (as built), `SPIDERPIG_OFFLINE=1` uses the box model and marks the design `estimated` | `pyproject.toml`, `servos/cad.py`, every `sys.path.insert` | M | audit's module map |
| 13 | **Spec versioning**: `version` + migration functions; linkage cards carry `since`; `resolve` refuses unknown versions with the hint | `spec.py` | S | 2 |
| 14 | **Async jobs**: process pool + job table in the store; MCP Tasks and HTTP `/jobs`; wall-clock caps for fabricate/BOM/sim (plan is already budget-bounded) | `api/`, `mcp/`, `server/app.py` | M | 4 |
| 15 | **Viewer by design id**: `spiderpig view <id>`; the bake reads stored parts instead of re-fabricating | `spiderpig/bake.py` (then the viewer's bake script), `server/app.py`, `controls.ts` | S–M | 3; audit #4 |
| 16 | **`compose(stages)` + a drive per input**: the engine work CLAUDE.md lists under "Stacking (future)" (fixed pivots as `offset(J1, J2, ...)` on the parent's output body, prefixed points, per-stage inputs, cross-stage clearances) and a second drive (today `ConstructionError` in `servos/mount.py`) | `linkage/`, `stack/`, `servos/mount.py`, `construction/` | L | out of v1; the spec field is reserved |

Order by value: 2 → 1 → 3 → 4 → 5 → 6 → 7 → 11 → 12 → 8 → 9 → 10 → 14 → 13 → 15 → 16. Items 1–7 + 11–12 are v1 ("compiler + REPL"); 8–10 make it a design tool; 14–15 are polish; 16 is the next product.

## 5. Worked example

Goal: "a 6-legged walker for 20 mm steps, ~150 mm/s, under $120, acrylic + PLA, one STS3215 per side." Numbers below are the repo's own (linkage report, `/api/walk`, BOM).

**1. `resolve`**

```json
{"version":"1","kind":"walker","legs":{"per_side":3,"sides":2},
 "motion":{"stride_mm":{"value":20,"hard":true},"speed_mm_s":{"min":150,"hard":true}},
 "materials":{"sheet":"acrylic_3mm","filament":"pla_filament","servo":"sts3215"},
 "budget":{"cost_usd":{"max":120,"hard":true}},"outputs":["step","print","dxf","bom","glb","report"]}
```
→ `{"design": null, "errors": [`
`{"path":"legs.per_side","code":"no_such_module","message":"no registered module has 3 legs per side","allowed":{"single":1,"double":2,"decker":2,"quad":4},"alternatives":[{"legs.per_side":4},{"legs.explicit":"[{orientation,phase_deg}]×3 (experimental)"}]},`
`{"path":"motion","code":"targets_conflict","message":"stride 20 mm at sts3215's 52 rpm is 17.3 mm/s, not ≥150; 150 mm/s at 52 rpm needs ≥173 mm/rev","numbers":{"rpm":52,"stride_needed_mm":173.1,"speed_at_stride_mm_s":17.3}}]}`

The agent realises "20 mm steps" meant step height, not stride, and takes the tested 4-per-side module:

```json
{"legs":{"per_side":4,"sides":2},"motion":{"lift_mm":{"min":20,"hard":true},"speed_mm_s":{"min":150,"hard":true}}, ...}
```
→ `{"design":"d_5c1e…","warnings":["linkage: any → search will choose"],"resolved":{"module":"quad","robot":true,"sheet":"acrylic_3mm","servo":"sts3215","pillar":"printed","pin":"printed","crank":"printed"}}`

**2. `search(spec)`** (task; ~3 min for 17 walkers × quad, plans and walk only) →

| rank | design | linkage | stride | speed mm/s | lift | layers / stack | cost | meets |
|---|---|---|---|---|---|---|---|---|
| 1 | `d_9a…` | strider quad | 177 | 154 | 33 | 28 / 84 mm | $89.7 | all |
| 2 | `d_4f…` | jansen quad | 150 | 130 | 36 | 16 / 48 mm | $89.7 | speed ✗ |
| 3 | `d_e2…` | klann_long_legs quad | 124 | 107 | 45 | 13 / 39 mm | $89.7 | speed ✗ |

**3. `check("d_9a…")`** → `{"ok":true,"steps":[...margins, transmission angles...],"clearances":[...],"ground_clearance_mm":21.4}`.

**4. The agent wants it smaller** — `size.envelope_mm: [200,150,90]` and patches `linkage.params.unit: 6.5 → 5.0` → `d_77…` (`derived_from: d_9a…`). `plan("d_77…")` →

```json
{"ok":false,"failure":{"stage":"plan","code":"no_plan_in_budget",
 "message":"strider_quad: no layer plan found with up to 31 layers after 60000 search steps",
 "blockers":[{"a":{"group":"pillar:A_leg0","layer":9},"b":{"group":"b2_leg1","layer":9},"count":1412,"gap_mm":7.9,"need_mm":9.0}],
 "recommendations":[
  {"change":"unit","before":5.0,"after":5.9,"why":"b2_leg1 clears pillar:A_leg0 by the 9.0 mm its neck needs","effects":"scale ×1.18: stride 161 mm, crank radius 23.6 mm","verified":"plans in 26 layers (78 mm)","patch":{"linkage":{"params":{"unit":5.9}}}},
  {"change":"link_radius","before":6.0,"after":5.0,"why":"thinner links clear at unit 5.0","effects":"narrower laser links","verified":"plans in 28 layers","patch":{"fit":{"link_radius":5.0}}}]}}
```

**5. `recommend` → apply the first patch** → `d_c0…`; `plan` ok (26 layers, 78 mm ≤ 90 ✓, `optimal: true`). `walk("d_c0…")` → `speed_mm_s: 140` ✗ (hard). `recommend` → `{"change":"unit","after":6.3,"why":"speed ≥150 at 52 rpm needs stride ≥173 mm","verified":"plans in 27 layers (81 mm)"}` and `{"change":"servo","after":"xl330_m288","why":"103 rpm","effects":"torque 5.3 kg·cm, marginal (catalog note)"}`. The agent takes unit 6.3 → `d_b8…`: 81 mm stack ✓, 151 mm/s ✓.

**6. `build("d_b8…")`** (task, ~10 s) → manifest: 287 bodies (71 laser, 150 printed, servos and screws purchased), mass 549 g, `build/parts/*.step`.

**7. `verify("d_b8…", "standard")`** (task, ~40 s after the BOM fix) →

| requirement | value | target | pass | tier |
|---|---|---|---|---|
| loops close (all legs) | min margin 4.1 mm | ≥ 0 | ✓ | proven |
| plan re-verified (2880 samples) | 0 violations | 0 | ✓ | proven |
| contract at t = 0, 1.6, 3.2, 4.8 | 0 outside claims | 0 | ✓ | proven |
| clashes at t = 1 | 0 | 0 | ✓ | measured |
| lift_mm | 33 | ≥ 20 | ✓ | measured |
| speed_mm_s | 151 | ≥ 150 | ✓ | measured (nominal mass) |
| envelope z (stack) | 81 | ≤ 90 | ✓ | proven |
| cost_usd | 89.7 | ≤ 120 | ✓ | estimated (3 unverified links) |
| DXF packs | 2 sheets of 300 × 300 | – | ✓ | measured |

**8. `export("d_b8…", ["step","print","dxf","bom","glb","report"], "out/hexapod")`** → `out/hexapod/{strider.step, print/*.stl + parts.csv, laser/strider_sheet_1..2.dxf, bom.{csv,md,json}, strider.glb, report.md, manifest.json}`. Optionally `tune(free=["phases"])` (Strider quad already scores 1.1) and `simulate(seconds=5)` for torque against the 19.5 kg·cm stall.

## 6. Risks and open questions (for the user to decide)

1. **Spec breadth vs precision.** A broad spec (`linkage: any`, `servo: any`) makes `search` the default path (minutes); a narrow one makes it a compiler (seconds). Proposal: v1's Spec has only fields the engine can verify today; unknown fields are rejected with a message, never ignored.
2. **Hard constraints vs objectives.** The example needs both. Default `hard: true` for size, budget, ground clearance and `hard: false` for gait quality (bob, slip)? And does `verify` fail on a soft miss or only report it?
3. **Metric semantics.** Stride/lift/speed exist in three definitions; the Spec must pin one per field (proposal in §3.1). Speed depends on a servo's no-load rpm — flag it as `estimated` until sim confirms it under load.
4. **How much build123d to expose.** Proposal: none in the tool layer (parts as files + measured numbers); an optional `inspect_part(design, part) → measure/section` escape hatch like build123d-mcp; new constructions stay Python plugins through the `Group` registry — "agents writing constructions" is a separate product.
5. **Sync vs async.** Python API sync; MCP with Tasks; a process pool because OCCT and the sympy caches are per process, so cache warmth moves from memory to the store (a linkage's compile is 0.2 s; fine).
6. **Store location and lifetime.** Per project (`./.spiderpig`) or per user (`~/.spiderpig`)? Retention of parts (tens of MB per robot)? Multi-user access to ids?
7. **Determinism across OCCT/build123d versions.** STEP bytes differ between versions; conformance compares metrics, not bytes. Is that acceptable as "the same design"?
8. **Optimality and budgets.** "No plan" is "no plan within budget"; expose the budget as a spec knob and always return `proof`. How long may `plan` run before it is a task (25 s today for Strider quad)?
9. **Custom modules.** `legs.explicit` needs a rule for which cranks are shared (today keyed on module names); Strider's own modules show a linkage may redefine the unit. Ship 1/2/4 per side first?
10. **Verified recommendations cost real time** (each re-run is a plan, 0.1–25 s). Cap per failure (the rewrite's choice) and let `recommend(design, deeper=true)` do more?
11. **Mechanism specs.** `verify` for a mechanism compares `OutputCheck` to targets and skips walking; a compound machine (stages, five-bar's second input) waits for #16 — say so in the guide resource rather than half-support it.
12. **Servo CAD downloads at install/build time** (licences, offline builds) — the `SPIDERPIG_OFFLINE` fallback marks such designs `estimated`.
13. **Is the viewer part of the library?** It needs a Node build; shipping `viewer/dist` as package data vs a separate `spiderpig-viewer` package.

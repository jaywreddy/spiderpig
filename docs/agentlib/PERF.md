# Where the time went, and what was stupid — 2026-10-01

A follow-up to [TIMING.md](TIMING.md): every slow step of the known path profiled, what it
was doing, what was stupid about it, the fix where the outputs stay identical, and the
saving measured after it. Two halves, done apart: the fabrication side below (`export`,
`verify`, `build`, the bake, the stage cache), the planning side under its own heading.

## Fabrication side

**Designs.** The round-4 Strider `double` of TESTDRIVE.md (`shin 16`, `unit 6.3`, XL330,
plywood: 20 layers, 161 parts) and the default Klann quad (173 parts).

**Method.** Every number is a fresh process (`import spiderpig.api`, `resolve`, `plan`
from the store, then the one step timed), the base commit `38158e6` and this branch back
to back on the same 4-core VM, with the plan already in the store; an export reloads the
build from its STEP files first (as the MCP worker does: 7 s for round 4, 5.5 s for
Klann, timed apart and not in the export's number). Another agent's runs shared the
machine (load 3-6), so wall times carry a few seconds of noise; CPU seconds are given
beside them. The profiles are a wall-clock stack sampler on the main thread (2 ms).

**The identity gate.** The seven formats exported by the base and by this branch (the
four changes below), both designs, a fresh build in one process: `bom.json` and `bom.csv` (text),
`print/parts.csv` and the laser `parts.csv`, every DXF's `ENTITIES`, the glb's BIN chunk
sha256 and its JSON chunk, the MJCF and its metadata, the whole-robot STL bytes and the
STEP's per-solid volumes. **All identical, both designs**, with the plan re-made from the
store on this branch (the base's bake solved it in-process). Also on the reload path
(the build from STEP, the MCP's): every format identical except the round-4 grouping,
where the base was wrong (section 1). The print STLs are the one thing that moved, and
section 1 shows why that is the fix, not a regression.

### Before / after

Fresh process, the plan in the store, seconds wall (CPU).

| step | round 4 before | round 4 after | Klann before | Klann after | what changed |
|---|---|---|---|---|---|
| `build`, a fresh fabrication | 23.7 (24.2) | **17.1** (18.9) | 14.8 (18.7) | 13.2 (19.3) | the servo record (3) |
| `build` reloaded from STEP | 7.9 | 7.0 | 5.6 | 5.5 | – (noise) |
| `verify standard`, cold | 64.2 (81.3) | **50.7** (74.0) | 53.3 (63.3) | 47.6 (72.3) | the servo record; the rest is what the checks cost (4) |
| `verify standard` after a `quick` | 48.7 (74.3) | **0.0** | 40.9 (52.8) | **0.0** | one report per level (4) |
| `export` print | 116.6 (160.5) | **26.4** (40.5) | 78.8 (137.7) | **27.2** (49.2) | the grouping (1) |
| `export` bom | 117.1 (158.4) | **25.7** (38.9) | 80.4 (140.2) | **25.8** (47.0) | the grouping (1) |
| `export` glb | 59.0 (71.7) | **24.4** (34.0) | 19.5 (30.3) | 18.8 (30.2) | no plan, no second fabrication (2) |
| `export` mjcf | 49.6 (58.0) | **17.4** (25.4) | 13.1 (19.9) | 13.0 (20.0) | the same (2) |
| `export` step / stl / dxf | 3.9 / 14.2 / 2.1 | 3.9 / 11.4 / 1.5 | 3.3 / 4.9 / 1.6 | 2.9 / 5.2 / 1.5 | – (noise) |
| **`export`, the seven formats in one process** | **185.2** (313.2) | **75.2** (117.5) | **121.8** (205.9) | **60.1** (105.4) | all of it |
| `spiderpig bake` (no design: plans from its options) | 60.3 (64.8) | 50.3 (64.1) | 19.4 (29.7) | 24.6 (28.3) | the servo record (5) |

The MCP's `export` of the seven formats, TIMING.md's 184.5 s, is the 185.2 s row plus the
reload: **192 → 82 s** for round 4, **128 → 66 s** for Klann. Klann's glb and mjcf barely
move: its plan takes 1.4 s and its servo model (STS3215, one solid) strips in 0.1 s, so
what each did twice was cheap; they share one fabrication when asked together.

### 1. `export` print and bom: the grouping — 117 → 26 s each

**What it did.** Both formats group the made parts by shape (`hardware/bom.py`
`group_made`: one row and one STL per different part). Every body is compared with every
group's reference by `congruent(a, b)`: volume, area, principal moments, then a proof that
`b` is `a` moved (or mirrored).

**Profile top (base, print, 100.6 s).** `congruent` 99.9 s = `_proper_fit` 88.2 s, of
which `Shape.__sub__` (OCCT cuts) 84.5 s; `volume` 10.3 s, `area` 1.9 s, `_frame` 2.0 s.

**The stupid things.**

1. Every call re-measured both parts: a `Part`'s `volume` and `area` are fresh `GProp`
   integrations, so the made parts' invariants were integrated again for every pair
   (14 s).
2. The proof tried up to eight matchings of the two principal frames in order and proved
   each with **two** cuts (`moved − b`, `b − moved`), before looking at anything cheap:
   a wrong sign flip costs two booleans of near-coincident solids, the slowest case for
   OCCT. For round 4's 125 made parts that is 750 comparisons and 368 cuts at 0.25 s
   (390 on the STEP-reloaded parts). The mirror image of the reference was rebuilt and
   re-measured for every candidate.
3. The cuts ran on the design's live parts, and OCCT widens its arguments' tolerances in
   place. So what a part was later meshed as depended on how often it had been compared:
   the base's print STLs of the parts its cuts touched (3 of 14 for round 4, 4 of 17 for
   Klann) are not the STLs of the parts as built, and a second `group_made` in the same
   process changes them again. On parts reloaded from STEP (the MCP's path) the cuts came
   out wrong: the base listed the round-4 pillar segment `pillar_J2_leg0_seg1` as three
   rows (2 + 1 + 1) where a fresh build has one row of 4: the right-side twins' proof
   came back with ~1 270 mm³ of difference for the right matching (the frames matched
   to 1e-8 mm).

**The fix** (`bom: measure each part once, ...`). Each part is measured once (`_Sig`:
volume, area, principal frame, surface centroid); a group's reference is mirrored once,
lazily; a frame matching is tried only when it carries the surface centroid onto the
other part's within 1e-3 mm (a wrong matching misses by 0.72 mm on the pillar, a right one
by 1e-8 after a STEP round trip); the proof is one `BRepAlgoAPI_Common` with
`SetNonDestructive`: `va + vb − 2·shared < tol`, the same tolerance, "same" before
"mirror" as before. The same 750 comparisons now run 103 booleans, and the parts are
left as they were (`test_bom`: their tolerances after two groupings equal those before;
it fails on the base).

**Measured.** print 116.6 → 26.4 s, bom 117.1 → 25.7 s (round 4); 78.8 → 27.2 and
80.4 → 25.8 (Klann).

**What remains** (after, print, 26.2 s): the proofs, 21.7 s (103 booleans, one per
match, ~0.2 s each); the invariants once per part, 3.1 s; the STLs, 0.8 s.

**Outputs.** Fresh build: the groups, their order, references and mirror counts, and
every BOM and CSV byte identical on both designs. The print STLs: exporting the groups
taken from `parts.csv` with no comparison at all gives the same STL bytes on the base and
on this branch for every file, and this branch's grouped export equals them for every
file; the base's grouped export differs exactly for the parts its cuts touched. Reload
path: this branch's grouping equals the fresh build's (22 rows); the base's did not
(24).

**Risk.** Low. The proof is still a boolean, on the same tolerance. The centroid filter
only skips a matching whose proof would fail: a motion that maps one part onto the other
within `tol` maps the surface centroid along with it; it could only differ for two parts
that differ by a feature smaller than `tol` (1e-4 of the volume) and still shift the
surface centroid by more than 1e-3 mm, which no construction here makes.

### 2. `export` glb and mjcf: a plan and a fabrication per format — 59 → 24 s, 50 → 17 s

**What it did.** `_export_files` called `bake_gltf(cfg)` and `build_mjcf(cfg)`. Each went
back to the *config*: `fabricate(template_for(cfg), cfg, 0)` → `design_side`, which in a
fresh process (the MCP worker, the CLI) solved the plan again, then fabricated the robot,
once per format.

**Profile top (base, glb, 56.9 s).** `_reference` 43.9 s = `design_side` (the plan
solve) 29.2 s + `fabricate_side` 13.0 s (the servo strip 4.6 of it); tessellation 9.5 s.
The mjcf: `fabricated` 45.1 s (the plan 29.9) of 52.2 s.

**The stupid thing.** The design already held its plan (re-made from the store in 1 s);
the two formats asked the planner for it again (27 s for round 4) and fabricated the same
robot at the same angle twice.

**The fix** (`export: fabricate the walker at the bake's angle once, ...`). The export
fabricates the walker at `T_REF` once from the design's own side (`fabricate_at`) and
hands it to both: `bake_gltf(fabricated=, side=)` (the side's layer plan for the drive
extras) and `mjcf.set_fabricated`. `mesh.tessellate` now cleans a shape's earlier
triangulation first: on the shared parts the hulls' 0.5 mm mesh would otherwise reuse
the bake's finer 0.1 mm one (the mesh is the parameters' alone, whoever meshed the shape
before). `bake_gltf(cfg)` with no walker given (`spiderpig bake`, the dev server) is
unchanged.

**Measured.** glb 59.0 → 24.4 s, mjcf 49.6 → 17.4 s (round 4, each ~4 s of it the servo
record of section 3). Klann: 19.5 → 18.8, 13.1 → 13.0 (its plan is 1.4 s); both formats
in one export share the fabrication. What remains of the glb (after, 25.1 s): the
fabrication at t = 0, 11.9 s; the bake 13.1 s (tessellation 9.7, the mesh sharing's mass
properties 2.2).

**Outputs.** The glb's BIN and JSON chunks, the MJCF and its metadata: identical to the
base's on both designs and both paths, now from the stored plan.

**Risk.** Low: the same fabrication from the same plan, re-made and `verify_plan`ed. A
design whose stored plan differs from what a fresh solve in a new process would find (a
solve cut by its deadline) now bakes its own plan, where the base baked whatever the
second solve found.

### 3. `build` / `fabricate`: the servo model's bounding boxes, every process — 23.7 → 17.1 s

**What it did** (round 4, 22.0 s in the base's profile). `fabricate_side` 14.6 s: OCCT
booleans 9.1 s (the links' `plate()` 3.2 s, the printed axles 3.1 s, the crank 1.9 s,
the chassis 1.1 s) and the drive group 6.3 s, **5.4 s of it the servo's manufacturer
model**: `cad_servo` → `strip_horn` → `_matches` takes an optimal bounding box of every
one of the XL330 model's 15 solids (4.65 s; build123d's `bounding_box()` is `Clean` +
`AddOptimal`) to find which solid is the horn, after a 0.7 s import. `attach_build`
5.3 s (`part_props` × 161 2.2 s for the masses; the STEP files 1.4 s);
`assemble_robot` 1.7 s (the right side is the left mirrored: one transform per part,
nothing rebuilt).

**The stupid thing.** Which solid of a hash-pinned file is the horn was decided again,
by the most expensive box there is, in every process that fabricates: the build, the
verify (once for its three fabrications), each export's bake and MJCF, the CLI.

**The fix** (`servos: record which solids the horn strip keeps, ...`). The kept indices
are recorded as JSON beside the download, keyed by the model's sha256, its transform,
scale, the strip boxes, `BBOX_TOL` and a format version; a record for another solid count,
or unreadable, is ignored and rewritten. The shape is built from the same import and the
same solids (the import carries no triangulation, so the boxes' `Clean` changed nothing),
and the strip-count warning still fires. A binary BREP of the stripped shape was tried
first and dropped: its meshes are not the same (the Klann quad's whole-robot STL
changed).

**Measured.** `cad_servo` for the XL330 in a fresh process: strip 4.17 → 0.01 s (the
0.7 s import stays); the STS3215's one-solid model 0.13 → 0.10 s. The round-4 build
23.7 → 17.1 s.

**What else is in a fabrication, and why it stays.** One boolean per feature: a link is
two pills (two fuses each), one union and one cut of all its holes; a crank segment a
union of discs and posts minus a union of bores and pockets; an axle segment a revolve
and one cut with all its slot cutters. No fillets, no repeated `Part` operations. The
identical links of the two legs are built twice (not fixed: see Not done).

### 4. `verify standard`: 64 s cold (now 51), 49 s "warm" (now 0)

**What it did** (round 4, cold, the base's profile at 53.8 s wall / 86.5 CPU-s):
`api.build` 19.5 s (fabrication 14.3, `attach_build` 5.2), `check_side` at t = 0 and
t = 3.2: 22.9 s (every group realized again at each angle, the claims' envelope solids
`claimed_solid` 5.4 s, `part − envelope` per part 4.4 s), `clashes` 7.3 s (pairwise `&`
after a bounding-box prefilter), `bad_solids` 1.8 s (`is_valid` × 161), the DXF pack and
the ungrouped BOM ≈ 1.5 s, `verify_plan` on fresh samples ≈ 0.5 s.

**Redundant?** Little. The contract's two angles are not the build's, and its side is
not the robot's (no frame ties), so neither the build nor the export's t = 0 fabrication
can stand in for it; the clash check runs on the build's own parts. The redundancy was
inside the fabrications: the servo strip of section 3, once per process (4.2 s). Cold:
64.2 → 50.7 s. After (a run at 56.6 s): `check_side` 27.2 s, `build` 16.6 s, `clashes`
8.5 s, `bad_solids` 2.0 s; OCCT booleans are 42 s of it.

**The 45-49 s warm miss** (`store: keep one verify report per level ...`). `verify.json`
held one level and `_cached` served it for that level only: a `quick` after a `standard`
overwrote it, and the next `standard` ran again (TIMING.md's warm pass: 45 s). The store
now also writes `verify.<level>.json` and reads the level's copy first (then
`verify.json`, so a store written before this still serves its own level); `verify.json`
stays the latest, for the cards and `stages()`. Validity is unchanged: the running
engine version. Measured: 48.7 → 0.0 s (round 4), 40.9 → 0.0 s (Klann).

### 5. The bake (`spiderpig/bake.py`)

**What it did** (`spiderpig bake`, a fresh process, round 4: 58.8 s in the base's
profile). `1_reference_build` 41 s = the plan solved (27 s: the CLI has no design, only
its options) + the fabrication 14 s; `2_tessellate_total` 8 s (printed parts 3.3, the two
servos 3.2: the right servo is a mirror, not a translate, so it is meshed on its own);
`2_mesh_share` 2 s; nodes and channels 1 s. Inside an export it was the same, the plan
included (section 2).

**Fixed.** Inside an export: no plan, no second fabrication (section 2). Everywhere: the
servo record. `spiderpig bake` alone still plans from its options (that solve is the
planning side's): 60.3 → 50.3 s round 4. The profiler's stage keys are unchanged; the
export's bake still runs with `profile=False`.

### 6. The rest: `stl`, `step`, `dxf`, the STEP reload

Nothing stupid found. `stl` is one `BRepMesh` of the whole robot at 1e-3 mm on three
cores (11-14 s round 4, 5 s Klann); `step` the XDE writer (3-4 s); `dxf` sections, kerf
offsets and the packing (1.5-2 s). The reload (7 s for round 4's 161 parts from 90
files): `import_step` 3.0 s (an XCAF reader per file), `part_props` 2.3 s,
`placed_part`'s deep copies 1.4 s; see Not done.

### Not done (would change outputs, or needs a decision), with its number

| what | saving | why not |
|---|---|---|
| bake at the build's crank angle (t = 1) and share `design.mech` | the one fabrication left in a glb/mjcf export: 11.9 s round 4 | the glb's frame 0 and the MJCF's `t_ref` (qpos0) change: a design decision |
| reuse `design.mech` when the build is at t = 0 | the same, for such builds | an edited (rechecked) part would then show in the glb and the MJCF, where the base fabricated afresh: a semantics decision |
| keep the grouping with the build (the store) | 25 s per later `print` or `bom` export, and for `spiderpig build` | print and bom asked in two exports group twice; a store-layout decision |
| keep the baked glb per design across export folders | 24 s round 4, 19 s Klann per export of `glb` into a new folder | a store-layout decision, and the bake's code is not in the engine version (a cached glb could outlive a bake change) |
| a binary BREP beside each STEP in the store | most of the reload's 3 s of `import_step` | a reloaded build's parts would become the fresh build's (its STL and DXF change, towards the fresh ones); the MCP hands out the STEP paths |
| build each identical link once and move it to the other legs | up to half of `plates.realize`'s 3.2 s per fabrication (× 3 in a verify) | a moved copy is a new B-rep; needs the gate on every output, not attempted |
| one 2D profile extruded per plate instead of pill fuses + a cut | part of the 9 s of booleans per fabrication | different faces and edge order: DXF polylines and glb vertex order change |
| skip build123d's deep copy in `placed_part()` for an identity pose | 1.4 s per reload, 2.2 s per verify | `to_compound` sets the label and colour on the copy; sharing the object would carry them onto the stored parts |
| the clash check in parallel | most of its 7-8.5 s per verify on 4 cores (an estimate) | needs processes (pickling 161 solids); not attempted |
| plain (not optimal) bounding boxes in `attach_build` and the contract's prefilter | ≈ 2.3 s per verify | the build report's `envelope_mm` (and the manifest's) would change |

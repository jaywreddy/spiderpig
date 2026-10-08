# The exporters: libraries, exporters, parallelism — measured, 2026-10-01

> **History** (2026-10-01; historical: the export study. Its parallel exports and grouping work have since landed (W3); `mise run scorecard` has the current numbers.)

A follow-up to [PERF.md](PERF.md), whose fabrication-side numbers are this study's base:
"Go look at the exporters. Are we using open source libraries effectively? Are there
efficient exporters that can be used? Or dumb parallelism?" Every exporter and heavy step
profiled; what each calls in OCCT / build123d; what those libraries offer that we don't
use; what can run beside what; and a prototype for each opportunity, with its measured
before/after and an identity check. The prototypes are commits on this branch, for
review; nothing is merged.

## The answer, short

- **The libraries are mostly used right.** OCCT does the heavy lifting (booleans,
  meshing, STEP, STL, mass properties) and its own parallel flags are already on:
  build123d sets `SetRunParallel(True)` on every boolean and we mesh with
  `isInParallel=True` (8.3 s wall / 15.8 CPU-s against 15.9 / 15.6 serial for the
  robot's 161 parts, bit-identical). Four things were wasteful; all are fixed here,
  output-identical:
  1. build123d's **`export_stl` meshes twice**: it constructs `BRepMesh_IncrementalMesh`
     (which meshes) and then calls `Perform()` (the whole mesher again over the finished
     mesh): a quarter of every STL.
  2. build123d's **`Shape.moved` deep-copies the B-rep and throws the copy away** (it
     replaces the copy's `wrapped` with `wrapped.Moved(loc)`, which shares the original's).
  3. **Our mesh extraction read every node one pybind call at a time** (five per node),
     where **OCCT's own glTF writer, `RWGltf_CafWriter`, does it in C++**: the same arrays,
     bit for bit, 0.43 s against 3.3 s for the 161 parts of a fabrication.
  4. **The same integrals, again and again**: the grouping integrated each part's volume
     twice and its surface twice, and proved with a boolean that each right-side part is
     its left twin's mirror image, which the assembly made it (45 of 103 booleans); the
     MJCF re-measured every part and its optimal bounding box right after the bake had.
- **Efficient exporters.** STEP (`STEPCAFControl_Writer`), STL (`StlAPI_Writer`) and DXF
  (ezdxf: 0.05 s of a 1.6 s DXF) are already the libraries' own. The one we didn't use is
  `RWGltf_CafWriter`; as the *writer* of our glb it can't produce our file (the animation,
  the drive extras, meshes shared by congruence rather than by shape), so it is used as
  the mesh *reader*. **No new dependency is worth it** (section 9).
- **Dumb parallelism.** OCP holds the GIL through every OCCT call (four threads run
  booleans, meshing or mass properties no faster than one), a child **forked** after OCCT's
  thread pool has run **deadlocks**, and `multiprocessing`'s spawn re-imports the caller's
  `__main__` (a script without the guard breaks): all measured. So the parallelism here is
  processes started as `python -c` (`spiderpig/workers.py`): the export's glb + MJCF (they
  need the plan, not the build) start before the export's build and run beside STEP, STL,
  the prints, the DXF and the BOM; the grouping runs beside STEP and STL; `verify`'s two
  contract angles run beside its build and clash check.

**Before / after** (round-4 Strider and Klann quad, a fresh process, the plan in the
store; seconds wall (CPU-s, workers included) at the load average shown; "work removed
only" is the final code with `SPIDERPIG_WORKERS=0`):

| step | base | work removed only | final | |
|---|---|---|---|---|
| **`export`, 7 formats, round 4**, the build reloaded inside (the MCP job's path) | 89.8 (164) | 61.3 (116) | **36.8** (116) | load 1.4-2.2 |
| the same, the build fresh | 88.6 (166) | 64.5 (126) | **45.6** (133) | 1.9-3.3 |
| **`export`, 7 formats, Klann**, reloaded | 72.6 (131) | 53.5 (97) | **34.2** (99) | 2.0-3.2 |
| the same, fresh | 75.1 (135) | 55.4 (102) | **41.0** (111) | 1.8-2.6 |
| **`verify standard`, round 4**, reloaded build | 43.7 (69) | | **32.0** (76) | 1.2-1.5 |
| the same, fresh build | 57.1 (92) | | **36.6** (94) | 1.6-2.1 |
| `verify standard`, Klann, reloaded / fresh | 45.8 (73) / 53.3 (89) | | **33.8** (80) / **37.2** (96) | 2.0-2.6 |
| `export` glb alone, round 4 / Klann | 23.2 (41) / 19.3 (33) | | 18.9 (37) / 16.8 (30) | 1.4-5.1 |
| `export` mjcf alone, round 4 / Klann | 16.8 (30) / 14.5 (24) | | 14.8 (27) / 13.5 (23) | 1.2-2.5 |
| `export` glb + mjcf, round 4 / Klann | 29.7 (53) / 23.4 (39) | | 21.3 (44) / 17.9 (33) | 1.2-1.9 |
| `export` print alone, round 4 / Klann | 33.3 (36) / 34.3 (38) | | 20.5 (22) / 19.5 (27) | 5.8-8.2 |
| `export` bom alone, round 4 / Klann | 32.8 (37) / 30.2 (45) | | 21.5 (23) / 21.7 (24) | 5.6-7.4 |
| `export` stl alone, round 4 / Klann | 15.5 (25) / 9.0 (14) | | 11.0 (18) / 8.1 (11) | 5.4-6.8 |
| `export` step alone, round 4 / Klann | 4.2 / 3.0 | | 3.8 / 2.9 | 6-7.6 |
| `export` dxf alone | 1.7 / 2.1 | | 1.7 / 1.6 | 6 |
| `build`, fresh, round 4 / Klann | 17.2 (25) / 14.6 (23) | | 16.6 (25) / 14.6 (23) | 1.6-2.3 |
| `spiderpig bake` by options, round 4, the plan in the store | 49.4 | | **26.7** | |

The export and verify rows at the top are the final commit at low load; the print, bom,
stl, step and dxf rows ran on a loaded machine, the base against an earlier commit of this
branch with the same code for those formats. PERF.md's 75 s for the seven formats excluded
the build's reload (7.5 s), which this table's top rows include. Free cores
matter to the final column: at load 7-11 the round-4 export (the build reloaded first)
took 106.5 s (base), 92.3 s (work removed only) and 91.4 s (final); at load 4-6.5, the
MCP path, 104.6 → 46.1 s.

## How it was measured

*Base* is `5f426b1` (`claude/great-carson-6nw2bz`), run from a `git archive` with its own
`PYTHONPATH`; each prototype ran from a copy of this branch's package at its commit, never
the live worktree. Designs: the round-4 Strider `double` (`shin 16`, `unit 6.3`, XL330,
plywood: 20 layers, 161 parts, 125 made) and the default Klann quad (173 parts, 129 made),
their plans in the store. Every number is a fresh process: the import, `resolve`, the plan
re-made from the store, then the step timed; the build either reloaded from the store's
STEP files (the MCP job's path) or fresh. Wall and CPU seconds, this process's and its
workers'. **The machine (4 cores) was shared with the planner study's agent the whole
time**, running 1-4 processes of its own (load average 1.2-11): every row says the load it
ran at, base and final ran alternately, CPU seconds are the steadier number, and parallel
prototypes gain less wall on a saturated machine. Profiles: a wall-clock stack sampler on
the main thread (2 ms; pybind confuses cProfile). Scripts and logs: the session
scratchpad's `pe/` (`t_*.py`, `gate.sh`, `vgate.sh`, `campaign.sh`, `camp/c1-c5.log`,
`gate_s*.log`, `micro.log`).

**The identity gate** (`pe/gate.sh`, `pe/compare.py`), run for every prototype: all seven
formats exported by the base and by the prototype, both designs, both paths (the build
reloaded from the store's STEP files, and fresh; for the last three commits also with the
export building itself): `bom.json` and `bom.csv` text, the print `parts.csv`, the print
STLs' bytes, every DXF's `ENTITIES`, the laser `parts.csv`, the glb's BIN chunk sha256 and
JSON chunk, the MJCF and its metadata text, the whole-robot STL bytes, the STEP's
per-solid volumes. **Every prototype: identical, both designs, both paths.** The STEP's
text is not in the gate: it orders its colour styles differently from run to run on the
base itself (four base runs, four sha256s); its colours, product names and style count
were compared and are equal. `verify standard` (`pe/vgate.sh`): the report's rows and
failures (timings aside), base against final, both designs, both paths: identical. Tests:
`tests/test_bom.py tests/test_bake_gltf.py tests/test_export.py tests/test_sim.py
tests/test_spiderpig_api.py tests/test_contract.py` pass (168), `ruff check .` clean.

## The commits

| commit | what | outputs |
|---|---|---|
| `1ba5de6` | `spiderpig bake` by options through the store's plan (`api.plan_config`) | glb identical |
| `bf65abe` | grouping: right-side twins without a boolean; one volume and one surface integration per part | identical |
| `8fd15aa` | `shapes.moved` (build123d's without the discarded copy); `mesh.export_stl` (meshes once) | identical |
| `2a53a55` | export: glb + MJCF in a worker; the MJCF reuses the bake's mass properties | identical |
| `b2c63b7` | verify: the contract angles in workers; `spiderpig/workers.py` (`python -c`, not `multiprocessing`) | identical |
| `e69471c` | mesh: the triangles read by `RWGltf_CafWriter` | identical; fell back silently for located parts, fixed by `b701457` |
| `1388086` | export: the grouping in a worker beside STEP/STL; a worker only when there is work to overlap | identical |
| `7b79581` | export: the glb/MJCF worker starts before the export's build; debug timings per step and per worker | identical |
| `b701457` | mesh: each part in a compound of its own for the writer (located parts) + a test against the loop | identical |

## 1. What decides what can run beside what (measured once, used everywhere)

| question | measured | consequence |
|---|---|---|
| Does OCP release the GIL in OCCT calls? | 8 jobs serially vs in 4 Python threads: booleans 14.6 / 14.6 s (CPU 14.5 / 14.4), meshing (`isInParallel=False`) 22.5 / 22.1 s, `VolumeProperties` 9.0 / 10.4 s | **No.** Threads never help; only processes, or OCCT's own parallel flags inside one call |
| Does OCCT parallelise for free where we call it? | `BRepMesh_IncrementalMesh` on the 161 parts, `isInParallel=True` vs `False`: 8.3 s wall / 15.8 CPU-s vs 15.9 / 15.6, arrays bit-identical (161/161) | already on (`mesh.py`, the STL); build123d sets `SetRunParallel(True)` on every boolean and the grouping's `BRepAlgoAPI_Common` does too. Small shapes leave it little to do: a fabrication runs at 1.2 cores |
| Can a worker be forked (cheap: it inherits the parts)? | a `fork` pool after the parent had meshed in parallel: all 4 children hung in `futex_wait` | **No** (OCCT's thread pool is not fork-safe) |
| `multiprocessing` spawn? | a script calling `api.verify` at module level without the `if __name__ == "__main__"` guard: the children re-import it and refuse ("An attempt has been made to start a new process before the current process has finished its bootstrapping phase"), the pool breaks | not from a library: `spiderpig/workers.py` starts `python -c` with the caller's `sys.path`; 0.12 s for a trivial job, 4-5 s for an engine job (the import 3.5 s, the plan re-made 0.5-1 s) |
| Shipping parts to a worker | `BinTools` (OCCT's binary BRep, every double exact) of the 161 parts: write 0.15 s, read 0.09 s (2.9 MB); meshes after the round trip identical 161/161. OCP's *stream* overload fails to read 13 of the 161 back ("UnExpected BRep_PointRepresentation = 15"); the *file* overload works; text BRep 0.31 s | parts cross as BinTools files (`workers.dump_shape` / `load_shape`) |
| Meshing the parts in a process pool (4 spawned workers, BinTools in) | 19.6 s wall / 44.9 CPU-s against 11.5 s in one process (load ≈ 3) | **slower**: every worker imports the engine and OCCT's mesher threads already use the cores. Not done |

## 2. `print` and `bom`: the grouping (`hardware/bom.py` `group_made`)

**Today (base).** Every made part is measured (`_sig`: build123d's `volume`, `area`,
`center(MASS)`, `principal_properties`, and our surface integration: two volume and two
surface integrations of the same part, plus the per-solid volumes) and compared with each
group's reference: invariants, then a frame matching that must carry the surface centroid
within 1e-3 mm, then the proof, one `BRepAlgoAPI_Common` (`SetNonDestructive`,
`SetRunParallel`), 0.25 s.

**Profile** (base, print alone, round 4, reloaded build): 30.8 s wall, 38.2 CPU-s;
`_shared_volume` (the booleans) 25.6 s, `_sig` 3.2 s, the 14 STLs 1.2 s.

**What the libraries offer.** Nothing that proves two solids congruent faster: OCCT's
booleans are the proof, already non-destructive and parallel. What we had and didn't use is
the build's own knowledge: the robot's right side is the left side mirrored
(`assemble_robot`: one transform per part), so a right-side part's group follows from its
twin's without a proof.

**Prototype `bf65abe`** (twins + one integration per kind). A right-side part whose
measurements mirror its left twin's (volume, area, moments to 1e-9, both centroids
z-flipped; an edited part fails this and is compared as before) joins the twin's group:
"same" when the twin is the reference's mirror image or the reference is achiral (one
proof per printed group, cached), else "mirror", which is what `congruent` answers. `_sig`
takes the frame and the area from one volume and one surface integration (build123d
integrates them again to the same numbers). Booleans 103 → **58** (round 4), 105 → **59**
(Klann); `group_made` 25.5 → **15.7 s** (42.0 → 25.4 CPU-s), Klann 27.9 → **17.2 s**.
Groups, references, names and mirrored counts identical to the base, both designs, both
paths. `tests/test_bom.py` adds the twin case (chiral and achiral, printed and laser),
checked against the plain comparison.

**Parallel (`1388086`).** The grouping needs only the made parts: they go to a worker as
BinTools files and it groups them while the export writes STEP and STL; the groups come
back by name (started only when STEP or STL is written meanwhile). In the final export
(round 4, load ≈ 2.3) this process writes STEP (3.6 s) and STL (8.5 s) while the worker
groups (22.6 s, 3.4 s of it the engine's import), then waits 11.7 s for it; Klann: 8.5 s
of STEP and STL, then 14.4 s of waiting. In-process the grouping is 15.7 s on this path,
so the worker takes ≈ 4 s off it.

**A fingerprint instead of the proof (not merged: a decision).** Under the same motion,
the multiset of the faces' (area, surface centroid) of one part onto the other's, within
1e-6 of the size (vertices won't do: a revolved pin's seam turns with it, measured: 2 000+
false negatives). Run beside every proof the base's grouping makes (no twin shortcut):
**208 proofs (103 round 4 + 105 Klann), the fingerprint agrees on all 208, in 1.8 s
against 52.9 s of booleans.** All 208 were positive: in these designs no pair that passed
the invariants and the centroid filter was ever rejected by the boolean, so the booleans
are confirmations, and the fingerprint was never tested on a near-miss. It would take the
grouping from 15.7 s to ≈ 3 s (the measurements and the STLs). It is not a proof (two
different solids could share every face's area and centroid), so whether "identical
recipe, identical faces" is enough evidence for a BOM row is the user's call; the boolean
could stay as the tie-breaker for any pair whose fingerprint matches only within a looser
tolerance.

**Still left:** 58 proofs × 0.25 s. Speculating them in parallel (run the cheap pass,
collect the proofs it will ask for, run them in workers, replay) would be identical and
take ≈ 4-5 s of wall on 4 idle cores, plus a worker's start; the grouping worker above
already hides most of it behind STEP and STL.

## 3. `glb`: the bake (`bake.py`)

**Today (base).** `fabricate_at(T_REF)` (the walker at the bake's angle: the build is at
t = 1), then the mesh sharing (`part_props` of the parts compared), then per
representative part `mesh.tessellate`: `BRepTools.Clean` + `BRepMesh_IncrementalMesh(part,
0.1, relative, 0.1, parallel)` and a Python loop over every face and node
(`poly.Node(i).Transformed(trsf)`, `X()`, `Y()`, `Z()`: five pybind calls per node),
then the pygltflib packing, the animation and the serialization (≈ 0.5 s together) and
the nodes and channels (1 s), as TIMING.md's bake table has them.

**Profile** (base, glb alone, round 4): 26.6 s wall, 37.9 CPU-s: the fabrication 11.2 s
(booleans 8.6), the meshing 8.6 s, the node loop ≈ 3.1 s, `part_props` 2.1 s.

**What the libraries offer that we didn't use: `RWGltf_CafWriter`**, OCCT's glTF writer.
As the writer of our file it can't do what the bake does (one mesh per congruence group
placed by its planar motion, the animation sampled from the template, the drive extras),
but it reads a meshed shape's triangles out in C++: faces in the shape's order, each
face's nodes in order, its location applied, a reversed face's triangles flipped, which
is what our loop did. One XCAF document with a free shape per part, written as a
temporary .glb with faces merged and no unit or axis conversion, read back with numpy:
**positions and indices bit-identical for all 161 parts of the reloaded build, 0.33 s
for all of them** (the loop: 11.4 s at load ≈ 6).

**What doesn't work.** One mesh of a compound of all parts (OCCT would parallelise
across parts): not identical, because the relative deflection is scaled by the whole
model's size (`BRepMesh_ModelBuilder` sets the model's `MaxSize` from its bounding box).
Meshing parts in worker processes: slower (section 1).

**Prototypes `e69471c`, `b701457`.** `mesh.tessellate_many`: each part meshed on its own
as before (parts that share faces, a hardware model placed twice, are read before the next
is meshed), all read out at once; anything but one untransformed node per part falls back
to the loop. The bake meshes each representative inside its kind's timer (the profiler's
keys are unchanged) and reads them together. glb alone (in-process: nothing else to
overlap): round 4 23.2 → **18.9 s** (41.4 → 36.8 CPU-s), Klann 19.3 → 16.8 s; the 161
parts of a fabrication read in 0.43 s against 3.36 s for the loop. The first version
(`e69471c`) fell back to the loop, silently, for every located part, which is every part
of a fresh fabrication (XCAF turns a located shape into a reference whose location goes
onto the glTF node and whose nodes stay local): it passed the gate and saved nothing in
the bake. `b701457` puts each part in a compound of its own (the location stays on its
faces, the writer applies it to the nodes as the loop does), logs any fallback with its
reason, and its test compares the writer's arrays with the loop's on plain, located,
mirrored, curved, multi-solid and face-sharing parts and requires the writer's path.

**Parallel (`2a53a55`, `1388086`, `7b79581`).** The glb and the MJCF need nothing from the
export but the design: a worker loads it from the store, re-makes and verifies the plan,
fabricates at `T_REF` and writes both while the export writes the rest; its warnings are
logged again in the export's report, in serial order. Started only when other formats are
written meanwhile (`1388086`): a worker's start (4-5 s) made a glb exported alone slower
(round 4: 29.6 → 34.8 s in a worker, measured). It starts before the export's own build
(`7b79581`): the MCP job reloads the parts inside `export` (7.5 s for round 4), which the
worker's fabrication now overlaps; it writes into a folder of its own whose files are
moved into the export's (dropped if the build fails). The final export's critical path
(round 4, load ≈ 2.3, `t_paths.py`): the glb/MJCF worker 32.3 s (47.3 CPU-s, 3.3 s of it
the import), this process 16 s of its own work plus 11.7 s waiting for the grouping and
5.9 s for the glb/MJCF; Klann: the worker 25.8 s, the wait 0.8 s. What is left on that
path is the fabrication at `T_REF` (≈ 9-11 s: section 7), the meshing (≈ 8 s,
OCCT-parallel already) and the worker's start.

## 4. `mjcf` (`sim/mjcf.py`)

**Today (base).** The robot at `T_REF` (shared with the glb since PERF.md), then
`robot_model`: every part's `part_props` (volume and surface integrations, and
build123d's optimal bounding box: `BRepTools.Clean` + `BRepBndLib.AddOptimal`) and its
optimal bounding box again for the z extent, then the base's collision hulls (torso,
frame plates, servos) via `mesh.tessellate` at 0.5 and scipy's `ConvexHull` (qhull).

**Profile** (base, mjcf alone, round 4): 17.0 s; `robot_model` 6.7 s: the hulls 4.0 s
(meshing 3.05), `part_props` 2.1 s, the bounding boxes 1.25 s.

**Prototype (`2a53a55`; the hulls `e69471c`, `b701457`).** The bake hands its parts' mass
properties on (`bake_gltf(props=)`, `set_fabricated(props=)`): the same numbers on the
same robot, z extents from the same boxes; the hulls' meshes are read out together. mjcf
alone: round 4 16.8 → 14.8 s, Klann 14.5 → 13.5 s (the hulls read together; no bake to
share with); **glb + mjcf: round 4 29.7 → 21.3 s (53.2 → 44.1 CPU-s), Klann 23.4 → 17.9
s**. The MJCF and its metadata are identical.

**Libraries.** qhull (through scipy) takes 36 ms on round 4's six hulled parts (61 920
points) against 2.66 s to mesh them; trimesh, manifold3d or pymeshlab would not help.
CoACD / V-HACD would *decompose* the parts into several hulls: a better collision model,
slower to make, and a different MJCF; not a speed question.

## 5. `stl` (`Mechanism.export_stl`)

**Today (base).** build123d's `export_stl` of the robot's compound: `to_compound()` (a
placed copy of every part), `BRepMesh_IncrementalMesh(shape, 1e-3, relative, 0.1,
parallel)`, then `mesh.Perform()`, then `StlAPI_Writer`.

**Profile** (base, round 4): 10.3 s wall, 25.2 CPU-s (2.5 cores: OCCT's parallel mesher):
the mesher 6.45 s, **the second `Perform()` 2.74 s**, `moved`'s deep copies 0.84 s.

**Prototype `8fd15aa`.** `mesh.export_stl`: the same parameters and writer, meshed once:
**STL bytes identical**. Measured alone: 19.1 → 14.7 s, 24.2 → 17.1 CPU-s (load ≈ 5.6);
the STL export: round 4 15.5 → 11.0 s (24.8 → 18.1 CPU-s), Klann 9.0 → 8.1 s (14.3 →
11.2). The print STLs use it too.

**Not done.** One STL per part in workers: not identical (the relative deflection is
scaled by the compound's size). numpy-stl / trimesh would only write what OCCT already
writes; the time is the meshing.

## 6. `step`, `dxf`

**STEP**: build123d's `export_step` (an XDE document, `STEPCAFControl_Writer`): Transfer
2.2 s, Write 1.5 s, `moved`'s deep copies 0.8 s (base, round 4, 5.0 s). Prototype
`8fd15aa` (`shapes.moved`): 4.19 → 3.75 s round 4, 2.99 → 2.88 s Klann. A plain
`STEPControl_Writer` would drop the names and colours: no. One document, one writer:
nothing to parallelise. Its text is not reproducible run to run (section "How").

**DXF** (1.6-2.1 s): the plates' sections (booleans 0.8 s), offsets and wire points
(0.5 s), ezdxf's write 0.05 s, rectpack milliseconds. Nothing to win.

## 7. `build`, `fabricate`, and build123d's `moved`

**Today.** `fabricate_side` (one boolean per feature: 8.8 s of booleans on round 4,
CPU/wall 1.2), `assemble_robot` (the right side mirrored, every part moved),
`attach_build` (`part_props` and optimal boxes 2.2 s, the STEP files 1.5-2.1 s).

**build123d's `Shape.moved`** deep-copies the shape (`BRepBuilderAPI_Copy`) and then
replaces the copy's B-rep with `wrapped.Moved(loc)`, sharing the original's: the copy is
thrown away. `shapes.moved` (`8fd15aa`) skips it and deep-copies the Python attributes as
before (so STEP's XDE tree is the same), used for placed parts, the robot's sides, the
grouping's proofs, the print plates, the servo and the printed segments. Measured on the
161 reloaded parts: 0.82 → 0.63 s per placement of all of them; most of what is left is
the deep copy of the anytree family a STEP-reloaded part carries (its parent compound and
siblings, copied B-reps and all), which STEP's assembly tree needs. A fresh build: 17.2 →
16.6 s round 4, 14.6 → 14.6 s Klann, the same CPU: the discarded copies were a small part
of it (they matter more on the reload path and in `to_compound`, 0.8 s per STEP or STL).

**`SetUseOBB`** (oriented-box prefilter of the interfering sub-shapes, not set by
build123d): the fabrication at t = 1 (round 4), twice each, 9.27 / 8.36 s without and 9.81
/ 8.75 s with it, every part's mesh and volume identical: slower here (few, small
arguments per boolean). Not done.

**Not prototyped: the constructions in parallel.** Within a group the parts are
independent (each link's plate, each axle's segments, each crank segment); built in
workers and shipped back as BinTools, a fabrication's 8.8 s of booleans could take ≈ 3 s on
4 idle cores. It means splitting each construction's `realize` into recipes a worker can
run, and the identity gate on every output: the biggest remaining wall-time item after
this branch (it is in the build, the export's `T_REF` fabrication and each contract angle).

## 8. `verify standard`

**Today.** The build (or its reload), `verify_plan` on fresh samples, `check_side` at t = 0
and 3.2 (each fabricates the side's groups again from the plan, builds the claims'
envelope solids and cuts every part by them: ≈ 13 s each), clashes (`&` of every pair whose
optimal boxes overlap) and `is_valid` at the build's angle, the DXF pack, the BOM.

**Prototype `b2c63b7`.** Each contract angle runs in a worker (the design loaded from the
store, its plan re-made) while this process builds and checks clashes; their problems come
back in order. Reports identical (rows, failures) to the base on both designs, both paths.
At low load: round 4 43.7 → **32.0 s** (the build reloaded), 57.1 → **36.6 s** (fresh);
Klann 45.8 → **33.8 s**, 53.3 → **37.2 s**. CPU goes up ≈ 7 s (the workers' imports and
plan re-makes: 68.9 → 75.7 CPU-s). `verify full` checks four angles, so it has more to
gain (not measured).

**Not done: the clash check in a worker.** The pairs need the build's parts; a worker
started after the build costs its start (4-5 s) plus the transfer, about what it would
save (7-8.5 s on one core). (`clashes` uses build123d's destructive `&` on placed copies
that share the design's TShapes, as the grouping's proofs did before PERF.md; checked: a
clash check before an export in the same process leaves the robot's STL and the print STLs
byte-identical, both designs, the reload path.)

## 9. The bake by options, the STEP reload, new libraries

**`spiderpig bake` by options (`1ba5de6`).** It planned from its options in every
process; now through `api.plan_config` and the store, as `build`, `explain` and `audit`
do (`--store`; the default output in that store's `bakes/`). Round 4: **49.4 → 26.7 s**
with the plan in the store (48.4 s the first time, which solves and records it); the glb's
BIN and JSON chunks identical to the base's.

**The STEP reload vs a binary BRep.** Round 4's 92 STEP part files: `import_step` 3.19 s
(6.5 MB) against `BinTools` 0.05 s (1.5 MB) for the same solids, in a 7.45 s reload (the
rest is `part_props` and the placements). A BinTools file beside each STEP would save ≈ 3
s per reload (every MCP export, verify or build in a new worker), but the reloaded parts
would then be the fresh build's exactly, not the STEP round trip's, so the reload path's
STL, print STLs and DXF would change (towards the fresh build's): the decision PERF.md
left open, now with its number.

**New dependencies: none proposed.** trimesh / numpy-stl (STL and glb from numpy: OCCT
writes the STL itself, and our glb packing and serialization are ≤ 0.5 s of the bake,
TIMING.md's table), manifold3d / pymeshlab
(hulls: qhull takes 36 ms), CoACD / V-HACD (a different collision model, slower),
a geometric-hash library for the grouping (section 2's fingerprint is 40 lines of numpy
over OCCT's face properties). Where the time is (meshing, booleans, the fabrication), it is
OCCT's, and these libraries would sit after it.

## What to merge, in order

1. **`bf65abe`, the grouping's twins and single integrations.** The largest CPU saving,
   no process machinery: `print` and `bom` −8 to −15 s each (and `spiderpig build`'s
   grouping), 103 → 58 booleans. Identical on both designs, both paths; a test of the twin
   rule against the plain comparison.
2. **`8fd15aa`, STL meshed once and `shapes.moved`.** −2.7 s of mesher per robot STL (and
   per print STL), −0.2 s per placement of all 161 reloaded parts (`to_compound` for STEP
   and STL, the clash check, the robot's assembly). Byte-identical; trivially reviewable.
3. **`e69471c` + `b701457`, the meshes read by `RWGltf_CafWriter`.** −2.9 s per bake and
   faster MJCF hulls. Bit-identical arrays, a test against the loop it replaces, any
   fallback logged. Merge them together (`e69471c` alone saves nothing in the bake).
4. **The mass properties shared with the MJCF (the `props` half of `2a53a55`).** ≈ −3 s
   per glb + MJCF export (`part_props` 2.1 s and the boxes 1.25 s). Identical. It sits
   in one commit with the first export worker; split it if the workers wait.
5. **`1ba5de6`, `spiderpig bake` through the stored plan.** 49.4 → 26.7 s per bake by
   options. Identical.

   *With 1-5 and no workers* (`SPIDERPIG_WORKERS=0`): `export` of the seven formats, the
   build reloaded inside, round 4 89.8 → 61.3 s (164 → 116 CPU-s), Klann 72.6 → 53.5 s.
6. **`b2c63b7`, `spiderpig/workers.py` and verify's contract angles in workers.** `verify
   standard` 43.7 → 32.0 s (reloaded build) and 57.1 → 36.6 s (fresh), Klann 45.8 → 33.8 s
   and 53.3 → 37.2 s; +7 CPU-s. Identical reports.
7. **The export's workers (`2a53a55`, `1388086`, `7b79581`).** On top of 1-5: round 4
   61.3 → **36.8 s** (reloaded inside) and 64.5 → **45.6 s** (fresh), Klann 53.5 → **34.2
   s** and 55.4 → **41.0 s**, at the same CPU. Identical.

**All seven merged (measured at load 1.2-3.3 on the 4 cores): `export` of the seven
formats 90 → 37 s (round 4, the MCP job's path), 73 → 34 s (Klann); `verify standard`
44-57 → 32-37 s.**
The MCP's export job (TIMING.md: 184.5 s at the start of the day, 82 s after PERF.md) would
be ≈ 40 s.

**What to review in 6-7 before merging** (the process machinery, not the outputs):
memory (an export now has up to three engine processes at once, and the MCP's pool runs
two jobs: RSS not measured here); a worker outlives a killed caller until it finishes
(seen once when a measurement was stopped; its temporary folders are the caller's to
remove); the in-process caches a serial export filled (`sim.mjcf`'s fabricated robot) stay
empty, so a `verify("full")` after an export in the same Python process fabricates the
robot again (≈ 11 s); a worker's exception comes back with its message, not its
traceback. `SPIDERPIG_WORKERS=0` runs everything in-process.

## What not to do, and why (measured)

| idea | measured | why not |
|---|---|---|
| Python threads around OCCT calls | 1.0× (section 1) | OCP holds the GIL |
| `multiprocessing` fork, or a fork pool | children deadlock in `futex_wait` | OCCT's thread pool isn't fork-safe |
| `multiprocessing` spawn from the library | breaks scripts without the `__main__` guard | it re-imports the caller's main; `python -c` workers instead |
| meshing parts in a process pool | 19.6 s vs 11.5 s | worker imports and OCCT's own mesher threads |
| one mesh (or STL) of a compound / per-part STLs | not identical | the relative deflection scales with the whole shape's size |
| `SetUseOBB` on the booleans | 9.8 / 8.8 s vs 9.3 / 8.4 s, identical | slower here |
| `RWGltf_CafWriter` as the glb writer | | it writes its own node tree, normals and instancing; our glb's animation, extras and congruence sharing would be lost |
| `STEPControl_Writer` instead of the XDE writer | | drops the names and colours |
| trimesh, numpy-stl, manifold3d, pymeshlab, CoACD / V-HACD | qhull 36 ms, glb packing ≤ 0.5 s | the time is OCCT's meshing and booleans; CoACD would change the MJCF |
| the clash check in a worker | start 4-5 s vs 7-8.5 s saved | needs the build's parts; about even |

## Not done: decisions, with their numbers

| what | saving | why it is a decision |
|---|---|---|
| a face fingerprint instead of the grouping's boolean proof | the grouping 15.7 → ≈ 3 s (print, bom, CLI build) | it agreed with all 208 proofs of both designs, but it is not a proof, and every one of those proofs was positive |
| bake at the build's crank angle (t = 1) | the `T_REF` fabrication, ≈ 9-11 s: the largest item on the export's critical path now | the glb's frame 0 and the MJCF's qpos0 change (PERF.md) |
| a BinTools file beside each stored STEP | ≈ 3 s of every reload (7.5 s) | the reload path's STL, print STLs and DXF change, towards the fresh build's |
| the constructions realized in parallel (per link, axle, crank segment) | a fabrication's 8.8 s of booleans → ≈ 3 s on 4 cores (estimate) | a refactor of every construction's `realize` into recipes a worker can run, and the gate on everything |
| the grouping kept with the build in the store | 15.7 s for every later `print` or `bom` export | a store-layout decision (PERF.md) |
| the remaining 58 proofs speculated in parallel | ≈ 10 s of the grouping on 4 idle cores | identical, but more machinery; the grouping worker already hides most of it behind STEP and STL |


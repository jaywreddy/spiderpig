# CLAUDE.md — pointers for future agents

This file captures non-obvious things you'll want to know before diving into
the code. Keep it terse.

## How to run things

```bash
uv run python cli.py bake       # bake viewer/data/klann.glb
uv run python cli.py test       # pytest
uv run python cli.py view       # dev server at :8000
```

Direct invocation of the bake script (more flags than the `cli.py` wrapper):

```bash
uv run python viewer/bake_gltf.py --mode multi --frames 120 --legs 1
```

## Baking the glTF — performance profiler

`viewer/bake_gltf.py` has a built-in stage-level profiler. It is **on by
default** and prints a summary table via `logging` at the end of every bake.

Flags:

| flag | default | purpose |
|---|---|---|
| `--profile / --no-profile` | on | stage timings + metrics summary |
| `--cprofile PATH` | off | also dump `cProfile` `.prof` file + `<PATH>.txt` top-30 cumulative functions |
| `--log-level LEVEL` | `INFO` | `DEBUG` for per-class tessellation and frame-sampling chatter |

Instrumented stages (keys in the summary table):

1. `1_reference_build` — build one reference `Mechanism` at `t=0` with build123d parts
2. `2_tessellate_total` + `2_tessellate.<class>` — OCCT tessellation per unique body class
3. `3_gltf_pack_geometry` — accessor/bufferview/material packing
4. `4_animation_sample_total` with three sub-timers (run once per bake now,
   not once per frame):
   - `4.1_template_build` — one-shot `MechanismTemplate` assembly
     (includes all `klann.create_geometry` / `lambdify` work)
   - `4.2_template_sample` — vectorized batched-BFS pose propagation
     over the whole `ts` array
   - `4.3_trs_batch` — batched Procrustes + quaternion hemisphere fix
     per body
5. `5_gltf_nodes_channels` — glTF node + animation sampler/channel assembly
6. `6_foot_path_extra` — 64-sample foot path written to scene extras
7. `7_serialize` — `pygltflib.GLTF2.save_binary`

Plus `bake_total` wrapping everything. The inner `klann.*` sub-timers
(`4.1a_klann.create_geometry`, `4.1b_klann.lambdify`,
`4.1c_klann.joints_at_eval`, `4.1d_klann.assemble_leg`) fire inside
`1_reference_build` + `4.1_template_build` and surface the SymPy /
lambdify cost broken down by leg and by intersection.

Metrics the summary reports: `n_frames`, `n_legs`, `n_bodies`, per-class
`verts.*` / `tris.*`, `blob_bytes`, `gltf_bytes`, `animation_channels`,
`accessors`, `peak_rss_mb`. Counters: `body_extract.calls` and
`body_extract.skipped_*` (the latter surface mechanism-side naming-mismatch
fallbacks).

### Known hot stage

`1_reference_build` now dominates (~65% of bake on `--frames 60 --legs
2`) — build123d/OCCT parts + tessellation prep. This is the one-shot
cost of generating the canonical geometry at `t=0` and is not
per-frame. The frame loop (`4_animation_sample_total`) is under 20%.

Historical: before `19e020e` + the template refactor,
`4.1_frame.build_assembly` dominated (> 90%) because
`klann.create_klann_geometry` + `sp.lambdify` ran once per leg per
frame. The `MechanismTemplate` / `SampledPoses` layer in
`mechanism.py` now caches one `KlannSolution` per `(orientation,
phase)` and evaluates all frames in one vectorized numpy pass. Don't
reintroduce per-frame symbolic rebuilds.

### How to extend

The profiler lives in `viewer/bake_gltf.py` as `_Profiler`. To add a new
bracket:

```python
with prof.timed("label"):
    ...
prof.bump("counter_name")
prof.set_metric("metric_key", value)
```

All output goes through `logging.getLogger("bake_gltf")` — do not revert to
`print`.

## Repository map

| file | role |
|---|---|
| `klann.py` | symbolic Klann solution + `build_klann_mechanism` / multi-leg / double / decker / quad assemblers. Each single-t `build_*_mechanism` has a parametric-in-t sibling `build_*_template` that returns a `MechanismTemplate`. |
| `mechanism.py` | `Body` / `Joint` / `Pose` / `Mechanism` kinematic-tree model; `Mechanism.solved()` does scalar BFS. `MechanismTemplate` / `BodyTemplate` / `JointTemplate` / `SampledPoses` hold a parametric-in-t version; `MechanismTemplate.sample(ts)` does a batched-BFS over `(T, 4, 4)` tensors. |
| `shapes.py` | build123d part generators (torso, segments, shaft connectors) |
| `layout.py` | sheet packing for DXF export |
| `viewer/bake_gltf.py` | end-to-end `.glb` bake for the three.js viewer |
| `server/app.py` | dev server; calls `bake_gltf()` on demand per mode |

### Pipeline contract

The bake has five pipeline stages, each with a single responsibility:

1. **Symbolic** — `klann.KlannSolution` holds a sympy `Symbol("t")` and
   symbolic `Point`/`Circle`/`Segment`. Depends on `(orientation, phase)`.
2. **Compiled** — `KlannSolution.callables` (lambdified `(t,) -> (x, y)`
   per named point) accept numpy arrays.
3. **Template** — `MechanismTemplate` carries static topology plus
   per-joint `pose_at: Callable[[np.ndarray], np.ndarray]` closures that
   evaluate the compiled callables and embed the result in SE(3) at the
   right joint layer.
4. **Sampled** — `SampledPoses` is the batched-BFS output:
   `world[body_name] -> (T, 4, 4)` + precomputed per-joint world XYZ.
5. **Serialized** — `pygltflib.GLTF2` + `.glb`.

When extending (new linkage type, new assembly mode), follow the pattern:
add a `build_*_mechanism` for single-t/parts callers, and a
`build_*_template` for the bake. Keep the two in lockstep —
`tests/test_bake_gltf.py::test_template_matches_scalar` guards this.

## House rules

- Don't add `print` statements to the bake path — use the `bake_gltf` logger.
- Don't regress the profiler (keep the stage keys stable; downstream scripts
  may parse them).
- `verbose=True` on `bake_gltf()` is back-compat only: it forces the logger
  to DEBUG. Prefer `--log-level DEBUG` from the CLI.

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
4. `4_animation_sample_total` with per-frame sub-timers:
   - `4.1_frame.build_assembly` — kinematic rebuild at each `t` (no parts)
   - `4.2_frame.solved` — BFS pose propagation via `Mechanism.solved()`
   - `4.3_frame.body_extract` — joint extraction + Procrustes + quaternion
5. `5_gltf_nodes_channels` — glTF node + animation sampler/channel assembly
6. `6_foot_path_extra` — 64-sample foot path written to scene extras
7. `7_serialize` — `pygltflib.GLTF2.save_binary`

Plus `bake_total` wrapping everything.

Metrics the summary reports: `n_frames`, `n_legs`, `n_bodies`, per-class
`verts.*` / `tris.*`, `blob_bytes`, `gltf_bytes`, `animation_channels`,
`accessors`, `peak_rss_mb`. Counters: `body_extract.calls` and
`body_extract.skipped_*` (the latter surface mechanism-side naming-mismatch
fallbacks).

### Known hot stage

`4.1_frame.build_assembly` dominates (> 90% of bake time on `--frames 60
--legs 2`). Per-frame SymPy reconstruction of the Klann geometry
(`klann.create_klann_geometry` + `lambdify`) is the culprit — not
tessellation, not the BFS solve, not pygltflib. Use `--cprofile` to confirm
before optimizing.

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
| `klann.py` | symbolic Klann solution + `build_klann_mechanism` / multi-leg / double / decker / quad assemblers |
| `mechanism.py` | `Body` / `Joint` / `Pose` / `Mechanism` kinematic-tree model; `Mechanism.solved()` does the BFS |
| `shapes.py` | build123d part generators (torso, segments, shaft connectors) |
| `layout.py` | sheet packing for DXF export |
| `viewer/bake_gltf.py` | end-to-end `.glb` bake for the three.js viewer |
| `server/app.py` | dev server; calls `bake_gltf()` on demand per mode |

## House rules

- Don't add `print` statements to the bake path — use the `bake_gltf` logger.
- Don't regress the profiler (keep the stage keys stable; downstream scripts
  may parse them).
- `verbose=True` on `bake_gltf()` is back-compat only: it forces the logger
  to DEBUG. Prefer `--log-level DEBUG` from the CLI.

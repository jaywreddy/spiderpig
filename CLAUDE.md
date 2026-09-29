# CLAUDE.md — pointers for future agents

This file captures non-obvious things you'll want to know before diving into
the code. Keep it terse.

## How to run things

Tasks live in `mise.toml`:

```bash
mise run view       # FastAPI + Vite (HMR); URL printed in startup banner
mise run bake       # bake viewer/data/*.glb
mise run test       # pytest (runs viewer-build first; -m e2e for browser tests)
mise run build      # STEP/STL/DXF -> build/
mise run lint       # ruff check
mise run audit      # do the parts physically fit? (see docs/audit/AUDIT.md)
mise run kill       # stop dev servers spawned from THIS worktree
mise run kill-port -- --port 5173   # force-stop whoever is on a port (orphan recovery)
mise run clean
```

Direct invocation of the bake script (more flags than `mise run bake`):

```bash
uv run python viewer/bake_gltf.py --mode multi --frames 120 --legs 1
```

The viewer is a Vite + TypeScript app under `viewer/src/`. In dev, Vite
serves on a port derived from a CRC32 hash of the worktree path
(range 5500-5999) and proxies `/api` + `/ws` to FastAPI on a similarly
hash-derived port (8500-8999). This means parallel worktrees get unique
stable ports with **zero manual config** — just `mise run view` in each
and bookmark the URL printed in the banner. Override via env vars in
`mise.local.toml` (gitignored) when the auto-picked port collides:

```toml
# mise.local.toml — per-worktree, not committed
[env]
VITE_PORT = "5173"   # pin the main checkout to the canonical port
API_PORT  = "8000"
```

For single-port runs (e2e tests, prod-like), build first with
`mise run viewer-build`; `server/app.py` auto-mounts `viewer/dist` when it
exists (override via `SPIDERPIG_VIEWER_DIST`).

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

1. `1_reference_build` — freeze the template at `t=0`, solve (or reuse) the
   stack plan, fabricate every part with build123d
2. `2_tessellate_total` + `2_tessellate.<kind>` — OCCT tessellation, one mesh
   per body except b1..b4, which every leg shares
3. `3_gltf_pack_geometry` — accessor/bufferview/material packing
4. `4_animation_sample_total` with three sub-timers (run once per bake now,
   not once per frame):
   - `4.1_template_build` — one-shot `MechanismTemplate` assembly
     (includes all `klann.create_geometry` / `lambdify` work)
   - `4.2_template_sample` — vectorized batched-BFS pose propagation
     over the whole `ts` array
   - `4.3_trs_batch` — batched planar rigid fit + quaternion hemisphere
     fix per body; hardware (`Body.rigid_with`) copies its host's motion
5. `5_gltf_nodes_channels` — glTF node + animation sampler/channel assembly
6. `6_foot_path_extra` — 64-sample foot path written to scene extras
7. `7_serialize` — `pygltflib.GLTF2.save_binary`

Plus `bake_total` wrapping everything. The inner `klann.*` sub-timers
(`4.1a_klann.create_geometry`, `4.1b_klann.lambdify`,
`4.1c_klann.joints_at_eval`, `4.1d_klann.assemble_leg`) fire inside
`1_reference_build` + `4.1_template_build`. `4.1b` fires once per process:
the symbolic program is compiled once and cached.

Metrics the summary reports: `n_frames`, `n_legs`, `n_bodies`, per-class
`verts.*` / `tris.*`, `blob_bytes`, `gltf_bytes`, `animation_channels`,
`accessors`, `peak_rss_mb`. Counters: `body_extract.calls` and
`body_extract.static` (bodies with no joints and no host).

### Known hot stage

`1_reference_build` (OCCT parts, ~65%) and `2_tessellate_total` (~30%)
dominate; the frame loop is ~1%. Quad bake ≈ 4.7 s, single ≈ 1.7 s.

Historical: the symbolic solve used to run per leg per call (per frame,
before `19e020e`), and substituted expressions grew to ~34k ops. The
straight-line program in `klann.py` is compiled once per process; a leg's
phase is a time shift. Don't reintroduce per-leg or per-frame solves.

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
| `klann.py` | symbolic core (`STEPS`: a straight-line program over exact `PROPORTIONS`, compiled once by `compile_program`) + one template builder per assembly (`build_*_template`). Single-t `build_*_mechanism` = template `.freeze_at(t)`, fabricated when `with_parts`. |
| `mechanism.py` | `Body` / `Joint` / `Pose` / `Mechanism`; `MechanismTemplate` / `SampledPoses` for batched sampling. All joints sit at z = 0: kinematics is planar. |
| `stack.py` | the layer plan: which slot each link occupies, where pin flanges/shafts go, and the built-up crankshaft. `StackProblem.solve()` searches slots against full-cycle clearance tables; `verify_plan()` re-checks a plan independently. |
| `fabricate.py` | plan -> build123d parts: links, frame (one solid), crank segments + crankpins, pins with press-on caps, sleeves. `plan_for(tmpl)` caches solved plans. |
| `shapes.py` | build123d primitives (disc, pill, link plate, pin, cap, sleeve) and dimensions |
| `layout.py` | DXF sheets of the laser-cut links (b1..b4); errors instead of dropping parts |
| `scripts/audit_fab.py` | `mise run audit`: solids, OCCT clashes, plan re-check, DXF |
| `viewer/bake_gltf.py` | end-to-end `.glb` bake for the three.js viewer |
| `server/app.py` | dev server; calls `bake_gltf()` on demand per mode |

### Pipeline contract

1. **Symbolic** — `klann.STEPS`: each point is a small sympy expression over
   earlier points' symbols, `t`, chirality `s` and the proportions.
2. **Compiled** — `compile_program()` (cached) lambdifies it once;
   `KlannSolution(orientation, phase).evaluate(ts)` runs it at `ts + phase`.
3. **Template** — `MechanismTemplate`: topology, per-body `outline`, and
   per-joint `pose_at` closures over the compiled program.
4. **Plan** — `stack.problem_from_template(tmpl).solve()`: Z slots for every
   link, pins and crank, valid over the whole crank cycle.
5. **Fabricated** — `fabricate(tmpl.freeze_at(t), plan)`: parts in world
   coordinates at `t`; hardware bodies carry `rigid_with`.
6. **Serialized** — STEP/STL/DXF (`main.py`) or `.glb` (`bake_gltf`).

To add an assembly: write a `build_*_template` (compose legs with
`_legs`, `combine_connectors`, `fuse_couplers`, `fuse_torsos`), then add it
to `main.py` / `bake_gltf._TEMPLATES`. The plan and parts follow; run
`mise run audit` to confirm it can be built.

Physical rule the planner enforces: every b1 sweeps within 0.05 mm of the
crank axis O, and the crank turns fully relative to b1, so nothing on the
crank except b1's own crankpin may pass through b1's slot. Multi-deck
assemblies therefore get a built-up crankshaft (webs either side of each b1).

## House rules

- Don't add `print` statements to the bake path — use the `bake_gltf` logger.
- Don't regress the profiler (keep the stage keys stable; downstream scripts
  may parse them).
- Don't put Z into joint poses. Z is the stack plan's job.
- A change that alters parts should leave `mise run audit` green.
- `verbose=True` on `bake_gltf()` is back-compat only: it forces the logger
  to DEBUG. Prefer `--log-level DEBUG` from the CLI.

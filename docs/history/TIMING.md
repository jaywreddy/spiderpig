# Where the time goes: a timing study of the agent surface — 2026-10-01

> **History** (2026-10-01; historical: a timing study of the agent surface before the planner, export and W3 build speedups. Its numbers are out of date; current ones come from `mise run scorecard`.)

One goal, driven three ways on the known path, every step timed: agent time versus
tool wall versus the engine's compute, cold versus warm. Numbers only; nothing fixed.

**The goal** (round 4's regression): "a four-legged walker in a 300 × 200 × 150 mm box,
at most $120 in purchased parts, ground clearance ≥ 40 mm, as fast as possible; sheets,
print files, BOM, and show it to me". The known answer is the Strider `double` on the
XL330 and plywood with `shin 16`, `unit 6.3` (design `4ee6eedf5b8caaca`: 20 layers / 60
mm proven, 50.6 mm of clearance, 169 mm/rev, 290 mm/s, 208 × 145 × 181 mm, 241 g, $134.91
with the $15 allowance — the budget row fails as round 4 found). The path: the survey
(`describe` the 17 walker cards), the goal spec at the Strider double defaults
(`resolve`, `check`, `plan`, `walk`, `verify quick`), three candidates (`derive` +
`verify quick`: plywood; + shin 16; + unit 6.3), the final (+ XL330: `verify quick`,
`verify standard`, `build`, `export` all seven formats, `view`, a headless Chromium
load of the page until the drive panel's HUD answers), then `verify standard` and
`export` again (warm).

## Method

- **Machine**: 4 cores, 16 GB, a Firecracker VM, Python 3.12, the worktree's package
  (`PYTHONPATH=.`, the MCP server as `python -m spiderpig.cli mcp --store <tmp>`).
  `uptime` before each run: MCP 0.00, API 0.24, CLI 1.34 (the API run's decay), the
  extras 1.45-2.30 (likewise); `top`/`ps` showed nothing but the harness. A load of 1.9
  at 00:08, fifteen minutes before the first run, had decayed to 0.00 when it started.
  Nothing else ran during a measured step; the browser steps raised the load to ~4
  (Chromium's software GL beside the view server) but overlapped no engine step.
- **Buckets**: *tool wall* = a tool call's round trip as the client measured it
  (`time.perf_counter()` around `client.call_tool`; a CLI step's process wall; a run's
  `rusage`). *Engine compute* = seconds inside the engine: the store's `log.jsonl`
  (`op`, `seconds`, `cached`), the job records, the reports' `seconds`, the bake
  profiler's table (`spiderpig bake --log-level INFO`; `api.export` bakes with
  `profile=False`, so the API's and MCP's bakes print none), per-format `api.export`
  calls into fresh folders, and `cProfile` for the planner's inside. *Agent time* =
  the gaps between tool calls, from a stamp (`t.sh LABEL`, epoch ns) at the start and
  end of every tool call of this session, from the first action to the last.
- **CPU versus wall**: `resource.getrusage` (self + children) around every API step
  and every CLI process (`/usr/bin/time` is not on the machine).
- **Raw logs**: the scratchpad's `timing/` (`agent.log`, `mcp_steps.jsonl`,
  `api_{cold,warm}_steps.jsonl`, `cli_steps.jsonl`, `export_formats_steps.jsonl`,
  `pw.jsonl`, `rusage.jsonl`, the stores, the results' JSON, the profiles).
- A scripted client drove each path back to back, so the per-call agent time inside a
  run is nil; this session's own gaps between hand-made tool calls (median 15 s, mean
  23 s with nothing running; round 4's log: 3.4 min of pauses over ~30 calls, ~7 s each)
  are the measured cost of composing one call, and the whole-study split (section 2)
  is the honest agent-versus-machine number.

## 1. Per-step tables

### MCP (one session, a fresh store; tool wall = client round trip)

| step | tool wall s | engine s | cold / warm | what the engine did |
|---|---|---|---|---|
| connect | 4.68 | – | cold | spawn the server (imports 3-4 s), initialize |
| read `spiderpig://guide` | 2.70 | – | cold | 27 kB rendered from the registries and `TARGET_FIELDS` |
| `list_tools` | 0.03 | – | | 20 tools |
| `list_linkages(walker)` | 0.11 | – | | 17 cards |
| `describe` × 17 | 12.78 | 12.7 | cold | the walk model per module + sensitivity; `trotbot_toe` 6.21, `strider` 1.72, `trotbot_heel` 0.96, median 0.23 |
| `catalog` | 0.02 | – | | |
| `resolve` (Strider double defaults) | 0.03 | 0.00 | | id `3c472682ca4d8676` |
| `check` | 0.62 | 0.58 | cold | program checks, static facts |
| `plan` | 1.40 | 1.37 | cold | 16 layers / 48 mm, optimal (1360 nodes rule out 15) |
| `walk` | 0.70 | 0.37 | cold | + 0.29 s re-making the stored plan |
| `verify quick` | 0.34 | 0.30 | warm | check / plan / walk from the store; fails clearance (22.6 mm) and the cost floor |
| `derive` c1 (plywood) | 0.04 | – | | |
| `verify quick` c1 | 2.09 | 2.08 | cold | a 2 s plan |
| `derive` c2 (+ shin 16) | 0.01 | – | | |
| `verify quick` c2 | 44.31 | 44.30 | cold | the plan 44.1 s (single-threaded: CPU = wall) |
| `derive` c3 (+ unit 6.3) | 0.01 | – | | |
| `verify quick` c3 | 44.85 | 44.83 | cold | the plan 44.1 s |
| `derive` final (+ XL330) | 0.01 | – | | id `4ee6eedf5b8caaca` |
| `verify quick` final | 27.22 | 27.21 | cold | the plan 26.4 s: 20 layers / 60 mm, proven (15 453 nodes rule out 19); ok |
| `verify standard` final (job) | 58.74 | 54.79 | cold | job 58.72: **3.9 s worker spawn + load**; inside: plan re-made 1.0, build 17.5, contract at t = 0 and 3.2, clash + solids at t = 1, plan re-check, DXF pack, BOM ≈ 36 |
| `build` final (job) | 7.86 | 7.81 | warm | the parts reloaded from STEP (`cached`), 161 parts |
| `export` final (job), 7 formats | 60.03 + 60.04 + 60.01 + 4.42 = **184.5** | 184.47 | cold | 29 files; inside (measured apart, below): build reload 8.0, print + BOM grouping ~100, glb 52, mjcf 14, stl 7.5, step 3.7, dxf 1.6 |
| `view` final | 6.58 | – | cold | starts the `spiderpig view --serve-only` child, probes its port |
| browser (headless Chromium) | page 3.7, glb (10.5 MB, `X-Spiderpig-Glb: export`) 8.1, drive GUI 11.0; HUD numbers **15.0** (drive toggle at 12.2, measured in the extra run) | – | warm (the export) | the first script waited 150 s for an `/api/walk` the glb's extras make unnecessary and read the HUD from `innerText` (lil-gui's inputs): a script defect, not page time |
| `verify standard` final again | 0.04 | 0.007 | warm | `verify.json` |
| `export` final again | 0.03 | 0.01 | warm | `export.json`, the files still there |
| `get_design(log)`, `explain` | 0.02, 0.65 | – | warm | explain re-makes the plan (0.35) + advise |
| **session** | **561.8** (≈ 416 without the script's 146 s wait) | **≈ 381** | | CPU of the server and its workers: 551 s user + 57 s sys |

### Python API (one process cold, a second process warm; CPU = the process's own)

| step | wall s | CPU s | engine s | cold / warm | what the engine did |
|---|---|---|---|---|---|
| `import spiderpig` | 0.00 | 0.0 | – | cold | the package's `__init__` imports nothing |
| `import spiderpig.api` | 3.06 (4.4 in a later process) | 3.6 | – | cold | build123d 2.1 (OCP 1.16, IPython 0.44 via `topology.shape_core`, exporters 0.25), sympy 0.40, `stack` 0.27 |
| `list_linkages` + 17 `describe` | 14.18 | 14.0 | 14.0 | cold | as the MCP: `trotbot_toe` 6.80, `strider` 1.81 |
| `resolve` | 0.004 | | | | |
| `check`, `plan`, `walk`, `verify quick` | 0.57, 1.38, 0.08, 0.006 | | 0.57, 1.38, 0.08 | cold | the base's 16-layer plan |
| three candidates: `derive` + `verify quick` | 2.13, 43.78, 43.55 | 2.1, 43.4, 43.3 | | cold | the plans (CPU = wall: one core) |
| `derive` + `verify quick` final | 26.76 | 26.7 | 26.4 | cold | the 20-layer proof |
| `verify standard` final | 54.88 | **89.2** | 54.87 | cold | 1.6 cores: OCCT's parallel booleans in the build (17.5) and the contract's fabrications |
| `build` final | 0.00 | | 17.47 (inside verify) | warm (in-process) | |
| `export` final, 7 formats | 119.53 | **242.6** | 119.52 | cold | 2.0 cores; 27 files (the manifest counted once) |
| `verify standard`, `export` again | 0.00, 0.00 | | | warm (in-process) | the handle's reports |
| `explain` | 0.28 | | | | |
| **cold process** | **311.6** | 431.7 user + 36.7 sys | ≈ 307 | | imports 1 % |
| second process: `import spiderpig.api` | 3.11 | 3.7 | | cold | |
| `load`, `check`, `walk`, `verify quick` | 0.007, 0.001, 0.003, 0.007 | | 0 | warm | JSON reads |
| `plan` | 0.87 | 0.9 | 0.87 | warm | the stored layout re-made and `verify_plan`ed (`reused = "store"`) |
| `verify standard` | **45.00** | 70.9 | 45.0 | **not warm** | `verify.json` holds one level: the `quick` call before it evicted the standard result; the build reloads from STEP (7.1 s) and the rest (≈ 37 s) runs again |
| `build`, `export` | 0.00, 0.002 | | | warm | the reload inside verify; `export.json` |
| `export(["glb"])` into a fresh folder | 47.22 | 64.5 | 47.21 | cold (the bake) | the bake fabricates from the config again |
| **warm process** | **97.8** | 133.2 + 8.4 | | | the cached operations 1.2 s in all |

Inside `export`, one format at a time in a warm process (the build reloaded from
STEP first, 7.86 s; each into a fresh folder so nothing is served):

| format | wall s | CPU s | what |
|---|---|---|---|
| `step` | 3.72 | 3.7 | one compound STEP |
| `stl` | 7.55 | 22.7 | tessellation (3 cores) |
| `print` | **100.31** | 221.9 | `group_made(printed)`: every part against every group's reference by `congruent` (volume, area, inertia frame, then a proper-fit boolean, both ways); 2.2 cores |
| `dxf` | 1.62 | 2.1 | the pack + ezdxf |
| `bom` | **100.63** | 222.3 | the same grouping (computed once when both are asked) + the BOM |
| `glb` | 52.52 | 69.3 | `bake_gltf(cfg)`: fabricates the robot again at t = 0, tessellates, animates |
| `mjcf` | 14.41 | 27.2 | `build_mjcf(cfg)`: its own build and hulls |
| `glb` again, same process | 19.32 | 35.8 | the fabrication cached in-process: tessellate + share + pack |

The sum (grouping once) is ≈ 188 s on STEP-reloaded parts, which is the MCP worker's
184.5 s; the API's in-process export was 119.5 s because its parts were fresh (the
grouping ≈ 40 s there: the reloaded solids' properties are recomputed per comparison).

The bake's own table (`spiderpig bake --log-level INFO`, a fresh process, 56.9 s with
4.4 s of imports): `bake_total` 52.5 = `1_reference_build` 41.0 (78 %: the 27 s plan
+ 14 s fabricating 165 bodies) + `2_tessellate_total` 8.0 (15 %: printed 3.3, the
two servos 3.2, metal 0.6, frame 0.6) + `2_mesh_share` 2.0 + `5_gltf_nodes_channels`
1.0 + the rest 0.5 (animation 0.08, serialize 0.08); 132 meshes, 10.3 MB blob.

### CLI (the final design by its build options; every step a fresh process)

| step | wall s | CPU s | cold / warm | what the process did |
|---|---|---|---|---|
| `spiderpig --help` | 0.03 | 0.0 | | no engine import |
| `python -c "import spiderpig.api"` | 4.38 | 4.9 | cold | the engine's imports (section 3) |
| `spiderpig explain --help` | 4.59 | 5.1 | cold | argparse's choices need the registries: the whole engine imports for `--help` |
| `explain` | 32.41 | 32.7 | cold | imports 4.4 + check 0.6 + the plan 27 (20 layers, proven) |
| `build` (STEP, STL, print, DXF, BOM to `--out`) | **132.97** | 233.7 | cold | imports 4.4 + plan 27 + fabricate 17.5 + step 3.7 + stl 7.5 + print/BOM grouping ≈ 70 + dxf 1.6 |
| `audit --proportion ...` | 4.31 | 4.8 | – | **refused**: `audit` has no `--proportion` (it takes `add_build_args` but not `add_design_args`); the final design's shin/unit cannot be audited from the CLI |
| `audit` at the default proportions | 91.35 | 154.3 | cold | plan 1.4 + build + contract at 4 angles + clashes at 2 + DXF + BOM |
| `bake --log-level INFO` | 56.85 | 74.4 | cold | the table above |
| `export` by options, 7 formats (fresh store) | **176.16** | 310.5 | cold | resolve into the store + plan 27 + build 17.5 + the formats ≈ 127 (grouping ≈ 48 on fresh parts, glb 52, mjcf 14, stl 7.5, step 3.7, dxf 1.6) |
| `export <id>` again | 4.41 | 4.9 | warm | imports; the store's export served as is |
| `view <id>` (warm store) | 3.6-3.9 to the URL | 6-7 | warm | imports + the glb served from the export + uvicorn |
| browser, warm | 15.0-15.4 to the HUD | | | launch 0.6, page 2.7-2.9, glb 5.6-6.0, drive GUI 6.5-9.1, status 9.0, toggle 12.2, HUD `169.07 mm/rev · 290.24 mm/s` 15.0 |
| `view` by options (fresh store) | 67.02 to the URL | 96.0 | cold | imports 4.4 + plan 27 + build 17.5 + the bake 18 (the fabrication cached in-process) |
| **the five asked steps** (explain, build, audit, export, cold view) | **≈ 500** | | | imports 5 × 4.4 = 22 s (4 %); the plan solved again in four of them, 4 × 27 = 108 s (22 %) |

## 2. Totals, the split by bucket, the top five

**By path, the goal to the files and the viewer:**

| path | wall | engine compute | overhead | agent |
|---|---|---|---|---|
| MCP, one session | 561.8 s (≈ 416 without the script's 146 s browser wait) | ≈ 381 s (68 %; 92 % of the corrected session) | ≈ 19 s (3 %): connect 4.7, guide 2.7, worker spawn 3.9, viewer start 6.6, JSON ≈ 1; the page 15 s | 0 scripted; by hand 32 calls × 7-23 s = 220-740 s (round 4's and this session's measured per-call cost) |
| Python API, cold process | 311.6 s | 307 s (98.5 %) | imports 3.1 s (1 %) | one script |
| Python API, warm process | 97.8 s | 93 s (the evicted standard verify 45, the fresh-folder bake 47; the cached operations 1.2) | imports 3.1 s | |
| CLI, five fresh processes | ≈ 500 s | ≈ 370 s | imports 22 s (4 %) + the plan solved four times 108 s (22 %) | five commands |

**The whole study** (this session, first stamp to the last measurement, 50 min):
machine busy 2407 s (80 %: the eight background runs 2403 s, the foreground reads 4
s), nothing running 604 s (20 %: reading the docs and the code, writing the drivers,
reading results); to the commit, 55 min: machine 74 %, agent 26 % (writing this
document); 62 stamped tool calls plus ~20 file writes; the gap between hand-made
calls: median 15 s, mean 23 s with nothing running (median 20 s, mean 57 s counting
the waits beside a run).

**CPU versus wall**: the planner is single-threaded pure Python (CPU = wall on every
plan); builds and exports run OCCT's parallel booleans and tessellation at 1.5-2.2
cores (verify standard 89 CPU-s / 55 wall-s, export 243 / 120, the grouping 222 /
100); the MCP session as a whole used 551 CPU-s in 562 wall-s. Nothing waits on a
subprocess but the MCP's jobs (the worker spawn 3.9 s once) and the viewer (6.6 s once).

**The top five steps by wall, and what is inside them:**

1. **`export` of the seven formats: 184.5 s (MCP worker) / 119.5 s (API, in-process)
   / 176 s (CLI, cold).** The print + BOM grouping (100 s on parts reloaded from STEP,
   ≈ 40-70 s on fresh ones: `group_made` compares every part to every group's
   reference with volume, area, inertia and a proper-fit boolean, the properties of a
   reloaded solid recomputed per comparison), then the glb bake 52 s (it fabricates the
   robot again at t = 0: 14 s, plus the plan when the process hasn't it; tessellation
   8 s), the MJCF 14 s (its own build and hulls), stl 7.5, step 3.7, dxf 1.6, the build
   reloaded 8 s. Three fabrications of one robot in one export.
2. **The plans of the three candidates and the final: 44.3 + 44.8 + 26.4 s (+ 1.4
   and 2.1 for the base and plywood).** `stack.StackProblem.solve`, single-threaded;
   under `cProfile` two thirds of the search is the crank router (`route.check` /
   `route.route`: 816 k `chain` calls enumerating runs along the crank's points), one
   third the layer assignment; the 60 s deadline was not reached (the final: 15 453
   nodes rule out 19 layers, 20 proven). Through the MCP the four ran one after the
   other in the server's single worker thread: 117 s of wall for 117 CPU-s on a
   four-core machine.
3. **`verify standard`: 54.8 s cold** (plan re-made 1.0, the build 17.5, contract at
   two crank angles — each a fabrication of the side — clash and solids at the build's
   angle, the plan re-check, the DXF pack, the ungrouped BOM ≈ 36), **45 s warm when
   the level was evicted** (the build reloads from STEP in 7-8 s instead of 17.5).
4. **CLI `build`: 133 s** (imports 4.4, plan 27, fabricate 17.5, the five formats ≈ 84
   with the grouping ≈ 70) and **CLI `audit`: 91 s** (four contract angles, two clash
   angles, DXF, BOM on the default Strider; refused on the final's proportions).
5. **The survey: 17 `describe` cards 12.8-14.2 s** (`trotbot_toe` 6.2-6.8 s alone:
   the walk model over its four modules), **the browser 15 s to the HUD** (the 10.5 MB
   glb at 5.6-8.1 s, the drive GUI at 6.5-11 s, the HUD 3 s after the toggle), **the
   MCP's connect + guide 7.4 s and `view` 6.6 s**.

## 3. Cold-start costs

| what | cost | where measured |
|---|---|---|
| a Python process + `import spiderpig` | 0.02 s | the package imports nothing |
| `import spiderpig.api` (the engine) | 3.06-4.4 s (3.6-4.9 CPU-s) | `-X importtime`: build123d 2.13 s (OCP 1.16, **IPython 0.44** pulled in by `build123d.topology.shape_core`, exporters 0.25), sympy 0.40, `spiderpig.stack` 0.27, `walk`/`config`/`construction` the rest; every CLI process pays it, `--help` included (4.6 s) |
| the first engine call | `describe(strider)` 1.9 s (the program's compile 0.54 s + the walk per module), `check` 0.46 s after it | `api_first_call` |
| MCP connect | 4.68 s | the server's imports + the registries + initialize |
| the guide | 2.70 s | rendered per read |
| the process pool | 3.9 s on the first job (`verify standard` 58.72 s job, 54.79 s engine: spawn, import, load); 0.01 s after | the job records against `log.jsonl` |
| the viewer server | MCP `view` 6.58 s (a `--serve-only` child); CLI `view <id>` 3.6-3.9 s to the URL (warm export); 67 s by options on a fresh store | `cli_steps` |
| the browser | Chromium 0.6-1.2 s, the page 2.7-3.7 s, the glb 5.6-8.1 s, the drive GUI 6.5-11 s, the HUD 15.0-15.4 s | `pw.jsonl` |
| the cards | 12.8-14.2 s for 17 walkers, every session | `describe` is not cached |
| the viewer's build (`vite build`) | 2.7 s (4.2 CPU-s) | once per checkout |

## 4. What the store's cache saves on the second pass

| operation | cold | warm | saved | condition |
|---|---|---|---|---|
| `verify standard` (MCP) | 54.8 s | 0.04 s | 54.7 s | the same level asked again; **a `quick` in between evicts it: 45.0 s** (API warm process) |
| `export` (MCP / API / CLI) | 184.5 / 119.5 / 176 s | 0.03 / 0.002 / 4.4 s | all of it | the same formats into the same folder, the files still there |
| `build` | 17.5 s | 7.1-8.0 s (reload from STEP) | ~10 s | the manifest's `t` and engine match |
| `plan` | 26.4 s (final), 44 s (c2, c3) | 0.3-1.0 s (re-made + `verify_plan`) | 25-43 s | every op that needs the plan pays the 0.3-1.0 s |
| `check`, `walk`, `verify quick` | 0.6, 0.1, 0.3 s | ≈ 0 | | JSON |
| `view` | 67 s to the URL (by options, fresh store) | 3.6-3.9 s (by id) | 63 s | the glb in `exports/` |
| the glb bake in one process | 52.5 s | 19.3 s | 33 s | the fabrication cached in-process only; the API's and MCP's bakes never reuse the design's build |

Across the MCP session: 381 s of engine work on the first pass, 0.4 s on the second;
nothing of the first pass but the plans' 0.3-1.0 s re-check is paid again.

## 5. Where the time goes

On the known path the engine is ~68 % of an MCP session and ~98 % of an API process:
an agent waits on the machine, not the other way round, and on three things: **the
export** (one third of the session, three fabrications of one robot and a pairwise
shape comparison for the BOM), **the plans** (one third, single-threaded, one at a time),
and **the standard verify** (a tenth). Imports, the protocol, the pool and the viewer
together are 5 %. The agent's own time is per call, 7-23 s each, so it is the same
order as the engine only where the calls are small (the survey, the resolve / check /
plan / walk sequence) — and on the long calls it is the polling: a 184 s export at the
default `wait_seconds` 15 is one call and twelve `wait_job` turns, ~12 × 7-23 s of agent
time and tokens for nothing. The CLI pays what the store already knows: four of its
five commands solve the same plan again (108 s) and every one imports the engine (22 s).

**Three changes that would cut the most wall time** (recommended, not made):

1. **One build per export.** Group the print and BOM parts by the build's identity
   (the manifest already records `same_as` + `mirror` for every right-side part, and the
   bake's `2_mesh_share` finds the exact translates in 2 s) instead of `group_made`'s
   pairwise booleans, and let `bake_gltf` and `build_mjcf` take the design's built
   mechanism instead of fabricating from the config again. Measured: the grouping 100 s
   (worker) / ≈ 40-70 s (in-process, CLI), the bake's reference build 14 s (+ 27 s of
   plan in a fresh process), the MJCF's build ≈ 10 s, the glb a second time in one
   process 19 s against 52. Saves ≈ 120 s of the MCP export's 184.5 s (→ ≈ 60 s), ≈ 60
   s of the API's 119.5 s, ≈ 70 s of the CLI `build`'s 133 s.
2. **Don't redo what the store holds.** Keep one `verify.json` per level (saves the 45
   s a `quick` after a `standard` costs on the second pass); route every CLI command
   given build options through the store as `export` and `view` already are (measured:
   `export <id>` 4.4 s against 176 s; `explain`, `build` and `audit` would reuse the 27
   s plan: 108 s of the CLI path's 500); cache the cards per engine version (12.8-14.2 s
   of every session's survey → ms). Together ≈ 165 s on the CLI path and ≈ 58 s on the
   MCP's second pass.
3. **Let the agent overlap the machine.** Run `plan` / `verify quick` as pool jobs too,
   so three candidates plan on three cores instead of one thread (44.3 + 44.8 + 26.4 =
   116 s → ≈ 45 s, the longest: each plan is CPU = wall so they don't contend); split
   `export` into per-format jobs so the glb and the viewer are ready at ≈ 50 s rather
   than 185; pre-spawn the pool and the viewer server at startup (3.9 + 6.6 s hidden
   under the agent's first calls); and let a long tool wait longer by default (or push
   its completion) so a 184 s export is one turn, not thirteen. Saves ≈ 70 s of wall on
   the survey and ≈ 135 s to the first picture, plus the polling's agent time.

Smaller, measured: `import spiderpig.api` pulls IPython (0.44 s) and sympy (0.40 s)
through build123d and the engine on every process, and `spiderpig <cmd> --help` imports
the whole engine for argparse's choices (4.6 s); `describe(trotbot_toe)` is 6.2-6.8 s of
every survey; `audit` cannot take `--proportion`, so the final design was audited at
the Strider's default proportions (91 s) and not as built.

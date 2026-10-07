# Testing: tiers, markers, the cache, recorded fixtures, the identity gate

The full suite is ~1150 tests and ~2 CPU-hours; nobody iterates on it. Each area of the
engine has a **module tier** that runs in seconds to a minute on a warm cache, the quick
tier is their union, and the full suite runs before a merge. This page is the contract
the module packages (PLAN P1-P6) build on; `tests/cache.py`, `tests/_modules.py`,
`tests/tiers.py`, `tests/conftest.py`, `tests/gate/` and the tasks below belong to P0
(send a one-line request to change them).

## Tiers

| command | what runs | when |
|---|---|---|
| `mise run test-<module>` | `-m "<module> and not slow and not e2e" -n 4 --dist worksteal` | iterating on one area |
| `mise run test-quick` | `-m "not slow and not e2e" -n 4` (every module's fast tier) | before handing back |
| full: `mise run remote-test`, or locally `pytest -p no:warnings -m 'not e2e' -n 12 --dist worksteal` | everything but the browser tests | before a merge |
| `mise run test-fixtures` | the currency tests (`-m fixture_regen`) with `--regen` | after an intended engine change |
| `mise run test-viewer` | `npm run typecheck` (P1 adds vitest) | viewer edits |
| `mise run gate -- snapshot DIR` / `compare DIR` | the identity gate (below) | before / after a product change |
| `mise run scorecard` | every ROADMAP number (below) | before and after a workstream |
| `mise run doc-check` | the docs' backticked names against the code | doc edits |

`<module>` is one of `linkage`, `planner`, `construction`, `hardware`, `strength`, `api`,
`sim`, `server`. Pytest args go after `--` (`mise run test-planner -- -k route -x`);
`SPIDERPIG_TIER_WORKERS` changes `-n`. A module with no fast test yet passes and says so
(`tests/tiers.py`'s runner turns pytest's "no tests" into success).

## The scorecard, CI, the doc check

`mise run scorecard` (`tests/scorecard.py`) writes `build/scorecard.json` and prints a table:
`spiderpig build --profile` of the Strider double (an empty store, then the same store: the
stages, wall and CPU), each module tier's and the quick tier's wall, CPU and tests (the
quick tier under coverage.py, `COVERAGE_CORE=sysmon`, pytest-cov combining the workers; its
10 slowest tests), pyright's errors (`[tool.pyright]`, basic; also without OCP/mujoco stub
noise), ruff's `RUF` findings, every module over 800 lines, CLAUDE.md's lines, the doc
check's misses and the load average around each section. `--gate`, `--full` (`--cold`: on
an empty test cache) and `--audit` add the heavy ones; `--runs 3` takes the build's medians;
`--compare A.json B.json` prints the deltas. CPU is each process's `wait4` rusage (its xdist
workers included). The baseline is `docs/agentlib/scorecard-baseline.json`.

CI (`.github/workflows/ci.yml`, every push and PR): ruff, `uv lock --check`, the viewer's
typecheck + vitest, the quick tier (`-n 4`, `SPIDERPIG_OFFLINE=1`, the fabrication cache
restored and saved with `actions/cache`, keyed by `python -m spiderpig.design`, the engine
version, plus `uv.lock` and `tests/cache.py`: no restore-keys, so a new engine starts
empty), and the doc check, report-only until W7. `.pre-commit-config.yaml` runs ruff and
`uv lock --check` (opt-in: `uvx pre-commit install`).

`mise run doc-check` (`tests/doc_check.py`) resolves every backticked dotted name, path,
`mise run X`, `spiderpig X`, `--flag` and env var in CLAUDE.md, AGENTS.md, README.md,
ARCHITECTURE.md, API.md, TESTING.md, ROADMAP.md and SCOPE.md statically (the package's AST;
no engine import); `-v` lists every check, `--strict` fails on a miss.
`tests/doc_check_allow.txt` holds what is legitimately not code (output file names, the
roadmap's planned names), never drift.

## Markers

Every test carries exactly one module marker. `tests/_modules.py` (`MODULE_OF_FILE`) gives
each file's default; a test that belongs elsewhere names its own
(`@pytest.mark.hardware`), which replaces the file's. The conftest checks it at collection
(`pytest_collection_modifyitems`, before `-m` selects): a file missing from the table, or a
test with two module markers, stops the run with the list. Adding a test file = one line in
`MODULE_OF_FILE` (each package edits only its own files' lines).

`slow` (> ~5 s, or a robot/side fabricated outside the cache) and `e2e` are unchanged;
`tiers.quick(values, keep)` still keeps one cheap case of a heavy parametrized test in the
fast tiers. `fixture_regen` marks a recorded fixture's currency test (always with `slow`).

## The fabrication cache (`tests/cache.py`)

```python
from tests import cache

cache.CACHE_ROOT        # $SPIDERPIG_TEST_CACHE, else $XDG_CACHE_HOME/spiderpig/test-cache (~/.cache/...)
cache.CACHE_DIR         # CACHE_ROOT / "<engine_version>-<env tag>"
cache.cached_design(cfg) -> (tmpl, SideDesign)               # design_side, plan seeded from disk
cache.cached_side(cfg, t=1.0, *, fresh=False) -> Mechanism   # fabricate_side, BREP-cached
cache.cached_robot(cfg, t=1.0, *, fresh=False) -> Mechanism  # fabricate (robot=True), BREP-cached
cache.seed_plan(cfg) -> bool       # seed fabricate._LAYOUTS only (a later design_side re-makes it)
cache.prebuilt_store(cfg, tmp_path, t=1.0) -> Store          # a copy of a resolved+planned+built store
cache.recorded(module, name, make) -> data                   # tests/fixtures/<module>/<name>.json
cache.assert_current(module, name, make) -> data             # that fixture's currency test
```

- `cfg` is a `spiderpig.config.BuildConfig`; `robot` is ignored by `cached_design` /
  `cached_side` (always the side) and forced by `cached_robot` (a mechanism has no robot:
  `ParamError`). The template is `fabricate.template_for(cfg)`.
- The conftest factories are wrappers with their old signatures: `design(module, servo,
  linkage="klann", **build)`, `side(module, t, servo, **build)`, `robot(module, t,
  **build)`. Every existing test uses the cache through them.
- **One object per key per process** (as the session factories always were): never mutate
  what comes back. `fresh=True` builds now and neither reads nor writes the cache (nor
  the memo): use it for a test that compares meshes, STL or DXF text with a fresh build,
  or that times or counts a fabrication.
- **Plans**: `cached_design` writes `CACHE_DIR/plans/<cfg.key>.json` (layers, top, the
  crank route, heads, gaps, thicknesses, optimality, proof, cost) after a solve, and on the
  next run seeds `fabricate._LAYOUTS` with it (and with the leg hint's single-module plan
  for a bigger module), so `design_side` re-makes and verifies it
  (`fabricate._reuse`: `problem.plan` + `verify_plan`, the path the robot's side always
  took) instead of searching. A re-made plan whose gaps, thicknesses or heads differ from
  the record is thrown away and solved again (logged). A seeded design carries the
  recorded `optimal` / `proof` / `cost`.
- **Keying**: `CACHE_DIR` is `design.engine_version()` (every engine source minus
  docstrings, the planner's defaults) plus a hash of this cache's `FORMAT`, the Python,
  build123d, OCP and numpy versions. An engine edit is a new folder: the first run after
  it builds each entry once (per machine, not per worker), nothing is invalidated in
  place. Entries: `mech/<cfg.key>_<side|robot>_t<t>/` and `stores/<cfg.key>_t<t>/`.
- **Where, and why there**: a user cache directory, not the checkout. The key already
  names the engine, so worktrees with the same engine sources (every worktree branched
  from one master commit, until it edits `spiderpig/`) share entries, and worktrees with
  different ones never meet; nothing lands in the repo or the project store
  (`.spiderpig/`, which tests keep empty). Engine folders unused for
  `SPIDERPIG_TEST_CACHE_DAYS` (14) days are removed at session start; the current one is
  touched. `rm -rf ~/.cache/spiderpig/test-cache` is always safe.
- **Off**: `pytest --no-test-cache` or `SPIDERPIG_TEST_CACHE=off` (each process builds what
  it needs once, the behaviour before the cache). The disk cache is also bypassed when
  servos aren't parametric (a manufacturer's CAD model in reach: a download, not the
  engine, would shape the parts); the tests' `_offline` fixture always makes them
  parametric.
- **xdist-safe**: one `fcntl.flock` per entry while it is built (a second worker waits,
  then loads), entries written under a temporary name and renamed into place, JSON written
  with `os.replace`. Locks are taken mechanism -> plan, never the other way.
- **Format**: one BinTools BREP of every distinct part (a compound: exact doubles,
  locations and shared sub-shapes kept, triangulations as written) plus a pickle of the
  `Mechanism` with every `part=None`; `mech.meta` and `bom_extras` survive (unlike the
  store's STEP reload). Parts come back as `Solid` / `Compound` / `Part`; a `Box`,
  `Cylinder` or other build123d class as a generic `Part` (`workers.load_shape` rebuilds by
  class name and fails on those).

**Fidelity, measured** (Klann single robot, 205 bodies, loaded vs fresh): every part's
volume and face count identical, `meta` and `bom_extras` equal, `clashes`, `bad_solids`
and `bom_from_mechanism(...).as_dict()` equal, every DXF (sheets and per-part) entity for
entity in the same order. Differences: the servo's optimal bounding box in the last
digit (2e-14 mm: a fresh build's servo shares its TShape with the process's lru-cached
model, a loaded one doesn't), and therefore one of 201 meshes when the parts are meshed
after another build's (a shared TShape keeps its first triangulation). DXF *files* differ
byte for byte between any two builds (timestamps, GUIDs): compare entities.

**Load vs build** and sizes: see Timings at the end.

## Recorded fixtures (`tests/fixtures/<module>/<name>.json`)

A recorded fixture is a small JSON document a fast test takes as **input**, so the test
checks the module's behaviour on it whatever the engine version:

```json
{"engine_version": "0.1.0+1b5e76d06b99", "generator": "tests/test_walk.py::test_foot_z_current", "data": ...}
```

```python
# the fast test: reads the data (writes it from make() only when the file is missing)
feet_z = cache.recorded("linkage", "foot_z", make=_foot_z)

# its currency test: the engine must still give the recorded data
@pytest.mark.slow
@pytest.mark.fixture_regen
def test_foot_z_current():
    cache.assert_current("linkage", "foot_z", _foot_z)
```

- `recorded` returns what JSON gives back (tuples as lists, keys as strings) whether it
  made or read the data, so a test behaves the same either way.
- **Stale** (written under another engine version): the data is still used, a
  `cache.StaleFixture` (`UserWarning`) is raised as a warning, never a skip, and the
  session header counts the stale files (visible under `-p no:warnings` too). The full
  suite's currency test is what catches a real change.
- `--regen` (or `SPIDERPIG_REGEN=1`): `recorded` and `assert_current` rewrite the file
  from `make()`; `mise run test-fixtures` runs every currency test that way. Review the
  diff; commit it. `generator` is the test's node id.
- `.gitignore` ignores `*.json` except `tests/fixtures/**/*.json`: fixtures are committed;
  cache entries never are.

## The prebuilt store

`prebuilt_store(cfg, tmp_path, t=1.0)` copies `CACHE_DIR/stores/<cfg.key>_t<t>/` (made
once per engine: `api.resolve(api.spec_of(cfg), store)` then `api.build(design, t)`:
resolved, planned, built, `build/manifest.json` + STEP parts) to `tmp_path / "store"` and
returns its `Store`; its one design is `store.ids()[0]`. `api.load(id, store)` +
`api.build(design, t)` then serves the parts from STEP (`_reload_build`), and the MCP's
jobs find a finished build. The copy is the test's own. `SPIDERPIG_STORE` stays a fresh
per-session folder (tests asserting on the project store expect it empty).

## The identity gate (`tests/gate/identity_gate.py`)

A script, not a test: run it before and after any change under `spiderpig/` that must
not change a design.

```bash
mise run gate -- snapshot /tmp/gate/before      # on the base (master)
mise run gate -- compare /tmp/gate/before       # on the branch: snapshots again, diffs
mise run gate -- diff A B                       # two snapshots
mise run gate -- compare DIR --designs klann_quad -j 1
```

Designs (`DESIGNS` at the top): the Strider double (`BuildConfig()`), the Strider quad and
the `klann_lego` quad (the user's three order designs), the demo `klann` quad,
`hoecken_pantograph` and `dwell_rocker`. Per design, in its own process: the plan
(layers, top, route, heads, gaps, thicknesses, sunk heads, height, proof, `describe()`),
`audit_module(..., sim=False)` minus `seconds`, every part of the fabrications at
`t=1` and `t=4.38` (class, fab, BOM key, sheet, colour, pose, volume, area, centre of
mass, box, solid/face/edge/vertex counts, body order), then `spiderpig build` of the
audit's own `t=1` fabrication: `bom.json` and every text it writes (`bom.csv/md`,
`ORDER.md`, the `parts.csv` / `order.csv` files, `manifest.json`), every DXF as its
entities, a hash of every STL. Servos parametric, a fresh store, `PYTHONHASHSEED=0` and
`SPIDERPIG_PLAN_SECONDS=3600` (node budgets alone bound the planner: the Strider quad's
search ends on its node budget at ~54 CPU-s, too near the 60 s default to be stable).

Verdicts per design, and the exit status: **identical** (0); **geometry identical, order
differs** (1: a DXF's entities in another order or a closed outline from another start
vertex, bodies reordered, a text file's lines reordered, a part's faces/edges counted
otherwise at the same volume, area and box, an STL mesh with the B-rep unchanged) — a
user decision, not a merge; **DIFFERENT** (2, every difference listed). Numbers compare
within 1e-9 relative / 1e-6 mm.

**Baseline of master** (`d53fbf1`, engine `0.1.0+1b5e76d06b99`):
`~/.cache/spiderpig/gate/master-d53fbf1/` on the development box (`/root/.cache/...`),
one `<design>.json` per design, their logs and `snapshot.json` (commit, branch, time).
Compare against it from any worktree whose `spiderpig/` should be master's:
`mise run gate -- compare ~/.cache/spiderpig/gate/master-d53fbf1`.

## Rules for a module package

- Write only under `tmp_path` / `tmp_path_factory` or through `tests/cache.py`; free
  ports only (`conftest._free_port`); `SPIDERPIG_WORKERS=0` where a test counts
  fabrications.
- Never mutate what a factory or `cached_*` returns: it is shared by every later test in
  the process (copy what you change, or build with `fresh=True`).
- A fast test that needs a whole design takes it from the cache; one that must build
  (`fresh=True`, a new fabrication path, meshes) is `slow`, or seeds its plan
  (`seed_plan`) and builds the smallest design that shows the point (hoecken: 3 s).
- Every recorded fixture has a `slow` + `fixture_regen` currency test.
- Tests marked `slow` stay in the full suite; module tiers never drop a test.
- Product edits: the gate before and after, "identical" in the PR.

## Timings

Measured 2026-10-06 on the 20-core development box, shared with other agents' jobs (the
load average at the end of each run in brackets): wall times move by 10-20 % with the
load, CPU sums (`/usr/bin/time`, user + sys over every worker) less.

| | master `d53fbf1` | with the cache, cold (empty `CACHE_DIR`) | with the cache, warm |
|---|---|---|---|
| full suite, `-m 'not e2e' -n 12` | 587 s [4.7], 694 s [4.4]; 625 s / 6125 CPU-s [4.2] | 689 s [14.9] | 583 s [6.6]; 580 s / 6151 CPU-s [6.1] |
| quick tier, `-m 'not slow and not e2e' -n 4` | 241 s [2.9], 244 s [4.7]; 252 s / 874 CPU-s [3.5] | 239 s [4.0] | 207 s [3.5]; 213 s / 751 CPU-s; 209 s / 725 CPU-s [4.3] |
| `mise run test-construction` | | 118 s / 441 CPU-s | 85 s / 324 CPU-s |

Every module tier, warm, at P0 (before the module packages move anything): `test-linkage`
394 tests 149 s (the `/api/walk` tests plan designs), `test-planner` 70 / 27 s,
`test-construction` 457 / 85 s, `test-hardware` 28 / 9 s, `test-strength` 37 / 31 s,
`test-api` 134 / 55 s, `test-sim` 0 (every sim test is slow today), `test-server` 17 /
20 s, `test-viewer` 102 s (the first `npm install` included).

Master has no state across runs, so its "warm" run is a second cold one. The factories'
share was smaller than the plan's estimate: a warm quick tier saves ~17 % CPU, the full
suite's CPU is unchanged within the noise (its time is in tests that fabricate, plan,
check or simulate by themselves). Those move onto the cache in P1-P6; what is left in a
warm construction tier is exactly them (`--durations`: `test_deck`'s own Strider-robot
fixture 17 s, the axle and contract tests' `check_side` and fabrications 5-11 s each).

Cache entries, loaded vs built (one process, OCCT pool 1):

| entry | bodies | load | build | size |
|---|---:|---:|---:|---:|
| Klann single side | 65 | 0.01 s | 4-8 s | 0.5 MB |
| Klann single robot | 205 | 0.04 s | 10 s | 1.4 MB |
| Klann quad side | 221 | 0.04 s | 29 s | 1.6 MB |
| Klann quad robot | 519 | 0.09 s | 34 s | 3.1 MB |
| plan seed, Klann quad / Strider double | | 3.7 / 1.3 CPU-s (re-made, verified) | 11.4 / 3.5 CPU-s (solved) | 1-2 kB |
| prebuilt store, hoecken (copy) | 43 | 0.02 s | 4.3 s | |

The whole suite fills ~23 MB per engine version (25 fabrications, the plans).

The identity gate: ~5 min wall for the six designs at once (the Strider quad 290 s, the
Strider double 150 s, the Klann quads 180-200 s, the mechanisms 45-50 s).


## Tier timings after the module packages (2026-10-06)

Warm cache, master after P0-P6, `mise run test-<module>` with its default workers
(`tests.tiers.TIER_WORKERS`), wall seconds; before = the same tier on master before P1-P6.

| tier | before | after |
|---|---|---|
| linkage | 151 | 11 |
| planner | 28-31 (96 tests) | 24 (126 tests) |
| construction | 112 | 33 |
| hardware | 29 | 7.5 |
| strength | 57 | 7.3 |
| api | 82 | 29 |
| sim | no fast tests | 27 (83 tests) |
| server | 21 | 12 |

The recorded fixtures were regenerated on that master: only their engine stamps changed, every
document's data byte-identical; the identity gate was identical on all six designs and the full
suite green.

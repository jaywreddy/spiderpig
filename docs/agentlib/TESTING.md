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
| `mise run test-cradle` | `tests/test_cradle.py -m "not slow and not e2e" -n 4` (about 20 s) | iterating on the cradles; the slow ones in chunks (below) |
| full: `mise run remote-test`, or locally `pytest -p no:warnings -m 'not e2e' -n 12 --dist worksteal` | everything but the browser tests | before a merge |
| `mise run test-fixtures` | the currency tests (`-m fixture_regen`) with `--regen` | after an intended engine change |
| `mise run test-viewer` | `npm run typecheck && npm test` (vitest) | viewer edits |
| `mise run gate -- snapshot DIR` / `compare DIR` | the identity gate (below) | before / after a product change |
| `mise run gate -- doc DIR` | `docs/agentlib/DESIGNS.md` from a snapshot | with each new baseline |
| `mise run scorecard` | every ROADMAP number (below) | before and after a workstream |
| `mise run doc-check -- --strict` | the docs' backticked names against the code | doc edits (CI blocks) |

`<module>` is one of `linkage`, `planner`, `construction`, `hardware`, `strength`, `api`,
`sim`, `server`. Pytest args go after `--` (`mise run test-planner -- -k route -x`);
`SPIDERPIG_TIER_WORKERS` changes `-n`. A module with no fast test yet passes and says so
(`tests/tiers.py`'s runner turns pytest's "no tests" into success).

### Chunks

An agent's step (a foreground command, or the wait for a background one) should end within
5 minutes: past that the model's prompt cache expires, and the next call re-reads the
whole context uncached. A job longer than that runs in the background and in chunks, each
one's end a notification. `tests/test_cradle.py`'s slow tests (each a fabrication of a
whole cradle or more: 25-35 s fresh, 4-6 s at another crank angle in the same worker, from
`cached_product` when it holds this code's) in eight chunks, at `-n 4` but the three pack
and build ones, one test each. Measured 2026-10-20
at load 1.5-4, the cache cold for the code but both cradles' t = 1.0 entries: 96, 87, 60,
86 and 84 s (they were 165-260 s before the inner plate's keep-out was fused in 2D and the
cradle box's parts memoized); every test under 65 s. What still fabricates afresh: every
pose, the room, the packs (meshed) and the whole side's claims (20 s each):

```bash
pytest tests/test_cradle.py -m slow -n 4 --dist worksteal -k "clash_free and decanter_rcm"
pytest tests/test_cradle.py -m slow -n 4 --dist worksteal -k "clash_free and decanter_hinge"
pytest tests/test_cradle.py -m slow -n 4 --dist worksteal -k "verify_rates or stands_turns"
R="not clash_free and not verify_rates and not stands_turns"
pytest tests/test_cradle.py -m slow -n 4 --dist worksteal -k "$R and decanter_rcm and not packs_in_one_box"
pytest tests/test_cradle.py -m slow -k "packs_in_one_box and decanter_rcm"
pytest tests/test_cradle.py -m slow -n 4 --dist worksteal -k "$R and not decanter_rcm and not packs_in_one_box and not builds_an_order"
pytest tests/test_cradle.py -m slow -k "packs_in_one_box and decanter_hinge"
pytest tests/test_cradle.py -m slow -k "builds_an_order"
```

(Pack turn 3, 2026-10-21: the fifth had grown past 4 minutes, A's pack test 2 minutes and
A's `build` test, which packs A too (its pack pose fabricated afresh), over 3: each is a
chunk of its own. Pack turn 4, the same day: the fourth ran out at 4 minutes with B's pack
test last, 130 s on its own: that one is a chunk of its own too.)

The clash test is one test per crank angle and the verify test one per cradle (each under
5 min, the checks as before). The identity gate likewise: `snapshot DIR --designs
strider_quad`, then the other seven a few at a time (`palletizer_turret` alone), then `diff
BASELINE DIR`.

A scratch script that fabricates a cradle: `SPIDERPIG_OCCT_THREADS=2` (B's first
fabrication 29 s at 1.4 cores, against 28 s at 1.7 on 3 and 33 s on 1; a second one in the
process 4 s on any; measured 2026-10-20 at load 1.5-3), 1 when several run at once.

## The scorecard, CI, the doc check

`mise run scorecard` (`tests/scorecard.py`) writes `build/scorecard.json` and prints a table:
`spiderpig build --profile` of the Strider double (an empty store, then the same store: the
stages, wall and CPU), each module tier's and the quick tier's wall, CPU and tests (the
quick tier under coverage.py, `COVERAGE_CORE=sysmon`, pytest-cov combining the workers; its
10 slowest tests), pyright's errors (`[tool.pyright]`, basic; OCP and mujoco typed Any by
`typings/`), ruff's `RUF` findings, every module over 800 lines, CLAUDE.md's lines, the doc
check's misses and the load average around each section. `--gate`, `--full` (`--cold`: on
an empty test cache) and `--audit` add the heavy ones; the build runs 3 times (`--runs`),
medians reported; the static checks run alone (`--parallel-static`: beside the tiers);
`--compare A.json B.json` prints changed or non-zero exit codes first, warns when the two
used other xdist workers or ran at loads over 2x apart, then every delta. CPU is each process's `wait4` rusage (its xdist
workers included). The baseline is `docs/agentlib/scorecard-baseline.json`.

CI (`.github/workflows/ci.yml`: pushes to master and every PR, read-only token, a newer
run of a ref cancels the older; `SPIDERPIG_PLAN_SECONDS=3600` for every job, as the gate's:
the node budgets alone bound a search, so a plan doesn't depend on the runner's speed): ruff
(W6's extended rules), pyright's ratchet (`mise run pyright-check`: `[tool.pyright]` covers
`spiderpig/` and `tests/`, each on its own count in `tests/pyright-baseline.json` (the
package's at the top, the tests' under `"tests"`, hundreds when they were added on
2026-10-14), neither of which may rise; `-- --update` lowers them),
`uv lock --check`, the viewer's typecheck + vitest,
the quick tier (`-n 4`, `SPIDERPIG_OFFLINE=1`; node and the viewer's packages installed with
`SPIDERPIG_REQUIRE_VIEWER_TESTS=1`, so a viewer test that would skip fails; the fabrication
cache restored and saved with `actions/cache`, keyed by the engine version, `python -m
spiderpig.tools.engine_version`, plus `uv.lock` and `tests/cache.py`: no restore-keys, so a
new engine starts empty), `lint-imports`, and the doc check with `--strict` (blocking
since W7). `.pre-commit-config.yaml` runs ruff and
`uv lock --check` (opt-in: `uvx pre-commit install`).

**The slow tier on a PR** (opt-in, the `slow` job of `ci.yml`): `-m "slow and not e2e" -n 4`,
set up as the nightly's (node, `SPIDERPIG_REQUIRE_VIEWER_TESTS=1`, the fabrication cache
under its own key, starting from the nightly's or the quick job's entries for the same
engine). It runs when

- the PR carries the label `slow`: adding it starts a run at once (the workflow listens
  for `labeled`), and every push runs it again while the label stays (`gh pr edit N
  --add-label slow`). Removing the label starts nothing (`unlabeled` isn't a trigger): the
  later pushes skip the slow tier, and a run already going goes on until it ends or is
  cancelled (`gh run cancel RUN`). Adding any other label re-runs the cheap jobs too
  (none of them is ever skipped, so a skipped check can't stand in for a failed one), in a
  concurrency group of its own: it never cancels the in-flight run, slow tier and all, nor
  starts the slow tier again;
- or on a manual run of the workflow: Actions > CI > Run workflow on a branch, or
  `gh workflow run ci.yml --ref BRANCH` (every job, the slow tier among them).

It takes about as long as the nightly's (up to 4 h on a cold cache; its timeout). The
browser tests and the gate's determinism check stay nightly-only.

Nightly (`.github/workflows/nightly.yml`: 06:17 UTC on master, and `workflow_dispatch`):
the slow tests (`-m "slow and not e2e" -n 4`, with node and
`SPIDERPIG_REQUIRE_VIEWER_TESTS=1`), the browser tests (`mise run viewer-build`, Playwright's
Chromium, `-m e2e`; `SPIDERPIG_SHOTS` screenshots kept as an artifact) and the identity
gate as a determinism check: CI has no baseline, so it snapshots the tree twice, each in a
fresh store, and `compare` must say identical (and every design pass its audit: a first
snapshot whose audit fails, exit 3, still goes on to the compare, then fails the job).
Each job keeps the fabrication cache under
its own key, starting from the quick job's entries for the same engine.

`mise run doc-check` (`tests/doc_check.py`) resolves every backticked dotted name, path,
task, `spiderpig` command, command-line flag and environment variable in CLAUDE.md,
AGENTS.md, README.md, ARCHITECTURE.md, API.md, TESTING.md and ROADMAP.md
statically against the package's AST (identifiers and key-like strings; docstrings,
comments and prose strings name nothing; no engine import); `-v` lists every check,
`--strict` fails on a miss (CI runs it so, and `tests/test_doc_check.py` checks the default
docs have none). `tests/doc_check_allow.txt` holds what is legitimately not code (output file
names, protocol fields, the roadmap's records of removed names), never drift. The dated
records under `docs/history/` are never checked (the check skips that folder): each is headed by its
date and status, and names the code as it was.

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
A test not marked `slow` that takes over 5 s (setup + call + teardown;
`SPIDERPIG_SLOW_WARN_S`) is listed at the end of the run, a warning only. The roadmap
asked for a failure when a test is over the budget twice in a row; it stays warn-only by
decision (W4a review): on the shared, loaded box every test slows down, and a timing
failure would be noise. Make it fast through a seam or the cache, keep a cheap case quick, or mark it
`slow` with a reason in the commit.

**Seam tests** (`tests/test_seam_*.py`, W4a): a construction's or the planner's rule on
inputs the test states, built with `tests/_ctx.py` (`topology()`, `context()`, `layout()`,
`link_claim()`, `disc_claim()`: a few links, an axle, a crank point in a few lines), each
in milliseconds. They are marked `no_fabricate`: the conftest swaps
`spiderpig.fabricate.fabricate` / `fabricate_side`'s code for a refusal while one runs, so a
fabrication fails the test whatever name it was imported under. `tests/brute.py` also takes
a hand-built problem with no router (the first layering that plans), for small
`StackProblem`s.

**The walking model's foot z**: a session fixture seeds each default design's plan (and its
leg hint's single module) from the test cache before `walk._default_plan_z`, so a walker
re-makes and verifies the plan instead of searching (`tests._linkage.seed_default_plan`);
the `foot_z` fixture is generated under `tests._linkage.node_budget` (no clock, 4000 search
steps), so it doesn't depend on the machine's load.

## The fabrication cache (`tests/cache.py`)

```python
from tests import cache

cache.CACHE_ROOT        # $SPIDERPIG_TEST_CACHE, else $XDG_CACHE_HOME/spiderpig/test-cache (~/.cache/...)
cache.CACHE_DIR         # CACHE_ROOT / "<engine_version>-<env tag>"
cache.cached_design(cfg) -> (tmpl, SideDesign)               # design_side, plan seeded from disk
cache.cached_side(cfg, t=1.0, *, fresh=False) -> Mechanism   # fabricate_side, BREP-cached
cache.cached_robot(cfg, t=1.0, *, fresh=False) -> Mechanism  # fabricate (robot=True), BREP-cached
cache.cached_product(cfg, t=1.0, *, fresh=False) -> Mechanism  # fabricate (robot=False): a cradle
cache.own_product(cfg, t=1.0) -> Mechanism   # cached_product's entry read again: the caller's own
cache.seed_plan(cfg) -> bool       # seed fabricate._LAYOUTS only (a later design_side re-makes it)
cache.prebuilt_store(cfg, tmp_path, t=1.0) -> Store          # a copy of a resolved+planned+built store
cache.recorded(module, name, make) -> data                   # tests/fixtures/<module>/<name>.json
cache.assert_current(module, name, make) -> data             # that fixture's currency test
```

- `cfg` is a `spiderpig.config.BuildConfig`; `robot` is ignored by `cached_design` /
  `cached_side` (always the side) and forced by `cached_robot` (a mechanism has no robot:
  `ParamError`). The template is `fabricate.template_for(cfg)`. `cached_product` is what
  `fabricate` makes with `robot` off: a decanting cradle whole (both sides, the box, the
  stand), 25-35 s fresh (4-6 s at another crank angle in the same process: the box's
  parts and the plates are memoized there), a tenth of a second loaded;
  `tests/test_cradle.py` takes its fabrications there but the one it meshes (the flat
  pack). The verify and build tests take theirs through `cached_cradle_fabrications` (the
  api's `fabricate_at`, `spiderpig build`'s and `cradle_report`'s `fabricate`), each from
  `own_product`: a mechanism of the caller's own, as the store's fabrication cache hands
  one out (`spiderpig build` adds its sheet lines to `bom_extras`).
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
- **Keying** (W3b, incremental): each layer is keyed by the code it can reach
  (`spiderpig/keys.py`: the closure of its roots through the import graph, symbol by
  symbol, lazy imports and the linkage registry's auto-import included, docstrings
  stripped; `python -m spiderpig.keys --why MODULE:NAME` says how a symbol is reached),
  plus a hash of this cache's `FORMAT`, the Python, build123d, OCP and numpy versions:
  plans in `plans/<keys.plan_key()>-<tag>/<cfg.key>.json`, fabrications in
  `fab/<fabcache.folder_name()>-<tag>/<cfg.key>_<side|robot>_t<t>/` (the plan's code plus
  every `realize`, the robot, the format), prebuilt stores still in
  the engine version's folder (`stores/<cfg.key>_t<t>/`: a store's ids name the engine). An
  edit is a new folder only for the layers that reach it: a deck colour keeps every plan,
  a planner edit re-plans; nothing is invalidated in place. `tests/test_keys.py` checks
  both directions on real edits. The keys themselves are kept under
  `SPIDERPIG_DIGEST_CACHE` by the sources' stats; after an edit each process walks the
  closures again, but reads every unedited source's index and stripped code back from
  the keys-index folder beside them, keyed by the source's bytes (`keys._load_index`,
  `keys.code_text`): seconds, not the ~10 CPU-s of parsing everything; and one process
  computes a key while the others that want it (the xdist workers start together) wait
  under its lock and read it (`keys._kept`); under xdist the controller starts
  `tests.cache.warm_keys` beside the workers, so the keys are made while they collect.
  Before, every worker's first test paid them all at once: 15-25 s in the suite for a
  test that takes 1-2 s alone. The fabrication
  format, its locks and atomic writes are the product's (`spiderpig/fabcache.py`, below).
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

## The product's fabrication cache (`spiderpig/fabcache.py`)

`spiderpig build` and `api.build` of a stored design serve `fabricate()` from
`<store>/fab/<fab key>-<env tag>/<cfg.key>_<side|robot>_t<t>_<hash>/` (the hash: the
template and config, the crank angle, the plan itself, the servo model in play) and
write it there after a cold fabrication (one `flock` per entry, atomic rename). The test
suite turns it off (`SPIDERPIG_FAB_CACHE=off`, the `_offline` session fixture): tests use
`tests/cache.py`, and `fresh=True` builds stay fresh. `Store.gc` removes other keys'
folders. Fidelity (`tests/test_fabcache.py`, slow): on the gate's designs a loaded
fabrication equals a fresh one on the gate's part table, the BOM and every DXF entity.
`spiderpig build` into a folder that already holds that very build does nothing
(`spiderpig/uptodate.py`; `--force` builds).

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
- **Currency key** (W3b): a fixture written since carries `source` (its generator's file
  and function) and `source_key` (`keys.function_key`: the code the generator reaches);
  it is stale only when that code changed (`cache.is_stale`), so an engine edit elsewhere
  no longer flags it. A fixture without them compares engine versions, as before (the
  next `mise run test-fixtures` stamps them).
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

## Judging a cradle's packing (`spiderpig pack-inspect`)

`mise run pack-inspect -- decanter_rcm` (`spiderpig/tools/pack_inspect.py`) packs a
decanting cradle by its plan (`cradle_pack.PACK_PLANS`) and with none (`--plan default
auto`, the default), or judges a saved layout (`--layout pack.json`: a
`/api/cradle/{key}/pack` body, or a `*.layout.json` it wrote), and writes per candidate a
contact sheet (`build/pack/<linkage>/<name>.png`: four views, three slices, the legend, the
pads' cut list), the metrics (`<name>.json`: `voxpack.metrics`) and the layout. The first
run fabricates and tessellates the cradle (B 47 s, A 37 s; the whole default run 70 s cold,
measured 2026-10-20); the input is then kept in the store's `pack/`
(`cradle_pack.cached_input`), and a run is seconds.

Read the metrics, not only the picture:

- `missing`, `missing_parts`, `warnings`: a plan's step with no room (an `into` or `on`
  that doesn't fit) stops the packer with the pieces placed so far (`voxpack.Layout`'s
  `missing`). The sheet's status line is then red, naming the missing pieces and why, and
  the command exits 1 (`pack_inspect.complete`). Exit 0 means every candidate holds every
  piece and part; a red line with exit 0 is a verdict (unsupported, overlaps, order faults,
  a pad with no room), not a broken pack.
- `spacers` (since pack turn 2, 2026-10-20): the spacers the payload carries
  (`voxpack.spacers`, the layout's `spacers`, each judged as a piece of its own at its
  place in the order: `cradle_pack.in_order`; one `taped` to the piece it stands on is
  judged with that piece, `cradle_pack.judged_pieces`). Each is a block of the judge's
  empty 3 mm cells, one run a column, never within a cell of a part nothing bears on
  (`cradle_pack.KEEP_CLEAR`: a gauge, a clip, a pin, the gearbox, a screw head; pack turn
  3): a *support* where a piece's pads were (from what is under it to the piece's
  underside, its lips round both), a *restraint* beside a piece that would still slide (in
  the gap that way, standing on what is under it), a *cap* on one that would lift
  (`restraint +y`), a corrugated *filler* at the board's ends (`filler -x` ..: cut from a
  spare carton, not printed), each grown until it doesn't slide itself; two of one piece
  that face each other are joined, and one that still slides is taped to what it stands
  on; `count`, `volume_cm3` (the solid) and `grams` (as printed, PETG:
  `cradle_pack.spacer_grams`, walls and 15 % gyroid; a filler's corrugated), and per
  spacer its `key` ("spacer_01" .., in the order they go in), `label`, `size_mm`, `why`,
  what it stands `on`, `taped` and its `words`. The build prints the printed ones
  (`print/pack_spacer_*.stl`, PETG, each laid with the least overhang, its number and an
  arrow engraved in its top: `cradle_pack.spacer_print`), the BOM lists them, and the
  guide places them (B's "Into its box" steps, a picture of the pack each).
- `pads`: what the spacers replace, on a layout judged without them (a payload's modules
  carry them). `voxpack.support` puts a pad at each corner of what a piece rests on where a
  gap of up to `PAD_MAX` needs one; `voxpack.measure_pads` measures each on the real
  meshes: a rectangular block, its footprint cut to the piece over it, `height_mm` the real
  gap over that footprint (`voxel_mm` the voxels' word beside it), what it is `under` and
  `on`, an id by the piece's number in the order (`4a`), `fits` false where no block has
  room. `count`, `volume_cm3` and `tallest_mm` total them; the sheet draws them black. With
  the spacers in, a piece rests on them and has none.
- `restraint`, and per piece `free_mm`, `meets`, `slide_mm`: how far each piece slides
  along x and z and lifts before it meets a neighbour or the box's wall or lid
  (`voxpack.free_travel`, on the 3 mm cells: the real travel is up to two cells more, a
  spacer's one; up counts since pack turn 3). `loose` lists the pieces (and spacers) that
  move more than `CLEAR` (3 mm, one cell) any way. B's is empty.
- per piece `rests` and `on`, judged with `bearless` (`cradle_pack.keep_clear_cells`):
  a piece doesn't rest where its bottom, or what is under it, is a part nothing bears on
  (pack turn 3, packing review 2's B1: the drive panel had rested 78 % on its gearbox).
- `search` (with `--search`): `--seeds N` anneals N seeds (`--workers` at a time, each
  `--seconds`: 60 by default, less when the seeds' rounds would take the search past
  `pack_inspect.SEARCH_BUDGET_S` in all), keeps the best and reports every run and the
  spread across them (`voxpack.spread`: the best, median and worst score and box litres,
  how many seeds came within half a litre of the best). One seed's box is a sample: quote
  the spread with it.

The search's score (`voxpack.score`) charges a layout's pads by count, volume and the
tallest (`voxpack.pad_cost`), so of two layouts in one box the one on fewer, smaller pads
wins. `--split KEY=GLOB` packs a what-if (parts off their piece as a piece of their own):
every piece keeps the orientation it had whole, the new one its parent's (the piece most
of its mass came from), so a plan's `down` names the same face; `--relevel KEY` levels a
piece afresh.

Its tests are the synthetic ones in `tests/test_cradle.py` (voxels, the drop, support and
pads, the pads' blocks, the spacers in their place, the guide's box steps, the free
travel, the score, the search and its seeds, a split, a plan that stops, the sheet: no
fabrication) and the slow `test_the_cradle_packs_in_one_box`, which judges each cradle's
flat pack on its real shapes with its spacers (every piece placed, every pad a block that
fits and a spacer's, nothing past `PACK_SLIDE`, up too). Its packing (`pack_input`, warm)
is about 27 s (B) and 32 s (A) with its spacers (20 and 35 s before pack turn 3's
keep-clear, caps, bridges and taping; 46 and 63 s with them before `voxpack._gap_field`);
A's test about 2 minutes in all.

## The identity gate (`tests/gate/identity_gate.py`)

A script, not a test: run it before and after any change under `spiderpig/` that must
not change a design.

```bash
mise run gate -- snapshot /tmp/gate/before      # on the base (master)
mise run gate -- compare /tmp/gate/before       # on the branch: snapshots again, diffs
mise run gate -- diff A B                       # two snapshots
mise run gate -- compare DIR --designs klann_quad -j 1
mise run gate -- doc DIR                        # docs/agentlib/DESIGNS.md from a snapshot
mise run gate -- audit DIR                      # every design of a snapshot passes its audit
```

`doc` writes `docs/agentlib/DESIGNS.md`, the one source of the designs' numbers (layers,
height, constructions, sheets, part counts, the gate's audit verdict, BOM cost): run it on
each new baseline and commit the result; `--out -` prints it.

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

Each design's contract angles and its `t=1` half (that fabrication's clashes, solids and
parts, and the build of it) run in worker processes of their own (`GATE_SPLIT`, default
`contract:0,1.6,3.2,4.8|build`; empty: one process), each taking the plan from the
design's store; unless given, `-j` and the split follow the free cores (`plan_cores`: the
cores less the load average). OCCT runs two threads per process, as the baseline did
(`GATE_OCCT_THREADS`; one thread moves a cut-rule number). The STEP file, never read, isn't written.

**The contract, fast and exact.** The audit (and so the gate) checks the contract with
`check_sides`: the side realized at the first angle, every group that declares how it
moves (`Group.motion`) carried to the others, and the declarations checked once more at a
guard angle no caller asks for (`GUARD_TURN`; a mismatch is a `DeclaredMotionWarning` and
that group is checked exactly at every angle). Its limit: a part, or a
`ConstructionError`, that a construction makes at one of the other checked angles alone,
and at neither the first nor the guard's, isn't seen. `GATE_EXACT_CONTRACT=1` checks with
`check_side` at every angle instead; the nightly workflow snapshots once each way and
compares, so the two are held equal on the gate's designs every night.

Verdicts per design, and the exit status: **identical** (0); **geometry identical, order
differs** (1: a DXF's entities in another order or a closed outline from another start
vertex, bodies reordered, a text file's lines reordered, a part's faces/edges counted
otherwise at the same volume, area and box, an STL mesh with the B-rep unchanged) — a
user decision, not a merge; **DIFFERENT** (2, every difference listed). Numbers compare
within 1e-9 relative / 1e-6 mm. Then every design's own audit (2026-10-14): a design whose
audit has a problem is listed (`FAILS ITS AUDIT`) and the exit status is 3 where it would
have been 0, unless `KNOWN_AUDIT_FAILURES` says why it fails (the demo `klann` quad's long
pins, the user's call: a known failure is listed, and one that passes is reported for the
table to lose it); with an identity difference too, `compare` exits 4 (order differs) or
5 (DIFFERENT), so neither verdict hides the other (`identity_gate.exit_status`); `mise run gate -- audit DIR` checks a snapshot alone, and the slow
`tests/test_gate_audits.py` runs the eight audits (a known failure a strict `xfail`).

**Current baseline** (the decanting cradles merged onto the platform, 2026-10-21):
`~/.cache/spiderpig/gate/merge-b52b88c/`, from which DESIGNS.md is generated; compare against
it: `mise run gate -- compare ~/.cache/spiderpig/gate/merge-b52b88c`. Its walkers and
mechanisms are the platform's (`platform-814096d`, below) planned by the cradle branch's
planner of 2026-10-14: each plan the cradle branch's (`planner-f667d6b`) layer for layer,
the parts master's on it (every such diff's source in `PLANNER-gate-diffs.md`); its two
arms planned anew by that planner (other layerings, their hubs and drives' rules made whole
on the way: DECISIONS.md 2026-10-21); the cradles the cradle branch's.

**The cradle branch's baseline before the merge** (`planner-f667d6b`, 2026-10-14: the
planner's lowest stack merged with the cradles' design review round 1): its walkers differ
from the integration branch's `next-320384d` (2026-10-08) by the planner's changes of
2026-10-14 alone, every diff with its source in `PLANNER-gate-diffs.md` (the planner's
first, `planner-9da0548`, by the price of the 15 mm hex standoff alone).

**Master's baseline before the merge** (the merged walkers-mechanisms-and-arm engine after
review round 2, 2026-10-14): `~/.cache/spiderpig/gate/platform-814096d/`. It differs from `platform-9aae3e3` (kept) by the
two mechanisms' BOMs alone (their controller unnamed is `none`: the kit's lines gone, the
torque-limit note added). `platform-9aae3e3` differs from `platform-78e35e8` (kept) in BOMs
and audits only, no part, plan or DXF moved: the cutting model's cut-length term (every
design), the walkers' foot rows, a mechanism's controller kit and its jam rating at its
torque limit, the arms' MJCF out of the URDF folder. `platform-78e35e8` differs from
`platform-5396098` (kept) by
`palletizer_turret` alone: the approved arm's pivots and links' sheets, its audit passing
(DECISIONS.md 2026-10-14, wave 2); the other seven identical.

**Earlier baseline** (the merged walkers-and-arm platform, `77c117c`, 2026-10-14: eight designs, the
user's approved arm `palletizer_reach` among them): `~/.cache/spiderpig/gate/platform-77c117c/`
on the development box (`/root/.cache/...`), one `<design>.json` per design, their logs and
`snapshot.json` (commit, branch, time; its walkers snapshotted at `982e67b`, which `77c117c`
leaves alone). Compare against it from any worktree whose `spiderpig/` should match its
output (historically): `mise run gate -- compare ~/.cache/spiderpig/gate/platform-77c117c`. It differs from
`platform-379ad7e` (kept) by every walker's spacers (stock tubes and shims, no printed
ones) and honest BOM prices, `palletizer_turret` on the `mt4_compact` turret (the pruning),
and `palletizer_reach`'s cut-rule fixes and `b4`'s hubs (DECISIONS.md 2026-10-14); from
`platform-867ae7e` (kept) further by `palletizer_reach` gated. That one differs
from `platform-7112cca` (kept) by the palletizer's tool holder (two cut plates, no print)
and the controller's rows in its BOM (the part-count loops, 2026-10-13), the walkers
identical; from `platform-a1d6e49` (kept) by the BOMs' prices and sources alone (the cost loop's
round 1: the catalog repriced, the cut model refitted; parts, plans and DXFs identical),
from `platform-68f96d5` (kept) by the palletizer a layer thinner (the belt's small pulley's
hub in the frame plate's window), from `platform-43363f3` (kept) by the palletizer's bearings priced at VXB, from
`platform-0b05b24` (kept) by the bearings' prices in the palletizer's BOM and the Klann
audits' strength fixes (the ground pin listed as a stronger pin), from `platform-ab69ece` by the arm's pivots alone (`ball` / `ball_pillar` on 6.6 mm
links: 11 bearings, wider links; DECISIONS.md 2026-10-12), from `next-320384d`
(2026-10-08) by the arm alone, and from W8's baseline `w8-2a130c8` (engine `0.1.0+e91ed4bc0da8`; kept, as are the branches'
own `bus-39502f7`, `bus2-92d11e4`, `bom-bbf7001`, `bom2-5989d50`, the first
combined `next-ba41c10` and `next-ba68611` (before the boxed cables and the
congruent centre plates) and `master-1eba876` /
`master-d53fbf1`) by the bus-socket change, the user's BOM decisions of 2026-10-08 and the
guide's labels: every diff with its source in `NEXT-gate-diffs.md` (which links
`BUS-gate-diffs.md` and `BOM-gate-diffs.md`); W8's own from its predecessors in
`W8-gate-diffs.md`.

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

The identity gate: ~5 min wall for the walkers' six designs at once (the Strider quad 290 s, the
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

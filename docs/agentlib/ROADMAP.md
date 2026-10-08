# Roadmap: modular, testable, fast (2026-10-07; status 2026-10-08)

The cleanup plan from the repo audit of 2026-10-07. Eight workstreams (W0-W7), each
with a goal, its scope, **acceptance criteria that a command can check**, and the angles
its adversarial review attacks. A workstream is done when its criteria pass and a review
round finds nothing (the loop, below). The scorecard (W0) measures every number here, so
"faster" and "smaller" are never a matter of opinion.

## Baseline (measured 2026-10-07, 20-core dev box, warm caches)

| metric | now | source |
|---|---:|---|
| `spiderpig build` (Strider double robot), wall / CPU | 32 s / 100 CPU-s | `/tmp/prof/stages.py` (in-process) |
| of which: import / template / plan (stored) | 2.8 / 0.2 / 0.3 s | |
| fabricate robot | 12.6 s | |
| `group_made` (laser + printed grouping: mass props) | 7.5 s | |
| STEP / STL / print STLs / DXF sheets / DXF parts / BOM | 2.2 / 1.8 / 0.9 / 2.2 / 0.9 / 0.02 s | |
| cold plan solve (Strider double) | ~2.5 s | cProfile |
| quick tier, `-n 4` | 102-209 s | audit run; TESTING.md |
| module tiers | 7-33 s each | TESTING.md |
| full suite, `-n 12` | 580 s / 6150 CPU-s | TESTING.md |
| first full run after any engine edit (cold test cache) | 689 s | TESTING.md |
| identity gate, six designs | ~5 min | TESTING.md |
| `audit` per module | 75-120 s | CLAUDE.md |
| largest modules | crank.py 3315, stack.py 2410, api.py 2207 lines | `wc -l` |
| pyright (basic, no config) | 739 errors | audit |
| ruff `--extend-select RUF,...` | 160 findings | audit |
| CI | none | |
| CLAUDE.md | 673 lines / 9.3k words | |

## Status (2026-10-08)

Every workstream is merged; the whole plan landed on `master` as PR #28 (`155c8ab`).

| workstream | state | merged |
|---|---|---|
| W0 measuring stick, CI, doc check | done | `1c533e4` |
| W1 server / MCP / store / viewer correctness | done | `65df0a8` |
| W2 prune the legacy constructions (D1) | done | `39f6c21` |
| W3a output-identical speed wins, parallel gate | done | `9b87918` |
| W3b fabrication cache, build skip, incremental keys | done | `f11da3f` |
| W4a seam unit tests, `no_fabricate`, seeded plans | done | `7d56d1b` |
| W5 packages, one screw table, one shim loop, layers | done | `8bfc039` |
| W6a viewer dependencies, Node 22, declared Python deps | done | `c6939bc` |
| W6b pyright baseline, extended ruff, rationales for the broad excepts | done | `b4d03c5` |
| W7 docs that stay true | done | `c75641b` |
| W8 approved output changes (D2-D5), gate baseline `w8-2a130c8` | done | `07f7e9a` |

**Measured outcomes.** `mise run scorecard` on 2026-10-08 at `8bfc039` (W5 merged),
against the baseline `docs/agentlib/scorecard-baseline.json` (2026-10-07, `41997b4`). Both on the 20-core development box shared with other agents' jobs: the
baseline ran at load 15-30, this one at 6 (build) rising to 20 (tiers, quick tier), so the
tier and quick-tier wall times compare only roughly. The full scorecard is kept with the W7
PR, not committed. The pyright, ruff and CLAUDE.md rows are as merged (`155c8ab`).

| metric | baseline | 2026-10-08 | target |
|---|---:|---:|---|
| `spiderpig build`, empty store: wall / CPU | 38.5 s / 112 CPU-s | 29.3 s / 88 CPU-s | W3: ≤ 12 s, not met |
| `spiderpig build` again, same design | 37.9 s | 0.08 s (the build skip) | W3: ≤ 4 s, met |
| `fabricate` / `group` stages (cold) | 13.2 / 7.6 s | 10.4 / 4.7 s | W3: ≤ 6 / ≤ 1 s, not met |
| quick tier, `-n 4`: wall / CPU / tests | 145 s / 483 CPU-s / 1330 | 155 s / 524 CPU-s / 1705 | W4: ≤ 60 s, not met (375 more tests, load 20) |
| quick-tier coverage of `spiderpig/` | 79.2 % | 85.7 % | W4: + 5 points, met |
| module tiers, wall | 12-61 s | 16-71 s (load 12-20) | W4: ≤ 20 s, not met under load |
| largest module | 3315 lines (`crank.py`) | 1145 (`keys.py`) | W5: ≤ 1200, met |
| pyright errors (without stub noise) | 733 (619) | 28, ratcheted in CI (21 since, "Next") | W6b: ≤ 150, met |
| ruff `RUF` findings | 117 | 0 (with `C4`, `PERF`, `PIE`, `RET`, `BLE`, `PLE`) | W6b: 0 with the extended set, met |
| CLAUDE.md lines | 679 | 261 | W7: ≤ 300, met |
| doc check misses | 48 | 0 (`--strict` in CI) | W7: 0, met |
| identity gate, six designs | ~5 min | 225 s (the `w8-2a130c8` snapshot) | W3: ≤ 2 min, not met |

The W3 and W4 wall-time targets were set on an idle machine; none was re-measured idle
(the box was never under load 4, invariant 4), so "not met" above means "not shown".
The one failing test of that run (counted in the api tier and the quick tier) was W7's
own check of the docs mid-edit (`test_doc_check.py`), green once W7's edits landed.

**What's left.**
- Re-measure W3's and W4's wall-time targets on an idle machine; if still missed, the
  next steps are W3 items 3-4 (each distinct part built once; exports as workers) and W4
  item 4 (one shared warm-up for the quick tier).
- The remote runner (`mise run remote-test`): `root@ao-server` is the development box
  itself and had no SSH server on 2026-10-08; the full suite ran locally instead.
- The demo `klann` quad's audit still fails on jam SF (W8, unchanged; the wobbly demo).
- The open product items under "Later", below.

## Next (started 2026-10-08)

The follow-ups after PR #28, each its own branch:

- **Assembly guide**: a PDF pipeline from the design (the assembly order, each step's
  parts and fasteners, drawn from the build), a prototype first.
- **STS3215 bus plugs**: done 2026-10-08 (top-entry sockets in the rear face, one per
  servo, each on its own boxed cable to one of the driver board's two bus ports, both rear
  screws kept: DECISIONS.md). Hand measurements remain: the plug-plus-wire height above the
  rear face (`ServoSpec.bus_ports`, expected 5.5-6.5) and whether the boxed cable reaches
  the board.
- **BOM consolidation**: a study of the BOM: fewer fastener sizes and SKUs, less pack
  waste, cheaper sources.
- **Speed round 2**: the contract check, the BOM's congruence grouping, the build's
  exports, the tests' start-up and `spiderpig/keys.py` (W3's and W4's targets, still not
  shown).
- **The quickfix round** (this list's own branch, 2026-10-08):
  - the stale material out: the generated architecture page and its broken generator,
    SCOPE.md moved to `docs/history/`, this status table;
  - pyright's possible bugs: the booleans' build123d ShapeList (`shapes.difference` /
    `intersection`: a part its holes split is a `Compound` that `bad_solids` reports, not
    a list the next boolean fails on, and `spiderpig build` stops on it,
    `fabricate.split_parts`; no design reaches it) and `chassis._port_slots`'
    possibly-`None` ports (unreachable: the XL servos have no `bus_ports`); pyright 28 -> 21;
  - a nightly workflow (`.github/workflows/nightly.yml`): the slow tests, the browser tests,
    the identity gate compared with itself; watch its first runs (the browser tests'
    timed drive on a runner with no GPU, the gate's two snapshots on 4 cores);
  - `lint-imports` with no exceptions: resolve, the stage records, check and plan moved
    under bake/build into `spiderpig/stages/`.
- **pyright's last 21**: JSON-typed returns in `spiderpig/mcp/` and `api/` and attribute
  reads on `object`-typed parts (`build.py`, `bake.py`, `deck.py`, `store.py`): typing work,
  none a known bug.

## Invariants (every workstream, every round)

1. **Identity gate `identical`** on the six designs (`mise run gate -- compare <baseline>`),
   unless the workstream's criteria name an intended difference; then the diff is listed in
   the PR and accepted by the user, and the baseline is re-snapshotted.
2. Full suite green (`mise run remote-test`), `mise run lint` clean, `test-viewer` green.
3. **No gaming**: no test deleted, skipped or marked `slow` without a line in the PR saying
   why (W2's pruned-construction tests are listed by name); no tolerance loosened; no
   timeout or node budget raised to make a test pass; the test count only drops by the
   listed deletions.
4. Timings are the median of 3 runs, on the same machine, with load average under 4; the
   scorecard records the load.
5. Every workstream lands as its own PR (or a short stack), branched from `master`.

## The loop (how each workstream is checked)

```
implement -> self-check -> review round -> fix -> review round -> ... -> done
```

1. **Self-check.** The implementer runs the workstream's criteria commands and
   `mise run scorecard`, and puts both outputs (and the gate's verdict) in the PR body.
2. **Review round**: two reviewers, each in a fresh context, given only this document's
   section for the workstream, the diff and the self-check output:
   - **Correctness reviewer**: `/code-review high` on the diff. It reports bugs with a failure
     scenario.
   - **Red-team reviewer**: tries to *falsify each criterion*. It re-runs the commands,
     looks for the gaming in invariant 3, and attacks the angles the workstream lists. Every
     finding needs a reproducing command or a file:line.
3. A finding is **CONFIRMED** when a third pass reproduces it. CONFIRMED findings are
   fixed and the round repeats; PLAUSIBLE ones are answered in the PR.
4. **Exit**: a round with zero CONFIRMED findings and every criterion green. After 4
   rounds without that, stop and bring the open findings to the user.

## Order and parallelism

```
W0 ─┬─ W1 ──────────────────────────┐
    ├─ W2 (prune) ── W5 (modularity) ┼── W6 (static quality) ── W7 (docs)
    ├─ W3 (build speed) ─────────────┤
    └─ W4 (test speed) ──────────────┘
```

W0 comes first: it builds the measuring stick. W1, W2, W3 and W4 then run in parallel
worktrees; they touch disjoint code (server and MCP; constructions; fabricate, store and
export; tests). W5 splits the big modules once W2 has made them smaller. W6 ratchets
quality on the final layout, and W7 documents the result.

---

## W0: Measuring stick and guardrails

**Goal.** Every number in this document is one command away, and CI runs the cheap checks
on every push.

**Scope.**
- `spiderpig build --profile`: stage timings like the bake's `_Profiler`, through the
  `bake_gltf`-style logger; the stage keys are `import`, `template`, `plan`,
  `fabricate`, `group`, `step`, `stl`, `prints`, `dxf_sheets`, `dxf_parts`, `bom`,
  `order`.
- `mise run scorecard` (`tests/scorecard.py`): writes `build/scorecard.json` and prints a
  table of:
  - the build stage times (Strider double)
  - each module tier's wall and CPU times, the quick tier's, and test counts per tier
  - the gate's wall time (opt-in `--gate`)
  - the size of each module over 800 lines
  - the pyright error count and the ruff `RUF` count
  - CLAUDE.md lines
  - coverage % of `spiderpig/` under the quick tier (`coverage.py`)

  `scorecard --compare A.json B.json` prints the deltas.
- CI (`.github/workflows/ci.yml`):
  - `ruff`, `uv lock --check`, `test-viewer` and the quick tier
  - the fabrication cache restored and saved with `actions/cache`, keyed by `engine_version`
  - the doc check (W7's script, wired in now in report-only mode)
- A pre-commit config: ruff and `uv lock --check`.

**Criteria.**
- [ ] `spiderpig build --profile` prints every stage key above; their sum is within 5 % of
      the wall time.
- [ ] `mise run scorecard` finishes in ≤ 6 min (gate excluded) and writes a JSON document
      with every metric in the baseline table.
- [ ] The baseline scorecard is committed as `docs/agentlib/scorecard-baseline.json`.
- [ ] CI is green on a PR. A deliberately broken PR (a ruff error, a failing quick test)
      is red; the red-team reviewer opens it.
- [ ] CI's quick tier on a warm cache: ≤ 10 min.

**Red-team angles.** Do the scorecard numbers match a manual `time`? Does the cache key in
CI let a stale cache pass? Does the profiler change the build's outputs (the gate)?

---

## W1: Correctness fixes (server, MCP, viewer)

**Goal.** The bugs from the audit are fixed, each with a regression test that fails before
the fix.

**Scope.**
1. `server/app.py` `_config_from_query`: a stored design switched to another linkage keeps
   `crank` / `crank_sheet` only when they aren't the old linkage's defaults
   (`config.default_crank` / `default_crank_sheet`). The same holds on a module change.
2. MCP: a per-design lock (`fcntl.flock` on `designs/<id>/.lock`) around
   `run_op`'s writes, `write_build` and `Store.remove`/`gc`. `Jobs.submit` reuses a running
   job for the same `(op, design, args)`.
3. MCP `_jobs`: evict finished jobs past 64 (or 1 h), keeping id and state; the store
   holds the reports.
4. The local server: `TrustedHostMiddleware` (localhost, 127.0.0.1, `VITE_ALLOWED_HOSTS`),
   and a WebSocket `Origin` check on `/ws` and `/ws/sim`.
5. `_FAILED` pruned with `_prune()`. `_rebake_all`'s mutations of loop-owned state go
   through `loop.call_soon_threadsafe`.
6. MCP `view`: a lock around starting the child; an `atexit` and SIGTERM stop.
7. Viewer: a generation counter in `loadMode` (as in `physics.ts`); an `AbortSignal` on
   `loadGlb`; the server skips a queued bake whose client has gone.
8. `store.py` id checks use `.fullmatch`.
9. `test-viewer`'s walk-model parity runs in `test-quick` and `remote-test`, as a pytest
   that runs `npm test` when `node_modules` exists and skips with a reason otherwise. A
   Strider double reference fixture is added beside the Klann quad's.

**Criteria.**
- [ ] One regression test per item. Each fails on `master` (shown in the PR by running it
      there) and passes on the branch.
- [ ] Two concurrent MCP `build` jobs of one design leave one complete, consistent build
      (the test runs them through the real pool).
- [ ] `curl -H 'Host: evil.example' :PORT/api/linkages` gets a 400; a WebSocket with a
      foreign `Origin` is refused; the viewer through Vite and `spiderpig view` still work
      (the e2e tests).
- [ ] The gate is `identical` (no part changes).

**Red-team angles.** Races the lock misses (gc against export; the in-process `_ENGINE`
path against a worker). Does the Host check break `tailscale serve` with
`VITE_ALLOWED_HOSTS`? Does an evicted job's id still answer sensibly? Does the parity test
silently skip in CI?

---

## W2: Prune the legacy constructions

**Goal.** Only the constructions a design defaults to remain. Each removed key fails
clearly. No coverage of the kept code is lost.

**Keep:**
- **crank**: `bolt` (hex, the default), and `bolt_round` (TrotBot's heel and toe,
  `LINKAGE_CRANKS`)
- **pin**: `chicago`
- **pillar**: `standoff` (one-piece)

**Remove:**
- **crank**:
  - `keyed`, `keyed_float` and `printed` (`PrintedCrank`, `KeyedCrank`, `_Stack`,
    `PostJoint`, `HornJoint`, `KeyedJoint`)
  - `bolt_hub_screw` and `bolt_unretained`
  - the acrylic two-plate crank: `_BoltPlates`' M6 path, the `two_layer_*` / `tip` /
    `low_count` / `share_stack` router rules, and `BoltCrank`'s M6 methods
- **pin**: `rod`, `ptfe`, `bolt`, `bearing`, `bushing`, `chicago_bushing` and the
  printed snap pin
- **pillar**:
  - `printed` (`PrintedAxle`, `construction/printed.py`)
  - `bolt`
  - the spliced `standoff_hand`, `standoff_bench` and `standoff_m3`, with their splice
    code in `standoff.py`
- **recommendation**: `recommend.printed_pillars`

**Decision D1 (the user's, 2026-10-07):** the keep set above. The printed pin and pillar
go too (`recommend.printed_pillars` with them).

**Scope.**
- Lift what `_WebPlates` and `BoltCrank` share out of `_BoltPlates` and `PrintedCrank`
  first (the shell: `buy`, `cut`, `stub`, `finish`; `_pitch_error`; the `JointRules`
  construction from `post_joint`); only then delete.
- A removed key (`BuildConfig(crank="keyed")`, a stored design or a spec naming one)
  raises `ParamError`, and the MCP / API return a `Failure`. Both name the replacement
  and the date the key was removed; `config.REMOVED_CONSTRUCTIONS` holds the table.
- Tests:
  - Re-point every test that used a removed construction only as a cheap design (17
    files `crank="keyed"`, 16 `pillar="printed"`) to the defaults, on the cache.
  - Delete the tests of the removed constructions themselves, listed in the PR.
  - Re-record `tests/_linkage.py`'s `REFERENCE_CONFIG` (the walk reference, now
    `crank="printed", pillar="printed"`) on the defaults, and the viewer's
    `walk_reference.json` with it.
  - Delete the planner fixtures `klann-*-keyed` / `*-printed-pivots`, or re-point them.
- Remove the BOM and catalog items only the removed constructions bought (brass key
  standoffs, M6 bolts, rod and clips, PTFE tube, MF63ZZ, igus) from `parts.py`,
  `sources.py` and `fastener_catalog.py`.

**Criteria.**
- [ ] `construction.CRANKS` == {`bolt`, `bolt_round`}; `AXLES` == {`standoff`, `chicago`}
      (D1).
- [ ] `grep -rn "KeyedCrank\|PrintedCrank\|_BoltPlates\|PrintedAxle\|RodAxle\|keyed_float\|standoff_hand"
      spiderpig tests` finds nothing but the removal table.
- [ ] Product lines down by ≥ 1,500 (scorecard), with no module growing.
- [ ] Coverage % of the *kept* modules under the full suite ≥ baseline (coverage.py, the
      same modules listed before and after).
- [ ] Every one of the gate's six designs is `identical` (none uses a removed
      construction).
- [ ] The full suite's wall time does not grow. Re-pointed tests run on cached defaults.
- [ ] A design stored with `crank="keyed"` loads as a `Failure` naming `bolt`, not a
      traceback (test).

**Red-team angles.** A kept path that only the removed code exercised (look at coverage of
`route.py` and `stack.py` before and after). Router rules deleted that `_WebPlates` still
needs (TrotBot heel `bolt_round`: plan it). Catalog items still referenced by a kept
`sources.py` or `order.py` line. `brute.py`'s planner comparisons still covering the
router after `POST_SCREWS`' users went.

---

## W3: Build speed (the product pipeline)

**Goal.** A default build is several times faster cold and near-instant when nothing
changed. Every tool shares the speedup: `build`, `bake`, `audit`, `export`, `verify`, the
viewer and the gate.

**Scope.**
1. **A product fabrication cache.** Promote `tests/cache.py`'s BREP + pickle mechanism into
   `spiderpig/store.py` (`<store>/fab/<config.key>_<side|robot>_t<t>/`, keyed by
   `engine_version` as the store already is). `fabricate()` is served from it when present.
   `tests/cache.py` becomes a thin wrapper around it.
2. **`group_made` / mass properties.** Mirrored and translated copies of one part share one
   `part_props` result (keyed by the part's `bom_key` + shape hash, or computed once per
   distinct TShape, as `mesh_share` does for the bake). Target: ≤ 1 s.
3. **Fabrication.** Profile `fabricate()` (12.6 s) per group with W0's profiler. Then:
   - build each distinct part once (the legs' identical links and pins, as the bake's
     `mesh_share` finds them) and place copies
   - build the robot's second side as the mirror of the first where the side is
     symmetric
   - run independent groups' OCCT work in `workers` processes
4. **Exports in parallel.** STEP, STL, print STLs and the DXFs run as `workers` jobs over
   the cached fabrication.
5. **The gate.** Each design's fabrication at `t=1` and `t=4.38` comes from the cache, and
   the audit reuses the gate's fabrication rather than rebuilding it.
6. **Audit.** Reuse the cached fabrication and the stored pin loads. The sim's loads are
   already cached per design; check that the clash checks run per distinct part pair.

**Criteria** (scorecard, median of 3):
- [ ] `spiderpig build` (Strider double, empty store) ≤ **12 s** wall (from 32).
- [ ] `spiderpig build` again with the same design ≤ **4 s** wall (served from the store).
- [ ] `group` stage ≤ 1 s; `fabricate` stage cold ≤ 6 s.
- [ ] `audit` (Strider double, warm store) ≤ **30 s** (from 75-120).
- [ ] Gate on six designs ≤ **2 min** (from ~5).
- [ ] Fidelity: for each of the six gate designs, a cache-loaded fabrication equals a fresh
      one. The comparison covers the gate's part table (volume, area, centre of mass, box,
      topology counts, body order), the BOM and the DXF entities. This is a test, `slow`.
- [ ] The gate is `identical`.

**Red-team angles.**
- Staleness: does an edit to a construction, `Params`, a servo model or the catalog
  invalidate the cached fabrication? Is anything that shapes parts outside `config.key` +
  `engine_version`, such as the servo CAD download (`SPIDERPIG_SERVO_CAD`) or the
  environment?
- Concurrency: two `build`s of one design at once (the W1 lock).
- Mirroring: a mirrored side that isn't actually symmetric (a design with different phases
  per side).
- Disk growth: is the store's `gc` aware of `fab/`?

---

## W4: Test speed and isolation (unit tests in the module system)

**Goal.**
- Iterating on one module takes seconds.
- The fast tiers test behaviour through small seams, not whole robots.
- An engine edit no longer turns the whole test cache cold.

**Scope.**
1. **Incremental cache keys.** Today `engine_version()` hashes every engine source, so any
   edit makes the next run cold (689 s vs 580 s warm, and the module tiers lose their
   cache). Key each cache layer by the sources it depends on:
   - plans: `linkage/`, `linkages/`, `stack*`, `construction/` claims, `config`
   - fabrications: those plus the realize paths, `hardware/`, `servos/`
   - recorded fixtures: their generator's import closure

   The closure is computed from the import graph (`modulefinder`, or the static imports
   W5's layering makes explicit), and a docstring-only edit stays a no-op as today.
2. **Seams for unit tests.** Every construction gets unit tests that don't fabricate a
   design:
   - a synthetic `Context` / `Layout` builder in `tests/_ctx.py`, with a few links, an
     axle and a crank point, hand-made in a few lines
   - tests of `fit_hex`, `hex_gap_fit`, `fit_web`, `horn_fit_web`, the shim helper,
     `ChicagoShaft.column`, the standoff `one_piece` fit, `chassis.tie_locals`,
     `deck.path_notches` and the cut rules on those inputs, each in milliseconds
   - the same for the planner: small `StackProblem`s built by hand (some exist in
     `test_stack.py`); `brute.py` on them
3. **Unmarked slow tests.** Fix the 11 tests over 5 s in the quick tier: make them fast
   through seams, or `tiers.quick()` for the parametrized contract and servo cases. Add a
   conftest check that fails the quick tier when a non-`slow` test takes over 5 s twice in
   a row (a `--durations` budget, warn-only on a loaded machine).
4. **Fast-tier composition.** `test-quick` = the union of module tiers, run with
   `--dist loadgroup` and one shared cache warm-up.

**Criteria.**
- [ ] Every module tier ≤ **20 s** wall warm (now 7-33 s); `test-construction` ≤ 20 s.
- [ ] Quick tier ≤ **60 s** wall warm, `-n 4` (now 102-209 s).
- [ ] Full suite ≤ **300 s** wall, ≤ 3,500 CPU-s, `-n 12` (now 580 s / 6,150).
- [ ] After a one-line edit to `construction/deck.py`, the next `test-planner` and
      `test-linkage` runs are as fast as warm: within 10 %, with no re-plan in their logs.
- [ ] No non-`slow` test over 5 s in the quick tier (the scorecard lists the 10 slowest).
- [ ] ≥ 60 new unit tests across `construction/`, `hardware/`, `stack` and `route` that
      build no `Mechanism` (`fabricate` not called: a conftest guard counts it); each runs
      in < 0.5 s.
- [ ] Coverage % under the quick tier ≥ baseline + 5 points.

**Red-team angles.**
- **Incremental keys**: construct an edit that changes a plan or a part but leaves its
  cache key alone, for example a constant in `materials.py` read through a lazy import, or
  a change in `Params`. Any such edit is CONFIRMED and blocks the workstream. The fix is
  to widen the key, not to special-case the edit.
- **Synthetic contexts**: do they test something real, or only the fixture?
- **Gaming**: tests moved to `slow` to meet the time limit.

---

## W5: Modularity and de-duplication

**Goal.** No engine module over ~1,200 lines, one copy of each table, and an enforced
dependency direction between layers.

**Scope.**
1. **One screw table.** `construction/crank.py`'s `ScrewKind`, `SHCS`, `BHCS`,
   `SELF_TAP`, `screw_from_key` and `screw_body` go; `hardware/fasteners.py` is the
   source. The copies have drifted: `SELF_TAP` is `(6, 8, 10, 12)` in crank.py vs
   `(4, 5, 6, 8, 10, 12)` in `fasteners.py`; that difference reaches the gate (decide,
   below).
2. **One shim helper.** `bom.stack(total, steps, round_to=None) -> (stack, left)`. It
   replaces `crank.shim_stack`, `StandoffAxle.splice_shims` (if kept after W2),
   `ChicagoShaft.shim_count`, `chassis._shims` and `materials.washer_stack`'s loop.
3. **Dead code.** `BoltCrank.top_segment`, `walk.body_motion` (the bake uses its own
   copy) and `CrankRouter._webs`. Also anything vulture finds at ≥ 80 % after W2.
4. **NETRF6 lengths** registered on demand (`crank_catalog.pillar_shaft(L)`), not 2,921
   items at import.
5. **Split the big modules** (pure moves; the old import paths keep working through
   re-exports for one release):
   - `construction/crank/`:
     - `fasteners`, which goes with item 1
     - `base.py`: routes, claims, `CrankGroup`
     - `bolt.py`: `BoltCrank`, split further into `hex.py` (the hex pin fit),
       `web.py` (web and horn fit) and `capacity.py`
     - `plates.py`: `_WebPlates`
   - `stack/`: `geometry.py`, `topology.py`, `search.py`
     (`StackProblem`, `_Search`), `verify.py`, `plan_z.py` (the plan's z; named so it doesn't shadow `finalize`)
   - `api/`: `reports`, `store_ops`, `cards`, `planning` (plan, advise), `building`
     (build, recheck), `exports`, `walking`
   - `server/app.py`: `config.design_from_query(query, base=None)` replaces
     `_config_from_query`'s duplicate `p.NAME` parsing
   - `viewer/src/drive/index.ts` (899 lines): `tune.ts`, `physics.ts` wiring, `hud.ts`
6. **Layering enforced** with `import-linter` in CI:

   ```
   linkage → stack → construction → fabricate → (bake | build | strength | sim) → api → (mcp | server | cli)
   ```

   `hardware/` and `materials` sit beside `construction` and may not import it. Today's
   violations are listed and either fixed or written into the contract with a reason.

**Criteria.**
- [ ] `grep -n "class ScrewKind\|^SHCS\|^BHCS" spiderpig/construction` finds nothing.
- [ ] One greedy shim loop in the package (grep `for s in steps` / the helper's callers).
- [ ] No file under `spiderpig/` over 1,200 lines (scorecard), except `walk.py` if justified.
- [ ] `lint-imports` passes in CI with ≤ 5 written exceptions.
- [ ] `crank_catalog` registers ≤ 200 items at import; `catalog_cards` lists only the
      stock lengths plus what designs used.
- [ ] Gate `identical`, except item 1's self-tap lengths if the decision is to unify on
      `fasteners.py`. In that case the PR shows the one intended difference: the rear
      screw may become `m2_self_tap_4`/`5`.
- [ ] The full suite, tiers and test counts are unchanged; the moves are covered by the
      existing tests.

**Red-team angles.** A re-export that hides a cycle. A split that left a private helper
duplicated in two new modules. The shim helper changes a stack's order or rounding (the
gate's BOM text catches the count, but check the parts' positions too). Pickles in the
test cache or stored designs that name the old module paths (`tests/cache.py` pickles
`Mechanism`: do the old paths still load?).

---

## W6: Static quality and dependencies

**Goal.** Types and lint catch bugs before tests do, ratcheted: the counts only go down.

**Scope.**
- pyright in basic mode in CI:
  - mujoco and OCP marked stub-less (`reportMissingModuleSource` off, their attribute
    access ignored by `# pyright: basic` per module or by `typeCheckingMode` overrides)
  - a committed baseline count, which CI fails if it rises
  - fix the "possibly unbound" variables first: `stack.py:1876`, `crank.py:2097`,
    `route.py:958` and the other three
- ruff: add `RUF`, `C4`, `PERF`, `PIE`, `RET`, `BLE` and `PLE` to `extend-select`; fix
  or `noqa` (with a reason) every finding. Remove the 12 dead `noqa`s.
- Declare `pydantic`, `anyio` and `starlette` in `pyproject.toml`.
- Viewer:
  - three 0.160 → current
  - vite 5 → current, vitest 2 → current (this clears the 7 `npm audit` findings, among
    them esbuild's dev-server advisory, which matters behind `tailscale serve`)
  - `npm audit --omit=dev` and `npm audit` in CI
- The five `except Exception` blocks without a rationale get one, or are narrowed:
  `servos/model.py:262`, `server/watcher.py:87` and `:110`, `workers.py:122`,
  `servos/cad.py:207`.

**Criteria.**
- [ ] pyright errors ≤ 150, from 739 (excluding stub noise), and the baseline is enforced in
      CI.
- [ ] Zero "possibly unbound" and zero `reportOptionalMemberAccess` in `stack`, `route`
      and `crank`.
- [ ] `ruff check` with the extended set is clean.
- [ ] `npm audit` reports 0 high or critical findings; `test-viewer` and the e2e tests
      pass on the upgraded viewer.
- [ ] Gate `identical`.

**Red-team angles.** `# type: ignore` / `cast` used to hide real optional bugs (sample 10
of them and check each). Ruff autofixes that changed behaviour (`RET`, `SIM` rewrites in
the planner's hot loops: re-run `test-planner` timing). three.js upgrade changing the
viewer's rendering (e2e screenshots).

---

## W7: Docs that stay true

**Goal.**
- CLAUDE.md is the terse pointer file it says it is.
- Every number in the docs has one source.
- Drift is caught by a check, not by an audit.

**Scope.**
- **CLAUDE.md to ≤ 300 lines:**
  - Keep: the run commands, a "defaults today" block, the environment, ports, the
    profiler, a one-line-per-file repository map, a 15-line pipeline contract and the
    house rules.
  - Move the dated hardware decisions (lines 55-343) to `DECISIONS.md` (one entry per
    decision: date, what, why, pointer).
  - Move the construction mechanics to `ARCHITECTURE.md` §6-7 and to the module
    docstrings, which already hold most of them.
  - Delete before/after numbers; those belong in git log and STRENGTH.md.
- **One source for design numbers.** `docs/agentlib/DESIGNS.md` is generated from the
  gate's snapshot (`mise run gate -- doc`): each gate design's layers, height, crank,
  pillar, pin, sheets and audit verdict. CLAUDE.md, the README and STRENGTH.md link to
  it instead of quoting figures, which today disagree: 14/66.5, 15/66.1 and 15/75.3 for
  the Strider double; 24 and 25 layers for the quad.
- **The doc check** (`tests/doc_check.py`, in CI since W0, made blocking here):
  - every backticked `mod.func` / `Class.attr` / path / `mise run X` / `--flag` / env
    var in CLAUDE.md, AGENTS.md, README.md, ARCHITECTURE.md, API.md, TESTING.md and
    ROADMAP.md resolves
  - an allow-list for the historical docs
- **Fix the known drift:**
  - "Known hot stage": regenerate it from the bake profiler
  - the `mesh.py` row, and the planner's "full budget" (it is `max_nodes // 2`)
  - the `-m 'not slow'` wording (it includes e2e)
  - CLAUDE.md:30 → not AUDIT.md
  - AGENTS.md gets the module tiers and the gate
  - the `BuildConfig` sheets sentence
  - `ARCHITECTURE.md`'s commit stamp, its Appendix C items, and the generated
    architecture page rebuilt (deleted instead on 2026-10-08: its generator was broken)
  - SCOPE.md's dead names
- **Historical docs** (TIMING, PERF*, TESTDRIVE, AUDIT) move to `docs/history/` with a
  one-line header each; future_work.md is folded into this roadmap's "Later" section.
- **README:** say `mise run view` installs npm packages (network); the Test section
  points to the module tiers and the remote runner.

**Criteria.**
- [x] `wc -l CLAUDE.md` ≤ 300.
- [x] `mise run doc-check` passes and is blocking in CI.
- [x] `grep -rn "66.5\|66.1\|75.3\|64.3" CLAUDE.md README.md docs/audit` finds nothing
      outside DESIGNS.md (generated) and dated history.
- [ ] A fresh agent given only CLAUDE.md answers 10 questions about the defaults, the run
      commands and where things live, all correctly. The red-team reviewer writes the
      questions from the code, not the docs.

**Red-team angles.** Facts that left CLAUDE.md and landed nowhere: diff the set of
backticked identifiers before and after, and each must appear in some doc or docstring.
The doc check's resolver accepting anything (feed it a made-up name). DESIGNS.md out of
step with the gate baseline.

---

## W8: Approved output changes (one batch, one new gate baseline)

**Goal.** Land the changes that alter output, which the user approved on 2026-10-07, as one
package. Each change gets its own commit and its own listed gate diff, and the batch ends
with a re-taken gate baseline.

| # | change | decision | expected gate diff |
|---|---|---|---|
| D2 | Cantilever standoff pillar: claim and fill the clearance gap over its last link, so the link can't slide (`test_pivots::test_nothing_on_a_cantilever_pillar_can_slide` stops being xfail) | fix | parts and BOM of designs with a cantilever pillar |
| D3 | Tie-stable rounding of reported measurements (`spiderpig.rounding`, branch `w3a-rounding`): mirror twins report the same number, and the number doesn't depend on OCCT's thread count | accept | klann_quad R.b4_leg0 3.93→3.92; klann_lego_quad L/R.b1_leg0 3.28→3.27 |
| D4 | `shapes.pill` as one extruded stadium instead of fused primitives (~25-30 % CPU everywhere) | do it | DXF start vertices, the Klann quad's sheet packing, STL meshes |
| D5 | Mirror-identical printed parts grouped as "same" (`bom._proper_fit` translation-first) | do it | six print groups "1 mirrored" → "2 same"; their `_mirrored.stl` files go |
| — | The robot's preview STL stays at 0.1 rad | keep | none |
| bugs | `materials.aluminium_sheets` loads the catalog; `build.export_prints` labels split filament rows by their own filament (found by W4a) | fix | PETG/TPU print rows |

**Criteria.**
- [x] Every gate diff is one of the expected ones above; nothing else moves
  (`W8-gate-diffs.md`; D4's packing reshuffle reaches all four robot designs, not only the
  demo Klann quad).
- [ ] `mise run audit` is green on the six gate designs. Five are green. The demo `klann`
  quad still fails with the jam-SF strength errors it had before W8: 5 without the sim,
  7 with it (the wobbly demo), and W8 adds none.
- [x] The new baseline is snapshotted and named in TESTING.md (`w8-2a130c8`).
- [x] The full suite is green.

## Later (not in this plan)

Product work, not cleanup. future_work.md (2026-10-05) was folded in here by W7; its code
items (one shim helper, NETRF6 lengths on demand, unmarked slow tests) were done by W5 and W4.

**Before the first build**
- **Cut-rule warnings on the default robot.** SendCutSend's 2 x t rules
  (`spiderpig/manufacture.py`) still warn on the Strider double's aluminium (a torso plate's
  web round a cut-out, a centre plate's hole-to-edge distance: DESIGNS.md lists them).
  Warnings, not errors (every web is over 1 x t), but each is a thin web to look at on the
  first cut (`mise run audit`, the "cut rules" table).
- **Bus plug height.** The centre stack holds one plug and its wires turned toward +x
  (`BusPorts.height`, from a clone drawing): seat a plug in a servo, lay its wires
  flat and measure their top above the rear face before cutting.
- **Unpriced McMaster-Carr lines.** McMaster shows prices only behind a login, so its lines
  are unpriced in the BOM; `ORDER.md` estimates them from a priced alternative
  (`hardware/order.py`). Price them from an account, or add priced alternatives in
  `hardware/sources.py`.
- **UNVERIFIED numbers** the code flags (grep UNVERIFIED): the round crankpin's friction
  coefficients, the charger's and protection board's sizes, the 6 mm M3 standoffs' alloy
  and length tolerance. Measure them on the first parts.
- **`bolt_round` designs have no assembly order** (TrotBot's heel and toe: an `assembly:`
  audit error; DECISIONS.md, 2026-10-04): a hex plan, a hub joint fastened from the horn
  side, or leaving them out of the first build (the user's call).

**Planner and model**
- **Proofs on the quads.** Quads whose legs must sit in separate blocks (Strider, 6-bar,
  TrotBot) can leave thinner sizes "not ruled out" within the budget (`StackPlan.proof`).
  Stronger lower bounds (legs that can't share layers, as a clique over blocks) or CP-SAT
  over the static tables would close them. A secondary objective (shortest axles, fewest
  shoulders) isn't modelled.
- **Rigid link frames.** Parts are modelled in world coordinates at the build angle, so the
  bake recovers each body's motion by a planar rigid fit of its joints. A local frame per
  link and a closed-form pose `(x, y, θ)(t)` would drop the fit and make every leg's link
  literally the same part (and W3's "build each distinct part once" exact).
- **Stacking** (mechanisms on mechanisms; not built). A stage would mount on its parent's
  output body (`Output.frame`: origin joint, x-axis joint). The engine would need: the
  child's fixed pivots placed as `offset(J1, J2, along, across)` on that body instead of
  `xy`; its point names prefixed per stage so the programs concatenate into one; its input
  added to `inputs` (its crank turns relative to the parent body, so its angle is its input
  plus that body's rotation); one drive per input; and the planner's clearances between
  bodies in relative motion across stages (the child's frame is a moving body, not the
  frame plates).

**Firmware.** The servo driver's gait and steering code (the PI phase lock the sim models,
`sim/run.py`; a design's torque limit, `config.LINKAGE_TORQUE_LIMITS`) is outside this
repository.

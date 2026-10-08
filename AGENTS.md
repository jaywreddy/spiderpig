# AGENTS.md — how to interact with this repo

**Canonical interface: `mise`.** All routine tasks (run the viewer, bake
GLBs, build STEP/STL/DXF, run tests, lint, clean) go through tasks
defined in `mise.toml`. Don't shell out to `uv`, `npm`, `pytest`, or
`uvicorn` directly when a `mise run …` task already exists — use the
task. This keeps tool versions, working directories, and dependency
chains consistent.

## Day-to-day commands

```bash
mise run view           # FastAPI + Vite (HMR) on per-worktree ports — open the banner's URL
mise run bake           # bake <store>/bakes/*.glb (the project store, .spiderpig/)
mise run build          # STEP/STL/DXF -> build/ (a current build/ is skipped; -- --force)
mise run test-<module>  # a module tier, seconds: linkage, planner, construction, hardware,
                        # strength, api, sim, server; test-viewer for the TypeScript
mise run test-quick     # the quick tier (-m 'not slow and not e2e', xdist); full: remote-test
mise run test           # pytest, every test, serial (auto-builds spiderpig/viewer/dist for e2e)
mise run gate -- compare ~/.cache/spiderpig/gate/next-SHA   # product edits: did a part move?
mise run scorecard      # every ROADMAP number -> build/scorecard.json (~10 min)
mise run lint           # ruff, then lint-imports (the engine's layers)
mise run pyright        # types, basic mode
mise run doc-check -- --strict   # the docs' backticked names resolve (CI blocks on a miss)
mise run clean          # rm build/, dist/, .spiderpig/bakes/, spiderpig/viewer/dist/, viewer/node_modules/
mise tasks              # list everything available
```

**Temporary files.** pytest, OCCT, the gate and the scorecard write under `$TMPDIR`
(default `/tmp`). On a box where `/tmp` is a small tmpfs, export TMPDIR to a roomy disk
first (e.g. `~/.cache/spiderpig/tmp`), and remove what you created.

`viewer-install`, `viewer-dev`, `viewer-build` exist as sub-tasks but are
auto-pulled by `view` / `test` via `depends`. Don't call them by hand
unless you're debugging the build itself.

## Running tests, audits and sims: quick here, heavy remotely

The full suite (~1150 tests: OCCT solids, plans, MuJoCo, bakes) takes about two CPU-hours;
don't run it serially on a laptop. Iterate with a module tier
(`docs/agentlib/TESTING.md`: seconds each on the fabrication cache and the recorded
fixtures), then:

```bash
mise run test-planner -- -k route -x     # one module's fast tier, narrowed
mise run test-quick                      # every module's fast tier: -m 'not slow and not e2e', -n 4
mise run test-quick -- tests/test_stack.py -k route   # narrowed further
mise run remote-test                     # every test on the remote, -n 12 (~6 min)
mise run remote-test -- -m slow -k sim   # any pytest args
mise run remote-audit                    # the default Strider's four modules' audits at once
mise run remote-audit -- --linkage klann # a linkage named: all four modules
mise run remote -- uv run python -m spiderpig.cli sim --module quad   # sims, bakes, any command
```

`remote` / `remote-test` / `remote-audit` (`spiderpig/tools/remote.py`) rsync the
working tree as it is (uncommitted edits included; not `.venv`, `node_modules`,
`.spiderpig/`, `build/`, caches) to `$SPIDERPIG_REMOTE` (default `root@ao-server`)
under `~/spiderpig-ci/<checkout>-<hash>/` (one folder per worktree and machine; a
second run from the same worktree waits for its lock), `uv sync --locked` there
(Python 3.12; uv's cache and interpreter under `~/spiderpig-ci/` too), stream the
output, and copy the log, the junit XML and the remote `build/` (audit reports,
exports) back to `build/remote/<run>/`. The exit status is the remote command's. The
remote keeps its own store (`.spiderpig/` in that folder), so plans and bakes stay
warm between runs of a worktree. `SPIDERPIG_REMOTE_WORKERS` changes `-n`.

**The remote may be unavailable.** On 2026-10-08 the default `root@ao-server` was the
development box itself (its hostname) and refused SSH connections; check with
`ssh root@ao-server true` first, or point `SPIDERPIG_REMOTE` elsewhere. Without it, run the full suite
locally on a many-core box: `uv run pytest -p no:warnings -m 'not e2e' -n 12 --dist
worksteal` (TESTING.md has its time), and the audits with `mise run audit`.

**Product edits** (anything that can move a part, a plan, the BOM or a cut file) also run
the identity gate before and after (`mise run gate -- compare <baseline>`, TESTING.md):
"identical", or each difference listed and accepted, and a new baseline with
`docs/agentlib/DESIGNS.md` regenerated (`mise run gate -- doc <snapshot>`).

Compare a run's failures with the junit XML in `build/remote/<run>/`, not with a local
run: a few tests hold the planner to a CPU-seconds deadline and a busier machine (more
workers than 12 there, or a loaded laptop) can run it out.

Mark a test `slow` when it takes more than ~5 s (a robot or side the session fixtures
don't share, a plan search, a bake, MuJoCo, a CLI run); a heavy parametrized check
keeps one cheap case quick with `tests/tiers.py`'s `quick(values, keep)`. The quick tier
skips the rest; the full tier runs everything. Tests must stay xdist-safe: write only
under `tmp_path` / `tmp_path_factory` (the session store already is one per worker),
pick free ports, never mutate the session fixtures' objects. Under xdist each worker
gets one BLAS and one OCCT thread (`tests/conftest.py`).

## When NOT to use mise

Use raw commands only for **one-off validation scripts** — small
throwaway probes for a hypothesis (e.g. `uv run python -c "from spiderpig import linkage;
print(linkage.get('klann').check())"`, a tiny temp `.py` to dump a value, an `npx tsc
--noEmit` to look at type errors during a refactor). If a probe is going
to be used more than twice, promote it to a `mise.toml` task instead.

Also raw, never via mise:

- `uv sync` — initial dependency install (no mise wrapper, by design).
- `mise install` — pin Python/uv/Node from `mise.toml` (bootstrap).

## Adding a new task

Edit `mise.toml`, not `spiderpig/tools/`. Tasks should be one-liners that
delegate to `uv run …` or `npm run …`. `spiderpig/tools/` otherwise holds the
`spiderpig` subcommands (audit, export, report, sim, tune) and the helpers the
tasks need: `dev.py` spawns FastAPI + Vite *in parallel* (something a single task
command can't express portably), `kill_dev.py` stops them, `remote.py` runs the
`remote*` tasks.

## What the server does

`spiderpig/server/app.py` (FastAPI + uvicorn) serves `spiderpig/viewer/dist/`
(Vite's output, shipped as package data) plus `/api/glb/{mode}` and a `/ws`
watch-and-rebake channel. In dev, Vite proxies `/api` and `/ws` to FastAPI
(per-worktree ports, see the banner); in prod or e2e, just hit FastAPI with
`spiderpig/viewer/dist` mounted. Override the static dir with
`SPIDERPIG_VIEWER_DIST` if needed.

## Testing the viewer end-to-end

`mise run test -- -m e2e` runs Playwright against the **built** bundle
(the `test` task depends on `viewer-build`). The fixture in
`tests/conftest.py` will refuse to start if `spiderpig/viewer/dist/` is missing —
that's intentional, it forces e2e to test what users actually see.

For a quick visual smoke check, `mise run view` and open the URL its banner prints
(ports come from a hash of the worktree's path; `VITE_PORT` / `API_PORT` pin them,
`VITE_ALLOWED_HOSTS` lets Vite and the API server answer other host names, e.g. behind
`tailscale serve`; the server refuses any other Host header with a 400, and a WebSocket
from a foreign origin).

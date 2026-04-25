# AGENTS.md — how to interact with this repo

**Canonical interface: `mise`.** All routine tasks (run the viewer, bake
GLBs, build STEP/STL/DXF, run tests, lint, clean) go through tasks
defined in `mise.toml`. Don't shell out to `uv`, `npm`, `pytest`, or
`uvicorn` directly when a `mise run …` task already exists — use the
task. This keeps tool versions, working directories, and dependency
chains consistent.

## Day-to-day commands

```bash
mise run view           # FastAPI :8000 + Vite :5173 (HMR) — open http://localhost:5173
mise run bake           # bake viewer/data/*.glb
mise run build          # STEP/STL/DXF -> build/
mise run test           # pytest (auto-builds viewer/dist for e2e)
mise run lint           # ruff
mise run clean          # rm build/, viewer/data/, viewer/dist/, viewer/node_modules/
mise tasks              # list everything available
```

`viewer-install`, `viewer-dev`, `viewer-build` exist as sub-tasks but are
auto-pulled by `view` / `test` via `depends`. Don't call them by hand
unless you're debugging the build itself.

## When NOT to use mise

Use raw commands only for **one-off validation scripts** — small
throwaway probes for a hypothesis (e.g. `uv run python -c "import klann;
print(klann.foo())"`, a tiny temp `.py` to dump a value, an `npx tsc
--noEmit` to look at type errors during a refactor). If a probe is going
to be used more than twice, promote it to a `mise.toml` task instead.

Also raw, never via mise:

- `uv sync` — initial dependency install (no mise wrapper, by design).
- `mise install` — pin Python/uv/Node from `mise.toml` (bootstrap).

## Adding a new task

Edit `mise.toml`, not `scripts/`. Tasks should be one-liners that
delegate to `uv run …` or `npm run …`. The only existing helper script,
`scripts/dev.py`, is justified because it spawns FastAPI + Vite *in
parallel* — something a single task command can't express portably.

## What the server does

`server/app.py` (FastAPI + uvicorn) serves `viewer/dist/` (Vite output)
plus `/api/glb/{mode}` and a `/ws` watch-and-rebake channel. In dev,
Vite on `:5173` proxies `/api` and `/ws` to FastAPI on `:8000`; in prod
or e2e, just hit FastAPI on `:8000` with `viewer/dist` mounted. Override
the static dir with `SPIDERPIG_VIEWER_DIST` if needed.

## Testing the viewer end-to-end

`mise run test -- -m e2e` runs Playwright against the **built** bundle
(the `test` task depends on `viewer-build`). The fixture in
`tests/conftest.py` will refuse to start if `viewer/dist/` is missing —
that's intentional, it forces e2e to test what users actually see.

For a quick visual smoke check, `mise run view` and look at
http://localhost:5173.

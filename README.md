# spiderpig — Klann walking-linkage generator

Python tooling that turns the symbolic definition of a
[Klann linkage](https://en.wikipedia.org/wiki/Klann_linkage) into ready-to-fabricate
STEP + STL assemblies and DXF sheets for a laser cutter.

Originally UC Berkeley CS194 coursework built on SolidPython + OpenSCAD +
`digifab`; ported to a modern Python 3.12 stack:

| concern             | tool                                     |
|---------------------|------------------------------------------|
| symbolic geometry   | [`sympy`](https://www.sympy.org)         |
| CAD / B-rep         | [`build123d`](https://build123d.readthedocs.io) |
| SE(3) kinematics    | [`pytransform3d`](https://dfki-ric.github.io/pytransform3d/) |
| DXF output          | [`ezdxf`](https://ezdxf.mozman.at)       |
| sheet packing       | [`rectpack`](https://github.com/secnot/rectpack) |
| dep + tool mgmt     | [`mise`](https://mise.jdx.dev) + [`uv`](https://docs.astral.sh/uv/) |
| frontend bundler    | [`vite`](https://vitejs.dev) + TypeScript |
| tests               | [`pytest`](https://docs.pytest.org)      |

## Install

```bash
mise install        # pin Python 3.12 + uv + node 20
uv sync             # resolve pyproject.toml (fetches OCP/OCCT; first run is slow)
```

## Quick start

```bash
mise run view           # FastAPI :8000 + Vite :5173 with HMR — open http://localhost:5173
mise run build          # STEP/STL/DXF → build/
mise run bake           # viewer/data/*.glb
mise run test           # pytest (unit; -m e2e for browser tests)
mise run audit          # do the parts physically fit? (clashes, solids, plan, DXF)
mise run lint           # ruff check
mise run clean          # rm build/, viewer/data/, viewer/dist/, viewer/node_modules/
```

`mise run view` starts FastAPI on `:8000` (bakes `.glb` on first request,
watches `*.py` and re-bakes on change, broadcasts over `/ws`) and Vite on
`:5173` (HMR for the TypeScript viewer; proxies `/api` and `/ws` to FastAPI).
Edit a `.ts` file → instant HMR. Edit a `.py` kinematics file → re-bake →
viewer hot-swaps the GLB without a full page reload.

For a production-style single-port run, build the bundle then start FastAPI
directly:

```bash
mise run viewer-build
uv run uvicorn server.app:app --host 127.0.0.1 --port 8000
```

## Run

```bash
uv run python main.py --out build/
```

It prints the stack plan (which part sits in which 3 mm slot) and produces:

- `build/klann.step` — full assembly, colour-tagged, viewable in FreeCAD,
  KiCad's 3D viewer, or any STEP importer.
- `build/klann.stl` — meshed assembly for slicers.
- `build/klann_sheet_*.dxf` — one DXF per 200 × 200 mm sheet with every
  laser-cut link (b1..b4 of every leg) laid flat and packed; outer contours
  on layer `CUT` as `LWPOLYLINE`, pin holes as `CIRCLE`, units = mm.

Everything else is printed: the frame (one piece: plate, posts, servo pad),
the crankshaft segments and crankpins, and a pin plus press-on cap at every
pivot.

Flags:

- `--out PATH` — output directory (created if missing; default `./build`).
- `--name STEM` — file-name stem for STEP/STL outputs.
- `--mode {single,double,decker,quad}` — which assembly (default `single`).
- `--no-dxf` — skip the DXF sheet-packing pass.

## How the parts fit together

The linkage is planar; `stack.py` decides the Z stack. Each link gets a
slot, and a slot may be shared only by parts that never touch anywhere in
the crank cycle (checked over 720 crank angles with 1 mm clearance). Pins
carry a head below their lowest link and a press-on cap above their
highest. The frame plate sits on top with the servo.

Every b1 sweeps across the crank axis O, so the crank is a built-up
crankshaft: in each b1's slot it is only that b1's crankpin, with webs
(arms across the centre) in the slots either side. That is what lets the
decker and quad walkers share one servo.

## Test

```bash
uv run pytest
```

Unit tests cover the symbolic core (reference foot values, rigidity,
phase as a time shift), assemblies, the stack plan (including an
independent full-cycle re-check), fabricated parts (no clashes, one solid
each, pins through every link they join), the glTF bake (animated meshes
match the fabricated parts) and STEP / STL / DXF emission. `-m e2e` runs
the Playwright viewer tests.

## Layout

```
spiderpig/
├── mise.toml        # tool versions (python/uv/node) + tasks (view/build/bake/test/lint/clean)
├── scripts/dev.py   # spawns FastAPI + Vite for `mise run view`
├── main.py          # fabrication CLI (STEP/STL/DXF)
├── klann.py         # symbolic Klann program + assembly templates
├── mechanism.py     # Pose, Joint, Body, Mechanism (pytransform3d-backed)
├── stack.py         # layer plan: slots, pins, crankshaft (full-cycle clearance)
├── fabricate.py     # plan -> build123d parts
├── shapes.py        # build123d part primitives
├── layout.py        # 2D section + rectpack + ezdxf sheet writer
├── scripts/audit_fab.py  # `mise run audit`
├── server/          # FastAPI dev server + watchfiles live-reload
│   ├── app.py       #   /api/modes, /api/glb/{mode}, /ws, static mount
│   └── watcher.py   #   source-change → re-bake → broadcast reload
├── viewer/          # Vite + TypeScript three.js client
│   ├── index.html
│   ├── package.json, tsconfig.json, vite.config.ts
│   ├── bake_gltf.py #   .glb baker (Python)
│   └── src/         #   main.ts, scene.ts, loader.ts, controls.ts, live-reload.ts
├── pyproject.toml
├── uv.lock
└── tests/           # unit tests; tests/e2e/ for Playwright
```

## Customising the linkage

The Klann proportions are exact rationals in `klann.PROPORTIONS` (lengths
as multiples of `OA`, angles in degrees). They are symbols in the compiled
program, so the program never needs re-deriving: change a value and every
link length, the foot path, the stack plan and the parts follow. Re-run
`main.py` (and `mise run audit`) to regenerate and check STEP/STL/DXF.

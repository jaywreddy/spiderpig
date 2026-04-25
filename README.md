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

Produces:

- `build/klann.step` — full assembly, colour-tagged, viewable in FreeCAD,
  KiCad's 3D viewer, or any STEP importer.
- `build/klann.stl` — meshed assembly for slicers.
- `build/klann_sheet_*.dxf` — one DXF per 200 × 200 mm sheet with every
  laser-cut link packed; outer contours on layer `CUT` as `LWPOLYLINE`,
  joint holes as `CIRCLE`, units = mm.

Flags:

- `--out PATH` — output directory (created if missing; default `./build`).
- `--name STEM` — file-name stem for STEP/STL outputs.
- `--no-dxf` — skip the DXF sheet-packing pass.

## Test

```bash
uv run pytest
```

20 smoke tests covering: `Pose` round-trips, `Mechanism.solved()` joint
alignment on a toy fixture and the real `KlannLinkage`, reference foot-tip
values for `create_klann_geometry`, and end-to-end STEP / STL / DXF
emission.

## Layout

```
spiderpig/
├── mise.toml        # tool versions (python/uv/node) + tasks (view/build/bake/test/lint/clean)
├── scripts/dev.py   # spawns FastAPI + Vite for `mise run view`
├── main.py          # fabrication CLI (STEP/STL/DXF)
├── klann.py         # symbolic Klann geometry + KlannLinkage assembly
├── mechanism.py     # Pose, Joint, Body, Mechanism (pytransform3d-backed)
├── shapes.py        # build123d part factories
├── layout.py        # 2D section + rectpack + ezdxf sheet writer
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
└── tests/
    ├── test_transforms.py
    ├── test_geometry.py
    ├── test_mechanism.py
    ├── test_poses.py
    └── test_export.py
```

## Customising the linkage

The Klann proportions live as constants in
`klann.create_klann_geometry()`. Change `mOA`, the angles on A and B, or
any of the radius ratios; every downstream link length and the foot path
update through the sympy chain. Re-run `main.py` to regenerate STEP/STL/DXF.

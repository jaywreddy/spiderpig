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
| dep management      | [`uv`](https://docs.astral.sh/uv/)       |
| tests               | [`pytest`](https://docs.pytest.org)      |

## Install

```bash
uv sync
```

This resolves `pyproject.toml` and fetches OCP (the OCCT binding that powers
build123d). First run takes a minute or two.

## Quick start

Every task goes through a single cross-platform Python CLI (no `sh` required):

```bash
uv run python cli.py view       # dev server w/ live reload at :8000
uv run python cli.py build      # STEP/STL/DXF → build/
uv run python cli.py bake       # viewer data → viewer/data/
uv run python cli.py test
uv run python cli.py clean      # rm build/ and viewer/data/
```

Or, if you have [`just`](https://github.com/casey/just):

```bash
just            # list recipes
just view
just build
just test
```

`just view` starts a FastAPI dev server on <http://127.0.0.1:8000> that
serves the three.js viewer, bakes data on first run, watches `*.py`
under the repo, and pushes a browser reload over a WebSocket after every
successful re-bake — the full edit → visualize loop without a manual refresh.

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
├── cli.py           # cross-platform task runner (view/build/bake/test/clean)
├── main.py          # fabrication CLI (STEP/STL/DXF)
├── klann.py         # symbolic Klann geometry + KlannLinkage assembly
├── mechanism.py     # Pose, Joint, Body, Mechanism (pytransform3d-backed)
├── shapes.py        # build123d part factories
├── layout.py        # 2D section + rectpack + ezdxf sheet writer
├── server/          # FastAPI dev server + watchfiles live-reload
│   ├── app.py       #   /api/frames, /api/parts/{name}, /ws, static mount
│   └── watcher.py   #   source-change → re-bake → broadcast reload
├── viewer/          # three.js client (HTML/JS + bake.py + poses.py)
├── justfile         # optional Unix alias for `uv run python cli.py ...`
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

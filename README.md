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

With [`just`](https://github.com/casey/just) installed:

```bash
just            # list recipes
just build      # STEP/STL/DXF → build/
just view       # bake viewer data, serve, open browser
just test
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
├── main.py          # CLI entry point
├── klann.py         # symbolic Klann geometry + KlannLinkage assembly
├── mechanism.py     # Pose, Joint, Body, Mechanism (pytransform3d-backed)
├── shapes.py        # build123d part factories
├── layout.py        # 2D section + rectpack + ezdxf sheet writer
├── pyproject.toml
├── uv.lock
└── tests/
    ├── test_transforms.py
    ├── test_geometry.py
    ├── test_mechanism.py
    └── test_export.py
```

## Customising the linkage

The Klann proportions live as constants in
`klann.create_klann_geometry()`. Change `mOA`, the angles on A and B, or
any of the radius ratios; every downstream link length and the foot path
update through the sympy chain. Re-run `main.py` to regenerate STEP/STL/DXF.

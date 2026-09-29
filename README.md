# spiderpig — Klann walking-linkage generator

Python tooling that turns the symbolic definition of a
[Klann linkage](https://en.wikipedia.org/wiki/Klann_linkage) into a
ready-to-build walking robot: laser-cut plates (DXF), 3D-printed parts
(STL), a bill of materials with vendor links, and an animated 3D view.

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
mise run build          # STEP/STL/DXF/BOM → build/
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
uv run python main.py --out build/                 # the quad robot (4 legs per side)
uv run python main.py --module single --out build/single
uv run python main.py --list                       # modules, servos, constructions, sheets
```

It prints the layer plan of one side and writes:

- `build/klann.step` / `klann.stl` — the whole robot (both sides, servos,
  frame), colour-tagged.
- `build/print/` — one STL per different printed part, flat on the build
  plate, `*_mirrored.stl` where the right side needs the mirror image, and
  `parts.csv` with how many of each to print.
- `build/laser/klann_sheet_*.dxf` — every laser-cut part, kerf-compensated
  and packed on the sheet stock; outer contours as `LWPOLYLINE`, holes as
  `CIRCLE`, layer `CUT`, mm. `klann_sheet_parts.csv` says which part is where.
- `build/bom.csv` / `bom.md` / `bom.json` — what to buy (quantities, packs,
  vendor links, whether each link was checked, estimated cost), what to
  print (filament) and what to cut (sheets).

Useful flags: `--module {single,double,decker,quad}` (legs per side),
`--side-only`, `--servo` (continuous-rotation servos only; default
`sts3215`), `--pillar` / `--pin` / `--crank` (constructions), `--sheet`,
`--thickness` (measure your sheet: acrylic varies by up to 8 %), `--kerf`,
`--no-dxf`.

## How it is built

The linkage is planar; the design (sympy) says only where the joints are
over the crank cycle. `fabricate.py` then *rationalizes* it, one functional
group at a time (see `construction/base.py`):

- **servo drive** — an STS3215 stands on the inner frame plate, output face
  down; its horn turns in a hole in the plate;
- **crank** — a printed built-up crankshaft bolted to the horn: every b1
  sweeps across the crank axis, so the crank reaches each b1 only along its
  crankpin, with webs in the layers either side;
- **pillars** — the frame pivots: printed stepped axles held by both the
  inner and the outer frame plate, with shoulders (built-in spacers) beside
  each link and thin necks where other links pass;
- **pins** — the pivots between links: printed stepped axles with a head
  and a snap cap;
- **links and frame plates** — laser-cut, holes cut for everything above.

Each group first *claims* the space it needs, per 3 mm layer and relative
to the link layers; the planner (`stack.py`) finds link layers where no
two groups' claims ever meet over the whole crank cycle. Then each group
builds its parts inside its claims, which a contract test checks. So the
parts can't collide, and a construction that can't fit is an error, never
broken geometry.

The robot is two mirror-image sides with their servos back to back on
centre plates, tied into one frame by printed columns.

## Test

```bash
uv run pytest
```

Unit tests cover the symbolic core (reference foot values, rigidity,
phase as a time shift), assemblies, the layer planner (including an
independent full-cycle re-check), the construction contract (every part
inside its claims, for every module), clashes, the printed axles and
crank, the robot frame, the BOM, the glTF bake (animated meshes match the
fabricated parts) and STEP / STL / DXF emission. `-m e2e` runs the
Playwright viewer tests. `mise run audit` checks every module end to end.

## Layout

```
spiderpig/
├── mise.toml        # tool versions (python/uv/node) + tasks (view/build/bake/test/lint/audit)
├── main.py          # fabrication CLI (STEP/STL/DXF/BOM)
├── klann.py         # symbolic Klann program + leg module templates
├── mechanism.py     # Pose, Joint, Body, Mechanism (pytransform3d-backed)
├── stack.py         # layer planner over claims (full-cycle clearance)
├── fabricate.py     # BuildConfig; design a side, fabricate the robot
├── construction/    # the groups: axle, crank, plates, robot; contract check
├── servos/          # servo data (spec, catalog), drive group, models, CAD cache
├── hardware/        # purchasable-item catalog and the bill of materials
├── shapes.py        # build123d part primitives
├── layout.py        # 2D section + rectpack + ezdxf sheet writer
├── scripts/audit_fab.py  # `mise run audit`
├── server/          # FastAPI dev server + watchfiles live-reload
├── viewer/          # Vite + TypeScript three.js client, bake_gltf.py
└── tests/           # unit tests; tests/e2e/ for Playwright
```

## Customising the linkage

The Klann proportions are exact rationals in `klann.PROPORTIONS` (lengths
as multiples of `OA`, angles in degrees). They are symbols in the compiled
program, so the program never needs re-deriving: change a value and every
link length, the foot path, the stack plan and the parts follow. Re-run
`main.py` (and `mise run audit`) to regenerate and check STEP/STL/DXF.

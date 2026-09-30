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
| DXF output          | [`ezdxf`](https://ezdxf.mozman.at)       |
| sheet packing       | [`rectpack`](https://github.com/secnot/rectpack) |
| dep + tool mgmt     | [`mise`](https://mise.jdx.dev) + [`uv`](https://docs.astral.sh/uv/) |
| frontend bundler    | [`vite`](https://vitejs.dev) + TypeScript |
| tests               | [`pytest`](https://docs.pytest.org)      |

## Install

From a checkout (`uv sync` installs the `spiderpig` package editable, with the
dev tools; the viewer is built once, into the package):

```bash
mise install            # pin Python 3.12 + uv + node 20
uv sync                 # .venv: spiderpig (editable) + dev tools (fetches OCP/OCCT; slow the first time)
mise run viewer-build   # the viewer -> spiderpig/viewer/dist (node; again after a viewer/ change)
uv run spiderpig --help
```

As a package, with no Node on the machine (the wheel carries the built viewer):

```bash
uv pip install .                  # or the wheel from `mise run release` (dist/*.whl)
spiderpig --help
spiderpig view <design-id>        # the viewer for a stored design (prints the URL; --open)
spiderpig mcp --store .spiderpig  # the MCP server for an agent (docs/agentlib/API.md)
```

`mise run release` builds the viewer, then the sdist and wheel into `dist/`
(`uv build`); a wheel built without the viewer fails with a message saying so
(`hatch_build.py`), and the wheel is checked to hold `spiderpig/viewer/dist`
and nothing of `viewer/` (sources, `node_modules`) or `tests/`.

## Quick start

```bash
mise run view           # FastAPI :8000 + Vite :5173 with HMR — open http://localhost:5173
mise run build          # STEP/STL/DXF/BOM → build/
mise run bake           # <store>/bakes/*.glb (the project store, .spiderpig/)
mise run test           # pytest (unit; -m e2e for browser tests)
mise run audit          # do the parts physically fit? (clashes, solids, plan, DXF)
mise run lint           # ruff check
mise run clean          # rm build/, dist/, .spiderpig/bakes/, spiderpig/viewer/dist/, viewer/node_modules/
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
uv run uvicorn spiderpig.server.app:app --host 127.0.0.1 --port 8000
```

## Run

```bash
spiderpig build --out build/                 # the quad robot (4 legs per side)
spiderpig build --module single --out build/single
spiderpig build --linkage jansen --module double --out build/jansen
spiderpig build --list                       # linkages, modules, servos, constructions, sheets
```

`--linkage` (Klann by default), `--module`, `--phases` and `--proportion
NAME=VALUE` (the linkage's parameters) work the same way for `spiderpig bake`,
`spiderpig explain`, the other tools and (`linkage=`, `module=`, `phases=`,
`p.NAME=`) the viewer's `/api/walk` and `/api/glb`: they all build one
validated `spiderpig.config.BuildConfig`; `spiderpig explain` and `spiderpig
audit` take the build options below (`--servo`, `--pin`, `--pillar`, `--sheet`,
`--thickness`) too, since the static facts and the plan depend on them. `spiderpig build` prints the layer
plan of one side and writes
(stem: the linkage, `--name` to change it):

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

`spiderpig view <design-id>` serves the animated viewer for a design recorded in
the project store by the Python API or the MCP server (`docs/agentlib/API.md`),
with the drive and tune panels answering for that design.

## How it is built

The linkage is planar; the design (sympy) says only where the joints are
over the crank cycle. `spiderpig/fabricate.py` then *rationalizes* it, one
functional group at a time (see `spiderpig/construction/base.py`):

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
to the link layers; the planner (`spiderpig/stack.py`) finds link layers where no
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
spiderpig/                   # the checkout
├── mise.toml                # tool versions (python/uv/node) + tasks (view/build/bake/test/lint/audit/...)
├── pyproject.toml           # the installable package (hatchling); `spiderpig` console script
├── spiderpig/               # the Python package: engine, agent API, CLI, server, built viewer
│   ├── cli.py               # `spiderpig <command>`: build | bake | audit | explain | tune | sim | report | mcp
│   ├── build.py             # fabrication CLI (STEP/STL/DXF/BOM)
│   ├── bake.py              # the viewer's animated .glb bake (cached in the store's bakes/)
│   ├── linkage/             # the symbolic engine (engine.py), stage checks (checks.py), leg module templates (assembly.py)
│   ├── linkages/            # one module per linkage family (Klann, Strider, Jansen, ...)
│   ├── mechanism.py         # Pose, Joint, Body, Mechanism, MechanismTemplate
│   ├── stack.py             # layer planner over claims (full-cycle clearance)
│   ├── config.py            # BuildConfig (what to build and how, validated); the shared CLI / query arguments
│   ├── fabricate.py         # design a side, fabricate the robot
│   ├── construction/        # the groups: axle, crank, plates, robot, chassis; contract check
│   ├── servos/              # servo data (spec, catalog), drive group, models, CAD cache
│   ├── hardware/            # catalog, screw families, materials and masses, the bill of materials
│   ├── walk.py              # quasi-static walking model (/api/walk, the viewer's drive mode)
│   ├── sim/                 # MuJoCo model of the fabricated robot and its runner
│   ├── shapes.py            # build123d part primitives
│   ├── layout.py            # 2D section + rectpack + ezdxf sheet writer
│   ├── tools/               # audit.py (`mise run audit`), tune.py, sim_walk.py, report.py, dev.py, kill_dev.py
│   ├── server/              # the viewer's FastAPI app + watchfiles live-reload
│   ├── viewer/dist/         # the built viewer (gitignored; `mise run viewer-build`; ships in the wheel)
│   ├── spec.py, api.py, …   # the agent-facing API and store (docs/agentlib/API.md)
│   └── mcp/                 # the MCP server over it
├── viewer/                  # Vite + TypeScript three.js client sources (never ship)
└── tests/                   # unit tests; tests/e2e/ for Playwright
```

## Customising the linkage

Each linkage's proportions are exact rationals in its module under
`spiderpig/linkages/` (Klann: `spiderpig/linkages/klann.py`, lengths as multiples of `OA`,
angles in degrees). They are symbols in the compiled program, so the program
never needs re-deriving: change a value (or pass `--proportion NAME=VALUE`)
and every link length, the foot path, the stack plan and the parts follow.
Re-run `spiderpig build` (and `mise run audit`) to regenerate and check STEP/STL/DXF.

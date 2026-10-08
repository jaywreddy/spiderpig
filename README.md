# spiderpig — walking-linkage generator

Python tooling that turns the symbolic definition of a walking linkage (the
[Strider](https://www.diywalkers.com/strider-linkage-plans.html) by default; the
[Klann linkage](https://en.wikipedia.org/wiki/Klann_linkage), Jansen, TrotBot and
others are registered too) into a
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

New to the code? [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) walks through how it is
structured, what each part can do, and its limitations and risks.

## Install

From a checkout (`uv sync` installs the `spiderpig` package editable, with the
dev tools; the viewer is built once, into the package):

```bash
mise install            # pin Python 3.12 + uv + node 22
uv sync                 # .venv: spiderpig (editable) + dev tools (fetches OCP/OCCT; slow the first time)
mise run viewer-build   # the viewer -> spiderpig/viewer/dist (node; again after a viewer/ change)
uv run spiderpig --help
```

As a package, with no Node on the machine (the wheel carries the built viewer):

```bash
uv pip install .                  # or the wheel from `mise run release` (dist/*.whl)
spiderpig --help
spiderpig view <design-id>        # the viewer for a stored design (prints the URL; --open)
spiderpig export <design-id> --formats step dxf bom glb mjcf   # a stored design's files
spiderpig sim <design-id>         # MuJoCo on a stored design (its exported MJCF if any)
spiderpig mcp --store .spiderpig  # the MCP server for an agent (docs/agentlib/API.md)
```

`mise run release` builds the viewer, then the sdist and wheel into `dist/`
(`uv build`); a wheel built without the viewer fails with a message saying so
(`hatch_build.py`), and the wheel is checked to hold `spiderpig/viewer/dist`
and nothing of `viewer/` (sources, `node_modules`) or `tests/`.

## Quick start

```bash
mise run view           # FastAPI + Vite with HMR — open the URL the banner prints
mise run build          # STEP/STL/DXF/BOM → build/ (skipped when build/ is current; -- --force)
mise run bake           # <store>/bakes/*.glb (the project store, .spiderpig/)
mise run test-planner   # one module's fast tier (seconds); test-quick: all of them
mise run audit          # do the parts fit and hold? (clashes, solids, plan, DXF, strength)
mise run lint           # ruff check, then the import layers (lint-imports)
mise run clean          # rm build/, dist/, .spiderpig/bakes/, spiderpig/viewer/dist/, viewer/node_modules/
```

`mise run view` first runs `npm install` in `viewer/` (the `viewer-install` task: it needs
the network the first time, and again when `viewer/package.json` changes), then starts FastAPI (bakes `.glb` on first request, watches `*.py` and
re-bakes on change, broadcasts over `/ws`) and Vite (HMR for the TypeScript viewer;
proxies `/api` and `/ws` to FastAPI) on ports derived from the worktree's path (Vite in
5500-5999, the API in 8500-8999), so parallel worktrees don't collide; `VITE_PORT` /
`API_PORT` pin them, and `VITE_ALLOWED_HOSTS` (comma-separated, e.g. `.ts.net` behind
`tailscale serve`) lets Vite and the API server answer other host names (the server answers
only loopback names, IP addresses and those, and refuses a WebSocket from another page's
origin: `spiderpig/server/app.py` `HostGuard`).
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
spiderpig build --out build/                 # the Strider double (two coupled pairs per side)
spiderpig build --linkage klann --out build/klann   # the Klann quad (the wobbly demo)
spiderpig build --module single --out build/single
spiderpig build --linkage jansen --module double --out build/jansen
spiderpig build --list                       # linkages, modules, servos, constructions, sheets
```

`--linkage` (Strider by default), `--module` (the linkage's own by default: Strider's
`double`, another walker's `quad`), `--phases` and `--proportion
NAME=VALUE` (the linkage's parameters) work the same way for `spiderpig bake`,
`spiderpig explain`, the other tools and (`linkage=`, `module=`, `phases=`,
`p.NAME=`) the viewer's `/api/walk` and `/api/glb`: they all build one
validated `spiderpig.config.BuildConfig`; `spiderpig explain` and `spiderpig
audit` take the build options below (`--servo`, `--pin`, `--pillar`, `--sheet`,
`--thickness`) too, since the static facts and the plan depend on them. `spiderpig build` prints the layer
plan of one side and writes
(stem: the linkage, `--name` to change it; into an `--out` already current for the same
design and engine it does nothing and says so, `spiderpig/uptodate.py`; `--force` builds;
fabrications are cached in the store, `spiderpig/fabcache.py`):

- `build/strider.step` / `strider.stl` — the whole robot (both sides, servos,
  frame); the STEP is colour-tagged.
- `build/print/` — one STL per different printed part, flat on the build
  plate, `*_mirrored.stl` where the right side needs the mirror image, and
  `parts.csv` with how many of each to print.
- `build/laser/strider_sheet_<service>_<sheet>_<i>.dxf` — every laser-cut part packed
  on the sheet stock, one set per cutting service and sheet (acrylic at Ponoko, the
  aluminium at SendCutSend), kerf-compensated where the service doesn't compensate
  itself (Ponoko's 0.2 mm; SendCutSend's files are nominal); exact lines and arcs as
  bulged `LWPOLYLINE`s, round holes as `CIRCLE`, layer `CUT`, mm.
  `strider_sheet_parts.csv` says which part is where.
- `build/laser/parts/` — the same laser-cut parts one DXF per different part
  (`<service>_<sheet>/<part>_x<qty>.dxf`, mm, blue `CUT` layer) and `order.csv`:
  what SendCutSend and Ponoko take (one part per file, the quantity at checkout).
- `build/bom.csv` / `bom.md` / `bom.json` — what to buy (quantities, packs,
  vendor links, whether each link was checked, estimated cost), what to
  print (filament) and what to cut (sheets); shims one line per thickness (the
  clamped 1.0 and 0.5 mm ones bought as DIN 433 washers, `hardware/shims.py`,
  `bom.SHIM_AS`).
- `build/ORDER.md` — the shopping list (`spiderpig/hardware/order.py`): a cart per
  vendor, each line the vendor's product page for the exact part
  (`spiderpig/hardware/sources.py`: the default build's every item, sourced
  2026-10-05, the makers' stores / McMaster-Carr / DigiKey / Mouser / Accu / MISUMI
  first, marketplaces last), an unpriced line estimated from a priced alternative
  (McMaster shows prices only behind a login), the uploads per cutting service, the
  prints per filament, shop supplies taken as on hand (filament, threadlocker: listed,
  not ordered), and what to check before ordering (a design-level servo torque limit,
  e.g. `klann_lego`'s 0.60 N·m, is a firmware note there and in the BOM).

Useful flags: `--module {single,double,decker,quad}` (legs per side; default: the
linkage's, `config.default_module`), `--side-only`, `--servo` (continuous-rotation servos
only; default `sts3215`), `--pillar` / `--pin` / `--crank` (constructions; the default is
`--pin chicago --pillar standoff --crank bolt`, below; the one other is `--crank
bolt_round`, TrotBot's heel and toe's; the other constructions were removed on 2026-10-07
and naming one fails with its replacement, `config.REMOVED_CONSTRUCTIONS`), `--sheet`,
`--thickness` (measure your sheet: acrylic varies by up to 8 %), `--kerf`,
`--no-dxf`. A mechanism (`--linkage hoecken`, `parallelogram_lift`, ...) needs
no `--module` or `--side-only`: it is its one module and one side, for `build`,
`explain`, `view`, `audit` and `bake` alike; `spiderpig report --linkages
parallelogram_lift watt_table_lift` compares mechanisms by their output numbers
(stroke, straightness, rotation) as it compares walkers by their foot paths.

`spiderpig export --linkage trotbot_heel --formats step stl print dxf bom glb mjcf`
writes every format at once (the build options as for `spiderpig build`, or a stored
design's id, into the store's `exports/` or `--out`; its `dxf` is the packed sheets:
the shopping list `ORDER.md` and the per-part DXFs `laser/parts/` come only from
`spiderpig build`), and `spiderpig sim --mjcf
out/trotbot_heel.xml --linkage trotbot_heel` (or `spiderpig sim <design-id>`) runs that
MJCF; `spiderpig report` covers every linkage, walkers and mechanisms, and says which
module it is planning (a quad takes the planner's minute: `SPIDERPIG_PLAN_SECONDS`
sets that CPU budget, 60 s by default; it bounds the search, not what a plan is, so it
doesn't enter the engine version stored designs are keyed by).
`spiderpig view <design-id>` serves the animated viewer for a design recorded in
the project store by the Python API or the MCP server (`docs/agentlib/API.md`),
with the drive and tune panels answering for that design. `spiderpig view
--linkage klann --module quad --pin bolt` (the build options above, as for
`spiderpig build`) resolves that design into the store first and shows it, so a
CLI build can be looked at without writing a spec. `build`, `explain` and `view`
warn (on stderr) when `--thickness` is more than 12 % off the sheet's nominal, as
the API's `resolve` does.

## How it is built

The linkage is planar; the design (sympy) says only where the joints are
over the crank cycle. `spiderpig/fabricate.py` then *rationalizes* it, one
functional group at a time (see `spiderpig/construction/base.py`):

- **servo drive** — an STS3215 stands on the inner frame plate, output face
  down; its horn turns in a hole in the plate;
- **crank** — a built-up crankshaft bolted to the horn: every b1 sweeps across the
  crank axis, so the crank reaches each b1 only along its crankpin, with webs in the
  layers either side. The default (`--crank bolt`, walkers and mechanisms alike,
  `config.DEFAULT_CRANKS`) is laser-cut from 0.100 in 6061-T6: every web one aluminium
  plate, every crankpin and journal a stock M3 x 5.5 AF steel hex standoff whose ends sit
  in hex pockets of the webs, an M3 button head and wide washer into each end, the riders
  turning on a printed sleeve over the hex, a round M3 standoff as the journal stub; the
  chain that ends in the hub plate is capped by it (the hub plate, horn, servo and inner
  plate go on as one unit). Each default design's layers and height are in
  [docs/agentlib/DESIGNS.md](docs/agentlib/DESIGNS.md). `--crank bolt_round` puts round
  standoffs, clamped by friction, in place of the hex (TrotBot's heel and toe, where the
  hex's sleeve doesn't clear b7); an acrylic crank sheet is refused;
- **pillars** — the frame pivots (`--pillar standoff`, the default): a 6 mm round standoff
  column from the outer plate to the inner, a button head and washer through each plate
  (no glue), the links turning on the standoff, a printed ring in every other layer. A
  column one stock length fills is a goBILDA 1501 aluminium standoff (M4; in 3 mm
  layers 12, 18, 24, 27, 30, 36, 42, 48, 54 or 60 mm); any other is one MISUMI NETRF6
  steel standoff made to its length (0.1 mm steps, M3 ends): never spliced;
- **pins** — the pivots between links: an M3 Chicago screw (a 4 mm barrel through
  the stack, a screw driven into it from above until it bottoms), printed spacer rings,
  one printed head spacer per end taking up the barrel's fixed length, the lowest link
  bonded to the barrel with epoxy (`--pin chicago`, the default: the axial play set by
  the barrel length to 0.05-0.15 mm, a stronger shaft than the rod, nothing to cut, and
  it comes apart; barrels in 1 mm steps from 4 to 16 mm, then 18-80, at most 23 mm on the
  Strider; the audit reports every link's tilt; `spiderpig/construction/pivots/`);
- **links and frame plates** — laser-cut, holes cut for everything above.

Each group first *claims* the space it needs, per 3 mm layer and relative
to the link layers; the planner (`spiderpig/stack/`) finds link layers where no
two groups' claims ever meet over the whole crank cycle. Then each group
builds its parts inside its claims, which a contract test checks. So the
parts can't collide, and a construction that can't fit is an error, never
broken geometry.

The robot is two mirror-image sides with their servos back to back on
aluminium centre plates, tied into one frame by four chains of 6 mm round M3 standoffs
(an M3 button head up through each inner plate, an M3 set screw through the centre
plates joining each pair; no glue). An electronics deck (ESP32 servo driver, 2S LiPo,
charger, protection board, switch) sits between the inner plates over the servos.
`construction/assembly.py`'s `ROBOT_ORDER` and each construction's `assembly` hook are
the order every fastener can be driven in; `spiderpig guide` draws it as `ASSEMBLY.pdf`.

## Test

```bash
mise run test-planner   # a module tier: linkage, planner, construction, hardware, strength,
                        # api, sim, server (seconds each on the fabrication cache)
mise run test-quick     # every module's fast tier: -m 'not slow and not e2e', xdist -n 4
mise run test-viewer    # the viewer's typecheck and vitest
mise run remote-test    # the full suite on the remote runner (AGENTS.md)
mise run gate -- compare ~/.cache/spiderpig/gate/next-ba41c10   # did a product edit move a part?
```

[docs/agentlib/TESTING.md](docs/agentlib/TESTING.md) has the tiers, markers, the caches, the
recorded fixtures and the identity gate. Unit tests cover the symbolic core (reference foot
values, rigidity, phase as a time shift), the layer planner (an independent brute force and
a full-cycle re-check), each construction through small seams (`tests/test_seam_*.py`, no
fabrication), the construction contract (every part inside its claims, for every module),
clashes, the robot frame, the BOM, the glTF bake and STEP / STL / DXF emission. `-m e2e`
runs the Playwright viewer tests. `mise run audit` checks a design end to end.

## Layout

```
spiderpig/                   # the checkout
├── mise.toml                # tool versions (python/uv/node) + tasks (view/build/bake/test/lint/audit/...)
├── pyproject.toml           # the installable package (hatchling); `spiderpig` console script
├── spiderpig/               # the Python package: engine, agent API, CLI, server, built viewer
│   ├── cli.py               # `spiderpig <command>`: build | bake | audit | explain | tune | sim | export | report | mcp | view
│   ├── build.py             # fabrication CLI (STEP/STL/DXF/BOM)
│   ├── bake.py              # the viewer's animated .glb bake (cached in the store's bakes/)
│   ├── linkage/             # the symbolic engine (engine.py), stage checks (checks.py), leg module templates (assembly.py)
│   ├── linkages/            # one module per linkage family (Klann, Strider, Jansen, ...)
│   ├── mechanism.py         # Pose, Joint, Body, Mechanism, MechanismTemplate
│   ├── stack/               # the layer planner over claims: geometry, topology, plan, search, plan_z, verify
│   ├── config.py            # BuildConfig (what to build and how, validated); the shared CLI / query arguments
│   ├── fabricate.py         # design a side, fabricate the robot
│   ├── fabcache.py, keys.py, uptodate.py   # the fabrication cache, its keys, the build skip
│   ├── construction/        # the groups: axle, crank/ (the bolt crank), pivots/ (standoff, chicago),
│   │                        # route, plates, robot, chassis, deck; the contract check
│   ├── servos/              # servo data (spec, catalog), drive group, models, CAD cache
│   ├── hardware/            # catalog, sources (product pages), the screw table, masses, BOM, ORDER.md
│   ├── materials.py, manufacture.py   # sheets per part; the cutting services' rules
│   ├── strength.py          # joint and link safety factors at the sim's loads
│   ├── walk.py              # quasi-static walking model (/api/walk, the viewer's drive mode)
│   ├── sim/                 # MuJoCo model of the fabricated robot and its runner
│   ├── shapes.py, rounding.py, mesh.py   # part primitives; tie-stable numbers; meshing
│   ├── layout.py            # 2D section + rectpack + ezdxf: packed sheets and per-part DXFs
│   ├── tools/               # audit.py (`mise run audit`), tune.py, sim_walk.py, report.py, dev.py, remote.py
│   ├── server/              # the viewer's FastAPI app + watchfiles live-reload
│   ├── viewer/dist/         # the built viewer (gitignored; `mise run viewer-build`; ships in the wheel)
│   ├── spec.py, api/, store.py, …   # the agent-facing API and store (docs/agentlib/API.md)
│   └── mcp/                 # the MCP server over it
├── viewer/                  # Vite + TypeScript three.js client sources (never ship); src/drive/
├── docs/                    # ARCHITECTURE.md; agentlib/ (API, TESTING, ROADMAP, DESIGNS, DECISIONS); history/
└── tests/                   # unit tests (module tiers, seams), tests/e2e/ for Playwright, gate/, doc_check.py
```

## Customising the linkage

Each linkage's proportions are exact rationals in its module under
`spiderpig/linkages/` (Klann: `spiderpig/linkages/klann.py`, lengths as multiples of `OA`,
angles in degrees). They are symbols in the compiled program, so the program
never needs re-deriving: change a value (or pass `--proportion NAME=VALUE`)
and every link length, the foot path, the stack plan and the parts follow.
Re-run `spiderpig build` (and `mise run audit`) to regenerate and check STEP/STL/DXF.

# Walker three.js viewer

Lightweight visual validator. The fabrication pipeline bakes a single
self-contained `.glb` per design (every fabricated part: laser-cut plates,
printed axles and crank, the servos) with the animation embedded as a glTF
clip; the browser plays it via three.js.

## Design

Three parts:

1. **Bake** (`spiderpig/bake.py`, `spiderpig bake`): one `.glb` per design (a
   `spiderpig.config.BuildConfig`) under `<store>/bakes/<config.key>.glb` (the
   project store, `.spiderpig/`). Parts come from `fabricate.fabricate`
   (build123d, world coordinates at `t = 0`); animation channels per body
   are sampled from the `MechanismTemplate` (`construction.robot.robot_template`
   for the whole robot). Bodies of one class whose parts are congruent under
   a planar motion plus a Z shift share one mesh (checked from exact B-rep
   mass properties): the legs' link plates, and the mirrored right side's
   plates, but not a mirrored part that isn't symmetric about its mid-plane.
2. **Server** (`../spiderpig/server/app.py`, FastAPI + uvicorn): exposes
   `/api/modes` and `/api/glb/{mode_id}`, baking a mode on first request
   (or when its cached `.glb` is older than the Python sources); mounts the
   built viewer bundle (`../spiderpig/viewer/dist`, Vite's `outDir`: package
   data a release wheel ships) as static files. A `watchfiles` watcher
   re-bakes on any `.py` change of the package and broadcasts `reload` over
   `/ws`.
3. **Client** (`index.html` + `src/*.ts`): Vite + TypeScript + three.js
   (npm). `loader.ts` fetches `/api/glb/<mode>`, plays the embedded
   `AnimationClip`, and overlays the foot path stashed in
   `scene.extras.foot_path`. On `reload` from `/ws`, the GLB is hot-swapped
   without a full page reload.

## Modes

A `/api/glb/{mode}` id is a module and a side or the robot (`spiderpig.server.app.MODES`);
the query's `module=` applies to the first two, the rest are the ids old URLs
use, and `/api/modes` lists the ones the dropdown offers with their labels.

| id (server / URL) | what |
|---|---|
| `robot` (default) | both mirror-image sides (`L.` / `R.`), `module` legs per side (quad), chassis between the servos |
| `side` | one side of `module` (quad) |
| `klann` | one side, one leg |
| `double` | one side, mirrored pair |
| `decker` | one side, two legs on one crankshaft |
| `double_double` | one side, four legs |
| `side` | one side of `module` (a one-sided stored design) |

`spiderpig bake --module single` bakes the robot with
another module per side, `--side` one side of it.

## Orientation and materials

The linkage moves in model XY with +Y up; the layer stack runs along model
Z. The glTF root node `walker` turns +90° about X (model +Y -> world +Z,
the stack horizontal along world Y) and lifts the gait's lowest point onto
`z = 0`; bodies are its animated children. The viewer's camera is Z-up.

Materials follow `Body.fab`: laser-cut plates are translucent acrylic
(`acrylic` for the leg links, orange-tinted `acrylic_frame` for the frame
and chassis plates), `printed` parts are opaque violet, and purchased parts
are `servo` (dark) or `metal` (horns, screws). Node extras carry
`{fab, rigid_with, bom, body}` (three.js exposes them as `userData`; node
names lose their `.` to three.js name sanitizing, `body` keeps the original).

## Run

From the repo root:

```bash
mise run view             # FastAPI + Vite (HMR); URL printed in the banner
```

Edit `.ts` → instant HMR. Edit `.py` → re-bake → viewer hot-swaps the GLB.

Single-port (built bundle):

```bash
mise run viewer-build
uv run uvicorn spiderpig.server.app:app --port 8000
```

## UI

- **Slider** — scrub to any phase instantly.
- **Play / Pause** — plays the clip in real time.
- **Mode dropdown** — the server's modes (`/api/modes`), robot first, of the
  tune panel's linkage.
- **Red loop** — leg 0's (first) foot trail over one cycle.
- **Orbit** — mouse drag to rotate; wheel to zoom; right-drag to pan.
- **Deep links** — `?mode=klann&view=side&t=0.3` picks the mode, a camera
  preset (`three-quarter`, `side`, `front`, `top`) and a paused clip time;
  `?design=<id>` a stored design (what `spiderpig view` opens): its glb, its
  mode (`robot`, or `side` for a one-sided design) and its linkage, module,
  phases and proportions in the tune panel (`/api/design/{id}`); every
  `/api/glb` and `/api/walk` query then carries the id, so the server starts
  from the design's config (servo, sheet, constructions included).

The page renders on demand (while playing, orbiting, or after a change), so
a paused viewer is idle. `window.__viewer` exposes the mixer, `seek(t)`,
`setView(view)`, `loadMode(id)` and `drive` (below) for the e2e tests.

## Drive mode and tuning (`src/drive/`)

Drive the robot over a ground plane to see how it walks, and tune its
geometry with an instant preview. The walking model is the shared SPEC one
(quasi-static, no physics engine): each frame the robot rests on the face of
its feet's lower convex hull under its centre of mass, and moves so that the
feet on the ground don't slide (least squares; the residual is the *slip*).

- **Drive panel** (top right, *drive mode*): the two sides' cranks turn
  independently at input × the servo's max rpm (× *speed*, with an
  acceleration limit). *tank*: W/S left side, ↑/↓ right side (gamepad: the
  sticks' Y). *arcade*: W/S throttle, A/D turn (gamepad: left stick).
  "Forward" is the cranks' design direction; left/right are the robot's
  as it walks. *R − L phase* offsets the right side's crank. Camera
  *chase* / *follow* / *free* (orbit works in all), *support + COM* overlay
  (support polygon, COM and its drop, green or red), contact feet (green
  dots), trail, *reset*.
- **HUD**: speed along the heading, yaw rate, height, pitch, roll, contacts,
  slip, stability margin, the cranks' rpm and relative phase, the last
  revolution (net distance and path, turn, bob, pitch / roll range, slip,
  min margin, tipping share), the model's straight-walk stride; one
  sparkline of a chosen metric; a red banner when tipping.
- **Tune panel** (top left): the linkage (`/api/linkages`: Klann, Jansen,
  Strider, ...) and its modules, per-leg phases, a slider per parameter of
  the linkage (lengths 0.5–1.5 × the default, angles ± 45°). Switching the
  linkage rebuilds the sliders and re-bakes the robot. Changes query
  `/api/walk` (debounced) and switch to a stick-figure preview of both sides
  that drives with the same model — instant. *Rebuild parts* bakes
  `/api/glb/robot?<params>` (a 422 shows its detail) and drives the full
  model again; *Reset to defaults*.

Walking data comes from the glb's `walker` extras `drive`, or from
`/api/walk` for the glb's design when it has none. Drive mode renders
continuously. Deep links: `?drive=1`, `?scheme=arcade`, `?linkage=jansen`,
`?tune=1&module=quad&phases=0,180,90,270&p.DF=2.7`.

## Files

```
viewer/
├── index.html
├── package.json, tsconfig.json, vite.config.ts
├── src/
│   ├── main.ts        # entry; render loop, deep links, window.__viewer
│   ├── scene.ts       # renderer, camera + view presets, lights, grid, orbit
│   ├── loader.ts      # GLTFLoader + foot-path overlay
│   ├── controls.ts    # slider / play / mode dropdown wiring
│   ├── live-reload.ts # /ws client → re-load GLB on rebake
│   ├── types.ts
│   ├── style.css
│   └── drive/
│       ├── model.ts   # SPEC walking model: support, no-slip motion, metrics
│       ├── sim.ts     # keyboard / gamepad input, crank rates, pose integration
│       ├── view.ts    # body pose, per-side animation, overlays, stick figure, camera
│       └── index.ts   # drive + tune panels (lil-gui), data sources, deep links
└── node_modules/      # gitignored; never ships
../spiderpig/viewer/dist/   # gitignored — `mise run viewer-build` output (Vite's outDir): package data
```

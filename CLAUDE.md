# CLAUDE.md — pointers for future agents

This file captures non-obvious things you'll want to know before diving into
the code. Keep it terse.
For the long-form walkthrough (background, every stage, limitations and risks,
doc drift), see `docs/ARCHITECTURE.md`.

## How to run things

Tasks live in `mise.toml`:

```bash
mise run view       # FastAPI + Vite (HMR); URL printed in startup banner
mise run bake       # bake <store>/bakes/<design>.glb (the project store, .spiderpig/)
mise run test-quick # the quick tier: -m "not slow", xdist -n 4 (~3.5 min; iterate with this)
mise run remote-test               # the full suite on ao-server, xdist (~6 min, not ~45)
mise run remote-audit              # modules audited at once there (~3 min, not ~8): the
                                   # Strider's four modules (the quad plans since max_top 60);
                                   # --linkage L: that linkage's four
mise run remote -- uv run python -m spiderpig.cli sim   # sims/bakes/anything there
mise run test       # pytest, serial (runs viewer-build first; -m e2e for browser tests)
mise run build      # STEP/STL/DXF -> build/
mise run lint       # ruff check
mise run audit      # do the parts physically fit? (see docs/audit/AUDIT.md)
mise run explain    # each pipeline stage's verdict on a design
mise run tune       # search crank phases for a smoother walk
mise run sim        # MuJoCo
mise run report     # compare every linkage -> build/linkages.json
mise run kill       # stop dev servers spawned from THIS worktree
mise run kill-port -- --port 5173   # force-stop whoever is on a port (orphan recovery)
mise run clean
```

`build`, `bake`, `audit`, `explain`, `tune`, `sim`, `export`, `report`, `mcp` and `view` are
the subcommands of the `spiderpig` console script (`spiderpig/cli.py`; `spiderpig
<command> --help`; from a checkout `uv run python -m spiderpig.cli <command>`, or
`mise run <command> -- <options>`, which passes options through); the tools' own
modules (`spiderpig/build.py`, `spiderpig/bake.py`, `spiderpig/explain.py`,
`spiderpig/tools/*.py`, `spiderpig/mcp/`, `spiderpig/view.py`) are what it runs. For
example:

```bash
spiderpig bake --module quad --frames 120
spiderpig bake --module single --side             # one side only
spiderpig bake --linkage jansen --module double   # another linkage
spiderpig bake --side --linkage hoecken           # a mechanism: one side
```

What every tool builds is a `spiderpig.config.BuildConfig` (linkage, module,
robot or side, phases, proportions, servo, constructions, sheet), which
validates itself; `--linkage` / `--module` / `--phases` / `--proportion
NAME=VALUE` and the build options are shared by `build.py`, `bake.py`,
`explain.py` and the tools (`config.add_design_args` / `add_build_args` /
`config_from_args`); the server takes `linkage=`, `module=`, `phases=`,
`p.NAME=` (`config.design_from_query`). `--module` defaults to `None` and
`config_from_args(robot=None)` fills both in from the linkage
(`config.default_module` / `default_robot`: the linkage's own `default_module` when
it names one (Strider's `double`), else a walker's `quad`, and the robot; a
mechanism's `single` and one side), so a CLI never needs `--module single
--side-only` for a mechanism. The project's default design is `linkage.DEFAULT`
(Strider) in its default module: `BuildConfig()` is the Strider double, built `--pin chicago
--pillar standoff --crank bolt` (since 2026-10-03; before, `--pillar printed --crank keyed`).
The **bolt crank** (`construction/crank.py` `BoltCrank`). On an aluminium crank sheet (the
default 0.100 in 6061-T6) it resolves to **single plates** (`BoltCrank.for_sheet` / `resolve(ctx)`, since
2026-10-04, `_WebPlates`): every web one aluminium plate; every crankpin (since the merge of
2026-10-04 a hex standoff in hex pockets, below; this paragraph is the round one, `--crank
bolt_round`) a goBILDA 1501 round
standoff (6 mm OD, stock lengths in 1-2 mm steps, two joined by a stud past 60 mm) clamped
between its two webs by an M4 button head into each end (heads in the clearance gaps beyond
the webs; DIN 988 shims and a "spacer" gap take up the standoff's length past the layers);
the riders turn on the standoff, as on a pillar. Two chains share a web, or a **journal
standoff** on O joins them (the same clamp); the last chain's top web is the hub plate, the
horn screws come up through it from below (heads in the gap under it). The stub: an M3 round
standoff screwed to a stub plate under the lowest web (its head in that web's hole), in a
6.6 mm hole (+/-0.3) of the outer plate. No plate stack anywhere, so no bond and no lock
screw. Why not the joinery plan's M6 bolt: ISO 4014's 18 mm thread leaves 4-5 bare layers
under every run unless the bolt is cut, and nothing is cut. Router rules (`JointRules`:
`j_last`, `j_spans`, `gap_head`, `horn_heads`): the router's gap pieces (`gap_pieces`,
blocked by other groups' heads and washers in a gap, keyed by its slot `layer + 0.5`) keep a
route off gaps a screw head can't use, which is what makes these plans fast (Strider double
15 layers / 66.1 mm in ~3 s, `klann_lego` quad 14 / 58.2 mm in ~3 s, on the thinnest sheets). A single-plate crank
plans its design `heads="gap_sink"` (`stack.HEADS_ORDER`, since 2026-10-04 evening): in gaps
first (a sunk crank head would stand in a rider's layer), and only when that search gives
up (below; a full gap search that finds nothing isn't followed by it: no design of the
2026-10-04 sweep planned that way) with the pivots' heads sunk and the crank's (and the drive's, `stack.GAP_GROUPS`) still in
their gaps (`heads_claims(keep=...)`, `finalize(sink_all_but=...)`: an axle crossing a crank
gap carries its washers there). TrotBot's heel and toe plan only that way (the heel single: 14 layers in ~6
s; in gaps no route keeps the crankpin's washers clear of the pins' caps, and the gap search
gives up after `stack.GIVE_UP` such layerings with no plan, ~3 s, instead of its 60 s
deadline). At a leaf the planner routes again round the gaps the plan has where the
crankpin's run washers met another group's (`JointRules.gap_washer`, `_Search._washer_blocks`,
`CrankRouter.washer_bit`: bits only the leaf sets, never the search). Its joint
is a friction clamp, rated in `_web_capacity` (UNVERIFIED coefficients). On an acrylic crank
sheet (stored designs before 2026-10-04) it is the two-plate stack crank: M6 bolts, head and
nylock in hex pockets, cemented stacks (`_BoltPlates`, the `two_layer_*`, `tip`,
`low_count`, `share_stack` rules). Mechanisms default to it too (`config.DEFAULT_CRANKS`,
2026-10-04): a short crank's top screw head over the hub plate sits in a pocket of the
printed horn spacer (`DriveGroup.realize`). The construction a design gets is data
(`config.default_crank`: `LINKAGE_CRANKS` per linkage, else `DEFAULT_CRANKS` per kind). The
mechanisms' sweep of 2026-10-04 (`docs/audit/STRENGTH.md`, on the round standoff) left
`hoecken_pantograph` and `dwell_rocker` on the keyed crank; with the hex crank both plan (9
layers: the horn spacer takes the short crank's head, `hub_head_need` / `hub_capped`), so
since the merge `LINKAGE_CRANKS` holds TrotBot's heel and toe on `bolt_round` (b7 passes
crankpin J1 at 10.2 mm, the hex's 8.5 mm sleeve needs 11.2; the round 6 mm standoff plans,
14 layers) and `klann_lego` (its 6061 b1's crank bore is 8.5 mm from pin C: the hex's 8.8 mm
bore leaves a 2.02 mm web, a cut-rule error; the round's 6.3 mm, 3.27), and
`config.MODULE_CRANKS` (per linkage and module, first) the Strider's decker and quad on
`bolt_round` (no hex plan in 600 CPU s: its sleeve's post blocks the links passing the
crankpins; the round plans them in 17 and 25 layers). An aluminium link riding a crankpin
grows a boss round its bore to 1 x its thickness of web where its width leaves less
(`plates.rider_bosses`, `RIDER_BOSS_T`; claimed, so the planner sees it). The demo Klann quad on the XL330 plans with the hex crank only after ~6 CPU
minutes (15 layers; the 60 s default gives up), the STS3215 in 3 s.
Since 2026-10-04 `StackSpec.max_top` is 60.
**Hex-standoff crankpins** (the user's decision of 2026-10-04, `BoltCrank.pin="hex"`, the
default; the round friction clamp above is `--crank bolt_round`): every crankpin and journal
is a stock M3 x 5.5 AF F-F steel hex standoff (`crank_catalog.HEX_M3_LENGTHS`) whose ends sit
in hex pockets of the single webs (`BoltCrank.hex_cut`: 0.1 mm over the AF, a dog-bone relief
of the service's 0.8 mm inside radius through each corner, so the flats stay whole); an M3
button head and a DIN 9021 washer into each end retain the plates; where the stock length
stands past a plate (at most `protrude_max` 1.2 mm per end, each stack inside a 4 mm gap) a
printed hex-bore collar takes it up, or both ends sit up to `recess_max` 0.3 mm inside their
pockets (`fit_hex`). The riders turn on a printed 8.5 mm sleeve over the hex (`rider_d`:
their holes), printed rings fill the gaps along a run. Rated by `hex_bearing_nm` in the
plate's own depth at min(plate, 300 MPa steel) yield (`hex_capacity`). The crank sheet is
**0.100 in 6061-T6** (`config.crank_sheet`, `materials.thinnest_sheet("crank")`: the
thinnest whose pockets hold 2 x 0.85 N·m at SF 2 with the recess; 5052 would need 0.125 in,
thicker than its 3 mm layer, which moved the pillars' columns off stock lengths). Horn
screws take DIN 988 shims under the head where a stock length is too long (the XL430,
`horn_fit_web`); the horn holes and the hub's and webs' rims keep SendCutSend's minimum hole
and 2 t edge distance; a crankpin within a head's reach of the horn's rim makes the horn
spacer a layer thicker (`BoltCrank.hub_head_need`, `DriveGroup.spacer`) or, wholly under it,
is capped by it (`hub_capped`). **Since the assembly audit of 2026-10-04 the chain that ends
in the hub plate is always capped** (`BoltCrank.hub_screw` False: no screw over the hub
plate, the hub plate held by the horn screws; `--crank bolt_hub_screw` keeps the screw):
no order drove that screw with the horn screws coming up through the hub plate from below,
so the hub plate, horn, servo and inner plate go on as one unit
(`construction.robot.ASSEMBLY`, the whole robot's order, which the pivots' and the crank's
docstrings defer to). Merged (2026-10-04, with gap_sink and the body plates): the
Strider double 15 layers / 75.3 mm (72.8 since the capped hub chain: the gap over layer 13
went), single 11, the demo `klann` quad 14 / 76.8, the Hoecken pantograph and dwell rocker
9 each. The round standoff (`bolt_round`: the Strider decker and quad, TrotBot,
`klann_lego`) still screws over the hub plate: its friction clamp needs both screws, so
**those designs have no assembly order** (`_WebPlates.assembly_issue`, the crank note's
`assembly`, an `assembly:` audit error since the second assembly audit of 2026-10-04; a
user decision: a hex plan for them, a hub joint fastened from the horn side, or leaving
them out of the first build). **Axial retention of the capped chain** (that audit): its
printed sleeve is a light press on the hex (`BoltCrank.capped_press` 0.1 mm), pushed onto
its lower web, so the sleeve, caught between the plates, carries the standoff; the crank
body stops toward the outer plate on a printed **thrust sleeve** round the stub
(`stub_thrust`, 8.5 mm, its end 0.1 mm over the outer plate: claimed in the stub's layers
and gaps, `stub_thrust_r`) and toward the hub on the capped sleeve, and the capped hex is
rated in the hub's depth less that 0.1 (2.44 mm). `--crank bolt_unretained` keeps the
build before (both slide, no stop). (An M3 retainer under the outer plate, which that audit
proposed, stops the stub moving *in*, which the capped sleeve already does, not out.)

**Clearance gaps, layer thicknesses, per-part sheets** (2026-10-04, `stack.finalize`,
`spiderpig/materials.py`). A fastener's head or nut beside a link (a Chicago screw's, a rod
pin's clip, the servo's screws) is a *clearance shape* (`Placed.gap`, `height`, `toward`):
it sits in a thin gap over/under its link's layer, or sinks into the layer beside it where
nothing there is in its way (the old full-layer head). An axle claims its washers through
every gap it crosses. `StackSpec.heads`: "sink" (every head in a layer, no gaps), "gap", or
"best" (the default: heads sunk; in gaps only when that finds no plan, the user's rule of
2026-10-04, which restored the suite's time; a single-plate crank plans "gap_sink", above). The plan's z (`stack.finalize`, also what re-makes a stored plan):
the heads that can't sink keep gaps, each gap sized to its tallest head (a gap a plate stack
runs through is a thin sheet's thickness, `materials.gap_options`; else any 0.1 mm of
washers and DIN 988 shims up to 4 mm), each layer as thick as its thickest plate
(`Placed.sheet`, `StackSpec.link_t`, `frame_t`), every claim made again at that z (the
crank's bolts, the stub, the Chicago barrels and the standoff segments are checked at it,
`Layout.final`), and a gap thickened (`_thicker_gaps`) when that lets a stock part fit; a
layering that doesn't build there is a dead end for the search (`PlanReject`).
`Layout.z`/`gap_z`/`slot_z`, `StackPlan.gaps`/`thick`/`heads`. Sheets: `BuildConfig.sheet`
(acrylic, the links, rings, deck), `frame_sheet` and `crank_sheet` (the thinnest that
passes: 5052 frame, 6061 crank, below), `link_sheets` (a Klann variant's foot link in 6061, and `klann_lego`'s
crank rider b1, the user's decision of 2026-10-04: `materials.LINK_SHEETS`); `Body.sheet`
carries it to the mass, the BOM (a line per sheet), the DXFs (`layout.save_sheets`: a set
per sheet) and the cut-rule review (`spiderpig/manufacture.py`: SendCutSend / Ponoko minimum
hole, edge distance, the web round every non-circular cut-out (`web`: to the edge, a hole or
another cut-out, since 2026-10-04), minimum part, inside-corner radius; levels below; every issue with its
`why` and `fix`, `manufacture.messages` / `summary`): the audit's problems and warnings, a
"cut rules" column and a per-part table in `audit.md` (and `manufacture` in `audit.json`),
`verify` standard's `manufacture.cut_rules` row (errors: the `manufacture` / `cut_rule`
failure) and soft `manufacture.warnings`, the build report's `cut_rules`
(`api.BuildReport.cut_rules`, the MCP build's too) and the design card's (`/api/design/{id}`
`cut_rules`, `api.cut_rules`: from the stored build, never fabricating). The Klann variants'
quads (`klann_patent`, `klann_lego`, `klann_long_legs`, `klann_high_step`) default to
phases 0,0,180,180 (`linkage.KLANN_QUAD`); the demo `klann` keeps the generic quad (at
0,0,180,180 its quad finds no plan with standoff pillars in 60000 nodes). The **standoff pillar**
(`construction/pivots/standoff.py`): goBILDA 1501 round 6 mm aluminium standoffs, an M4
button head and washer through each frame plate (no glue; the inner one's head is between
the robot's inner plates), printed spacer sleeves in every other layer (an 8.5 mm laser ring
is under both services' smallest part; every pivot's rings are printed now), spliced (an M4
stud) where a column is longer than 60 mm, at the fewest splice layers that leave every
segment a length goBILDA sells, placed where the beam's moment under unit loads is least; a
splice "plate" is a stack of DIN 988 steel shims (`splice_shims`), rated at the **supported
splice**'s clamp (`splice_nm` 1.0 N·m, the segments turned together in soft-jaw pliers;
`splice_capacity_nmm` 1.42 N·m to open; the user's decision 2 of 2026-10-04, accepted
UNVERIFIED and to be tested on the first build; it was the end screws' preload on an acrylic
ring, then 0.4 N·m finger tight). A segment up to 2 mm short of its gap takes DIN 988 end
shims under the face over it (in the clearance gap there, its washers trimmed, else in the
spacer layer; the end screw or stud sized for the plate plus shims; `shims_mm` in the note). Strength: a beam per bay between its supports
(the plates: a splice is a joint with its gapping moment checked, not a support). Chicago
screw pins between the links, from the pivot review of 2026-10-03:
`construction/pivots/chicago.py` has the table, why and how to assemble (a planner rule
since: a pin's links must fit a stock barrel, `ChicagoShaft.column`); `--pin rod`,
`--pin ptfe`, `--pin printed` (the zero-hardware snap pin) stay selectable, as do
`--crank keyed` / `keyed_float` / `printed` and `--pillar printed`; the audit reports each
link's tilt, `construction/wobble.py`, and every joint's strength (`docs/audit/STRENGTH.md`);
Klann stays registered as the wobbly demo (`--linkage klann`: its quad);
`audit` and `report` take mechanisms too. A bake is cached as
`<store>/bakes/<config.key>.glb` (`bake.default_bake_dir()`: the project
store, `$SPIDERPIG_STORE` else `./.spiderpig`; `strider_double_robot.glb`, a hash
suffix for a non-default design). A design with no layer plan (the planner says why, e.g.
TrotBot's heel scaled back to its drawing's 7 mm unit, `p.unit=7`) bakes a
422; its `/api/walk` still works.

**Thinnest sheet per part** (2026-10-04, the user's rule: no weight budget, every plate as
thin as strength and the service allow): `materials.thinnest_sheet(role)` takes the
thinnest stock 5052 that passes the role's checks (`materials.ROLES`: the out-of-plane
bending at a pin's clamped end at 155 N jam, SF 2; and the role's smallest hole against
SendCutSend's minimum hole = thickness). Frame plates 0.080 in (`al5052_2mm`: 0.100 in and
up can't take the servo's 2.4 mm holes, 0.063 in fails the pillar end), crank plates
0.100 in 6061-T6 (`al6061_2p5mm`, since the hex crankpins: their pockets' SF 2); `BuildConfig` pins both and `tests/test_joinery.py` checks it.
The centre plates take the thinnest that seats the most rear screws
(`chassis.centre_sheet`: 0.090 in on the STS3215). A crank plate thinner than its 3 mm layer
sits on the layer's floor, the hub plate at its top (`BoltCrank.plate_z`). Cut rules
(`spiderpig/manufacture.py`): a hole closer than 1 x the thickness to an edge or hole in
metal, or under the service's minimum hole, is an **error** (the audit fails); under 2 x a
warning. Standoff pillar splices are rated at a 1.0 N·m clamp (segments turned together in
soft-jaw pliers, `StandoffAxle.splice_nm`, UNVERIFIED; 0.4 N·m finger tight failed the
Strider quad's jam; the user accepted it 2026-10-04: to be tested on the first build).
The body plates meet the 2 x t rule without warnings on the default designs: every round
hole in a frame plate gets a boss of 2 t (`plates.boss_web`), the frame ties move along the
servo until their holes are 2 t off the servo's front and rear screw holes and recesses
(`chassis.tie_locals` / `tie_neighbours`, which the underside reads too), the centre
plates' outline keeps 2 t round each rear screw recess (`chassis.recess_wall`) and their
bump reliefs have 1 mm corners (`RELIEF_CORNER`). Centre plates 0.090 in (decision 4).
**Bus plugs** (assembly audit, 2026-10-04): the servo's sockets are in the connector housing
on the rear face, screwed flat to the centre plates, so `ServoSpec.bus_ports` (the
STS3215's: 5264 3P, opening "end", UNVERIFIED: measure a servo and a plug before cutting)
gives the plates an open slot from the housing to their far edge through every plate the
plugs stand in (`chassis._port_slots`); `centre_plates` counts the two servos' plugs (at
the same place from either side), the rear holes within 2 t of the slot are dropped (each
servo keeps one rear screw, its near hole), the ties beside it move out to 2 t
(`tie_locals`), and `centre_sheet` ranks the most screws, then the thinnest stack (0.090 in
x 4 over 0.080 x 5). `opening="pocket"` (no access) is kept to compare. Since the second
assembly audit of 2026-10-04 the STS3215's rear idler horn stays in the box (`Idler.fitted`;
`servos.model.unfit_idler` cuts it from the CAD model): its 21 mm square relief left the
near rear screw 1.1 mm of web; the plates clear the 6 mm boss (a round relief,
`Relief.round`, `chassis.RoundRelief`). Reliefs closer than the service's web are one cut
(`chassis._merge_close`: the SO-ARM100 model's pins relief, 0.31 mm off the slot, merged
into it) and a head recess that close opens into the relief (`_recess_bridges`). The
inner plate's front screws: a hole with less than `Params.servo_screw_web_t` (1.0) x t of
web to the horn's hole or a relief is left out (`DriveGroup.screw_web`): the STS3215 keeps
its two far ones (2.05 mm to the panel relief, now cut at the models' rectangle,
`mount.RELIEF_CUT_GROW` 0; the near ones had 1.01 mm); `servo_screw_web_t=0` keeps all four.

**Glue-free chassis, feet, link plates** (2026-10-04, the joinery plan). Frame ties
(`construction/chassis.py`): per tie and side a goBILDA 1501 standoff chain from the inner
plate to the centre plates (stock lengths, DIN 988 shims), an M4 button head up through the
inner plate from the leg side, an M4 set screw through the centre plates into both chains;
the centre plates are cut from the frame's aluminium and clamped by those studs (no
adhesive). The deck rails (`construction/deck.py`) are screwed to the inner plates (M3 from
the leg side into a nut trap in the printed rail), the deck plate has cable-tie slots. Every
screw that comes up through the inner plate from the leg side (the servo's four front
screws, the ties', the rails') is the drive group's claim: its head in the gap under the
plate (`DriveGroup.claims`, `chassis_screws`; the tie and rail positions are known before a
plan: `chassis.tie_points_ctx`, `deck.rail_screw_points`). The servo uses all four front
screws when the crank's hub sits a layer under the horn (`horn_layers`). Feet
(`construction/plates.py` `foot_sock`): a printed TPU 95A sock round each foot link's toe,
in its own layer, snapped into two notches; the sim's floor friction is 0.65
(`SimParams.friction`). Frame plates get a chord between pillars next to each other about
O (`plates.chords`, also in the underside). The audit's strength check rates every link
plate (`strength.link_rows`: net section at the most loaded hole, plus bending for a link of
three or more pins) at the sim's pin loads and names the aluminium sheet a link would need.

The viewer is a Vite + TypeScript app under `viewer/src/`. In dev, Vite
serves on a port derived from a CRC32 hash of the worktree path
(range 5500-5999) and proxies `/api` + `/ws` to FastAPI on a similarly
hash-derived port (8500-8999). This means parallel worktrees get unique
stable ports with **zero manual config** — just `mise run view` in each
and bookmark the URL printed in the banner. Override via env vars in
`mise.local.toml` (gitignored) when the auto-picked port collides:

```toml
# mise.local.toml — per-worktree, not committed
[env]
VITE_PORT = "5173"   # pin the main checkout to the canonical port
API_PORT  = "8000"
```

For single-port runs (e2e tests, prod-like), build first with
`mise run viewer-build`: Vite's `outDir` is `spiderpig/viewer/dist` (git-ignored
package data, what a release wheel ships); `spiderpig/server/app.py` mounts it
when it exists (override via `SPIDERPIG_VIEWER_DIST`; without a build `/`
answers 503 and the API still works). `spiderpig view <design>` serves that
app for a stored design with no Node on the machine (the wheel carries the
dist; `mise run release` builds both): see `docs/agentlib/API.md`, "View".

## Baking the glTF — performance profiler

`spiderpig/bake.py` (the viewer's bake) has a built-in stage-level profiler. It is **on by
default** and prints a summary table via `logging` at the end of every bake.

Flags:

| flag | default | purpose |
|---|---|---|
| `--profile / --no-profile` | on | stage timings + metrics summary |
| `--log-level LEVEL` | `INFO` | `DEBUG` for per-class tessellation and frame-sampling chatter |

(For a function-level profile run the script under `python -m cProfile`.)

Instrumented stages (keys in the summary table):

1. `1_reference_build` — freeze the template at `t=0`, rationalize (or
   reuse) the side design and its layer plan, fabricate every part
2. `2_mesh_share` — find bodies whose parts are exact translates of another
   (mirrored right-side plates, the legs' links) so they share one mesh
3. `2_tessellate_total` + `2_tessellate.<kind>` — OCCT tessellation, one
   mesh per shared shape (positions and indices only: the viewer shades flat)
3. `3_gltf_pack_geometry` — accessor/bufferview/material packing
4. `4_animation_sample_total` with three sub-timers (run once per bake now,
   not once per frame):
   - `4.1_template_build` — one-shot `MechanismTemplate` assembly (the
     linkage's program was compiled once per process by then)
   - `4.2_template_sample` — vectorized batched-BFS pose propagation
     over the whole `ts` array
   - `4.3_trs_batch` — batched planar rigid fit + quaternion hemisphere
     fix per body; hardware (`Body.rigid_with`) copies its host's motion
5. `5_gltf_nodes_channels` — glTF node + animation sampler/channel assembly
6. `6_foot_path_extra` — 64-sample foot path written to scene extras;
   `6b_drive_extra` — the drive data (feet over the crank cycle, COM, mass,
   servo rpm) on the root node `walker` for the viewer's drive mode
7. `7_serialize` — `pygltflib.GLTF2.save_binary`

Plus `bake_total` wrapping everything.

Metrics the summary reports: `n_frames`, `n_legs`, `n_bodies`, per-class
`verts.*` / `tris.*`, `blob_bytes`, `gltf_bytes`, `animation_channels`,
`accessors`, `n_meshes`, `peak_rss_mb`. Counters: `body_extract.calls`,
`body_extract.static` (bodies with no joints and no host), `mesh_shared`.

### Known hot stage

`1_reference_build` (OCCT parts, ~70%) and `2_tessellate_total` (~20%)
dominate; the frame loop is ~1%. Robot quad (two sides, 91 bodies, 33
meshes) ≈ 6.6 s.

Historical: the symbolic solve used to run per leg per call (per frame,
before `19e020e`), and substituted expressions grew to ~34k ops. The
straight-line program of each linkage (`spiderpig/linkage/engine.py`) is compiled once per
process; a leg's phase is a time shift. Don't reintroduce per-leg or
per-frame solves.

### How to extend

The profiler lives in `spiderpig/bake.py` as `_Profiler`. To add a new
bracket:

```python
with prof.timed("label"):
    ...
prof.bump("counter_name")
prof.set_metric("metric_key", value)
```

All output goes through `logging.getLogger("bake_gltf")` — do not revert to
`print`.

## Repository map

Everything Python is the `spiderpig` package (installable: `pyproject.toml`,
hatchling; `uv sync` installs it editable, `spiderpig` is its console script).
`tests/` and the viewer's TypeScript sources (`viewer/`) stay beside it.

| file | role |
|---|---|
| `spiderpig/linkage/` | the symbolic side, one package re-exporting everything. `engine.py`: compass-and-ruler helpers (`crank`, `circle_x_circle`, `extend`, `offset`), `Linkage` (a straight-line program over exact `params`, compiled once per linkage), the registry (`get` / `available(kind)`), `LegSolution` (mirror = reflect x at crank angle π − t), `scale_params`. `checks.py`: the stage checks (`check_steps`: every loop's margin and transmission angle; `check_output`: a mechanism's output against its promises). `assembly.py`: the generic leg template (bodies `coupler`, `b<k>` links, `conn`, `torso`; connections from shared joint names), composition (`combine_connectors`, `fuse_*`), `build_module_template(module, phases, params, linkage)` and `feet_of`. A walker has `feet`; a mechanism an `Output` (`output_check()`, promises enforced as `OutputError`) and maybe a second input (`inputs`, `crank_at`). |
| `spiderpig/linkages/` | one module per linkage family (Klann, Strider, Jansen, ...); each registers its `Linkage` (and variants). Auto-imported; Strider first (the default, `linkage.DEFAULT`; a linkage's `default_module` names the module it builds when none is asked for). `mechanisms.py`: building blocks (straight lines, lifts, xy, rockers), one side only; `tests/test_mechanisms.py`. |
| `spiderpig/explain.py` | prints each pipeline stage's verdict for a design (program checks, static facts, plan with its crank route and proof, or the stage's error and what would clear it) |
| `spiderpig/recommend.py` | what would clear a static or plan failure, checked by re-running the stage: the least practical scale of the linkage (`linkage.scale_params`), thinner `Params` parts within every construction's `dims()`, the default scale after a scale-down, printed pillars for a plan a bolt pillar's stock screw bounds; and, for a design that plans but misses a target that scales with the linkage (a mechanism's stroke or straightness, a walker's lift), `target_scale`: the least practical scale that meets it, measured again and planned (`api.advise`) |
| `spiderpig/mechanism.py` | `Body` / `Joint` / `Pose` / `Mechanism`; `MechanismTemplate` / `SampledPoses` for batched sampling (numpy 4x4s). All joints sit at z = 0: kinematics is planar. `Body.fab` / `bom_key` / `rigid_with`. |
| `spiderpig/stack.py` | the layer planner. Knows only **claims** (`Claim` -> `Placed` discs/pills per layer, relative to link layers; an `early` part checked as soon as a group's own links are placed), a `Router` (a group whose shape it chooses per layering: the crank), a `Topology` (links, axles as named points, points fixed to the crank) and sampled `Geometry` (distances are lower bounds that cover motion between samples). `StackProblem.solve()` (see "The planner" below); `verify_plan()` re-checks exhaustively on fresh sampling. |
| `spiderpig/construction/` | the rationalization: one **group** per functional part (`base.py` is the contract). `axle.py` (pillars + link pins: the claims, `AxleDims`, the `printed` snap axle), `crank.py` (routes, claims, the crankshafts: `BoltCrank`, the walkers' default, laser-cut two-plate web stacks keyed on M6 hex-bolt crankpins (head and nylock in hex pockets, riders on the plain shank, cemented segments, a standoff stub; `_BoltPlates` builds it); `KeyedCrank`, the mechanisms' default, keys every printed post to the web above with a brass hex standoff (pressed in, `key_fit`; `keyed_float` slides) and clamps each chain through a two-layer top web; `PrintedCrank` is the friction-only one; `hex_bearing_nm`, the one hex-in-socket bearing model), `route.py` (the crank's router: static facts, detours, the exact route per layering; `JointRules.two_layer_top` ends a keyed chain one layer higher; the bolt crank's `two_layer_bottom`, `tip`, `low_count`, `share_stack`, `inner_webs`, `bottom_layers`), `underside.py` (the body's underside: the envelope, ground clearance), `plates.py` (laser links + frame plates), `robot.py` (two mirrored sides, the frame ties' holes, the assembly), `chassis.py` (the servo frames in the plate plane, centre plates, rear screws, tie columns), `deck.py` (the electronics deck: a laser-cut plate on two printed rails spigoted into the inner plates, over the servos between the frames, nothing moving in its z band; ESP32 servo driver, 2S LiPo in a strapped cradle, IP2326 charger, HX-2S-JH20 BMS, toggle switch; catalog in `hardware/electronics.py`, items with a `mass_g`; `deck_clearance` proves it clear over the cycle; the sim's `payload_g` is 0 now), `contract.py` (parts inside claims), `envelope.py` (solids of claims). Registries in `__init__.py`. |
| `spiderpig/construction/pivots/` | metal-shaft pivots (`--pin` / `--pillar` keys; its docstring holds the hardware research): `standoff` (**the default pillar**: goBILDA 1501 round 6 mm aluminium standoffs, rings in every layer, spliced at a ring layer past 60 mm, an M4 screw through each frame plate (no glue); a cantilever where a link's sweep stops it short of one plate; the `column` hook refuses a column no stock segments fill; `standoff.py`), `chicago` (**the default pin**: an M3 Chicago screw's 4 mm barrel through the stack, rings, PTFE washer and DIN 988 shims taking up the barrel's fixed length, lowest link bonded to the barrel; pins only; `chicago.py`, whose docstring holds the pivot review's table), `chicago_bushing` (the same with igus GFM-0405-03 in the other links, printed sleeves), `rod` (3 mm rod, laser-cut spacer rings, Starlock clips, glued into the frame plates; `rod.py`; the default before the Chicago screw: 19 / 17 layers on the Strider double / demo Klann quad with the keyed crank (17 / 13 with `--crank printed`; a layer each for the chassis screws' heads under the inner plate since 2026-10-04) in seconds, the least play of the plain pivots, assembled bottom up; the BOM lists its cut lengths), `bolt` (M3 SHCS axle, rings, washer + nylock; a pillar clamps both plates, the nut end claims 2-3 layers, and a pin's head is unreachable once its lowest link is on, so it is not the default; `bolt.py`), `bearing` (MF63ZZ flanged bearing glued in each link, rod, printed sleeves; `insert.py`), `bushing` (igus GFM-0304-03 pressed in each link, same; `insert.py`), `ptfe` (the rod with a 3 x 4 mm PTFE tube liner pressed in each link; its limit the liner's 10 MPa bearing pressure; `ptfe.py`). Their claims fill every layer (`AxleDims.fill`: a rod can't neck, so `neck` is the narrowest ring or sleeve), flanges need a free face (`AxleDims.flange`, `flange_sides`), retainers come from the construction's `ends` hook. Catalog additions in `hardware/fastener_catalog.py`. |
| `spiderpig/servos/` | `ServoSpec` data (continuous-rotation servos only), the drive group (`mount.py`: servo on the inner frame plate, `DriveInterface` for the crank), models and CAD cache. |
| `spiderpig/hardware/` | purchasable-item catalog (`catalog.py`, data in `parts.py` and `servos/catalog.py`; the sheet helpers), the screw families (`fasteners.py`: heads, stock lengths, keys, solids), materials and exact mass properties (`mass.py`: the one density table, `material_of`, `part_props`) and the BOM (`bom.py`). |
| `spiderpig/config.py` | `BuildConfig`: what to build and how, validated on construction (the linkage's module, one phase per leg, the linkage's proportions; defaults dropped so a design has one config and one `key`), the shared CLI arguments and the server's query parsing. |
| `spiderpig/fabricate.py` | orchestration: `design_side()` (groups -> claims -> plan, cached; the robot's side is the side's design), `fabricate_side()`, `fabricate()` (the robot unless `robot=False`: the frame ties join at build time). |
| `spiderpig/shapes.py` | build123d primitives (disc, pill, plate, cuts incl. D-holes and rectangles) |
| `spiderpig/mesh.py` | `tessellate(part)` / `tessellate_many(parts)`: OCCT's incremental mesh of each part, skipping (and counting) faces the mesher leaves without a triangulation (the XL330's model has three); the triangles read out by OCCT's glTF writer (`RWGltf_CafWriter`, each part in a compound of its own) instead of node by node, to the same arrays (a fallback is logged); the bake's `_tessellate` and the MJCF's hulls both use it, so a purchased model never fails either. `export_stl`: build123d's, meshing once |
| `spiderpig/workers.py` | `submit(fn, *args)`: a module-level function of the package in a fresh `python -c` process (OCP holds the GIL; a fork after OCCT's thread pool deadlocks; multiprocessing's spawn re-imports the caller's `__main__`); `dump_shape` / `load_shape` (BinTools through a file). The export's glb/MJCF and grouping and verify's contract angles run in workers; `SPIDERPIG_WORKERS=0` keeps everything in-process. `docs/agentlib/PERF_EXPORT.md` has the measurements |
| `spiderpig/layout.py` | DXF sheets of every laser-cut body, kerf-compensated; errors instead of dropping parts |
| `spiderpig/cli.py` | the one entry point, the `spiderpig` console script (`python -m spiderpig.cli` from a checkout): `build` (`spiderpig/build.py`: STEP/STL/DXF/BOM), `bake` (`spiderpig/bake.py`), `audit`, `tune`, `sim`, `export`, `report` (`spiderpig/tools/`), `explain`, `mcp`, `view` (each a module's `main(argv)`); the `mise` tasks run it. It imports nothing of the engine until a command runs |
| `spiderpig/view.py` | `spiderpig view <design> [--store] [--port] [--open]`, or `spiderpig view --linkage ... --pin bolt` (the build options: `resolve_args` turns them into a design in the store through `api.spec_of`): the store as the MCP picks it, `api.export(design, ["glb"])` (cached), the server below on a free port, the URL `/?design=<id>`; `start_background()` runs it as a child process (`--serve-only`) for the MCP `view` tool |
| `spiderpig/tools/` | `audit.py` (`mise run audit`: plan re-check, contract, OCCT clashes, DXF, BOM, snap strain, link tilt, and the joints' strength (`spiderpig/strength.py`: every pin's, pillar's and the crank's safety factor at the design's own MuJoCo loads, `spiderpig/sim/loads.py`, walking p99 and jammed at the servo's 45 % torque limit (a foot pinned stiffly, 24 angles, every case must stall: the loads note and audit warn otherwise), cached per design in the store's `pin_loads/`; `--pin-load WALK,JAM` overrides, `--no-sim` falls back to the family's; a two-link pin bends `F s / 2`, a pillar is a cantilever or a beam between the plates; jam SF < 1 fails the audit, < 2 jammed or < 3 walking warns, each finding with recomputed fixes; `verify` standard/full has `strength.joints` and the `strength` / `joint_overload` failure; `explain --strength`; the sweep of every walker x module is `docs/audit/STRENGTH.md`), `construction/wobble.py`; `construction.contract` has the checks), `tune.py` (crank phases and proportions for a smoother walk), `sim_walk.py` (the MuJoCo CLI: the build options, a stored design's id, or `--mjcf FILE` with its `.json`), `export.py` (`spiderpig export`: `api.export` on the command line, a stored design or the build options, every format), `report.py` (every linkage compared, mechanisms included, a log line per plan), `dev.py` / `kill_dev.py` (`mise run view` / `kill`: the dev servers, a checkout only) |
| `spiderpig/walk.py` | quasi-static walking model (support plane, no-slip velocity, per-revolution metrics); feeds `/api/walk`, the bake's drive data and `spiderpig/tools/tune.py`. The viewer's `viewer/src/drive/model.ts` implements the same model. |
| `spiderpig/sim/` | MuJoCo: `mjcf.py` builds the MJCF of the fabricated robot (exact masses plus a `payload_g` for the electronics, loop equalities, pin friction, velocity drives held to the servo's speed-torque line by `motor_line`; bounded per-design caches, `clear_caches`; its docstring states the assumptions: phase-locked sides, a rigid crank, unvalidated contact softness) and its viewer metadata; `run.py` steps it (`simulate` through a `PhaseLock`, the PI crank-phase controller the real servo bus needs, against a `servo_mismatch`; `walk_metrics` with the torque, support, side-phase, pin-load and acceleration numbers and a `walks` flag; `compare_with_walk` against the quasi-static model; `steering_check` with a straight run, the differentials and the bounded excursion `step_deg`; kinematic playback); `live.py` is the session the viewer's physics drive steps over `/ws/sim` (`LiveSim`: commands in through the lock, binary frames with the base pose, cranks, feet down, drive torques, body down, side phase, vertical acceleration and pin load out; a lock around stepping, resets requested from the reader thread; the hello carries `steering_check`'s verdict, computed once per model build). `spiderpig/tools/sim_walk.py` is the CLI. |
| `spiderpig/bake.py` | end-to-end `.glb` bake for the three.js viewer (`spiderpig bake`; cached in the store's `bakes/`) |
| `viewer/` | the Vite + TypeScript three.js client (`src/`), built by `mise run viewer-build` into `spiderpig/viewer/dist` (package data); its `node_modules` never ship |
| `spiderpig/server/app.py` | the viewer's FastAPI app (the dev server and `spiderpig view`, serving `spiderpig/viewer/dist`; `configure(store, prebake_default)`): `/api/glb/{mode}?linkage=&module=&phases=&p.NAME=` bakes on demand (cached per design; `mode` is `robot`, `side`, or one of the side-only ids old URLs use, `MODES`), `/api/walk` (same params) answers walk metrics for a design without building parts, `/api/linkages` lists the linkages (`kind`, `output`) and their params/modules for the viewer's tune panel and mechanism picker, `/api/modes` the dropdown's ids and labels; `?design=<id>` on `/api/glb` and `/api/walk` starts from a stored design's `BuildConfig` (`resolved.json`) and applies the other parameters on top (the glb from the store's export when it matches: `X-Spiderpig-Glb`), `/api/design/{id}` is its card for the page; `/ws/sim?linkage=&module=` drives the design live in MuJoCo (the model built in a worker process, one build per design, rebuilt when the sources change mid-build; validated commands, kept when sent during the build; `{"status": "stale"}` when the sources change; `SIM_MAX_SESSIONS` at once). Bakes plan through the store (`api.plan_config`); a planner budget that ran out (`api.PlanTimeout`) is not remembered as a failure |
| `spiderpig/` | the agent-facing surface (`docs/agentlib/API.md`): `spec.py` (Spec v1: a validated, JSON-schema'd document; every metric a `Target`, hard or soft per `TARGET_FIELDS`, unknown fields and wildcards rejected with `SpecError[]`), `api.py` (`resolve` -> `Design` with a content-addressed id; `check`, `plan`, `explain`, `recommend`, `walk`, `build`, `recheck`, `verify`, `export` as pure functions of the handle, each mapping onto one engine pass and returning a report), `failure.py` (every engine exception as a `Failure`: stage, code, culprits, numbers, blockers, recommendations with spec patches), `verify.py` (rows with a `proven` / `measured` / `estimated` tier at `quick` / `standard` / `full`), `design.py` (the handle, `Part` with the live build123d `solid`; an edited solid is outside the guarantee until `recheck` passes), `store.py` (the per-project store, `$SPIDERPIG_STORE` else `./.spiderpig`, git-ignored: `designs/<id>/` with the spec, the resolved record, one JSON per stage, the build's STEP parts, exports and a log; every op reads its stage when valid for the engine version, a plan is re-made and `verify_plan`ed on reload, `load` / `derive` / `compare` / `list_designs` / `gc`; `store=None` for memory). It only calls the engine; CLI and MCP come after it. |
| `spiderpig/mcp/` | the MCP server over that API (`spiderpig mcp --store PATH`, `mise run mcp`, `python -m spiderpig.mcp`; stdio; the official `mcp` SDK 2.x, `MCPServer`): one tool per operation, files and numbers only (a design is its id, a part the path of its STEP file in the store, a failure the `Failure` document under `failures` with `ok: false`; a misused tool sets `isError` with the same envelope). `outputs.py` holds the TypedDict output schemas, `jobs.py` the process pool per store (`build`, `verify` standard/full and `export` come back as jobs after `wait_seconds`; `get_job` / `wait_job`; `view(design)` starts or reuses a `spiderpig view --serve-only` child process and returns the URL), `guide.md` the `spiderpig://guide` resource (the vocabulary tables are generated from `TARGET_FIELDS` and the registries). Engine calls run in a worker thread one at a time; `tune` / `search` are not in v1. `tests/test_spiderpig_mcp.py` drives it through the SDK's in-memory client. |

### Pipeline contract

1. **Symbolic** — a `Linkage`'s steps (`spiderpig/linkages/*.py`): each point is a
   small sympy expression over earlier points' symbols, `t` and the params.
   `O` is the crank centre at the origin, y up, feet lowest.
2. **Compiled** — `Linkage.compiled` lambdifies it once;
   `LegSolution(orientation, phase).evaluate(ts)` runs it at `ts + phase`
   (a mirrored leg: reflected, at `π − (ts + phase)`).
3. **Template** — `MechanismTemplate`: topology, per-body `outline`, and
   per-joint `pose_at` closures over the compiled program. One template is
   one *side* of the robot (a leg module: single, double, decker, quad).
4. **Rationalized** — `fabricate.design_side(tmpl, config)`: groups in
   dependency order (drive, crank, axles, links, frame), each with the
   construction the config picks; their claims; the layer plan.
5. **Fabricated** — `fabricate(tmpl, config, t)`: every group realizes its
   parts at `t` inside its claims; plates are cut last with every hole the
   other groups asked for; the robot mirrors the side and adds the chassis.
6. **Serialized** — STEP/STL/DXF/BOM (`spiderpig/build.py`) or `.glb` (`spiderpig/bake.py`).

Every stage says what fails, so no follow-up digging is needed
(`spiderpig explain --linkage K --module M` prints all three):

- **template**: `Linkage.assert_assembles` raises `AssemblyError` naming the
  step whose bars can't meet, by how much and at which crank angles.
  `Linkage.check()` gives every loop's margin and transmission angle (over
  the torus of both inputs for a two-input mechanism). A mechanism's
  `output_check()` measures its output; `assert_output` raises `OutputError`
  when it breaks a promise (a platform that turns, a line not straight to
  its tolerance, a dwell too short).
- **drive**: one servo turns `t`; a second input stops at
  `ConstructionError` (`spiderpig/servos/mount.py`).
- **static facts**: each group declares `keepouts(ctx)` (an axle's neck over
  its span, a pillar's to a plate, the crank's journal at O).
  `side_clearances` lists every link that can never share their layers. The
  crank's router (`construction.route.crank_facts`) knows which links sweep O
  (their layer needs the crank off its axis) and which crank points each
  clears; `fabricate.static_stage` raises `ClearanceError` for a link no
  crankpin and no detour inside the body's underside clears, with the
  distances (`NoCrankPoint`).
- **plan**: the planner tallies what blocked it, and claims raise
  `Unbuildable(reason)` rather than returning `None`. `PlanError` lists the
  blockers with distances and the static clearances involved. A plan says
  whether it is proven the thinnest (`StackPlan.optimal`, `proof`: nodes per
  size ruled out, or which sizes a budget left open, and when the crank's
  joints forced a taller stack).
- Both `ClearanceError` and `PlanError` carry `recommendations`
  (`stack.Recommendation`: what to change, from, to, why, side effects, and
  what re-running showed), printed under "what would clear it:". A
  recommendation is only given once the stage passes with it
  (`recommend.py`); what can't help goes in the notes.

A new construction or claim must keep this up: raise with a reason, and
declare its keep-outs.

Correct by construction: the planner guarantees claims of different groups
never meet over the whole crank cycle, and `construction.contract` checks
that every part lies inside its own group's claims. A construction that
can't be built with the given parameters raises `ConstructionError` before
planning; a layout it can't be built in makes its claim return `None`.

To add a construction: implement `dims(ctx)` (validation, the radii its
claims use) and `realize(group, build)` (parts inside those claims), register
it in `spiderpig/construction/__init__.py`, run the contract tests. To add a kind of
group (a second drive, spacer rings): subclass `construction.base.Group`
(`claims`, `realize(build, done)`; `keepouts` / `interface` if it has any;
`cuts = True` if it cuts what the others asked for) and append its factory
to `construction.GROUP_FACTORIES`, in dependency order. To add a leg
module: a `linkage.Module` (its legs, and which of them share one crank
body) in `linkage.MODULES` or a linkage's own `modules`. To
add a linkage: a module in `spiderpig/linkages/` with its params, program, links
(`b<k>` -> joints, outline), frame, crank and feet (a mechanism: its
`output`); `tests/test_linkage.py` checks it assembles, stays rigid and
plans (`tests/test_mechanisms.py`: outputs against the research's numbers).

### The planner

`stack.StackProblem.solve()` finds the thinnest stack, and the cheapest
crank route in it:

1. **Static facts** (before any layer): the keep-outs above; each link's own
   shapes per layer; the crank's facts. A link with no crank point stops here.
2. **Search per stack size** (`_Search`), fewest layers first: link layers by
   fewest open layers, then the assembly tree from the crank out. Forward
   checking (every placed shape removes the layers it rules out; an axle's
   links bound where links that can't pass it may go; a pillar must reach a
   plate), the router as a sub-check at every node (`check`: bit-mask
   reachability over layers x crank states, and which states each layer still
   has on a route, which prunes more), conflicts as the links behind a
   failure, backjumping to the latest of them, learned nogoods (watched), and
   branch and bound on the route's cost.
3. **The route** for a complete layering (`CrankRouter.route`): exact, a
   shortest path over layers and **chains** (runs along one point whose webs
   meet: one screw). Buildable only: a stock screw per chain
   (`JointRules.spans`, from `PrintedCrank.post_joint`, end-play faces
   included), one chain per point, pockets of consecutive chains (and the
   last one and the horn screws) apart, a chain ending set back in the hub's
   lowest layer only if the horn screws still fit the shortened hub
   (`JointRules.hub_play`). Cost, in order: added features (run
   layers no rider needs, detour runs), detour sweep, a dropped bearing
   (`StackSpec.drop_bearing`, off by default), then fewer runs.
4. **Verification**: `problem.plan(layers, top, choices)` and `verify_plan`;
   a failure there is a bug (it raises).

Effort: a short search per size until one finds a plan (sizes in turn while
they are ruled out; once one exhausts its short budget, in doubling steps,
then back down the skipped sizes; for a multi-leg module, a second
strategy: a leg at a time at the single module's layers, `hint`; when no
size found one, the sizes left open get the full budget, thinnest first),
then the thinner sizes it didn't rule out with the full budget (the next
thinner first), then a cheaper route. A group may bound the sizes searched
(`Group.max_top(ctx)`: a bolt pillar's longest stock screw clamps at most
15 layers of 3 mm); `side_problem` caps `StackSpec.max_top` and the
`PlanError` names the bound (`StackProblem.notes`). Budgets (`StackSpec.quick_nodes`, `max_nodes`,
`max_total_nodes`) and the wall-clock deadline (`StackSpec.max_seconds`,
60 s: a node's cost grows with the stack size) bound all of it, never
validity: a plan found is returned with `optimal=False` and a `proof`
naming the sizes left open and what ran out; none found raises `PlanError`
with the tally (what blocked it, and per size: ruled out, left open at its
budget with nodes and seconds, or not tried). The checks `recommend.py`
re-runs share one more such deadline, so `design_side` returns within a few
minutes at worst. `tests/brute.py` is an independent brute force (every
layering, every route) the tests compare the planner's optimum with. Opt-in
`StackSpec` flags, all off by default and measured in
`docs/agentlib/PERF_PLANNER.md`: `workers` (stack sizes searched in forked
processes, `spiderpig/stack_pool.py`; the serial answer, proof included),
`symmetry` (one of each mirrored leg pair, `spiderpig/stack_symmetry.py`;
checked on the sampled layouts only), `quick_first` (a short search stops at its
first plan) and `prove=False` (the first plan, returned unproven).

**Envelope** (`spiderpig/construction/underside.py`): the body (frame plates, the
crank's own sweep, servo, centre plates) has an underside profile; what the
planner adds to the crank turns with it, so a detour sweeps a full circle
about O, which must stay `margin` above the profile and within the body's
x-extent (`Underside.allows`). `SideDesign.ground_clearance_mm`: the body's
lowest point above the lowest foot point.

### Stacking (future)

Not built. A stage would mount on its parent's output body (`Output.frame`:
origin joint, x-axis joint). The engine would need: the child's fixed pivots
placed as `offset(J1, J2, along, across)` on that body instead of `xy`; its
point names prefixed per stage so the programs concatenate into one; its
input added to `inputs` (its crank turns relative to the parent body, so its
angle is its input plus that body's rotation); one drive per input; and the
planner's clearances between bodies in relative motion across stages (the
child's frame is a moving body, not the frame plates).

Physical rules the claims encode:

- The links riding a crankpin (Klann's b1; Jansen's j and k; Strider's
  bars) sweep over the crank axis O, and the crank turns fully relative to
  them, so the crank crosses a rider's layer only along its crankpin: a
  built-up crankshaft with webs either side of each rider. A link pinned to
  a rider inside the crank circle (TrotBot's B8, the 6-bar's B6) sweeps O
  too: the crank runs along a post (the crankpin, or a detour point fixed to
  the crank) through its layer, a post it must clear.
- Layers 0 (outer frame plate) and `top` (inner frame plate) hold nothing
  but the plates and parts seated in their holes.
- Pillars (frame pivots) are anchored in both frame plates whenever the
  mechanism lets them reach both; every link on an axle is held in its
  layer by a shoulder, head, cap, plate or neighbouring link on each side.

## House rules

- Don't add `print` statements to the bake path — use the `bake_gltf` logger.
- Don't regress the profiler (keep the stage keys stable; downstream scripts
  may parse them).
- Don't put Z into joint poses. Z is the stack plan's job.
- A group builds only inside its own claims; keep `check_side` at `[]`.
- A change that alters parts should leave `mise run audit` green.
- Tests build each design, side and robot once per session (the `design` /
  `side` / `robot` factories in `tests/conftest.py`): read them, never
  mutate them. `tests/test_contract.py` is the contract and clash check for
  every module and servo; bakes, the CLI build, MuJoCo, the tuner and every
  test over ~5 s are marked `slow` (in the default run; `-m 'not slow'`, the quick
  tier, skips them). A heavy parametrized check keeps one cheap case in the quick
  tier through `tests/tiers.py` (`quick(values, keep)`; ids unchanged).
- Don't run the full suite locally (40-50 min): `mise run test-quick` while
  iterating, `mise run remote-test` before handing back (AGENTS.md, "Running tests").
  Audits (75-120 s each), sims and bakes go through `mise run remote -- ...`; its
  output, the junit XML and the remote `build/` land in `build/remote/<run>/`. Tests
  must stay xdist-safe: write under `tmp_path`, free ports only.
- One density table, one OCCT mass query: `spiderpig/hardware/mass.py`. One
  screw table: `spiderpig/hardware/fasteners.py` (`spiderpig/construction/crank.py` still carries its
  own until its rewrite lands).

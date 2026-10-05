# future_work.md

Open items, 2026-10-05. The code is under `spiderpig/`; `CLAUDE.md` says where things are
and `docs/ARCHITECTURE.md` how they work.

## Before the first build

- **Cut-rule warnings on the default robot.** SendCutSend's 2 x t rules
  (`spiderpig/manufacture.py`) still warn on the Strider double's aluminium: 8 parts
  break the web round a cut-out (worst a torso plate, 2.05 mm between a cut-out and a
  2.4 mm hole, under 4.06) and 2 the hole-to-edge distance (worst a centre plate,
  2.58 mm, under 4.57). Warnings, not errors (every web is over 1 x t), but each is a
  thin web to look at on the first cut (`mise run audit`, the "cut rules" table).
- **One rear screw per servo.** The bus plugs' slot through the centre plates takes each
  servo's far rear holes (`chassis._port_slots`), so each servo holds the centre plates
  by one rear screw; the frame ties carry the rest. The plug's direction is UNVERIFIED
  (`ServoSpec.bus_ports`): measure a servo and a plug before cutting.
- **Unpriced McMaster-Carr lines.** McMaster shows prices only behind a login, so its
  lines are unpriced in the BOM; `ORDER.md` estimates them from a priced alternative
  (`hardware/order.py`). Price them from an account, or add priced alternatives in
  `hardware/sources.py`.
- **UNVERIFIED numbers** the code flags (grep `UNVERIFIED`): the round crankpin's friction
  coefficients, the bench splice, the charger's and protection board's sizes, the 6 mm M3
  standoffs' alloy and length tolerance. Measure them on the first parts.

## Code

- **One shim-stacking helper.** Five functions stack shims thickest first, each with its
  own steps: `hardware.bom.shim_breakdown`, `ChicagoShaft.shim_count`
  (`construction/pivots/chicago.py`), `StandoffAxle.splice_shims`
  (`construction/pivots/standoff.py`), `construction.chassis._shims` and
  `construction.crank.shim_stack` (and `materials.washer_stack` for gap washers). One
  function over a family's steps (`bom.stack_steps`) would keep the BOM's split and the
  parts in step.
- **Register the NETRF6 lengths lazily.** `hardware/crank_catalog.py` registers every
  one-piece pillar length, 8-300 mm in 0.1 mm steps (2,921 catalog items), at import;
  registering a length when a construction first asks for it would shrink the catalog
  and its import time.
- **Slow tests without the mark.** The quick tier (`-m "not slow"`) relies on every test
  over ~5 s being marked `slow`, and some aren't: `pytest --durations=30` on the quick
  tier finds them (`tests/tiers.py` keeps one cheap case of a heavy parametrized check).

## Planner and model

- **Proofs on the quads.** Most sides are proven the thinnest, but quads whose legs must
  sit in separate blocks (Strider, 6-bar, TrotBot) can leave thinner sizes "not ruled out"
  within the budget (`StackPlan.proof`). Stronger lower bounds (legs that can't share
  layers, as a clique over blocks) or CP-SAT over the static tables would close them. A
  secondary objective (shortest axles, fewest shoulders) isn't modelled.
- **Rigid link frames.** Parts are modelled in world coordinates at the build angle, so
  the glTF bake recovers each body's motion by a planar rigid fit of its joints. A local
  frame per link and a closed-form pose `(x, y, θ)(t)` would drop the fit and make every
  leg's link literally the same part.
- **Firmware.** The servo driver's gait and steering code (the PI phase lock the sim
  models, `sim/run.py`; a design's torque limit, `config.LINKAGE_TORQUE_LIMITS`) is
  outside this repository.

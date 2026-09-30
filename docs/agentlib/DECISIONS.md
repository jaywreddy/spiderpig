# Agentic harness spec: decisions (SCOPE.md section 6)

1. Spec breadth: NARROW. v1 Spec has only fields the engine can verify today; unknown fields
   and wildcards are rejected with a message. compile(spec) is a compiler (seconds); search is
   a separate explicit call built on top later. (2026-09-30)

2. Hard vs soft: physical limits (size, budget, stack thickness, ground clearance) HARD by
   default (a miss fails verify); gait quality (bob, slip, speed, stride) SOFT by default
   (reported with a score); any field can flip with `hard: true/false`. (2026-09-30)

3. CAD exposure: EXPOSE build123d objects. The Python API hands back live solids
   (design.parts[name].solid) for agents to measure or modify; files + numbers stay the
   MCP-side representation (solids can't cross that boundary). Implication: modified parts
   bypass the claims contract unless re-checked; the API should offer a re-check
   (contract.clashes / bad_solids) on any edited part. (2026-09-30)

4. Store: PER PROJECT, ./.spiderpig (git-ignored): stable design ids across runs, plan cache,
   parts kept until an explicit gc; multi-user by sharing the folder. (2026-09-30)

5. Leg modules: NAMED PRESETS only in v1 (single | double | decker | quad, plus a linkage's
   own modules) with phases per leg; explicit leg lists later on Module(legs, cranks).
   (2026-09-30)

6. Viewer: SHIP viewer/dist AS PACKAGE DATA; `spiderpig view <design>` serves it without Node
   on the user's machine; the build runs at release time. (2026-09-30)

7. Sequencing: BOUND THE PLANNER FIRST (a compile must never hang: budgets bound the search,
   PlanError with a tally otherwise), then v1 in order, each in a Fable subagent on a
   worktree, merged and verified: 1) Spec + compile/verify Python API with solids exposed,
   2) per-project store, 3) MCP server over files and numbers, 4) viewer as package data and
   `spiderpig view`. Assumed without asking (SCOPE.md proposals): metric semantics pinned per
   field (section 3.1), speed flagged `estimated` until sim confirms it; sync Python API, MCP
   with Tasks and a process pool; conformance compares metrics not STEP bytes; "no plan" means
   "no plan within budget" with the proof always returned; mechanism specs verify OutputCheck
   and skip walking, compound machines unsupported and said so; offline servo CAD marks a
   design `estimated`. (2026-09-30)

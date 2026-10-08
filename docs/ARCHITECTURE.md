# How spiderpig works

spiderpig turns a walking linkage, such as Joe Klann's spider leg or Theo Jansen's
Strandbeest leg, into a robot you can build. Name the linkage and a few options, and it
writes the laser-cutting files, the 3D-printing files, the parts list and an animated
model. Its central idea is to share out space between the parts before any part is drawn,
so that no two of them can collide as the crank turns. This report explains how the code
does that, what it can and cannot do, and where it could mislead you.

It describes the code at commit `8bfc039` (2026-10-08, the merge of W5's package splits),
brought up to date by the documentation pass of 2026-10-08 (W7). It was first written
against commit `97bec2d` (2026-10-01), and the running example's measured numbers (stage
timings, part counts, sizes, the walk and the sim) are still that snapshot's, taken with
the constructions of the time (a printed crank, rod pins); they are marked as such where
they appear. The default designs' current layer counts, heights, parts and costs live in
one generated file, [docs/agentlib/DESIGNS.md](agentlib/DESIGNS.md). An illustrated version, with renders
from the viewer and drawings computed from the code, is
[docs/architecture/index.html](architecture/index.html), generated from this text as of 2026-10-05 and out of date since (its builder, `build_page.py`, needs its figure anchors updated to regenerate it).

**In short.**

- A design passes through six stages, from the linkage's geometry to files (section 3).
- The core is a layer planner that gives every link a layer and routes the crankshaft
  through the layers; most designs plan in seconds, proven as thin as possible
  (section 5).
- Parts are then built inside the space the planner gave them. Every failure comes back
  as data, often with a fix the program has already checked (sections 6 and 9).
- The largest risks: no built robot has been measured, and the three walking estimates
  disagree by a factor of up to three; large designs plan differently on slower machines;
  and stored results can go stale without warning (section 12).

## About this report

**Who it is for.** An engineer who reads Python, has never opened this repository and has
never designed a walking machine. After reading it you should be able to explain what the
system does from end to end, find the code for any part of it, say what it can and cannot
do, and know where its risks are.

**How to read it.** Sections 1 to 3 orient you: what the project is, the mechanical and
software background, and the pipeline at a glance. Sections 4 to 7 follow one design
through the pipeline, a stage at a time. Section 8 covers how a design is seen and judged,
and section 9 the interface built for AI agents and the command line. Sections 10 to 12
assess the project: how quality is kept, where the time goes, and what it can't do or
might get wrong. Section 13 says where to start. The appendices are for looking things
up.

**The running example.** One design runs through the whole report: a robot built on the
Klann linkage with four legs on each side (`spiderpig build --linkage klann`, the demo
Klann quad). It was the project's default when the report was first written; the default
is now the Strider `double` (`linkage.DEFAULT`), which is what `spiderpig build` makes with
no options. When the example can't show a point (it never fails to plan, for example),
another design stands in, and the text names it.

**The numbers** were measured at `97bec2d` (2026-10-01) unless marked "(project docs)",
meaning they come from the repository's own reports, or dated otherwise. Timings are
indicative; Appendix C says how they were taken. For today's figures of the default
designs see [DESIGNS.md](agentlib/DESIGNS.md).

**Contents.**
[1. What spiderpig is](#1-what-spiderpig-is) ·
[2. Background](#2-background-walking-machines-in-ten-minutes) ·
[3. The pipeline at a glance](#3-the-pipeline-at-a-glance) ·
[4. Motion](#4-motion-stages-1-to-3) ·
[5. Layer planning](#5-layer-planning-stage-4) ·
[6. Fabrication](#6-fabrication-stage-5) ·
[7. Outputs](#7-outputs-stage-6) ·
[8. Seeing and judging a design](#8-seeing-and-judging-a-design) ·
[9. The agent surface and the command line](#9-the-agent-surface-and-the-command-line) ·
[10. How quality is kept](#10-how-quality-is-kept) ·
[11. Performance](#11-performance) ·
[12. Limitations and risks](#12-limitations-and-risks) ·
[13. Where to start](#13-where-to-start) ·
[A. Glossary](#appendix-a-glossary) ·
[B. File index](#appendix-b-file-index) ·
[C. Documents, history and method](#appendix-c-documents-history-and-method) ·
[D. Where the documents and the code disagree](#appendix-d-where-the-documents-and-the-code-disagree)

## 1. What spiderpig is

You name a linkage (Klann, Jansen, Strider and others) and make a handful of choices: how
many legs a side, which servo motor, what sheet material, what kind of axle. spiderpig
answers with what a maker needs:

- laser-cutting files (DXF), packed onto sheets and corrected for the width of the laser's
  cut;
- 3D-printing files (STL): one per distinct printed part, with quantities;
- a bill of materials with vendors, pack sizes and prices;
- the whole assembly as STEP (for CAD programs) and STL;
- an animated 3D model for a browser viewer (a binary glTF file, `.glb`);
- a physics model for the MuJoCo simulator (MJCF);
- reports on every stage: did it pass, how thick is each side of the robot, how is it
  expected to walk, what does it weigh and cost.

Section 2.6 explains each of these formats and tools.

It behaves like a compiler. The input is a design. The output is either the files, or an
error that names the stage that failed and gives the numbers behind the failure. Where a
fix exists, the error also proposes it, and the fix has already been checked by running
the failed stage again with it.

The central promise is that designs are **correct by construction**: no two parts of a
side collide anywhere in the crank's turn. This promise has a history. On 2026-09-29 an
audit of the earlier code found every test passing while no design could physically be
built: two links shared a plane and collided for 13.6 % of every turn, and a foot link
silently dropped out of the cutting files. The answer was to share out space before any
part exists (section 5) and then check that each part stays inside its share (section 6).
The first half is a sound geometric bound over the whole turn; the second is checked at a
few crank angles, as is the chassis that joins the two sides, and section 6.3 says what
that leaves open.

There are three ways to drive it, all calling one engine, and a browser viewer for looking
at the result:

- **a command line**, `spiderpig <command>` (`build`, `bake`, `explain`, `audit` and six
  more);
- **a Python API**, `spiderpig.api`, designed for AI agents: a validated JSON *spec* goes
  in, typed reports come out, and an agent can reach the live CAD solids and edit them;
- **an MCP server** that offers the same operations to AI agents speaking the Model
  Context Protocol.

It does not write firmware or a gait controller for the servos. Its strength analysis is
of the joints and link plates only, at the design's own simulated loads (section 6.2); it
does no finite-element analysis of the parts. It does not invent mechanisms: every
linkage is a program in its catalog, and every part follows fixed construction rules.

In numbers (2026-10-08):

| measure | value |
|---|---|
| Python package `spiderpig/` | 40,462 lines in 123 Python files |
| tests | 27,009 lines; about 2,000 tests |
| browser viewer (TypeScript) | 3,018 lines |
| linkages in the catalog | 28: 17 walkers and 11 mechanisms |
| history | 483 commits: 2015 coursework, rebuilt almost entirely from September 2026 (Appendix C) |

Following a design through the code needs a few mechanical ideas and a few tools. Section
2 gives them.

## 2. Background: walking machines in ten minutes

This section introduces the ideas the rest of the report depends on: linkages, legs and
robots, how a flat-built robot is layered, and the software underneath. Where the code has
its own name for something, the name follows in `monospace`.

### 2.1 Linkages

A **linkage** is a set of rigid bars, the **links**, joined by pins so that two joined
links can only rotate about their pin. In a **planar** linkage every link moves in
parallel planes, so the whole motion can be drawn in two dimensions. Every machine
spiderpig builds is planar. The code draws linkages with x along the walking direction and
y up; z, across the robot, is left for the layers of section 2.4.

One body doesn't move: the **frame** (`torso`). It carries the **fixed pivots**. A motor
turns the **crank**, a short bar that rotates fully about one fixed pivot, the **crank
centre** (always `O`, at the origin). The moving end of the crank is the **crankpin**, and
a link pivoted on it, dragged round by it, is a **crank rider**. The crank angle is the
linkage's input; every other joint moves as a function of it. In the code the crank angle
is `t`, in radians.

The other links come in a few kinds. A **rocker** is pivoted on the frame and swings back
and forth without turning fully. A **coupler** hangs between moving joints, attached to
neither the frame nor the motor, and points on it trace closed curves as the crank turns.
A walking leg is a linkage designed so that one point, the **foot**, traces a loop whose
bottom stays low and nearly straight. Along the bottom the foot is on the ground and
moves backwards, pushing the robot forwards (the **stance**); then it lifts and swings
forward over the top of the loop (the **swing**). The loop is the **foot path**.

Each joint of a linkage is found the way you would draw it with a compass: it lies at
given distances from two joints already placed, so it sits where two circles cross. Two
circles cross at two points, and the design says which one to take (the **branch**). Each
such joint closes a **loop**, a closed chain of bars such as O–M–C–A in the figure below.
If, at some crank angle, the two known joints move farther apart than the two bars can
reach, the circles stop crossing and the linkage **can't assemble** there. The
**transmission angle** at a joint is the angle between the two bars that meet there. Near
0° or 180° the joint is close to locking (**toggle**) and the motor can barely drive it. A
good design keeps every loop well clear of both failures over the whole turn.

**Figure 1.** The Klann leg, the running example's linkage, drawn to scale at crank angle
t = 1 rad (one character is 4 mm across, one line 8 mm down). The dots trace the crank
circle around O and the loop the foot F follows. O, A and B are fixed; everything else
moves.

```
                             --E
                 b2    ------   \\
                    B--           \\
                                    \
                                     \\          b4
                                       \\
    ........M----------------C----------D\\
   ..      / ..    b1       /              \
   .      /    .           /   b3           \\
   .     O     .         //                   \\
   ..         ..        /                       \
    ...    ...         /                         \\       ...
       .....          A                            \\    .. .
                                                     \  ..  ..
                                                      \\     ..
                                                     .. \\     ..
                                                   ..     \\    ....
                                                  ..        \      .....
                                               ...           \\        .
                                              ..               \\    ...
                                            ..                  .F....
                                           .          ...........
                                            ..........    foot path

O        crank centre, on the servo's axis        A, B   fixed pivots on the frame
O–M      the crank; M is the crankpin             C, D, E   pins joining links
b1       crank rider M–C–D, riding the crankpin   b2     upper rocker B–E
b3       lower rocker A–C                         b4     the leg E–D–F, foot F
```

As the crank turns, the foot runs clockwise round its path in this view: right to left
along the bottom on the ground, then up and forward, left to right, over the top. At about
t = 217° the crank rider b1 lies right across O, and for about a quarter of the turn it is
within 13 mm of it; section 2.4 explains why that matters.

### 2.2 The linkage catalog

The linkage catalog holds 17 walkers in six families:

- **Klann** (Joe Klann's US patent 6,260,862): six bars and two fixed pivots, as in Figure
  1. It lifts its foot high. It stays registered as the wobbly demo (`--linkage klann`);
  the catalog also has four variants transcribed from published drawings.
- **Jansen** (Theo Jansen's Strandbeest): eight bars and one fixed pivot. Its stance is
  long, flat and smooth, and it lifts its foot little.
- **Strider** (Wade and Ben Vagle, diywalkers.com): one "leg" is a coupled,
  mirror-symmetric pair with two feet. It is the project's default (`linkage.DEFAULT`),
  built as its `double` module.
- **TrotBot** (also from diywalkers.com): eight bars, plus versions with a heel (ten
  bars, two feet) and a retractable toe (twelve bars, three feet).
- **Four-bar**: the simplest walker, from diywalkers.com, including the legs of a LEGO
  walker called Spot Micro.
- **Six-bar**: the four-bar with its rocker extended, in four versions.

The catalog also holds 11 **mechanisms**. They are not walkers but building blocks with a
promised output: a point that moves in a straight line (Hoekens, Watt, Peaucellier), a
platform that lifts without tilting, a rocker that swings a fixed angle or pauses. A
mechanism declares its output and its promises ("straight to within 0.05 mm over this part
of the turn", "never rotates", "pauses for at least 120°"), and the pipeline holds it to
them. One of them, the five-bar, has two inputs and so needs two motors; spiderpig accepts
it as a design but can't build it (section 6.2).

### 2.3 From one leg to a robot

A **leg** is one copy of a linkage with two settings. Its **phase** is an offset added to
the crank angle: a leg at phase φ is where the basic leg would be at angle t + φ. Legs are
spread across the turn so that, at every moment, some feet are on the ground while others
swing forward. A robot walks only if its feet take turns like this.

A leg's **orientation** is +1 (as drawn) or −1 (**mirrored**). A mirrored leg is the
drawing flipped about the vertical axis (x → −x), evaluated at crank angle π − (t + φ).
Flipping x and replacing the angle t by π − t sends the crankpin (r cos t, r sin t) to (−r
cos(π − t), r sin(π − t)) = (r cos t, r sin t): the same place. So a mirrored leg rides
the same crank, turning the same way, while facing backwards.

A **module** is a named set of legs driven by one crank. There are four:

| module | legs (orientation, phase) | what it is |
|---|---|---|
| `single` | (+1, 0°) | one leg |
| `double` | (+1, 0°), (−1, 0°) | a mirrored pair on one crankpin |
| `decker` | (+1, 0°), (+1, 90°) | two legs a quarter turn apart (as in double-decker) |
| `quad` | (+1, 0°), (−1, 180°), (+1, 90°), (−1, 270°) | a decker plus a mirrored decker half a turn later |

A **side** is one module on one crank, driven by one servo, between two frame plates.
A walking **robot** has two sides, the right side a mirror image of the left across the
robot's mid-plane (z → −z), with their servos back to back on a chassis. The running
example, a Klann `quad`, therefore has eight legs and two servos. It steers by turning the
two servos at different speeds.

Two different mirrors appear here and are easy to confuse. A mirrored *leg* is flipped
front to back and stays on its own side. The right *side* is the whole left side reflected
across the mid-plane. And module names count legs per side, so `quad` is an eight-legged
robot.

### 2.4 Building it flat

The robot is built from flat parts. Each link is a plate cut from 3 mm sheet (acrylic by
default): a bar 12 mm wide with rounded ends, following the line between its joints.
Because the mechanism is planar, two links can only pass each other if they lie in
different planes. So each side is a **stack of layers**, each one sheet thick. Every link
lives in one layer, and two links whose outlines come close at any point of the turn must
be in different layers. The bottom layer (index 0) is the **outer frame plate** and the
top layer is the **inner frame plate**, which carries the servo. Nothing else may sit in
either, except parts seated in their holes: the ends of axles, the crank's stub, the
servo's horn. (That is the planner's model. In the built stack a layer is as thick as its
thickest plate, and thin clearance gaps between layers hold screw heads; section 5.7.)

Joints become axles that run across layers. An axle that joins links to the frame is a
**pillar**, anchored in the frame plates; one that joins links only to links is a **pin**.
Each axle has a **head** at its lower end and, unless the inner plate holds it, a **cap**
at its upper end. A **shoulder**, a collar above and below each of its links, holds the
link in its layer. Between shoulders the axle **necks** down to a thinner shaft, so that
other links can pass close by. A link that comes nearer to an axle than even the neck
allows can never share a layer the axle runs through.

The crank is the hard part. The crank rider (`b1`) is dragged round a full circle by the
crankpin, so it sweeps right across the crank centre O. In its layer nothing can sit on O:
the crank has to leave its axis, cross the layer along the crankpin, and come back. The
crank is therefore a **built-up crankshaft**, like an engine's, made of three kinds of
piece:

- a **journal**: the shaft on O, in layers where nothing crosses O;
- a **web**: an arm from O out to the crankpin, in the layer just below and the layer just
  above each rider;
- a **post**: the crankpin itself, through the rider's layer.

At the top, a **hub** joins the crank to the servo's **horn**, the disc that fits on the
servo's output shaft. At the bottom, a stub of the journal turns in a hole in the outer
frame plate: the crank's **bottom bearing**.

Other links that sweep across O need the crank off its axis in their layers too. The crank
can leave its axis along any crankpin that such a link never comes near. When every
crankpin comes too close, the crank can take a **detour**: a post at an extra point fixed
to the crank, provided the circle that point sweeps stays above the bottom of the robot's
body.

Choosing a layer for every link and a route for the crankshaft, so that nothing ever
collides and the stack is as thin as possible, is the **layer planning** problem. It is
the heart of this codebase (section 5). Here is the plan spiderpig found for a single
Klann leg at the 2026-10-01 snapshot, with the printed crank of the time: 7 layers and
21 mm. (The running example's 12 layers are too many to read here.)

**Figure 2.** One Klann leg's layers, from 6 (top) down to 0, and the crank's route
through them (the 2026-10-01 snapshot's printed crank).

```
          what each layer holds
layer 6   inner frame plate (the servo sits on top of it)
layer 5   crank hub, pillar A neck, pillar B shoulder, pin E cap, servo horn, servo screw head
layer 4   b2, crank hub, pillar A shoulder, pin C cap, pin D cap
layer 3   b3, b4, crank journal and web, pillar B shoulder
layer 2   b1, pillar A shoulder, pillar B neck, pin E head
layer 1   crank journal and web, pillar A neck, pillar B neck, pin C head, pin D head
layer 0   outer frame plate
layer -1  (below the outer plate) the heads of pillars A and B

          the crank alone, seen from the side
                 O                          M
layer 6   ======[horn]=====================================   servo above; its horn sits in
                                                              the plate's hole, reaching
                                                              just into layer 5
layer 5         [####]                                        hub, screwed to the horn
layer 4         [####]
layer 3         [###########################]                 journal + upper web (nut)
layer 2   <============ b1 ==================|#|=======>      b1 turns on the post at M,
                                                              sweeping across O
layer 1         [###########################]                 journal + lower web (screw head)
layer 0   ======[stub]=====================================   the stub turns in the plate
```

The crank rider `b1` sits alone in layer 2, threaded on the post at M. The crank leaves O
with a web in layer 1, crosses layer 2 along the post, and returns to O with a web in
layer
3. Links `b3` and `b4` share layer 3 with that web, and `b2` shares layer 4 with the hub,
because none of them ever comes near O. In this printed crank one screw ran up through the
post and held the two halves together. The route is the same with today's crank, the
laser-cut `bolt` crank (section 6.2): each web is one aluminium plate, the post a stock
steel hex standoff whose ends sit in hex pockets of the two webs, retained by a screw into
each end, and the rider turns on a printed sleeve over the hex. Its screw heads stand in
thin clearance gaps beside the webs (section 5.7). The printed crank, and the keyed one
that followed it, were removed on 2026-10-07.

### 2.5 How parts are made

Every part is made in one of three ways, and the code tags each part with it (`fab`):

- **laser**: links (acrylic, a few in aluminium), the aluminium frame plates, crank
  plates and the chassis's centre plates, the electronics deck, cut from sheet;
- **printed**: every pivot's spacer rings and head spacers, the crank's sleeves, collars
  and rings, the horn spacer, the deck's rails and battery cradle, the feet's TPU socks,
  on a 3D printer;
- **purchased**: servos, electronics, screws, nuts, washers and shims, Chicago screws,
  round and hex standoffs, heat-set inserts, epoxy.

The pins are M3 Chicago screws, the pillars 6 mm round standoff columns and the crank
laser-cut on steel hex standoffs (section 6.2). These are the only constructions since
2026-10-07: the printed axles and cranks, and the other metal pivots (steel rod, M3 bolt,
ball bearing, plastic bushing, PTFE liner), were removed (`config.REMOVED_CONSTRUCTIONS`
names each one's replacement).

The motors are **continuous-rotation servos**: geared motors with built-in speed control,
which turn fully at a commanded speed rather than holding an angle as a hobby servo does.
The screws are metric: **M3** means 3 mm in diameter, and screws come in a fixed set of
**stock lengths**. A **heat-set insert** is a brass thread pressed into plastic with a hot
iron.

### 2.6 The tools and formats underneath

- **build123d** is the Python CAD library every part is modelled in. It sits on **OCCT**
  (Open CASCADE Technology), the open-source C++ geometry kernel, which represents solids
  exactly as surfaces and edges (a **BRep**, boundary representation) and does the
  **booleans** (union, cut, intersection), the **meshing** into triangles, mass properties
  and STEP files. OCP is its Python binding.
- **sympy** does symbolic mathematics; its `lambdify` turns an expression into a fast
  **numpy** function.
- **STEP** files hold exact solids for CAD programs. **STL** files hold triangle meshes,
  which 3D-printing software reads. **DXF** files hold 2D outlines for the laser cutter.
  **glTF** is a 3D scene format for the web, with animation; `.glb` is its one-file binary
  form. **MJCF** is the XML model format of **MuJoCo**, a rigid-body physics simulator.
- A laser burns away a thin strip as it cuts, the **kerf** (the sheet's service's: 0.2 mm
  at Ponoko; SendCutSend compensates for its own), so where the service doesn't, outlines
  are moved out by half of it and holes in by half of it. A 3D printer of the
  kind assumed here (FDM) lays down plastic layer by layer; an overhang needs printed
  support under it, and **infill** is how solid the inside of a part is.
- The viewer is a **three.js** page built with **Vite** and served by a **FastAPI** web
  server. **uv** manages the Python environment and **mise** runs the project's tasks.
- **MCP**, the Model Context Protocol, is a standard way for AI assistants to call a
  program: a server offers **tools** (functions), **resources** (documents) and
  **prompts**.

A few words have two meanings in this codebase, and the report keeps them apart:

- **module**: a leg module (section 2.3), never a Python module; the report calls Python
  modules files.
- **frame**: the fixed body; frames of the animation are "animation frames".
- **straight-line program** (section 4.1): a compiler term for a program with no loops or
  branches. It has nothing to do with straight-line mechanisms.
- **envelope**: the underside of the robot's body, which bounds a detour (section 5.2),
  not the robot's overall size, which the spec calls `envelope_x/y/z_mm`.
- **budget**: a cap on the planner's search effort (section 5.4), or money (section 9).
- **body**: a template body (20 a side in the running example, section 4.4), a fabricated
  body (179 in the running example's robot, 173 of which carry a part), or a MuJoCo body
  (35: the base, and each side's crank and links).
- **`Params` and `params`**: `Params` holds shared part dimensions such as the link
  radius; the spec's `params` are a linkage's parameters, which the command line calls
  *proportions*.
- **top and bottom**: of the layer stack, along z (layer 0 is the bottom); of the robot,
  along y (the feet are lowest).

The next section shows the six stages the code goes through to turn a linkage into these
parts.

## 3. The pipeline at a glance

A design passes through six stages. Each consumes the one before and adds one kind of
knowledge. The six stages are the **pipeline**, and the code that runs them is the
**engine**.

| # | stage | adds | code |
|---|---|---|---|
| 1 | symbolic | a compass-and-ruler program over exact parameters | `linkage/`, `linkages/` |
| 2 | compiled | numpy functions of the crank angle, built once | `linkage/engine.py` |
| 3 | template | one side's bodies, joints and connections, every joint a function of `t` | `linkage/assembly.py`, `mechanism.py` |
| 4 | rationalized | groups (one per functional part: the drive, the crank, each axle, the links, the frame), the space each claims in every layer, the layer plan, the crank route | `construction/`, `stack/`, `fabricate.py` |
| 5 | fabricated | every part as a solid at one crank angle, tagged laser, printed or purchased | `construction/`, `fabricate.py` |
| 6 | serialized | STEP, STL, DXF, BOM, glb, MJCF | `build.py`, `layout.py`, `hardware/bom.py`, `bake.py`, `sim/` |

All paths are under `spiderpig/`. *Rationalized* is the code's word for turning ideal
moving lines into parts with thickness; this report calls that stage **layer planning**.

Here is the running example at each stage:

| stage | the running example (a Klann `quad` robot, two sides) |
|---|---|
| 1 symbolic | 8 steps placing joints O, A, B, M, C, D, E, F; 11 parameters |
| 2 compiled | about 1.8 s once per process, mostly first-use imports; the compile itself about 0.1 s |
| 3 template | each side: 4 legs, 16 links, 20 bodies |
| 4 rationalized | at the snapshot, 12 layers (36 mm) per side with the printed crank; today's figure is in [DESIGNS.md](agentlib/DESIGNS.md) (`klann_quad`) |
| 5 fabricated | at the snapshot, 197 parts: 39 laser-cut, 42 printed, 116 purchased (the pins rods, rings and clips then) |
| 6 serialized | at the snapshot, STEP 11.4 MB, 17 print STLs, 2 DXF sheets, a BOM of at least $124.87 (a lower bound, section 7.5), glb 9.5 MB |

**One record says what to build.** Every stage takes a `BuildConfig`
(`spiderpig/config.py`), which holds:

- the linkage and the module, and whether to build the robot or one side;
- the legs' phases, and any parameter overrides;
- the sheet material and its measured thickness;
- the servo;
- the pillar, pin and crank constructions: the pins M3 Chicago screws, `chicago`, the
  pillars round standoff columns, `standoff`, and the crank the laser-cut `bolt` crank
  (`bolt_round` on TrotBot's heel and toe), the only ones since 2026-10-07;
- the sheets: the links' (`sheet`), the frame plates' (`frame_sheet`), the crank's
  (`crank_sheet`) and any per-link sheet (section 5.7);
- shared dimensions (`Params`: the 6 mm link radius, axle diameters, the 1 mm clearance
  margin and so on).

It checks the linkage, module, servo, phases, parameters, sheets (the crank's must be
aluminium) and constructions when made (a removed construction or `Params` field fails with its replacement:
`config.REMOVED_CONSTRUCTIONS`, `config.REMOVED_PARAMS`), and drops values equal to
their defaults, so one design has one config however it was asked for. Its `key`
(`strider_double_robot`, plus a hash for anything non-default) names every cache.

**Bodies are named by convention**, and the names appear throughout: `b1` to `b<n>` are
links, `conn` is a crank, `torso` the frame, and `coupler` a shaft coupler at O (not the
kinematic coupler of section 2.1). In a module with several legs each name gets a leg
suffix (`b1_leg0`), and in a robot a side prefix (`L.b1_leg0`, `R.b1_leg0`).

**What sits above the engine.** Section 9 explains the spec, the design id and the store;
the outline is:

```
   Spec (JSON) --resolve--> Design (content-addressed id)
                                 |
        check · plan · walk · build · recheck · verify · export     spiderpig/api/
                                 |
        Store: ./.spiderpig/designs/<id>/   (every stage's report, the parts)   store.py

   Ways in:  Python API (live solids) · MCP server (files, jobs) · CLI `spiderpig`
   Viewer:   FastAPI server + three.js page (play, drive, tune)
   Judges:   walk.py (quasi-static walking model) · sim/ (MuJoCo) · tools/audit.py
```

**Where things live.** Appendix B lists every file with its size and tests.

| directory or file | what it holds | section |
|---|---|---|
| `spiderpig/linkage/` | the symbolic engine: program helpers, compilation, checks, templates | 4 |
| `spiderpig/linkages/` | the linkage catalog: one file per family | 4 |
| `spiderpig/mechanism.py` | bodies, joints, poses; the template and the concrete mechanism | 4 |
| `spiderpig/config.py` | `BuildConfig` and the shared command-line options | 3 |
| `spiderpig/stack/` | the layer planner | 5 |
| `spiderpig/construction/` | the groups and their constructions: claims, then parts | 5, 6 |
| `spiderpig/fabricate.py` | orchestration: plan a side, fabricate a side or a robot | 5, 6 |
| `spiderpig/recommend.py`, `explain.py` | checked fixes for failures; each stage's verdict in prose | 5 |
| `spiderpig/servos/`, `hardware/`, `materials.py`, `manufacture.py`, `strength.py` | servo data and models; the parts catalog, fasteners, mass, BOM, ordering; sheets, cut rules, joint strength | 6, 7 |
| `spiderpig/build.py`, `layout.py`, `bake.py`, `sim/`, `mesh.py`, `workers.py`, `fabcache.py`, `keys.py`, `uptodate.py` | outputs, and the caches that keep them from being recomputed | 7 |
| `spiderpig/walk.py`, `server/`, `viewer/` | the walking model, the web server, the browser viewer | 8 |
| `spiderpig/spec.py`, `api/`, `design.py`, `failure.py`, `verify.py`, `store.py` | the agent surface | 9 |
| `spiderpig/mcp/`, `cli.py`, `tools/`, `view.py` | the MCP server and the command line | 9 |

Sections 4 to 7 follow the running example through these stages, starting with the
motion.

## 4. Motion: stages 1 to 3

The first three stages (`spiderpig/linkage/`, `spiderpig/linkages/`,
`spiderpig/mechanism.py`) answer one question: where is every joint at every crank angle?
They know nothing about layers, parts or materials.

### 4.1 A linkage is a short program

A linkage is defined as a **straight-line program**: an ordered list of steps, each
placing one joint as a small symbolic expression of the crank angle `t`, the parameters,
and the joints placed before it. Here is the Klann linkage's whole program
(`spiderpig/linkages/klann.py`):

```python
def program(p):
    oa = p["OA"]
    return [
        ("O", xy(0, 0)),                                    # crank centre
        ("A", rotate(xy(0, -oa), p["angA"])),               # lower fixed pivot
        ("B", rotate(xy(0, p["OB"] * oa), p["angB"])),      # upper fixed pivot
        ("M", crank(p["OM"] * oa)),                         # crankpin: (r cos t, r sin t)
        ("C", circle_x_circle(P("M"), p["MC"] * oa, P("A"), p["AC"] * oa, 1)),
        ("D", extend(P("M"), P("C"), p["CD"] * oa)),        # on the ray M -> C, beyond C
        ("E", circle_x_circle(P("B"), p["BE"] * oa, P("D"), p["DE"] * oa, 1)),
        ("F", extend(P("E"), P("D"), p["DF"] * oa)),        # the foot
    ]
```

The helpers (`spiderpig/linkage/engine.py:22-80`) are the compass and ruler: `xy`,
`rotate`, `crank`, `P` (a joint placed earlier, such as `P("M")`), `circle_x_circle` (two
circles crossing, with its branch), `extend` (a point along a ray), `offset` (a point
rigid with a bar), and `crank_at` for a second input. Parameters are exact numbers (sympy
rationals). Klann's lengths are multiples of `OA`, the distance from O to pivot A, 60 mm
by default; most other linkages give lengths in drawing units times a `unit` in mm.
Scaling a linkage means changing that one parameter. (The command line and the viewer call
parameter overrides *proportions*; the spec calls them `params`.)

Around the program, a `Linkage` declares its **links** (each `b<k>` with its joints and
the segments its plate is cut along: `b1` joins M, C and D and is cut along M–D), its
**frame** joints, its **crank**, and either its **feet** (a walker) or its **output** (a
mechanism). Each file in `spiderpig/linkages/` builds its linkages and calls `register()`
when imported; the registry imports these files on first use, Strider first so that it is
the default (`linkage.DEFAULT`; Klann stays registered as the wobbly demo). A linkage may
name its `default_module`, the module a design gets when none is asked for (Strider's
`double`; a walker's `quad` otherwise).

### 4.2 Compiled once, evaluated on arrays

`Linkage.compiled` turns each step into a numpy function with sympy's `lambdify`, once per
process, and runs the steps in order, feeding each step's values into the next. No step's
expression is ever substituted into another, so the cost of compiling doesn't grow with
the program's depth. The parameters stay symbols, so overriding one needs no
recompilation.

This design came out of two performance failures. Earlier versions solved the linkage
symbolically for every leg at every animation frame, and their substituted expressions
grew to about 34,000 operations. A later version compiled one fully substituted expression
per joint, which took 140–227 s for TrotBot; compiling step by step takes about 0.1 s
(commit `33c5bee`). A test keeps every step under 200 operations.

One leg is a `LegSolution(orientation, phase, values)`, where `values` are any parameter
overrides. Its `evaluate(ts)` runs the compiled program on a whole array of crank angles:
a phase is a shift in crank angle, a mirror the reflection at π − t from section 2.3. One
compiled program therefore serves every leg of every module.

### 4.3 The template stage checks the motion

Before building anything, the template stage checks the motion
(`spiderpig/linkage/checks.py`):

- `check_steps` samples 720 crank angles (every half degree). For each joint placed by
  two crossing circles it measures the **margin** (how far the circles are from parting
  at the worst angle) and the range of the transmission angle, and flags a joint within
  15° of toggle. For Klann: "C: bars A-C 54.5 mm and M-C 68.6 mm close with 21.24 mm to
  spare (worst at 336°), transmission angle 31°..86°". A loop that fails raises
  (`Linkage.assert_assembles`) `AssemblyError` naming the joint, by how much it fails and over which crank angles. A
  fixed pivot whose coordinates are undefined (a negative number under a square root) is
  refused with that root named.
- For a mechanism, `check_output` measures the output against its promises: it fits a line
  over the promised part of the turn and reports stroke and straightness, a platform's
  rotation, a rocker's swing and dwell. A broken promise raises `OutputError`. For a
  two-input mechanism the checks cover a 180 × 180 grid of both inputs.

These checks sample; they are not proofs. A failure narrower than half a degree of crank
could slip between samples.

### 4.4 The template: one side as bodies and joints

`build_module_template(module, phases, proportions, linkage)`
(`spiderpig/linkage/assembly.py:222`) runs those checks, then builds one side as a
`MechanismTemplate` (`spiderpig/mechanism.py`):

- each leg contributes its links `b<k>`, its crank `conn` and its frame `torso`, every
  name suffixed `_leg<k>`;
- bodies that share a joint *name* are connected at it, so the topology comes free from
  the definition;
- the cranks of legs that share one are fused into a single rigid crank, and the frames
  into one frame (a step inherited from the 2016 code).

The running example's side therefore has 20 bodies: 16 links, two crank bodies (`conn` and
`conn_upper`, each shared by a mirrored pair of legs), one frame and the shaft coupler.
Once built, the crank construction joins the two crank bodies and the coupler into one
crankshaft (Figure 3 in section 5.3).

Every joint's position is a function of `t` at z = 0. Height is deliberately absent: the
project's house rules (in [CLAUDE.md](../CLAUDE.md), summarised in section 13) forbid
putting z into joint poses, because z is the layer planner's decision.
`MechanismTemplate.sample(ts)` evaluates every joint over a batch of angles in one pass
(a `SampledPoses`; a leg's phase runs it at `ts + phase`, a mirrored leg reflected);
`freeze_at(t)` produces the concrete `Mechanism` at one angle that fabrication works on,
so sampling and fabrication can't disagree about where a joint is.

One oddity trips newcomers: every body's pose in the template is the identity, and only
joint positions move. Code that needs a link's rotation (the viewer's animation, the
walking model) recovers it by fitting a planar rigid motion to the link's joints.

### 4.5 The linkage catalog in detail

| family | keys | scale parameter and default |
|---|---|---|
| Klann | `klann` | `OA` = 60 mm (crank radius 24.7 mm; the foot lifts 86.5 mm) |
| Klann variants | `klann_patent`, `klann_lego`, `klann_long_legs`, `klann_high_step` | `unit` = 8 mm |
| Jansen | `jansen` | `unit` = 1.6 mm, not the drawing's 1.5: at 1.5 a link passes 8.7 mm from a pillar, under the 9 mm its neck needs |
| Strider | `strider` | `unit` = 6.5 mm, so the foot links clear the crank; its own modules |
| TrotBot | `trotbot`, `trotbot_heel`, `trotbot_toe` | `unit` = 10.5 mm, the drawing's 7 mm scaled 1.5×, so the heel's `b7` clears a 6 mm crankpin; the heel and toe take the `bolt_round` crank (`config.LINKAGE_CRANKS`), whose round 6 mm standoff it clears, where the default hex crankpin's 8.5 mm sleeve doesn't |
| four-bar | `fourbar`, `fourbar_spot_micro`, `fourbar_spot_micro_v2` | `unit` = 0.6–0.8 mm |
| six-bar | `sixbar`, `sixbar_v1`, `sixbar_v2`, `sixbar_v3` | `unit` = 0.6 or 1.0 mm |

The mechanisms (`spiderpig/linkages/mechanisms.py`) are:

- three straight-line blocks: `hoecken`, `watt_crank`, and `peaucellier_crank`, which is
  straight to 3 × 10⁻¹³ mm;
- two lifts that never tilt: `parallelogram_lift`, `watt_table_lift`;
- a pantograph that doubles a straight stroke: `hoecken_pantograph`;
- three rockers: `crank_rocker` swings 60°, `rocker_amplifier` 153°, and `dwell_rocker`
  pauses for 125°;
- a lift-and-carry table: `hoecken_table`;
- the two-input `five_bar`.

The catalog is checked against its sources. Klann and the diywalkers linkages are compared
with the joint positions in the published plans, and TrotBot, its heel version, Strider
and Jansen with the diywalkers site's own simulators over the whole turn, to within 10⁻⁹
mm.

### 4.6 Limits of the motion model

- **Revolute joints only, in one plane.** There are no sliding joints; straight lines come
  from linkages that make them (Watt, Peaucellier).
- **Fixed branches.** A linkage that would need to switch branch mid-turn simply can't
  assemble there; nothing passes through a toggle.
- **One buildable input.** A two-input mechanism resolves and checks but can't be built.
- **Sampled checks.** Margins and output promises are measured at 720 angles.
- **A guess at which parameters may be negative.** Angles may be, and so may any parameter
  whose default is zero or below; every other parameter is treated as a length that must
  be positive. A coordinate with a positive default therefore can't be made negative
  (`watt_crank` with `cy=-1` is refused as "a length").

Everything so far is points and lines at z = 0. The next stage gives them thickness.

## 5. Layer planning: stage 4

Section 4 ended with points that move in a plane. This stage decides, for one side, which
layer each link sits in, how many layers the side needs, and how the crankshaft runs
through them. Fewer layers make a lighter, stiffer side with shorter axles, so the planner
looks for the thinnest stack. It does all this without building a single part. It is the
largest and most original part of the code.

### 5.1 Groups and claims

The planner (`spiderpig/stack/`) doesn't know how anything is built. The side is divided
into **groups**, one per functional part: the drive (the servo), the crank, one per
pillar, one per pin, the links, the frame plates. For one Klann leg the groups are
`drive`, `crank`, `pillar:A`, `pillar:B`, `pin:C`, `pin:D`, `pin:E`, `links` and `frame`,
in the order they depend on each other. In Figure 1's terms, A and B are fixed pivots, so
their axles are pillars; C, D and E join links to links, so theirs are pins; O and M
belong to the crank.

Each group states its **claims**: the space it will occupy in each layer, as a function of
the layers given to the links it depends on. Claims are built from two kinds of shape: a
**disc** around a named point, and a **pill**, a capsule around the segment between two
named points. The points are the linkage's joints, so the shapes move as the crank turns.
A pin's claim, for example, is a disc of the pin's own radius in each of its links'
layers, a shoulder disc beside each link, a smaller neck disc in the layers it only
crosses, and a head and a cap at its ends. A link's claim is a pill of the link radius
along each segment of its outline, in its layer. A shape marked as a **seat** sits inside
another part's hole (an axle through its link) and is exempt from collision checks.

The planner guarantees one thing: **no two shapes with different owners in one layer ever
come within the margin (1 mm) of each other, at any crank angle.** Each shape's owner is
its group, except that every link owns its own shapes: the `links` group makes all the
links, but two links in one layer are still checked against each other.

Why claims rather than parts? Because the search needs cheap geometry. A distance between
two discs or capsules is a few arithmetic operations; a boolean between two CAD solids
takes milliseconds to seconds. Claims also keep the planner independent of how things are
built: the Chicago screw and the steel rod it replaced were different **constructions**
of the same group (a pin), which differed only in the radii and shapes their claims use.
Swapping one for the other touched no planner code. And a claim can refuse a layout outright, raising `Unbuildable` with a
reason the user will see.

How close do two moving shapes come? The planner samples every named point at 1,440 crank
angles (every quarter degree) and measures each distance at every sample. It also
measures, from the same samples, the farthest each point moves between two neighbouring
samples, and subtracts half the sum for the two shapes from the closest distance. Between
two samples a point is never more than half a step from the nearer of them, so the
result is a lower bound on the true closest approach over the whole turn, not an estimate.
(The bound treats each step as a straight line; the error of that is about 10⁻⁵ mm here.
For Klann the subtraction is up to 0.4 mm, so two shapes must be about 1.4 mm apart at
every sample to count as 1 mm apart.)

### 5.2 Static facts: what can be ruled out before searching

Some conclusions need no search, and the planner draws them first:

- **Keep-outs.** Each group declares the thinnest shape it will always have: an axle's
  neck over its whole span, the crank's journal on O. A link that comes closer to an
  axle's neck than the neck allows can never share the axle's layers. These **static
  clearances** become constraints (the running example has 28 of them per side).
- **Crank facts** (`spiderpig/construction/route.py`, `crank_facts`). A link "needs O
  free" when its outline comes within the journal's radius, the link's 6 mm and the 1 mm
  margin of O (10.5 mm with the default hex crank's 3.5 mm journal). For each such link,
  the planner records which crankpins it always stays at least the post radius, the link
  radius and the margin from (11.25 mm for the hex crankpin's 8.5 mm sleeve, 10 mm for
  `bolt_round`'s 6 mm standoff); the link may share a layer with those crankpins' posts.
  (A rider is carried on its own crankpin, so its layer is simply a run along that post;
  the rule is for the other links.) A
  link that clears no crankpin gets a search for a detour point: rings from 1 to 150 mm
  around O, every 2.5°.
- **The envelope** (`spiderpig/construction/underside.py`). A detour sweeps a full circle
  about O as the crank turns, so that circle must stay above the underside of the robot's
  body (its frame plates, the crank's own sweep, the servo and the patch of plate under
  it, and the robot's centre plates). Otherwise the robot would hang lower. The code calls
  this underside profile the envelope; `SideDesign.ground_clearance_mm` is the body's
  lowest point above the lowest foot point.

A link that no crankpin and no detour inside the envelope can clear makes the design
impossible as drawn, and the stage (`fabricate.static_stage`) stops with a `ClearanceError`
that gives the distances (`spiderpig explain --linkage K --module M` prints every stage's
verdict).
TrotBot's heel at the drawing's 7 mm unit is the standard example: link `b7` passes
crankpin J1 at 6.8 mm, under the 10 mm its `bolt_round` post there needs. Section 9.3 shows the fix.

### 5.3 The crank router

The crank is the one group whose shape depends on the whole layering, so it is a
**router**: the planner chooses its route for each layering it considers. Below the hub,
which always takes the top layers under the servo, each layer finds the crankshaft in one
of a few **states**: a stub on O in its bottom bearing, the journal on O, or a **run**
along a crankpin or a detour point. A run along a point over some layers has webs in the
layer below and the layer above it. Runs along one point whose webs share a layer or sit
in neighbouring layers join into a **chain**, which one screw holds together. In Figure 2
there is one run, along M over layer 2, with webs in layers 1 and 3: one chain and one
screw. TrotBot's single leg needs two runs along its crankpin J1, over layers 2–5 and over
layer 8; their webs in layers 6 and 7 are neighbours, so one screw spans the whole chain,
from the web in layer 1 to the web in layer 9.

**Figure 3.** The running example's crankshaft at the 2026-10-01 snapshot (the printed
crank): four runs, one per leg, each its own chain.

```
layer 11  inner frame plate (the servo above it)
layer 10  hub
layer  9  hub, web to M3
layer  8  post at M3        b1 of leg 3
layer  7  journal, webs to M2 and M3
layer  6  post at M2        b1 of leg 2
layer  5  journal, webs to M1 and M2
layer  4  post at M1        b1 of leg 1
layer  3  journal, webs to M0 and M1
layer  2  post at M0        b1 of leg 0
layer  1  journal, web to M0
layer  0  outer frame plate, the crank's stub in its hole
```

Each crankpin has one run, so each run is a chain of its own with its own screw, though
neighbouring runs share their web layers. The printed crank of the snapshot came out as
five segments, split at the runs: layers 0–1, 3, 5, 7 and 9–10; the `bolt` crank builds
the same route as a plate per web and a hex standoff per run (section 6.2). Legs 0 and 1 are a mirrored pair half a turn apart, so
M0 and M1 sit on opposite sides of O, as do M2 and M3; each pair's crank body of section
4.4 is a bar through O with a crankpin at each end.

`CrankRouter.route` finds the exact cheapest route for a complete layering by dynamic
programming over layers and chains. Only buildable routes count, and buildability comes
from the bolt crank's own rules (`JointRules`, `BoltCrank.joint_rules`):

- each chain is one run between two plates (no plate between two runs of one point: it
  would turn loose on the standoff), and needs a stock standoff that fits it
  (`JointRules.spans`);
  two chains share a plate or a journal standoff on O joins them (`j_spans`), and the
  last ends in the hub plate; the first chain's lowest web may sit only where
  the stub's stock standoff reaches (`bottom_layers`);
- the screw heads beyond a chain's plates need their clearance gaps (`gap_head`), and so
  do the horn screws' heads in the gap under the hub (`horn_heads`);
- the screw pockets of neighbouring chains, and of the last chain and the horn screws,
  must not meet;
- the washers a run carries through every clearance gap along it must clear what else is
  in that gap (`JointRules.gap_washer`). The search doesn't block these: at a leaf, the planner sets
  the bits of the gaps its plan has where the run's washers met another group's
  (`CrankRouter.washer_bit`, `_Search._washer_blocks`) and routes again;
- a rider can't sit in layer 1 (there would be no layer below it for its web) or in the
  hub's layers under the servo.

The router also offers the search its **gap pieces** (`gap_pieces`): one per clearance
gap slot (keyed `layer + 0.5`), blocked when another group's heads or washers stand in
that gap. They keep a route off gaps a screw head can't use, which is what makes the bolt
crank's plans fast.

Routes are ranked first by added features (run layers that no rider needs, detours), then
by how far a detour sweeps, then by whether the bottom bearing was dropped (an option, off
by default, that lets the crank hang from the servo alone). `plan.cost` encodes this
ranking as one number, 10⁹ per added feature, so a cost of 3,000,000,000 means three added
features, not a price.

The router also answers a question during the search: given only some links' layers,
does any route still exist, and which states can each layer still have? That makes it a
**sub-check** the search consults at every step.

### 5.4 The search

`StackProblem.solve()` tries **stack sizes** (numbers of layers) from the thinnest up. For
each size it searches over which layer each link takes (`_Search`). Each partial
assignment the search visits is a **node**, and a **node budget** caps how many it may
visit. Readers who know constraint solving will recognise the techniques; for everyone
else, here is what each does:

- **Most constrained first.** The next link to place is the one with the fewest layers
  left, and its candidate layers are tried nearest its already-placed partners first.
- **Forward checking.** Placing a link places its shapes, which immediately removes from
  every unplaced link the layers it could no longer take. Axles add their own rules: a
  link that can't pass an axle can't sit between that axle's links, and a pillar must
  still be able to reach a frame plate.
- **The router's sub-check** prunes the layers that would leave the crank no route.
- **Backjumping and learned conflicts.** On a dead end, the search jumps straight back to
  the most recent link actually involved in the conflict, instead of the last one placed,
  and remembers small conflicting combinations (up to eight links) so it never tries them
  again.
- **Branch and bound.** Once a plan exists, branches whose crank route can't be cheaper
  are cut.

Every complete plan is re-checked from scratch before it is accepted (`verify_plan`: every
claim re-made, every pair of shapes in every layer tested). A failure there is a bug and
raises an error.

**The effort schedule.** The search spends its effort in four steps:

1. **Rule out thin stacks cheaply.** From the thinnest size up, each size gets a short
   search (1,500 nodes). Sizes that it rules out are done.
2. **Jump ahead.** Once a size runs out of its short budget without an answer, the search
   moves up in doubling steps (one size, then two, then four) until a size has a plan,
   then goes back to the sizes it skipped.
3. **Try harder if nothing planned.** The sizes the short searches left open get the
   full budget (`max_nodes`, 20,000 nodes), thinnest first.
4. **Prove.** Every thinner size not yet ruled out is searched again with half the full
   budget (`max_nodes // 2`, `spiderpig/stack/search.py:299-301`), and the plan's own
   size gets the full budget once more to look for a cheaper crank route.

A multi-leg module first plans its `single` module and uses that plan as a hint for which
layers to try first, and for a second strategy that places one leg at a time. The search
itself is capped at 60,000 nodes in all and a **60 CPU-second deadline** (`stack.Deadline`,
the thread's CPU time, so a loaded machine doesn't change how far a search gets;
`SPIDERPIG_PLAN_SECONDS` sets it); the plan used as a hint shares it, and any
recommendation checks after a failure get one more.

**Where the heads go.** Every fastener head beside a link either sinks into the layer
beside it or stands in a thin clearance gap (section 5.7). `StackSpec.heads` picks:
`"sink"`, `"gap"`, or `"best"` (the default: sunk, and in gaps only when that finds no
plan). A design on the single-plate crank plans `heads="gap_sink"` the other way round
(`StackProblem.solve_heads`, `stack.HEADS_ORDER`): in gaps first, since a sunk crank head
would stand in a rider's layer; and only when that search gives up (after `stack.GIVE_UP`
layerings that fail on the crank's washers in the plan's gaps) with the pivots' heads sunk
and the crank's and the drive's (`stack.GAP_GROUPS`) still in their gaps
(`heads_claims(keep=...)`, `finalize(sink_all_but=...)`). TrotBot's heel and toe plan
only that way.

**The answer** is a `StackPlan`: each link's layer, the stack size, the crank route, the
cost, whether the plan is **optimal** (no thinner stack and no cheaper route, both
searched to the end), and a **proof** saying how that was established. For the running
example with the keyed crank of 2026-10-03 (since removed): 16 layers, 48 mm, optimal,
with the proof "no plan in 15 layers or fewer (240 nodes); the crank's own rules forced it
taller: in 15 layers the search met 239 layouts that fit everything else but no crank
route passes; 16 layers searched to the end for the cheapest crank route (17 nodes)"
(with the snapshot's printed crank: 12 layers, 36 mm, "no plan in 11 layers or fewer (48
nodes); 12 layers searched to the end ... (18 nodes)"). Today's plans and whether each is
proven are in [DESIGNS.md](agentlib/DESIGNS.md). A plan
that ran out of budget or time is still valid, because it was verified, but its proof
names the sizes left open.

### 5.5 When planning fails

A `PlanError` lists what blocked the search, with distances, the static clearances
involved, and the fate of each size: ruled out, left open when its budget ran out, or
never tried. At the 2026-10-01 snapshot (printed pillars), for the Jansen linkage scaled
down to a 1.5 mm unit, in the `decker` module, the search ran out of its 60,000 nodes. The error named the static clearance behind it,
"b2_leg0 passes pillar:A_leg0 at 8.7 mm, under the 9.0 mm" a neck needs (section 4.5),
and offered two checked fixes: a unit of 1.6, or a link radius of 5.5 mm.

Both kinds of failure, `ClearanceError` and `PlanError`, carry **recommendations** made by
`spiderpig/recommend.py`. A recommendation is offered only after the failed stage has been
run again with it and passed. The fixes it tries are:

- the smallest practical uniform scale of the linkage that clears every measured gap
  (practical: the scale parameter rounded up to a step of 0.5, or 0.05 below 2);
- thinner parts (`Params` such as link radius and axle diameters), within each
  construction's limits;
- the default scale, for a design that was scaled down;

The same checking covers one other case: a design that plans but misses a target that grows with the linkage (stroke, straightness,
lift) gets the smallest scale that meets it. These checks share one more 60 s deadline,
and anything left unchecked appears as a note, never as a recommendation. `spiderpig
explain` prints each stage's verdict, the plan's layer table, and any failure with what
would clear it.

### 5.6 What planned at the snapshot

This section is history: the sweep of 2026-10-01, with the constructions of the time
(the printed crank). The default designs' plans today are in
[DESIGNS.md](agentlib/DESIGNS.md); the strength sweep of every walker and module with the
current constructions is [docs/audit/STRENGTH.md](audit/STRENGTH.md). For the first
version of this report all 79 combinations of linkage and module were planned with the
default constructions of 2026-10-01, twice: once one at a time, and once three at a time
on a busy machine.

| outcome | cases |
|---|---|
| planned and proven thinnest | 67–68, most in seconds (the TrotBot deckers take up to 26 s) |
| planned, unproven (`optimal=False`) because the deadline or node budget ran out | 6–8: the six-bar and Strider quads, the TrotBot quads when they plan, the TrotBot decker under load |
| no plan | 2–5: the TrotBot heel and toe `double` always; the three TrotBot quads when the machine was busy |
| stopped at the drive stage | 1: `five_bar` |

Layers per side with the `printed` crank of the snapshot (unproven results marked *):

| linkage | `single` | `double` | `decker` | `quad` |
|---|---|---|---|---|
| Klann | 7 | 8 | 11 | 12 |
| Jansen | 8 | 9 | 13 | 16 |
| four-bar | 6 | 7 | 10 | 14 |
| six-bar | 9 | 12 | 14 | 24–25* |
| Strider | 10 | 16 | 16 | 28* |
| TrotBot | 12 | 12 | 20 | about 36*, or no plan |

Every mechanism planned in 6–10 layers in under 2 s. Two results need explaining:

- **The TrotBot heel and toe `double` couldn't be built.** A `double` puts both legs on
  one crankpin, so one chain of the crank must span both riders, and no layering of these
  links let a stock screw fit that chain.
- **The deadline made large quads machine-dependent.** On the busy machine the TrotBot
  quad failed; run alone, it planned in 36 layers, unproven, after 61 s. The six-bar quad
  came out at 25 layers on the busy machine and 24 without load. The deadline has since
  become CPU time (section 5.4), which removes the load's effect but not the machine's
  speed. A plan found is always valid; what varies is how thin it is, and whether one is
  found at all (section 12.2).

The planner also has four opt-in speed-ups, all off by default (section 11).

### 5.7 The plan's z: clearance gaps, layer thicknesses and sheets

The search works on a nominal stack, every layer one `pitch` (the default sheet's
thickness) and nothing between layers. The plan a layering becomes has real heights
(`stack.finalize`, in `spiderpig/stack/plan_z.py`; the same function re-makes a stored
plan, deterministically).

**Clearance gaps.** A fastener's head or nut beside a link (a Chicago screw's head, the
servo's screws, the crank's screw heads) is a *clearance shape* (`Placed.gap`, with the
`height` it needs and the layer it is `toward`). It either sinks into the layer beside its
link, where nothing there is in its way, or stands in a thin gap over or under the link's
layer. An axle crossing a gap claims its washers there. `finalize` decides which heads can
sink, gives every other head a gap sized to the tallest it holds (a gap a plate stack runs
through is a thin sheet's thickness, `materials.gap_options`; any other is a printed ring
in 0.1 mm steps up to 4 mm), makes each layer as thick as its thickest plate (`Placed.sheet`,
`StackSpec.link_t`, `frame_t`), and makes every claim again at that z (`Layout.final`), so
the crank's standoffs, the stub, the Chicago barrels and the pillar columns are checked at
the heights they will be built at. Where a stock part fits only with a gap a little
thicker, the gap is thickened (`_thicker_gaps`). A layering that doesn't build at its own
z raises `PlanReject`, and the search takes it as a dead end. The plan records its z:
`StackPlan.gaps`, `thick` and `heads`; a `Layout` answers `z`, `gap_z` and `slot_z`.

**Sheets.** A design has several: `BuildConfig.sheet` (3 mm acrylic: the links, rings and
deck), `frame_sheet` (0.080 in 5052, the frame and centre plates) and `crank_sheet` (0.100
in 6061-T6, `config.CRANK_SHEET`, or the linkage's own from `config.LINKAGE_CRANK_SHEETS`),
and `link_sheets`, links cut from another sheet (a Klann variant's foot links in 6061,
`klann_lego`'s crank rider `b1`: `materials.LINK_SHEETS`). The aluminium defaults are each
role's thinnest stock sheet that passes (`materials.thinnest_sheet(role)` over
`materials.ROLES`: the out-of-plane bending at a pin's clamped end under the 155 N jam load
at a safety factor of 2, the crank's hex pockets under the jam twist, and the role's
smallest hole against the service's minimum hole). The centre plates take the thinnest
stack that seats the most rear screws (`chassis.centre_sheet`). `Body.sheet` carries a
part's sheet to the mass, the BOM (a line per sheet), the DXFs (a set per service and sheet)
and the cut rules (section 7.1).

A plan says where every part may go. The next stage builds the parts.

## 6. Fabrication: stage 5

Section 5 ended with a plan: a layer for every link and a route for the crank, but no
parts. This stage builds the parts, each inside the space its group claimed, and then
checks that each one stayed there. It is also where bought parts, servos and mass come in.

### 6.1 How a side is built

`fabricate(template, config, t=1.0)` (`spiderpig/fabricate.py:289`) first gets the side's
`SideDesign` (its groups, claims and plan) from `design_side`, which caches it per
template and config. It then calls `fabricate_side`, which asks each group to **realize**
its parts at crank angle `t`, inside its own claims:

1. the groups that don't cut anything, in dependency order: drive, crank, pillars, pins;
2. extra groups, such as the robot's frame ties (section 6.2);
3. last, the groups that cut (links and frame plates), with every hole the others asked
   for, and every **pad** (an extra area added to a plate's outline, such as the patch the
   servo sits on).

Parts are rigid, so the angle only fixes the pose in which they are drawn; builds use
t = 1.0 rad. For a robot, `assemble_robot` then places the left side so that the mid-plane
is at z = 0, mirrors it to make the right side, and adds the chassis. The side is planned
once: both sides use the same plan.

### 6.2 What each group builds

Every group has one construction now: the `bolt` crank (`bolt_round` on TrotBot's heel and
toe), the `standoff` pillar and the `chicago` pin. The printed and keyed cranks, the
acrylic two-plate M6-bolt crank and the hex crank's variants, the printed snap axle, the
rod, bolt, bearing, bushing and PTFE-lined pivots and the spliced standoff pillars were
removed on 2026-10-07: a config, spec or stored design naming one fails with its
replacement (`config.REMOVED_CONSTRUCTIONS`, and `config.REMOVED_PARAMS` for their
`Params`); [DECISIONS.md](agentlib/DECISIONS.md) keeps why. Which crank a design gets is
data: `config.default_crank` reads `config.MODULE_CRANKS` (per linkage and module), then
`config.LINKAGE_CRANKS` (per linkage), then `config.DEFAULT_CRANKS` (per kind).

**Links and frame plates** (`construction/plates.py`). A link is a pill of the link radius
(6 mm) along each outline segment, minus its holes, cut from its sheet (section 5.7). An
aluminium link riding a crankpin grows a boss round its bore to 1 x its thickness of web
where its width leaves less (`plates.rider_bosses`, `RIDER_BOSS_T`), sized on the crank
resolved for the crank sheet, and claimed, so the planner sees it. Each frame plate is a
disc on O with arms to every pillar, plus pads, minus cut-outs; every round hole in a
frame plate gets a boss of 2 x its thickness (`plates.boss_web`), and pillars next to each
other about O are joined by a chord (`plates.chords`, which the underside reads too). Each
foot link carries a printed TPU 95A sock round its toe, in its own layer, snapped into two
notches (`foot_sock`); the BOM prints it in TPU (`hardware.bom.part_filament`: a filament
line per filament, PETG for a printed part pressed on metal, such as the capped crankpin's
sleeve), and the sim's floor friction is set for it (`SimParams.friction`, 0.65).

**The standoff pillar** (`construction/pivots/standoff.py`, `StandoffAxle`). A 6 mm round
standoff column runs from the outer frame plate's inner face to the inner plate's, a
button head and washer through each plate from outside (no glue; the inner heads stand in
the bay between the robot's inner plates, which the deck is notched round). The links turn
on the standoff (a running fit), and a printed spacer ring fills every other layer, so
every link has a face on both sides; where an aluminium link made a layer thicker, the
rings grow to take up the air (`ring_fill`), and every gap the column crosses takes a
printed gap ring. A column one stock goBILDA 1501 length fills is that aluminium M4
standoff; any other is **one piece** (`StandoffAxle.one_piece`): a MISUMI NETRF6 round 1018
steel standoff made to its length in 0.1 mm steps, tapped M3 both ends, an M3 button head
and DIN 9021 washer each end (`crank_catalog.pillar_shaft` writes its catalog key). It is
never spliced. A short column no stock length fills is refused unless goBILDA standoffs
spliced at link-free layers would have filled it (`StandoffAxle._spliceable`, the rule the
removed spliced pillar left behind, kept so the plans stay where they were); the axle's
`column` hook is how the planner learns that a column can't be built. A column up to 2 mm
short of its gap takes steel take-up shims under the face over it, in whole 1 mm and
`SHIM_STEP` (0.5 mm) steps (on M3 within `COLUMN_TOL`, 0.25 mm, of the gap), in the
clearance gap there with its gap ring trimmed, else in the spacer layer; the note's
`shims_mm` lists them. A link sweeping close to a pillar stops it short of one plate (a
ring can't neck): it is then a cantilever from the other plate, its free end closed by the
same screw and washer, and the clearance gap there takes a gap ring so the link can't
slide.

**The Chicago screw pin** (`construction/pivots/chicago.py`, whose docstring holds the pivot
review's table and the assembly steps). An M3 Chicago screw's 4 mm barrel runs the pin's
whole stack; the screw's head bottoms on the barrel's end, so the head-to-head distance is
the barrel length however tight the screw is, and the links turn between the heads.
Barrels come in fixed lengths (`fastener_catalog.CHICAGO_LENGTHS`: 1 mm steps from 4 to
16 mm, then 18, 20, 22, 23, 25, 28 and on to 80 mm); the construction picks the shortest
that clears the stack and takes up the rest with one printed head spacer per end, which
sets the column's axial play (0.05-0.15 mm) by the barrel length rather than by feel. That
a pin's links fit a stock barrel is a planner rule (`ChicagoShaft.column`), and on the
Strider barrels are capped at 23 mm (`chicago.MAX_BARREL`, applied in
`ChicagoAxle.resolve`), since a long barrel is a long, weak span. The lowest link is bonded
to the barrel with epoxy, so the barrel is held by holding that link while the screw is
driven from above. Every pivot's rings are printed (an unclamped spacer only sets play);
a metal shaft can't neck, so every layer it crosses holds a spacer as wide as its
narrowest ring.

**Strength.** The audit's strength check (also `verify` standard's `strength.joints` row
and `explain --strength`; `spiderpig/strength.py`,
[docs/audit/STRENGTH.md](audit/STRENGTH.md)) takes each design's own MuJoCo pin loads
(walking 99th percentile, and jammed at the servo's torque limit, which
`config.LINKAGE_TORQUE_LIMITS` lowers per linkage), bends a two-link pin by `F s / 2`,
rates a pillar as a beam per bay between its supports (the plates' faces) or as a
cantilever, and fails a joint under jam SF 1, warns under 2 jammed or 3 walking. Every beam
is at the plan's own z (`wobble.column_wobble`'s `layer_z`, `wobble.layer_mid`): each link
at its layer's mid-plane, a pillar's supports at the plates' faces, clearance gaps and
thicker plates included. Every link plate is rated too (`strength.link_rows`: net section
at the most loaded hole, plus bending for a link of three or more pins), naming the
aluminium sheet a link would need. The audit also reports each link's tilt out of plane
(`construction/wobble.py`): free (bore clearance over bearing length) and held by the faces
beside it; a link beside a clearance gap bears on what the column holds there (an end's
head or printed spacer, a gap ring).

**The bolt crank** (`construction/crank/`: `BoltCrank` in `bolt.py`, its fits in `hex.py`
and `web.py`, its ratings in `capacity.py`, its parts in `plates.py`'s `_WebPlates`). It is
laser-cut from the crank sheet (0.100 in 6061-T6 by default; `BoltCrank.for_sheet` and
`resolve(ctx)` read the sheet's thickness and yield, and a non-metal sheet is refused).
Every web is one aluminium plate, on its layer's floor, the hub plate at its layer's top
(`BoltCrank.plate_z`). With the default `pin="hex"`, every crankpin and journal is a stock
M3 x 5.5 mm across-flats steel hex standoff (`crank_catalog.HEX_M3_LENGTHS`) whose ends sit
in hex pockets of its two webs (`BoltCrank.hex_cut`: 0.1 mm over the across-flats, with a
dog-bone relief of the service's 0.8 mm inside radius through each corner, so the flats
stay whole); an M3 button head and a DIN 9021 washer screwed into each end retain the
plates. Where the stock length stands past a plate (at most `protrude_max`, 1.2 mm, per
end, each end's stack inside a 4 mm gap), a printed hex-bore collar takes it up, or both
ends sit up to `recess_max` (0.3 mm) inside their pockets (`fit_hex`, which splits what
stands past the plates so the taller gap need is least, the upper end using the air over
its plate, `air_over`). Where no stock length fits a chain at the plan's z, `hex_gap_fit`
opens the gaps along it (the one over its lowest web first, each to at most 4 mm, printed
rings on the sleeve filling them) to the next stock length. The riders turn on a printed
8.5 mm sleeve over the hex (`rider_d` sizes their holes), and printed rings fill the gaps
along a run. Two chains share a web, or a journal standoff on O joins them; the last
chain's top web is the hub plate, held to the horn by the horn screws, which come up
through it from below (their heads in the gap under it).

The chain that ends in the hub plate is **capped**: it has no screw over the hub plate
(`hub_capped`), so the hub plate, horn, servo and inner plate go on as one unit
(`construction.robot.ASSEMBLY`, the whole robot's assembly order, which the pivots' and the
crank's docstrings defer to). Its printed sleeve is a light press on the hex
(`BoltCrank.capped_press`, 0.1 mm), so the sleeve, caught between its plates, carries the
standoff. The crank body stops toward the outer plate on a printed thrust sleeve round the
stub (`stub_thrust` in the crank's note, 8.5 mm, its end 0.1 mm over the outer plate,
claimed in the stub's layers and gaps: `stub_thrust_r`) and toward the hub on the capped
sleeve. The stub is an M3 round standoff screwed up into the lowest web, turning in a
6.6 mm hole of the outer plate (the bottom bearing).

The horn screws take shims under the head where a stock length is too long
(`horn_fit_web`: whole 1 mm ones, bought as DIN 433 pairs, where the horn keeps enough
thread, else 0.1 mm DIN 988 steps). The horn holes keep the service's minimum hole and
2 x t edge distance, the hub's and webs' rims 1 x t (`BoltCrank.web_edge_t`, the cut rules'
error level). A crankpin within a screw head's reach of the horn's rim makes the printed
horn spacer a layer thicker (`BoltCrank.hub_head_need`, `DriveGroup.spacer`). The crank is
rated by the hex's bearing in the plate's own pocket depth at the weaker of the plate's and
the 300 MPa steel's yield (`hex_bearing_nm`, `hex_capacity`); the capped hex is rated in
the hub's depth less the press's 0.1 mm.

With `pin="round"` (`--crank bolt_round`: TrotBot's heel and toe through
`config.LINKAGE_CRANKS`, whose `b7` clears a 6 mm crankpin but not the hex's 8.5 mm sleeve),
every crankpin is a goBILDA 1501 round 6 mm standoff clamped between its two webs by an M4
button head into each end (heads in the clearance gaps beyond the webs, DIN 988 shims and a
spacer gap taking up the standoff's length past the layers: `fit_web`), and the riders turn
on the standoff. It is a friction clamp, rated in `BoltCrank.capacity` with unverified
coefficients. Its clamp needs a screw over the hub plate as well, so a `bolt_round` chain
that ends in the hub plate has no assembly order (`_WebPlates.assembly_issue`, the crank
note's `assembly`, an `assembly:` audit error).

The router's rules for the crank (`BoltCrank.joint_rules`, `JointRules`) are in section
5.3; the crank's plans use `heads="gap_sink"` (section 5.4).

**The drive** (`servos/mount.py`, `DriveGroup`). The servo sits on top of the inner plate,
output face down, its axis on O, its body pointing away from the pillars. Its horn is
turned on the toothed output shaft so the horn screws fall between the crank's webs, and a
printed horn spacer sits between the horn and the hub plate. A linkage with a second input
stops here with `ConstructionError`: there is one servo per side, so a second input has no
drive. Every screw that comes up through the inner plate from the leg side (the servo's
front screws, the frame ties', the deck rails') is the drive group's claim, its head in the
gap under the plate (`DriveGroup.claims`, `chassis_screws`; the tie and rail positions are
known before a plan: `chassis.tie_points_ctx`, `deck.rail_screw_points`). A front screw
hole with less than `Params.servo_screw_web_t` (1.0) x t of web to the horn's hole or a
relief is left out (`DriveGroup.screw_web`; the front reliefs are cut at the model's
rectangle, `mount.RELIEF_CUT_GROW`); the servo uses all four front screws when the crank's
hub sits a layer under the horn (`horn_layers`); `servo_screw_web_t=0` keeps all four.

**The robot's chassis** (`construction/robot.py`, `chassis.py`). The two servos sit back to
back, their rear faces screwed into a stack of centre plates cut from the frame's
aluminium, the thinnest stack that seats the most rear screws (`chassis.centre_sheet`).
Four **frame ties** join the two inner plates beside the servos: per tie and side a chain
of 6 mm round M3 standoffs from the inner plate to the centre plates (stock lengths, 1 mm
shims), an M3 button head up through the inner plate from the leg side, and an M3 set
screw through the centre plates joining the two chains and clamping the plates between
them (no adhesive). The ties move along the servo until their holes are 2 x t off the
servo's screw holes and recesses (`chassis.tie_locals`, `tie_neighbours`, which the
underside reads too); the centre plates' outline keeps 2 x t round each rear screw recess
(`chassis.recess_wall`), and their bump reliefs have 1 mm corners (`RELIEF_CORNER`). The
servo's bus sockets are in its rear connector housing, so `ServoSpec.bus_ports` gives the
centre plates an open slot from the housing to their far edge (`chassis._port_slots`;
`opening="pocket"`, a closed pocket with no access, is kept to compare; the
rear holes within 2 x t of it are dropped, so each servo keeps one rear screw). The
STS3215's rear idler horn stays in its box (`Idler.fitted`; `servos.model.unfit_idler` cuts
it from the CAD model), and the plates clear its 6 mm boss with a round relief
(`Relief.round`, `chassis.RoundRelief`). Reliefs closer than the service's web are merged
into one cut (`chassis._merge_close`), and a head recess that close opens into the relief
(`_recess_bridges`).

**The electronics deck** (`construction/deck.py`). A laser-cut plate between the inner
plates over the servos, on two printed rails screwed to the inner plates (M3 from the leg
side into nut traps in the rails), carrying the ESP32 servo driver, a 2S LiPo in a printed
cradle screwed to the deck and strapped, the charger, the protection board and a toggle
switch (the catalog in `hardware/electronics.py`, each item with its mass). Nothing moves in
its z band, and `deck_clearance` proves it over the whole cycle. The deck lowers straight
down past the pillars' inner screw heads: `deck.path_notches` notches the plate round every
static part in its way, `deck.insert_z` moves the rails' inserts into the bay where a notch
would leave a deck screw hole too little web, and `deck.deck_path` checks the way on the
parts' geometry (`deck_clearance`'s `blocked`, an audit problem).

The chassis and the deck sit outside both sides' stacks, so the planner never sees them;
they are checked for collisions by intersecting solids at sampled crank angles (and the
deck's z band over the whole cycle).

### 6.3 The contract, and how far the guarantee goes

The planner promises that claims never meet. The **contract**
(`construction/contract.py`) checks the other half: that each part a group builds lies
inside that group's own claims. `check_side` realizes every group at a crank angle and
measures any volume outside its claims (tolerance 0.001 mm³). Frame plates must stay in
their layer; the drive is checked below the inner plate's top face. Two companion checks
use OCCT directly: `clashes` intersects every pair of parts, and `bad_solids` requires
each made part to be one valid solid.

Together, the planner and the contract make the correct-by-construction argument. Not
every part of it is checked equally densely:

| what is checked | where | how densely |
|---|---|---|
| the loops close; a mechanism keeps its promises | template stage | 720 crank angles (section 4.3) |
| claims of different owners never meet | the planner | 1,440 angles, with a bound that covers the motion between them |
| the plan, independently | `verify` standard, `audit`, a plan stored by another engine version | 2,880 fresh angles, the same bound |
| each part inside its claims (the contract) | tests, `verify`, `audit` | 2–4 crank angles |
| no two solids intersect, chassis included | tests, `verify`, `audit` | 1–4 crank angles |

The planner's half is a sound bound over the whole turn. The contract and the clash checks
are sampled. Since parts are built from the same moving points as the claims, a check at a
few angles is strong evidence, but it is not a proof, and the chassis has nothing else.

### 6.4 Bought parts, servos and mass

- **The parts catalog** (`hardware/catalog.py`, data in `parts.py`, `fastener_catalog.py`,
  `crank_catalog.py`, `sheet_catalog.py`, `electronics.py`): at the snapshot, 67 items with
  162 offers, 48 of them verified (the vendor's page was fetched and showed the product)
  and 32 priced; the rule is "nothing is priced from memory". Every bought item's first
  offer is a direct product page (`hardware/sources.py`). Every screw family comes from
  one table, `hardware/fasteners.py`.
- **Mass** (`hardware/mass.py`): one density table, printed parts at 100 % infill (an
  upper bound), servos at their datasheet weight, and exact volumes and inertias from
  OCCT. The animated model, the walking model and the MuJoCo model share these numbers.
- **Servos** (`servos/`): three continuous-rotation servos, the Feetech STS3215 (the
  default: 19.5 kg·cm of torque, 52 rpm, 55 g), the ROBOTIS XL430-W250 and the XL330-M288.
  Each has a datasheet in code with every dimension cited. Published STEP models of the
  servos (the STS3215's from the open SO-ARM100 robot-arm project, the others from
  ROBOTIS) are downloaded at build time, checked against a pinned sha256 hash, cached in
  `~/.cache/spiderpig/cad`, and never committed (the ROBOTIS models and an alternate
  STS3215 model state no licence). Offline, or on any download problem, a parametric model
  drawn from the datasheet takes their place.

### 6.5 Limits of fabrication

- Strength is checked for the joints and link plates only (section 6.2), as beams at the
  simulated loads, with unverified friction coefficients for the `bolt_round` clamp; the
  frame and centre plates are dimensioned by the cut rules and the sheet roles, not by
  loads.
- One crank construction (the `bolt` crank, round or hex crankpins), and only
  continuous-rotation servos, one per side.
- The chassis, frame ties, centre plates and deck are outside the planner's guarantee.

With every part built, the last stage writes them out.

## 7. Outputs: stage 6

Every output is written from a fabricated `Mechanism`, never from a separate model of the
robot. There are two fabrications: STEP, STL, print STLs, DXF and BOM come from one at
t = 1.0 rad; the glb and the MuJoCo model come from another at t = 0. Parts are rigid, so
the two differ only in pose, and files from one fabrication can't disagree about geometry
or counts.

### 7.1 Files for making it: `spiderpig build`

`spiderpig/build.py` plans the side (through the store, section 9.5), fabricates the robot
(served from the fabrication cache, `spiderpig/fabcache.py`, when the parts' code and the
design are unchanged) and writes the files below. A build into a folder that already
holds its outputs, from the same sources and options, does nothing
(`spiderpig/uptodate.py`; `--force` builds anyway). The running example's column is the
2026-10-01 snapshot's.

| file | contents | running example (snapshot) |
|---|---|---|
| `klann.step` | the whole robot, one named, coloured product per part | 11.4 MB, 173 solids |
| `klann.stl` | the whole robot as one binary mesh | 21.8 MB, 436,932 triangles |
| `print/*.stl`, `print/parts.csv` | one STL per distinct printed part, with quantities, grams and filament; a `_mirrored.stl` only where a part is its twin's mirror image (`bom._proper_fit` tries a pure translation first, so a mirror-symmetric ring counts as the same part) | 16 parts and 1 mirrored |
| `laser/<name>_sheet_<service>_<sheet>_<i>.dxf`, `laser/<name>_sheet_parts.csv` | the laser-cut parts packed onto sheets, one set per cutting service and sheet stock (`layout.save_sheets`) | 2 sheets of 300 × 300 mm |
| `laser/parts/<service>_<sheet>/<part>_x<qty>.dxf`, `laser/parts/order.csv` | the same parts, one DXF per distinct part (blue `CUT` layer, R2007), with each file's material, thickness and quantity: SendCutSend and Ponoko take one part per file (`layout.save_parts`) | not in the snapshot |
| `bom.csv`, `bom.md`, `bom.json` | the bill of materials | 11 purchase rows, at least $124.87 |
| `ORDER.md` | the shopping list (`hardware/order.py`) | not in the snapshot |

`api.export` (and `spiderpig export`, the MCP's `export`) writes the formats a spec names
(section 9.2); its `dxf` is the packed sheets only: `ORDER.md` and `laser/parts/` come
from `spiderpig build`.

**DXF sheets** (`spiderpig/layout.py`): each laser part is cut through its mid-thickness,
turned so its long axis runs along x, and offset by half its sheet's kerf where the
service doesn't compensate for it (`layout.sheet_kerf`: the sheet item's `kerf_mm`, 0.2 mm
at Ponoko, so outer contours grow by 0.1 mm and holes shrink by as much; 0 at SendCutSend,
which compensates itself, so its files are nominal; `--kerf`, or a spec's `fit.kerf_mm`,
overrides every sheet). The parts' bounding rectangles are packed with `rectpack`, and any
part that doesn't fit raises an error rather than vanishing. Round holes are written as
exact circles; every other contour is one closed `LWPOLYLINE` whose lines and arcs are
exact (an arc is a vertex's bulge), any other curve flattened within 0.02 mm (`CHORD_TOL`).
`layout.fidelity` reads the contours back against the solid (the cut-rule review's `dxf`
check).

**Cut rules** (`spiderpig/manufacture.py`): every laser-cut part is reviewed against its
sheet's service (SendCutSend for aluminium, Ponoko for acrylic): the minimum hole, the edge
distance from a hole to an edge or another hole, the web round every non-circular cut-out
(`web`: to the edge, a hole or another cut-out), the minimum part and the inside-corner
radius, and that the DXF matches the solid. In metal a hole closer than 1 x the thickness
to an edge or hole, or under the service's minimum hole, is an **error** (the audit fails);
under 2 x it is a warning. Every issue carries its `why` and a `fix`. `manufacture.messages`
turns them into the audit's problems and warnings (a "cut rules" column and a per-part table
in `audit.md`, `manufacture` in `audit.json`) and `verify` standard's `manufacture.cut_rules`
row (errors: the `manufacture` / `cut_rule` failure) with a soft `manufacture.warnings`;
`manufacture.summary` is the build report's `cut_rules` (`api.BuildReport`, the MCP build's
too) and the design card's (`/api/design/{id}`, `api.cut_rules`: read from the stored build,
never fabricating).

**The BOM** (`hardware/bom.py`): every purchased body is one unit of its catalog key.
Laser-cut and printed parts are grouped by shape. Two parts join a group only if their
volume, area and principal moments agree and a boolean intersection proves that they
coincide; a right-side part that mirrors its left twin joins without the boolean. A laser
part and its mirror image are the same cut (flip the sheet); a printed part and its mirror
image are different prints unless one is a translate of the other. A line per sheet for
the plates, a line per filament for the prints (`hardware.bom.part_filament`). Shims are
ordered one line per thickness (`bom.split_shims`, `hardware/shims.py`), the clamped 1.0 and
0.5 mm ones bought as DIN 433 washers (`bom.SHIM_AS`). Purchases are rounded up to whole
packs of the preferred offer. The total leaves out rows with no price, and the BOM says so;
for the running example at the snapshot the unpriced rows were 8 M2 self-tapping screws and
4 M3 × 18 mm screws, and the made parts came to 39 laser-cut parts in 8 shapes, 90 printed
parts in 16 shapes, and about 95 g of plastic.

**The shopping list** (`ORDER.md`, `hardware/order.py`): a cart per vendor, each line the
item's first offer, a direct product page (`hardware/sources.py`, checked in a browser
where the offer is `verified`; McMaster-Carr's prices need a login); the uploads per
cutting service; the prints per filament. Shop supplies (filament, threadlocker) are taken
as on hand (`bom.ON_HAND`): listed, not ordered or totalled. An unpriced line is estimated
from its first priced alternative and totalled apart. A design's servo torque limit
(`config.LINKAGE_TORQUE_LIMITS`) appears as a servo-firmware note.

### 7.2 The animated model: `spiderpig bake`

`spiderpig/bake.py` writes one self-contained glb of the robot over one crank turn, for
the browser viewer. A built-in profiler times its stages, and their names are kept stable
on purpose, because scripts parse them:

| stage | does | running example (snapshot) |
|---|---|---|
| `1_reference_build` | fabricate the robot at t = 0 | 16.1 s (56 %) |
| `2_mesh_share` | let a body reuse another's mesh: candidates by body name, confirmed by exact mass properties | 1.7 s |
| `2_tessellate_total` | mesh each distinct part with OCCT | 9.7 s (34 %) |
| `3_gltf_pack_geometry` | pack positions, indices and one material per kind of part | under 1 s |
| `4_animation_sample_total` | sample the template and fit each body's motion | 0.09 s |
| `5_gltf_nodes_channels` | one node and one animation channel per body | 0.7 s |
| `6_foot_path_extra`, `6b_drive_extra` | the foot path, and the data the viewer's drive mode needs | under 1 s |
| `7_serialize` | write the file | under 1 s |

At the snapshot the running example's glb was 9.5 MB: 179 bodies, 173 of which carry a part, drawn with 139
meshes, baked in 28.6 s (36 s on the command line, imports included). Bakes are cached in
the store as `bakes/<config key>.glb`. The drive data written into the file (every foot's
path, the centre of mass, the mass, the servo's speed) let the viewer drive the robot
without asking the server.

### 7.3 The physics model: MJCF

`spiderpig/sim/mjcf.py` writes one self-contained MuJoCo model. A free-floating base
carries the frame, servos and chassis; each side's crank turns on a hinge at O; each link
is a body on a hinge; the joints that close each loop become equality constraints. Masses
and inertias are exact, taken from the fabricated parts. Contacts are computed only
between the robot and the floor; self-collision is off, because the planner already
guarantees the parts never meet. Feet are spheres, links capsules, and the base is made of
convex hulls (every link capsule stops short of a foot joint, so the foot spheres alone
touch the floor). Each side has a velocity-controlled motor limited to the servo's speed
and stall torque, and every step narrows its torque to a DC motor's speed-torque line
(`motor_line`: the loop alone would deliver the stall torque at the no-load speed, which no
servo does; the clamp cut the running example's peak from 1.26 to 0.43 N·m and costs 2 %
of speed). The model also maps every node of the glb to its MuJoCo body, which is how the
viewer's physics drive (8.4) poses the baked meshes. At the snapshot the running example had 35 bodies, 16
loop constraints (joints C and E of each of the 8 legs) and a mass of 460.6 g: sheet 227 g,
servos 110 g, printed parts 95 g, and 29 g of screws, horns and inserts. The crank's
reflected rotor inertia (`crank_armature`) is an unverified estimate; the speed is
insensitive to it over 5e-4..2e-2 kg·m², the torque peaks are not (0.89 to 0.16 N·m), and
below 5e-4 the model is ill-posed and refused.

### 7.4 Shared machinery

- **Meshing** (`spiderpig/mesh.py`): OCCT's mesher on each part. The triangles are read
  back through OCCT's own glTF writer, about eight times faster than walking them from
  Python (project docs). The bake meshes each shared part (`mesh_part`) and reads them all
  back at once (`read_meshes`); the MJCF's hulls use `tessellate` / `tessellate_many`.
  Faces the mesher leaves untriangulated (three in the XL330's model) are skipped and
  counted rather than crashing the bake.
- **Worker processes** (`spiderpig/workers.py`): OCCT's Python binding holds Python's
  global interpreter lock, so threads don't speed up CAD work. Forking a process after
  OCCT has started its thread pool deadlocks, and `multiprocessing`'s spawn mode
  re-imports the caller's main script. Parallel work therefore runs as a function of the
  package in a fresh `python -c` process, with arguments and results passed as files.
  Exports and `verify` use workers; `SPIDERPIG_WORKERS=0` keeps everything in one process.
  `spiderpig build` starts one before it fabricates: it loads the fabrication from the
  store's cache once that holds it, groups the parts and writes the DXFs while the build
  writes the robot's STEP and STL (`build._start_exports`; with the cache off, in-process).

### 7.5 Limits of the outputs

- **DXF accuracy.** Lines and arcs are written exactly (at the snapshot, a 96-point
  polyline per outline cut up to 0.47 mm inside a long link's rounded end); only a curve
  that is neither, which none of the plates has, is flattened, within 0.02 mm. The kerf is
  the service's published figure, not a measured one: cut a test coupon.
- **Sheet use.** Packing is by bounding rectangle, not true nesting.
- **Printing.** Print STLs keep their as-built orientation; nothing chooses how to lay a
  part on the printer's bed or where supports go.
- **Cost.** The BOM total is a lower bound. Unpriced rows are left out (McMaster-Carr
  shows prices only behind a login; `ORDER.md` estimates such a line from a priced
  alternative and totals those apart). Every row is costed in whole packs of its preferred
  offer (a 100-pack for 8 screws, a whole spool for 95 g). The prices are snapshots, the
  default build's from the sourcing round of 2026-10-05.
- **Reproducibility.** STEP files differ byte for byte between runs (the order of their
  colour records); compare geometry, not hashes.
- **The glb's size.** Mesh sharing is decided by body names, so the 24 identical pin heads
  of the running example are 24 separate meshes.

The files describe a robot that should work. The next section is about whether it walks.

## 8. Seeing and judging a design

A design that builds may still walk badly. Three models estimate how it walks: a
quasi-static model (8.1), the MuJoCo simulator (8.2) and a simple kinematic gait (8.3).
The running example's figures in this section are the 2026-10-01 snapshot's, with the
constructions and masses of the time. A
server and a viewer show it (8.4), and three command-line tools check, tune and compare
designs (8.5).

### 8.1 The quasi-static walking model

`spiderpig/walk.py` estimates how the robot stands and moves from the kinematics alone, in
a fraction of a second, with no physics engine and no parts. (Its first call for a linkage
and module plans the default design to learn where the feet sit across the robot, which
takes as long as a plan.) *Quasi-static* means it ignores inertia:
at each moment the robot is assumed to be at rest. The browser runs the same model in
TypeScript (`viewer/src/drive/model.ts`), so the viewer can drive and tune a robot
instantly. Its assumptions:

- flat ground, and feet that are points;
- at every moment the robot rests on the three or more feet that form the plane it would
  settle onto, the one under its centre of mass;
- the body moves with the velocity that keeps its feet on the ground from sliding, found
  by least squares; whatever residual is left over is reported as slip;
- the centre of mass is fixed (its average over the turn), and each foot's position across
  the robot is the middle of its link's layer in the plan.

For the running example it gives 102.4 mm of travel per crank turn, about 89 mm/s at the
servo's 52 rpm, and 23.7 mm of **bob** (how far the body rises and falls each turn).

One consequence trips everyone up: **a robot with two or fewer feet a side travels 0 mm
per turn in this model**, and that covers the `single`, `double` and `decker` modules of
most linkages. Each foot on the left moves exactly like one on the right. With one foot a
side the robot can't stand at all. With two, its four feet form two mirrored pairs, and
four such feet always lie in one plane, so all of them touch the ground all the time and
none ever lifts to swing forward. The body then moves opposite to the feet's average
motion (found by least squares; the remainder counts as slip), and since every foot goes
round a closed loop, that average comes back to where it started each turn; so does the
body. Only the `quad` of every linkage travels, along with Strider's `double` and `decker`
and some TrotBot variants. The API attaches a note saying so to every such design, and the
linkage catalog's cards mark which modules walk. Whether a real Klann `double` would
shuffle forward isn't modelled.

### 8.2 MuJoCo

`spiderpig sim`, and `verify` at level `full`, run the MJCF of section 7.3 in the MuJoCo
simulator, with gravity, friction (coefficient 0.65 since the TPU feet; 0.5 at the
snapshot) and soft contacts. For the running example at the snapshot, with both servos at 80 % of their no-load speed (a loaded servo can't reach the
full no-load speed), 4.5 simulated seconds take
4.2 s of wall-clock time. The robot walks at 134 mm/s, 193 mm per crank turn, with 24 mm
of bob and 8.6° of peak tilt. It doesn't fall; its peak torque is 0.59 N·m, 31 % of the
servo's stall torque, with the drives held to the motor's speed-torque line (1 % of the
time on it at 80 %, 77 % at a full command), and its mean load is 8 % of the rated 0.49
N·m. One side stands on a single foot 41 % of the time, which is why it cannot steer
(8.4).

Three assumptions sit behind those figures, and the model now states them
(`spiderpig/sim/mjcf.py`, "What the model assumes"):

- **The sides are phase-locked.** Every straight-walk figure comes from two cranks that
  keep their phase. Two servos told the same speed open-loop do not: with the right one
  2 / 5 / 10 % slower the Klann quad rolls over at 17 / 8 / 3 s, and a held side offset of
  90° or more rolls it over within a second (measured). The sim therefore runs the
  controller the real STS3215 bus needs, `sim.run.PhaseLock`: a PI lock on the sides'
  crank difference (kp 1, ki 4 in fractions of the no-load speed per rad and rad·s), the
  right servo 3 % slower than told (`SimParams.servo_mismatch`), the correction split
  between the drives. With it the sides stay within 0.7°, a 5 % mismatch walks 10 s at
  under 15° of tilt, and the heading still drifts 1.3°/s at a full command (the servo on
  its torque line has little speed authority, so the weaker side pushes less; 0.2°/s at
  80 %). The lock also bounds a steering excursion: `steering_check` finds the largest
  side offset the design walks with (`step_deg`: 45° for the Klann quad at 18° of tilt;
  90° and 180° roll it over), and the live session re-locks after it. Position feedback
  and this controller, not open-loop wheel mode, is what the hardware must provide.
- **A rigid crankshaft** (one body per side) carries the pin loads, which the metrics now
  report (`loop_force`: the in-plane constraint force per loop, 99.9th percentile and
  peak; the Klann quad's walking peak is of the order of 100 N, the jammed load 155 N),
  with the base's peak vertical acceleration (7.5 g at full speed) and the airborne
  fraction per revolution (2 %); the passive pins carry a Coulomb friction estimate
  (`pin_frictionloss`, 1.5 mN·m); at the snapshot the frame carried a 100 g payload for a
  battery and board the BOM didn't list (`payload_g`; 561 g in all), which is 0 now that
  the deck's electronics are parts with their catalogued masses.
- **The contact softness is unvalidated**, as before; the support and impact figures
  follow it, and the torque peak is a range (0.33–0.60 N·m over the servo's loop
  stiffness, 0.16–0.89 N·m over its rotor inertia) that `compare_with_walk` notes.

The wobble itself is kinematic, not numerical: binned against the crank angle, the sim's
base height and pitch match the quasi-static support plane of 8.1 within 0.6 mm and 0.3°
rms (the means apart: the base sits a link radius higher), unchanged from a 2 ms step to
0.25 ms (`tests/test_sim.py::test_the_sim_height_and_pitch_follow_the_quasi_static_support`).
A design that covers no ground is now flagged by both models (`walks`: Klann's double,
whose four feet stay coplanar, strides 2e-13 mm in the quasi-static model and crawls at
11 mm/s with its body on the floor 16 % of the time in MuJoCo).

### 8.3 Three estimates that disagree

The third and simplest estimate, the kinematic gait in `spiderpig/sim/run.py`, assumes the
body rides on whichever foot is lowest. For the running example the three disagree by a
factor of three:

| estimate | how | travel per crank turn |
|---|---|---|
| quasi-static model (viewer, `/api/walk`, `verify`) | rests on its feet; feet don't slide | 102.4 mm |
| MuJoCo | dynamics, friction, soft contacts | 192 mm, with heavy foot slip |
| kinematic gait | the body rides the lowest foot | 296 mm |

They disagree on whether some robots stand, too. MuJoCo has the TrotBot heel quad roll
over within a second, while the quasi-static model sees no tipping at all and a 45 mm
**stability margin** (how far the centre of mass is inside the feet's support area); these
figures are from the project docs (test-drive round 5). The `verify` report says when and
how the simulated robot fell, but not why, and the repository holds no measurement of a
built robot that could say which model is right. Section 12 ranks this as the largest open
risk.

### 8.4 The server and the viewer

`spiderpig/server/app.py` is a FastAPI web server. `/api/glb/{mode}` bakes on demand and
caches per design; `/api/walk` answers the walking model for a design without building
parts; `/api/linkages` lists the linkage catalog; `/api/design/{id}` describes a stored
design; `/ws` pushes live reloads during development. The viewer (`viewer/src/`, three.js,
built with Vite) plays and scrubs the animation and has two panels:

- **Drive** steers the robot with keyboard or gamepad by per-side crank speed, and shows
  contacts, slip and stability live, on the quasi-static model (8.1). Its **physics**
  toggle hands the same parts to the MuJoCo model instead: the server (`/ws/sim`,
  `spiderpig/sim/live.py`) builds the MJCF of the glb on screen in a worker process,
  steps it in real time (60 frames a second, deadline-ticked, the physics in a thread)
  and streams every body's pose with the drive torques; the viewer interpolates between
  frames and reads speed, yaw rate, torque against the servo's ratings (over the rated
  torque for two seconds is a stall warning), the sim/wall ratio (a loaded server runs
  the session in slow motion, and says so) and per-revolution stats from a short history
  of them. Commands go up on every key change and at 20 Hz besides, so a slow or hidden
  tab can't leave the robot walking on its last key. The two modes are exclusive with
  the stick preview. The gate is MuJoCo's own verdict: the server's steering check (run
  once per model build, `sim.run.steering_check`) starts with a straight run, and a
  design that fell in it is refused; the quasi-static margin under 15 mm is a warning
  in the status (Jansen's quad, 4 mm, walks at 17° of tilt and connects). The check then
  runs a 0.4 differential while walking, a half-speed turn in place and the bounded
  excursions, four seconds each; the default Klann quad rolls over in the differential
  and tilts 23° in the spin, so it gets 0 for both, walks a 45° excursion, and the HUD's
  steering row says so ("45° excursions only"; the steering tests also record the
  differentials as expected failures). The turn authority defaults to what the server
  proved (the slider only raises it beyond, with the warning); a design with nothing
  proven shows "steering DISABLED" and a banner when a steering key is held, instead of
  silently walking straight. A glb whose nodes the model doesn't name is refused rather
  than half-animated. The websocket reader never touches the simulation state under the
  stepping thread: a reset is requested and applied by the next tick; a reset is a new
  epoch of the sim clock on the client (every HUD history starts over, the real-time
  factor is clamped at 1 and read over 3 s with hysteresis). The frame carries, besides
  the poses and torques, whether a body is on the floor, the sides' phase, the vertical
  acceleration and the largest pin load, which the HUD shows ("loads"). The server
  rebuilds a model whose sources changed while it was building rather than caching the
  stale one, keeps a client's command or reset sent during the build, compiles the
  model off the event loop, steps at most three frames of physics per tick (slow motion
  under load, not bursts) and caps the live sessions at ten (`{"error": "busy"}`).
- **Tune** has a slider per parameter and redraws a stick figure from `/api/walk` at once.
  It rebuilds the parts only on request, with a full bake that takes tens of seconds.

Layer plans for the server's bakes go through the store (`api.plan_config`, as the bake
CLI does), so a design that planned once is never searched for again; the planner's
budget is CPU time (`stack.Deadline`), so how far a search gets no longer depends on the
machine's load (a 60 s wall-clock budget made the Strider quad plan at load 3 and fail at
load 25), and a search that ran out of it (`no_plan_in_time`) is not remembered as
unbuildable by the server or the store.

There are two ways to run it. `mise run view` (section 10) starts the development servers
(Vite with hot reload, and the API under uvicorn) on ports derived from a hash of the
checkout's path, so parallel checkouts rarely collide. `spiderpig view <design>` serves
the built viewer, which ships inside the Python package, for one stored design, and needs
no Node.

### 8.5 Audit, tune and report

- `spiderpig audit` (`tools/audit.py`) re-checks each module of a linkage end to end: the
  plan on fresh samples, the contract at four crank angles, valid solids, OCCT clashes at
  two angles, the DXF packing and the BOM's catalog keys. It writes `build/audit/` and
  exits 1 on any failure. A change that alters parts should leave it passing.
- `spiderpig tune` (`tools/tune.py`) searches crank phases, and optionally parameters
  within a percentage, for a smoother walk by the quasi-static model's score (bob, pitch,
  roll, slip, tipping and a minimum stride). It is the project's only search over designs,
  and it exists only on the command line.
- `spiderpig report` (`tools/report.py`) runs every linkage, mechanisms included, through
  foot path, planning per module and walking, and writes `build/linkages.json`. A full run
  takes about 12 minutes.

Everything so far can be driven from the command line. The next section describes the
interface built for AI agents, which drives the same engine.

## 9. The agent surface and the command line

The engine was built first, and it reported failures as exceptions with numbers in their
text. In September 2026 an interface was added to make it usable as a compiler by an AI
agent: give it a document, get back structured results and structured failures, and never
hang. The project calls this interface the **agent surface**. The command line, which is
older, now runs on the same store (section 9.7).

### 9.1 The spec

A **spec** (`spiderpig/spec.py`) is a JSON document naming one design: a linkage key with
optional parameter overrides, the leg module, the materials and servo, the constructions,
fit dimensions (the `Params` of section 3), and **targets**. A target is
`{min, max, value, tol, weight, hard}` on a named metric. There are 22 metrics, among
them:

- for a walker: stride, lift, speed, bob, slip and tipping;
- for a mechanism: stroke, straightness and swing;
- for both: stack height, mass, overall size, cost, printed grams and sheets.

A **hard** target must pass; a **soft** one is scored. One table, `TARGET_FIELDS`, pins
each metric's unit, where it is measured, how trustworthy that is, and whether it is hard
by default.

`validate` reports every error at once, with the path, the allowed values and the nearest
match. A spec with an unknown field, a misspelt linkage, a bare number where a target
belongs, a target that doesn't exist, a servo under its marketing name and a wildcard gets
eight errors back, among them `linkage.key: unknown value 'klan' (did you mean 'klann'?)`
and `constructions.pin: 'any' is a wildcard: name one value (v1 compiles one design;
search is a separate step)`. `spec_schema()` emits a JSON Schema with the live
vocabularies.

### 9.2 Designs, ids and operations

`api.resolve(spec)` validates the spec, writes every default into it explicitly, builds
the `BuildConfig`, and hashes the resolved spec with the **engine version** into a
16-hex-digit **design id**. The same spec on the same engine always gets the same id, and
so the same folder in the store. The engine version (`design.engine_version`) is the
package version plus a hash of every package source but the front-ends (the server, the
MCP layer and the command lines, `design.ENGINE_EXCLUDE`), docstrings stripped, and the
planner's default settings less its time budget. The fabrication cache is keyed more
finely, by the code its computation can reach (`spiderpig/keys.py`, section 11).

Each operation is a function of the design handle that maps onto one engine pass and
returns a report; the stage operations also cache it on the handle and in the store:

| operation | runs | time for the running example |
|---|---|---|
| `check` | the program checks, the drive, the static facts, ground clearance | 0.6 s |
| `plan` | the layer planner, or the stored plan re-made and re-verified | 0.5–1 s |
| `walk` | the walking model, with feet at their planned positions | 0.16 s |
| `build` | fabrication; every part becomes a `Part` with its live solid | tens of seconds |
| `recheck` | an agent's edited solids: valid, no clashes, inside their group's claims | depends on the edits |
| `verify` | rows of evidence at level `quick`, `standard` or `full` | about 1 s, 32–37 s, about 85 s (project docs) |
| `export` | any of STEP, STL, print, DXF, BOM, glb, MJCF, plus a manifest | 34–41 s for all seven (project docs) |
| `explain`, `recommend` | the stages in prose; the failing stage's checked fixes | about as long as `plan` |
| `derive`, `compare` | apply a patch to make a child design; diff two designs | instant |

### 9.3 Failures as data

A stage that fails returns `ok: false` and a `Failure` (`spiderpig/failure.py`): a stage,
a code (`program/loop_cannot_close`, `static/link_no_layer`, `plan/no_plan`,
`drive/second_input_no_drive` and about two dozen more), the culprits, the numbers, and
the engine's checked recommendations, each with a JSON merge patch (RFC 7386) over the
spec. Operations raise exceptions only for programming errors, and `resolve` for an
invalid spec.

Here is the whole loop for the TrotBot heel at its drawing's 7 mm unit (section 5.2), as
an agent runs it:

```python
from spiderpig import api

spec = {"kind": "walker", "linkage": {"key": "trotbot_heel", "params": {"unit": 7}},
        "legs": {"module": "single", "sides": 1},
        "size": {"stack_mm": {"max": 45}}, "motion": {"ground_clearance_mm": {"min": 30}}}
d = api.resolve(spec)
r = api.check(d)        # ok=False, static/link_no_layer: "b7 ... passes crankpin J1 at
                        # 6.8 mm, under the 10.0 mm a post there needs"
fix = r.failures[0].recommendations[0]
                        # unit 7 -> 10.5, "checked: the static stage passes, and it plans
                        # in 13 layers (47.564 mm)" (2026-10-08);
                        # fix.patch == {"linkage": {"params": {"unit": 10.5}}}
d = api.derive(d, fix.patch)
api.plan(d)
print(api.verify(d, "standard").describe())
api.build(d)
part = d.parts["b7"]    # a live build123d solid: edit it, then call api.recheck(d)
api.export(d, ["step", "dxf", "bom"], "out/heel")
```

### 9.4 Verify and its evidence tiers

`verify(design, level)` returns one row per requirement. Each row has a value, its target,
a pass or fail, whether it is hard, and a **tier** saying how the value was obtained:

- **proven**: from the engine's own guarantees: the loop checks and a mechanism's
  promises, the plan re-verified on fresh samples, the contract. Most of these rest on
  samples (the loop and promise checks at 720 crank angles, the contract at two or four),
  so "proven" means "checked by the guarantee's own machinery" rather than proven for
  every angle (section 6.3).
- **measured**: read off a model or a solid: the mass of the built parts, the sheets used,
  the printed grams.
- **estimated**: from nominal inputs: speed at the servo's no-load rpm, mass before a
  build, prices.

`quick` uses only check, plan and walk, plus estimates of mass, size and a cost floor.
`standard` adds a build, the plan re-checked on 2,880 fresh samples, the contract at two
crank angles, OCCT clashes and solid validity, sheet packing and the full cost. `full`
adds two more contract angles, a second clash angle and, for walkers, a MuJoCo run. A
design is `ok` when every hard row passes; its `score` is a weighted mean over the soft
targets.

A hard cost target never passes on a walker unless the spec's `budget.allowance_usd`
covers the unpriced items, because every walker needs some screws the parts catalog has no
price for.

### 9.5 The store

The store (`spiderpig/store.py`) lives at `$SPIDERPIG_STORE`, or else `./.spiderpig` in
the current directory:

```
designs/<id>/spec.json            the spec as given
             resolved.json        every default written in; engine version; parent and patch
             check.json plan.json walk.json export.json verify.<level>.json
             build/manifest.json  and build/parts/*.step, one STEP file per distinct part
             exports/             export()'s default folder
             log.jsonl            every operation, its time, cached or not
cache/<source version>/           the linkage catalog's cards, per code version
bakes/<config key>.glb            the viewer's and `spiderpig bake`'s cache
fab/<fab key>-<env tag>/...       fabricated mechanisms (spiderpig/fabcache.py)
builds/<hash of --out>.json       what a `spiderpig build` into a folder wrote (uptodate.py)
pin_loads/<config key>.json       each design's MuJoCo pin loads, for the strength check
```

A stage file is served again only if its engine version matches, and since the engine
version hashes every engine source, any engine edit retires every stored stage. A stored **plan** is
never trusted: it is re-made from its layers and route and verified on every load, which
takes about 0.5 s in a fresh process and about 25 ms in a running server (project docs).
The cards' cache is keyed by the *source version*, a hash of all the package's code,
unlike the engine version. A stored build reloads its STEP files when the crank angle and
engine version match. Writes are atomic.

### 9.6 Processes, jobs and the MCP server

The MCP server (`spiderpig/mcp/`, built on the official MCP Python SDK 2.x, talking over
stdio) offers 20 tools: `list_linkages`, `describe` and `catalog` for the catalogs;
`resolve`, `check`, `plan`, `explain`, `recommend` (which runs `advise`), `walk`, `build`,
`verify` and `export`; `get_job` and `wait_job`; `compare`, `derive`, `get_design`,
`list_designs` and `gc`; and `view`. There is no `recheck` tool, because it needs live
solids. It also serves 33 resources (a guide whose vocabulary tables are generated from
the code, the spec's schema, a card per linkage, and pages of the parts catalog) and 3
prompts. Across this boundary a design is its id, and a part is the path of its STEP file
in the store.

Short tools run one at a time in a worker thread behind one lock, because the engine's
caches aren't thread-safe. `build`, `verify` at the standard and full levels, and `export`
become **jobs** in a pool of spawned processes (two by default; spawning is safe here,
because the server's entry point has the guard a spawned process needs): the tool waits up
to `wait_seconds` (15 by default) and otherwise returns a job id to poll. Job records live
only in the server's memory.

### 9.7 The command line

The command line predates the agent surface. `spiderpig <command>` dispatches to the
`main(argv)` of the command's file and imports nothing of the engine until a command runs:

| command | does |
|---|---|
| `build` | STEP, STL, print STLs, DXF sheets and BOM into `build/` |
| `bake` | the viewer's glb into the store |
| `explain` | each stage's verdict in prose |
| `audit` | the end-to-end re-check of section 8.5 |
| `export` | `api.export` for a stored design or build options |
| `view` | serve the viewer for a design |
| `sim` | a MuJoCo run |
| `tune` | search phases and parameters |
| `report` | compare every linkage |
| `mcp` | the MCP server |

The commands that take a design share options: `--linkage` (Strider by default), `--module`
(by default the linkage's own: its `default_module` when it names one, Strider's `double`,
else `quad`, and the robot for a walker; `single` and one side for a mechanism;
`explain` reads one side), `--phases` in degrees,
`--proportion NAME=VALUE`, `--servo`, `--pillar`, `--pin`, `--crank` (by default
`standoff`, `chicago`, `bolt`; TrotBot's heel and toe `bolt_round`), `--sheet` and
`--thickness`. Given build options, `build`, `bake`, `explain`, `audit`, `export` and
`view` resolve them into a design in the store, so they share one stored plan.

### 9.8 The decisions behind it

Seven decisions shaped the agent surface ([DECISIONS.md](agentlib/DECISIONS.md)):

| # | decision | what it means in practice |
|---|---|---|
| 1 | a narrow spec | only fields the engine can verify; unknown fields and wildcards are errors |
| 2 | physical limits hard, gait soft | by default, size, budget, stack and ground clearance must pass; stride, speed and the like are scored |
| 3 | live solids in Python, files over MCP | Python callers may edit the CAD solids and `recheck` them; MCP callers get STEP paths |
| 4 | a store per project | `./.spiderpig`, git-ignored, with stable ids |
| 5 | named leg modules only | `single`, `double`, `decker`, `quad` and a linkage's own; no custom lists of legs |
| 6 | the viewer inside the package | `spiderpig view` needs no Node |
| 7 | bound the planner first | "a compile must never hang": the 60 s deadline, then everything else |

The rest of the report steps back from the pipeline to assess the project as a whole.

## 10. How quality is kept

This section describes how the project checks itself: its tests, the checks that guarded
the performance work, outside test drives, the tooling, and what is missing.

**Tests.** `pytest` collects about 2,000 tests (2026-10-08), run in tiers
([TESTING.md](agentlib/TESTING.md) has the commands and times):

| run | how |
|---|---|
| one module's fast tier, seconds to a minute | `mise run test-<module>` (`linkage`, `planner`, `construction`, `hardware`, `strength`, `api`, `sim`, `server`; `test-viewer` for tsc and vitest) |
| the quick tier, everything not marked `slow` or `e2e` | `mise run test-quick` (`-m 'not slow and not e2e'`, xdist) |
| the full suite | `mise run remote-test` (xdist on a remote machine), or locally `uv run pytest -n 12` |
| browser tests, marked `e2e` | `-m e2e`, with Playwright's Chromium |

A command-line `-m` replaces the default `-m 'not e2e'`, so `-m 'not slow'` alone would also
run the browser tests. The slow tests bake, build and export from the command line, run
MuJoCo and the tuner, and exercise the planner's deadline. The tiers are fast because the
suite's fabrications are cached on disk (`tests/cache.py`, keyed like the store's
fabrication cache) and a few results are recorded fixtures (`tests/fixtures/`,
`mise run test-fixtures` rewrites them). Several things make the suite more than a set of
examples:

- **Session factories** in `tests/conftest.py` build each design, side and robot once per
  session; tests read them and must never change them.
- **An independent planner.** `tests/brute.py` enumerates every layering and every crank
  route for small designs, and the planner's optimum must match it for Klann and TrotBot
  `single`, in the plan's size and the two sizes below it.
- **Every linkage is parametrized.** Assembly, rigidity and the phase and mirror rules are
  checked for all 28 linkages; that the feet are the lowest points and that the `single`
  module plans, for every walker; that every mechanism plans; the contract and clash
  checks run for every Klann module and every servo.
- **Seam tests** (`tests/test_seam_*.py`): each pins one boundary between layers (the
  planner, the crank, the pivots, the chassis, the hardware, the reports, the tools) on a
  hand-made context (`tests/_ctx.py`), without fabricating anything (the `no_fabricate`
  fixture fails a test that does).
- **The identity gate** (`tests/gate/identity_gate.py`, `mise run gate`): every product
  edit is compared, part by part, against a recorded baseline of the default designs'
  parts, plans, BOM and DXFs; its snapshot also generates
  [DESIGNS.md](agentlib/DESIGNS.md) (`mise run gate -- doc`).
  The scorecard (`tests/scorecard.py`, `mise run scorecard`) gathers every ROADMAP number
  into one file to compare against a baseline.
- **The doc check** (`tests/doc_check.py`, `mise run doc-check`): every backticked name,
  path, task, flag and environment variable in the docs must exist in the code
  (`tests/doc_check_allow.txt` lists what isn't code).

The suite forces the servos offline, so it always uses the parametric servo models, never
the downloaded ones that users get by default.

**Identical-output checks.** Every performance change of 2026-10-01 was merged only after
its outputs proved identical to the old code's (project docs): plans field for field on 86
designs (the 79 combinations of section 5.6 and seven variants; an earlier gate's four
failures differed only in the seconds they print), and DXF entities, STLs, BOMs and
per-solid volumes on exports (STEP files can't be compared byte for byte).

**Test drives.** In five rounds an outside agent drove the public surface towards a goal
("a four-legged walker in a box under $100", "a straight-line mechanism over MCP"),
logging every friction point, and the findings were fixed between rounds
([TESTDRIVE.md](history/TESTDRIVE.md)). Round 1 took 55 minutes with three dead ends. By
round 5 the earlier rounds' failing paths ran clean, and two problems remained open: the
viewer showing a mechanism inside a walker's controls, and the disagreement between MuJoCo
and the walking model.

**Tooling.** `uv` manages the Python environment (`uv.lock`, Python 3.12 only); `mise`
runs the tasks (`mise run test`, `lint`, `audit`, `view`, `release`); `ruff` lints, and
import-linter enforces the engine's import layers (`pyproject.toml`'s
`[tool.importlinter]`). A session-start hook in `.claude/` installs everything when the repository opens in
Claude Code on the web, where `mise` isn't available. `mise run release` builds the viewer
and then a wheel; a build hook refuses a wheel without the built viewer, or with viewer
sources or `node_modules` in it.

**Continuous integration** (`.github/workflows/ci.yml`, on pushes to master and every pull
request): ruff and the import layers, the lock, the viewer's typecheck and vitest, the
quick tier on a cached fabrication cache, and the doc check, blocking. The full suite, the
audits and the identity gate run on request (`mise run remote-test`, `remote-audit`,
`gate`), not in CI.

**Gaps.** The planner's opt-in speed-ups and the workers' failure paths have few tests of
their own.

## 11. Performance

This section shows where the time went for the running example at the 2026-10-01
snapshot, what the performance work of that day fixed, what it chose to leave, and the
caches added since.

**Not recomputing** (W3, 2026-10-07). Three caches keep unchanged work from being done
again, each keyed so that it can't go stale:

- the **fabrication cache** (`spiderpig/fabcache.py`, `<store>/fab/`): a fabricated
  mechanism on disk (one BRep compound of its distinct parts and a pickle of the rest),
  keyed by the code fabrication can reach (`keys.fab_key`), the library versions and the
  design; `spiderpig build` and `api.build` of a design in a store read it, and the test
  suite keeps its own (`tests/cache.py`);
- **incremental keys** (`spiderpig/keys.py`): the hash of exactly the definitions a
  computation reaches, found statically from the sources' syntax trees, so an edit
  elsewhere (a deck constant, the BOM, the DXF writer) leaves the plan and fabrication
  caches warm (`plan_key`, `fab_key`; `engine_digest` for the build check);
- the **build check** (`spiderpig/uptodate.py`): a `spiderpig build` into a folder that
  already holds the same build's outputs answers in well under a second, before the
  engine is imported (`--force` builds anyway).

`SPIDERPIG_OCCT_THREADS` sets OCCT's thread pool per process (`workers.ENV_OCCT`), so that
parallel workers (the identity gate's, xdist's) don't each take every core.

**Where the time went at the snapshot:**

| step | time |
|---|---|
| `import spiderpig.api` | 3–4.5 s, of which build123d 2.1 s (project docs) |
| plan, the running example | about 1 s |
| re-make a stored plan | 0.45 s in a fresh process; 25 ms in a running server (project docs) |
| plan, a large quad (TrotBot, six-bar, Strider) | the 60 s deadline, after the `single` module's plan as a hint |
| recommendations after a failed plan | up to 60 s more |
| `spiderpig build` into a fresh store | 55 s, the plan included |
| `spiderpig bake` | 28.6 s (56 % fabrication, 34 % meshing) |
| the MuJoCo model | 21.6 s, a fabrication included |
| 4.5 s of MuJoCo simulation | 4.2 s |
| `export` of all seven formats | 34–41 s (project docs) |
| `verify` quick, standard, full | about 1 s, 32–37 s, about 85 s (project docs) |
| test suite, default and browser | 28 min and 5 min |

A fabrication of the robot spends about 9 s in OCCT booleans. The planner's cost is per
node: 0.5–5 ms of pure-Python set and dictionary work (41 % in the crank route's dynamic
program, 39 % in forward checking), times a few stack sizes. (These figures are from the
project docs.) The search is narrow, trying 2–4 layers per node.

**What was fixed** (project docs). Three performance reports ([PERF.md](history/PERF.md),
[PERF_EXPORT.md](history/PERF_EXPORT.md), [PERF_PLANNER.md](history/PERF_PLANNER.md))
found mostly redundant work: the robot fabricated three times per export, a plan re-solved
by four of five command-line calls, boolean proofs repeated for mirror twins, meshes read
point by point from Python, build123d meshing every STL twice. The seven-format export
went from 185 s to about 37 s, `verify standard` from 44–64 s to 32–37 s, and re-making a
stored plan in a server from 0.3 s to 25 ms.

**Opt-in planner speed-ups** (project docs). `StackSpec` has four, all off by default:

- `workers`: each stack size searched in its own forked process. The same answer as the
  serial search, in 33–50 % less wall time for 1.05–1.9× the CPU. It needs `fork`, so not
  on Windows.
- `symmetry`: skips layerings that a re-timing of the legs maps onto ones already
  searched. It halves the nodes on the Strider of test-drive round 4, but its key check is
  sampled, so it may not yet back a claim of optimality.
- `quick_first`: a short search stops at its first plan (5–8 % faster on proven designs).
- `prove=False`: return the first plan, unproven, in about a third of the time on that
  Strider; a six-bar quad then comes out 26 layers instead of 24. Making this the default
  would change what `plan` returns, so it was left as a decision.

A fifth flag, `drop_bearing` (`StackSpec.drop_bearing`), is a design option rather than a speed-up: it lets the
router drop the crank's bottom bearing as a last resort. None of these flags can be set
from `BuildConfig`, the command line or the API: `side_problem` always builds a default
`StackSpec`, so they are reachable only from Python. No test in the suite turns them on.

**What was left as a decision** (project docs), each with its measured gain:

| option | gain | why it wasn't taken |
|---|---|---|
| import build123d lazily | `import spiderpig.api` 3.1 → 0.6 s, in every process | touches every file that builds parts; left for later |
| bake at the build's angle, t = 1 | 9–12 s per export (one fabrication) | moves the glb's first animation frame and the MJCF's rest pose |
| a binary BRep cache beside each stored STEP | about 3 s of a 7.5 s reload | a reloaded build's STL and DXF would change |
| group parts by face fingerprint | the grouping 15.7 → about 3 s | not a proof (it agreed with all 208 proofs, but all were positive) |
| realize constructions in parallel | 8.8 s of booleans → about 3 s on 4 cores (estimate) | a refactor of every construction |
| the route search in numpy or compiled code | up to half of each planner node | a different representation, not a fix |

## 12. Limitations and risks

Sections 4 to 7 each ended with what their stage can't do. This section collects those
limits in two lists: boundaries the project chose (12.1), and risks, meaning ways the
system could mislead a user or cost a maintainer, ranked (12.2).

### 12.1 Boundaries by design

These are things spiderpig doesn't do, by design; asking for one gets a clear refusal
rather than a wrong answer.

- Planar mechanisms with revolute joints only; every linkage comes from the catalog.
- One input per machine and one servo per side, continuous-rotation servos only. The
  five-bar resolves and checks, then stops at the drive.
- Four named leg modules: no six-legged robot, no custom lists of legs.
- The robot is two identical mirrored sides; a robot with different sides can't be
  expressed.
- One crank construction, the laser-cut `bolt` crank (hex or round standoff crankpins), on
  an aluminium crank sheet; one pillar (`standoff`) and one pin (`chicago`).
- No stacking of mechanisms (a stage mounted on another's output);
  [CLAUDE.md](../CLAUDE.md) sketches what it would take.
- No search or tuning over designs in the API or MCP: the agent is the optimiser.
  `recommend` proposes checked changes to one design and never searches, and `tune`
  exists only on the command line.
- No firmware and no gait controller.

### 12.2 Risks, ranked

1. **How a robot walks is unvalidated.** Three estimates of the running example's stride
   disagree by a factor of three, and MuJoCo has the TrotBot heel quad fall over while the
   walking model calls it stable (section 8.3). The repository holds no measurement of a
   built robot to settle either question, yet stride and speed are what a user optimises
   for.
2. **Planning results depend on the machine.** The planner's 60 s deadline is CPU time,
   so a loaded machine no longer changes a result, but a slower one can still leave a
   large quad thicker, unproven, or with no plan at all (section 5.6). The design id
   doesn't capture this. A search that ran out of time (`no_plan_in_time`) is not
   remembered as a failure by the server or the store.
3. **Stored results were able to go stale silently** (fixed). At the snapshot the engine
   version hashed only the linkage definitions and the planner's defaults, so a change to
   a construction left stored parts in place as current. It now hashes every engine
   source (section 9.2), and the fabrication cache is keyed by the code it reaches
   (section 11), so an engine edit retires the stored stages; layer plans are still
   re-verified on every load.
4. **Part of the geometric guarantee rests on samples.** The planner's distances are sound
   lower bounds over the whole turn. The contract, the OCCT clash checks and everything
   about the chassis are checked at one to four crank angles, and the motion checks at
   720 (section 6.3). `verify` labels the sampled contract and loop rows `proven`. The
   `symmetry` flag, if turned on, rests on a sampled check too.
5. **Strength rests on a beam model and unmeasured loads.** The joints and link plates are
   rated at the simulated pin loads (section 6.2), whose contact softness is unvalidated
   (section 8.2); the `bolt_round` clamp's friction coefficients are estimates; the frame
   and centre plates are dimensioned by the sheet roles and cut rules, not by loads.
6. **Costs are lower bounds** (section 7.5). A user comparing designs by cost compares
   partial sums.
7. **The kerf is the service's figure, not a measured one** (section 7.5): cut a test
   coupon before trusting the fits. (At the snapshot the DXF also lost up to 0.47 mm on
   long links' rounded ends; lines and arcs are now written exactly.)
8. **Failure codes come partly from parsing text** (the codes are in section 9.3). The
   drive failure, the node-budget failure and plan blockers are recognised by matching
   message text, so rewording a message can change a code. A plan that ran out of its 60 s
   is coded `plan/no_plan`, the same code as a search that ruled every size out, although
   nothing was proven; only the message and notes say which.
9. **Processes and memory are unmeasured** (sections 7.4 and 9.6). One export can run
   three engine processes at once, and the MCP server keeps two more; one bake alone
   peaked at 587 MB. Workers have no timeout or cancellation, and an exception from a
   worker comes back without its traceback. Several per-process caches never evict
   (`fabricate._DESIGNS`, `recommend._DONE`, the MuJoCo builders), which matters for a
   long-running server. In the MCP server, one slow plan blocks every other short tool for
   a minute or more (up to about three for a large quad that fails).
10. **The full suite runs on request.** CI runs the quick tier, the lint, the viewer's
    tests and the doc check; the full suite, the browser tests, the audits and the
    identity gate run only when someone runs them (section 10).
11. **The documents drift.** The doc check (section 10) now catches a backticked name the
    code lost, but not a number or a claim that changed; Appendix D lists what is still
    known to disagree.
12. **Rough edges in the tools:**
    - the viewer shows a mechanism inside the walker's drive and tune controls;
    - the development server's npm dependencies carry five advisories; none ships in the
      viewer bundle, but one lets any web page query a running development server;
    - `bake` spells `--side` where every other command spells `--side-only`;
    - some subcommands' `--help` (`build`'s, about 3 s) import the engine to list their
      choices.

## 13. Where to start

Everything you need for a first afternoon with the code.

**Set up.** `uv sync`, then build the viewer (`mise run viewer-build`, or
`cd viewer && npm install && npm run build`). Then try:

```bash
uv run spiderpig explain --linkage klann --module quad   # every stage's verdict, the layer table
uv run spiderpig build --linkage klann                   # the running example into build/
uv run spiderpig view --linkage klann --open             # the viewer for it, in your browser
mise run test-planner                                    # one module's fast tests (seconds)
uv run pytest -m 'not slow and not e2e'                  # the quick tier
```

**Read, in this order.**

1. [CLAUDE.md](../CLAUDE.md): the map, the pipeline contract and the house rules.
2. [spiderpig/linkages/klann.py](../spiderpig/linkages/klann.py): a whole linkage on one
   page.
3. [spiderpig/linkage/engine.py](../spiderpig/linkage/engine.py) and
   [assembly.py](../spiderpig/linkage/assembly.py): programs, legs and templates.
4. [spiderpig/fabricate.py](../spiderpig/fabricate.py): `side_problem`, `design_side` and
   `fabricate`, which tie the stages together.
5. [spiderpig/construction/base.py](../spiderpig/construction/base.py): the group
   contract; then `axle.py` and `construction/crank/` (`base.py`, then `bolt.py`), two
   groups in full.
6. [spiderpig/stack/search.py](../spiderpig/stack/search.py): `StackProblem.solve`, then
   `_Search`; [plan_z.py](../spiderpig/stack/plan_z.py) for the plan's z.
7. [spiderpig/construction/route.py](../spiderpig/construction/route.py): the crank
   router.
8. For the agent surface, [docs/agentlib/API.md](agentlib/API.md) with
   [spiderpig/api/](../spiderpig/api/__init__.py).

**Common changes.**

- *Add a linkage:* a file in `spiderpig/linkages/` with its parameters, program, links,
  frame, crank, and feet or output, that calls `register()`. The parametrized tests in
  `tests/test_linkage.py` pick it up and check that it assembles, stays rigid and plans.
- *Add a construction* for an existing group: implement `dims(ctx)` (validate, and return
  the radii its claims use) and `realize(group, build)` (parts inside those claims),
  register it in `spiderpig/construction/__init__.py`, and run `tests/test_contract.py`
  and the identity gate (`mise run gate`).
- *Add a kind of group:* subclass `construction.base.Group` (`claims`, `realize`, and
  `keepouts` and `interface` if it has any; `cuts = True` if it cuts what the others asked
  for), and append its factory to
  `construction.GROUP_FACTORIES` in dependency order. It must raise with a reason and
  declare its keep-outs, so failures stay explained.
- *Add a leg module:* a `linkage.Module` in `linkage.MODULES`, or in a linkage's own
  `modules`.

**House rules** (from CLAUDE.md):

- no `print` in the bake path: use the `bake_gltf` logger;
- keep the bake profiler's stage names;
- no z in joint poses;
- a group builds only inside its own claims;
- after a change to parts, `mise run audit` must still pass;
- never mutate the tests' session-scoped designs;
- one density table and one screw table.

From here, CLAUDE.md is the terse reference and [API.md](agentlib/API.md) the agent
surface's manual. The appendices below are for looking things up.

## Appendix A. Glossary

| term | meaning |
|---|---|
| axis | the code's name for coincident joints of several bodies: one physical axle |
| bob | how far the robot's body rises and falls each crank turn |
| bottom bearing | the crank's journal stub turning in a hole in the outer frame plate |
| clearance gap | a thin gap between two layers that holds screw heads and washers (section 5.7) |
| body | a rigid body: in a template, in the fabricated robot (one per part, plus a few with none), or in the MuJoCo model |
| branch | which of two circle crossings a step takes |
| `BuildConfig` | the validated record of what to build; its `key` names caches |
| cap | an axle's upper end |
| chain | a run of the crankshaft along one point between two plates; one standoff spans it, a screw into each end |
| claim | the space a group will occupy in each layer, as discs and pills around moving points |
| construction | how a group is built (`bolt` or `bolt_round` for the crank, `standoff` for a pillar, `chicago` for a pin); sets its claims' radii |
| contract | the check that every part lies inside its own group's claims |
| coupler | a link attached to neither frame nor motor; in the code, the body `coupler` is a shaft coupler at O |
| crank, crankpin | the driven bar turning about O; its moving end |
| crank rider, rider | a link on a crankpin; it sweeps across O |
| design id | 16 hex digits: a hash of the resolved spec and the engine version |
| detour | an extra point fixed to the crank, used as a post where no crankpin is clear |
| engine | the code that runs the six stages of the pipeline |
| engine version | the package version plus a hash of the linkage definitions and the planner's defaults |
| envelope | the underside profile of the robot's body; a detour's circle about O must stay above it |
| foot path | the loop a foot traces in one crank turn |
| frame | the fixed body (`torso`); its outer and inner frame plates |
| group | one functional part of a side: the drive, the crank, a pillar, a pin, the links, the frame |
| head | an axle's lower end |
| horn | the disc on a servo's output shaft; the crank's hub is screwed to it |
| hub | the top of the crank, under the servo horn |
| journal | the crankshaft where it runs on O |
| kerf | the width of material a laser burns away |
| keep-out | the thinnest shape a group always has, such as an axle's neck over its span |
| layer | a slab one sheet thick; every link lives in one |
| leg | one copy of a linkage, with an orientation and a phase |
| margin | the clearance between shapes of different owners, 1 mm by default |
| mechanism | a non-walking catalog entry with an output and promises |
| module | the set of legs one crank drives: `single`, `double`, `decker` or `quad` |
| neck | the thin part of an axle between shoulders |
| node | one partial assignment of links to layers that the search visits |
| node budget | a cap on the nodes searched |
| optimal | no thinner stack and no cheaper crank route, both searched to the end |
| orientation | +1 for a leg as drawn, −1 for a mirrored leg |
| pad | an area added to a plate's outline, such as the patch under the servo |
| phase | a leg's offset in crank angle |
| pillar | an axle anchored in the frame plates |
| pin | an axle joining links only |
| pitch (code) | the nominal layer thickness, the default sheet's; a plan's layer may be thicker (section 5.7) |
| `plan.cost` | the crank route's rank as one number: 10⁹ per added feature |
| post | the crankpin as a part, through a rider's layer |
| proof | the planner's text saying how a plan was shown optimal, or which sizes were left open |
| quasi-static | ignoring inertia: the robot is assumed at rest at every moment |
| recommendation | a fix the engine re-ran the failed stage with and saw pass |
| robot | two mirrored sides joined by a chassis |
| router | a group whose shape the planner picks for each layering; only the crank |
| run | a stretch of the crankshaft along one post, between two webs |
| seat | a shape inside another part's hole, exempt from collision checks |
| shoulder | a collar on an axle holding a link in its layer |
| side | one module with its frame plates and servo |
| `SideDesign` | a side's groups, claims and plan |
| slip | the residual foot sliding the walking model can't remove |
| spec | the agent's JSON document naming one design and its targets |
| stability margin | how far the centre of mass is inside the feet's support area |
| stack size | the number of layers in a side |
| stance, swing | the part of the foot path on the ground; the part in the air |
| static facts | what the planner concludes before searching: keep-outs, crank facts, the envelope |
| store | `./.spiderpig`: designs, their reports and parts |
| target (hard, soft) | a bound on a metric; a hard one must pass, a soft one is scored |
| template | one side's bodies and joints, with joint positions as functions of `t` |
| tier | how a `verify` row was obtained: proven, measured or estimated |
| `top` (code) | the index of the inner plate's layer; a stack has `top + 1` layers |
| transmission angle, toggle | the angle between the bars at a joint; near 0° or 180° the joint locks |
| web | an arm of the crankshaft from O to a crankpin |

## Appendix B. File index

Paths are under `spiderpig/` except `viewer/src/` and `tests/`. Line counts are at
`8bfc039` plus the W7 pass (2026-10-08). W5 split the old stack.py, construction/crank.py
and api.py into the packages below, each a pure move that keeps every name's old import
path; `pyproject.toml`'s `[tool.importlinter]` enforces the engine's import layers.

| file | lines | role | tests |
|---|---|---|---|
| `linkage/engine.py` | 508 | program helpers, `Linkage`, registry, `LegSolution`, `scale_params` | `test_linkage.py` |
| `linkage/checks.py` | 276 | `check_steps`, `check_output` | `test_stage_checks.py`, `test_mechanisms.py` |
| `linkage/assembly.py` | 272 | leg templates, module composition (`combine_connectors`, `fuse_*`), `feet_of` | `test_multi_linkage.py` |
| `linkages/*.py` | 1,213 | the 28 linkages | `test_linkage.py`, `test_linkages_klann.py`, `test_linkages_diywalkers.py`, `test_mechanisms.py` |
| `mechanism.py` | 425 | `Pose`, `Body`, `Mechanism`, `MechanismTemplate` | `test_transforms.py`, and most others indirectly |
| `config.py` | 634 | `BuildConfig`, the default and removed constructions, shared CLI options, query parsing | `test_removed_constructions.py`, indirect |
| `stack/` | 2,725 | the planner: `geometry.py` (shapes, claims, `Layout`), `topology.py` (`Topology`, static clearances, `PlanError`, the `Router` protocol), `plan.py` (`StackSpec`, `StackPlan`, `Deadline`), `search.py` (`StackProblem`, `_Search`), `plan_z.py` (`finalize`, the plan's z), `verify.py` (`verify_plan`) | `test_stack.py`, `test_planner_bounds.py`, `test_route.py` with `brute.py`, `test_seam_stack.py` |
| `stack_pool.py`, `stack_symmetry.py` | 342, 171 | the opt-in parallel and symmetry search (`spiderpig/stack_symmetry.py`: one of each mirrored leg pair) | few |
| `construction/base.py` | 292 | the `Group` contract, `Params`, `Context`, `Build` | indirect |
| `construction/__init__.py` | 88 | construction registries, `GROUP_FACTORIES` | `test_pivots.py` |
| `construction/axle.py` | 254 | pillars and pins: the claims, `AxleDims` | `test_axle.py`, `test_stack.py` |
| `construction/pivots/` | 1,227 | `standoff.py`, `chicago.py`, `common.py` | `test_pivots.py`, `test_standoff.py`, `test_seam_pivots.py` |
| `construction/crank/` | 1,918 | `base.py` (routes, claims, `CrankGroup`), `bolt.py` (`BoltCrank`), `hex.py` and `web.py` (its crankpin fits), `capacity.py` (its ratings, `hex_bearing_nm`), `plates.py` (`_WebPlates`, its parts) | `test_crank.py`, `test_bolt_crank.py`, `test_seam_crank.py` |
| `construction/route.py` | 863 | crank facts, joint rules, the crank router | `test_route.py` |
| `construction/underside.py`, `envelope.py` | 170, 69 | the body's underside; solids of claims | `test_route.py`, the contract |
| `construction/plates.py` | 287 | links, frame plates, rider bosses, feet's socks | `test_contract.py`, `test_joinery.py` |
| `construction/contract.py` | 157 | `check_side`, `clashes`, `bad_solids` | `test_contract.py` |
| `construction/robot.py`, `chassis.py`, `deck.py` | 305, 821, 857 | two sides, frame ties, `ASSEMBLY`; centre plates; the electronics deck | `test_robot.py`, `test_fabricate.py`, `test_deck.py`, `test_seam_chassis.py` |
| `construction/wobble.py` | 405 | link tilt and the columns' beams at the plan's z | `test_wobble.py` |
| `fabricate.py` | 308 | `side_problem`, `design_side`, `fabricate` | `test_fabricate.py`, and most via `conftest.py` |
| `fabcache.py`, `keys.py`, `uptodate.py` | 424, 1,145, 368 | the fabrication cache, incremental cache keys, the build check | `test_fabcache.py`, `test_keys.py`, `test_uptodate.py` |
| `rounding.py` | 30 | tie-stable rounding of measured numbers for reports | `test_rounding.py` |
| `recommend.py` | 434 | checked recommendations | `test_recommend.py`, `test_stage_checks.py` |
| `explain.py` | 180 | each stage's verdict in prose | `test_recommend.py` |
| `materials.py`, `manufacture.py`, `strength.py` | 285, 318, 485 | sheets and their roles; the cut rules; joint and link strength | `test_joinery.py`, `test_cutfiles.py`, `test_strength.py` |
| `servos/` | 1,784 | servo data (`servos/catalog.py`), CAD download, models, the drive group | `test_servos.py` |
| `hardware/` | 3,153 | parts catalog (`hardware/fastener_catalog.py`, `crank_catalog.py`, `sheet_catalog.py`, `electronics.py`), sources, fasteners, shims, mass, BOM (`bom.csv/md/json`), `ORDER.md` | `test_bom.py`, `test_order.py`, `test_robot.py`, `test_seam_hardware.py` |
| `shapes.py` | 189 | build123d primitives | indirect |
| `mesh.py` | 229 | meshing and STL | `test_spiderpig_api.py` |
| `layout.py` | 661 | DXF sheets and per-part DXFs | `test_export.py`, `test_cutfiles.py` |
| `build.py` | 358 | `spiderpig build` | `test_export.py`, `test_build_profile.py` |
| `bake.py` | 930 | the glb and its profiler | `test_bake_gltf.py` |
| `sim/` | 2,526 | MJCF, simulation, metrics, pin loads, the live session | `test_sim.py`, `test_sim_live.py` |
| `workers.py` | 151 | `python -c` worker processes, OCCT's thread pool | indirect |
| `walk.py` | 1,047 | the quasi-static walking model | `test_walk.py`, `test_viewer_parity.py` |
| `server/` | 1,125 | FastAPI app, live reload, `/ws/sim` | `test_walk.py`, `test_view.py`, `test_server_hardening.py`, browser tests |
| `spec.py` | 920 | Spec v1, targets, validation, schema | `test_spiderpig_api.py` |
| `api/` | 2,587 | the operations: `reports.py`, `store_ops.py`, `cards.py`, `planning.py`, `walking.py`, `building.py`, `exports.py` | `test_spiderpig_api.py`, `test_seam_reports.py` |
| `design.py` | 386 | the handle, ids, the engine version, `Part` | `test_spiderpig_api.py` |
| `failure.py` | 274 | failures and patches | `test_spiderpig_api.py` |
| `verify.py` | 788 | verify levels and rows | `test_spiderpig_api.py` |
| `store.py` | 732 | the store | `test_spiderpig_store.py`, `test_store_concurrency.py` |
| `mcp/` | 1,555 | the MCP server, jobs, output schemas | `test_spiderpig_mcp.py` |
| `cli.py`, `view.py` | 80, 335 | the console script (`mise run <command> -- <options>` passes options through); `spiderpig/view.py`: `spiderpig view <design> [--store] [--port] [--open]`, or `spiderpig view --linkage ... --pin bolt` (the build options, resolved into the store) | `test_view.py` |
| `tools/` | 2,224 | `spiderpig/tools/*.py`: audit, tune (`spiderpig/tools/tune.py`), sim (`spiderpig/tools/sim_walk.py`), export, report, the build profiler (`spiderpig/tools/build_profile.py`), development servers | `test_view.py`, `test_walk.py`, `test_sim.py`, `test_seam_tools.py` |
| `viewer/src/` | 3,018 | the three.js viewer; the drive model in `drive/` | `tests/e2e/`, vitest |
| `tests/_ctx.py` | 118 | hand-made planner and construction contexts for the seam tests | `tests/test_seam_*.py` |
| `tests/cache.py`, `tests/tiers.py` | 624, 88 | the suite's fabrication cache; a heavy check's cheap quick-tier case | the tiers |
| `tests/gate/identity_gate.py` | 792 | the identity gate; generates `docs/agentlib/DESIGNS.md` | `mise run gate` |
| `tests/doc_check.py`, `tests/scorecard.py` | 672, 584 | the doc check; the scorecard | `test_doc_check.py`, `test_scorecard.py` |

## Appendix C. Documents, history and method

**The other documents.**

| document | what it is | status |
|---|---|---|
| [CLAUDE.md](../CLAUDE.md) | the terse map for agents: tasks, profiler, repository map, pipeline contract, planner, house rules | current |
| [README.md](../README.md) | install, quick start, outputs, how the robot is built | current |
| [AGENTS.md](../AGENTS.md) | how to run things and tests, for agents | current |
| [viewer/README.md](../viewer/README.md) | how the viewer is fed and drawn | current, with the drift in Appendix D |
| [agentlib/API.md](agentlib/API.md) | the agent surface's reference | current, with the drift in Appendix D |
| [agentlib/DESIGNS.md](agentlib/DESIGNS.md) | the default designs' layers, heights, parts, audits and costs: the one place those numbers live | generated (`mise run gate -- doc`), with each gate baseline |
| [agentlib/ROADMAP.md](agentlib/ROADMAP.md) | the workstreams and their measures; its "Later" section holds the open items | current |
| [agentlib/TESTING.md](agentlib/TESTING.md) | the test tiers, the fabrication cache, the fixtures, the identity gate | current |
| [agentlib/DECISIONS.md](agentlib/DECISIONS.md) | the agent surface's seven decisions, and the hardware decisions with their dates and numbers | current |
| [agentlib/W8-gate-diffs.md](agentlib/W8-gate-diffs.md) | what each of W8's approved output changes did to the gate's designs | record (2026-10-07) |
| [agentlib/SCOPE.md](agentlib/SCOPE.md) | the agent surface's proposal | historical (2026-09-30) |
| [history/TESTDRIVE.md](history/TESTDRIVE.md) | the five test-drive rounds | historical |
| [history/TIMING.md](history/TIMING.md), [PERF.md](history/PERF.md), [PERF_EXPORT.md](history/PERF_EXPORT.md), [PERF_PLANNER.md](history/PERF_PLANNER.md) | the timing study and the three performance reports | historical (2026-10-01) |
| [history/AUDIT.md](history/AUDIT.md) | the 2026-09-29 audit of the Klann-only code | historical |
| [audit/STRENGTH.md](audit/STRENGTH.md) | joint strength at each design's own loads: the model, every walker x module | current |

**History.**

- *2015–2016.* The project began as UC Berkeley CS194 coursework: a Klann linkage
  generator on SolidPython and OpenSCAD (26 commits to January 2016).
- *April 2026.* It was ported to build123d, three.js, Vite and mise, gaining the viewer,
  the baked glb and multi-leg modules (45 commits).
- *2026-09-29.* An audit found the tests passing while no design could physically be
  built: two links collided for 13.6 % of every turn, and a foot link silently dropped out
  of the DXF. The same day the core was rebuilt: the claims-based layer planner, the
  construction groups, the generic linkage engine and its catalog, the servos and printed
  crank, the walking model, MuJoCo and the metal pivots (58 commits).
- *2026-09-30.* The code was cleaned up and the planner bounded by its 60 s deadline; the
  agent surface was built in four steps (spec, store, MCP, packaging) and refined by five
  test drives.
- *2026-10-01.* A timing study and three performance passes removed most of the redundant
  work.
- *2026-10-03 to 2026-10-05.* The hardware for a first build: Chicago screw pins, standoff
  pillars, the laser-cut aluminium crank on hex standoffs, clearance gaps and per-part
  sheets, the cut rules, the strength check, the glue-free chassis, the deck, the ordering
  outputs ([DECISIONS.md](agentlib/DECISIONS.md) has each decision).
- *2026-10-07 to 2026-10-08.* The cleanup workstreams (W0 to W8,
  [ROADMAP.md](agentlib/ROADMAP.md)): CI, the scorecard and the doc check; the legacy
  constructions removed; server and store fixes; the caches; the test tiers and seam
  tests; the package splits; the approved output changes and a new gate baseline; this
  documentation pass.

**How this report was made.** Four reviewers each read one area of the code end to end and
ran it: the domain and the symbolic engine; the layer planner and the constructions;
parts, outputs, the viewer and the simulator; and the agent surface, tooling, tests and
history. Their numbers were measured at `97bec2d` on a shared four-core Linux machine,
often with other work running, so timings are indicative; section 5.6 shows where load
changed a result. The draft was then reviewed three times: by a newcomer reading it cold,
against the code for accuracy, and for its prose. The hardware sections were brought up to
date on 2026-10-05, and the whole report checked against `8bfc039` on 2026-10-08.

## Appendix D. Where the documents and the code disagree

First checked against the code at `97bec2d`; the documentation passes of 2026-10-05 and
2026-10-08 (W7) fixed the README, AGENTS.md, CLAUDE.md and this report's rows (the ports,
the removed future_work.md, the planner's budget, the meshing calls, the test-marker
wording, the AUDIT.md pointer). The doc check (section 10) now fails CI on a backticked
name the code lost. What remains:

### docs/agentlib and the MCP guide

| document says | the code |
|---|---|
| offline servo CAD marks a design `estimated` (DECISIONS.md #7) | not implemented; the fallback only logs a warning |
| the failure-code table (API.md, and the `failure.py` docstring) | lacks `program/point_undefined`, `plan/plan_verification_failed`, `store/corrupt_record`, `job/cancelled`, `job/worker_died` |
| read-only hints on every MCP tool but `export` and `gc` (API.md) | `view` is marked as changing state too |
| `advise`, `spec_of` and `plan_config` are operations (API.md) | none is in `api.__all__` |
| a worked example with ids, costs and body counts; optional extras `[viewer]`, `[sim]`, `[mcp]` (SCOPE.md, a historical proposal) | the example's numbers are invented illustrations; those dependencies are mandatory |

### Docstrings and comments

These are left to W6b, the workstream that owns code comments.

| where | says | the code |
|---|---|---|
| `stack/search.py`, `StackProblem.solve` | proves the thinner sizes "with the full effort" | half of it, `max_nodes // 2` (`stack/search.py:299-301`) |
| `construction/route.py` | its docstring names a j_last rule | `JointRules` has no such field |
| `sim/mjcf.py` | refers to fabricate.BuildConfig and viewer.bake_gltf | `config.BuildConfig`, `spiderpig.bake` |
| `cli.py` | `--help` is instant | `spiderpig --help` and `spiderpig view --help` are; `build --help` imports the engine (about 3 s) |
| `servos/cad.py` | `python -m servos.cad` | `python -m spiderpig.servos.cad` |
| `mesh.py`, `sim/mjcf.py` | tessellation tolerance in mm | a relative deflection |
| `hardware/bom.py` | `MadeRow.size_mm` is a laser part's footprint | the world bounding box at the build angle |
| `servos/mount.py` | "one servo per machine" | one per side |
| `server/watcher.py` | watches the repository root | watches the root it is given |
| `mechanism.py` | `base_pose` lets legs sit at their own z_base | nothing sets it; z is the planner's |
| `linkage/assembly.py` | cites the 2016 code's main script | that file isn't in the repository |
| `linkages/mechanisms.py` | numbers checked by "the research (its verify.py)" | not in the repository; the numbers survive in `tests/test_mechanisms.py` |
| `linkages/trotbot.py` | the heel version compiles in about 11 s instead of 4 minutes | every program compiles in about 0.1 s |
| `viewer/src/drive/model.ts` | cites SPEC.md | no such file has ever existed; `walk.py`'s docstring is the specification |
| `viewer/README.md` | the modes table | lists `side` twice |

### Tests

| test | says | the code |
|---|---|---|
| `test_linkages_diywalkers.py` | linkages that can't be laid out are defined but not registered | all are registered, and all plan |
| `test_walk.py` | Klann's foot heights come "from its table" | every linkage's default design is planned |

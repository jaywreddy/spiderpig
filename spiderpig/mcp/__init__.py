"""spiderpig over MCP (harness v1, step 3): the agent-facing API as tools over files and
numbers.

    spiderpig mcp --store .spiderpig      # stdio; or: python -m spiderpig.mcp

Every tool maps one-to-one onto :mod:`spiderpig.api`. Across this boundary a design is
its id, a report is JSON, a part is the path of its STEP file inside the store
(decision 3: solids stay in the Python API), and a failure is the
:class:`spiderpig.failure.Failure` document, ``{stage, code, message, culprits,
numbers, blockers, recommendations, notes}``, under ``failures`` of a result whose
``ok`` is false: never an exception's text. The store (decision 4: ``--store``, else
``$SPIDERPIG_STORE``, else ``./.spiderpig``) is the state shared between calls; the
server keeps nothing else but its job pool (:mod:`spiderpig.mcp.jobs`: ``build``,
``verify`` at standard or full, and ``export`` run in worker processes and come back
as jobs when they outlast a grace period).

Tools: ``list_linkages``, ``describe``, ``catalog``, ``resolve``, ``check``, ``plan``,
``explain``, ``recommend``, ``walk``, ``build``, ``verify``, ``export``, ``compare``,
``derive``, ``get_design``, ``list_designs``, ``gc``, ``get_job``, ``wait_job``, and
``view`` (the viewer's URL for a design: a ``spiderpig view --serve-only`` child
process over the store, started once and reused, :mod:`spiderpig.view`).
Resources: ``spiderpig://guide`` (how to design with spiderpig), ``spiderpig://schema/spec``,
``spiderpig://linkages/{key}``, ``spiderpig://catalog/{servos|sheets|constructions}``,
``spiderpig://designs/{design}/{stage}``. Prompts: ``design_walker``, ``diagnose``,
``iterate``. ``tune`` and ``search`` are not in v1 (the guide says so).

Engine calls run in a worker thread, one at a time (the engine's caches are per
process and not thread-safe), so the loop keeps answering; the outputs are the
TypedDicts of :mod:`spiderpig.mcp.outputs`, published as each tool's output schema.
"""

from __future__ import annotations

import argparse
import functools
import json
import logging
import threading
from dataclasses import dataclass, fields
from functools import partial
from pathlib import Path
from typing import Annotated, Any, Literal

import anyio
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ResourceNotFoundError
from mcp.server.mcpserver.resources import FunctionResource
from mcp.types import CallToolResult, TextContent, ToolAnnotations
from pydantic import Field

from spiderpig import api, construction, linkage, servos
from spiderpig import walk as walk_model
from spiderpig.design import Design, engine_version, jsonable
from spiderpig.failure import Failure
from spiderpig.hardware import catalog as hw_catalog
from spiderpig.mcp import outputs as o
from spiderpig.mcp.jobs import Jobs
from spiderpig.spec import TARGET_FIELDS, SpecErrors, nearest, sheet_keys, spec_schema
from spiderpig.store import STAGES, Store, StoreError, report_doc

log = logging.getLogger("spiderpig.mcp")

SERVER_NAME = "spiderpig"
LEVELS = Literal["quick", "standard", "full"]
FORMATS = Literal["step", "stl", "print", "dxf", "bom", "glb", "mjcf"]
DESIGN_STAGES = ("summary", "spec", "resolved", *STAGES, "log")
STAGE_TOOL = {"check": "check", "plan": "plan", "walk": "walk", "build": "build",
              "recheck": "recheck (Python API only)", "verify": "verify", "export": "export"}
CATEGORIES = ("servos", "sheets", "constructions")
GRACE_SECONDS = 15.0

INSTRUCTIONS = (
    "spiderpig compiles a Spec (a linkage, a leg module, materials, constructions and "
    "targets) into verified walking-linkage geometry. Read the resource spiderpig://guide "
    "first: the passes, the Spec vocabulary, and the two loops (resolve -> check -> plan -> "
    "verify -> export; explain -> recommend -> derive). Every tool returns {ok, failures, ...}; "
    "a failing stage is an ordinary result with ok=false and failures as data, each with "
    "checked recommendations and the spec patch that applies them (derive takes it). A design "
    "is its id (from resolve); build, verify(standard|full) and export are jobs: follow them "
    "with wait_job or get_job. Parts are STEP files inside the store, never solids. "
    "view(design) returns the URL of the viewer for a design."
)

_ENGINE = threading.Lock()      # the engine's caches are per process and not thread-safe
_ROOT = Path(__file__).resolve().parent
_GUIDE_CACHE: dict[str, str] = {}


class Misuse(Exception):
    """A tool called wrongly (an unknown design, a bad id, ``gc`` without arguments): the
    result is the failure envelope with ``isError`` set."""

    def __init__(self, failure: Failure):
        super().__init__(failure.describe())
        self.failure = failure


@dataclass
class State:
    """What the server holds between calls: the store, its job pool, and the viewer's
    server once the ``view`` tool has started it."""

    store: Store
    jobs: Jobs
    viewer: Any = None      # a spiderpig.view.ViewServer: the child serving this store

    @property
    def root(self) -> str:
        return str(self.store.root.resolve())

    def view_server(self):
        """The viewer's server over this store: a ``spiderpig view --serve-only`` child
        process on a free port (:func:`spiderpig.view.start_background`), started on
        first use and reused while it lives."""
        from spiderpig import view as view_module

        if self.viewer is None or not self.viewer.alive():
            if view_module.viewer_built() is None:
                raise Misuse(Failure("view", "viewer_not_built", (
                    "the package has no built viewer (spiderpig/viewer/dist): from a "
                    "checkout run `mise run viewer-build`; a release wheel ships it"),
                    notes=["SPIDERPIG_VIEWER_DIST=<dir> points at another build"]))
            self.viewer = view_module.start_background(self.store)
        return self.viewer

    def stop_viewer(self) -> None:
        if self.viewer is not None:
            self.viewer.stop()
            self.viewer = None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


async def _run(fn, *args, **kwargs):
    """Run an engine call in a worker thread, one at a time."""

    def call():
        with _ENGINE:
            return fn(*args, **kwargs)

    return await anyio.to_thread.run_sync(call)


def _load(state: State, design: str) -> Design:
    """The recorded design, or :class:`Misuse`."""
    try:
        return api.load(design, state.store)
    except StoreError as e:
        raise Misuse(Failure("store", "corrupt_record", str(e))) from None
    except ValueError as e:
        raise Misuse(Failure("store", "bad_design_id", str(e), notes=[
            "a design id is the 16 hex digits resolve returned"])) from None
    except KeyError:
        raise Misuse(Failure("store", "no_such_design",
                             f"no design {design!r} in {state.root}", notes=[
                                 "list_designs lists what the store holds; resolve records "
                                 "a new design and returns its id"])) from None


def _error(failure: Failure) -> CallToolResult:
    doc = {"ok": False, "failures": [failure.to_dict()]}
    return CallToolResult(content=[TextContent(type="text", text=json.dumps(doc, indent=1))],
                          structured_content=doc, is_error=True)


def _report(design: Design, rep) -> dict:
    """A stage report as the tool's output: its JSON form, rows in their ``pass`` form,
    the design's id."""
    doc = report_doc(rep)
    if hasattr(rep, "rows"):
        doc["rows"] = [r.to_dict() if hasattr(r, "to_dict") else r for r in rep.rows]
    doc["ok"] = bool(rep.ok)
    doc["design"] = design.id
    return jsonable(doc)


def _design_out(design: Design) -> dict:
    cfg = design.config
    return jsonable({
        "ok": True, "failures": [], "design": design.id, "kind": design.kind,
        "linkage": cfg.linkage, "module": cfg.module, "sides": 2 if cfg.robot else 1,
        "engine_version": design.engine_version, "resolved": design.resolved,
        "warnings": list(design.warnings), "derived_from": design.derived_from,
        "patch": design.patch, "created_at": design.created_at,
        "store": str(design.store.root.resolve()) if design.store else "",
    })


def _spec_errors_out(e: SpecErrors) -> dict:
    return {"ok": False, "failures": [Failure.from_exception(e).to_dict()],
            "errors": [x.to_dict() for x in e.errors]}


def _spec_schema_extra(schema: dict) -> None:
    """The ``spec`` argument's schema is the Spec's own JSON Schema."""
    schema.clear()
    schema.update({k: v for k, v in spec_schema().items() if not k.startswith("$")})


SpecArg = Annotated[dict[str, Any], Field(
    description="the Spec (spiderpig://schema/spec): kind, linkage, legs, materials, "
                "constructions, fit, outputs and the targets under motion, size and budget",
    json_schema_extra=_spec_schema_extra)]
PatchArg = Annotated[dict[str, Any], Field(
    description="a JSON merge patch over the design's spec (RFC 7386: objects merge key by "
                "key, null removes a key), such as a recommendation's patch")]
DesignArg = Annotated[str, Field(description="a design id (16 hex digits, from resolve)")]


def _long_out(job) -> dict:
    """A long tool's output: the finished result with the job's record, the failure, or
    the running job alone."""
    j = job.to_dict()
    state = j["state"]
    if state == "done":
        result = j.pop("result")
        return jsonable({**result, "job": j})
    if state == "failed":
        return {"ok": False, "failures": [j["error"]], "job": j}
    return {"ok": True, "failures": [], "job": j}


async def _long(state: State, op: str, design: str, args: dict, wait_seconds: float) -> dict:
    await _run(_load, state, design)         # a misuse surfaces here, not in the worker
    job = state.jobs.submit(op, design, args)
    if wait_seconds > 0:
        await anyio.to_thread.run_sync(state.jobs.wait, job, float(wait_seconds))
    return _long_out(job)


def _job(state: State, job: str):
    j = state.jobs.get(job)
    if j is None:
        raise Misuse(Failure("job", "no_such_job", f"no job {job!r} in this server", notes=[
            "jobs live as long as the server process; the report a finished job wrote is "
            "in the store (get_design)"]))
    return j


def _job_result(j) -> dict:
    """``get_job`` / ``wait_job``: the same shape as the long tool itself returns (the
    finished result flat with the job's record under ``job``; a failure's envelope; or the
    running record alone)."""
    return _long_out(j)


# ---------------------------------------------------------------------------
# Cards: catalog, stages, the guide
# ---------------------------------------------------------------------------


def _offer(item) -> dict | None:
    of = item.offer if item is not None else None
    if of is None:
        return None
    return {"vendor": of.vendor, "url": of.url, "sku": of.sku, "pack_qty": of.pack_qty,
            "price_usd": of.price_usd, "verified": of.verified, "note": of.note}


def _servo_card(key: str) -> dict:
    s = servos.get(key)
    item = hw_catalog.CATALOG.get(s.bom_key)
    info = walk_model.servo_info(key)
    return {
        "key": key, "name": s.name, "bom_key": s.bom_key, "mass_g": info["mass_g"],
        "weight_g": s.weight_g, "speed_rpm": s.speed_rpm, "rpm_max": info["rpm_max"],
        "torque_kgcm": s.torque_kgcm, "voltage": list(s.voltage) if s.voltage else None,
        "interface": s.interface, "body_mm": list(s.body), "continuous": s.continuous,
        "price_usd": item.offer.price_usd if item is not None and item.offer else None,
        "offer": _offer(item), "cad": bool(s.cads), "notes": s.notes,
    }


def _sheet_card(key: str) -> dict:
    it = hw_catalog.get(key)
    size = it.dims.get("sheet_mm")
    return {
        "key": key, "name": it.name, "thickness_mm": float(it.dims["thickness"]),
        "sheet_mm": list(size) if size else list(hw_catalog.sheet_size(key)),
        "price_usd": it.offer.price_usd if it.offer else None,
        "pack_qty": it.offer.pack_qty if it.offer else None, "offer": _offer(it),
        "adhesive": hw_catalog.adhesive(key), "notes": it.notes,
    }


def _construction_card(c, roles: list[str]) -> dict:
    hw_catalog._load()
    knobs, hardware = {}, {}
    for f in fields(c):
        v = getattr(c, f.name)
        if f.name in ("key", "label"):
            continue
        if isinstance(v, str) and v in hw_catalog.CATALOG:
            it = hw_catalog.CATALOG[v]
            hardware[f.name] = {"key": v, "name": it.name,
                                "price_usd": it.offer.price_usd if it.offer else None}
        elif isinstance(v, (int, float, str, bool)):
            knobs[f.name] = v
    return {"key": c.key, "label": getattr(c, "label", ""),
            "doc": (c.__doc__ or "").strip().split("\n")[0], "roles": roles,
            "hardware": hardware, "knobs": knobs}


def catalog_cards(category: str = "all") -> dict:
    """Servos, sheets and constructions with prices and dimensions."""
    hw_catalog._load()
    out: dict = {}
    if category in ("servos", "all"):
        out["servos"] = [_servo_card(k) for k in servos.available()]
    if category in ("sheets", "all"):
        out["sheets"] = [_sheet_card(k) for k in sheet_keys()]
    if category in ("constructions", "all"):
        out["constructions"] = {
            "axles": [_construction_card(c, ["pillar", "pin"])
                      for _, c in sorted(construction.AXLES.items())],
            "cranks": [_construction_card(c, ["crank"])
                       for _, c in sorted(construction.CRANKS.items())],
        }
    return jsonable(out)


def linkage_card(key: str) -> dict:
    try:
        return jsonable(api.describe(key))
    except KeyError as e:
        raise Misuse(Failure("spec", "unknown_linkage", str(e).strip("'\""),
                             culprits=[{"path": "linkage.key"}],
                             numbers={}, notes=[f"have: {', '.join(linkage.available())}"]
                             )) from None


def stage_doc(state: State, design: str, stage: str) -> dict:
    """One stored stage of a design as JSON (``build``: the manifest with each part's
    file path)."""
    store = state.store
    try:
        known = store.has(design)
    except ValueError as e:
        raise Misuse(Failure("store", "bad_design_id", str(e))) from None
    if not known:
        raise Misuse(Failure("store", "no_such_design", f"no design {design!r} in {state.root}"))
    if stage not in DESIGN_STAGES:
        raise Misuse(Failure("store", "no_such_stage", f"unknown stage {stage!r}",
                             notes=[f"have: {', '.join(DESIGN_STAGES)}"],
                             culprits=[{"nearest": nearest(stage, DESIGN_STAGES)}]))
    if stage == "summary":
        doc = store.summary(design)
    elif stage == "spec":
        doc = store.read_spec(design)
    elif stage == "resolved":
        doc = store.read_design(design)
    elif stage == "log":
        doc = {"entries": store.read_log(design)}
    else:
        doc = store.read_report(design, stage)
        if doc is not None and stage == "build":
            build_dir = store.dir(design) / "build"
            doc["dir"] = str(build_dir)
            doc["parts"] = [{**e, "path": str(build_dir / e["file"]) if e.get("file") else None}
                            for e in doc.get("parts", [])]
    if doc is None:
        raise Misuse(Failure("store", "no_such_stage",
                             f"design {design} holds no {stage} yet",
                             notes=[f"run the {STAGE_TOOL.get(stage, stage)} tool first"]))
    return jsonable(doc)


def _targets_table() -> str:
    lines = ["| metric | kind | unit | hard | measured by | tier | meaning |",
             "|---|---|---|---|---|---|---|"]
    for section, table in TARGET_FIELDS.items():
        for name, f in table.items():
            lines.append(f"| `{section}.{name}` | {'/'.join(f.kinds)} | {f.unit} | "
                         f"{'**hard**' if f.hard else 'soft'} | {f.source} | {f.tier} | "
                         f"{f.doc} |")
    return "\n".join(lines)


def _linkages_table() -> str:
    lines = ["| key | kind | name | modules (legs per side) | scale params | output |",
             "|---|---|---|---|---|---|"]
    for key in linkage.available():
        lk = linkage.get(key)
        mods = ", ".join(f"{m} ({len(legs)})" for m, legs in lk.leg_modules.items())
        scale = ", ".join(linkage.scale_params(lk)) or "-"
        out = lk.output.motion if lk.output else "feet"
        lines.append(f"| `{key}` | {lk.kind} | {lk.name} | {mods} | {scale} | {out} |")
    return "\n".join(lines)


def _fit_defaults() -> str:
    from spiderpig.construction.base import Params
    from spiderpig.layout import DEFAULT_KERF

    p = Params()
    items = [f"`{f.name}` {getattr(p, f.name):g}" for f in fields(Params)]
    items.append(f"`kerf_mm` {DEFAULT_KERF:g}")
    return ", ".join(items)


def _materials_table() -> str:
    cards = catalog_cards("all")
    lines = ["| kind | key | what | price (USD) | numbers |", "|---|---|---|---|---|"]
    for s in cards["servos"]:
        lines.append(f"| servo | `{s['key']}` | {s['name']} | {s['price_usd'] or '?'} | "
                     f"{s['rpm_max']:g} rpm no load, {s['torque_kgcm'] or '?'} kg.cm, "
                     f"{s['mass_g']:g} g |")
    for s in cards["sheets"]:
        lines.append(f"| sheet | `{s['key']}` | {s['name']} | {s['price_usd'] or '?'} per "
                     f"{s['pack_qty'] or '?'} | {s['thickness_mm']:g} mm, usable "
                     f"{s['sheet_mm'][0]:g} x {s['sheet_mm'][1]:g} mm |")
    for c in cards["constructions"]["axles"]:
        hw = ", ".join(h["key"] for h in c["hardware"].values()) or "printed"
        lines.append(f"| pillar / pin | `{c['key']}` | {c['label']} | - | {hw} |")
    for c in cards["constructions"]["cranks"]:
        lines.append(f"| crank | `{c['key']}` | {c['label']} | - | - |")
    return "\n".join(lines)


def render_guide(state: State | None = None) -> str:
    """The guide (``guide.md``) with the live vocabularies written in."""
    if "text" not in _GUIDE_CACHE:
        text = (_ROOT / "guide.md").read_text()
        _GUIDE_CACHE["text"] = (text.replace("<<TARGETS>>", _targets_table())
                                .replace("<<LINKAGES>>", _linkages_table())
                                .replace("<<FIT>>", _fit_defaults())
                                .replace("<<MATERIALS>>", _materials_table())
                                .replace("<<ENGINE>>", engine_version()))
    root = state.root if state is not None else "(none)"
    return _GUIDE_CACHE["text"].replace("<<STORE>>", root)


# ---------------------------------------------------------------------------
# The server
# ---------------------------------------------------------------------------


def make_server(store: Store | str | Path | None = None, workers: int = 2,
                log_level: str = "WARNING") -> MCPServer:
    """The MCP server over ``store`` (the project store by default: ``$SPIDERPIG_STORE``,
    else ``./.spiderpig``) with ``workers`` processes for the long operations. The
    :class:`State` hangs on the server as ``spiderpig``."""
    st = Store.default() if store is None else Store.of(store)
    state = State(st, Jobs(str(st.root.resolve()), workers))
    server = MCPServer(SERVER_NAME, title="spiderpig", version=engine_version(),
                       description="a compiler from a Spec to verified walking-linkage geometry",
                       instructions=INSTRUCTIONS, log_level=log_level)
    server.spiderpig = state
    _register_tools(server, state)
    _register_resources(server, state)
    _register_prompts(server)
    return server


def _register_tools(server: MCPServer, state: State) -> None:
    def tool(fn=None, *, mutates: bool = False, destructive: bool = False):
        """Register ``fn`` with failures as data: a :class:`Misuse` or a programming
        error becomes the failure envelope, never a raw exception."""
        if fn is None:
            return partial(tool, mutates=mutates, destructive=destructive)
        ann = ToolAnnotations(read_only_hint=not mutates, destructive_hint=destructive,
                              idempotent_hint=True if not mutates else None,
                              open_world_hint=False)

        @functools.wraps(fn)
        async def wrapper(**kwargs):
            try:
                return await fn(**kwargs)
            except Misuse as e:
                return _error(e.failure)
            except SpecErrors as e:
                return _spec_errors_out(e)
            except Exception as e:  # noqa: BLE001 - a programming error still crosses as data
                log.exception("%s failed", fn.__name__)
                return _error(Failure.from_exception(e, stage="engine"))

        server.add_tool(wrapper, name=fn.__name__, annotations=ann)
        return fn

    @tool
    async def list_linkages(kind: Literal["walker", "mechanism"] | None = None
                            ) -> o.LinkagesOut:
        """Every registered linkage (``kind`` keeps walkers or mechanisms): key, name,
        family, kind, its modules with legs per side, its parameters and their defaults,
        feet per leg or the output's motion. ``describe`` gives one linkage's full card."""
        return {"ok": True, "failures": [],
                "linkages": jsonable(await _run(api.list_linkages, kind))}

    @tool
    async def describe(key: Annotated[str, Field(description="a linkage key")]) -> o.DescribeOut:
        """One linkage's card: parameters (default, angle or length, which only scale it),
        links and labels, feet or output, modules with their default phases and, for a
        walker, each module's ``stride_mm`` and whether it ``walks`` in the walk model, the
        loop closures at the defaults (margins, transmission angles, toggles), one foot's
        path numbers and the ``sensitivity`` of the foot path (lift, stride, height, width)
        to +10 % of each parameter (a walker), or the output check (a mechanism)."""
        return {"ok": True, "failures": [], "card": await _run(linkage_card, key)}

    @tool
    async def catalog(category: Literal["servos", "sheets", "constructions", "all"] = "all"
                      ) -> o.CatalogOut:
        """The buildable vocabulary with prices and dimensions: servos (continuous
        rotation; rpm, torque, mass, price), sheet stock (thickness, usable size, price
        per pack) and constructions (axles for pillars and pins, cranks: label, hardware,
        knobs). The keys are what ``materials`` and ``constructions`` of a Spec take."""
        return {"ok": True, "failures": [], **(await _run(catalog_cards, category))}

    @tool
    async def resolve(spec: SpecArg) -> o.DesignOut:
        """Validate a Spec, infer what it leaves out and record the design in the store:
        its id (what every other tool takes), the resolved spec with every inferred value,
        warnings. A spec that doesn't validate returns ``ok: false`` with ``errors``: each
        path, message, allowed values and the nearest key."""
        return _design_out(await _run(api.resolve, spec, state.store))

    @tool
    async def check(design: DesignArg) -> o.CheckOut:
        """The stages before any layer: every loop closes (else ``program``), a mechanism's
        output keeps its promises (``output``), the drive (one servo), the static facts and
        the crank's route points (else ``static``, with checked recommendations), the
        ground clearance. No parts are built."""
        d = await _run(_load, state, design)
        return _report(d, await _run(api.check, d))

    @tool
    async def plan(design: DesignArg) -> o.PlanOut:
        """The layer plan of one side: every link's layer, the stack (layers, height), the
        crank's route along its posts, whether it is proven the thinnest (``optimal``,
        ``proof``), the ground clearance, the constructions' ``warnings`` and the plan's
        table. A failure carries the blockers (count, the two shapes, gap, need) and checked
        recommendations. A stored plan is re-made and verified rather than searched again.
        A search is bounded by a 60 s deadline (and the recommendation checks by another):
        the Klann quad plans in under a second, a big or scaled-down design can take a
        minute, and a design that fails takes the deadline plus the checks."""
        d = await _run(_load, state, design)
        return _report(d, await _run(api.plan, d))

    @tool
    async def explain(design: DesignArg) -> o.ExplainOut:
        """Every stage's verdict on the design in prose: the program checks, the static
        facts, the plan with its crank route and proof, or the failing stage's error and
        what would clear it."""
        d = await _run(_load, state, design)
        return {"ok": True, "failures": [], "design": d.id, "text": await _run(api.explain, d)}

    @tool
    async def recommend(design: DesignArg) -> o.RecommendOut:
        """The engine's checked recommendations for the stage that fails (the static
        stage's, else the planner's), each with the spec patch that applies it (hand it to
        ``derive``), and the failure's ``notes`` on what can't help or wasn't checked;
        empty when the design plans. The engine recommends only what it re-ran and saw
        pass: a scale of the linkage or thinner parts for a link-to-axle gap, the linkage's
        default scale for a plan that ran into the stack's own room."""
        d = await _run(_load, state, design)
        recs = await _run(api.recommend, d)
        failing = next((r for s in ("check", "plan") if (r := d.reports.get(s)) is not None
                        and not r.ok), None)
        stage = failing.failures[0].stage if failing and failing.failures else None
        notes = list(failing.failures[0].notes) if failing and failing.failures else []
        return {"ok": True, "failures": [], "design": d.id, "stage": stage,
                "recommendations": [r.to_dict() for r in recs], "notes": notes}

    @tool
    async def walk(design: DesignArg) -> o.WalkOut:
        """The quasi-static walk model of the robot (no parts): stride, bob, slip, tipping,
        speed at the servo's no-load rpm (estimated), the nominal mass, and each motion
        target's verdict as rows. The feet sit at their planned layers when the design
        plans. A stride near zero comes with a note saying why (the module's feet cancel
        each other: a mirrored pair at one phase) and which module of the linkage walks.
        A mechanism is skipped."""
        d = await _run(_load, state, design)

        def run():
            api.plan(d)          # feet at their planned layers; a plan failure is walk's own
            return api.walk(d)

        return _report(d, await _run(run))

    @tool
    async def build(design: DesignArg, t: Annotated[float, Field(
            description="the crank angle the parts are built at (radians)")] = 1.0,
            wait_seconds: Annotated[float, Field(
                description="how long to wait before returning the running job", ge=0)]
            = GRACE_SECONDS) -> o.BuildOut:
        """Fabricate every part at crank angle ``t`` (the robot, or one side): the manifest
        with each part's group, side, fabrication, material, mass, layers and the path of
        its STEP file in the store; the total mass, the envelope, the counts. A long
        operation: a job when it outlasts ``wait_seconds`` (``wait_job`` / ``get_job``)."""
        return await _long(state, "build", design, {"t": float(t)}, wait_seconds)

    @tool
    async def verify(design: DesignArg, level: LEVELS = "quick",
                     wait_seconds: Annotated[float, Field(
                         description="standard/full: how long to wait before returning the "
                                     "running job", ge=0)] = GRACE_SECONDS) -> o.VerifyOut:
        """Pass/fail per requirement with an evidence tier (proven, measured, estimated):
        ``quick`` = check + plan + walk (~1 s, inline); ``standard`` adds the build, the
        contract at two crank angles, clashes and solids, the plan re-verified on fresh
        sampling, the DXF pack and the BOM (30-45 s, a job); ``full`` adds two more
        contract angles, a second clash angle and a MuJoCo run (85 s, a job). ``ok`` is
        false on any hard miss or stage failure; ``score`` is the soft targets' mean."""
        if level == "quick":
            d = await _run(_load, state, design)
            return _report(d, await _run(api.verify, d, "quick"))
        return await _long(state, "verify", design, {"level": level}, wait_seconds)

    @tool(mutates=True)
    async def export(design: DesignArg, formats: Annotated[list[FORMATS] | None, Field(
            description="the files to write (default: the spec's outputs)")] = None,
            out_dir: Annotated[str | None, Field(
                description="where to write (default: the design's exports/ in the store)")]
            = None,
            wait_seconds: Annotated[float, Field(
                description="how long to wait before returning the running job", ge=0)]
            = GRACE_SECONDS) -> o.ExportOut:
        """Write the files: ``step`` / ``stl`` (the whole machine), ``print`` (one STL per
        printed part + parts.csv), ``dxf`` (kerf-compensated sheets + parts.csv), ``bom``
        (csv, md, json), ``glb`` (the viewer's animated bake), ``mjcf`` (MuJoCo); always
        ``manifest.json``. Builds first if nothing is built. Returns the files' paths and
        the manifest. A long operation (a job when it outlasts ``wait_seconds``). An
        unknown format is refused by the input schema before the tool runs."""
        args = {"formats": list(formats) if formats else None, "out_dir": out_dir}
        return await _long(state, "export", design, args, wait_seconds)

    @tool
    async def get_job(job: Annotated[str, Field(
            description="a job id: the `job` field of the record build, verify or export "
                        "returned")]) -> o.JobResult:
        """A long operation's state, in the shape the tool itself returns: while it runs,
        ``job`` alone (``{job, op, design, args, state: queued|running, started_at, ...}``);
        once done, the tool's own result (a build's manifest, a verify's rows, an export's
        files) flat beside ``job`` (``state: done``, ``seconds``); if it failed, ``ok:
        false`` with the failure. Records live as long as the server; the store keeps the
        report (``get_design``)."""
        return _job_result(_job(state, job))

    @tool
    async def wait_job(job: Annotated[str, Field(
            description="a job id: the `job` field of the record build, verify or export "
                        "returned")],
            seconds: Annotated[float, Field(description="how long to wait", ge=0)] = 60.0
            ) -> o.JobResult:
        """Wait up to ``seconds`` for a job, then return it as ``get_job`` does: the tool's
        own result flat beside ``job`` once done, else the running record."""
        j = _job(state, job)
        await anyio.to_thread.run_sync(state.jobs.wait, j, float(seconds))
        return _job_result(j)

    @tool
    async def compare(a: DesignArg, b: DesignArg) -> o.CompareOut:
        """Two designs side by side: the merge patch from ``a``'s spec to ``b``'s (and
        between the resolved specs), whether one derives from the other, and every stage
        report both hold with each differing value (``reports.<stage>.<path>: {a, b}``)."""
        try:
            doc = await _run(api.compare, a, b, state.store)
        except KeyError as e:
            raise Misuse(Failure("store", "no_such_design", str(e).strip("'\""))) from None
        except ValueError as e:
            raise Misuse(Failure("store", "bad_design_id", str(e))) from None
        return {"ok": True, "failures": [], **jsonable(doc)}

    @tool
    async def derive(design: DesignArg, patch: PatchArg) -> o.DesignOut:
        """Resolve the design's spec with ``patch`` merged in (a recommendation's patch, or
        your own change) as a new design that records its parent and the patch; the same
        result as ``resolve``. An empty patch is the same design."""
        d = await _run(_load, state, design)
        return _design_out(await _run(api.derive, d, patch, state.store))

    @tool
    async def get_design(design: DesignArg, stage: Literal[
            "summary", "spec", "resolved", "check", "plan", "walk", "build", "recheck",
            "verify", "export", "log"] = "summary") -> o.StageOut:
        """Any stored stage of a design as JSON: ``summary`` (what it is, which stages it
        holds, the latest verify verdict), ``spec`` (as given), ``resolved`` (the record:
        id, engine version, every inferred value), a stage report (``check``, ``plan``,
        ``walk``, ``build`` = the manifest with part file paths, ``recheck``, ``verify``,
        ``export``), or the ``log`` of operations. Also ``spiderpig://designs/{id}/{stage}``."""
        return {"ok": True, "failures": [], "design": design, "stage": stage,
                "report": await _run(stage_doc, state, design, stage)}

    @tool
    async def list_designs() -> o.DesignsOut:
        """Every design in the store, oldest first: id, kind, linkage, module, sides, engine
        version, when it was recorded and last used, what it derives from, the stages it
        holds (each with ``ok``) and its latest verify verdict."""
        return {"ok": True, "failures": [], "store": state.root,
                "designs": jsonable(await _run(api.list_designs, state.store))}

    @tool(mutates=True, destructive=True)
    async def gc(keep: Annotated[list[str] | None, Field(
            description="design ids to keep: every other design is removed")] = None,
            older_than_seconds: Annotated[float | None, Field(
                description="remove designs last used more than this many seconds ago",
                ge=0)] = None) -> o.GcOut:
        """Remove designs from the store: those not in ``keep`` and/or last used before
        ``older_than_seconds`` ago (both given: only what is neither kept nor recent).
        Refuses to run without either. Returns the ids removed; nothing else in the
        store is touched."""
        if keep is None and older_than_seconds is None:
            raise Misuse(Failure("store", "gc_needs_arguments",
                                 "gc(keep=[ids]) and/or gc(older_than_seconds=age) says what "
                                 "to remove; a bare gc would empty the store"))
        removed = await _run(api.gc, keep, older_than_seconds, state.store)
        return {"ok": True, "failures": [], "removed": list(removed)}

    @tool(mutates=True)
    async def view(design: DesignArg) -> o.ViewOut:
        """The viewer for a design, as a URL (``spiderpig view <design>`` from a shell):
        the animated robot (or one side), drive mode, and the tune panel seeded with the
        design's linkage, module, phases and proportions, its servo and constructions
        behind every call. The server starts once per MCP server (a child process over
        the store, on a free port) and is reused; a design's first load bakes it
        (seconds) unless ``export`` wrote its ``glb``. ``ok: false`` with code
        ``viewer_not_built`` when the package carries no built viewer."""
        d = await _run(_load, state, design)
        srv = await anyio.to_thread.run_sync(state.view_server)
        return {"ok": True, "failures": [], "design": d.id, "url": srv.url(d.id),
                "server": srv.base, "mode": "robot" if d.config.robot else "side"}


def _register_resources(server: MCPServer, state: State) -> None:
    @server.resource("spiderpig://guide", name="guide", title="Designing with spiderpig",
                     description="how to design with spiderpig: the passes and what each "
                                 "proves, the Spec vocabulary with defaults and hard/soft, the "
                                 "two loops, the limits of v1", mime_type="text/markdown")
    async def guide() -> str:
        return await _run(render_guide, state)

    @server.resource("spiderpig://schema/spec", name="spec-schema", title="Spec JSON Schema",
                     description="the JSON Schema of a v1 Spec document",
                     mime_type="application/json")
    async def schema() -> str:
        return json.dumps(await _run(spec_schema), indent=1)

    @server.resource("spiderpig://linkages/{key}", name="linkage",
                     title="A linkage's card", mime_type="application/json",
                     description="one linkage's card (describe): parameters, links, "
                                 "closures, foot path or output, modules")
    async def linkage_resource(key: str) -> str:
        try:
            return json.dumps(await _run(linkage_card, key), indent=1)
        except Misuse as e:
            raise ResourceNotFoundError(e.failure.message) from None

    for key in linkage.available():
        lk = linkage.get(key)
        server.add_resource(FunctionResource(
            uri=f"spiderpig://linkages/{key}", name=f"linkage-{key}", title=lk.name,
            description=f"{lk.kind}: the card of {lk.name} [{key}]",
            mime_type="application/json", fn=partial(_json_card, linkage_card, key)))

    @server.resource("spiderpig://catalog/{category}", name="catalog", title="The catalog",
                     mime_type="application/json",
                     description="servos, sheets or constructions with prices and dimensions")
    async def catalog_resource(category: str) -> str:
        if category not in (*CATEGORIES, "all"):
            raise ResourceNotFoundError(f"no catalog {category!r}; have {', '.join(CATEGORIES)}")
        return json.dumps(await _run(catalog_cards, category), indent=1)

    for category in CATEGORIES:
        server.add_resource(FunctionResource(
            uri=f"spiderpig://catalog/{category}", name=f"catalog-{category}",
            title=f"The {category}", description=f"the {category} with prices and dimensions",
            mime_type="application/json", fn=partial(_json_card, catalog_cards, category)))

    @server.resource("spiderpig://designs/{design}/{stage}", name="design-stage",
                     title="A design's stage", mime_type="application/json",
                     description="a stored stage of a design: summary, spec, resolved, "
                                 "check, plan, walk, build (the manifest), recheck, verify, "
                                 "export, log")
    async def design_resource(design: str, stage: str) -> str:
        try:
            return json.dumps(await _run(stage_doc, state, design, stage), indent=1)
        except Misuse as e:
            raise ResourceNotFoundError(e.failure.message) from None


def _json_card(fn, *args) -> str:
    with _ENGINE:
        return json.dumps(fn(*args), indent=1)


def _register_prompts(server: MCPServer) -> None:
    @server.prompt(name="design_walker", title="Design a walker",
                   description="resolve -> check -> plan -> verify -> export for a goal")
    def design_walker(goal: str) -> str:
        return (
            f"Design a walking robot with spiderpig for this goal: {goal}\n\n"
            "1. Read spiderpig://guide (the passes, the Spec vocabulary, the loops).\n"
            "2. Call list_linkages(kind='walker') and describe(key) for the candidates; call "
            "catalog() for the servos, sheets and constructions and their prices.\n"
            "3. Write one Spec (spiderpig://schema/spec): kind 'walker', a linkage key, a "
            "module (legs per side: single 1, double 2 as a mirrored pair, decker 2 one way, "
            "quad 4; the card says which modules walk: a mirrored pair stands still in the "
            "walk model for most linkages), materials and constructions, and the goal's "
            "numbers as targets under motion, size and budget (objects with min/max/value; "
            "physical limits are hard by default, gait quality soft).\n"
            "4. resolve(spec) -> the design id. On errors, fix each path it names.\n"
            "5. check(design), then plan(design). On ok=false read failures[].recommendations "
            "and apply one with derive(design, patch); repeat on the new id.\n"
            "6. verify(design, 'quick'); then verify(design, 'standard') as a job "
            "(wait_job). Read every row: value, target, pass, tier, hard.\n"
            "7. Compare alternatives with compare(a, b); pick the best.\n"
            "8. export(design, formats) and report the file paths and the manifest.\n"
            "Report the design id, the plan (layers, height, optimal), the verify verdict "
            "with each target's row and tier, and what remains estimated."
        )

    @server.prompt(name="diagnose", title="Diagnose a failing design",
                   description="explain -> recommend -> derive on a design that fails a stage")
    def diagnose(design: str) -> str:
        return (
            f"Diagnose spiderpig design {design}.\n\n"
            "1. get_design(design, 'summary') to see which stages it holds and their ok.\n"
            "2. explain(design): every stage's verdict in prose; name the first stage that "
            "fails and the numbers behind it (margins, distances, needs, blockers).\n"
            "3. recommend(design): the engine's checked fixes, each with a spec patch and "
            "what re-running showed. If there are none, read failures[].notes and the "
            "blockers, and propose a change yourself (a scale parameter, a thinner fit, "
            "another construction or module).\n"
            "4. derive(design, patch) for the fix; plan(new) and verify(new, 'quick').\n"
            "5. compare(design, new) to state exactly what moved and what it cost (stack, "
            "clearance, mass, stride).\n"
            "Report the failing stage and code, the fix applied, and the new design's id "
            "and verdict."
        )

    @server.prompt(name="iterate", title="Move a metric",
                   description="a recommend/derive/compare loop on one metric of a design")
    def iterate(design: str, metric: str) -> str:
        return (
            f"Improve `{metric}` of spiderpig design {design}.\n\n"
            "1. verify(design, 'quick') and read the row of the metric: its value, target, "
            "tier and whether it is hard.\n"
            "2. describe(linkage) for the parameters that move it (scale parameters scale "
            "everything; others change the gait), and the guide for what the metric means.\n"
            "3. Change one thing at a time with derive(design, patch): a linkage parameter, "
            "the module or its phases, the servo, a fit value, or the target itself when it "
            "was unrealistic. There is no tune or search in v1: you are the optimizer.\n"
            "4. plan(new) then verify(new, 'quick'); compare(design, new) for the metric and "
            "its side effects (stack, ground clearance, bob, slip, mass).\n"
            "5. Keep the better design and repeat until the target is met or the trade-off "
            "is clear; finish with verify(best, 'standard') via wait_job.\n"
            "Report each step's patch and the metric's value, the best design's id and "
            "verdict, and what remains estimated."
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    """``spiderpig mcp [--store PATH] [--workers N]``: serve over stdio."""
    ap = argparse.ArgumentParser(prog="spiderpig mcp",
                                 description="serve the spiderpig API over MCP (stdio)")
    ap.add_argument("--store", metavar="PATH",
                    help="the design store (default: $SPIDERPIG_STORE, else ./.spiderpig)")
    ap.add_argument("--workers", type=int, default=2, metavar="N",
                    help="worker processes for build, verify and export (default 2)")
    ap.add_argument("--log-level", default="WARNING",
                    choices=("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
                    help="the server's logging (stderr)")
    args = ap.parse_args(argv)
    server = make_server(args.store, workers=args.workers, log_level=args.log_level)
    try:
        server.run("stdio")
    finally:
        server.spiderpig.stop_viewer()
        server.spiderpig.jobs.shutdown()
    return 0


__all__ = ["INSTRUCTIONS", "Misuse", "State", "catalog_cards", "linkage_card", "main",
           "make_server", "render_guide", "stage_doc"]

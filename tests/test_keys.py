"""The incremental cache keys (:mod:`spiderpig.keys`): an edit that can change a plan
changes the plan key, one that can change a part the fabrication key, and an edit outside
what a layer reaches (a deck colour, the shopping list, a docstring) keeps it.

Each case re-indexes the edited module in memory (``keys.Graph(sources=..., base=...)``)
and compares the closures' digests: no file is touched, nothing is imported.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from spiderpig import keys
from tests.tiers import quick

PKG = Path(keys.__file__).resolve().parent


@pytest.fixture(scope="module")
def base():
    g = keys.Graph()
    return g, {"plan": g.closure(keys.PLAN_ROOTS).digest(),
               "fab": g.closure(keys.FAB_ROOTS).digest()}


def _module_path(module: str) -> Path:
    rel = Path(*module.split(".")[1:])
    p = PKG / rel.with_suffix(".py")
    return p if p.is_file() else PKG / rel / "__init__.py"


def _edited(base_graph, module: str, old: str | None, new: str):
    """The graph with ``module``'s source edited (``old`` -> ``new``; ``old`` None: a new
    module whose source is ``new``)."""
    if old is None:
        return keys.Graph(sources={module: new}, base=base_graph)
    text = _module_path(module).read_text()
    assert text.count(old) >= 1, f"{module}: {old!r} not found (update the test)"
    return keys.Graph(sources={module: text.replace(old, new, 1)}, base=base_graph)


EDITS = [
    # (case, module, old, new, plan key changes, fabrication key changes)
    ("deck colour", "spiderpig.construction.deck",
     'DECK_COLOR = "#eb6834"', 'DECK_COLOR = "#eb6835"', False, True),
    ("deck strap width", "spiderpig.construction.deck",
     "STRAP_W = 10.0", "STRAP_W = 11.0", False, True),
    ("deck rail screw head (a claim under the inner plate)", "spiderpig.construction.deck",
     "RAIL_SCREW_R = 5.7 / 2 + 0.3", "RAIL_SCREW_R = 5.7 / 2 + 0.4", True, True),
    ("planner constant", "spiderpig.stack", "GIVE_UP = 200", "GIVE_UP = 201", True, True),
    ("planner docstring only", "spiderpig.stack",
     "(``gap_sink``) The first gap search gives up",
     "(``gap_sink``) The first gap search stops", False, False),
    ("a comment", "spiderpig.stack", "GIVE_UP = 200", "GIVE_UP = 200  # a comment",
     False, False),
    ("materials, read through a lazy import", "spiderpig.materials",
     '"al5052_2mm", "al5052_2p3mm", "al5052_2p5mm")',
     '"al5052_2mm", "al5052_2p3mm")', True, True),
    ("a Params default", "spiderpig.construction.base",
     "link_radius: float = 6.0", "link_radius: float = 6.5", True, True),
    ("a servo's dimensions", "spiderpig.servos.catalog",
     "body=(45.22, 24.72, 32.0)", "body=(45.22, 24.72, 32.5)", True, True),
    ("the shopping list", "spiderpig.hardware.order",
     '"""The shopping list of a build: carts per vendor, uploads per service, prints."""',
     '"""The shopping list."""\n    _ = 1', False, False),
    ("the parts' grouping (BOM)", "spiderpig.hardware.bom",
     "def group_made(bodies, method: str) -> list[MadeGroup]:",
     "def group_made(bodies, method: str, _x=1) -> list[MadeGroup]:", False, False),
    ("a new linkage file (the registry auto-imports it)", "spiderpig.linkages.zz_new",
     None, "X = 1\n", True, True),
    ("a module that patches the planner when imported", "spiderpig.zz_patch",
     None, "from spiderpig import stack\nstack.GIVE_UP = 5\n", True, True),
    ("a function elsewhere that setattr()s the planner", "spiderpig.zz_rt", None,
     "from spiderpig import stack\n\n\ndef go():\n    setattr(stack, 'GIVE_UP', 5)\n",
     True, True),
    ("a function elsewhere that writes vars() of the planner", "spiderpig.zz_vars", None,
     "import spiderpig.stack as st\n\n\ndef go():\n    vars(st)['GIVE_UP'] = 5\n",
     True, True),
    ("a method elsewhere that patches the planner", "spiderpig.zz_cls", None,
     "class P:\n    def go(self):\n        from spiderpig import stack\n"
     "        stack.GIVE_UP = 5\n", True, True),
    ("setattr() on the planner when imported", "spiderpig.zz_imp", None,
     "from spiderpig import stack\nsetattr(stack, 'GIVE_UP', 5)\n", True, True),
    ("a function elsewhere that patches a deck colour", "spiderpig.zz_deck", None,
     "from spiderpig.construction import deck\n\n\ndef go():\n"
     "    deck.DECK_COLOR = 'x'\n", False, True),
    ("a module that registers a sheet when imported", "spiderpig.zz_sheet", None,
     "from spiderpig.hardware.catalog import register\nregister(None)\n", True, True),
    ("the fabrication cache's format", "spiderpig.fabcache",
     "FORMAT = 2", "FORMAT = 3", False, True),
]


@pytest.mark.parametrize(("case", "module", "old", "new", "plan", "fab"),
                         quick(EDITS, keep=[EDITS[0], EDITS[3]]), ids=[e[0] for e in EDITS])
def test_an_edit_changes_exactly_the_keys_it_can(base, case, module, old, new, plan, fab):
    g0, digests = base
    g = _edited(g0, module, old, new)
    assert (g.closure(keys.PLAN_ROOTS).digest() != digests["plan"]) == plan, case
    assert (g.closure(keys.FAB_ROOTS).digest() != digests["fab"]) == fab, case


def test_the_plan_closure_holds_the_planner_and_not_the_outputs(base):
    g0, _ = base
    c = g0.closure(keys.PLAN_ROOTS)
    reached = {m for m, _ in c.symbols}
    for module in ("spiderpig.stack", "spiderpig.construction.crank",
                   "spiderpig.construction.route", "spiderpig.linkage.engine",
                   "spiderpig.linkages.strider", "spiderpig.materials", "spiderpig.config",
                   "spiderpig.servos.mount", "spiderpig.hardware.catalog"):
        assert module in c.modules, module
        assert module in reached, module
    for module in ("spiderpig.layout", "spiderpig.hardware.order", "spiderpig.build",
                   "spiderpig.server.app", "spiderpig.mcp", "spiderpig.tools.audit"):
        assert module not in c.modules, (module, c.path(module))
    assert ("spiderpig.construction.deck", "rail_screw_points") in c.symbols
    assert ("spiderpig.construction.deck", "DECK_COLOR") not in c.symbols
    f = g0.closure(keys.FAB_ROOTS)
    assert ("spiderpig.construction.deck", "DECK_COLOR") in f.symbols
    assert c.modules <= f.modules


def test_keys_are_cached_on_disk_by_the_sources_stats(tmp_path, monkeypatch):
    monkeypatch.setenv("SPIDERPIG_DIGEST_CACHE", str(tmp_path))
    monkeypatch.setattr(keys, "_KEYS", {})
    first = keys.plan_key()
    assert first.startswith("plan-")
    assert list(tmp_path.glob("keys-*.json"))
    monkeypatch.setattr(keys, "_KEYS", {})
    monkeypatch.setattr(keys, "closure", lambda roots: pytest.fail("not read from disk"))
    assert keys.plan_key() == first


def test_keys_import_nothing_of_the_engine():
    """``spiderpig build``'s up-to-date check computes the engine digest before importing
    the engine: the key module is stdlib only."""
    code = ("import sys; from spiderpig import keys; keys.engine_digest(); "
            "print(sorted(m for m in ('build123d', 'OCP', 'numpy', 'sympy', 'spiderpig.config')"
            " if m in sys.modules))")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         check=True, cwd=PKG.parent).stdout
    assert json.loads(out.replace("'", '"')) == []


def _generator_of_a_fixture():
    from spiderpig.construction.deck import rail_screw_points

    return rail_screw_points


def test_a_fixture_is_stale_when_its_generators_code_changes(tmp_path, monkeypatch):
    """A recorded fixture carries the key of what its generator reaches
    (``tests/cache.py``): the same code is current whatever else changed; another key is
    stale; a fixture from before the keys compares engine versions."""
    from spiderpig.design import engine_version
    from tests import cache

    monkeypatch.setenv("SPIDERPIG_DIGEST_CACHE", str(tmp_path))
    src = cache._source(_generator_of_a_fixture)
    assert src == {"file": "tests/test_keys.py", "name": "_generator_of_a_fixture"}
    key = cache._source_key(src)
    assert key == keys.callable_key(_generator_of_a_fixture)
    assert not cache.is_stale({"source": src, "source_key": key, "engine_version": "old"})
    assert cache.is_stale({"source": src, "source_key": "gen-other"})
    assert cache._source(lambda: 1)["name"] is None               # a lambda: its module
    assert not cache.is_stale({"engine_version": engine_version()})
    assert cache.is_stale({"engine_version": "0.0.0+old"})


PATCH_ALLOWED = {
    ("spiderpig/__init__.py", "__getattr__"): "caches a lazy export under its own name",
    ("spiderpig/uptodate.py", "preparse"): "sets a field of its own argparse.Namespace",
    ("spiderpig/view.py", "resolve_args"): "sets a field of a SimpleNamespace of options",
    ("spiderpig/mcp/jobs.py", "_init_worker"):
        "points the worker's sys.stdout at stderr (the MCP's stdio is the protocol)",
    ("spiderpig/tools/build_profile.py", "instrumented"):
        "the profiler wraps the build's calls in timers for the run, puts them back after; "
        "the results are the calls' own",
}
"""Every store into another object's namespace by name in the package, reviewed: none
patches an engine module in a way that changes what it computes."""


def _patch_sites(path: Path, pkg: Path | None = None) -> set[tuple[str, str, str]]:
    """``(file, top-level holder, what)`` of every place in ``path`` that could change a
    module's namespace, read conservatively: ``setattr`` / ``delattr`` /
    ``__setattr__`` / ``__delattr__`` on anything but ``self`` / ``cls``; any use of
    ``sys.modules[...]``; a store into ``vars(x)[...]``, ``x.__dict__[...]``,
    ``globals()[...]``; an attribute stored on ``import_module(...)``, or on a name bound
    to an imported module (through simple local aliases: ``_m = mod; _m.X = ...``)."""
    import ast

    pkg_root = (pkg or PKG).parent
    rel = path.relative_to(pkg_root).as_posix()
    tree = ast.parse(path.read_bytes())

    def is_module(dotted: str) -> bool:
        p = pkg_root / Path(*dotted.split("."))
        return p.with_suffix(".py").is_file() or (p / "__init__.py").is_file()

    def bound(scope) -> set[str]:
        """Names bound to a module in ``scope`` (imports, then simple aliases of them)."""
        names = set()
        for n in ast.walk(scope):
            if isinstance(n, ast.Import):
                names |= {a.asname or a.name.split(".")[0] for a in n.names}
            elif isinstance(n, ast.ImportFrom) and n.module:
                names |= {a.asname or a.name for a in n.names
                          if is_module(f"{n.module}.{a.name}")}
        grew = True
        while grew:
            grew = False
            for n in ast.walk(scope):
                if (isinstance(n, ast.Assign) and isinstance(n.value, ast.Name)
                        and n.value.id in names):
                    for t in n.targets:
                        if isinstance(t, ast.Name) and t.id not in names:
                            names.add(t.id)
                            grew = True
        return names

    top_names = bound(tree)
    out = set()
    for top in tree.body:
        holder = getattr(top, "name", "<module>")
        mods = top_names | bound(top)
        for n in ast.walk(top):
            what = None
            if isinstance(n, ast.Call):
                f = n.func
                callee = f.id if isinstance(f, ast.Name) else (
                    f.attr if isinstance(f, ast.Attribute) else "")
                if callee in ("setattr", "delattr", "__setattr__", "__delattr__"):
                    if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) \
                            and f.value.id != "object":
                        target = f.value                    # x.__setattr__(...)
                    else:
                        target = n.args[0] if n.args else None
                    if not (isinstance(target, ast.Name) and target.id in ("self", "cls")):
                        what = callee
            elif isinstance(n, ast.Subscript):
                v = n.value
                if (isinstance(v, ast.Attribute) and v.attr == "modules"
                        and isinstance(v.value, ast.Name) and v.value.id == "sys"):
                    what = "sys.modules"
                elif isinstance(n.ctx, (ast.Store, ast.Del)) and (
                        (isinstance(v, ast.Call) and isinstance(v.func, ast.Name)
                         and v.func.id in ("vars", "globals"))
                        or (isinstance(v, ast.Attribute) and v.attr == "__dict__")):
                    what = "namespace store"
            elif isinstance(n, ast.Attribute) and isinstance(n.ctx, (ast.Store, ast.Del)):
                v = n.value
                if isinstance(v, ast.Name) and v.id in mods:
                    what = f"store on module {v.id}"
                elif isinstance(v, ast.Call) and (
                        (isinstance(v.func, ast.Attribute) and v.func.attr == "import_module")
                        or (isinstance(v.func, ast.Name)
                            and v.func.id in ("import_module", "__import__"))):
                    what = "store on import_module()"
                else:
                    root = v
                    while isinstance(root, ast.Attribute):
                        root = root.value       # spiderpig.construction.plates.X = ...
                    if isinstance(root, ast.Name) and root.id in mods:
                        what = f"store on module {root.id}..."
            if what:
                out.add((rel, holder, what))
    return out


def test_nothing_in_the_package_monkeypatches_the_engine():
    """Anything in ``spiderpig/`` (outside ``tests/``) that could change a module's
    namespace (:func:`_patch_sites`, read conservatively) is a site the keys can't always
    see through (a patched module computes something else under the same key): each must
    be in :data:`PATCH_ALLOWED`, with why it is harmless."""
    found = set()
    for path in sorted(PKG.rglob("*.py")):
        found |= _patch_sites(path)
    unknown = sorted(f for f in found if f[:2] not in PATCH_ALLOWED)
    assert unknown == [], unknown


EVASIONS = [
    "import sys\nsys.modules['spiderpig.construction.plates'].NOTCH = 1\n",
    "import importlib\nimportlib.import_module('spiderpig.construction.plates').NOTCH = 1\n",
    "from spiderpig.construction import plates as _pl\n_m = _pl\n_m.NOTCH = 1\n",
    "from spiderpig.construction import plates as _pl\n_pl.__setattr__('NOTCH', 1)\n",
    "from spiderpig.construction import plates as _pl\nsetattr(_pl, 'NOTCH', 1)\n",
    "import spiderpig.construction.plates\nspiderpig.construction.plates.NOTCH = 1\n",
    "from spiderpig.construction import plates\n\n\ndef f():\n    vars(plates)['N'] = 1\n",
    "import spiderpig.stack as st\n\n\nclass P:\n    def go(self):\n"
    "        object.__setattr__(st, 'GIVE_UP', 5)\n",
]


@pytest.mark.parametrize("source", EVASIONS)
def test_the_guard_sees_the_evasive_forms(source, tmp_path):
    """Each evasive form, in a module of a stand-in package (the real one untouched)."""
    fake = tmp_path / "spiderpig"
    (fake / "construction").mkdir(parents=True)
    (fake / "construction" / "__init__.py").write_text("")
    (fake / "construction" / "plates.py").write_text("NOTCH = 0\n")
    (fake / "stack.py").write_text("GIVE_UP = 0\n")
    (fake / "zz_guard_probe.py").write_text(source)
    assert _patch_sites(fake / "zz_guard_probe.py", fake), source

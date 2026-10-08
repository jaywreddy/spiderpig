"""Incremental cache keys: a hash of exactly the code a cached result depends on.

:func:`spiderpig.design.engine_version` hashes every engine source, so any edit starts
every cache afresh. The caches of plans and fabrications are keyed here instead by the
code their computation can reach (:func:`source_key`), so an edit elsewhere (a deck
constant only the deck's parts read, the BOM, the DXF writer) leaves them warm:

- :func:`plan_key`: what :func:`spiderpig.fabricate.design_side` (template, groups,
  claims, planner, router, verification) and :class:`spiderpig.config.BuildConfig` reach;
- :func:`fab_key`: that plus :func:`spiderpig.fabricate.fabricate` (every group's
  ``realize``, the robot, the chassis, the servos, the shapes) and the fabrication
  cache's own format (:mod:`spiderpig.fabcache`);
- :func:`function_key` / :func:`callable_key`: what a function reaches (a recorded
  fixture's generator, ``tests/cache.py``);
- :func:`engine_digest`: every engine source (``spiderpig build``'s up-to-date check).

**How the closure is found** (statically, from the sources' ASTs; nothing is imported):
the reached *symbols* (a module's top-level function, class or assignment; a class's
method) from the roots, following

- every name a reached definition loads, resolved through its module's imports (and
  the imports inside the definition itself: a lazy import counts); annotations aren't
  followed (never evaluated: ``from __future__ import annotations``, and nothing reads
  type hints), but they are part of the hashed code;
- ``obj.attr`` on an object of unknown type: **every** method and class attribute named
  ``attr`` in the engine, and every module-level symbol named ``attr`` of a module that
  is handled as a value in reached code (passed, stored, imported by a computed name); on
  a name bound to a module, that module's symbol (or submodule) ``attr``; dunders aside
  (a reached class brings its own);
- keyword argument names and the string constants given to ``getattr`` / ``hasattr`` /
  ``setattr`` as attributes; a ``getattr`` by a computed name: every identifier-like
  string constant of its module;
- a reached class reaches its bases, decorators, class attributes, nested classes and
  every dunder method; its other methods are reached by name;
- a module is *loaded* when a loaded module or reached code imports it, it is a parent
  package of one, a package auto-imports it (``pkgutil.iter_modules``: the linkages, so
  a new linkage file is a new key), a computed ``importlib.import_module`` in reached
  code names it, or its import-time code calls or changes a reached symbol (registers
  into a reached registry, patches a reached module). Loading reaches its import-time
  effects: every top-level statement that isn't a definition, an import or a plain
  constant (a call into the package, a registry filled, a loop, an ``if``, a decorator
  other than the standard pure ones). A method matched by name alone in a module
  nothing loads is hashed, but loads nothing (its instances' module was imported by
  whatever made them); the module's ``if __name__ == "__main__":`` never runs.

Docstrings are stripped and line numbers aren't hashed: a docs-only edit (or moving code)
keeps every key. Hashed beside the reached definitions: the names of the loaded modules
(a new import is a new key), the package version, the Python version, the version of
every third-party distribution a loaded module imports (numpy, sympy, build123d, OCP,
...), and this module's own code (a change of the rules is a new key).

**Conservative by design**: whatever can't be told apart statically is included. What it
still cannot see, and why that is safe here: code reached only through ``eval`` /
``exec`` (none in the engine), a module object reached through ``sys.modules``, and
environment variables (none of the engine's change a plan or a part but the servo CAD
switches, which the fabrication cache keys itself: :func:`spiderpig.fabcache.servo_state`).
Data files: the engine reads none (the servo CAD downloads are pinned by hash).
``tests/test_keys.py`` checks both directions on real modules.

The answer is kept on disk (``$SPIDERPIG_DIGEST_CACHE``, like
:func:`spiderpig.design.engine_version`'s) under a signature of every source's stats and
the roots, so a process on unchanged sources reads it in milliseconds.
"""

from __future__ import annotations

import ast
import hashlib
import json
import os
import sys
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent          # the spiderpig package
PACKAGE = ROOT.name
REPO = ROOT.parent

PLAN_ROOTS = (
    "spiderpig.fabricate:design_side",
    "spiderpig.fabricate:template_for",
    "spiderpig.fabricate:remember",
    "spiderpig.config:BuildConfig",
    "spiderpig.config:default_module",
    "spiderpig.config:default_robot",
)
"""What a plan is a function of: the side's template and design (groups, claims, the
planner, the router, verification, the recommendations a failure carries) and the
config that names it."""

FAB_ROOTS = PLAN_ROOTS + (
    "spiderpig.fabricate:fabricate",
    "spiderpig.fabricate:fabricate_side",
    "spiderpig.construction.robot:FrameTies",
    "spiderpig.construction.robot:assemble_robot",
    "spiderpig.mechanism:Mechanism",
    "spiderpig.fabcache:dump_mechanism",
    "spiderpig.fabcache:load_mechanism",
    "spiderpig.fabcache:FORMAT",
)
"""What a fabrication is a function of: the plan's code, every group's ``realize`` (the
attribute reaches every one), the robot and the cache's own format."""

PURE_DECORATORS = frozenset({
    "dataclass", "property", "staticmethod", "classmethod", "lru_cache", "cache",
    "cached_property", "contextmanager", "total_ordering", "wraps", "overload", "final",
    "abstractmethod", "unique", "runtime_checkable", "setter", "getter", "deleter",
})
"""Decorators that register nothing anywhere (a definition so decorated is no import-time
effect). Anything else (a registry's ``@register``) makes the definition an effect."""

PURE_CALLS = frozenset({
    "dict", "list", "tuple", "set", "frozenset", "float", "int", "str", "bool", "range",
    "len", "min", "max", "sorted", "sum", "abs", "round", "zip", "enumerate", "map",
    "field", "compile", "getLogger", "Path", "MappingProxyType", "namedtuple", "TypeVar",
    "fromkeys", "get", "radians", "degrees", "sqrt", "array", "asarray", "deepcopy",
    "Lock", "RLock", "local", "ContextVar", "defaultdict", "OrderedDict", "Counter",
    "partial", "cos", "sin", "pi", "hypot", "Decimal", "Fraction", "tan", "atan2", "exp",
    "join", "format", "replace", "upper", "lower", "split", "strip", "items", "keys",
    "values", "environ", "getenv",
})
"""Callees (by their last name) whose call in a top-level assignment can't register
anything in the package; any other call there (into the package) makes it an effect."""


def _is_dunder(name: str) -> bool:
    return name.startswith("__") and name.endswith("__")


# ---------------------------------------------------------------------------
# one module's index
# ---------------------------------------------------------------------------


def _strip_docstrings(tree: ast.AST) -> None:
    for node in ast.walk(tree):
        body = getattr(node, "body", None)
        if (isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                and body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]


def _decorator_name(d: ast.expr) -> str:
    if isinstance(d, ast.Call):
        d = d.func
    if isinstance(d, ast.Attribute):
        return d.attr
    if isinstance(d, ast.Name):
        return d.id
    return ""


def _callee_name(c: ast.Call) -> str:
    f = c.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return ""


def _targets(node: ast.stmt) -> list[ast.expr]:
    if isinstance(node, ast.Assign):
        return list(node.targets)
    if isinstance(node, (ast.AnnAssign, ast.AugAssign)):
        return [node.target]
    return []


def _names_of(target: ast.expr) -> list[str] | None:
    """The plain names an assignment target binds (``None``: it stores into something)."""
    if isinstance(target, ast.Name):
        return [target.id]
    if isinstance(target, (ast.Tuple, ast.List)):
        out = []
        for e in target.elts:
            sub = _names_of(e.value if isinstance(e, ast.Starred) else e)
            if sub is None:
                return None
            out += sub
        return out
    return None


def _main_guard(stmt: ast.stmt) -> bool:
    """``if __name__ == "__main__":``"""
    t = stmt.test if isinstance(stmt, ast.If) else None
    return (isinstance(t, ast.Compare) and isinstance(t.left, ast.Name)
            and t.left.id == "__name__" and len(t.comparators) == 1
            and isinstance(t.comparators[0], ast.Constant)
            and t.comparators[0].value == "__main__")


def _noop(stmt: ast.stmt) -> bool:
    """A bare string (an attribute's docstring), ``pass`` or ``...``."""
    return isinstance(stmt, ast.Pass) or (isinstance(stmt, ast.Expr)
                                          and isinstance(stmt.value, ast.Constant))


def _walk(node: ast.AST):
    """``ast.walk`` without annotations (never evaluated: ``from __future__ import
    annotations``, and nothing in the package reads type hints)."""
    todo = [node]
    while todo:
        n = todo.pop()
        yield n
        for name, value in ast.iter_fields(n):
            if name in ("annotation", "returns") or (name == "type_comment"):
                continue
            if isinstance(value, ast.AST):
                todo.append(value)
            elif isinstance(value, list):
                todo.extend(v for v in value if isinstance(v, ast.AST))


@dataclass
class _Module:
    name: str
    path: Path
    is_pkg: bool
    excluded: bool
    symbols: dict[str, list[ast.AST]] = field(default_factory=dict)
    classes: dict[str, ast.ClassDef] = field(default_factory=dict)  # qualname -> node
    effects: list[ast.AST] = field(default_factory=list)
    imports: dict[str, tuple[str, str | None]] = field(default_factory=dict)
    star: list[str] = field(default_factory=list)
    top_imports: list[ast.AST] = field(default_factory=list)
    autoload: bool = False
    getattr_hook: bool = False
    strings: set[str] = field(default_factory=set)
    effect_refs: set[tuple[str, str]] = field(default_factory=set)
    # (module, name, the top-level symbol holding it): a store into another module's (or
    # its own) namespace anywhere in this module's code: ``mod.X = ...``,
    # ``setattr(mod, "X", ...)``, ``vars(mod)[...] = ...``, ``globals()[...] = ...``;
    # name "*" when it is computed
    patches: list[tuple[str, str, str]] = field(default_factory=list)

    @property
    def package(self) -> str:
        return self.name if self.is_pkg else self.name.rpartition(".")[0]


def _absolute(mod: _Module, node: ast.ImportFrom) -> str:
    if not node.level:
        return node.module or ""
    base = mod.package
    for _ in range(node.level - 1):
        base = base.rpartition(".")[0]
    return f"{base}.{node.module}" if node.module else base


def _import_aliases(mod: _Module, node: ast.AST) -> dict[str, tuple[str, str | None]]:
    """The names an ``import`` / ``from ... import`` binds: alias -> (module, attr)."""
    out: dict[str, tuple[str, str | None]] = {}
    if isinstance(node, ast.Import):
        for a in node.names:
            if a.asname:
                out[a.asname] = (a.name, None)
            else:
                top = a.name.split(".")[0]
                out[top] = (top, None)
    elif isinstance(node, ast.ImportFrom):
        src = _absolute(mod, node)
        for a in node.names:
            if a.name != "*":
                out[a.asname or a.name] = (src, a.name)
    return out


def _index(name: str, path: Path, is_pkg: bool, excluded: bool,
           source: str | bytes | None = None) -> _Module:
    tree = ast.parse(path.read_bytes() if source is None else source, filename=str(path))
    _strip_docstrings(tree)
    mod = _Module(name, path, is_pkg, excluded)
    for n in ast.walk(tree):
        if isinstance(n, ast.Constant) and isinstance(n.value, str) and len(n.value) < 200:
            mod.strings.add(n.value)
        if isinstance(n, ast.Attribute) and n.attr == "iter_modules":
            mod.autoload = True

    def classes(node: ast.ClassDef, prefix: str) -> None:
        qual = f"{prefix}{node.name}"
        mod.classes[qual] = node
        for b in node.body:
            if isinstance(b, ast.ClassDef):
                classes(b, f"{qual}.")

    for stmt in tree.body:
        if _main_guard(stmt) or _noop(stmt):
            continue                    # never runs on import / does nothing
        if isinstance(stmt, (ast.Import, ast.ImportFrom)):
            mod.top_imports.append(stmt)
            mod.imports.update(_import_aliases(mod, stmt))
            if isinstance(stmt, ast.ImportFrom) and any(a.name == "*" for a in stmt.names):
                mod.star.append(_absolute(mod, stmt))
            continue
        if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            mod.symbols.setdefault(stmt.name, []).append(stmt)
            if isinstance(stmt, ast.ClassDef):
                classes(stmt, "")
            if any(_decorator_name(d) not in PURE_DECORATORS for d in stmt.decorator_list):
                mod.effects.append(stmt)
            if stmt.name == "__getattr__" and not isinstance(stmt, ast.ClassDef):
                mod.getattr_hook = True     # reached by a name the module lacks
            continue
        targets = _targets(stmt)
        names = [_names_of(t) for t in targets]
        if targets and not isinstance(stmt, ast.AugAssign) and all(n is not None for n in names):
            for ns in names:
                for n in ns or ():
                    mod.symbols.setdefault(n, []).append(stmt)
            value = getattr(stmt, "value", None)
            calls = [c for c in ast.walk(value) if isinstance(c, ast.Call)] if value else []
            if any(_callee_name(c) not in PURE_CALLS for c in calls):
                mod.effects.append(stmt)        # a call into the package: may register
            continue
        # anything else runs at import with effects we don't model: always reached; the
        # names it binds (a try/except import, an if-defined function) are its symbols
        mod.effects.append(stmt)
        for n in ast.walk(stmt):
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                mod.symbols.setdefault(n.name, []).append(stmt)
            elif isinstance(n, ast.Name) and isinstance(n.ctx, ast.Store):
                mod.symbols.setdefault(n.id, []).append(stmt)
            elif isinstance(n, (ast.Import, ast.ImportFrom)):
                mod.imports.update(_import_aliases(mod, n))
        for n in ast.walk(stmt):
            if isinstance(n, ast.ClassDef):
                classes(n, "")
    # what the import-time code *changes or calls*: a function called (a registry's
    # ``register``), an object whose method is called or that is stored into (``REG[k] =``,
    # ``REG.update(...)``), through this module's imports; (module, name). A name only
    # read (passed to ``fields()``) changes nothing.
    def touch(expr: ast.expr) -> None:
        while isinstance(expr, (ast.Attribute, ast.Subscript, ast.Call)):
            if isinstance(expr, ast.Attribute) and isinstance(expr.value, ast.Name):
                alias = mod.imports.get(expr.value.id)
                if alias is not None:       # module.attr (``from pkg import module`` too)
                    src = alias[0] if alias[1] is None else f"{alias[0]}.{alias[1]}"
                    mod.effect_refs.add((src, expr.attr))
            expr = expr.func if isinstance(expr, ast.Call) else expr.value
        if isinstance(expr, ast.Name):
            if expr.id in mod.imports:
                src, attr = mod.imports[expr.id]
                mod.effect_refs.add((src, attr or ""))
            elif expr.id in mod.symbols:
                mod.effect_refs.add((name, expr.id))

    for stmt in mod.effects:
        for n in _walk(stmt):
            if isinstance(n, ast.Call):
                touch(n.func)
            elif isinstance(n, (ast.Attribute, ast.Subscript)) and isinstance(
                    getattr(n, "ctx", None), (ast.Store, ast.Del)):
                touch(n)        # ``mod.X = ...`` touches mod.X, ``REG[k] = ...`` REG
            elif isinstance(n, ast.AugAssign):
                touch(n.target)
    for stmt in tree.body:
        holder = (stmt.name if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef,
                                                 ast.ClassDef))
                  else None)
        for target, attr in _patch_sites(mod, stmt):
            if holder is None:
                mod.effect_refs.add((target, attr))
            mod.patches.append((target, attr, holder or ""))
    return mod


def _patch_sites(mod: _Module, stmt: ast.AST) -> list[tuple[str, str]]:
    """Where ``stmt`` stores into a module's namespace (module docstring of :class:`_Module`
    ``patches``): (the module, by the import alias it is reached through, or ``mod``'s
    own for ``globals()``; the name, or ``"*"``)."""
    local = dict(mod.imports)
    for n in ast.walk(stmt):
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            local.update(_import_aliases(mod, n))

    def module(expr: ast.expr) -> str | None:
        if isinstance(expr, ast.Name) and expr.id in local:
            src, attr = local[expr.id]
            return src if attr is None else f"{src}.{attr}"
        if isinstance(expr, ast.Attribute):
            base = module(expr.value)
            return None if base is None else f"{base}.{expr.attr}"
        return None

    def namespace(expr: ast.expr) -> str | None:
        """``vars(mod)``, ``mod.__dict__``, ``globals()``: the module whose namespace."""
        if isinstance(expr, ast.Call) and isinstance(expr.func, ast.Name):
            if expr.func.id == "globals" and not expr.args:
                return mod.name
            if expr.func.id == "vars" and expr.args:
                return module(expr.args[0])
        if isinstance(expr, ast.Attribute) and expr.attr == "__dict__":
            return module(expr.value)
        return None

    def const(expr: ast.expr | None) -> str:
        return expr.value if isinstance(expr, ast.Constant) and isinstance(
            expr.value, str) else "*"

    out = []
    for n in ast.walk(stmt):
        if isinstance(n, (ast.Attribute, ast.Subscript)) and isinstance(
                getattr(n, "ctx", None), (ast.Store, ast.Del)):
            if isinstance(n, ast.Attribute) and (m := module(n.value)) is not None:
                out.append((m, n.attr))
            elif isinstance(n, ast.Subscript) and (m := namespace(n.value)) is not None:
                out.append((m, const(n.slice)))
        elif (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
              and n.func.id in ("setattr", "delattr") and n.args):
            if (m := module(n.args[0])) is not None:
                out.append((m, const(n.args[1] if len(n.args) > 1 else None)))
        elif (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr in ("update", "setdefault", "pop", "__setitem__")
              and (m := namespace(n.func.value)) is not None):
            out.append((m, "*"))
    return out


# ---------------------------------------------------------------------------
# the package
# ---------------------------------------------------------------------------


def _engine_exclude() -> tuple[str, ...]:
    # (design.py's list, read without importing it: design imports the engine)
    tree = ast.parse((ROOT / "design.py").read_bytes())
    for stmt in tree.body:
        if (isinstance(stmt, ast.Assign) and any(isinstance(t, ast.Name)
                                                 and t.id == "ENGINE_EXCLUDE"
                                                 for t in stmt.targets)):
            return tuple(ast.literal_eval(stmt.value))
    return ()


def _module_files(extra: Iterable[Path] = ()) -> dict[str, tuple[Path, bool]]:
    """Every module of the package (and ``extra`` files outside it, by their dotted path
    from the repository root): name -> (path, is a package)."""
    out = {}
    for p in sorted(ROOT.rglob("*.py")):
        rel = p.relative_to(REPO).with_suffix("")
        parts = list(rel.parts)
        is_pkg = parts[-1] == "__init__"
        if is_pkg:
            parts = parts[:-1]
        out[".".join(parts)] = (p, is_pkg)
    for p in extra:
        p = Path(p).resolve()
        try:
            rel = p.relative_to(REPO).with_suffix("")
        except ValueError:
            rel = Path(p.stem)
        parts = list(rel.parts)
        is_pkg = parts[-1] == "__init__"
        if is_pkg:
            parts = parts[:-1]
        out.setdefault(".".join(parts), (p, is_pkg))
        # its package's __init__ beside it, if any (tests/ has none: a namespace)
    return out


class Graph:
    """The package's modules, indexed once; :meth:`closure` from roots. ``extra``: files
    outside the package to index too (a test module). ``sources``: module name ->
    source text, in place of (or beside) the files on disk (what an edit would make of
    a key, without making it: ``tests/test_keys.py``)."""

    def __init__(self, extra: Iterable[Path] = (), sources: dict[str, str] | None = None,
                 base: Graph | None = None):
        excluded = _engine_exclude()
        files = _module_files(extra)
        sources = dict(sources or {})
        for name in sources:
            if name not in files:
                rel = Path(*name.split(".")).with_suffix(".py")
                files[name] = (REPO / rel, False)
        self.modules: dict[str, _Module] = {}
        for name, (path, is_pkg) in files.items():
            parts = name.split(".")
            rel0 = parts[1] if len(parts) > 1 and parts[0] == PACKAGE else ""
            ex = parts[0] == PACKAGE and (rel0 in excluded or f"{rel0}.py" in excluded)
            if base is not None and name in base.modules and name not in sources:
                self.modules[name] = base.modules[name]     # (indexes are read-only)
            else:
                self.modules[name] = _index(name, path, is_pkg, ex, sources.get(name))
        # every method / class attribute by name (engine modules only: the excluded
        # front-ends are never reached from the engine but through an explicit import)
        self.members: dict[str, list[tuple[str, str]]] = {}
        for m in self.modules.values():
            if m.excluded:
                continue
            for qual, cls in m.classes.items():
                for b in cls.body:
                    for n in self._member_names(b):
                        self.members.setdefault(n, []).append((m.name, f"{qual}.{n}"))

    def written(self, mod: str, name: str) -> list[tuple[str, str]]:
        """What a write to ``mod``'s ``name`` changes: that binding, and where ``mod`` is a
        package re-exporting ``name`` from its own submodules, theirs too (the package
        forwards the write: :mod:`spiderpig.reexport`)."""
        out, seen = [(mod, name)], {(mod, name)}
        m = self.modules.get(mod)
        while m is not None and m.is_pkg and name in m.imports:
            src, attr = m.imports[name]
            if attr is None or not src.startswith(mod + ".") or (src, attr) in seen:
                break
            out.append((src, attr))
            seen.add((src, attr))
            m, name = self.modules.get(src), attr
        return out

    @staticmethod
    def _member_names(stmt: ast.stmt) -> list[str]:
        if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            return [stmt.name]
        out = []
        for t in _targets(stmt):
            out += _names_of(t) or []
        return out

    def _module_value(self, m: _Module, name: str, local: dict) -> str | None:
        """The module ``name`` is bound to in ``m`` (``None``: not a module)."""
        src = local.get(name) or m.imports.get(name)
        if src is None:
            if name not in m.symbols and f"{m.package}.{name}" in self.modules and m.is_pkg:
                return f"{m.package}.{name}"
            return None
        mod, attr = src
        if attr is None:
            return mod if mod in self.modules else None
        sub = f"{mod}.{attr}"
        if sub in self.modules and attr not in self.modules.get(mod, _EMPTY).symbols:
            return sub
        return None

    # -- the closure -----------------------------------------------------------------

    def closure(self, roots: Iterable[str]) -> Closure:
        return _Walk(self).run(roots)


_EMPTY = _Module("", Path(), False, False)


@dataclass
class Closure:
    """What a set of roots reaches: the loaded modules, the reached symbols (module,
    qualified name) with their code, and the third-party modules imported."""

    modules: set[str]
    symbols: dict[tuple[str, str], str]
    external: set[str]
    why: dict = field(default_factory=dict, repr=False)   # what reached each, first

    def path(self, target) -> list:
        """How ``target`` (a module name, or ``(module, symbol)``) was reached: itself,
        what reached it, ... back to a root."""
        out = []
        while target is not None and target not in out:
            out.append(target)
            target = self.why.get(target)
        return out

    def digest(self) -> str:
        h = hashlib.sha256()
        for m in sorted(self.modules):
            h.update(f"module {m}\n".encode())
        for (m, q), code in sorted(self.symbols.items()):
            h.update(f"symbol {m}:{q}\n".encode())
            h.update(code.encode())
        for e in sorted(self.external):
            h.update(f"external {e}\n".encode())
        return h.hexdigest()


class _Walk:
    def __init__(self, g: Graph):
        self.g = g
        self.loaded: set[str] = set()
        self.reached: dict[tuple[str, str], ast.AST] = {}
        self.external: set[str] = set()
        self.todo: list[tuple[str, str, ast.AST]] = []
        self.attrs_done: set[str] = set()
        self.why: dict = {}         # what reached each symbol or module first
        self.escaped: set[str] = set()
        # visiting code of a module nothing loads (a method matched by its name only):
        # its references are reached but load nothing; visited again if it is loaded
        self.phantom = False
        self.deferred: dict[str, list[tuple[str, str, ast.AST]]] = {}
        self.current = None

    def run(self, roots: Iterable[str]) -> Closure:
        for r in roots:
            mod, _, name = r.partition(":")
            if mod not in self.g.modules:
                raise KeyError(f"no module {mod} in the package")
            self.load(mod)
            if name and not self.resolve(mod, name):
                raise KeyError(f"{r}: no such symbol")
        while True:
            while self.todo:
                mod, key, node = self.todo.pop()
                self.current = (mod, key)
                self.phantom = mod not in self.loaded
                if self.phantom:
                    self.deferred.setdefault(mod, []).append((mod, key, node))
                self.visit(self.g.modules[mod], node)
                self.phantom = False
            # a module whose import-time code touches what is reached (registers into a
            # reached registry) is loaded too, whoever imports it
            touched = {(m, k.split("#")[0]) for m, k in self.reached if m in self.loaded}
            more = [m.name for m in self.g.modules.values()
                    if m.name not in self.loaded and not m.excluded
                    and any(w in touched for ref in m.effect_refs
                            for w in self.g.written(*ref))]
            # code anywhere (a front-end too) that patches a reached module's namespace
            # at run time is reached: whatever runs it changes what the closure computes
            touched_mods = {m for m, _ in touched}
            patched = False
            for m in self.g.modules.values():
                for target, attr, holder in m.patches:
                    hit = (any(w in touched for w in self.g.written(target, attr))
                           if attr != "*" else target in touched_mods)
                    if hit and holder and (m.name, holder) not in self.reached:
                        self.current = ("<patches the closure>", target, attr)
                        self.reach_symbol(m.name, holder)
                        cls = m.classes.get(holder)
                        for b in cls.body if cls is not None else ():   # (the patch's method)
                            for name in Graph._member_names(b):
                                self.reach_member(m.name, f"{holder}.{name}")
                        patched = True
            if not more and not patched:
                break
            for name in sorted(more):
                self.current = ("<registers into the closure>", name)
                self.load(name)
        return Closure(set(self.loaded),
                       {k: ast.dump(v) for k, v in self.reached.items()},
                       set(self.external), dict(self.why))

    # -- modules -------------------------------------------------------------------

    def load(self, name: str) -> None:
        if name in self.loaded:
            return
        if name not in self.g.modules:
            top = name.split(".")[0]
            if top != PACKAGE and top not in self.g.modules:
                self.external.add(top)
            return
        if self.phantom:
            return
        self.loaded.add(name)
        self.todo.extend(self.deferred.pop(name, ()))
        self.why[name] = self.current
        parent = name.rpartition(".")[0]
        if parent:
            self.load(parent)
        m = self.g.modules[name]
        for i, stmt in enumerate(m.effects):
            self.reach(name, f"<effect {i}>", stmt)
        for stmt in m.top_imports:
            self.imports(m, stmt)
        if m.autoload:
            prefix = f"{m.name}."
            for sub in self.g.modules:
                if sub.startswith(prefix) and "." not in sub[len(prefix):]:
                    self.load(sub)

    def imports(self, m: _Module, stmt: ast.AST) -> None:
        if isinstance(stmt, ast.Import):
            for a in stmt.names:
                parts = a.name.split(".")
                for k in range(1, len(parts) + 1):
                    self.load(".".join(parts[:k]))
        elif isinstance(stmt, ast.ImportFrom):
            src = _absolute(m, stmt)
            self.load(src)
            for a in stmt.names:
                if f"{src}.{a.name}" in self.g.modules:
                    self.load(f"{src}.{a.name}")

    # -- symbols -------------------------------------------------------------------

    def reach(self, mod: str, key: str, node: ast.AST) -> None:
        if (mod, key) in self.reached:
            return
        self.reached[(mod, key)] = node
        self.why[(mod, key)] = self.current
        self.todo.append((mod, key, node))

    def reach_symbol(self, mod: str, name: str) -> None:
        m = self.g.modules[mod]
        for i, node in enumerate(m.symbols[name]):
            if isinstance(node, ast.ClassDef):
                self.reach_class(mod, name)
            else:
                self.reach(mod, name if i == 0 else f"{name}#{i}", node)

    def reach_class(self, mod: str, qual: str) -> None:
        m = self.g.modules[mod]
        cls = m.classes.get(qual)
        if cls is None or (mod, qual) in self.reached:
            return
        # the shell: bases, decorators, class attributes, nested classes and the dunders;
        # the other methods are reached by name
        shell = ast.ClassDef(
            name=cls.name, bases=cls.bases, keywords=cls.keywords,
            decorator_list=cls.decorator_list, type_params=getattr(cls, "type_params", []),
            body=[b for b in cls.body
                  if not isinstance(b, (ast.FunctionDef, ast.AsyncFunctionDef))
                  or _is_dunder(b.name)] or [ast.Pass()])
        self.reach(mod, qual, shell)

    def reach_member(self, mod: str, qual: str) -> None:
        # (not a load: the instance's class was made by code that imported its module)
        cls_qual, _, name = qual.rpartition(".")
        cls = self.g.modules[mod].classes[cls_qual]
        for i, b in enumerate(cls.body):
            if name in Graph._member_names(b) and not isinstance(b, ast.ClassDef):
                self.reach(mod, f"{qual}#{i}", b)
            elif isinstance(b, ast.ClassDef) and b.name == name:
                self.reach_class(mod, f"{cls_qual}.{name}")

    def resolve(self, mod: str, name: str, seen: frozenset = frozenset()) -> bool:
        """Reach ``mod``'s ``name`` (a symbol, an imported name, a submodule)."""
        if (mod, name) in seen:
            return False
        seen = seen | {(mod, name)}
        m = self.g.modules.get(mod)
        if m is None:
            self.load(mod)
            return False
        self.load(mod)
        found = False
        if name in m.symbols:
            self.reach_symbol(mod, name)
            found = True
        if name in m.imports:
            src, attr = m.imports[name]
            if attr is None:
                self.load(src)
            else:
                found = self.resolve(src, attr, seen) or found
            found = True
        if f"{mod}.{name}" in self.g.modules:
            self.load(f"{mod}.{name}")
            found = True
        for s in m.star:
            found = self.resolve(s, name, seen) or found
        if not found and m.getattr_hook:
            self.reach_symbol(mod, "__getattr__")   # the module's dynamic attributes
            found = True
        return found

    def attribute(self, name: str) -> None:
        """``obj.name`` on an object of unknown type."""
        if name in self.attrs_done or _is_dunder(name):
            # (a dunder runs on an instance of a reached class, or of a base it names:
            # every reached class brings its own)
            return
        self.attrs_done.add(name)
        for mod, qual in self.g.members.get(name, ()):
            self.reach_member(mod, qual)
        for mod in sorted(self.escaped):
            m = self.g.modules.get(mod)
            if m is not None and (name in m.symbols or name in m.imports
                                  or f"{mod}.{name}" in self.g.modules):
                self.resolve(mod, name)

    def escape(self, mod: str) -> None:
        """Module ``mod`` is handled as a value: attribute names on objects of unknown type
        may be its symbols too (those already seen are matched now)."""
        if mod in self.escaped or mod not in self.g.modules or self.phantom:
            return
        self.escaped.add(mod)
        self.load(mod)
        m = self.g.modules[mod]
        for name in sorted(self.attrs_done):
            if name in m.symbols or name in m.imports or f"{mod}.{name}" in self.g.modules:
                self.resolve(mod, name)

    # -- a reached definition ----------------------------------------------------------

    def visit(self, m: _Module, node: ast.AST) -> None:
        local: dict[str, tuple[str, str | None]] = {}
        strings: list[str] = []
        dynamic = False
        for n in _walk(node):
            if isinstance(n, (ast.Import, ast.ImportFrom)):
                local.update(_import_aliases(m, n))
                self.imports(m, n)
                if isinstance(n, ast.ImportFrom):
                    src = _absolute(m, n)
                    for a in n.names:
                        if a.name != "*" and src in self.g.modules:
                            self.resolve(src, a.name)
                        elif a.name == "*" and src in self.g.modules:
                            for sym in self.g.modules[src].symbols:
                                self.resolve(src, sym)      # (everything it binds)
            elif isinstance(n, ast.Constant) and isinstance(n.value, str):
                strings.append(n.value)
        parents: dict[int, ast.AST] = {}
        for p in _walk(node):
            for c in ast.iter_child_nodes(p):
                parents[id(c)] = p
        for n in _walk(node):
            if isinstance(n, ast.Name):
                self.name(m, n.id, local)
                target = self.g._module_value(m, n.id, local)
                parent = parents.get(id(n))
                if target is not None and not (isinstance(parent, ast.Attribute)
                                               and parent.value is n):
                    self.escape(target)     # a module passed, stored or returned
            elif isinstance(n, ast.Attribute):
                base = self.module_of(m, n.value, local)
                if base is not None:
                    if not self.resolve(base, n.attr):
                        self.attribute(n.attr)
                else:
                    self.attribute(n.attr)
            elif isinstance(n, ast.keyword) and n.arg:
                self.attribute(n.arg)
            elif isinstance(n, ast.Call):
                callee = _callee_name(n)
                if callee in ("getattr", "hasattr", "setattr", "delattr", "attrgetter",
                              "methodcaller"):
                    named = [a for a in n.args[1:2] if isinstance(a, ast.Constant)]
                    if callee in ("attrgetter", "methodcaller"):
                        named = [a for a in n.args if isinstance(a, ast.Constant)]
                    if named:
                        for a in named:
                            if isinstance(a.value, str):
                                for part in a.value.split("."):
                                    self.attribute(part)
                    else:
                        dynamic = True
                elif callee in ("import_module", "__import__"):
                    arg = n.args[0] if n.args else None
                    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                        self.load(arg.value)
                    else:               # a computed import: what its module names
                        for s in sorted(m.strings):
                            if s in self.g.modules:
                                self.escape(s)
                elif callee in ("globals", "vars", "locals") and not n.args:
                    for s in m.symbols:
                        self.reach_symbol(m.name, s)
        if dynamic:     # getattr by a computed name: any identifier its module spells
            for s in sorted(m.strings | set(strings)):
                if s.isidentifier():
                    self.attribute(s)

    def name(self, m: _Module, name: str, local: dict) -> None:
        src = local.get(name)
        if src is not None:
            mod, attr = src
            if attr is None:
                self.load(mod)
            else:
                self.resolve(mod, attr)
            return
        if name in m.symbols or name in m.imports:
            self.resolve(m.name, name)
        elif m.star:
            for s in m.star:
                self.resolve(s, name)

    def module_of(self, m: _Module, expr: ast.expr, local: dict) -> str | None:
        """The package module ``expr`` names (``mod``, ``pkg.sub``), else ``None``."""
        if isinstance(expr, ast.Name):
            target = self.g._module_value(m, expr.id, local)
            if target is None:
                src = local.get(expr.id) or m.imports.get(expr.id)
                if src is not None and src[1] is None:
                    self.load(src[0])       # a third-party module: recorded
            return target
        if isinstance(expr, ast.Attribute):
            base = self.module_of(m, expr.value, local)
            if base is not None and f"{base}.{expr.attr}" in self.g.modules:
                bm = self.g.modules[base]
                if expr.attr not in bm.symbols:
                    return f"{base}.{expr.attr}"
        return None


# ---------------------------------------------------------------------------
# keys
# ---------------------------------------------------------------------------


def _versions(external: Iterable[str]) -> list[str]:
    """``dist==version`` of every distribution that provides one of ``external``
    (stdlib modules are the Python version's)."""
    from importlib import metadata

    dists = metadata.packages_distributions()
    out = set()
    for top in external:
        if top in sys.stdlib_module_names:
            continue
        for d in dists.get(top, [top]):
            try:
                out.add(f"{d}=={metadata.version(d)}")
            except metadata.PackageNotFoundError:
                out.add(f"{d}==?")
    return sorted(out)


_GRAPH: list[Graph] = []
_KEYS: dict[tuple, str] = {}


def graph() -> Graph:
    if not _GRAPH:
        _GRAPH.append(Graph())
    return _GRAPH[0]


def closure(roots: Iterable[str]) -> Closure:
    """The closure of ``roots`` (``"module:symbol"`` or ``"module"``) in the package."""
    return graph().closure(roots)


def _package_version() -> str:
    try:
        from importlib import metadata

        return metadata.version(PACKAGE)
    except Exception:       # noqa: BLE001 - a checkout without an install
        return "?"


def _self_digest() -> str:
    tree = ast.parse((ROOT / "keys.py").read_bytes())
    _strip_docstrings(tree)
    return hashlib.sha256(ast.dump(tree).encode()).hexdigest()[:16]


def source_key(roots: Iterable[str], label: str = "src") -> str:
    """``<label>-<hash>``: the code ``roots`` reach (module docstring), the third-party
    versions they import, the Python and package versions, and these rules. Cached per
    process and on disk by the sources' stats."""
    roots = tuple(roots)
    memo = (label, roots)
    if memo in _KEYS:
        return _KEYS[memo]
    entry = _disk_entry(roots, label)
    known = entry.read() if entry is not None else None
    if known is None:
        c = closure(roots)
        h = hashlib.sha256()
        h.update(c.digest().encode())
        h.update("\n".join(_versions(c.external)).encode())
        h.update(f"{sys.version_info[:2]}|{_package_version()}|{_self_digest()}".encode())
        known = f"{label}-{h.hexdigest()[:16]}"
        if entry is not None:
            entry.write(known)
    _KEYS[memo] = known
    return known


def plan_key() -> str:
    """The key of a cached plan (:data:`PLAN_ROOTS`)."""
    return source_key(PLAN_ROOTS, "plan")


def fab_key() -> str:
    """The key of a cached fabrication (:data:`FAB_ROOTS`)."""
    return source_key(FAB_ROOTS, "fab")


def engine_digest() -> str:
    """``engine-<hash>`` of every engine source (the package minus
    ``design.ENGINE_EXCLUDE``), docstrings stripped, and the package version: what
    :func:`spiderpig.design.engine_version` hashes (it adds the planner's defaults, which
    are these sources' code), computed without importing the engine (``spiderpig build``'s
    up-to-date check). Cached like :func:`source_key`."""
    memo = ("engine", ())
    if memo in _KEYS:
        return _KEYS[memo]
    entry = _disk_entry((), "engine")
    known = entry.read() if entry is not None else None
    if known is None:
        excluded = _engine_exclude()
        h = hashlib.sha256()
        for p in sorted(ROOT.rglob("*.py")):
            rel = p.relative_to(ROOT)
            if rel.parts[0] in excluded:
                continue
            tree = ast.parse(p.read_bytes())
            _strip_docstrings(tree)
            h.update(str(rel).encode() + b"\0" + ast.dump(tree).encode() + b"\0")
        h.update(f"{sys.version_info[:2]}|{_package_version()}".encode())
        known = f"engine-{h.hexdigest()[:16]}"
        if entry is not None:
            entry.write(known)
    _KEYS[memo] = known
    return known


_TEST_GRAPH: list[Graph] = []


def _test_files() -> list[Path]:
    return sorted((REPO / "tests").rglob("*.py")) if (REPO / "tests").is_dir() else []


def function_key(path: str | Path, name: str | tuple[str, ...] | None,
                 label: str = "gen") -> str:
    """``<label>-<hash>`` of what the top-level function ``name`` (or names) of the file
    ``path`` (a test module: a recorded fixture's generator) reaches: its helpers in
    ``tests/`` and the package code they call (``name`` None: the whole module, e.g. for a
    lambda). Cached per process and on disk by the package's and ``tests/``'s stats."""
    path = Path(path).resolve()
    names = (name,) if isinstance(name, str) else tuple(name or ())
    roots = (str(path), *names)
    memo = (label, roots)
    if memo in _KEYS:
        return _KEYS[memo]
    entry = _disk_entry(roots, label, extra=_test_files() + [path])
    known = entry.read() if entry is not None else None
    if known is None:
        if not _TEST_GRAPH or path not in {m.path for m in _TEST_GRAPH[0].modules.values()}:
            _TEST_GRAPH[:] = [Graph(extra=[*_test_files(), path])]
        g = _TEST_GRAPH[0]
        mod = next(m for m in g.modules.values() if m.path == path)
        if names and all(n in mod.symbols for n in names):
            c = g.closure([f"{mod.name}:{n}" for n in names])
        else:
            c = g.closure([mod.name] + [f"{mod.name}:{s}" for s in mod.symbols])
        h = hashlib.sha256()
        h.update(c.digest().encode())
        h.update("\n".join(_versions(c.external)).encode())
        h.update(f"{sys.version_info[:2]}|{_package_version()}|{_self_digest()}".encode())
        known = f"{label}-{h.hexdigest()[:16]}"
        if entry is not None:
            entry.write(known)
    _KEYS[memo] = known
    return known


def callable_key(fn, label: str = "gen") -> str:
    """:func:`function_key` of the function object ``fn`` (a lambda or a nested function:
    its whole module)."""
    import inspect

    path = inspect.getsourcefile(fn)
    if not path:
        raise ValueError(f"{fn!r}: no source file")
    qual = getattr(fn, "__qualname__", "")
    name = None if ("<" in qual or "." in qual) else qual
    return function_key(path, name, label)


# ---------------------------------------------------------------------------
# the on-disk memo (by the sources' stats)
# ---------------------------------------------------------------------------


@dataclass
class _Entry:
    path: Path
    signature: str

    def read(self) -> str | None:
        try:
            doc = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return None
        if not isinstance(doc, dict) or doc.get("signature") != self.signature:
            return None
        v = doc.get("key")
        return v if isinstance(v, str) else None

    def write(self, key: str) -> None:
        import tempfile

        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=self.path.parent, prefix=".keys-")
            with os.fdopen(fd, "w") as f:
                json.dump({"signature": self.signature, "key": key}, f)
            os.replace(tmp, self.path)
        except OSError:
            pass


def _disk_entry(roots: tuple[str, ...], label: str, extra: Iterable[Path] = ()
                ) -> _Entry | None:
    """Where a key is kept: a file named by the signature of what it depends on (the
    package's sources' and ``extra``'s paths, sizes, times and inodes, the roots, the
    Python, and the installed distributions: the site-packages folders' times change
    with any install), so any of those changing computes afresh."""
    env = os.environ.get("SPIDERPIG_DIGEST_CACHE", "").strip()
    if env.lower() in ("off", "0", "false", "no"):
        return None
    if env:
        base = Path(env).expanduser()
    else:
        base = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "spiderpig" \
            / "engine-version"
    h = hashlib.sha256()
    for part in (str(ROOT), sys.version, label, *roots):
        h.update(part.encode() + b"\0")
    import site

    sites = [*site.getsitepackages(), site.getusersitepackages()]
    try:
        for p in [*sorted(ROOT.rglob("*.py")), *extra]:
            st = p.stat()
            h.update(f"{p}\0{st.st_size}\0{st.st_mtime_ns}\0"
                     f"{st.st_ctime_ns}\0{st.st_ino}\n".encode())
        for d in sites:
            if os.path.isdir(d):
                h.update(f"{d}\0{os.stat(d).st_mtime_ns}\n".encode())
    except OSError:
        return None
    sig = h.hexdigest()
    return _Entry(base / f"keys-{sig[:32]}.json", sig)


def main(argv=None) -> int:
    """``python -m spiderpig.keys``: the plan and fabrication keys and the engine digest.
    ``... ROOT [ROOT ...]``: the closure of roots (``module:symbol``) module by module;
    ``--why TARGET [ROOT ...]`` (a module, or ``module:symbol``): how the roots (the
    plan's by default) reach it."""
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        print(plan_key())
        print(fab_key())
        print(engine_digest())
        return 0
    if args[0] == "--why" and len(args) > 1:
        c = closure(PLAN_ROOTS if len(args) < 3 else args[2:])
        mod, _, sym = args[1].partition(":")
        hits = [mod] if not sym and mod in c.why else [
            k for k in c.symbols if k[0] == mod and k[1].split("#")[0] == sym]
        if not hits:
            print(f"{args[1]}: not reached")
            return 1
        for step in c.path(hits[0]):
            print(f"  {step}")
        return 0
    c = closure(args)
    by_mod: dict[str, list[str]] = {}
    for m, q in c.symbols:
        by_mod.setdefault(m, []).append(q)
    for m in sorted(c.modules | set(by_mod)):
        print(f"{m}{'' if m in c.modules else ' (not loaded)'}: {len(by_mod.get(m, []))} "
              "symbols")
    print(f"external: {sorted(c.external)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""The doc check: every name the docs put in backticks resolves against the code
(``mise run doc-check``; ``docs/agentlib/ROADMAP.md`` W0 / W7).

    python -m tests.doc_check                 # report the misses of the default docs, exit 0
    python -m tests.doc_check --strict        # exit 1 on a miss the allow-list doesn't hold
    python -m tests.doc_check docs/agentlib/API.md -v    # one doc, every check listed

What it resolves, per backticked span (and every ``mise run X`` anywhere, code blocks
included):

- ``mise run X``: a task of ``mise.toml``; ``spiderpig X``: a command of
  ``spiderpig.cli.COMMANDS``.
- a dotted name (``stack.finalize``, ``BoltCrank.for_sheet``, ``api.plan_config(...)``): a
  module of the package by its dotted suffix (``pivots.standoff``, ``bom``), then the rest
  in it statically (its top-level names, its imports followed, a class's methods, class
  attributes, ``self.X`` attributes and its bases' members); else a class of the package
  and its member; else an importable outside module (``os.replace``) by ``getattr``. A
  name whose first part is neither (an instance: ``mech.meta``) passes when its last part
  is a name in the code (``lenient``).
- a path (``spiderpig/stack.py``, ``tests/cache.py``, ``crank.py``, ``docs/...``): a file or
  folder of the repo, by itself or as a suffix of one (``<placeholders>`` and ``*`` as
  globs). Generated places (``build/``, the store, ``~``, ``/tmp``) are not checked; a
  server route (``/api/...``, ``/ws``) must appear in ``spiderpig/server``.
- a ``--flag``: the literal in the code, ``mise.toml`` or the workflows, or one of pytest's
  options (``--no-X`` passes with ``--X``).
- an environment variable (``SPIDERPIG_STORE``, ``$VITE_PORT``, ``X=1``): the name in the code.
- a single identifier (``design_side``, ``BuildConfig()``, ``gap_sink``): a name, attribute,
  argument or string in the code (``spiderpig/``, ``tests/``, ``viewer/src``, the tasks).

Anything else (expressions, prose in backticks) is counted as unchecked. The allow-list
(``tests/doc_check_allow.txt``) holds spans that are legitimately not code: one per line,
``token`` or ``doc-path: token``; ``#`` comments. The dated records under ``docs/history/``
name the code as it was then and are never checked (:data:`HISTORY`), even when named.
"""

from __future__ import annotations

import argparse
import ast
import importlib
import importlib.util
import re
import sys
import tomllib
from dataclasses import dataclass
from functools import cache, cached_property
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DOCS = ("CLAUDE.md", "AGENTS.md", "README.md", "docs/ARCHITECTURE.md",
                "docs/agentlib/API.md", "docs/agentlib/TESTING.md", "docs/agentlib/ROADMAP.md",
                "docs/agentlib/SCOPE.md")
ALLOW_FILE = ROOT / "tests" / "doc_check_allow.txt"
HISTORY = ("docs/history/",)
"""The dated records (each headed by its date and status): the code they name is gone."""
CODE_DIRS = ("spiderpig", "tests")
SELF = ("tests/doc_check.py", "tests/test_doc_check.py")
"""Left out of the index: the check and its test name made-up names on purpose."""
TEXT_SOURCES = ("viewer/src", "viewer/vite.config.ts", "viewer/package.json", "mise.toml",
                "pyproject.toml", ".github", ".pre-commit-config.yaml", "hatch_build.py",
                ".gitignore")
SKIP_DIRS = {"node_modules", ".venv", "__pycache__", "dist", ".git", ".ruff_cache",
             ".pytest_cache"}
PYTEST_FLAGS = {
    "--durations", "--dist", "--junitxml", "--lf", "--ff", "--maxfail", "--co",
    "--collect-only", "--pdb", "--tb", "--cov", "--cov-report", "--headed", "--browser",
    "--basetemp", "--numprocesses", "--forked", "--timeout", "--deselect", "--ignore",
    "--strict-markers", "--setup-show", "--runxfail", "--exitfirst", "--capture",
    "--fixtures", "--markers", "--version", "--help",
}
"""Options of pytest and its plugins the docs may name (not in this repo's code)."""
GENERATED = ("build/", "dist/", ".spiderpig", "~", "/tmp", "/root", "$", "http://",
             "https://", "<store>", "<out>", "<run>", "store/", "designs/", "bakes/",
             "laser/", "print/", "pin_loads/", "mech/", "plans/", "stores/", "fab/",
             "out/", "exports/", "cache/", "viewer/node_modules", "viewer/dist",
             "spiderpig/viewer/dist", "node_modules")
FILE_EXT = re.compile(r"\.(py|md|toml|json|ts|js|yml|yaml|txt|csv|cfg|html|lock|xml|glb|"
                      r"dxf|stl|step|ini|sh|ipynb)$")
IDENT = r"[A-Za-z_]\w*"
DOTTED = re.compile(rf"^{IDENT}(\.{IDENT})+$")
CALL = re.compile(rf"^({IDENT}(?:\.{IDENT})*)\(.*\)$", re.S)
ENV = re.compile(r"^\$?([A-Z][A-Z0-9]*_[A-Z0-9_]+)(=\S*)?$")
FLAG = re.compile(r"^(--[a-z][a-z0-9-]*)(?:[ =].*)?$", re.S)
MISE_RUN = re.compile(r"mise run ([A-Za-z][\w:-]*)(?![\w<…-])")
OUTPUT_EXT = (".json", ".md", ".csv", ".dxf", ".stl", ".step", ".glb", ".xml", ".log")
"""What a run writes: a pattern with a placeholder that names one isn't looked for."""


@dataclass(frozen=True)
class Result:
    doc: str
    line: int
    token: str
    kind: str          # mise, cli, dotted, path, flag, env, name, unchecked
    status: str        # ok, lenient, miss, allowed, unchecked
    why: str = ""


# --------------------------------------------------------------------------- the code index


def _uncommented(path: Path) -> str:
    """A non-Python source's text without its comments (``//`` and ``/* */`` in TypeScript
    and JavaScript, ``#`` elsewhere)."""
    text = path.read_text()
    if path.suffix in (".ts", ".js"):
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
        return re.sub(r"(?m)(^|[^:\\])//.*$", r"\1", text)
    if path.suffix == ".json":
        return text
    return re.sub(r"(?m)(^|\s)#.*$", r"\1", text)


@dataclass
class _Module:
    name: str
    path: Path

    @cached_property
    def tree(self) -> ast.Module:
        return ast.parse(self.path.read_text())

    @cached_property
    def names(self) -> dict[str, ast.AST]:
        """Top-level names: defs, classes, assignments, imports (the alias node), and the
        branches of top-level ``if`` / ``try`` blocks."""
        out: dict[str, ast.AST] = {}

        def visit(body):
            for node in body:
                if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef):
                    out[node.name] = node
                elif isinstance(node, ast.Assign):
                    for t in node.targets:
                        for n in ast.walk(t):
                            if isinstance(n, ast.Name):
                                out[n.id] = node
                elif isinstance(node, ast.AnnAssign | ast.AugAssign) and isinstance(
                        node.target, ast.Name):
                    out[node.target.id] = node
                elif isinstance(node, ast.Import | ast.ImportFrom):
                    for a in node.names:
                        out[(a.asname or a.name).split(".")[0]] = node
                elif isinstance(node, ast.If | ast.Try | ast.With):
                    visit(node.body)
                    visit(getattr(node, "orelse", []))
                    for h in getattr(node, "handlers", []):
                        visit(h.body)
                    visit(getattr(node, "finalbody", []))
        visit(self.tree.body)
        return out

    @cached_property
    def docstrings(self) -> set[int]:
        """The ids of the docstring nodes: every string standing as a statement (a module's,
        class's or function's docstring, an attribute's after its assignment): prose, never a
        name of the code."""
        out = set()
        for n in ast.walk(self.tree):
            if (isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant)
                    and isinstance(n.value.value, str)):
                out.add(id(n.value))
        return out

    @cached_property
    def strings(self) -> set[str]:
        """Every string constant of the code but the docstrings."""
        docs = self.docstrings
        return {n.value for n in ast.walk(self.tree) if isinstance(n, ast.Constant)
                and isinstance(n.value, str) and id(n) not in docs}


class Index:
    """What the code defines: the package's modules, classes and names, every word of the
    code, the flags, the tasks and the commands."""

    def __init__(self, root: Path = ROOT):
        self.root = root
        self.modules: dict[str, _Module] = {}
        for base in CODE_DIRS:
            for p in sorted((root / base).rglob("*.py")):
                if SKIP_DIRS & set(p.relative_to(root).parts) or (
                        p.relative_to(root).as_posix() in SELF):
                    continue
                parts = list(p.relative_to(root).with_suffix("").parts)
                if parts[-1] == "__init__":
                    parts.pop()
                self.modules[".".join(parts)] = _Module(".".join(parts), p)

    # ---- words, flags, tasks

    @cached_property
    def files(self) -> list[str]:
        out = []
        for p in self.root.rglob("*"):
            rel = p.relative_to(self.root)
            if SKIP_DIRS & set(rel.parts) or rel.parts[0] in ("build", ".spiderpig", ".claude"):
                continue
            out.append(rel.as_posix() + ("/" if p.is_dir() else ""))
        return out

    @cached_property
    def _texts(self) -> list[str]:
        """The code's strings (no docstrings, no comments: the AST's) and the other sources
        with their comments stripped."""
        texts = ["\n".join(sorted(m.strings)) for m in self.modules.values()]
        for src in TEXT_SOURCES:
            p = self.root / src
            files = [p] if p.is_file() else sorted(
                f for f in p.rglob("*") if f.is_file()
                and f.suffix in (".ts", ".js", ".yml", ".yaml", ".json", ".toml")
                and not SKIP_DIRS & set(f.parts)) if p.is_dir() else []
            texts += [_uncommented(f) for f in files]
        return texts

    @cached_property
    def words(self) -> set[str]:
        """Every identifier, attribute, argument and string (whole and split in words) of
        the code, and every word of the other sources (TypeScript, the tasks)."""
        out: set[str] = set()
        for m in self.modules.values():
            for n in ast.walk(m.tree):
                if isinstance(n, ast.Name):
                    out.add(n.id)
                elif isinstance(n, ast.Attribute):
                    out.add(n.attr)
                elif isinstance(n, ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef):
                    out.add(n.name)
                elif isinstance(n, ast.arg) or (isinstance(n, ast.keyword) and n.arg):
                    out.add(n.arg)
                elif isinstance(n, ast.alias):
                    out.update((n.asname or n.name).split("."))
                    out.update(n.name.split("."))
            # strings that are keys (no space: "gap_sink", "SPIDERPIG_STORE", "2_mesh_share"),
            # whole and in words; prose strings (messages) and docstrings name nothing
            for value in m.strings:
                if not re.search(r"\s", value):
                    out.add(value)
                    for w in re.findall(r"\w+", value):
                        out.update((w, w.lstrip("0123456789_")))   # 2_mesh_share: mesh_share
        for text in self._texts[len(self.modules):]:
            out.update(re.findall(r"\w+", text))
        return out

    @cached_property
    def flags(self) -> set[str]:
        out = set(PYTEST_FLAGS)
        for text in self._texts:
            out.update(re.findall(r"(--[a-z][a-z0-9-]*)", text))
        return out

    @cached_property
    def tasks(self) -> set[str]:
        doc = tomllib.loads((self.root / "mise.toml").read_text())
        return set(doc.get("tasks", {}))

    @cached_property
    def commands(self) -> set[str]:
        mod = self.modules.get("spiderpig.cli")
        node = mod.names.get("COMMANDS") if mod else None
        if isinstance(node, ast.AnnAssign | ast.Assign) and isinstance(node.value, ast.Dict):
            return {k.value for k in node.value.keys if isinstance(k, ast.Constant)}
        return set()

    @cached_property
    def classes(self) -> dict[str, list[tuple[_Module, ast.ClassDef]]]:
        out: dict[str, list[tuple[_Module, ast.ClassDef]]] = {}
        for m in self.modules.values():
            if not m.name.startswith("spiderpig"):
                continue
            for node in ast.walk(m.tree):
                if isinstance(node, ast.ClassDef):
                    out.setdefault(node.name, []).append((m, node))
        return out

    @cached_property
    def strings(self) -> set[str]:
        """Every string constant of the code, whole."""
        return set().union(*(m.strings for m in self.modules.values()))

    def classes_named(self, var: str) -> list[tuple[_Module, ast.ClassDef]]:
        """The package's classes a variable ``var`` would hold by its name (``design``:
        ``Design``, ``plan``: ``StackPlan``)."""
        want = var.replace("_", "").lower()
        return [c for name, found in self.classes.items() for c in found
                if name.lower() == want or (name.lower().endswith(want) and len(want) > 3)]

    # ---- resolution

    def modules_by_suffix(self, dotted: str) -> list[_Module]:
        name = dotted if dotted.startswith(("spiderpig", "tests")) else f"spiderpig.{dotted}"
        exact = self.modules.get(name)
        if exact:
            return [exact]
        return [m for k, m in self.modules.items()
                if k.startswith(("spiderpig.", "tests.")) and k.endswith("." + dotted)]

    def in_module(self, mod: _Module, rest: list[str], depth: int = 0) -> str | None:
        """None when ``rest`` resolves in ``mod``, else why not."""
        if not rest:
            return None
        if depth > 8:
            return "import cycle"
        head = rest[0]
        sub = self.modules.get(f"{mod.name}.{head}")
        if sub is not None:
            return self.in_module(sub, rest[1:], depth + 1)
        node = mod.names.get(head)
        if node is None:
            if "__getattr__" in mod.names and head in mod.strings:
                return None                     # a PEP 562 lazy name
            return f"{mod.name} has no {head!r}"
        if isinstance(node, ast.ImportFrom | ast.Import):
            return self._through_import(mod, node, head, rest[1:], depth)
        if isinstance(node, ast.ClassDef):
            return self.member(mod, node, rest[1:])
        if len(rest) == 1:
            return None
        # an attribute of a function or a value: its name must be a name in the code
        return None if rest[-1] in self.words else f"{'.'.join(rest)} not in the code"

    def _through_import(self, mod, node, head, rest, depth) -> str | None:
        if isinstance(node, ast.Import):
            for a in node.names:
                if (a.asname or a.name.split(".")[0]) == head:
                    target = a.name if a.asname else a.name.split(".")[0]
                    return self._external_or_module(target.split("."), rest, depth)
            return None
        base = node.module or ""
        if node.level:
            pkg = mod.name.split(".")
            if mod.path.name != "__init__.py":
                pkg = pkg[:-1]
            pkg = pkg[:len(pkg) - (node.level - 1)]
            base = ".".join([*pkg, *([base] if base else [])])
        for a in node.names:
            if (a.asname or a.name) == head:
                return self._external_or_module([*base.split("."), a.name], rest, depth)
        return None

    def _external_or_module(self, parts: list[str], rest: list[str], depth: int) -> str | None:
        dotted = ".".join(parts)
        if dotted in self.modules:
            return self.in_module(self.modules[dotted], rest, depth + 1)
        parent = ".".join(parts[:-1])
        if parent in self.modules:
            return self.in_module(self.modules[parent], [parts[-1], *rest], depth + 1)
        if parts[0] in ("spiderpig", "tests"):
            return f"no module {dotted}"
        return self.external(parts + rest)

    def member(self, mod: _Module, cls: ast.ClassDef, rest: list[str],
               seen: frozenset = frozenset()) -> str | None:
        if not rest:
            return None
        name = rest[0]
        members: set[str] = set()
        for node in cls.body:
            if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef):
                members.add(node.name)
            elif isinstance(node, ast.Assign):
                members.update(n.id for t in node.targets for n in ast.walk(t)
                               if isinstance(n, ast.Name))
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                members.add(node.target.id)
        for n in ast.walk(cls):
            if (isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)
                    and isinstance(n.value, ast.Name) and n.value.id in ("self", "cls")):
                members.add(n.attr)
        if name in members:
            if len(rest) == 1:
                return None
            inner = next((n for n in cls.body if isinstance(n, ast.ClassDef)
                          and n.name == name), None)
            if inner is not None:
                return self.member(mod, inner, rest[1:], seen)
            return None if rest[-1] in self.words else f"{'.'.join(rest)} not in the code"
        external_base = False
        for base in cls.bases:
            bname = base.id if isinstance(base, ast.Name) else (
                base.attr if isinstance(base, ast.Attribute) else None)
            if bname is None or bname in seen:
                continue
            found = self.classes.get(bname)
            if not found:
                external_base = bname not in ("object", "Generic", "Protocol")
                continue
            for bmod, bcls in found:
                if self.member(bmod, bcls, rest, seen | {cls.name}) is None:
                    return None
        if external_base and name in self.words:
            return None             # (an Enum's, a NamedTuple's, an Exception's member)
        if name in ("__init__", "__call__", "__post_init__"):
            return None
        return f"{cls.name} has no {name!r}"

    def external(self, parts: list[str]) -> str | None:
        try:
            obj = importlib.import_module(parts[0])
        except Exception as e:                  # noqa: BLE001  (any import failure is a miss)
            return f"no module {parts[0]} ({type(e).__name__})"
        for i, p in enumerate(parts[1:], 1):
            if hasattr(obj, p):
                obj = getattr(obj, p)
                continue
            try:
                obj = importlib.import_module(".".join(parts[:i + 1]))
            except Exception:                   # noqa: BLE001
                return f"{'.'.join(parts[:i])} has no {p!r}"
        return None

    def dotted(self, token: str) -> tuple[str, str]:
        """(status, why) of a dotted name."""
        parts = token.split(".")
        if parts[0] == "spiderpig" and len(parts) > 1 and f"spiderpig.{parts[1]}" in self.modules:
            parts = parts[1:]
        why = ""
        for k in range(len(parts), 0, -1):
            mods = self.modules_by_suffix(".".join(parts[:k]))
            if not mods:
                continue
            whys = [self.in_module(m, parts[k:]) for m in mods]
            if None in whys:
                return "ok", ""
            # a variable named after its class (design.mech: a Design's), or a field path
            # the code spells as a string (sim.speed_mm_s, a Spec target)
            for c in self.classes_named(parts[k - 1]) if k == 1 else ():
                if self.member(*c, parts[k:]) is None:
                    return "lenient", f"a {c[1].name}'s attribute"
            if token in self.strings or (len(parts) - k == 1 and parts[-1] in self.strings):
                return "lenient", f"{parts[-1]!r} is a string of the code"
            return "miss", whys[0] or ""        # the longest module prefix decides
        if parts[0] in self.classes:
            whys = [self.member(m, c, parts[1:]) for m, c in self.classes[parts[0]]]
            return ("ok", "") if None in whys else ("miss", whys[0] or "")
        if parts[0] not in self.words and importlib.util.find_spec(parts[0]) is None:
            return "miss", f"{parts[0]!r} is no module, class or name of the code"
        if importlib.util.find_spec(parts[0]) is not None and parts[0] not in ("tests",):
            why = self.external(parts)
            if why is None:
                return "ok", ""
            if parts[0] not in self.words:
                return "miss", why
        # an instance's attribute (mech.meta): its last name must be a name in the code
        if parts[-1] in self.words:
            return "lenient", f"{parts[0]!r} is not a module or class; {parts[-1]!r} is a name"
        return "miss", f"{parts[-1]!r} is not a name in the code"

    def path(self, token: str) -> tuple[str, str]:
        t = re.sub(r"(:\d+(-\d+)?)+$", "", token.strip())
        t = t.split("?")[0].split("#")[0]
        if t.startswith(("/api", "/ws")):
            route = re.sub(r"\{[^}]*\}", "", t).rstrip("/")
            server = "\n".join(s for k, m in self.modules.items() for s in m.strings
                             if k.startswith("spiderpig.server"))
            head = "/".join(route.split("/")[:3])
            return ("ok", "") if head in server else ("miss", f"no route {head} in the server")
        if "://" in t:              # a resource URI (spiderpig://guide): its fixed part
            head = re.split(r"[{<]", t)[0]
            return ("ok", "") if head in self.text else ("miss", f"{head} is not in the code")
        t, _, test = t.partition("::")
        t = t.removeprefix("./")
        if t.startswith(GENERATED) or any(g in t for g in ("<store>", "<out>", "<run>")):
            return "skip", "a generated place"
        if t.startswith("/"):
            return "skip", "an absolute path"
        if re.fullmatch(r"\.\w+", t):
            return "skip", "an extension"
        first = t.split("/")[0]
        if "/" in t and re.fullmatch(r"[A-Z][A-Z0-9_]+", first):
            return "skip", f"under {first}, a variable"
        pattern = re.sub(r"<[^>]*>|\{[^}]*\}", "*", t)
        if "*" in pattern:
            rx = re.compile("^(.*/)?" + re.escape(pattern.rstrip("/"))
                            .replace(r"\*\*/", "(.*/)?").replace(r"\*", "[^/]*") + "/?$")
            if any(rx.match(f) for f in self.files):
                return "ok", ""
            if pattern.endswith(OUTPUT_EXT) and not t.startswith(("spiderpig/", "tests/",
                                                                   "docs/", "viewer/")):
                return "skip", "a generated name"
            return "miss", f"no file matches {pattern}"
        bare = t.rstrip("/")
        found = [f for f in self.files
                 if f.rstrip("/") == bare or f.rstrip("/").endswith("/" + bare)]
        if (self.root / bare).exists() or found:
            if test:                    # tests/test_x.py::test_name: the test is there
                name = re.split(r"[\[:]", test)[0]
                path = self.root / bare if (self.root / bare).exists() else (
                    self.root / found[0])
                if not re.search(rf"\bdef {re.escape(name)}\b", path.read_text()):
                    return "miss", f"no {name} in {bare}"
            return "ok", ""
        if ("/" in bare and first not in {f.split("/")[0] for f in self.files}
                and not FILE_EXT.search(bare) and not t.endswith("/")):
            # not a path of the repo (a failure code plan/no_plan, an action actions/cache):
            # every part a name of the code
            parts = [p for p in re.split(r"[/.]", bare) if p]
            if all(p in self.words or p in self.text for p in parts):
                return "lenient", "not a repo path; its parts are names of the code"
            return "miss", f"no file or folder {bare}"
        if "/" not in bare:
            stem = bare.rsplit(".", 1)[0]
            if bare.endswith(".py") and importlib.util.find_spec(stem) is not None:
                return "ok", ""             # a package by its name (coverage.py)
            if bare in self.text:
                return "lenient", "a file the code writes or names"
        return "miss", f"no file or folder {bare}"

    @cached_property
    def text(self) -> str:
        """The code and the other sources as one text (for names inside strings)."""
        return "\n".join(self._texts)


# --------------------------------------------------------------------------- the docs


def _spans(text: str):
    """(line, span) of every inline backticked span outside fenced code blocks."""
    fence = False
    for i, line in enumerate(text.splitlines(), 1):
        if line.lstrip().startswith("```"):
            fence = not fence
            continue
        if fence:
            continue
        for m in re.finditer(r"(`+)(.+?)\1", line):
            yield i, m.group(2).strip()


def _classify(token: str) -> str:
    if token.startswith("mise run"):
        return "mise"
    if re.match(r"^spiderpig [a-z]", token):
        return "cli"
    if FLAG.match(token):
        return "flag"
    if ENV.match(token):
        return "env"
    if " " not in token and ("/" in token or FILE_EXT.search(token.split(":")[0])):
        return "path"
    if DOTTED.match(token):
        return "dotted"
    if CALL.match(token):
        return "call"
    if re.fullmatch(IDENT, token):
        return "name"
    if re.fullmatch(rf"{IDENT}=\S+", token):
        return "keyword"
    return "unchecked"


def check_token(index: Index, token: str) -> tuple[str, str, str]:
    """(kind, status, why) of one backticked span."""
    kind = _classify(token)
    if kind == "mise":
        task = token.split()[2] if len(token.split()) > 2 else ""
        if re.search(r"[<…]", task) or task.endswith("-"):
            return kind, "skip", "a placeholder"
        if task in index.tasks:
            return kind, "ok", ""
        return kind, "miss", f"no task {task!r}"
    if kind == "cli":
        cmd = token.split()[1]
        if cmd in index.commands:
            return kind, "ok", ""
        return kind, "miss", f"no spiderpig command {cmd!r}"
    if kind == "flag":
        flag = FLAG.match(token).group(1)
        known = flag in index.flags or (flag.startswith("--no-")
                                        and "--" + flag[5:] in index.flags)
        return kind, "ok" if known else "miss", "" if known else f"no {flag} in the code"
    if kind == "env":
        name = ENV.match(token).group(1)
        if name in index.words:
            return kind, "ok", ""
        return kind, "miss", f"{name} is not in the code"
    if kind == "path":
        status, why = index.path(token)
        return kind, status, why
    if kind == "dotted":
        status, why = index.dotted(token)
        return kind, status, why
    if kind == "call":
        callee = CALL.match(token).group(1)
        status, why = (index.dotted(callee) if "." in callee else
                       (("ok", "") if callee in index.words else
                        ("miss", f"{callee!r} is not a name in the code")))
        return kind, status, why
    if kind in ("name", "keyword"):
        name = token.split("=")[0]
        if name in index.words:
            return kind, "ok", ""
        if re.fullmatch(r"[0-9a-f]{7,40}", name):
            return kind, "skip", "a commit"
        if importlib.util.find_spec(name) is not None:
            return kind, "ok", ""           # a module of Python or a dependency
        return kind, "miss", f"{name!r} is not a name in the code"
    return kind, "unchecked", ""


def load_allow(path: Path = ALLOW_FILE) -> set[tuple[str | None, str]]:
    out: set[tuple[str | None, str]] = set()
    if not path.exists():
        return out
    for raw in path.read_text().splitlines():
        line = raw.split(" #")[0].strip() if not raw.lstrip().startswith("#") else ""
        if not line:
            continue
        doc, sep, token = line.partition(": ")
        out.add((doc, token) if sep and doc.endswith(".md") else (None, line))
    return out


@cache
def default_index() -> Index:
    """The index of this checkout, built once per process."""
    return Index()



def check(docs, index: Index | None = None, allow=None, *, every: bool = False
          ) -> list[Result]:
    """The misses (every result with ``every``) of ``docs`` (repo-relative paths)."""
    index = index or default_index()
    allow = load_allow() if allow is None else allow
    out: list[Result] = []
    seen: dict[str, tuple[str, str, str]] = {}
    for doc in docs:
        path = ROOT / doc if not Path(doc).is_absolute() else Path(doc)
        rel = path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)
        if rel.startswith(HISTORY):
            continue
        text = path.read_text()
        items = list(_spans(text))
        spans = {t for _, t in items}
        items.extend((i, f"mise run {m.group(1)}")       # `mise run X` in code blocks too
                     for i, line in enumerate(text.splitlines(), 1)
                     for m in MISE_RUN.finditer(line) if f"mise run {m.group(1)}" not in spans)
        for line_no, token in items:
            if token not in seen:
                seen[token] = check_token(index, token)
            kind, status, why = seen[token]
            if status == "miss" and ((rel, token) in allow or (None, token) in allow):
                status = "allowed"
            if every or status == "miss":
                out.append(Result(rel, line_no, token, kind, status, why))
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("docs", nargs="*", default=list(DEFAULT_DOCS))
    p.add_argument("--strict", action="store_true", help="exit 1 on a miss")
    p.add_argument("-v", "--verbose", action="store_true", help="list every check")
    args = p.parse_args(argv)
    results = check(args.docs, every=True)
    counts: dict[str, int] = {}
    for r in results:
        counts[r.status] = counts.get(r.status, 0) + 1
        if r.status == "miss" or (args.verbose and r.status != "ok"):
            print(f"{r.doc}:{r.line}: {r.status} {r.kind} `{r.token}`"
                  + (f": {r.why}" if r.why else ""))
    print("doc check: " + ", ".join(f"{v} {k}" for k, v in sorted(counts.items()))
          + ("" if args.strict else " (report only; --strict fails on a miss)"))
    return 1 if args.strict and counts.get("miss") else 0


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    raise SystemExit(main())

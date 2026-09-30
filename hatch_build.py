"""Hatchling build hook: a spiderpig wheel or sdist carries the built viewer.

``spiderpig/viewer/dist`` (Vite's output, ``mise run viewer-build``) is what
``spiderpig view`` serves without Node on the user's machine (decision 6 in
``docs/agentlib/DECISIONS.md``), so the release build makes it first
(``mise run release``) and this hook refuses to build without it, then checks the
artifact holds ``index.html`` and nothing of ``viewer/`` (the TypeScript sources,
``node_modules``). An editable install (``uv sync`` on a checkout) is exempt: the
viewer is built afterwards, in place.
"""

from __future__ import annotations

import tarfile
import zipfile
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface

DIST = Path("spiderpig") / "viewer" / "dist"
FORBIDDEN = ("node_modules/", "viewer/src/", "viewer/package.json", "/tests/")


class ViewerDistHook(BuildHookInterface):
    PLUGIN_NAME = "custom"

    def initialize(self, version: str, build_data: dict) -> None:
        if version == "editable":
            return
        index = Path(self.root) / DIST / "index.html"
        if not index.is_file():
            raise RuntimeError(
                f"{DIST}/index.html is missing: the {self.target_name} must ship the built "
                "viewer. Build it first with `mise run viewer-build` (or `cd viewer && npm ci "
                "&& npm run build`), or run `mise run release`, which builds the viewer and "
                "then the sdist and wheel."
            )

    def finalize(self, version: str, build_data: dict, artifact_path: str) -> None:
        if version == "editable":
            return
        names = _names(artifact_path)
        want = f"{DIST.as_posix()}/index.html"
        if not any(n == want or n.endswith("/" + want) for n in names):
            raise RuntimeError(f"{artifact_path} lacks {want}")
        bad = sorted(n for n in names if any(f in n for f in FORBIDDEN))
        if bad:
            raise RuntimeError(f"{artifact_path} must not carry {bad[:5]}")


def _names(artifact_path: str) -> list[str]:
    if artifact_path.endswith(".whl"):
        with zipfile.ZipFile(artifact_path) as z:
            return z.namelist()
    with tarfile.open(artifact_path) as t:
        return t.getnames()

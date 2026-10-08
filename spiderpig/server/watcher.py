"""File watcher that re-bakes viewer data and pushes reload events.

On any ``*.py`` change under the repo root (skipping tests, caches, and
build output), runs the bake in a worker thread and broadcasts a JSON
``{"type": "reload"}`` to every connected ``/ws`` client. Clients receive
``{"type": "rebaking"}`` first so the UI can show a spinner.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Callable
from pathlib import Path

from fastapi import WebSocket
from watchfiles import Change, awatch

log = logging.getLogger("watcher")

_IGNORED_DIRS = {
    ".venv", ".git", "__pycache__", ".ruff_cache", ".pytest_cache", "build", "data",
    "node_modules", "dist",
}


def is_ignored_dir(name: str) -> bool:
    """Directories a scan from the repo root skips (caches, envs, build output, dot-dirs
    such as ``.claude`` holding other worktrees)."""
    return name in _IGNORED_DIRS or name.startswith(".")


def is_source(path: Path) -> bool:
    """Is ``path`` a Python source the bake depends on (not tests, caches or build output)?

    ``path`` may be absolute (watcher events), so only the named ignored
    directories apply here: a worktree itself lives under ``.claude/``.
    """
    p = Path(path)
    if p.suffix != ".py":
        return False
    if set(p.parts) & _IGNORED_DIRS:
        return False
    # Skip test files — they don't affect runtime geometry.
    return not (p.name.startswith("test_") or "tests" in p.parts)


def _is_source_change(_change: Change, path: str) -> bool:
    return is_source(Path(path))


class WatchBroadcaster:
    """Owns the WebSocket client set and the file-watching task."""

    def __init__(self, root: Path, rebake: Callable[[], None]) -> None:
        self._root = root
        self._rebake = rebake
        self._clients: set[WebSocket] = set()
        self._task: asyncio.Task | None = None
        self._bake_lock = asyncio.Lock()
        self._stop = asyncio.Event()

    async def start(self) -> None:
        self._stop.clear()
        self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None

    async def connect(self, ws: WebSocket) -> None:
        self._clients.add(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        self._clients.discard(ws)

    async def _broadcast(self, msg: dict) -> None:
        stale = []
        for ws in list(self._clients):
            try:
                await ws.send_json(msg)
            # a socket that can't take a message is dropped, whatever the transport raised
            # (WebSocketDisconnect, RuntimeError after a close, an OSError): one bad client
            # mustn't stop the broadcast to the others
            except Exception:  # noqa: BLE001
                stale.append(ws)
        for ws in stale:
            self._clients.discard(ws)

    async def _run(self) -> None:
        try:
            async for changes in awatch(
                self._root, watch_filter=_is_source_change, stop_event=self._stop
            ):
                await self._on_changes(changes)
        except asyncio.CancelledError:
            pass

    async def _on_changes(self, changes: set) -> None:
        if not changes:
            return
        paths = sorted({Path(p).name for _, p in changes})
        log.info("change detected: %s", ", ".join(paths))
        async with self._bake_lock:
            await self._broadcast({"type": "rebaking", "files": paths})
            try:
                await asyncio.to_thread(self._rebake)
            # the engine's failures aren't one class: whatever the bake raised goes to the
            # viewer as an error, and the watcher keeps watching for the fix
            except Exception as e:  # noqa: BLE001
                log.error("bake failed: %s", e)
                await self._broadcast({"type": "error", "msg": str(e)})
                return
            await self._broadcast({"type": "reload"})

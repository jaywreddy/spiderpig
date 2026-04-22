"""File watcher that re-bakes viewer data and pushes reload events.

On any ``*.py`` change under the repo root (skipping tests, caches, and
build output), runs the bake in a worker thread and broadcasts a JSON
``{"type": "reload"}`` to every connected ``/ws`` client. Clients receive
``{"type": "rebaking"}`` first so the UI can show a spinner.
"""

from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Callable
from pathlib import Path

from fastapi import WebSocket
from watchfiles import Change, awatch

_IGNORED_DIRS = {".venv", ".git", "__pycache__", ".ruff_cache", ".pytest_cache", "build", "data"}


def _is_source_change(_change: Change, path: str) -> bool:
    p = Path(path)
    if p.suffix != ".py":
        return False
    parts = set(p.parts)
    if parts & _IGNORED_DIRS:
        return False
    # Skip test files — they don't affect runtime geometry.
    return not (p.name.startswith("test_") or "tests" in p.parts)


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
            except Exception:
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
        print(f"[watcher] change detected: {', '.join(paths)}")
        async with self._bake_lock:
            await self._broadcast({"type": "rebaking", "files": paths})
            try:
                await asyncio.to_thread(self._rebake)
            except Exception as e:
                print(f"[watcher] bake failed: {e}")
                await self._broadcast({"type": "error", "msg": str(e)})
                return
            await self._broadcast({"type": "reload"})

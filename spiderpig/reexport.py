"""Packages split from one module keep its namespace: reads and writes.

``stack``, ``api`` and ``construction.crank`` were single modules; W5 split each into a
package whose ``__init__`` re-exports every name at its old path. Reading ``stack.X`` works
by the re-export alone. *Writing* it (``monkeypatch.setattr(stack, "MAX_SECONDS", ...)``,
``api.fabricate_at = ...``) would only rebind the package's copy, while the code that reads
``X`` as a global lives in a submodule. :func:`forward_writes` makes the package forward
such a write to every one of its submodules that binds the same object, so a patch reaches
the code that reads it, as it did before the split (:mod:`spiderpig.keys` follows a write
to a package's re-exported name to the submodules it came from, the same rule).
"""

from __future__ import annotations

import sys
import types

_MISSING = object()


class ForwardingPackage(types.ModuleType):
    """A package whose attribute writes reach its submodules' bindings of the same object."""

    def __setattr__(self, name: str, value) -> None:
        old = self.__dict__.get(name, _MISSING)
        if old is not _MISSING and not (name.startswith("__") and name.endswith("__")):
            prefix = self.__name__ + "."
            for modname, mod in list(sys.modules.items()):
                if mod is None or not modname.startswith(prefix):
                    continue
                d = mod.__dict__
                if d.get(name, _MISSING) is old and not isinstance(old, types.ModuleType):
                    d[name] = value
        super().__setattr__(name, value)


def forward_writes(name: str) -> None:
    """Make the package ``name`` (call it at the end of its ``__init__``) forward writes."""
    sys.modules[name].__class__ = ForwardingPackage

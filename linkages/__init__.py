"""Walking-linkage definitions. Importing this package registers them all.

Each module here defines one family with :class:`linkage.Linkage` (and its
published variants with :meth:`linkage.Linkage.variant`) and registers it.
Klann registers first: it is the default. Every version is registered, even
one the current constructions can't build: the pipeline says why (e.g. a link
that sweeps across the crank axis needs a crank overhung from the servo side).
"""

import importlib
import pkgutil

from . import klann  # noqa: F401 - the default registers first

for _mod in sorted(m.name for m in pkgutil.iter_modules(__path__)):
    importlib.import_module(f"{__name__}.{_mod}")

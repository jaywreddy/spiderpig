"""Write a design's files: ``spiderpig export <design or build options> --formats ...``.

    spiderpig export 1a2b3c4d5e6f7a8b                       # the spec's outputs, into the store
    spiderpig export 1a2b3c4d5e6f7a8b --formats glb mjcf --out out/heel
    spiderpig export --linkage trotbot_heel --formats step stl print dxf bom glb mjcf

The API's :func:`spiderpig.api.export` on the command line: a stored design by its id,
or the design the build options describe (``--linkage``, ``--module``, ``--pin``, ...:
the same options as ``spiderpig build``; a mechanism is its one module and one side),
resolved into the store first as ``spiderpig view`` does. ``--formats`` is any of
``step``, ``stl``, ``print``, ``dxf``, ``bom``, ``glb``, ``mjcf`` (default: the spec's
``outputs``; a CLI design's are step, stl, print, dxf and bom); the files go into
``--out`` (default: the design's ``exports/`` in the store) with ``manifest.json``. An
export the store already holds for these formats and folder is returned as is
(``--force`` rewrites). The MJCF (and its ``.json`` beside it) is what ``spiderpig sim
--mjcf`` runs.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    from spiderpig.config import add_build_args, add_design_args
    from spiderpig.spec import OUTPUTS

    ap = argparse.ArgumentParser(
        prog="spiderpig export",
        description="write a stored design's files (or those of the design the build "
                    "options describe): STEP, STL, print STLs, DXF sheets, BOM, glb, MJCF")
    ap.add_argument("design", nargs="?", metavar="DESIGN",
                    help="a design id (16 hex digits, from resolve); or give the build "
                         "options below and the design is resolved into the store first")
    add_design_args(ap)
    add_build_args(ap)
    ap.add_argument("--side-only", action="store_true",
                    help="with the build options: one side (no second side, no chassis)")
    ap.set_defaults(linkage=None, module=None, servo=None, pillar=None, pin=None, crank=None,
                    sheet=None)
    ap.add_argument("--formats", nargs="+", choices=OUTPUTS, metavar="FORMAT",
                    help=f"what to write: {', '.join(OUTPUTS)} (default: the spec's outputs)")
    ap.add_argument("--out", type=Path, default=None,
                    help="output directory (default: the design's exports/ in the store)")
    ap.add_argument("--store", metavar="PATH",
                    help="the design store (default: $SPIDERPIG_STORE, else ./.spiderpig)")
    ap.add_argument("--force", action="store_true", help="rewrite an export the store holds")
    args = ap.parse_args(argv)
    from spiderpig import api
    from spiderpig.store import Store
    from spiderpig.view import load_design, resolve_args

    store = Store.of(args.store) if args.store else Store.default()
    try:
        if args.design:
            design = load_design(args.design, store)
        else:
            design = resolve_args(args, store)
            if design is None:
                ap.error("a design id or the build options (--linkage, --module, --pin, ...) "
                         "are required")
            print(f"resolved the build options into {store.root.resolve()} as design "
                  f"{design.id} (spiderpig export {design.id} writes it again)", file=sys.stderr)
    except (KeyError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    for w in design.warnings:
        print(f"warning: {w}", file=sys.stderr)
    rep = api.export(design, args.formats, args.out, force=args.force)
    for w in rep.warnings:
        print(f"warning: {w}", file=sys.stderr)
    if not rep.ok:
        for f in rep.failures:
            print(f"error: {design.id} can't be exported: {f.stage} ({f.code}): {f.message}",
                  file=sys.stderr)
        return 1
    print(f"wrote {len(rep.files)} files ({', '.join(rep.formats)}) to {rep.out_dir}:")
    for f in rep.files:
        print(f"  {Path(f).relative_to(rep.out_dir)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

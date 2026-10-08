"""The assembly order as the docs quote it: the section of ``docs/ARCHITECTURE.md`` between
:data:`BEGIN` and :data:`END`, rendered from the default robot's steps
(:func:`construction.assembly.prose`: the robot's order and every construction's hook).

    python -m spiderpig.guide.prose            # say whether the section is current (exit 1)
    python -m spiderpig.guide.prose --write    # write it

``tests/test_guide.py`` fails while the section is stale, as the doc check does for names.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

DOC = Path(__file__).resolve().parents[2] / "docs" / "ARCHITECTURE.md"
BEGIN = ("<!-- assembly-order: generated from the default robot by "
         "`python -m spiderpig.guide.prose --write`; don't edit by hand -->")
END = "<!-- assembly-order: end -->"


def section(mech, design) -> str:
    """The generated text: the order as numbered paragraphs."""
    from spiderpig.construction.assembly import assembly_steps, prose

    return "\n\n".join(prose(assembly_steps(mech, design)))


def current(doc: str) -> str | None:
    """The section the doc holds now (None: no markers)."""
    if BEGIN not in doc or END not in doc:
        return None
    return doc.split(BEGIN, 1)[1].split(END, 1)[0].strip("\n")


def replaced(doc: str, text: str) -> str:
    head, rest = doc.split(BEGIN, 1)
    return f"{head}{BEGIN}\n{text}\n{END}{rest.split(END, 1)[1]}"


def main(argv=None) -> int:
    from spiderpig import fabcache
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import design_side, fabricate, template_for
    from spiderpig.stages.planning import plan_config
    from spiderpig.store import Store

    p = argparse.ArgumentParser(prog="python -m spiderpig.guide.prose", description=__doc__)
    p.add_argument("--write", action="store_true", help="write the section")
    p.add_argument("--store", type=Path, default=None)
    args = p.parse_args(argv)
    config = BuildConfig()
    store = Store.of(args.store) if args.store else Store.default()
    tmpl = template_for(config)
    plan_config(config, store)
    design = design_side(tmpl, config)
    with fabcache.serving(store):
        mech = fabricate(tmpl, config, 1.0)
    text = section(mech, design)
    doc = DOC.read_text()
    if current(doc) is None:
        print(f"{DOC}: no generated section (its markers)", file=sys.stderr)
        return 2
    if current(doc) == text:
        print(f"{DOC}: the assembly order is current")
        return 0
    if args.write:
        DOC.write_text(replaced(doc, text))
        print(f"wrote the assembly order into {DOC}")
        return 0
    print(f"{DOC}: the assembly order is stale (--write)", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

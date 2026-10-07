"""Print the engine version (``spiderpig.design.engine_version``): the key of the stores,
the test cache and CI's cache of it. ``python -m spiderpig.tools.engine_version``
(``mise run engine-version``). Lives under ``tools/`` so it is outside the hash it prints."""

from __future__ import annotations


def main(argv=None) -> int:
    from spiderpig.design import engine_version

    print(engine_version())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

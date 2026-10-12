"""``spiderpig cad``: the bought parts' vendor models (:data:`hardware.cad.PART_CAD`).

Vendor models are never committed: each lives in the CAD cache (``$SPIDERPIG_CAD_CACHE``,
default ``~/.cache/spiderpig/cad``) and a build uses it when it is there and matches the
catalog's size (otherwise the part is drawn from its catalog dims). This command fills
the cache::

    spiderpig cad list                       # every part model: cached or not, from where
    spiderpig cad import bearing_mr84zz ~/Downloads/57155K....STEP
    spiderpig cad login                      # McMaster-Carr: log in once in a browser window
    spiderpig cad fetch [KEY ...]            # ... then fetch the missing models with it
    spiderpig cad api-fetch KEY --part-number PN   # McMaster's Product Information API

**McMaster-Carr** shows its catalog only to a logged-in browser (a scripted session gets a
login wall), and its pages are the user's to use under its terms. So the fetch is the
user's: ``login`` opens a real browser window (Playwright's Chromium, headed: it needs a
display) on mcmaster.com, the user logs in with their own account, and the session is saved
(``$SPIDERPIG_MCMASTER_STATE``, default ``~/.config/spiderpig/mcmaster-state.json``);
``fetch`` then opens each missing part's listing with that session, takes the part number
(the catalog offer's, else the listing's first), opens its product page and downloads
the "3-D STEP" model, which goes into the cache through :func:`hardware.cad.import_file`.
A login wall stops it with the advice to log in again; a page it can't read says so and
how to ``import`` the file by hand. ``api-fetch`` uses McMaster's Product Information API
instead (an approved customer's client certificate and login: ``$SPIDERPIG_MCMASTER_CERT``
(a .pfx), ``_CERT_PASSWORD``, ``_USER``, ``_PASSWORD``): subscribe, read the product's
``"3-D STEP"`` link, download it.

Neither McMaster path was exercised against the live site from the development box (its
login wall; no API approval): the import, the cache and the use of a model are tested.
Each fetched model's part number and hash are printed so they can be pinned in the
catalog (:mod:`hardware.bearing_catalog`).
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import sys
import tempfile
import time
from pathlib import Path

STATE_ENV = "SPIDERPIG_MCMASTER_STATE"
PART_NUMBER = re.compile(r"\b(\d{4,5}[A-Z]\d{1,4})\b")
LOGIN_WALL = "please log in"


def state_path() -> Path:
    env = os.environ.get(STATE_ENV)
    return Path(env).expanduser() if env else Path.home() / ".config" / "spiderpig" / \
        "mcmaster-state.json"


def _refs() -> dict:
    from spiderpig.hardware import cad
    from spiderpig.hardware.catalog import _load

    _load()
    return dict(sorted(cad.PART_CAD.items()))


def _listing(key: str) -> tuple[str, str | None]:
    """The McMaster offer's page and part number for catalog item ``key``."""
    from spiderpig.hardware.catalog import get

    for o in get(key).offers:
        if "mcmaster" in o.vendor.lower():
            return o.url, o.sku
    raise SystemExit(f"{key}: no McMaster-Carr offer in the catalog")


def _sidecar(path: Path) -> Path:
    return path.with_suffix(path.suffix + ".json")


def cmd_list(_args) -> int:
    from spiderpig.hardware import cad

    for key, ref in _refs().items():
        path = cad.fetch(ref, allow_download=False)
        what = "parametric (not in the cache)"
        if path is not None:
            meta = {}
            with contextlib.suppress(OSError, ValueError):
                meta = json.loads(_sidecar(path).read_text())
            pn = meta.get("part_number") or ref.part_number or "?"
            what = f"cached {cad.sha256_file(path)[:16]} (part {pn}) {path}"
        print(f"{key:20} {ref.vendor or '-':10} {what}")
    return 0


def _import(key: str, src: Path, part_number: str = "") -> Path:
    from spiderpig.hardware import cad

    ref = _refs().get(key)
    if ref is None:
        raise SystemExit(f"no part model registered for {key!r}; have {sorted(_refs())}")
    path = cad.import_file(ref, src)
    _sidecar(path).write_text(json.dumps({
        "key": key, "part_number": part_number or ref.part_number,
        "sha256": cad.sha256_file(path), "source": str(src),
        "imported": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))
    print(f"{key}: {path} (sha256 {cad.sha256_file(path)}"
          + (f", McMaster {part_number}" if part_number else "") + ")")
    return path


def cmd_import(args) -> int:
    _import(args.key, Path(args.file).expanduser(), args.part_number or "")
    return 0


def cmd_login(_args) -> int:
    """A headed browser on mcmaster.com: the user logs in; the session is saved."""
    from playwright.sync_api import sync_playwright

    out = state_path()
    out.parent.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=False)
        ctx = browser.new_context()
        page = ctx.new_page()
        page.goto("https://www.mcmaster.com/")
        input("Log in to McMaster-Carr in the browser window, then press Enter here... ")
        ctx.storage_state(path=str(out))
        browser.close()
    os.chmod(out, 0o600)
    print(f"session saved to {out}")
    return 0


def _fetch_one(page, key: str) -> Path | None:
    url, pn = _listing(key)
    if not pn:
        page.goto(url, wait_until="domcontentloaded")
        page.wait_for_timeout(3000)
        if LOGIN_WALL in page.inner_text("body").lower():
            raise SystemExit("McMaster asks for a login: run `spiderpig cad login` again")
        found = PART_NUMBER.findall(page.inner_text("body"))
        if not found:
            print(f"{key}: no part number on {url}; download the STEP by hand and "
                  f"`spiderpig cad import {key} FILE --part-number PN`")
            return None
        pn = found[0]
    page.goto(f"https://www.mcmaster.com/{pn}/", wait_until="domcontentloaded")
    page.wait_for_timeout(3000)
    if LOGIN_WALL in page.inner_text("body").lower():
        raise SystemExit("McMaster asks for a login: run `spiderpig cad login` again")
    try:
        page.get_by_text("CAD", exact=True).first.click()
        page.get_by_text("3-D STEP").first.click()
        with page.expect_download(timeout=30000) as dl:
            page.get_by_role("button", name=re.compile("download", re.I)).first.click()
        tmp = Path(tempfile.mkdtemp(prefix="spiderpig-cad-")) / dl.value.suggested_filename
        dl.value.save_as(str(tmp))
    # the page's layout isn't ours: whatever it does, say how to finish by hand
    except Exception as e:  # noqa: BLE001
        print(f"{key}: couldn't download the STEP from https://www.mcmaster.com/{pn}/ ({e}); "
              f"download it by hand and `spiderpig cad import {key} FILE --part-number {pn}`")
        return None
    return _import(key, tmp, pn)


def cmd_fetch(args) -> int:
    from playwright.sync_api import sync_playwright

    from spiderpig.hardware import cad

    state = state_path()
    if not state.is_file():
        print("no McMaster session: run `spiderpig cad login` first", file=sys.stderr)
        return 2
    keys = args.keys or [k for k, r in _refs().items()
                         if r.vendor == "mcmaster" and cad.fetch(r, allow_download=False) is None]
    if not keys:
        print("every McMaster part model is in the cache")
        return 0
    missing = 0
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_context(storage_state=str(state), accept_downloads=True).new_page()
        for key in keys:
            missing += _fetch_one(page, key) is None
        browser.close()
    return 1 if missing else 0


def _api_session():
    """An ``ssl`` context with the client certificate, and the bearer token (login)."""
    import ssl
    import urllib.request

    from cryptography.hazmat.primitives.serialization import (
        Encoding,
        NoEncryption,
        PrivateFormat,
        pkcs12,
    )

    env = {k: os.environ.get(f"SPIDERPIG_MCMASTER_{k}") for k in
           ("CERT", "CERT_PASSWORD", "USER", "PASSWORD")}
    if not all(env.values()):
        raise SystemExit("api-fetch needs SPIDERPIG_MCMASTER_CERT (.pfx), _CERT_PASSWORD, "
                         "_USER and _PASSWORD (McMaster's Product Information API: an "
                         "approved customer's; eCommerce@mcmaster.com)")
    key, cert, _ = pkcs12.load_key_and_certificates(
        Path(str(env["CERT"])).expanduser().read_bytes(), str(env["CERT_PASSWORD"]).encode())
    if key is None or cert is None:
        raise SystemExit("the .pfx holds no key and certificate")
    fd, pem = tempfile.mkstemp(suffix=".pem")       # (mkstemp: created 0600)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(cert.public_bytes(Encoding.PEM))
            f.write(key.private_bytes(Encoding.PEM, PrivateFormat.PKCS8, NoEncryption()))
        ctx = ssl.create_default_context()
        ctx.load_cert_chain(pem)
    finally:
        os.unlink(pem)
    req = urllib.request.Request(
        "https://api.mcmaster.com/v1/login", method="POST",
        data=json.dumps({"UserName": env["USER"], "Password": env["PASSWORD"]}).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, context=ctx, timeout=60) as r:
        token = json.load(r)["AuthToken"]
    return ctx, token


def cmd_api_fetch(args) -> int:
    import urllib.request

    ctx, token = _api_session()
    auth = {"Authorization": f"Bearer {token}", "Content-Type": "application/json"}

    def call(method: str, path: str, body: dict | None = None):
        req = urllib.request.Request(f"https://api.mcmaster.com{path}", method=method,
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers=auth)
        return urllib.request.urlopen(req, context=ctx, timeout=120)

    pn = args.part_number
    with call("PUT", "/v1/products", {"URL": f"https://mcmaster.com/{pn}"}) as r:
        product = json.load(r)
    links = {link.get("Key"): link.get("Value") for link in product.get("Links", [])}
    step = links.get("3-D STEP")
    if not step:
        print(f"{pn}: no 3-D STEP link ({sorted(links)})", file=sys.stderr)
        return 1
    tmp = Path(tempfile.mkdtemp(prefix="spiderpig-cad-")) / Path(step).name
    with call("GET", step) as r:
        tmp.write_bytes(r.read())
    _import(args.key, tmp, pn)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="spiderpig cad",
                                 description="bought parts' vendor models: list, import, "
                                             "fetch (the CAD cache; never committed)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="every registered part model and whether it is cached")
    i = sub.add_parser("import", help="put a downloaded model into the cache")
    i.add_argument("key", help="the catalog item (spiderpig cad list)")
    i.add_argument("file", help="the STEP file")
    i.add_argument("--part-number", default="", help="the vendor's part number (recorded)")
    sub.add_parser("login", help="McMaster-Carr: log in once in a browser window")
    f = sub.add_parser("fetch", help="McMaster-Carr: fetch the missing models with the "
                                     "saved session")
    f.add_argument("keys", nargs="*", help="catalog items (default: every missing one)")
    a = sub.add_parser("api-fetch", help="McMaster-Carr's Product Information API")
    a.add_argument("key")
    a.add_argument("--part-number", required=True)
    args = ap.parse_args(argv)
    return {"list": cmd_list, "import": cmd_import, "login": cmd_login, "fetch": cmd_fetch,
            "api-fetch": cmd_api_fetch}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())

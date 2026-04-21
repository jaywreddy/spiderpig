# Cross-platform task runner. All recipes dispatch to cli.py so Windows
# PowerShell, macOS zsh, and Linux bash behave identically.

set windows-shell := ["powershell.exe", "-NoLogo", "-Command"]

default:
    @just --list

# STEP/STL/DXF → build/
build *ARGS:
    uv run python cli.py build {{ARGS}}

# Bake viewer meshes + frames.json → viewer/data/
bake *ARGS:
    uv run python cli.py bake {{ARGS}}

# Dev server with live reload → http://localhost:8000
view *ARGS:
    uv run python cli.py view {{ARGS}}

test *ARGS:
    uv run python cli.py test {{ARGS}}

lint:
    uv run ruff check .

clean:
    uv run python cli.py clean

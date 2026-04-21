default:
    @just --list

# STEP/STL/DXF → build/
build:
    uv run python main.py

# Bake viewer meshes + frames.json → viewer/data/
bake:
    uv run python viewer/bake.py

# Bake, serve, and open the viewer in a browser
view: bake
    @echo "→ http://localhost:8000"
    @(sleep 0.3 && (command -v open >/dev/null && open http://localhost:8000 \
        || command -v xdg-open >/dev/null && xdg-open http://localhost:8000 \
        || true)) &
    cd viewer && python -m http.server 8000

test:
    uv run pytest -v

lint:
    uv run ruff check .

clean:
    rm -rf build/ viewer/data/

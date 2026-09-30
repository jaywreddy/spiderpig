#!/bin/bash
# SessionStart hook for Claude Code on the web: install what the tests and
# linters need (mise isn't in the container, so uv and npm are called directly).
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

cd "${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"

uv sync                                       # .venv: runtime + dev group (ruff, pytest, playwright)
(cd viewer && npm install && npm run build)   # viewer deps; dist/ for single-port and e2e runs

if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  echo "export PATH=\"$PWD/.venv/bin:\$PATH\"" >> "$CLAUDE_ENV_FILE"
fi

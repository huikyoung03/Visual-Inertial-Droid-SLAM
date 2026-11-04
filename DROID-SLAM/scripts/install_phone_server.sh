#!/usr/bin/env bash
set -euo pipefail

# Creates the lightweight FastAPI environment. DROID inference remains in droid_env.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${1:-${REPO_ROOT}/.venv-phone}"

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install -r "${REPO_ROOT}/requirements-phone-server.txt"
echo "[DONE] Phone server venv: ${VENV_DIR}"

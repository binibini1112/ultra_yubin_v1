#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

if [ -x "${ROOT}/.venv/bin/python3" ]; then
  PY="${ROOT}/.venv/bin/python3"
else
  PY="${PYTHON:-python3}"
fi

exec "${PY}" jetson/jetson_node.py "$@"

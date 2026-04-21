#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <step_file> [--port N] [--x-rotate DEG]" >&2
  exit 1
fi

cd "$(dirname "$0")"
exec python scripts/find_bottom_center.py "$@" --ui

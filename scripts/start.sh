#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
source "$PROJECT_ROOT/scripts/env.sh"
export PYTHONPATH="$PROJECT_ROOT/rm_flatness_nav:$PYTHONPATH"
exec python -u "$PROJECT_ROOT/scripts/control_panel.py" "$@"

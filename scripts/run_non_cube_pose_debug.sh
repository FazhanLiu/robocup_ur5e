#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SCENE_WIDE_NON_CUBE_RANKING=true
export DEBUG_NON_CUBE_ONLY=true
export EXECUTE_AFTER_SCENE_RANKING=false

exec "$SCRIPT_DIR/run_hybrid_cleanup.sh" "$@"

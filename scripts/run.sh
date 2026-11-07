#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source .venv/bin/activate
# First argument selects the profile; everything after it is passed through to
# the profile's entry point (e.g. `run.sh sim curve --dashboard`).
profile="${1:-prototype}"
if [ $# -gt 0 ]; then shift; fi
MAV_PROFILE="$profile" PYTHONPATH=src python -m mav "$@"

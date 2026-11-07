#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source .venv/bin/activate
MAV_PROFILE="${1:-prototype}" PYTHONPATH=src python -m mav

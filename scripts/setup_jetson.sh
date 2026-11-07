#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# system-site-packages: agar cv2/tensorrt bawaan JetPack terlihat di venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements-jetson.txt

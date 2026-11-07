"""Cross-profile web dashboard (sim, prototype, final).

Serves the frontend in ``static/`` (its <head> comment is the wire contract),
pushes telemetry over a WebSocket at ~10 Hz, streams the camera as MJPEG and
exposes the REST control/config API. Standard library only - no new dependency,
same rule the sim dashboard follows.

Safety (competition rule 5.6: no communication with the robot during a run):
the ``final`` profile only starts this server when explicitly asked for, and
the pipeline shuts it down the moment a run starts. The dashboard estop is a
software stop, never a substitute for the physical e-stop.
"""
from mav.dashboard.bridge import Control, FakeBridge, NullBridge, parse_control
from mav.dashboard.server import serve_in_thread, stop
from mav.dashboard.state import DashboardState

__all__ = [
    "Control",
    "DashboardState",
    "FakeBridge",
    "NullBridge",
    "parse_control",
    "serve_in_thread",
    "stop",
]

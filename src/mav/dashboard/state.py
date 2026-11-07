"""JSON-serialisable dashboard snapshot, shared by every profile.

Pure data + conversion: no I/O and no numpy in the output, so this module is
testable anywhere and the browser contract (static/index.html) lives in
exactly one place on each side.

The frontend expects the robot state in UPPERCASE ("BOOT", "WAIT_START",
"RUN", "STOPPED"); ``mav.control.state_machine`` uses lowercase enum values,
so whoever builds a DashboardState maps them (see ``label_of``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

#: Robot-state labels the frontend understands, in lifecycle order.
STATE_LABELS = ("BOOT", "WAIT_START", "RUN", "STOPPED")

_LABELS = {label.lower(): label for label in STATE_LABELS}


def label_of(state: object) -> str:
    """Map a state-machine value (or any string) to the frontend label.

    ``State.RUN`` -> "RUN", "wait_start" -> "WAIT_START". Unknown values are
    upper-cased as-is so a new state still shows up in the UI.
    """
    text = getattr(state, "value", state)
    return _LABELS.get(str(text).lower(), str(text).upper())


def _plain(obj: Any) -> Any:
    """Recursively convert to JSON-safe primitives.

    numpy scalars/arrays become int/float/list; NaN and +-inf become None
    (JSON has no representation for them and they only mean "no reading").
    """
    if obj is None or isinstance(obj, (str, bool)):
        return obj
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        f = float(obj)
        return f if math.isfinite(f) else None
    if isinstance(obj, np.ndarray):
        return [_plain(v) for v in obj.tolist()]
    if isinstance(obj, dict):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_plain(v) for v in obj]
    return obj


@dataclass(frozen=True)
class DashboardState:
    """One tick of a run, ready to ship to a browser.

    The pipeline owns these fields. The server adds ``recording`` (from the
    recorder) and ``log`` (from its ring buffer) when composing the WS
    message, so the run loop never needs to know about them.

    out: hardware values the mapper produced (e.g. {"left_mps": ..., 
        "right_mps": ...} for the prototype, {"steer_rad": ..., "throttle":
        ...} for final) - shown as out.<key> rows in the UI.
    extra: any additional debug rows the profile wants to show.
    """

    state: str = "BOOT"                # BOOT | WAIT_START | RUN | STOPPED
    profile: str = "prototype"         # sim | prototype | final
    checkpoints: int = 0
    violations: int = 0                # resumes after an e-stop (referee proxy)
    fps: float | None = None           # wall-clock loop rate
    latency_ms: float | None = None    # per-tick processing time
    temp_c: float | None = None        # Jetson SoC temperature (None on laptop)
    power_mode: str | None = None      # nvpmodel mode (None on laptop)
    cmd: dict | None = None            # {"speed": m/s, "curvature": 1/m}
    out: dict = field(default_factory=dict)
    extra: dict = field(default_factory=dict)
    # Filled by the run loop from the recorder / its own log lines. The browser
    # APPENDS log lines, so publish only the new ones each tick.
    recording: dict = field(default_factory=lambda: {"on": False, "frames": 0})
    log: list = field(default_factory=list)

    def to_dict(self) -> dict:
        """JSON-safe dict. ``json.dumps(state.to_dict())`` must always work."""
        return _plain(
            {
                "state": self.state,
                "profile": self.profile,
                "checkpoints": self.checkpoints,
                "violations": self.violations,
                "fps": self.fps,
                "latency_ms": self.latency_ms,
                "temp_c": self.temp_c,
                "power_mode": self.power_mode,
                "cmd": self.cmd,
                "out": self.out,
                "extra": self.extra,
                "recording": self.recording,
                "log": self.log,
            }
        )

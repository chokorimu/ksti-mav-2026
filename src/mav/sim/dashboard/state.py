"""JSON-serialisable snapshot of the simulator, for the web dashboard.

Pure data + conversion, no I/O and no numpy in the output, so this module is
testable anywhere and the browser contract lives in exactly one place.

Conventions match the rest of the sim: X right, Y forward, heading theta
left-positive, curvature > 0 = left. Offsets are +right.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class Pose:
    """Vehicle pose in world coordinates (metres, radians)."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


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
class SimState:
    """One tick of the simulator, ready to ship to a browser.

    measurement: offset_m / curvature_inv_m / confidence / n_points, or None
        when the lane was not detected this tick.
    debug: detector internals (sigma, threshold, mask fractions, polarity,
        band_points [[row, x, pixels]], fit {a, b, c}, resid, lookahead_row,
        lookahead_x) — everything the camera-panel overlay needs.
    """

    step: int = 0
    t: float = 0.0                    # sim time, seconds
    scenario: str = ""
    robot_state: str = "boot"         # boot | wait_start | run | stopped
    checkpoints: int = 0
    pose: Pose = field(default_factory=Pose)
    lookahead_m: float = 0.0          # forward distance the offset is measured at
    true_offset_m: float | None = None  # ground truth at lookahead_m (sim only!)
    measurement: dict | None = None
    debug: dict = field(default_factory=dict)
    command: dict = field(default_factory=dict)   # speed, curvature
    metrics: dict = field(default_factory=dict)   # run-level counters
    fps: float = 0.0                  # wall-clock ticks per second

    def to_dict(self) -> dict:
        """JSON-safe dict. `json.dumps(state.to_dict())` must always work."""
        return _plain(
            {
                "step": self.step,
                "t": self.t,
                "scenario": self.scenario,
                "robot_state": self.robot_state,
                "checkpoints": self.checkpoints,
                "pose": {"x": self.pose.x, "y": self.pose.y, "theta": self.pose.theta},
                "lookahead_m": self.lookahead_m,
                "true_offset_m": self.true_offset_m,
                "measurement": self.measurement,
                "debug": self.debug,
                "command": self.command,
                "metrics": self.metrics,
                "fps": self.fps,
            }
        )

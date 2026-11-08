"""Shared types. Everything above the hardware layer speaks `Command`."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Command:
    """Vehicle-agnostic motion command.

    speed:     m/s, forward positive.
    curvature: 1/m, left turn positive, 0 = straight (curvature = 1 / turning radius).
    """

    speed: float = 0.0
    curvature: float = 0.0


STOP = Command(0.0, 0.0)


@dataclass(frozen=True)
class VehicleParams:
    drive: str            # "differential" (prototype) or "ackermann" (final)
    wheel_base_m: float
    track_width_m: float
    max_steer_rad: float
    max_speed_mps: float


_REQUIRED = ("drive", "wheel_base_mm", "track_width_mm", "max_steer_deg", "max_speed_mps")
# Only the values the chosen drive actually uses must be measured.
_NEEDED = {
    "differential": ("track_width_mm", "max_speed_mps"),
    "ackermann": ("wheel_base_mm", "max_steer_deg", "max_speed_mps"),
}


def load_vehicle_params(path: str | Path) -> VehicleParams:
    """Load configs/vehicle.yaml. Refuses to run with unmeasured (null) values that the drive needs."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    drive = data.get("drive")
    if drive not in _NEEDED:
        missing = [k for k in _REQUIRED if data.get(k) is None]
        if missing:
            raise ValueError(f"{path}: measure the robot and fill in: {', '.join(missing)}")
        raise ValueError(f"{path}: drive must be 'differential' or 'ackermann'")
    missing = [k for k in _NEEDED[drive] if data.get(k) is None]
    if missing:
        raise ValueError(f"{path}: measure the robot and fill in: {', '.join(missing)}")

    def val(key: str) -> float:
        return float(data.get(key) or 0.0)  # unused-by-this-drive values stay 0

    return VehicleParams(
        drive=drive,
        wheel_base_m=val("wheel_base_mm") / 1000.0,
        track_width_m=val("track_width_mm") / 1000.0,
        max_steer_rad=math.radians(val("max_steer_deg")),
        max_speed_mps=val("max_speed_mps"),
    )

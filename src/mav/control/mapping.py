"""Map a vehicle-agnostic Command to hardware-level values.

prototype (4 independently driven wheels) -> DifferentialMapper -> left/right wheel speeds
deploy    (1 steering servo + 1 rear motor) -> AckermannMapper   -> steering angle + speed
Conventions: curvature > 0 is a left turn; steering angle > 0 is left.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from mav.types import Command, VehicleParams


@dataclass(frozen=True)
class WheelSpeeds:
    left: float   # m/s
    right: float  # m/s


@dataclass(frozen=True)
class SteerDrive:
    steer_rad: float
    speed: float      # m/s
    saturated: bool   # True if the requested curvature exceeded the steering limit


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class DifferentialMapper:
    def __init__(self, p: VehicleParams) -> None:
        self._half_track = p.track_width_m / 2.0
        self._vmax = p.max_speed_mps

    def map(self, cmd: Command) -> WheelSpeeds:
        v = _clamp(cmd.speed, -self._vmax, self._vmax)
        omega = v * cmd.curvature  # yaw rate, rad/s
        left = v - omega * self._half_track
        right = v + omega * self._half_track
        peak = max(abs(left), abs(right))
        if peak > self._vmax:  # keep the turn radius, slow both wheels
            scale = self._vmax / peak
            left, right = left * scale, right * scale
        return WheelSpeeds(left, right)


class AckermannMapper:
    def __init__(self, p: VehicleParams) -> None:
        self._wheel_base = p.wheel_base_m
        self._max_steer = p.max_steer_rad
        self._vmax = p.max_speed_mps

    def map(self, cmd: Command) -> SteerDrive:
        v = _clamp(cmd.speed, -self._vmax, self._vmax)
        steer = math.atan(self._wheel_base * cmd.curvature)
        clamped = _clamp(steer, -self._max_steer, self._max_steer)
        return SteerDrive(clamped, v, saturated=clamped != steer)

"""Minimal lane-following policy: LaneMeasurement -> Command(speed, curvature).

Pure-pursuit style, deliberately conservative (competition scoring punishes
leaving the track far more than it rewards speed):
  - steering: kappa = -2 * offset / L^2, clamped to the tightest turn the track
    allows (rules: minimum turning radius 1.5 m -> |kappa| <= 1/1.5)
  - speed: scales down with the commanded curvature AND the curvature measured
    ahead, and with detection confidence
  - lost lane (no measurement / low confidence / implausible offset): STOP by
    default. Safe by default; `on_lost="hold"` keeps the last command instead.

This is a v1 placeholder policy: it has no heading term and no checkpoint
logic. Tune on recordings before trusting it on the track.
"""
from __future__ import annotations

from dataclasses import dataclass

from mav.perception.lane import LaneMeasurement
from mav.types import STOP, Command

# Rules 6: minimum turning radius (outer) 1.5 m +- 10%.
_MIN_TURN_RADIUS_M = 1.5


@dataclass(frozen=True)
class LanePolicyConfig:
    lookahead_m: float = 1.0        # pure-pursuit lookahead distance
    max_offset_m: float = 0.35     # beyond this the lane is considered lost
    v_max: float = 1.0              # m/s, start low and raise after testing
    v_min: float = 0.15             # m/s floor while still tracking
    curv_slow: float = 1.0          # |kappa| at which speed drops to v_min
    conf_min: float = 0.3           # below this the measurement is not trusted
    on_lost: str = "stop"           # "stop" | "hold"


class LanePolicy:
    def __init__(self, cfg: LanePolicyConfig | None = None) -> None:
        self.cfg = cfg or LanePolicyConfig()

    def command(self, meas: LaneMeasurement | None, last: Command = STOP) -> Command:
        cfg = self.cfg
        if (
            meas is None
            or meas.confidence < cfg.conf_min
            or abs(meas.offset_m) > cfg.max_offset_m
        ):
            return STOP if cfg.on_lost == "stop" else last

        kappa = -2.0 * meas.offset_m / (cfg.lookahead_m ** 2)
        kappa = max(-1.0 / _MIN_TURN_RADIUS_M, min(1.0 / _MIN_TURN_RADIUS_M, kappa))

        severity = max(abs(kappa), abs(meas.curvature_inv_m))
        speed = cfg.v_min + (cfg.v_max - cfg.v_min) * max(
            0.0, 1.0 - severity / cfg.curv_slow
        )
        speed *= meas.confidence
        return Command(speed=speed, curvature=kappa)

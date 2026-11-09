"""Where to aim so the robot misses a cone without leaving the lane.

Deliberately a PURE function: no state, no detector. Given the offset pure
pursuit wants and the cones in front, return the offset to aim for instead.
That keeps the safety reasoning in one place and testable without a camera.

The penalty table decides the strategy (rules §7/§8):

    hitting a cone      k = 0.5
    leaving the track   k = 0.8

So swerving is only worth it while the swerve stays on the track. When it does
not fit, this returns the lane offset unchanged: hitting the cone is the
cheaper mistake, and steering off the track to dodge it is strictly worse.
That is why the result is clamped rather than pushed as far as possible.
"""
from __future__ import annotations

from dataclasses import dataclass

from mav.perception.obstacle import Obstacle


@dataclass(frozen=True)
class AvoidConfig:
    """Geometry of the gap a cone leaves. All metres, offsets +right."""

    clearance_m: float = 0.12   # lateral gap wanted between cone centre and car centre
    max_offset_m: float = 0.15  # never aim further than this from the lane centre
    horizon_m: float = 1.2      # ignore cones further ahead than this
    enabled: bool = True


def choose_offset(
    lane_offset_m: float,
    obstacles: list[Obstacle],
    cfg: AvoidConfig | None = None,
) -> tuple[float, dict]:
    """Target lane offset (m) to steer for, plus a debug dict.

    `lane_offset_m` is what pure pursuit would aim for with no obstacles: +right.
    Returns the same value when there is nothing to avoid, when avoidance is
    disabled, or when the only way past is off the track.
    """
    c = cfg or AvoidConfig()
    dbg: dict = {"action": "none", "reason": None}
    if not c.enabled:
        dbg["reason"] = "disabled"
        return lane_offset_m, dbg

    ahead = [o for o in obstacles if o.distance_m <= c.horizon_m]
    if not ahead:
        dbg["reason"] = "clear"
        return lane_offset_m, dbg

    # The nearest cone decides. A cone behind it cannot be reached before we
    # pass this one, and a compromise between two would thread between them.
    cone = min(ahead, key=lambda o: o.distance_m)
    dbg.update(cone_distance_m=cone.distance_m, cone_lateral_m=cone.lateral_m)

    # Two ways past: aim to the cone's left (smaller x) or its right (larger x).
    # Take whichever needs the smaller deviation, but only if it stays on track.
    left = cone.lateral_m - c.clearance_m
    right = cone.lateral_m + c.clearance_m
    fits = [t for t in (left, right) if abs(t) <= c.max_offset_m]
    if not fits:
        dbg.update(action="hold", reason="no room inside the lane")
        return lane_offset_m, dbg

    target = min(fits, key=abs)
    dbg.update(action="swerve", target_m=target, side="left" if target < cone.lateral_m else "right")
    return target, dbg

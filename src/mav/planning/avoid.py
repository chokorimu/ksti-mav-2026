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
    """Geometry of the gap a cone leaves. All metres, offsets +right.

    `clearance_m` is centre-to-centre, because that is the only thing the camera
    can estimate: the car's half-width (0.10 m) plus the cone's own radius
    (unmeasured, ~0.04 m for a small traffic cone) plus a margin. 0.16 m puts the
    car's corner past the cone instead of clipping it. The first version of this
    file used 0.12 m and that was WRONG: it would have aimed the car's centre
    0.12 m from the cone's centre, i.e. 0.02 m from its edge, with a 0.20 m wide
    car. Clearance has to cover the car's half-width, not just a symbolic gap.

    `max_offset_m` is a RULE number, not a tuning number, so the derivation is
    written down. Under ARC (rules §8) the car is off track when 50% of its body
    is outside the edge line; AR is stricter at 30%, and ARC is what we race:

        lane 0.50 m, car body 0.20 m  ->  violation when the centre is
        0.25 m + 0.10 m = 0.35 m from the lane centre.

    The default stays well inside that on purpose. Hitting a cone costs k=0.5
    and leaving the track costs k=0.8, so aiming at the exact violation
    threshold would trade a 0.5 penalty for a 0.8 one on any detection error.
    0.18 m leaves 0.17 m of margin, and it is the first knob to turn if cones
    near the lane centre keep being hit.

    Note what these two numbers imply. A side fits when |lateral ± clearance|
    <= max_offset, so with 0.16 and 0.18 a cone is passable anywhere within
    |lateral| <= 0.34 m - i.e. effectively anywhere inside the lane. Only a cone
    beyond that (already outside the track, so not a checkpoint) makes the
    planner hold its line. What DOES happen for cones near the centre is that the
    needed offset is close to the budget: a cone at the lane centre wants 0.16 m
    of the 0.18 m available. That is the intended trade, not a bug.
    """

    clearance_m: float = 0.16   # car-centre to cone-centre: car half-width + cone radius
    max_offset_m: float = 0.18  # ARC budget: violation at 0.35, 0.17 m of margin kept
    max_offset_m: float = 0.18  # ARC budget: violation at 0.35, 0.17 m of margin kept
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

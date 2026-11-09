"""Camera geometry helpers for ground-plane calibration."""
from __future__ import annotations

import math


def bottom_row_ground_distance(
    height_m: float,
    pitch_deg: float,
    vertical_fov_deg: float,
    frame_height_px: int,
) -> float:
    """Return forward ground distance at the center of the bottom image row.

    ``pitch_deg`` is the downward angle of the optical axis below horizontal.
    The camera is assumed to point at a flat floor with no roll.
    """
    if height_m <= 0:
        raise ValueError("height_m must be positive")
    if not 0 < vertical_fov_deg < 180:
        raise ValueError("vertical_fov_deg must be between 0 and 180 degrees")
    if frame_height_px <= 0:
        raise ValueError("frame_height_px must be positive")

    ray_angle_deg = (
        pitch_deg
        + vertical_fov_deg / 2
        - vertical_fov_deg / (2 * frame_height_px)
    )
    if not 0 < ray_angle_deg < 90:
        raise ValueError("bottom row ray must intersect the floor in front of the camera")

    return height_m / math.tan(math.radians(ray_angle_deg))

"""Frame rendering for the simulator and the synthetic tests. Numpy-only.

The lane is drawn from a callable `centre_x(forward_m) -> lateral_m` (vehicle
frame), so the same renderer serves the test generator (parabolic centreline)
and the simulator (interpolated polyline).
"""
from __future__ import annotations

from typing import Callable

import numpy as np

CentreFn = Callable[[float], float]


def render_frame(
    centre_x: CentreFn,
    *,
    y_near_m: float = 0.35,
    y_far_m: float = 2.2,
    px_per_m: float = 130.0,
    taper: float = 0.0,
    lane_width_m: float = 0.5,
    line_width_m: float = 0.05,
    dash_m: float = 0.4,
    gap_m: float = 0.2,
    marking: str = "white",
    floor_bgr: tuple[int, int, int] = (115, 115, 115),
    brightness: float = 1.0,
    noise: float = 4.0,
    size: tuple[int, int] = (360, 640),  # (height, width)
    rng: np.random.Generator | None = None,
    draw_lane: bool = True,
) -> np.ndarray:
    """Return an HxWx3 uint8 BGR frame with the lane drawn on a noisy floor.

    taper: horizontal scale shrinks linearly with distance (first-order
    perspective); 0 = affine. Pass an `rng` to advance the noise per frame.
    """
    h, w = size
    cx = w / 2.0
    img = np.broadcast_to(
        np.array(floor_bgr, dtype=np.float32) * brightness, (h, w, 3)
    ).copy()
    if noise > 0:
        img += (rng or np.random.default_rng(0)).normal(0.0, noise, (h, w, 3)).astype(np.float32)

    if not draw_lane:
        return np.clip(img, 0, 255).astype(np.uint8)

    mark_val = 235.0 if marking == "white" else 15.0
    mark = np.array([mark_val] * 3, dtype=np.float32) * brightness
    span = max(y_far_m - y_near_m, 1e-6)

    def scale_at(y: float) -> float:
        return px_per_m * (1.0 - taper * (y - y_near_m) / span)

    def paint(x: float, y: float) -> None:
        s_y = scale_at(y)
        u, v = cx + s_y * x, (h - 1) - px_per_m * (y - y_near_m)
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return
        r = max(1, int(round(s_y * line_width_m / 2.0)))
        img[max(0, vi - r):vi + r + 1, max(0, ui - r):ui + r + 1] = mark

    y = y_near_m
    while y <= y_far_m:
        xc = centre_x(y)
        paint(xc - lane_width_m / 2.0, y)
        paint(xc + lane_width_m / 2.0, y)
        if (y - y_near_m) % (dash_m + gap_m) < dash_m:  # dashed centre line
            paint(xc, y)
        y += 0.005

    return np.clip(img, 0, 255).astype(np.uint8)

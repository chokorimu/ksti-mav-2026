"""Synthetic track frames for lane-detection tests (no camera, no cv2 needed).

Thin wrapper over `mav.sim.render.render_frame`: draws a 50 cm lane (two
continuous edge lines + dashed centre line) on a noisy floor, through either an
affine or a first-order perspective ("taper") projection, so tests can assert
exact numbers (affine) and realistic scale-variation behaviour (taper).

Conventions match the detector: X right, Y forward, curvature > 0 = left turn.
"""
from __future__ import annotations

import numpy as np

from mav.sim.render import render_frame

AFFINE_DEFAULTS = dict(px_per_m=130.0, y_near_m=0.35)
TAPER_DEFAULTS = dict(px_per_m=130.0, y_near_m=0.35, taper=0.25)


def make_frame(
    *,
    offset_m: float = 0.0,
    curvature: float = 0.0,
    marking: str = "white",
    floor_bgr: tuple[int, int, int] = (115, 115, 115),
    brightness: float = 1.0,
    noise: float = 4.0,
    size: tuple[int, int] = (360, 640),  # (height, width)
    seed: int = 0,
    projection: str = "affine",
    y_far_m: float = 2.2,
    lane_width_m: float = 0.5,
    line_width_m: float = 0.05,
    dash_m: float = 0.4,
    gap_m: float = 0.2,
    draw_lane: bool = True,
    **kw: float,
) -> np.ndarray:
    """Return an HxWx3 uint8 BGR frame with a lane drawn on it.

    offset_m: lane centre X at the near row. curvature: 1/m, +left.
    projection="affine": constant scale (exact math for tests).
    projection="taper": horizontal scale shrinks linearly with distance
        (first-order perspective: a straight lane stays straight, but the
        pixel-per-metre scale the detector assumes is only right at one row).
    Extra kwargs: px_per_m / y_near_m / taper.
    """
    if projection not in ("affine", "taper"):
        raise ValueError(projection)
    s = kw.get("px_per_m", TAPER_DEFAULTS["px_per_m"])
    y_near = kw.get("y_near_m", TAPER_DEFAULTS["y_near_m"])
    taper = kw.get("taper", TAPER_DEFAULTS["taper"]) if projection == "taper" else 0.0

    def centre_x(y: float) -> float:
        return offset_m - 0.5 * curvature * (y - y_near) ** 2

    return render_frame(
        centre_x,
        y_near_m=y_near,
        y_far_m=y_far_m,
        px_per_m=s,
        taper=taper,
        lane_width_m=lane_width_m,
        line_width_m=line_width_m,
        dash_m=dash_m,
        gap_m=gap_m,
        marking=marking,
        floor_bgr=floor_bgr,
        brightness=brightness,
        noise=noise,
        size=size,
        rng=np.random.default_rng(seed),
        draw_lane=draw_lane,
    )


def expected_offset_affine(
    *, offset_m: float, curvature: float, cfg, frame_height: int, **kw: float
) -> float:
    """True lane-centre X (m) at the detector's lookahead row, affine projection."""
    from mav.perception.lane import lookahead_row

    s = kw.get("px_per_m", AFFINE_DEFAULTS["px_per_m"])
    y_near = kw.get("y_near_m", AFFINE_DEFAULTS["y_near_m"])
    y_look = y_near + (frame_height - 1 - lookahead_row(cfg, frame_height)) / s
    return offset_m - 0.5 * curvature * (y_look - y_near) ** 2

"""Cross-line (checkpoint / start-finish) detector tests.

Frames come from the shared synthetic generator (tests/synthetic_lane.py);
the cross-line itself is painted on top as a solid band across the lane,
which is what a line perpendicular to the track looks like to the camera.
"""
import time

import numpy as np
import pytest

from mav.perception.checkpoint import CheckpointConfig, CrossLineDetector
from mav.perception.lane import LaneDetector
from synthetic_lane import make_frame

GREEN = (0, 200, 0)      # BGR, hue 120 - inside the default checkpoint band
RED = (0, 0, 200)        # BGR, hue 0 - the guidebook's (unconfirmed) finish colour
BLUE = (200, 0, 0)       # BGR, hue 240 - outside every default band
YELLOW = (0, 255, 255)   # BGR, hue 60 - just outside the default green band
CYAN = (255, 255, 0)     # BGR, hue 180 - outside the default green band
WHITE = (235, 235, 235)  # marking colour: unsaturated
BLACK = (15, 15, 15)     # marking colour: unsaturated and dark

FINISH_CFG = CheckpointConfig(finish_hues=((340.0, 360.0), (0.0, 20.0)))


def paint_band(frame, *, y0, y1, color_bgr, half_width_px=33):
    """Paint a solid horizontal band (a line ACROSS the lane) onto a frame."""
    f = frame.copy()
    h, w = f.shape[:2]
    cx = w // 2
    f[y0:y1, cx - half_width_px:cx + half_width_px] = color_bgr
    return f


def detect(frame, cfg=None):
    return CrossLineDetector(cfg).detect(frame)


def test_green_band_across_the_lane_is_a_checkpoint():
    frame = paint_band(make_frame(seed=1), y0=200, y1=205, color_bgr=GREEN)
    m = detect(frame)
    assert m is not None
    assert m.kind == "checkpoint"
    assert m.confidence >= 0.35
    assert 200 <= m.row <= 204
    assert 0.08 < m.span_frac < 0.2


def test_band_seen_far_and_near():
    for y0 in (100, 340):  # ~2 m ahead and right at the bumper
        frame = paint_band(make_frame(seed=2), y0=y0, y1=y0 + 4, color_bgr=GREEN)
        m = detect(frame)
        assert m is not None, y0
        assert m.kind == "checkpoint"


def test_dim_and_bright_frames():
    for brightness in (0.6, 1.3):
        frame = paint_band(
            make_frame(seed=3, brightness=brightness), y0=200, y1=205, color_bgr=GREEN
        )
        m = detect(frame)
        assert m is not None, brightness
        assert m.kind == "checkpoint", brightness


def test_marking_coloured_bands_are_not_cross_lines():
    # White/black are the LANE MARKING colours (rules §6): the saturation
    # mask must reject them, or every edge line would count as a checkpoint.
    for color in (WHITE, BLACK):
        frame = paint_band(make_frame(seed=4), y0=200, y1=205, color_bgr=color)
        assert detect(frame) is None, color


def test_red_band_ignored_while_finish_is_unconfigured():
    # [A11]: the finish colour is unknown, so the finish band is disabled by
    # default and a red line must NOT be reported.
    frame = paint_band(make_frame(seed=5), y0=200, y1=205, color_bgr=RED)
    assert detect(frame) is None


def test_red_band_detected_when_finish_is_configured():
    frame = paint_band(make_frame(seed=6), y0=200, y1=205, color_bgr=RED)
    m = detect(frame, FINISH_CFG)
    assert m is not None
    assert m.kind == "finish"
    assert m.confidence >= 0.35


def test_hues_outside_the_default_bands_are_rejected():
    for color in (BLUE, YELLOW, CYAN):
        frame = paint_band(make_frame(seed=7), y0=200, y1=205, color_bgr=color)
        assert detect(frame) is None, color


def test_coloured_floor_patch_is_too_tall_to_be_a_line():
    frame = make_frame(seed=8).copy()
    h, w = frame.shape[:2]
    frame[80:300, w // 2 - 200:w // 2 + 200] = BLUE  # a big saturated area
    assert detect(frame) is None


def test_narrow_band_is_rejected():
    # 17 px of 640 = 2.7% of the width: thinner than any lane, so it is a
    # coloured post or a stain, not a line across the track.
    frame = paint_band(make_frame(seed=9), y0=200, y1=205, color_bgr=GREEN, half_width_px=8)
    assert detect(frame) is None


def test_blank_floor_returns_none():
    assert detect(make_frame(draw_lane=False, seed=10)) is None


def test_noisy_flat_floor_returns_none():
    assert detect(make_frame(draw_lane=False, noise=15.0, seed=11)) is None


def test_rejects_wrong_frame_layout():
    with pytest.raises(ValueError):
        detect(np.zeros((10, 10), dtype=np.uint8))
    with pytest.raises(ValueError):
        detect(np.zeros((10, 10, 3), dtype=np.float32))


def test_debug_reports_why():
    frame = paint_band(make_frame(seed=12), y0=200, y1=205, color_bgr=GREEN)
    m, dbg = CrossLineDetector().detect_with_debug(frame)
    assert m is not None
    assert dbg["reason"] is None
    assert dbg["checkpoint"]["enabled"] is True
    assert dbg["finish"]["enabled"] is False

    _, dbg2 = CrossLineDetector().detect_with_debug(make_frame(draw_lane=False, seed=13))
    assert dbg2["reason"] == "no matching colour"

    # A hue that matches but fails the geometry test reports "geometry".
    # 31 px wide: hot per row (>= min_band_pixels) but only 4.8% of the width,
    # so it reaches the span check and fails there.
    narrow = paint_band(make_frame(seed=14), y0=200, y1=205, color_bgr=GREEN, half_width_px=15)
    _, dbg3 = CrossLineDetector().detect_with_debug(narrow)
    assert dbg3["reason"] == "geometry"
    assert dbg3["checkpoint"]["reason"] == "band too narrow"
    assert dbg3["checkpoint"]["span_frac"] < 0.06


def test_lane_detection_survives_a_cross_line():
    # The rules put the line ACROSS the lane symmetrically, so centring must
    # keep working while a checkpoint line is in view.
    frame = paint_band(make_frame(offset_m=0.1, seed=15), y0=200, y1=205, color_bgr=GREEN)
    m = LaneDetector().detect(frame)
    assert m is not None
    assert abs(m.offset_m - 0.1) < 0.05


def test_latency_smoke():
    frame = paint_band(make_frame(seed=16), y0=200, y1=205, color_bgr=GREEN)
    det = CrossLineDetector()
    t0 = time.perf_counter()
    for _ in range(20):
        det.detect(frame)
    mean_ms = (time.perf_counter() - t0) / 20 * 1000.0
    assert mean_ms < 20.0, f"{mean_ms:.2f} ms/frame"

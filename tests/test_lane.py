"""Lane detector tests. Affine projection -> exact expected numbers;
taper projection (first-order perspective) -> sign and rough magnitude only,
because the detector's single px_per_m is only exact at one row."""
import time

import numpy as np
import pytest

from mav.perception.lane import LaneConfig, LaneDetector
from synthetic_lane import AFFINE_DEFAULTS, expected_offset_affine, make_frame

# Matches the affine generator defaults: 130 px/m, bottom row 0.35 m ahead.
CFG = LaneConfig(px_per_m=AFFINE_DEFAULTS["px_per_m"], bottom_row_m=AFFINE_DEFAULTS["y_near_m"])
H = 360


def detect(frame, cfg=CFG):
    return LaneDetector(cfg).detect(frame)


def test_straight_centred_lane():
    m = detect(make_frame(offset_m=0.0, curvature=0.0, seed=1))
    assert m is not None
    assert abs(m.offset_m) < 0.02
    assert abs(m.curvature_inv_m) < 0.05
    assert m.confidence > 0.5


def test_offset_right_and_left():
    for off in (0.2, -0.2):
        m = detect(make_frame(offset_m=off, curvature=0.0, seed=2))
        assert m is not None
        assert abs(m.offset_m - off) < 0.02


def test_curvature_left_is_positive():
    m = detect(make_frame(offset_m=0.0, curvature=0.5, seed=3))
    assert m is not None
    assert abs(m.curvature_inv_m - 0.5) < 0.25
    exp = expected_offset_affine(offset_m=0.0, curvature=0.5, cfg=CFG, frame_height=H)
    assert abs(m.offset_m - exp) < 0.05  # centre bends left of the vehicle


def test_curvature_right_is_negative():
    m = detect(make_frame(offset_m=0.0, curvature=-0.5, seed=4))
    assert m is not None
    assert abs(m.curvature_inv_m + 0.5) < 0.25


def test_white_and_black_markings_both_work():
    for marking in ("white", "black"):
        m = detect(make_frame(offset_m=0.1, curvature=0.0, marking=marking, seed=5))
        assert m is not None, marking
        assert m.confidence > 0.5, marking
        assert abs(m.offset_m - 0.1) < 0.02, marking


def test_dim_and_bright_frames():
    for brightness in (0.6, 1.3):
        m = detect(make_frame(offset_m=0.0, curvature=0.0, brightness=brightness, seed=6))
        assert m is not None, brightness
        assert m.confidence > 0.4, brightness


def test_blank_floor_returns_none():
    assert detect(make_frame(draw_lane=False, seed=7)) is None


def test_noisy_flat_floor_returns_none():
    assert detect(make_frame(draw_lane=False, noise=15.0, seed=8)) is None


def test_rejects_wrong_frame_layout():
    with pytest.raises(ValueError):
        detect(np.zeros((10, 10), dtype=np.uint8))
    with pytest.raises(ValueError):
        detect(np.zeros((10, 10, 3), dtype=np.float32))


def test_latency_smoke():
    # Offsets kept inside the frame so every frame is a valid detection case.
    frames = [make_frame(offset_m=-0.3 + 0.6 * i / 99, curvature=0.1, seed=100 + i)
              for i in range(100)]
    detector = LaneDetector(CFG)
    t0 = time.perf_counter()
    found = sum(detector.detect(f) is not None for f in frames)
    elapsed = time.perf_counter() - t0
    assert found == 100
    assert elapsed < 10.0  # generous: real budget check happens on the Jetson


def test_taper_projection_offset():
    # 130*(1 - 0.25*0.41) ~ 117 px/m at the lookahead row (~1.11 m ahead).
    cfg = LaneConfig(px_per_m=116.7, bottom_row_m=0.35)
    m = detect(make_frame(offset_m=0.15, curvature=0.0, projection="taper", seed=9), cfg)
    assert m is not None
    assert m.confidence > 0.3
    assert abs(m.offset_m - 0.15) < 0.02  # straight lane: offset survives the scale change


def test_taper_projection_curve_sign():
    cfg = LaneConfig(px_per_m=116.7, bottom_row_m=0.35)
    m = detect(make_frame(offset_m=0.0, curvature=0.5, projection="taper", seed=10), cfg)
    assert m is not None
    assert m.confidence > 0.3
    assert m.curvature_inv_m > 0.1  # left curve stays positive under a varying scale

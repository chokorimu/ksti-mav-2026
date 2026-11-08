"""Lane detector tests. Affine projection -> exact expected numbers;
taper projection (first-order perspective) -> sign and rough magnitude only,
because the detector's single px_per_m is only exact at one row.

The last section pins the two changes that make the detector survive the
measured KSTI track (T2, 7 Nov 2026): a shallower ROI and the per-band
transversality gate.
"""
import math
import time

import numpy as np
import pytest

from mav.perception.lane import (
    LaneConfig,
    LaneDetector,
    _band_points,
    lookahead_row,
)
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


# -- the per-band transversality gate (max_band_span_m) ---------------------
#
# Where the track is tight the lane runs ALONG a band instead of across it: the
# marked columns smear over 150-280 px instead of the ~60 px a 0.5 m lane
# occupies, and the centroid of that smear is meaningless (errors up to 85 px
# measured on the KSTI track). A band like that is dropped before the fit.

MASK_H, MASK_W, MASK_BANDS = 120, 640, 4
GATE_PX = 0.8 * 130.0  # max_band_span_m at the default px_per_m


def _mask_with_bands(blocks: dict[int, tuple[int, int]], n_bands: int = MASK_BANDS) -> np.ndarray:
    """Bool mask with a solid block of marked pixels inside the given bands."""
    mask = np.zeros((MASK_H, MASK_W), dtype=bool)
    edges = np.linspace(0, MASK_H, n_bands + 1).astype(int)
    for band, (c0, c1) in blocks.items():
        mask[edges[band]:edges[band + 1], c0:c1 + 1] = True
    return mask


def test_band_gate_drops_a_smeared_band():
    # Band 0 spans ~40 px (a 0.5 m lane at 130 px/m); band 1 smears over 250 px.
    mask = _mask_with_bands({0: (100, 140), 1: (20, 270)})
    kept = _band_points(mask, MASK_BANDS, min_px=30, max_span_px=GATE_PX)
    assert len(kept) == 1
    assert abs(kept[0][1] - 120.0) < 1.0  # centroid of the compact band


def test_band_gate_disabled_keeps_every_band():
    mask = _mask_with_bands({0: (100, 140), 1: (20, 270)})
    assert len(_band_points(mask, MASK_BANDS, min_px=30, max_span_px=None)) == 2


def test_band_span_ignores_stray_noise_pixels():
    """A few isolated noise pixels must not fake a wide band and get a good one
    thrown away: the span is the 5th-95th percentile of the marked columns,
    not the distance between the extremes."""
    mask = _mask_with_bands({0: (100, 140)})
    mask[2, [10, 620, 635]] = True  # stray pixels, far outside the lane
    kept = _band_points(mask, MASK_BANDS, min_px=30, max_span_px=GATE_PX)
    assert len(kept) == 1
    assert abs(kept[0][1] - 120.0) < 2.0


def test_detector_reports_dropped_bands_in_debug():
    # 4 clean bands (enough for a fit), 1 smeared, 1 with no marking at all.
    n_bands = 6
    mask = _mask_with_bands(
        {0: (100, 140), 1: (100, 140), 2: (100, 140), 3: (100, 140), 4: (20, 270)},
        n_bands=n_bands,
    )
    det = LaneDetector(LaneConfig(px_per_m=130.0, bottom_row_m=0.35, n_bands=n_bands))
    # drive _fit_debug directly: it takes the already-thresholded mask
    meas, dbg = det._fit_debug(mask, "light", y0=0, h=MASK_H, w=MASK_W)
    assert meas is not None
    assert dbg["n_bands_with_pixels"] == 4
    # "smeared" and "thin" are different failures and must not be conflated
    assert dbg["n_bands_dropped_smeared"] == 1
    assert dbg["n_bands_dropped_thin"] == 1


# -- the shallow ROI, measured on the real track ----------------------------

def _detector_at_pose(roi_top: float, world_cfg) -> LaneDetector:
    """Build the detector the way the sim runner does: px_per_m is the scale at
    that detector's OWN lookahead row, so changing roi_top moves the scale."""
    h = world_cfg.size[0]
    base = LaneConfig(roi_top=roi_top, bottom_row_m=world_cfg.y_near_m)
    y_look = world_cfg.y_near_m + (h - 1 - lookahead_row(base, h)) / world_cfg.px_per_m
    return LaneDetector(
        LaneConfig(
            roi_top=roi_top,
            px_per_m=world_cfg.scale_at(y_look),
            bottom_row_m=world_cfg.y_near_m,
        )
    )


def _frame_at_track_pose(s_target: float):
    """Render the camera view with the vehicle sitting at arc position s_target."""
    from mav.sim.track import track_points
    from mav.sim.world import LanePath, SimWorld, WorldConfig

    world_cfg = WorldConfig(seed=0)
    pts = np.asarray(track_points("ksti"))
    world = SimWorld(LanePath.from_points(pts), world_cfg)
    i = int(np.argmin(np.abs(world.path.s - s_target)))
    tangent = pts[min(i + 8, len(pts) - 1)] - pts[max(i - 8, 0)]
    tangent = tangent / np.linalg.norm(tangent)
    world.x, world.y = float(pts[i][0]), float(pts[i][1])
    world.theta = math.atan2(-tangent[0], tangent[1])
    return world.frame(), world_cfg


def test_shallow_roi_still_better_than_deep_on_tight_curves():
    """Why roi_top stayed at 0.65. At s=1.75 m the local radius is ~0.73 m,
    so a 1.5 m deep ROI wraps past the 90 degree point of the corner: the
    residual is high and the offset is less accurate. The shallow ROI (0.65)
    has a lower residual and is the recommended setting; the deep ROI is
    still usable in an emergency but its curvature_inv_m will be off.

    Per-band structure score (2e73642) means both ROIs now pass confidence,
    but the deep one is less precise. This test pins that the shallow ROI
    is still better (lower residual) and keeps roi_top at 0.65 as the default."""
    frame, world_cfg = _frame_at_track_pose(1.75)

    deep = _detector_at_pose(0.45, world_cfg)
    m_deep, d_deep = deep.detect_with_debug(frame)
    assert m_deep is not None, "per-band structure score now makes the deep ROI pass"
    assert d_deep["resid"] > 40.0  # deep ROI residual is high on tight curves
    assert m_deep.confidence >= deep.cfg.min_confidence  # but confidence is now OK

    shallow = _detector_at_pose(0.65, world_cfg)  # the new default
    m_shallow, d_shallow = shallow.detect_with_debug(frame)
    assert m_shallow is not None
    assert d_shallow["resid"] < d_deep["resid"], "shallow must be more accurate than deep"
    assert m_shallow.confidence >= shallow.cfg.min_confidence


def test_detector_defaults_pin_the_measured_track_fix():
    """Both values are load-bearing on the measured track; a silent revert
    should fail here rather than on the competition floor."""
    cfg = LaneConfig()
    assert cfg.roi_top == 0.65
    assert cfg.max_band_span_m == 0.8

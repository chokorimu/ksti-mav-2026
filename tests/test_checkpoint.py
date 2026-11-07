"""Cross-line (checkpoint / start-finish) detector tests.

Frames come from the shared synthetic generator (tests/synthetic_lane.py);
the cross-line itself is painted on top as a solid band across the lane,
which is what a line perpendicular to the track looks like to the camera.
"""
import time

import numpy as np
import pytest

from mav.perception.checkpoint import (
    CheckpointConfig,
    CrossLineDetector,
    CrossLineTracker,
)
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


# ---- tracker: one event per physical line --------------------------------
# Frame height is 360, so the default exit row is 0.8 * 360 = 288: a band at
# row 200 is "approaching", a band at row 300+ is "level with the robot".


def band_frame(row, color=GREEN, half_width_px=33, seed=20):
    return paint_band(
        make_frame(seed=seed), y0=row, y1=row + 4, color_bgr=color,
        half_width_px=half_width_px,
    )


def feed(tracker, rows, color=GREEN):
    """Feed a scripted sequence; None = a frame with no line. Returns events."""
    events = []
    for r in rows:
        frame = (
            band_frame(r, color=color)
            if r is not None
            else make_frame(draw_lane=False, seed=99)
        )
        ev = tracker.update(frame)
        if ev is not None:
            events.append(ev)
    return events


def test_line_approaching_and_leaving_fires_exactly_once():
    tr = CrossLineTracker()
    events = feed(tr, [100, 150, 200, 250, 300, 320, None, None])
    assert len(events) == 1
    assert events[0].kind == "checkpoint"


def test_line_first_seen_deep_never_counts():
    # Boot on the start line, or a restart placed on the last checkpoint:
    # the line is first seen at the bumper, so it was never approached.
    tr = CrossLineTracker()
    assert feed(tr, [340, None, None]) == []


def test_no_double_fire_while_line_stays_visible():
    tr = CrossLineTracker()
    events = feed(tr, [100, 200, 300, 320, 330, None, None])
    assert len(events) == 1
    # A dropped frame mid-crossing does not end it either.
    tr2 = CrossLineTracker()
    events2 = feed(tr2, [100, 200, 300, None, 320, None, None])
    assert len(events2) == 1


def test_single_dropped_frame_is_not_a_crossing():
    tr = CrossLineTracker()
    assert feed(tr, [100, 300, None]) == []  # gone for 1 frame only


def test_rolling_back_over_a_line_does_not_recount():
    tr = CrossLineTracker()
    assert len(feed(tr, [100, 200, 300, None, None])) == 1
    # The same line re-enters from the bottom: not approaching, no event.
    assert feed(tr, [340, None, None]) == []


def test_line_lost_while_approaching_does_not_count():
    tr = CrossLineTracker()
    assert feed(tr, [100, 150, None, None]) == []
    # ...and when it reappears it is already deep, so it still does not count.
    assert feed(tr, [300, None, None]) == []


def test_finish_line_event_kind():
    tr = CrossLineTracker(CrossLineDetector(FINISH_CFG))
    events = feed(tr, [100, 200, 300, None, None], color=RED)
    assert len(events) == 1
    assert events[0].kind == "finish"


def test_tracker_reset_forgets_state():
    tr = CrossLineTracker()
    feed(tr, [100, 200, 300])  # mid-crossing
    tr.reset()
    assert tr.state.value == "idle"
    assert feed(tr, [340, None, None]) == []


def test_tracker_debug_is_json_safe():
    import json

    tr = CrossLineTracker()
    feed(tr, [100, 300])
    json.dumps(tr.debug)  # must not raise


def test_tracker_does_not_change_a_run_without_lines():
    # No cross-lines are rendered yet (subtask 3), so attaching a tracker must
    # leave the run bit-identical - the same guarantee the dashboard has.
    from mav.sim.runner import run_sim

    plain = run_sim("straight", seed=3)
    tracked = run_sim("straight", seed=3, tracker=CrossLineTracker())
    assert tracked.checkpoints == 0
    assert (tracked.steps, tracked.detected, tracked.lost, tracked.distance_m) == (
        plain.steps, plain.detected, plain.lost, plain.distance_m,
    )


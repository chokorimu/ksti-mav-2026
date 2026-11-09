"""Orange-cone detection + the swerve decision that follows it.

Frames are the shared synthetic lane with a cone painted on top: a filled
triangle sitting on the floor, which is what a cone looks like from a low
camera. The point of these tests is the geometry contract (lateral position in
metres, nearest first, rejects non-cones), not photorealism.
"""
import numpy as np
import pytest

from mav.perception.obstacle import ConeDetector, ConePassTracker, Obstacle, ObstacleConfig
from mav.planning.avoid import AvoidConfig, choose_offset
from synthetic_lane import make_frame

ORANGE = (0, 120, 255)    # BGR, hue ~19 - inside the default cone band
RED = (0, 0, 200)         # hue 0 - outside the band
GREEN = (0, 200, 0)       # hue 120 - outside the band
WHITE = (235, 235, 235)   # unsaturated: a lane marking, not a cone
GREY = (115, 115, 115)    # the floor


def paint_cone(frame, *, row_bottom, col_centre, height_px=40, width_px=26, color=ORANGE):
    """Paint a cone: a triangle widening towards the bottom (closer) rows."""
    f = frame.copy()
    for i in range(height_px):
        r = row_bottom - height_px + i
        half = max(1, int(round(width_px * (i + 1) / (2.0 * height_px))))
        f[r, col_centre - half:col_centre + half + 1] = color
    return f


def detect(frame, cfg=None):
    return ConeDetector(cfg).detect(frame)


# ---------------------------------------------------------------- detection ----
def test_orange_cone_is_detected():
    frame = paint_cone(make_frame(seed=1), row_bottom=330, col_centre=200)
    blobs = detect(frame)
    assert len(blobs) == 1
    b = blobs[0]
    assert b.pixels > 100
    assert 0.02 < b.width_frac < 0.1
    assert b.lateral_m < 0.0  # left of the camera axis


def test_cone_lateral_position_follows_its_column():
    cfg = ObstacleConfig(px_per_m=130.0)
    centre = paint_cone(make_frame(seed=2), row_bottom=330, col_centre=320)
    right = paint_cone(make_frame(seed=2), row_bottom=330, col_centre=420)
    left = paint_cone(make_frame(seed=2), row_bottom=330, col_centre=220)
    assert abs(detect(centre, cfg)[0].lateral_m) < 0.05
    assert detect(right, cfg)[0].lateral_m > 0.5
    assert detect(left, cfg)[0].lateral_m < -0.5


def test_nearest_cone_comes_first():
    frame = make_frame(seed=3)
    frame = paint_cone(frame, row_bottom=340, col_centre=250)   # close
    frame = paint_cone(frame, row_bottom=200, col_centre=400)   # far
    blobs = detect(frame)
    assert len(blobs) == 2
    assert blobs[0].distance_m < blobs[1].distance_m


def test_non_cone_colours_are_ignored():
    # The lane markings are white/black (rules §6) and the floor is grey: none
    # of them may be mistaken for an obstacle.
    for colour in (RED, GREEN, WHITE, GREY):
        frame = paint_cone(make_frame(seed=4), row_bottom=330, col_centre=320, color=colour)
        assert detect(frame) == [], colour


def test_blank_lane_has_no_obstacles():
    assert detect(make_frame(seed=5)) == []


def test_empty_hue_list_disables_the_detector():
    cfg = ObstacleConfig(hues=())
    frame = paint_cone(make_frame(seed=6), row_bottom=330, col_centre=320)
    blobs, dbg = ConeDetector(cfg).detect_with_debug(frame)
    assert blobs == [] and dbg["reason"] == "disabled"


def test_far_field_is_ignored():
    # roi_top keeps the far half of the frame out: a distant cone is a few
    # pixels and would only add noise to the swerve decision.
    cfg = ObstacleConfig(roi_top=0.6)
    frame = paint_cone(make_frame(seed=7), row_bottom=170, col_centre=320, height_px=18)
    assert detect(frame, cfg) == []


def test_bad_frame_layout_raises():
    with pytest.raises(ValueError, match="HxWx3 uint8"):
        ConeDetector().detect(np.zeros((10, 10), dtype=np.uint8))


def test_debug_reports_why_nothing_was_found():
    frame = paint_cone(make_frame(seed=8), row_bottom=330, col_centre=320, color=GREEN)
    blobs, dbg = ConeDetector().detect_with_debug(frame)
    assert blobs == []
    assert dbg["reason"] == "no matching colour"


# ---------------------------------------------------------------- avoidance ----
def _cone(lateral_m, distance_m=0.8):
    return Obstacle(row=300.0, col=320.0, lateral_m=lateral_m, distance_m=distance_m,
                    width_frac=0.04, height_frac=0.1, pixels=200)


def test_no_obstacle_keeps_the_lane_offset():
    assert choose_offset(0.02, [])[0] == 0.02


def test_cone_ahead_is_passed_on_the_cheaper_side():
    # A cone slightly right of centre: going left is the smaller deviation.
    target, dbg = choose_offset(0.0, [_cone(0.05)])
    assert dbg["action"] == "swerve"
    assert target < 0.0
    assert abs(target - (0.05 - AvoidConfig().clearance_m)) < 1e-9


def test_cone_far_away_is_ignored_until_it_is_close():
    target, dbg = choose_offset(0.0, [_cone(0.05, distance_m=3.0)])
    assert target == 0.0 and dbg["reason"] == "clear"


def test_nearest_cone_decides():
    near = _cone(0.0, distance_m=0.5)
    far = _cone(0.0, distance_m=1.0)
    _, dbg = choose_offset(0.0, [far, near])
    assert dbg["cone_distance_m"] == 0.5


def test_cone_near_the_edge_is_passed_on_the_other_side():
    # A cone near the left edge cannot be passed on the left without leaving the
    # lane, so the right-hand gap is used instead.
    target, dbg = choose_offset(0.0, [_cone(-0.14)])
    assert dbg["action"] == "swerve"
    assert abs(target) <= AvoidConfig().max_offset_m
    assert target > -0.14


def test_no_room_holds_the_lane_instead_of_leaving_it():
    # clear_margin = max_offset - clearance = 0.18 - 0.16 = 0.02, so a cone more
    # than 2 cm off the centre can be passed on one side. Raise the clearance past
    # the whole budget and nothing fits: the robot must accept the cone, not the
    # track (cone k=0.5 is cheaper than off-track k=0.8).
    assert AvoidConfig().max_offset_m > AvoidConfig().clearance_m  # one side must fit
    cfg = AvoidConfig(clearance_m=0.30)
    target, dbg = choose_offset(0.03, [_cone(0.0)], cfg)
    assert dbg["action"] == "hold"
    assert dbg["reason"] == "no room inside the lane"
    assert target == 0.03


def test_a_cone_dead_centre_is_still_passable_in_arc():
    # The ARC budget (0.18) exceeds the clearance (0.16), so even a cone exactly
    # on the centre line leaves a gap. Under the old 0.15 budget with 0.12
    # clearance this also fitted; the point is that the rule change did not break
    # the middle-of-the-lane case.
    target, dbg = choose_offset(0.0, [_cone(0.0)])
    assert dbg["action"] == "swerve"
    assert abs(target) == pytest.approx(AvoidConfig().clearance_m)


def test_needle_cone_is_the_only_thing_that_holds_the_lane():
    # Beyond max_offset - clearance the cone is outside the passable band.
    target, dbg = choose_offset(0.0, [_cone(0.5)])
    assert dbg["action"] == "hold"
    assert target == 0.0


def test_disabled_avoidance_keeps_the_lane_offset():
    target, dbg = choose_offset(0.0, [_cone(0.0)], AvoidConfig(enabled=False))
    assert target == 0.0 and dbg["reason"] == "disabled"


# ---------------------------------------------------------------- counting ----
# ARC §8: "Avoiding an obstacle counts as passing the checkpoint", so a cone that
# is physically passed is worth one `cp`. These drive a cone down the frame the
# way the robot would see it approach.

def _walk(tracker, frame_rows, cones_per_frame, gone_tail=3):
    """Feed a cone moving down the frame, then frames with no cone."""
    events = []
    for row_bottom in cones_per_frame:
        f = paint_cone(make_frame(seed=9), row_bottom=row_bottom, col_centre=320)
        ev = tracker.update(f)
        if ev is not None:
            events.append(ev)
    blank = make_frame(seed=9)
    for _ in range(gone_tail):
        ev = tracker.update(blank)
        if ev is not None:
            events.append(ev)
    return events


def test_approaching_then_leaving_counts_one_cone():
    t = ConePassTracker()
    events = _walk(t, 360, [140, 200, 260, 320, 350])
    assert len(events) == 1
    assert t.passed == 1
    assert abs(events[0].lateral_m) < 0.05  # centre column 320 of 640


def test_a_cone_first_seen_at_the_bottom_never_counts():
    # The restart case: the robot is placed next to the cone it must not re-count.
    t = ConePassTracker()
    t.update(paint_cone(make_frame(seed=9), row_bottom=355, col_centre=320))
    for _ in range(4):
        assert t.update(make_frame(seed=9)) is None
    assert t.passed == 0


def test_a_cone_that_never_leaves_does_not_count():
    # Still approaching, never at the exit row: the frame just ends.
    t = ConePassTracker()
    for row_bottom in (140, 180, 220):
        assert t.update(paint_cone(make_frame(seed=9), row_bottom=row_bottom, col_centre=320)) is None
    assert t.passed == 0


def test_one_dropped_frame_is_not_a_pass():
    t = ConePassTracker(min_gone_frames=2)
    for row_bottom in (140, 250, 350):
        t.update(paint_cone(make_frame(seed=9), row_bottom=row_bottom, col_centre=320))
    assert t.update(make_frame(seed=9)) is None   # one blank frame: still tracked
    assert t.update(paint_cone(make_frame(seed=9), row_bottom=350, col_centre=320)) is None
    assert t.passed == 0


def test_two_cones_in_a_row_count_twice():
    t = ConePassTracker()
    for _ in range(2):
        _walk(t, 360, [140, 220, 300, 350])
    assert t.passed == 2


def test_reset_forgets_the_tracked_cone_but_keeps_the_score():
    t = ConePassTracker()
    _walk(t, 360, [140, 220, 300, 350])
    assert t.passed == 1
    t.reset()
    assert t.passed == 1 and t.tracked is None and t.state.value == "idle"


def test_tracker_reports_its_state_for_the_overlay():
    t = ConePassTracker()
    t.update(paint_cone(make_frame(seed=9), row_bottom=200, col_centre=200))
    dbg = t.debug
    assert dbg["state"] == "approaching" and dbg["passed"] == 0
    assert dbg["lateral_m"] is not None and dbg["lateral_m"] < 0.0

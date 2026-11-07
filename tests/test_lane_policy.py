import pytest

from mav.perception.lane import LaneMeasurement
from mav.planning.lane_policy import LanePolicy, LanePolicyConfig
from mav.types import STOP, Command


def meas(offset, curvature=0.0, conf=0.9, lookahead=1.0):
    return LaneMeasurement(offset_m=offset, lookahead_m=lookahead,
                           curvature_inv_m=curvature, confidence=conf, n_points=10)


def test_steers_right_when_lane_is_right():
    c = LanePolicy().command(meas(0.2))
    assert c.curvature < 0
    assert c.curvature == pytest.approx(-2 * 0.2 / 1.0)


def test_steers_left_when_lane_is_left():
    c = LanePolicy().command(meas(-0.2))
    assert c.curvature > 0
    assert c.curvature == pytest.approx(2 * 0.2 / 1.0)


def test_curvature_clamped_to_min_turn_radius():
    # Rules: minimum turning radius 1.5 m -> |kappa| <= 1/1.5.
    p = LanePolicy(LanePolicyConfig(lookahead_m=0.5, max_offset_m=1.0))
    c = p.command(meas(-0.6))
    assert c.curvature == pytest.approx(1 / 1.5)


def test_stops_when_measurement_missing_or_untrusted():
    p = LanePolicy()
    assert p.command(None) == STOP
    assert p.command(meas(0.1, conf=0.1)) == STOP
    assert p.command(meas(0.5)) == STOP  # beyond max_offset_m


def test_hold_option_keeps_last_command():
    p = LanePolicy(LanePolicyConfig(on_lost="hold"))
    last = Command(0.5, 0.1)
    assert p.command(None, last) == last


def test_speed_drops_with_curvature_and_confidence():
    p = LanePolicy()
    assert p.command(meas(0.0, curvature=0.0, conf=1.0)).speed == pytest.approx(1.0)
    assert p.command(meas(0.0, curvature=2.0, conf=1.0)).speed == pytest.approx(0.15)
    assert p.command(meas(0.0, curvature=0.0, conf=0.5)).speed == pytest.approx(0.5)

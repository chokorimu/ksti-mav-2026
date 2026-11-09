import pytest

from mav.control.pwm_calibration import pwm_for


TABLE = ((0, 0.0), (80, 0.20), (120, 0.50), (180, 0.90), (240, 1.20))


def test_pwm_for_returns_zero_at_zero_speed():
    assert pwm_for(0.0, TABLE) == 0


def test_pwm_for_uses_first_moving_point_inside_deadband():
    assert pwm_for(0.01, TABLE) == 80
    assert pwm_for(0.20, TABLE) == 80


def test_pwm_for_interpolates_between_measured_points():
    assert pwm_for(0.35, TABLE) == 100
    assert pwm_for(0.70, TABLE) == 150


def test_pwm_for_clamps_outside_measured_range():
    assert pwm_for(2.0, TABLE) == 240


def test_pwm_for_rejects_invalid_speed_and_tables():
    with pytest.raises(ValueError, match="must not be negative"):
        pwm_for(-0.1, TABLE)
    with pytest.raises(ValueError, match="start with"):
        pwm_for(0.2, ((20, 0.0), (80, 0.2)))
    with pytest.raises(ValueError, match="increase strictly"):
        pwm_for(0.2, ((0, 0.0), (100, 0.2), (120, 0.2)))
    with pytest.raises(ValueError, match="at least one moving"):
        pwm_for(0.2, ((0, 0.0),))

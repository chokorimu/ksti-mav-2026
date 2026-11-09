"""The speed calibration fit must be right, because nothing downstream checks it.

`MPS_TO_PWM` and `PWM_DEADBAND` are open loop: the robot has no encoders, so a
wrong constant is never noticed by the car - it just drives at the wrong speed,
and the lane offsets and cone distances inherit the error. So the arithmetic is
tested here, on the laptop, with numbers whose answer is known by hand.

The failure this file is really guarding is the ONE-constant model: setting
`K = 255 / top_speed` draws the line through the origin, which overstates the
speed at low PWM. The policy drives at 0.15-0.3 m/s, so that is where the error
lands. `test_one_constant_model_overstates_low_speed` measures that error.
"""
from __future__ import annotations

import pytest

from mav.tools import calibrate_speed as cs

# pwm = 40 + speed * 200, i.e. a motor that only moves above PWM 40.
PERFECT = [(60.0, 0.100), (100.0, 0.300), (140.0, 0.500)]


def test_recovers_a_known_line_exactly():
    model = cs.fit_speed_model(PERFECT)
    assert model.pwm_per_mps == pytest.approx(200.0)
    assert model.deadband == pytest.approx(40.0)
    assert model.rms_mps == pytest.approx(0.0, abs=1e-9)
    assert model.n_points == 3
    assert not model.deadband_clamped


def test_pwm_for_inverts_the_model():
    model = cs.fit_speed_model(PERFECT)
    # 0.100 m/s -> 40 + 20 = 60. The absolute floor of the sweep.
    assert model.pwm_for(0.100) == 60
    assert model.pwm_for(0.300) == 100
    assert model.pwm_for(-0.300) == -100  # reverse keeps the deadband
    assert model.pwm_for(0.0) == 0, "stop must coast, not sit on the deadband"
    # Round trip through the model the firmware uses.
    for v in (0.05, 0.15, 0.30, 0.50):
        assert model.speed_for(model.pwm_for(v)) == pytest.approx(v, abs=0.005)


def test_one_constant_model_overstates_low_speed():
    """Why the firmware takes two numbers. This is the bug, quantified.

    The tempting calibration measures one point at full PWM and sets
    K = 255 / top. Do that here: top is 0.5 m/s at PWM 140, so K = 280.
    At 0.15 m/s it commands PWM 42, and the real motor needs 70 - the car gets
    PWM 42, which is barely above breakaway, so it crawls or stalls. That is the
    whole reason `PWM_DEADBAND` exists.
    """
    model = cs.fit_speed_model(PERFECT)
    # The naive calibration: one measurement at full PWM, K = 255 / that speed.
    top_speed = model.speed_for(cs.PWM_MAX)
    naive_k = cs.PWM_MAX / top_speed
    for speed in (0.15, 0.30):
        naive_pwm = speed * naive_k
        correct_pwm = model.pwm_for(speed)
        # The naive model is BELOW the truth across the driving range, because it
        # spends the breakaway budget on the origin instead of on the motor.
        assert naive_pwm < correct_pwm, (speed, naive_pwm, correct_pwm)
    # At the bottom of the range the shortfall is a large share of the command:
    # the car is told to do 0.15 m/s and gets PWM that is barely moving.
    assert model.pwm_for(0.15) - 0.15 * naive_k > 25


def test_a_stalled_point_raises_the_deadband():
    """A PWM that did not move the car is evidence, not noise."""
    pts = [(40.0, 0.0), (60.0, 0.100), (100.0, 0.300), (140.0, 0.500)]
    model = cs.fit_speed_model(pts)
    assert model.deadband >= 40.0
    assert model.n_points == 3, "the stalled point is excluded from the line fit"


def test_stalled_points_alone_cannot_produce_a_number():
    with pytest.raises(ValueError, match="moved the car"):
        cs.fit_speed_model([(40.0, 0.0), (50.0, 0.0)])


def test_refuses_a_flat_or_backwards_run():
    with pytest.raises(ValueError, match="higher PWM"):
        cs.fit_speed_model([(60.0, 0.3), (100.0, 0.2), (140.0, 0.1)])


def test_refuses_a_run_with_no_pwm_spread():
    with pytest.raises(ValueError, match="same PWM"):
        cs.fit_speed_model([(100.0, 0.2), (100.0, 0.3)])


def test_refuses_non_finite_measurements():
    with pytest.raises(ValueError, match="non-finite"):
        cs.fit_speed_model([(60.0, float("nan")), (100.0, 0.3)])


def test_deadband_is_never_negative():
    """A line through the origin fits 'deadband 0', not a negative PWM.

    Floating-point noise must not trip the "check your run" warning: a line that
    genuinely passes through the origin lands on -1e-14, not on -5.
    """
    model = cs.fit_speed_model([(60.0, 0.100), (120.0, 0.200), (180.0, 0.300)])
    assert model.deadband == 0.0
    assert not model.deadband_clamped


def test_a_line_above_the_origin_is_flagged():
    """Motor moving at PWM 0 is impossible: clamp AND say so, loudly."""
    model = cs.fit_speed_model([(60.0, 0.170), (100.0, 0.250), (140.0, 0.330)])
    assert model.deadband_clamped
    assert model.deadband == 0.0
    assert model.pwm_for(0.0) == 0


def test_pwm_for_never_exceeds_the_avr_range():
    model = cs.fit_speed_model(PERFECT)
    assert model.pwm_for(10.0) == cs.PWM_MAX
    assert model.pwm_for(-10.0) == -cs.PWM_MAX


def test_the_sweep_commands_match_the_protocol():
    """protocol.md: 'W l r' differential, 'D steer speed' ackermann."""
    assert cs._command_line("differential", 0.25) == "W 0.250 0.250"
    assert cs._command_line("ackermann", 0.25) == "D 0.0000 0.250"


def test_dry_run_touches_no_hardware(capsys):
    assert cs.main(["--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "no hardware" in out
    assert "MPS_TO_PWM" not in out, "a dry run must not print a value to paste"
    # The worked example has to be right; it is what the operator checks by hand.
    assert "20 + speed * 400" in out
    model = cs.fit_speed_model([(60.0, 0.100), (100.0, 0.200), (140.0, 0.300)])
    assert (model.pwm_per_mps, model.deadband) == pytest.approx((400.0, 20.0))


def test_pwm_outside_the_avr_range_is_refused(capsys):
    assert cs.main(["--dry-run", "--pwm", "0", "300"]) == 2
    assert "out of range" in capsys.readouterr().out


def test_a_real_sweep_needs_a_port(capsys):
    assert cs.main([]) == 2
    assert "--port is required" in capsys.readouterr().out


def test_sweep_refuses_a_zero_k_current():
    """With K = 0 every command is 'W 0.000 0.000': a run that measures nothing."""
    with pytest.raises(SystemExit, match="k-current"):
        cs.sweep("/dev/null", 0.0, [60])

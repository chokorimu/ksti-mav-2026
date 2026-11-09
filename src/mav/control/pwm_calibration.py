"""Map measured motor speeds to PWM commands."""
from __future__ import annotations

from collections.abc import Sequence

CalibrationPoint = tuple[int, float]


def pwm_for(speed_mps: float, table: Sequence[CalibrationPoint]) -> int:
    """Return PWM for a non-negative speed using a measured calibration table.

    The table contains ``(pwm, speed_mps)`` points, including ``(0, 0)`` and
    at least one moving point. Positive requests below the first measured
    moving speed use that point's PWM, which avoids the motor deadband.
    Requests above the measured range use the highest measured PWM.
    """
    if speed_mps < 0:
        raise ValueError("speed_mps must not be negative")
    points = _validate_table(table)
    if speed_mps == 0:
        return 0

    first_pwm, first_speed = points[1]
    if speed_mps <= first_speed:
        return first_pwm
    if speed_mps >= points[-1][1]:
        return points[-1][0]

    for (low_pwm, low_speed), (high_pwm, high_speed) in zip(points, points[1:]):
        if speed_mps <= high_speed:
            fraction = (speed_mps - low_speed) / (high_speed - low_speed)
            return round(low_pwm + fraction * (high_pwm - low_pwm))

    raise RuntimeError("speed interpolation fell outside the validated table")


def _validate_table(table: Sequence[CalibrationPoint]) -> tuple[CalibrationPoint, ...]:
    points = tuple((int(pwm), float(speed)) for pwm, speed in table)
    if len(points) < 2:
        raise ValueError("table must contain (0, 0) and at least one moving point")
    if points[0] != (0, 0.0):
        raise ValueError("table must start with (0, 0)")

    previous_pwm = -1
    previous_speed = -1.0
    for pwm, speed in points:
        if not 0 <= pwm <= 255:
            raise ValueError("PWM values must be between 0 and 255")
        if speed < 0:
            raise ValueError("measured speeds must not be negative")
        if pwm <= previous_pwm or speed <= previous_speed and (pwm, speed) != (0, 0.0):
            raise ValueError("table PWM and speed values must increase strictly")
        previous_pwm = pwm
        previous_speed = speed
    if points[1][1] <= 0:
        raise ValueError("the first moving point must have positive speed")
    return points

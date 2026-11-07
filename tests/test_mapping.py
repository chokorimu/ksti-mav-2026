import math

from mav.control.mapping import AckermannMapper, DifferentialMapper
from mav.types import Command, VehicleParams


def params(drive="ackermann"):
    # Made-up numbers for testing only; real values come from configs/vehicle.yaml.
    return VehicleParams(drive=drive, wheel_base_m=0.30, track_width_m=0.20,
                         max_steer_rad=math.radians(30), max_speed_mps=1.0)


def test_differential_straight():
    w = DifferentialMapper(params("differential")).map(Command(0.5, 0.0))
    assert w.left == w.right == 0.5


def test_differential_left_turn_right_wheel_faster():
    w = DifferentialMapper(params("differential")).map(Command(0.5, 2.0))
    assert w.right > w.left
    assert abs((w.left + w.right) / 2 - 0.5) < 1e-9  # mean speed stays v


def test_differential_right_turn_left_wheel_faster():
    w = DifferentialMapper(params("differential")).map(Command(0.5, -2.0))
    assert w.left > w.right


def test_differential_clamps_and_keeps_radius():
    w = DifferentialMapper(params("differential")).map(Command(5.0, 2.0))
    assert max(abs(w.left), abs(w.right)) <= 1.0 + 1e-9
    assert w.right > w.left


def test_ackermann_straight():
    s = AckermannMapper(params()).map(Command(0.5, 0.0))
    assert s.steer_rad == 0.0 and s.speed == 0.5 and not s.saturated


def test_ackermann_angle_matches_geometry():
    k = 1.0  # 1 m radius
    s = AckermannMapper(params()).map(Command(0.5, k))
    assert abs(s.steer_rad - math.atan(0.30 * k)) < 1e-9
    assert not s.saturated


def test_ackermann_saturates_symmetrically():
    left = AckermannMapper(params()).map(Command(0.5, 10.0))
    right = AckermannMapper(params()).map(Command(0.5, -10.0))
    assert left.saturated and right.saturated
    assert abs(left.steer_rad - math.radians(30)) < 1e-9
    assert abs(right.steer_rad + math.radians(30)) < 1e-9


def test_ackermann_clamps_speed():
    assert AckermannMapper(params()).map(Command(9.0, 0.0)).speed == 1.0

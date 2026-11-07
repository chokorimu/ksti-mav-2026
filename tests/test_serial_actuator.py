"""SerialActuator: the host half of firmware/arduino/protocol.md.

The port is faked, so these tests need no Arduino and no pyserial.
"""
import math

import pytest

from mav.control.mapping import AckermannMapper, DifferentialMapper
from mav.io.actuator import SafeActuator, SerialActuator
from mav.types import Command, VehicleParams


def params(drive="differential"):
    # Made-up numbers for testing only; real values come from configs/vehicle.yaml.
    return VehicleParams(drive=drive, wheel_base_m=0.30, track_width_m=0.20,
                         max_steer_rad=math.radians(30), max_speed_mps=1.0)


class FakePort:
    """Minimal serial port: records everything written, can fail on demand."""

    def __init__(self, fail_on: bytes | None = None):
        self.written = b""
        self.fail_on = fail_on  # a line containing this substring makes write() raise

    def write(self, data: bytes) -> int:
        if self.fail_on is not None and self.fail_on in data:
            raise OSError("port gone")
        self.written += data
        return len(data)

    @property
    def lines(self):
        return self.written.decode("ascii").splitlines()


def diff_actuator(port=None):
    return SerialActuator(DifferentialMapper(params("differential")), port or FakePort())


def ack_actuator(port=None):
    return SerialActuator(AckermannMapper(params()), port or FakePort())


def test_differential_straight_sends_equal_wheel_speeds():
    port = FakePort()
    diff_actuator(port).set(Command(0.5, 0.0))
    assert port.lines == ["W 0.500 0.500"]


def test_differential_left_turn_inner_wheel_slower():
    # track 0.20 m -> half-track 0.10; v=0.5, curvature 2.0 -> yaw rate 1.0 rad/s
    port = FakePort()
    diff_actuator(port).set(Command(0.5, 2.0))
    assert port.lines == ["W 0.400 0.600"]


def test_differential_reverse_is_negative():
    port = FakePort()
    diff_actuator(port).set(Command(-0.5, 0.0))
    assert port.lines == ["W -0.500 -0.500"]


def test_ackermann_line_format():
    port = FakePort()
    ack_actuator(port).set(Command(0.5, 1.0))  # 1 m radius
    assert port.lines == ["D 0.2915 0.500"]  # atan(0.30 * 1.0)


def test_ackermann_saturated_steer_is_clamped():
    port = FakePort()
    ack_actuator(port).set(Command(0.5, 10.0))
    assert port.lines == ["D 0.5236 0.500"]  # clamped to max_steer_rad = 30 deg


def test_stop_sends_s():
    port = FakePort()
    diff_actuator(port).stop()
    assert port.written == b"S\n"


def test_precision_is_fixed_decimals():
    port = FakePort()
    diff_actuator(port).set(Command(0.123456, 0.0))
    assert port.lines == ["W 0.123 0.123"]


def test_unsupported_mapper_output_raises():
    class WeirdMapper:
        def map(self, cmd):
            return "nonsense"

    with pytest.raises(TypeError):
        SerialActuator(WeirdMapper(), FakePort()).set(Command(0.5, 0.0))


def test_write_error_propagates_and_stop_is_attempted():
    port = FakePort(fail_on=b"W")
    safe = SafeActuator(diff_actuator(port), max_speed_mps=1.0)
    safe.enable()
    with pytest.raises(OSError):
        safe.set(Command(0.5, 0.0))
    # the wrapper attempted a stop before re-raising
    assert port.written.endswith(b"S\n")


def test_safe_actuator_disabled_turns_set_into_stop():
    port = FakePort()
    safe = SafeActuator(diff_actuator(port), max_speed_mps=1.0)
    safe.set(Command(0.5, 0.0))  # never enabled
    assert port.lines and all(not line.startswith("W") for line in port.lines)


def test_safe_actuator_timeout_stops():
    port = FakePort()
    t = [0.0]
    safe = SafeActuator(diff_actuator(port), max_speed_mps=1.0,
                        timeout_s=0.3, clock=lambda: t[0])
    safe.enable()
    safe.set(Command(0.5, 0.0))
    assert port.lines[-1] == "W 0.500 0.500"
    t[0] += 0.31
    safe.tick()
    assert port.lines[-1] == "S"
    assert safe.timed_out

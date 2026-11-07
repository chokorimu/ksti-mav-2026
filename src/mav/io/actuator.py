"""Actuator layer. Real motor/servo drivers are NOT implemented yet (hardware path undecided).

Everything goes through `SafeActuator`, which is stopped by default, clamps speed,
and stops the robot if commands stop arriving.
"""
from __future__ import annotations

import time
from typing import Callable, Protocol

from mav.control.mapping import SteerDrive, WheelSpeeds
from mav.types import STOP, Command


class Actuator(Protocol):
    def set(self, cmd: Command) -> None: ...

    def stop(self) -> None: ...


class NullActuator:
    """Does nothing but record commands. Default for development without hardware and for tests."""

    def __init__(self) -> None:
        self.history: list[Command] = []
        self.last: Command = STOP

    def set(self, cmd: Command) -> None:
        self.history.append(cmd)
        self.last = cmd

    def stop(self) -> None:
        self.set(STOP)


class CallbackActuator:
    """Simulator actuator: forwards commands to a function (e.g. send over WebSocket)."""

    def __init__(self, send: Callable[[Command], None]) -> None:
        self._send = send

    def set(self, cmd: Command) -> None:
        self._send(cmd)

    def stop(self) -> None:
        self._send(STOP)


class SerialActuator:
    """Real actuator: sends mapped commands to the Arduino over USB serial.

    The mapper (DifferentialMapper for prototype, AckermannMapper for final)
    converts the vehicle-agnostic Command into hardware values; this class
    serialises them per firmware/arduino/protocol.md ("W left right" or
    "D steer_rad speed"). Wrap in SafeActuator: stopped default, clamp,
    command timeout.

    The port is injected: tests pass a fake port, production passes
    open_serial_port(...). pyserial is deliberately NOT imported here, so the
    test suite runs on machines that do not have it.
    """

    BAUDRATE = 115200

    def __init__(self, mapper, port) -> None:
        self._mapper = mapper
        self._port = port

    def set(self, cmd: Command) -> None:
        out = self._mapper.map(cmd)
        if isinstance(out, WheelSpeeds):
            line = f"W {out.left:.3f} {out.right:.3f}\n"
        elif isinstance(out, SteerDrive):
            line = f"D {out.steer_rad:.4f} {out.speed:.3f}\n"
        else:
            raise TypeError(f"unsupported mapper output: {type(out).__name__}")
        self._port.write(line.encode("ascii"))

    def stop(self) -> None:
        self._port.write(b"S\n")


def open_serial_port(port: str, baudrate: int = SerialActuator.BAUDRATE):
    """Open a pyserial port for SerialActuator. Imported lazily so tests (and
    machines without pyserial) never need it. Install: pip install pyserial
    (on the Jetson, sudo apt install python3-serial also works)."""
    import serial

    return serial.Serial(port, baudrate, timeout=0)


class SafeActuator:
    """Safety wrapper. Starts disabled (stopped). Rules:
    - disabled          -> every set() becomes stop()
    - speed clamped to +-max_speed_mps
    - no set() within timeout_s while enabled -> stop() (call tick() every loop)
    - if the inner actuator raises, stop() is attempted and the error is re-raised
    """

    def __init__(
        self,
        inner: Actuator,
        max_speed_mps: float,
        timeout_s: float = 0.3,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._inner = inner
        self._max_speed = max_speed_mps
        self._timeout = timeout_s
        self._clock = clock
        self._enabled = False
        self._last_set = clock()
        self.timed_out = False
        inner.stop()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        if not self._enabled:
            self._enabled = True
            self._last_set = self._clock()
            self.timed_out = False

    def disable(self) -> None:
        self._enabled = False
        self._inner.stop()

    def set(self, cmd: Command) -> None:
        if not self._enabled:
            self._inner.stop()
            return
        speed = max(-self._max_speed, min(self._max_speed, cmd.speed))
        self._last_set = self._clock()
        self.timed_out = False
        try:
            self._inner.set(Command(speed, cmd.curvature))
        except Exception:
            try:
                self._inner.stop()
            finally:
                raise

    def tick(self) -> None:
        if self._enabled and self._clock() - self._last_set > self._timeout:
            self.timed_out = True
            self._inner.stop()

    def stop(self) -> None:
        self._inner.stop()

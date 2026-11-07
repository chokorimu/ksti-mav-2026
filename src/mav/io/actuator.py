"""Actuator layer. Real motor/servo drivers are NOT implemented yet (hardware path undecided).

Everything goes through `SafeActuator`, which is stopped by default, clamps speed,
and stops the robot if commands stop arriving.
"""
from __future__ import annotations

import time
from typing import Callable, Protocol

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

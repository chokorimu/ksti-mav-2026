"""Start / emergency button events."""
from __future__ import annotations

import queue
import time
from enum import Enum
from typing import Callable, Protocol


class Event(Enum):
    START = "start"
    EMERGENCY = "emergency"


class Buttons(Protocol):
    def poll(self) -> list[Event]:
        """Return events since the last call (already debounced)."""


class GpioButtons:
    """Non-blocking, debounced buttons using Jetson.GPIO BOARD numbering.

    Buttons are wired from each GPIO input to ground, so an unpressed button is
    HIGH and a pressed button is LOW through the internal pull-up resistor.
    ``gpio`` and ``clock`` are injectable for tests without Jetson hardware.
    """

    def __init__(
        self,
        start_pin: int,
        emergency_pin: int,
        *,
        debounce_s: float = 0.05,
        gpio=None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if debounce_s < 0:
            raise ValueError("debounce_s must not be negative")
        if start_pin == emergency_pin:
            raise ValueError("start_pin and emergency_pin must differ")

        if gpio is None:
            try:
                import Jetson.GPIO as gpio_module
            except ImportError as exc:
                raise RuntimeError(
                    "GpioButtons requires the Jetson.GPIO package on the Jetson"
                ) from exc
            gpio = gpio_module

        self._gpio = gpio
        self._clock = clock
        self._debounce_s = debounce_s
        self._pins = {
            start_pin: Event.START,
            emergency_pin: Event.EMERGENCY,
        }
        self._raw_state = {}
        self._stable_state = {}
        self._changed_at = {}

        try:
            gpio.setmode(gpio.BOARD)
            for pin in self._pins:
                gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_UP)
                state = gpio.input(pin)
                self._raw_state[pin] = state
                self._stable_state[pin] = state
                self._changed_at[pin] = clock()
        except Exception:
            self.close()
            raise

    def poll(self) -> list[Event]:
        now = self._clock()
        events: list[Event] = []
        for pin, event in self._pins.items():
            state = self._gpio.input(pin)
            if state != self._raw_state[pin]:
                self._raw_state[pin] = state
                self._changed_at[pin] = now
            elif (
                state != self._stable_state[pin]
                and now - self._changed_at[pin] >= self._debounce_s
            ):
                self._stable_state[pin] = state
                if state == self._gpio.LOW:
                    events.append(event)
        return events

    def close(self) -> None:
        cleanup = getattr(self._gpio, "cleanup", None)
        if cleanup is not None:
            cleanup(list(self._pins))


class ScriptedButtons:
    """Events scheduled by poll count. For tests and simulator runs.

    ScriptedButtons({0: [Event.START], 50: [Event.EMERGENCY]})
    """

    def __init__(self, script: dict[int, list[Event]]) -> None:
        self._script = script
        self._tick = 0

    def poll(self) -> list[Event]:
        events = self._script.get(self._tick, [])
        self._tick += 1
        return list(events)


class QueueButtons:
    """Events pushed at runtime (web dashboard, interactive runs). Thread-safe."""

    def __init__(self) -> None:
        self._queue: queue.Queue[Event] = queue.Queue()

    def push(self, event: Event) -> None:
        self._queue.put(event)

    def poll(self) -> list[Event]:
        events: list[Event] = []
        while True:
            try:
                events.append(self._queue.get_nowait())
            except queue.Empty:
                return events

    def clear(self) -> None:
        """Drop pending events. Used after a scenario switch / reset: the robot
        then waits for a fresh START, like the real competition flow."""
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

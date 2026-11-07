"""Start / emergency button events.

The real GPIO implementation is NOT written yet: it depends on which pins are used and how the
buttons are wired (and the physical e-stop must also cut motor power in hardware).
"""
from __future__ import annotations

import queue
from enum import Enum
from typing import Protocol


class Event(Enum):
    START = "start"
    EMERGENCY = "emergency"


class Buttons(Protocol):
    def poll(self) -> list[Event]:
        """Return events since the last call (already debounced)."""


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

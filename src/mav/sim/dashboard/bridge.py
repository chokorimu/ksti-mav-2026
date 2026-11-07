"""Bridge between the simulator loop and a viewer (web dashboard, tests).

The runner publishes a SimState + frame every tick and drains controls. The
default NullBridge does nothing, so `run_sim()` behaves exactly as it did before
the dashboard existed (asserted in tests/test_dashboard.py).

Controls mirror the physical buttons, so the dashboard demonstrates the real
competition flow: START, EMERGENCY, then START again to RESUME (the checkpoint
count survives), plus scenario switching and reset.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from mav.sim.dashboard.state import SimState
from mav.sim.scenarios import SCENARIOS

ACTIONS = ("start", "emergency", "scenario", "reset")


@dataclass(frozen=True)
class Control:
    """One request from the viewer. `scenario` is only set for action="scenario"."""

    action: str
    scenario: str | None = None


def parse_control(payload: object) -> Control:
    """Validate a control payload coming from the browser. Raises ValueError."""
    if not isinstance(payload, dict):
        raise ValueError("control payload must be a JSON object")
    action = payload.get("action")
    if action not in ACTIONS:
        raise ValueError(f"unknown action {action!r}; have: {', '.join(ACTIONS)}")
    scenario = payload.get("scenario")
    if action == "scenario":
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario {scenario!r}; have: {', '.join(sorted(SCENARIOS))}")
    elif scenario is not None:
        raise ValueError("'scenario' is only valid with action='scenario'")
    return Control(action=action, scenario=scenario)


class Bridge(Protocol):
    def publish(self, state: SimState, frame: np.ndarray | None) -> None:
        """Called once per tick with the newest snapshot and camera frame."""
        ...

    def poll_control(self) -> Control | None:
        """Return the next pending control, or None. Called once per tick."""
        ...


class NullBridge:
    """Default: no viewer attached."""

    def publish(self, state: SimState, frame: np.ndarray | None) -> None:
        pass

    def poll_control(self) -> Control | None:
        return None


class FakeBridge:
    """Scripted controls for tests; records every published state."""

    def __init__(self, controls: list[Control] | tuple[Control, ...] = ()) -> None:
        self._controls = list(controls)
        self.published: list[SimState] = []

    def publish(self, state: SimState, frame: np.ndarray | None) -> None:
        self.published.append(state)

    def poll_control(self) -> Control | None:
        return self._controls.pop(0) if self._controls else None

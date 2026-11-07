"""Bridge between a run loop (any profile) and the web dashboard.

Same pattern as ``mav.sim.dashboard.bridge``: the loop calls ``publish()`` once
per tick and drains ``poll_control()``; the default ``NullBridge`` does nothing,
so a run without a dashboard behaves exactly as before (``tests/test_dashboard.py``
asserts this by comparing results with the same seed).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from mav.dashboard.state import DashboardState

ACTIONS = ("start", "emergency", "reset", "profile", "record")
PROFILES = ("sim", "prototype", "final")


@dataclass(frozen=True)
class Control:
    """One request from the viewer. ``profile`` and ``session`` are only set
    for ``action == "profile"`` and ``action == "record`` respectively.
    """
    action: str
    profile: str | None = None
    session: str | None = None
    on: bool | None = None      # record on / off (only with action == "record``
    # out: we could add steering output fields here if needed, but the pipeline
    # publishes them via the DashboardState out: dict.


def parse_control(payload: object) -> Control:
    """Validate a control payload coming from the browser.

    Raises ``ValueError`` for malformed input. Caller must check that the
    Control's ``action`` is allowed (e.g. that the current robot state accepts
    it - see ``DashboardState.state``).
    """
    if not isinstance(payload, dict):
        raise ValueError("control payload must be a JSON object")
    action = payload.get("action")
    if action not in ACTIONS:
        raise ValueError(f"unknown action {action!r}; have: {', '.join(ACTIONS)}")
    profile = payload.get("profile")
    if action == "profile":
        if profile not in PROFILES:
            raise ValueError(f"unknown profile {profile!r}; have: {', '.join(PROFILES)}")
        session = None
        on = None
    elif action == "record":
        profile = None
        on = bool(payload.get("on", False))
        session = payload.get("session")
        if session is not None and not isinstance(session, str):
            raise ValueError("session must be a string")
    else:
        # start / emergency / reset accept no extra fields (besides optional unused)
        profile = None
        session = None
        on = None
        if profile is not None or session is not None:
            raise ValueError(f"profile or session must not be present with {action}")
    return Control(action=action, profile=profile, session=session, on=on)


@runtime_checkable
class Bridge(Protocol):
    def publish(self, state: DashboardState, frame) -> None:
        """Called once per tick with the newest snapshot and camera frame."""
        ...

    def poll_control(self) -> Control | None:
        """Return the next pending control, or None. Called once per tick."""
        ...


class NullBridge:
    """Default: no viewer attached."""

    def publish(self, state: DashboardState, frame) -> None:
        pass

    def poll_control(self) -> Control | None:
        return None


class FakeBridge:
    """Scripted controls for tests; records every published state."""

    def __init__(self, controls: list[Control] | tuple[Control, ...] = ()):
        self._controls = list(controls)
        self.published: list[DashboardState] = []

    def publish(self, state: DashboardState, frame) -> None:
        self.published.append(state)

    def poll_control(self) -> Control | None:
        return self._controls.pop(0) if self._controls else None

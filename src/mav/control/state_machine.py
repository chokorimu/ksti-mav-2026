"""Robot run states.

BOOT -> WAIT_START -> RUN <-> STOPPED
Competition rule: after a violation a human presses emergency, moves the robot to the last
checkpoint and presses start again. That is a RESUME, not a new attempt: the checkpoint count
must survive STOPPED -> RUN. Only new_attempt() resets it, and it must be called deliberately.
finish() (finish line crossed) also lands in STOPPED, but is NOT resumable: START is ignored
until new_attempt(), so a completed run cannot be accidentally restarted.
"""
from __future__ import annotations

from enum import Enum

from mav.io.buttons import Event


class State(Enum):
    BOOT = "boot"
    WAIT_START = "wait_start"
    RUN = "run"
    STOPPED = "stopped"


class RobotStateMachine:
    def __init__(self) -> None:
        self.state = State.BOOT
        self.checkpoints = 0
        self.finished = False

    @property
    def is_running(self) -> bool:
        return self.state is State.RUN

    def boot_done(self) -> None:
        if self.state is State.BOOT:
            self.state = State.WAIT_START

    def handle(self, event: Event) -> None:
        if event is Event.EMERGENCY:
            self.state = State.STOPPED
        elif event is Event.START and self.state in (State.WAIT_START, State.STOPPED):
            if self.finished:
                return  # a finished attempt is not resumable; new_attempt() first
            self.state = State.RUN

    def add_checkpoint(self) -> bool:
        """Count a checkpoint. Only counts while running. Returns True if counted."""
        if self.state is State.RUN:
            self.checkpoints += 1
            return True
        return False

    def finish(self) -> bool:
        """Finish line crossed: stop and end the attempt.

        The checkpoint count is kept for reporting. START is ignored until
        new_attempt(), so a finished run cannot be "resumed" by accident.
        Returns True if this call ended the run (only possible while RUNNING).
        """
        if self.state is not State.RUN:
            return False
        self.state = State.STOPPED
        self.finished = True
        return True

    def new_attempt(self) -> None:
        """Explicit reset between attempts. Never called automatically by START."""
        self.checkpoints = 0
        self.finished = False
        self.state = State.WAIT_START

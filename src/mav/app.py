"""One control-loop iteration. Same code for sim, prototype and deploy."""
from __future__ import annotations

from typing import Callable, Optional

from mav.control.state_machine import RobotStateMachine
from mav.io.actuator import SafeActuator
from mav.io.buttons import Buttons
from mav.io.camera import Camera, Frame
from mav.types import STOP, Command

Policy = Callable[[Optional[Frame], RobotStateMachine], Command]


def step(
    sm: RobotStateMachine,
    camera: Camera,
    buttons: Buttons,
    actuator: SafeActuator,
    policy: Policy,
) -> Command:
    """Poll buttons, update state, and drive the actuator. Returns the command sent (STOP if idle)."""
    for event in buttons.poll():
        sm.handle(event)

    if not sm.is_running:
        actuator.disable()
        return STOP

    frame = camera.read()
    cmd = policy(frame, sm) if frame is not None else STOP  # no frame -> stop, stay in RUN
    actuator.enable()
    actuator.set(cmd)
    actuator.tick()
    return cmd

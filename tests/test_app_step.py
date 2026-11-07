from mav.app import step
from mav.control.state_machine import RobotStateMachine, State
from mav.io.actuator import NullActuator, SafeActuator
from mav.io.buttons import Event, ScriptedButtons
from mav.io.camera import CallbackCamera
from mav.types import STOP, Command


def build(script, frame="frame"):
    sm = RobotStateMachine()
    sm.boot_done()
    inner = NullActuator()
    act = SafeActuator(inner, max_speed_mps=1.0)
    cam = CallbackCamera(lambda: frame)
    return sm, ScriptedButtons(script), act, cam, inner


def policy(frame, sm):
    return Command(0.5, 0.1)


def test_idle_until_start():
    sm, buttons, act, cam, inner = build({})
    assert step(sm, cam, buttons, act, policy) == STOP
    assert inner.last == STOP


def test_runs_after_start_then_stops_on_emergency():
    sm, buttons, act, cam, inner = build({0: [Event.START], 2: [Event.EMERGENCY]})
    assert step(sm, cam, buttons, act, policy) == Command(0.5, 0.1)
    assert inner.last == Command(0.5, 0.1)
    step(sm, cam, buttons, act, policy)
    assert step(sm, cam, buttons, act, policy) == STOP
    assert sm.state is State.STOPPED and inner.last == STOP


def test_missing_frame_stops_but_stays_in_run():
    sm, buttons, act, cam, inner = build({0: [Event.START]}, frame=None)
    assert step(sm, cam, buttons, act, policy) == STOP
    assert sm.state is State.RUN and inner.last == STOP


def test_resume_after_emergency():
    sm, buttons, act, cam, inner = build(
        {0: [Event.START], 1: [Event.EMERGENCY], 2: [Event.START]})
    step(sm, cam, buttons, act, policy)
    sm.add_checkpoint()
    step(sm, cam, buttons, act, policy)
    assert sm.state is State.STOPPED
    step(sm, cam, buttons, act, policy)
    assert sm.state is State.RUN and sm.checkpoints == 1

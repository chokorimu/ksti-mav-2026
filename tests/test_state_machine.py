from mav.control.state_machine import RobotStateMachine, State
from mav.io.buttons import Event


def running_machine():
    sm = RobotStateMachine()
    sm.boot_done()
    sm.handle(Event.START)
    return sm


def test_start_ignored_during_boot():
    sm = RobotStateMachine()
    sm.handle(Event.START)
    assert sm.state is State.BOOT


def test_normal_start():
    sm = running_machine()
    assert sm.state is State.RUN


def test_emergency_from_any_state():
    sm = RobotStateMachine()
    sm.handle(Event.EMERGENCY)
    assert sm.state is State.STOPPED


def test_checkpoints_only_count_while_running():
    sm = RobotStateMachine()
    assert sm.add_checkpoint() is False
    sm.boot_done()
    assert sm.add_checkpoint() is False
    sm.handle(Event.START)
    assert sm.add_checkpoint() is True
    assert sm.checkpoints == 1


def test_resume_after_emergency_keeps_checkpoint_count():
    sm = running_machine()
    sm.add_checkpoint()
    sm.add_checkpoint()
    sm.handle(Event.EMERGENCY)
    assert sm.state is State.STOPPED
    assert sm.add_checkpoint() is False
    sm.handle(Event.START)  # human placed robot at last checkpoint and pressed start
    assert sm.state is State.RUN
    assert sm.checkpoints == 2


def test_start_while_running_does_nothing():
    sm = running_machine()
    sm.add_checkpoint()
    sm.handle(Event.START)
    assert sm.state is State.RUN and sm.checkpoints == 1


def test_new_attempt_is_explicit_and_resets():
    sm = running_machine()
    sm.add_checkpoint()
    sm.handle(Event.EMERGENCY)
    sm.new_attempt()
    assert sm.state is State.WAIT_START and sm.checkpoints == 0


def test_finish_stops_and_is_not_resumable():
    sm = running_machine()
    sm.add_checkpoint()
    assert sm.finish() is True
    assert sm.state is State.STOPPED
    assert sm.finished is True
    sm.handle(Event.START)  # a finished attempt is over, not paused
    assert sm.state is State.STOPPED
    assert sm.checkpoints == 1  # count kept for reporting
    sm.new_attempt()
    sm.handle(Event.START)
    assert sm.state is State.RUN and sm.checkpoints == 0 and sm.finished is False


def test_finish_only_while_running():
    sm = RobotStateMachine()
    assert sm.finish() is False
    assert sm.finished is False
    sm.boot_done()
    assert sm.finish() is False  # waiting for start is not a run to finish
    sm.handle(Event.START)
    assert sm.finish() is True


def test_emergency_after_finish_keeps_the_run_finished():
    sm = running_machine()
    sm.finish()
    sm.handle(Event.EMERGENCY)
    assert sm.state is State.STOPPED and sm.finished is True
    sm.handle(Event.START)
    assert sm.state is State.STOPPED

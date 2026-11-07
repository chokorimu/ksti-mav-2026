from mav.io.actuator import NullActuator, SafeActuator
from mav.types import STOP, Command


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(timeout=0.3, vmax=1.0):
    inner = NullActuator()
    clock = FakeClock()
    return SafeActuator(inner, max_speed_mps=vmax, timeout_s=timeout, clock=clock), inner, clock


def test_starts_stopped_and_ignores_commands_until_enabled():
    act, inner, _ = make()
    act.set(Command(0.5, 0.0))
    assert inner.last == STOP


def test_enabled_passes_command_and_clamps_speed():
    act, inner, _ = make(vmax=1.0)
    act.enable()
    act.set(Command(5.0, 0.2))
    assert inner.last == Command(1.0, 0.2)
    act.set(Command(-5.0, 0.0))
    assert inner.last.speed == -1.0


def test_disable_stops():
    act, inner, _ = make()
    act.enable()
    act.set(Command(0.5, 0.0))
    act.disable()
    assert inner.last == STOP and not act.enabled


def test_timeout_stops_when_commands_stop_arriving():
    act, inner, clock = make(timeout=0.3)
    act.enable()
    act.set(Command(0.5, 0.0))
    clock.t = 0.2
    act.tick()
    assert inner.last == Command(0.5, 0.0) and not act.timed_out
    clock.t = 0.5
    act.tick()
    assert inner.last == STOP and act.timed_out


def test_fresh_command_clears_timeout():
    act, inner, clock = make(timeout=0.3)
    act.enable()
    clock.t = 1.0
    act.tick()
    assert act.timed_out
    act.set(Command(0.4, 0.0))
    assert not act.timed_out and inner.last == Command(0.4, 0.0)


def test_inner_error_triggers_stop_and_reraises():
    class Broken(NullActuator):
        def __init__(self):
            super().__init__()
            self.fail = False
            self.stops = 0

        def set(self, cmd):
            if self.fail and cmd != STOP:
                raise OSError("bus error")
            super().set(cmd)

        def stop(self):
            self.stops += 1
            super().stop()

    inner = Broken()
    act = SafeActuator(inner, max_speed_mps=1.0, clock=FakeClock())
    act.enable()
    inner.fail = True
    stops_before = inner.stops
    try:
        act.set(Command(0.5, 0.0))
        raised = False
    except OSError:
        raised = True
    assert raised and inner.stops > stops_before

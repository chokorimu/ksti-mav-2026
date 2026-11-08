from types import SimpleNamespace as NS

from mav.robot import Driver
from mav.types import STOP, Command


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


class Detector:
    def __init__(self):
        self.meas = None

    def detect_with_debug(self, frame):
        return self.meas, {}


class Policy:
    cfg = NS(conf_min=0.3, max_offset_m=0.35)

    def command(self, meas, last=STOP):
        if meas is None or meas.confidence < self.cfg.conf_min:
            return last
        return Command(0.3, -2.0 * meas.offset_m)


class Tracker:
    def __init__(self, kind):
        self.kind = kind

    def update(self, frame):
        return NS(kind=self.kind)


class SM:
    def __init__(self):
        self.cp = 0
        self.done = False

    def add_checkpoint(self):
        self.cp += 1

    def finish(self):
        self.done = True


def good(off=0.1):
    return NS(offset_m=off, confidence=0.9)


def make(max_hold_s=0.5, tracker=None):
    det, clock = Detector(), Clock()
    return det, clock, Driver(det, Policy(), tracker, max_hold_s, clock)


def test_tracks_when_lane_is_seen():
    det, _, d = make()
    det.meas = good(0.1)
    cmd = d("frame", SM())
    assert cmd == Command(0.3, -0.2) and not d.lost


def test_holds_last_command_briefly_then_stops():
    det, clock, d = make(max_hold_s=0.5)
    det.meas = good()
    held = d("f", SM())
    det.meas = None
    clock.t = 0.2
    assert d("f", SM()) == held and d.lost
    clock.t = 0.8
    assert d("f", SM()) == STOP


def test_recovers_when_lane_returns():
    det, clock, d = make(max_hold_s=0.5)
    det.meas = None
    clock.t = 1.0
    d("f", SM())
    clock.t = 2.0
    assert d("f", SM()) == STOP
    det.meas = good(0.0)
    assert d("f", SM()) == Command(0.3, -0.0) and not d.lost


def test_implausible_offset_counts_as_lost():
    det, clock, d = make(max_hold_s=0.5)
    det.meas = good(0.9)  # beyond max_offset_m
    d("f", SM())
    assert d.lost


def test_reset_clears_held_command():
    det, _, d = make()
    det.meas = good()
    d("f", SM())
    d.reset()
    det.meas = None
    assert d("f", SM()) == STOP


def test_cross_line_events_update_state_machine():
    det, _, d = make(tracker=Tracker("checkpoint"))
    sm = SM()
    det.meas = good()
    d("f", sm)
    assert sm.cp == 1 and not sm.done
    _, _, d2 = make(tracker=Tracker("finish"))
    sm2 = SM()
    d2("f", sm2)
    assert sm2.done

from dataclasses import dataclass
from types import SimpleNamespace as NS

import pytest

from mav.perception.obstacle import Obstacle
from mav.planning.avoid import AvoidConfig
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


@dataclass(frozen=True)
class Meas:
    """The two fields Driver and the fake policy read; frozen so Driver can replace it."""
    offset_m: float
    confidence: float = 0.9


def good(off=0.1):
    return Meas(off)


class Cones:
    def __init__(self, blobs):
        self.blobs = blobs

    def detect(self, frame):
        return self.blobs


def cone(lateral_m, distance_m=0.8):
    return Obstacle(row=300.0, col=320.0, lateral_m=lateral_m, distance_m=distance_m,
                    width_frac=0.04, height_frac=0.1, pixels=200)


class ConeTracker:
    """Stub for ConePassTracker: emits an event on the frames listed in `events`."""

    def __init__(self, events):
        self.events = events
        self.calls = 0

    def update(self, frame):
        self.calls += 1
        return self.events.pop(0) if self.events else None


def make(max_hold_s=0.5, tracker=None, cones=None, cone_tracker=None):
    det, clock = Detector(), Clock()
    return det, clock, Driver(det, Policy(), tracker, max_hold_s, clock, cones=cones,
                              cone_tracker=cone_tracker)


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


# ---------------------------------------------------------------- avoidance ----
def test_cone_ahead_shifts_the_aim_point_not_the_measurement():
    det, _, d = make(cones=Cones([cone(0.05)]))
    det.meas = good(0.0)
    cmd = d("f", SM())
    # Cone slightly right of centre: aim CLEARANCE to its left -> -0.11 m -> left.
    # The fake policy is curvature = -2 * offset, so 0.11 -> 0.22.
    target = 0.05 - AvoidConfig().clearance_m
    assert cmd.speed == 0.3 and cmd.curvature == pytest.approx(-2.0 * target)
    assert cmd.curvature > 0.0            # positive curvature = left turn
    assert d.meas.offset_m == 0.0         # the logged measurement stays the real lane offset
    assert d.avoid["action"] == "swerve"


def test_no_cone_leaves_the_command_untouched():
    det, _, d = make(cones=Cones([]))
    det.meas = good(0.1)
    assert d("f", SM()) == Command(0.3, -0.2)
    assert d.avoid["reason"] == "clear"


def test_avoidance_cannot_mask_a_lost_lane():
    # The lane is 0.9 m off: still lost, even though a swerve target would be in range.
    det, _, d = make(cones=Cones([cone(-0.5)]))
    det.meas = good(0.9)
    d("f", SM())
    assert d.lost


# ---------------------------------------------------------------- ARC score ----
def test_passing_a_cone_counts_as_a_checkpoint():
    # ARC §8: "Avoiding an obstacle counts as passing the checkpoint", so `cp`
    # is the number of cones passed - the biggest term in the score.
    det, _, d = make(cone_tracker=ConeTracker(["passed"]))
    det.meas = good(0.0)
    sm = SM()
    d("f", sm)
    assert sm.cp == 1


def test_a_frame_without_a_pass_counts_nothing():
    det, _, d = make(cone_tracker=ConeTracker([]))
    det.meas = good(0.0)
    sm = SM()
    for _ in range(3):
        d("f", sm)
    assert sm.cp == 0


def test_cones_are_not_counted_while_the_robot_is_stopped():
    # The REAL state machine gates on RUN, so a cone drifting past while stopped
    # (or while a human carries the robot back to a checkpoint) cannot inflate
    # the score. Uses the real class because that gate is its behaviour.
    from mav.control.state_machine import RobotStateMachine

    det, _, d = make(cone_tracker=ConeTracker(["passed"]))
    det.meas = good(0.0)
    sm = RobotStateMachine()
    sm.boot_done()
    d("f", sm)                      # WAIT_START, not RUN
    assert sm.checkpoints == 0


def test_no_cone_tracker_means_no_counting():
    det, _, d = make()
    det.meas = good(0.0)
    sm = SM()
    d("f", sm)
    assert sm.cp == 0


# ---------------------------------------------------- startup, before the loop --
def test_autodetect_uses_the_single_board(monkeypatch):
    """AGENTS.md forbids hardcoding serial port names, and the ACM/USB split
    means any hardcoded name is wrong on half the boards."""
    import mav.robot as robot

    monkeypatch.setattr(robot, "list_serial_ports", lambda: ["/dev/ttyUSB0"])
    assert robot._autodetect_port("configs/prototype.yaml") == "/dev/ttyUSB0"


def test_autodetect_refuses_when_nothing_is_plugged_in(monkeypatch):
    import mav.robot as robot

    monkeypatch.setattr(robot, "list_serial_ports", lambda: [])
    with pytest.raises(SystemExit, match="no Arduino was found"):
        robot._autodetect_port("configs/prototype.yaml")


def test_autodetect_refuses_to_guess_between_two_boards(monkeypatch):
    """Picking the wrong port would send motor commands to a random device."""
    import mav.robot as robot

    monkeypatch.setattr(robot, "list_serial_ports", lambda: ["/dev/ttyUSB0", "/dev/ttyACM0"])
    with pytest.raises(SystemExit) as exc:
        robot._autodetect_port("configs/prototype.yaml")
    assert "2 serial devices" in str(exc.value)
    assert "--port" in str(exc.value)


def test_autodetect_reports_missing_pyserial_not_a_traceback(monkeypatch):
    import mav.robot as robot
    from mav.io.actuator import SerialUnavailable

    def boom():
        raise SerialUnavailable("pyserial is not installed")

    monkeypatch.setattr(robot, "list_serial_ports", boom)
    with pytest.raises(SystemExit, match="pyserial is not installed"):
        robot._autodetect_port("configs/prototype.yaml")


def test_first_frame_proves_the_camera_delivers():
    """A camera that opens but never produces a frame looks like a robot that
    simply ignores its commands, so startup waits for one real frame."""
    import numpy as np

    import mav.robot as robot

    class Cam:
        def __init__(self, frames):
            self.frames = list(frames)

        def read(self):
            return self.frames.pop(0) if self.frames else None

    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    assert robot._first_frame(Cam([None, None, frame]), timeout_s=1.0) is frame


def test_first_frame_gives_up_instead_of_hanging():
    import mav.robot as robot

    class Dead:
        def read(self):
            return None

    assert robot._first_frame(Dead(), timeout_s=0.05) is None


def test_serial_is_checked_before_the_camera_opens(monkeypatch):
    """The 10 Nov 2026 crash, pinned.

    On the Jetson the camera opened fine, then `open_serial_port()` raised
    `ModuleNotFoundError: No module named 'serial'`. The traceback landed right
    under the Argus log, so a missing Python package read as a camera fault.
    With an explicit --port (or a `port:` in config) the autodetect helper does
    not run, so the check has to be its own step before the camera.
    """
    import sys

    import mav.io.camera as camera
    import mav.robot as robot

    opened = []
    monkeypatch.setitem(sys.modules, "serial", None)   # venv without pyserial
    monkeypatch.setenv("MAV_PROFILE", "prototype")
    monkeypatch.setattr(camera, "open_video", lambda path: opened.append(path))

    with pytest.raises(SystemExit, match="pyserial is not installed"):
        robot.main(["--port", "/dev/ttyUSB0", "--video", "whatever.mp4"])
    assert opened == [], "the camera was opened before the dependency was checked"


def test_no_board_message_says_how_to_check(monkeypatch):
    """A dead end is not a message. It must name the commands that find the board.

    The Nano is powered over USB, so "not found" almost always means unplugged -
    but it can also mean a missing CH340 driver or a permission problem, and the
    operator needs to be able to tell those apart without reading the source.
    """
    import mav.robot as robot

    monkeypatch.setattr(robot, "list_serial_ports", lambda: [])
    with pytest.raises(SystemExit) as exc:
        robot._autodetect_port("configs/prototype.yaml")
    msg = str(exc.value)
    assert "--list-ports" in msg          # how to list what is there
    assert "lsusb" in msg                 # how to see the USB device itself
    assert "dmesg" in msg                 # whether the kernel saw it
    assert "dialout" in msg               # the permission fix


# ------------------------------------------- module-level name resolution --
def test_module_level_helpers_only_use_module_level_names():
    """A NameError that the test suite should never have let through.

    `_open_serial_or_exit` is a module-level function, but `open_serial_port` was
    imported INSIDE `main()`, so it existed only in main's local scope. Calling
    the helper therefore raised

        NameError: name 'open_serial_port' is not defined

    on the Jetson, after the camera had already opened - so the robot ran, the
    camera worked, and startup still died with a traceback that looked unrelated.

    Unit tests missed it because they exercised `_autodetect_port` and
    `_first_frame` but never `_open_serial_or_exit`. This checks the whole class
    of bug instead of that one function: every global name referenced by a
    module-level function must resolve at module level.
    """
    import builtins
    import pathlib
    import symtable

    import mav.robot as robot

    source = pathlib.Path(robot.__file__).read_text()
    table = symtable.symtable(source, robot.__file__, "exec")
    module_names = set(vars(robot))
    builtins_ = set(dir(builtins))

    problems = []
    for child in table.get_children():
        if child.get_type() != "function":
            continue
        # Only look INSIDE the function body: nested scopes are checked too.
        for sym in child.get_symbols():
            if not sym.is_global():
                continue
            name = sym.get_name()
            if name in module_names or name in builtins_:
                continue
            problems.append(f"{child.get_name()}() uses undefined global {name!r}")

    assert not problems, "module-level functions reference names that do not exist:\n" + \
        "\n".join(problems)


def test_open_serial_or_exit_actually_opens(monkeypatch):
    """The call site that raised NameError, exercised directly."""
    import sys
    import types

    opened = []

    class FakeSerialModule:
        SerialException = type("SerialException", (Exception,), {})

        @staticmethod
        def Serial(port, baud, timeout=0):
            opened.append((port, baud, timeout))
            return f"handle:{port}"

    monkeypatch.setitem(sys.modules, "serial", FakeSerialModule)
    import mav.robot as robot

    assert robot._open_serial_or_exit("/dev/ttyCH341USB0") == "handle:/dev/ttyCH341USB0"
    assert opened == [("/dev/ttyCH341USB0", 115200, 0)]


def test_open_serial_or_exit_reports_a_failure_as_a_clean_exit(monkeypatch):
    """A startup problem must exit with a message, not a traceback."""
    import sys

    import types

    class FakeSerialModule:
        SerialException = type("SerialException", (Exception,), {})

        @staticmethod
        def Serial(port, baud, timeout=0):
            raise FakeSerialModule.SerialException("device busy")

    tools = types.ModuleType("serial.tools")
    tools.list_ports = type("LP", (), {"comports": staticmethod(lambda: [])})
    FakeSerialModule.tools = tools
    monkeypatch.setitem(sys.modules, "serial", FakeSerialModule)
    monkeypatch.setitem(sys.modules, "serial.tools", tools)
    import mav.robot as robot

    with pytest.raises(SystemExit, match="cannot open serial port"):
        robot._open_serial_or_exit("/dev/ttyCH341USB0")


def test_main_reaches_the_control_loop_with_a_serial_port(monkeypatch):
    """End to end: config -> camera -> serial -> control loop.

    This is the test that would have caught the NameError before the Jetson did.
    Everything between config validation and the loop is exercised with a fake
    camera and a fake serial module, so any undefined name on that path fails
    here instead of after the camera has already opened on the robot.
    """
    import sys
    import types

    import numpy as np

    import mav.io.camera as cam
    import mav.robot as robot

    opened = []

    class FakeSerialModule:
        SerialException = type("SerialException", (Exception,), {})

        @staticmethod
        def Serial(port, baud, timeout=0):
            opened.append(port)
            return type("P", (), {"write": lambda self, d: None,
                                  "close": lambda self: None})()

    tools = types.ModuleType("serial.tools")
    tools.list_ports = type("LP", (), {"comports": staticmethod(lambda: [])})
    FakeSerialModule.tools = tools
    monkeypatch.setitem(sys.modules, "serial", FakeSerialModule)
    monkeypatch.setitem(sys.modules, "serial.tools", tools)

    class FakeCam:
        def read(self):
            return np.zeros((360, 640, 3), dtype=np.uint8)

        def close(self):
            pass

    monkeypatch.setattr(cam, "open_csi", lambda **kw: FakeCam())
    monkeypatch.setenv("MAV_PROFILE", "prototype")
    # main() starts a daemon thread reading stdin; left real, it raises when
    # pytest closes stdin and pollutes the run with a thread warning.
    monkeypatch.setattr(robot, "_stdin_buttons", lambda *a, **k: None)

    # Break out of the infinite loop once the loop has clearly started.
    calls = []

    def fake_sleep(_s):
        calls.append(1)
        if len(calls) > 2:
            raise KeyboardInterrupt

    monkeypatch.setattr(robot.time, "sleep", fake_sleep)

    assert robot.main(["--port", "/dev/ttyCH341USB0"]) == 0
    assert opened == ["/dev/ttyCH341USB0"]

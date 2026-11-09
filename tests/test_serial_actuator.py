"""SerialActuator: the host half of firmware/arduino/protocol.md.

The port is faked, so these tests need no Arduino and no pyserial.
"""
import math

import pytest

from mav.control.mapping import AckermannMapper, DifferentialMapper
from mav.io.actuator import SafeActuator, SerialActuator
from mav.types import Command, VehicleParams


def params(drive="differential"):
    # Made-up numbers for testing only; real values come from configs/vehicle.yaml.
    return VehicleParams(drive=drive, wheel_base_m=0.30, track_width_m=0.20,
                         max_steer_rad=math.radians(30), max_speed_mps=1.0)


class FakePort:
    """Minimal serial port: records everything written, can fail on demand."""

    def __init__(self, fail_on: bytes | None = None):
        self.written = b""
        self.fail_on = fail_on  # a line containing this substring makes write() raise

    def write(self, data: bytes) -> int:
        if self.fail_on is not None and self.fail_on in data:
            raise OSError("port gone")
        self.written += data
        return len(data)

    @property
    def lines(self):
        return self.written.decode("ascii").splitlines()


def diff_actuator(port=None):
    return SerialActuator(DifferentialMapper(params("differential")), port or FakePort())


def ack_actuator(port=None):
    return SerialActuator(AckermannMapper(params()), port or FakePort())


def test_differential_straight_sends_equal_wheel_speeds():
    port = FakePort()
    diff_actuator(port).set(Command(0.5, 0.0))
    assert port.lines == ["W 0.500 0.500"]


def test_differential_left_turn_inner_wheel_slower():
    # track 0.20 m -> half-track 0.10; v=0.5, curvature 2.0 -> yaw rate 1.0 rad/s
    port = FakePort()
    diff_actuator(port).set(Command(0.5, 2.0))
    assert port.lines == ["W 0.400 0.600"]


def test_differential_reverse_is_negative():
    port = FakePort()
    diff_actuator(port).set(Command(-0.5, 0.0))
    assert port.lines == ["W -0.500 -0.500"]


def test_ackermann_line_format():
    port = FakePort()
    ack_actuator(port).set(Command(0.5, 1.0))  # 1 m radius
    assert port.lines == ["D 0.2915 0.500"]  # atan(0.30 * 1.0)


def test_ackermann_saturated_steer_is_clamped():
    port = FakePort()
    ack_actuator(port).set(Command(0.5, 10.0))
    assert port.lines == ["D 0.5236 0.500"]  # clamped to max_steer_rad = 30 deg


def test_stop_sends_s():
    port = FakePort()
    diff_actuator(port).stop()
    assert port.written == b"S\n"


def test_precision_is_fixed_decimals():
    port = FakePort()
    diff_actuator(port).set(Command(0.123456, 0.0))
    assert port.lines == ["W 0.123 0.123"]


def test_unsupported_mapper_output_raises():
    class WeirdMapper:
        def map(self, cmd):
            return "nonsense"

    with pytest.raises(TypeError):
        SerialActuator(WeirdMapper(), FakePort()).set(Command(0.5, 0.0))


def test_write_error_propagates_and_stop_is_attempted():
    port = FakePort(fail_on=b"W")
    safe = SafeActuator(diff_actuator(port), max_speed_mps=1.0)
    safe.enable()
    with pytest.raises(OSError):
        safe.set(Command(0.5, 0.0))
    # the wrapper attempted a stop before re-raising
    assert port.written.endswith(b"S\n")


def test_safe_actuator_disabled_turns_set_into_stop():
    port = FakePort()
    safe = SafeActuator(diff_actuator(port), max_speed_mps=1.0)
    safe.set(Command(0.5, 0.0))  # never enabled
    assert port.lines and all(not line.startswith("W") for line in port.lines)


def test_safe_actuator_timeout_stops():
    port = FakePort()
    t = [0.0]
    safe = SafeActuator(diff_actuator(port), max_speed_mps=1.0,
                        timeout_s=0.3, clock=lambda: t[0])
    safe.enable()
    safe.set(Command(0.5, 0.0))
    assert port.lines[-1] == "W 0.500 0.500"
    t[0] += 0.31
    safe.tick()
    assert port.lines[-1] == "S"
    assert safe.timed_out


# ------------------------------------------------------- the pyserial crash --
# The robot died on the Jetson on 10 Nov 2026 with
#     ModuleNotFoundError: No module named 'serial'
# raised from open_serial_port() AFTER the CSI camera had already opened, so the
# traceback sat directly under the Argus log and read like a camera fault. The
# cause was duller: pyserial was never declared in requirements-jetson.txt (the
# laptop only had it as a platformio transitive dependency).
import sys

import mav.io.actuator as A


@pytest.fixture
def no_pyserial(monkeypatch):
    """Make `import serial` fail, the way a venv without pyserial behaves.

    sys.modules[name] = None is the documented way to force an ImportError;
    monkeypatch restores the real module afterwards.
    """
    monkeypatch.setitem(sys.modules, "serial", None)


def test_missing_pyserial_names_the_install_command(no_pyserial):
    """A bare ModuleNotFoundError sends the operator looking at the wrong thing."""
    with pytest.raises(A.SerialUnavailable) as exc:
        A.require_serial()
    msg = str(exc.value)
    assert "pyserial" in msg
    assert "pip install pyserial" in msg
    # It must say explicitly that this is not the camera: the camera opened first.
    assert "NOT a camera" in msg


def test_missing_pyserial_is_not_an_oserror(no_pyserial):
    """robot.main catches OSError around the loop for mid-run cable faults.

    A startup dependency problem must not be reported as a runtime fault.
    """
    with pytest.raises(A.SerialUnavailable):
        A.require_serial()
    assert not issubclass(A.SerialUnavailable, OSError)


def test_open_serial_port_wraps_the_missing_import(no_pyserial):
    with pytest.raises(A.SerialUnavailable, match="pyserial is not installed"):
        A.open_serial_port("/dev/ttyACM0")


def test_list_serial_ports_wraps_the_missing_import(no_pyserial):
    with pytest.raises(A.SerialUnavailable, match="pyserial is not installed"):
        A.list_serial_ports()


def test_bad_port_reports_which_ports_do_exist():
    """Wrong port is the next failure: genuine Nano is ACM, a clone is USB."""
    with pytest.raises(A.SerialUnavailable) as exc:
        A.open_serial_port("/dev/definitely-not-a-real-port")
    msg = str(exc.value)
    assert "cannot open serial port" in msg
    assert "definitely-not-a-real-port" in msg
    # Either the real ports, or an explicit "nothing is plugged in".
    assert "Ports that do exist" in msg or "No serial ports found" in msg


def test_list_serial_ports_hides_onboard_uart_ports():
    """A desktop exposes dozens of /dev/ttyS* ports that are never an Arduino.

    Listing them buries the one port that matters, so the list keeps only real
    USB devices (or, where USB metadata is missing, the ACM/USB/COM families).
    """
    ports = A.list_serial_ports()
    assert isinstance(ports, list)
    for name in ports:
        assert not name.startswith("/dev/ttyS"), f"onboard UART {name} leaked into the list"


def test_both_requirement_files_declare_pyserial():
    """The root cause, pinned: an undeclared dependency fails only at runtime.

    The Jetson venv is created with --system-site-packages and installs only
    requirements-jetson.txt, so anything missing there is missing on the robot.
    """
    from mav.config import repo_root

    root = repo_root()
    for name in ("requirements-jetson.txt", "requirements-dev.txt"):
        lines = [
            ln.split("#")[0].strip()
            for ln in (root / name).read_text().splitlines()
        ]
        assert "pyserial" in lines, f"{name} must declare pyserial explicitly"


# ------------------------------------------------------- permission, not wiring --
def test_permission_denied_is_diagnosed_as_rights_not_wiring(monkeypatch):
    """The confusing case: the port is RIGHT THERE in the list, yet cannot open.

    "cannot open /dev/ttyUSB0" followed by "ports that exist: /dev/ttyUSB0"
    reads like nonsense. It means the user is not in `dialout`, which is the
    default on a fresh JetPack install - so the message has to say that, and say
    that the fix needs a fresh login.
    """
    class FakeSerial:
        class SerialException(Exception):
            pass

        @staticmethod
        def Serial(port, baud, timeout=0):
            raise FakeSerial.SerialException(
                "[Errno 13] could not open port /dev/ttyUSB0: "
                "[Errno 13] Permission denied: '/dev/ttyUSB0'"
            )

    monkeypatch.setitem(sys.modules, "serial", FakeSerial)
    with pytest.raises(A.SerialUnavailable) as exc:
        A.open_serial_port("/dev/ttyUSB0")
    msg = str(exc.value)
    assert "permission" in msg.lower()
    assert "dialout" in msg
    assert "log out" in msg.lower() or "LOG OUT" in msg
    # It must NOT be reported as a missing port: the port exists.
    assert "Ports that do exist" not in msg


# ------------------------------------------- the WCH vendor driver's port name --
def test_vendor_ch341_driver_port_is_not_dropped(monkeypatch):
    """WCH's own out-of-tree driver (`usb_ch341`) makes /dev/ttyCH341USB0.

    Some Jetson images carry it. It does NOT create /dev/ttyUSB0, and it does not
    always expose vid/pid through sysfs, so a filter that knows only ttyUSB/ttyACM
    reports "no Arduino found" while the board sits there bound to a driver.
    That is exactly what happened on the Jetson on 10 Nov 2026: lsusb -t showed
    `Driver=usb_ch341` and there was no /dev/ttyUSB* anywhere.
    """
    import types

    class P:
        def __init__(self, device, vid=None):
            self.device, self.vid = device, vid

    class FakeListPorts:
        @staticmethod
        def comports():
            return [P("/dev/ttyS0"), P("/dev/ttyS1"), P("/dev/ttyCH341USB0")]

    fake_serial = type("S", (), {})()
    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    tools = types.ModuleType("serial.tools")
    tools.list_ports = FakeListPorts
    monkeypatch.setitem(sys.modules, "serial.tools", tools)
    monkeypatch.setattr(A, "_dev_nodes", lambda *a, **k: [])

    assert A.list_serial_ports() == ["/dev/ttyCH341USB0"]


def test_onboard_uart_is_still_excluded_from_the_fallback(monkeypatch):
    """The widened name list must not let /dev/ttyS* back in.

    A desktop exposes 32 of them and they can never be the Arduino.
    """
    import types

    class P:
        def __init__(self, device):
            self.device, self.vid = device, None

    class FakeListPorts:
        @staticmethod
        def comports():
            return [P(f"/dev/ttyS{i}") for i in range(32)]

    monkeypatch.setitem(sys.modules, "serial", type("S", (), {})())
    tools = types.ModuleType("serial.tools")
    tools.list_ports = FakeListPorts
    monkeypatch.setitem(sys.modules, "serial.tools", tools)
    monkeypatch.setattr(A, "_dev_nodes", lambda *a, **k: [])

    assert A.list_serial_ports() == []


def test_flash_script_finds_the_vendor_port_name():
    """flash_firmware.sh must not glob only ttyACM/ttyUSB.

    It now asks the same Python helper the runner uses, so the two cannot
    disagree; the glob fallback also covers /dev/ttyCH341*.
    """
    from mav.config import repo_root

    script = (repo_root() / "scripts" / "flash_firmware.sh").read_text()
    assert "list_serial_ports" in script, (
        "the flash script should reuse the runner's port enumeration"
    )
    assert "ttyCH341" in script, "the glob fallback must cover WCH's driver name"


def test_port_pyserial_cannot_see_is_still_found(monkeypatch):
    """THE bug, exactly as it happened on the Jetson on 10 Nov 2026.

    pyserial's comports() globs a hardcoded list - ttyS*, ttyUSB*, ttyXRUSB*,
    ttyACM*, ttyAMA*, rfcomm*, ttyAP* (serial/tools/list_ports_linux.py). It does
    NOT know /dev/ttyCH341*, the name WCH's out-of-tree driver creates. So the
    device was invisible to pyserial and list_serial_ports() returned [] while
    `ls /dev/ttyCH341*` clearly showed the port. `--list-ports` found it only
    because the bash fallback globbed the name itself.

    A filter cannot fix this: the device never reaches the filter. Only globbing
    /dev does.
    """
    import types

    class EmptyListPorts:
        @staticmethod
        def comports():
            return []          # pyserial sees nothing at all

    monkeypatch.setitem(sys.modules, "serial", type("S", (), {})())
    tools = types.ModuleType("serial.tools")
    tools.list_ports = EmptyListPorts
    monkeypatch.setitem(sys.modules, "serial.tools", tools)
    monkeypatch.setattr(A, "_dev_nodes", lambda *a, **k: ["/dev/ttyCH341USB0"])

    assert A.list_serial_ports() == ["/dev/ttyCH341USB0"]


def test_dev_nodes_globs_the_vendor_name_and_skips_onboard_uart(tmp_path):
    """The glob is the ground truth for 'does this device node exist'."""
    for name in ("ttyCH341USB0", "ttyUSB0", "ttyACM0", "ttyS0", "ttyS1", "null"):
        (tmp_path / name).write_text("")

    found = [p.rsplit("/", 1)[-1] for p in A._dev_nodes(str(tmp_path))]
    assert found == ["ttyACM0", "ttyUSB0", "ttyCH341USB0"] or sorted(found) == sorted(
        ["ttyACM0", "ttyUSB0", "ttyCH341USB0"]
    )
    assert "ttyS0" not in found
    assert "null" not in found


def test_results_are_deduplicated(monkeypatch):
    """A port can come from both pyserial and the glob; list it once."""
    import types

    class One:
        device, vid = "/dev/ttyUSB0", 0x1A86

    class LP:
        @staticmethod
        def comports():
            return [One()]

    monkeypatch.setitem(sys.modules, "serial", type("S", (), {})())
    tools = types.ModuleType("serial.tools")
    tools.list_ports = LP
    monkeypatch.setitem(sys.modules, "serial.tools", tools)
    monkeypatch.setattr(A, "_dev_nodes", lambda *a, **k: ["/dev/ttyUSB0"])

    assert A.list_serial_ports() == ["/dev/ttyUSB0"]


def test_serial_actuator_remembers_the_last_line_written():
    """The status line must show what was sent, not "-", once a real port is in use.

    `line=-` whenever a real serial port was open meant the operator could not tell
    "the policy commanded stop" from "the command never reached the board" - which
    is exactly the question asked while debugging a robot that will not move.
    """
    port = FakePort()
    act = diff_actuator(port)
    assert act.last_line == "", "nothing sent yet"

    act.set(Command(0.25, 0.0))
    assert act.last_line == "W 0.250 0.250"

    act.stop()
    assert act.last_line == "S", "a stop must be visible too"

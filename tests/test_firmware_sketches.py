"""Guard the .ino sketches against the bugs that unit tests never caught.

`MavCore` is tested on the laptop (scripts/test_firmware_core.sh), but the
sketches themselves are only ever compiled, never executed - so a wrong constant
in an .ino is invisible until the car is on the track. That already happened
once: `final.ino` had `SERVO_US_NEUTRAL = 0`, and `Servo::writeMicroseconds()`
clamps to MIN_PULSE_WIDTH = 544, so "boot = neutral" actually booted the steering
HARD OVER to one side.

These are source-level assertions, deliberately. They are not a substitute for a
hardware test; they are a trap for the specific mistake of filling in a
calibration constant with a placeholder that looks like a number.
"""
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
FW = REPO / "firmware" / "arduino"
PROTOTYPE = (FW / "prototype" / "src" / "prototype.ino").read_text()
FINAL = (FW / "final" / "src" / "final.ino").read_text()

# The Servo library's own limits (Servo/src/Servo.h), which is what writeMicroseconds
# clamps to. A neutral outside them is silently not neutral.
SERVO_MIN_PULSE_US = 544
SERVO_MAX_PULSE_US = 2400


def _int_const(source, name):
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"const int {name}"):
            return int(stripped.split("=")[1].split(";")[0].strip())
    raise AssertionError(f"{name} not found as a const int")


def test_servo_neutral_is_a_valid_pulse_width():
    """0 is not neutral: writeMicroseconds(0) clamps to 544 us = hard lock."""
    neutral = _int_const(FINAL, "SERVO_US_NEUTRAL")
    assert SERVO_MIN_PULSE_US <= neutral <= SERVO_MAX_PULSE_US, (
        f"SERVO_US_NEUTRAL={neutral} would be clamped by the Servo library; "
        "the robot would boot with the wheels turned, not straight"
    )
    assert neutral == 1500  # the library's DEFAULT_PULSE_WIDTH until measured


def _function_body(source, signature):
    """The text of a function DEFINITION, by matching braces.

    Must skip forward declarations: `final.ino` declares `void apply_outputs();`
    near the top, and a naive `index(signature)` finds that one and then parses
    the next function it can see.
    """
    start = 0
    while True:
        start = source.index(signature, start)
        after = start + len(signature)
        open_brace = source.find("{", after)
        if open_brace == -1:
            raise AssertionError(f"{signature} has no definition")
        # A definition is `signature {` (whitespace only between); a declaration
        # is `signature ;`.
        between = source[after:open_brace]
        if between.strip() == "":
            break
        start = after
    depth = 0
    for i in range(open_brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace + 1:i]
    raise AssertionError(f"unbalanced braces after {signature}")


def test_uncalibrated_servo_cannot_be_commanded():
    """A zero range must be detected, not divided into nonsense."""
    assert "SERVO_CALIBRATED" in FINAL
    # The gate has to be in apply_outputs, i.e. on the path a real "D" line
    # takes, and it must actually guard - `if (!SERVO_CALIBRATED)` - not merely
    # be mentioned in a comment nearby.
    apply_body = _function_body(FINAL, "void apply_outputs()")
    assert "if (!SERVO_CALIBRATED)" in apply_body, (
        "apply_outputs() must refuse to steer on an unmeasured servo range"
    )
    # ...and the refusal must come BEFORE the microseconds are computed, or the
    # car has already been commanded by the time it bails out.
    gate = apply_body.index("if (!SERVO_CALIBRATED)")
    write = apply_body.index("steering.writeMicroseconds(clamp_int(")
    assert gate < write


@pytest.mark.parametrize("source,name", [
    (PROTOTYPE, "prototype"),
    (FINAL, "final"),
])
def test_speed_calibration_gap_is_not_silent(source, name):
    """MPS_TO_PWM == 0 makes every command PWM 0: valid commands, no movement."""
    assert "SPEED_CALIBRATED" in source
    # Announced at boot, so the operator is not left guessing.
    setup_body = _function_body(source, "void setup()")
    assert "SPEED_CALIBRATED" in setup_body, f"{name}.ino must warn at boot"
    assert "Serial.println" in setup_body


def test_firmware_never_boots_into_a_moving_state():
    for source in (PROTOTYPE, FINAL):
        setup_body = _function_body(source, "void setup()")
        stop = "stop_motors()" if "stop_motors" in source else "stop_motor()"
        assert stop in setup_body, "setup() must stop the motors"


def test_the_e_stop_split_is_documented_where_it_matters():
    """The e-stop cuts the motor rail; the Jetson stays powered (user, 9 Nov 2026).

    Guards against someone "improving" the firmware by cutting the serial link or
    assuming software can stop the car: the hardware e-stop is path one of three.
    """
    protocol = (FW / "protocol.md").read_text()
    assert "hardware emergency" in protocol.lower()
    assert "watchdog" in protocol.lower()


# ---------------------------------------------------- the 10 Nov 2026 rewire --
# The robot turned out to be a Nano with 2x L298N, and its own pin map. The
# sketch used to carry a two-channel Uno map on pins 2/4/7/8, which is the wrong
# pins on that hardware. These tests pin the map that is actually wired.
import re as _re


def _const_ints(source):
    """{name: value} for `const int NAME = ...;`, with A0-A7 resolved to 14-21.

    The ATmega328P maps the analog pins onto digital numbers 14+, which is what
    pinMode()/digitalWrite() actually use, so resolving them here lets the
    duplicate check work on real pin numbers instead of on names.
    """
    out = {}
    for line in source.splitlines():
        stripped = line.strip()
        m = _re.match(r"const\s+int\s+(\w+)\s*=\s*([^;]+);", stripped)
        if not m:
            continue
        raw = m.group(2).strip()
        analog = _re.fullmatch(r"A(\d)", raw)
        out[m.group(1)] = 14 + int(analog.group(1)) if analog else int(raw)
    return out


def test_prototype_pin_map_has_no_duplicates():
    """Two functions on one pin means one of them silently does nothing."""
    pins = _const_ints(PROTOTYPE)
    used = {k: v for k, v in pins.items() if k.startswith("PIN_")}
    assert used, "no PIN_ constants found in prototype.ino"
    seen = {}
    for name, num in used.items():
        assert num not in seen, f"{name} and {seen[num]} both use pin {num}"
        seen[num] = name


def test_every_enable_pin_can_actually_do_pwm():
    """analogWrite() on a non-PWM pin is a plain on/off: speed control vanishes.

    On the ATmega328P only these digital pins are PWM-capable.
    """
    nano_pwm = {3, 5, 6, 9, 10, 11}
    pins = _const_ints(PROTOTYPE)
    enables = {k: v for k, v in pins.items() if "EN" in k}
    assert len(enables) == 4, f"expected 4 enable pins (2x L298N), got {enables}"
    for name, num in enables.items():
        assert num in nano_pwm, (
            f"{name} = pin {num} is not PWM-capable; analogWrite() would be on/off"
        )


def _strip_comments(source, marker="//"):
    """Code only. Comments explain history, and history mentions the old values.

    Without this, a comment saying "this was FULL_PWM until 10 Nov" fails a test
    that forbids FULL_PWM - the test would then punish good documentation.
    """
    return "\n".join(
        line.split(marker)[0] for line in source.splitlines()
    )


def test_prototype_targets_the_nano():
    """The robot is a Nano. `board = uno` uploads with the wrong settings."""
    ini = (FW / "prototype" / "platformio.ini").read_text()
    active = _strip_comments(ini, ";")
    assert "nanoatmega328" in active, "prototype must build for the Nano"
    assert "board = uno" not in active, (
        "the ini comment may mention the old board, but the setting must not be it"
    )


def test_speed_is_proportional_not_pinned_to_full():
    """FULL_PWM would throw away speed control, and with it the whole model.

    The reference sketch this firmware replaced sent 255 for ANY non-zero
    command. That makes the policy's corner speed schedule, the max_speed_mps
    ceiling and MPS_TO_PWM/PWM_DEADBAND all do nothing - the car is always at
    full throttle. Speed must come from the command.
    """
    code = _strip_comments(PROTOTYPE)
    assert "FULL_PWM" not in code, (
        "prototype.ino must scale PWM by the commanded speed, not saturate it"
    )
    body = _function_body(PROTOTYPE, "int mps_to_pwm(float mps)")
    assert "speed_to_pwm" in body
    # And the measured constants must reach it.
    assert "MPS_TO_PWM" in body
    assert "PWM_DEADBAND" in body


def test_the_direction_invert_does_not_change_the_magnitude():
    """A reversed motor must be reversed, not also made slower."""
    body = _function_body(PROTOTYPE, "void set_motor(int en_pin, int in1_pin, int in2_pin, int pwm, bool invert)")
    assert "if (invert) pwm = -pwm;" in body
    # The duty cycle comes from abs(pwm) - sign already consumed by the IN pins.
    assert "abs(pwm)" in body


def test_watchdog_outlives_the_control_loop():
    """The bug this guards: TIMEOUT_MS = 30 against a ~31 ms control loop.

    With a timeout shorter than the command period the motors get PWM for a few
    tens of milliseconds, then the watchdog drops them to 0, then the next
    command arrives: the car buzzes and its effective speed is a fraction of
    what was asked. It also makes a speed measurement meaningless, because the
    duty cycle depends on host jitter rather than on the command.

    The control loop runs at `robot.rate_hz` (20 Hz -> 50 ms), and the measured
    loop is ~31 ms, so the watchdog needs real headroom, not a near miss.
    """
    core = (FW / "lib" / "MavCore" / "src" / "MavCore.h").read_text()
    m = _re.search(r"#define\s+MAV_WATCHDOG_MS\s+(\d+)", core)
    assert m, "MAV_WATCHDOG_MS not found"
    timeout_ms = int(m.group(1))

    import yaml

    cfg = yaml.safe_load((REPO / "configs" / "prototype.yaml").read_text())
    hz = float((cfg.get("robot") or {}).get("rate_hz", 20.0))
    period_ms = 1000.0 / hz
    assert timeout_ms >= 4 * period_ms, (
        f"watchdog {timeout_ms} ms is too close to the {period_ms:.0f} ms command "
        "period; the motors would stutter between commands"
    )


def test_the_sketches_do_not_reimplement_the_speed_map():
    """One implementation, in MavCore, where the laptop can test it."""
    for source, name in ((PROTOTYPE, "prototype"), (FINAL, "final")):
        body = _function_body(source, "int mps_to_pwm(float mps)")
        assert "* MPS_TO_PWM" not in body, (
            f"{name}.ino still does the arithmetic itself instead of calling MavCore"
        )


# ------------------------------------------------------- flash_firmware.sh --
def test_list_ports_does_not_depend_on_platformio():
    """The Jetson has no pio (the AVR toolchain is x86-only) - and is exactly
    where you need to find out which port the Nano landed on.

    `--list-ports` is pure bash globbing, so its block must come BEFORE the
    `command -v pio` guard. Asserted on the script's structure rather than by
    running it, because the script sources .venv/bin/activate itself: any PATH
    a test sets is undone, and pio is found on a dev machine either way.
    """
    script = (REPO / "scripts" / "flash_firmware.sh").read_text()
    list_block = script.find('if [ "$list_ports" -eq 1 ]')
    pio_guard = script.find("command -v pio")
    assert list_block != -1, "--list-ports handling not found"
    assert pio_guard != -1, "pio guard not found"
    assert list_block < pio_guard, (
        "--list-ports sits behind the pio guard; it would fail on the Jetson, "
        "which has no pio"
    )


def test_list_ports_actually_runs_and_says_something():
    """Either a real port or an explicit 'nothing plugged in' - never silence."""
    import subprocess

    result = subprocess.run(
        ["bash", str(REPO / "scripts" / "flash_firmware.sh"), "prototype", "--list-ports"],
        capture_output=True, text=True, cwd=str(REPO),
    )
    assert result.returncode == 0, result.stderr
    out = result.stdout.strip()
    assert out.startswith("/dev/tty") or "No serial ports found" in out


def test_flash_script_never_hardcodes_a_port_name():
    """AGENTS.md: do not hardcode serial port names.

    A genuine Nano is /dev/ttyACM*, a CH340 clone is /dev/ttyUSB* - any fixed
    name is wrong on half the boards.
    """
    script = (REPO / "scripts" / "flash_firmware.sh").read_text()
    for line in script.splitlines():
        code = line.split("#")[0]
        assert "/dev/ttyACM0" not in code, f"hardcoded port in: {line}"
        assert "/dev/ttyUSB0" not in code, f"hardcoded port in: {line}"

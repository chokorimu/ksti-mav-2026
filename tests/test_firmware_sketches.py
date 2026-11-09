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

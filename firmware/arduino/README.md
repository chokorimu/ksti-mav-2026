# Arduino Firmware (MCU code)

This directory holds the microcontroller firmware: the Arduino sketch that
drives the motor driver and the steering servo, commanded by the Jetson over
USB serial. The wire contract is `protocol.md` in this directory — read it
first, it is the single source of truth for both sides.

**Status (7 Nov 2026):** the shared core is written and unit-tested on the
laptop; the two sketches are **scaffolds** with the Q1 hardware constants
deliberately empty. Blocked on Q1 in `docs/open-questions.md` (Arduino model,
motor driver model, pins, servo model + angle range, battery/BEC, e-stop
wiring). Do not guess these; ask the user and record the answers in
`docs/hardware.md` first.

## Architecture (decided 7 Nov 2026)

- The Jetson is the CV brain. It never touches the 40-pin header for
  actuators (3.3 V logic, mA-level current, not real-time).
- **The Jetson maps, the Arduino executes.** `DifferentialMapper` /
  `AckermannMapper` (`src/mav/control/mapping.py`, already unit-tested) turn
  the vehicle-agnostic `Command(speed, curvature)` into hardware values; the
  Arduino only converts those to PWM / servo microseconds. The mapping logic
  exists once, in Python — never duplicated in C++.
- Host side: `SerialActuator` in `src/mav/io/actuator.py`, wrapped in the
  existing `SafeActuator` (stopped default, clamp, command timeout).

## Two variants, one core

The two robots differ in hardware, so there are two sketches — but NOT two
copies. Identical in both: protocol parsing, watchdog, safety. Different:
pin map and output stage (PWM channels vs servo + motor).

| Variant | Hardware | Speaks | Sketch |
|---|---|---|---|
| `prototype` | 4 motors, one per wheel, skid steer | `W <left> <right>` | `prototype/prototype.ino` |
| `final` | 1 rear motor + 1 steering servo | `D <steer_rad> <speed>` | `final/final.ino` |

Naming follows the repo profiles (`configs/`, `models/`, `scripts/run.sh`):
`prototype` and `final` — not "production", not "deploy".

## Layout

```
firmware/arduino/
  README.md          this file
  protocol.md        the serial contract, mirrored by src/mav/io/actuator.py
  MavCore/           shared Arduino library: protocol.md parser + watchdog.
                     Board-agnostic C++ (no Arduino headers), so it is
                     unit-tested on the laptop with g++ - see below.
  prototype/
    prototype.ino    4-motor skid steer, speaks "W". SCAFFOLD: Q1 constants empty.
  final/
    final.ino        steering servo + rear motor, speaks "D". SCAFFOLD: Q1 constants empty.
  platformio.ini     [env:prototype] + [env:final]; written once Q1 names the board
```

Build system (planned): **PlatformIO** (`pip install platformio`), because it
makes the variant a build flag instead of a code copy — the same "select via
configuration, not by editing code" rule the Python profiles follow:

```
pio run -e prototype -t upload
pio run -e final -t upload
```

`platformio.ini` waits for Q1: the board model decides the platform/fqbn.

### Scaffold rules (why the .ino constants are empty)

- Every unknown is an explicit `TODO(Q1)` with value `0` / `-1`. Nothing is
  guessed: pin numbers, PWM ranges and servo ranges are hardware facts.
- Flashed as-is, a scaffold cannot move anything (zero PWM, neutral/zero
  servo, no-op output stage). That is the correct fail-safe: firmware that
  knows nothing about the hardware must not be able to drive it.
- The sketches do not compile until the Q1 block is filled and the toolchain
  is installed. The shared core below compiles and passes tests TODAY.

## Safety rules the firmware must follow

(from `protocol.md` and AGENTS.md rule 4)

- Boot state: all motors 0, servo neutral — before anything else.
- Watchdog: no valid line within 500 ms -> stop. The hardware e-stop (cuts
  motor power) and the host-side `SafeActuator` timeout are the other two
  stop paths; none of them may be the only one.
- Malformed lines are ignored and counted; they do not reset the watchdog.
- Values are clamped in firmware too (defense in depth).
- Calibration constants (m/s -> PWM, servo microseconds range, deadzone) are
  hardware facts: measured wheels-lifted, recorded in `docs/hardware.md`.

## Testing order (AGENTS.md rule 5)

1. Shared core, on the laptop (done 7 Nov 2026):

   ```
   cd firmware/arduino/MavCore
   g++ -std=c++11 -Wall -Wextra -I src test/test_protocol.cpp src/MavCore.cpp \
       -o /tmp/mav_proto_test && /tmp/mav_proto_test
   ```

   Covers: verb dispatch, CRLF tolerance, malformed-line counting, watchdog
   expiry and re-arming, boot state, buffer overflow rejection.
2. Host side: `tests/test_serial_actuator.py` with a fake serial port — done.
3. Firmware bench test, wheels-lifted: every motor spins the right way, stop
   on watchdog (pull the USB cable mid-command), stop on garbage input,
   e-stop cuts power.
4. Then the track.

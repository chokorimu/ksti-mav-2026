# Arduino Firmware (MCU code)

This directory holds the microcontroller firmware: the Arduino sketch that
drives the motor driver and the steering servo, commanded by the Jetson over
USB serial. The wire contract is `protocol.md` in this directory — read it
first, it is the single source of truth for both sides.

**Status (7 Nov 2026):** hardware confirmed — Arduino Uno, L298N, LiPo 3S
(see `docs/hardware.md`). Both variants **compile for real** with PlatformIO
(prototype: 4966 B flash / 276 B RAM; final: 5532 B / 317 B). Still open:
the servo model + range (final), and the wheels-lifted m/s→PWM calibration
(both). Record measured values in `docs/hardware.md` — do not guess.

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
pin map and output stage.

| Variant | Hardware | Speaks | Project |
|---|---|---|---|
| `prototype` | 4 motors, one per wheel, skid steer (2× L298N channels) | `W <left> <right>` | `prototype/` |
| `final` | 1 rear motor (L298N channel) + 1 steering servo | `D <steer_rad> <speed>` | `final/` |

Naming follows the repo profiles (`configs/`, `models/`, `scripts/run.sh`):
`prototype` and `final` — not "production", not "deploy".

## Layout

```
firmware/arduino/
  protocol.md        the serial contract, mirrored by src/mav/io/actuator.py
  lib/MavCore/       shared Arduino library: protocol.md parser + watchdog.
                     Board-agnostic C++ (no Arduino headers), unit-tested on
                     the laptop with g++ - see below.
  prototype/         PlatformIO project (Uno): src/prototype.ino
  final/             PlatformIO project (Uno): src/final.ino
```

Each variant is its own PlatformIO project (PlatformIO only builds `.ino`
files at the root of a project's `src/`), and both share `lib/MavCore` via
`lib_extra_dirs`. The variant is therefore a directory, not a code copy —
the shared core is compiled into both.

## Build / flash / monitor

Requires `pip install platformio` (in `requirements-dev.txt`).

One script does compile + upload. Run it on the laptop with the Uno over
USB (the AVR toolchain is x86_64 and is not verified on the Jetson):

```
scripts/flash_firmware.sh <prototype|final> [--port /dev/ttyACM0]
```

It auto-detects the port when exactly one Uno is plugged in;
`--compile-only` builds without uploading, `--list-ports` shows the
candidates, and everything after `--` is passed to `pio run`. The raw
PlatformIO commands, if you need them:

```
cd firmware/arduino/prototype
pio run                  # compile
pio run -t upload        # flash (add --upload-port if several ports exist)
pio device monitor       # serial monitor, 115200

cd ../final              # same three commands for the servo robot
```

`final` also pulls the official Arduino Servo library (`servo@1.3.0`, pinned
in `final/platformio.ini`); the AVR framework PlatformIO installs does not
bundle it.

## Safety rules the firmware follows

(from `protocol.md` and AGENTS.md rule 4)

- Boot state: all motors 0, servo neutral — before anything else.
- Watchdog: no valid line within 500 ms → stop. The hardware e-stop (cuts
  motor power) and the host-side `SafeActuator` timeout are the other two
  stop paths; none of them may be the only one.
- Malformed lines are ignored and counted; they do not reset the watchdog.
- Values are clamped in firmware too (defense in depth).
- L298N specifics: remove the ENA/ENB jumpers or PWM does nothing; common
  ground with the Arduino and battery is mandatory; full wiring notes in
  `docs/hardware.md`.

## Testing order (AGENTS.md rule 5)

1. Shared core, on the laptop (done):

   ```
   scripts/test_firmware_core.sh
   ```

   Covers: verb dispatch, CRLF tolerance, malformed-line counting, watchdog
   expiry and re-arming, boot state, buffer overflow rejection.
2. Real compile for the Uno (done 7 Nov 2026): `pio run` in both projects.
3. Host side: `tests/test_serial_actuator.py` with a fake serial port — done.
4. Firmware bench test, wheels-lifted: every motor spins the right way, stop
   on watchdog (pull the USB cable mid-command), stop on garbage input,
   e-stop cuts power, then measure m/s→PWM → `docs/hardware.md`.
5. Then the track.

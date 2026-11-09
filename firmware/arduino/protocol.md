# Serial protocol: Jetson <-> Arduino

Contract between `SerialActuator` (`src/mav/io/actuator.py`) and the Arduino
firmware (`firmware/arduino/`). Version 1, 7 Nov 2026. Both sides must be
changed together; this file is the single source of truth for the wire format.

## Physical layer

- USB serial (the Arduino's own USB port). No GPIO wiring between Jetson and
  Arduino: the Jetson header stays 3.3 V logic only.
- 115200 baud, 8N1. On native-USB boards (Leonardo, Micro, ESP32) the baud
  rate is ignored; on Uno/Nano (ATmega16U2) both sides must match.
- v1 is one-way: the Jetson sends, the Arduino listens. No telemetry yet.

## Framing

- One command per line, terminated by `\n`.
- The firmware MUST also tolerate `\r\n` (the Arduino Serial Monitor's default
  line ending is often "Both NL & CR" — bench testing depends on this).
- ASCII, space-separated fields, fixed decimals.

## Commands (Jetson -> Arduino)

| Line | Meaning |
|---|---|
| `W <left> <right>` | prototype (differential): left/right wheel speeds, m/s, signed, 3 decimals |
| `D <steer_rad> <speed>` | final (ackermann): steering angle in rad, 4 decimals + speed in m/s, 3 decimals, both signed |
| `S` | stop: all motors 0, servo neutral; holds until the next drive command |

Sign conventions (identical to `src/mav/control/mapping.py`): speed > 0 is
forward; steering angle > 0 and curvature > 0 mean a LEFT turn.

Which drive verb a build speaks is fixed per firmware variant:
`prototype/prototype.ino` speaks `W`, `final/final.ino` speaks `D`. The host
side picks the variant through the mapper it injects into `SerialActuator`
(`DifferentialMapper` -> `W`, `AckermannMapper` -> `D`).

Examples:

```
W 0.500 0.500      # prototype, straight, 0.5 m/s
W 0.400 0.600      # prototype, gentle left (inner wheel slower)
W -0.500 -0.500    # prototype, reverse
D 0.2915 0.500     # final, 0.29 rad left, 0.5 m/s
S                  # stop
```

## Watchdog (firmware side)

- If no valid line arrives within `WATCHDOG_MS` (default 500 ms), the firmware
  stops (same output state as `S`).
- **`WATCHDOG_MS` must be several times the host's command period.** The host
  sends at `robot.rate_hz` (20 Hz = 50 ms, measured ~31 ms per loop). A timeout
  shorter than the period makes the motors run for a few tens of milliseconds and
  then coast until the next command: the car buzzes, its effective speed depends
  on host jitter instead of on the command, and any speed measurement is garbage.
  A reference sketch seen on 10 Nov 2026 used 30 ms against a ~31 ms loop - do not
  copy that value. `tests/test_firmware_sketches.py` pins the margin.
- This is the second of three stop paths. The first is the hardware emergency
  stop (cuts motor power, independent of any software); the third is the
  host-side `SafeActuator` command timeout (0.3 s). No single path may be the
  only thing keeping the robot safe.

## Error handling (firmware side)

- Malformed lines (unknown verb, wrong field count, unparseable numbers) are
  ignored and counted. They do NOT reset the watchdog.
- Out-of-range values are clamped to the firmware's configured limits. This is
  defense in depth: the host (`SafeActuator` + the mappers) already clamps.

## Boot state (firmware side)

- Before anything else runs: all motor outputs 0, servo at neutral. The
  firmware must never boot into a moving state, and a reset mid-run must stop
  the robot, not continue it.

## Deliberately not in v1

- Telemetry / heartbeat from the Arduino. Add it when bench debugging needs
  to distinguish "Arduino dead" from "Arduino stopped by watchdog".
- Encoder feedback: the current hardware has no encoders, so speed control is
  open-loop (m/s -> PWM calibration, measured wheels-lifted, recorded in
  `docs/hardware.md`).

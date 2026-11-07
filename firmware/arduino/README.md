# Arduino Firmware (MCU code)

This directory holds the microcontroller firmware for the robot: the Arduino sketch that
drives the motor driver and the steering servo, commanded by the Jetson over USB serial.

**Status: not written yet.** Blocked on Q1 in `docs/open-questions.md`:
Arduino model, motor driver model, pins used, servo model + angle range, battery/BEC,
and the emergency-stop wiring. Do not guess these; ask the user and record the answers
in `docs/hardware.md` first.

## Architecture (decided)

- The Jetson is the CV brain. It never touches the 40-pin header for actuators
  (3.3 V logic, mA-level current, not real-time).
- The Arduino receives commands over USB serial and drives the motor driver + servo.
- Host side (Python): `SerialActuator` in `src/mav/io/actuator.py`, wrapped in the
  existing `SafeActuator` (stopped default, speed clamp, command timeout). The serial
  protocol between the two sides is still to be decided together with the firmware.

## Safety rules the firmware must follow

- Start from a stopped state: throttle 0, steering neutral.
- On serial timeout, invalid command, or any error: stop (throttle 0, neutral steering).
- The physical emergency stop cuts motor power in hardware (relay/MOSFET), independent
  of any software. The firmware must never be the only stop path.

## Planned layout (adjust once Q1 is answered)

```
firmware/arduino/
  README.md          (this file)
  <sketch>/          the .ino sketch, named after the Arduino model once known
  protocol.md        the serial protocol shared with src/mav/io/actuator.py
```

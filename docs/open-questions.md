# Open questions

Blocking questions for the user. Each entry names the task it blocks.
Ask all pending questions together at the end of a session, in one message.

Last updated: 7 November 2026.

## Q1 — Motor/servo driver and GPIO pins [PARTLY ANSWERED 7 Nov 2026]
Decided: Arduino over USB serial drives the motor driver and servo; the Jetson 40-pin
header is not used for actuators.
Still needed for the Arduino firmware and the power design:
- Arduino model (Uno/Nano/ESP32?) and motor driver model (L298N/DRV8833/TB6612/4WD shield?).
- Which Arduino pins, and the servo model + angle range.
- Battery type/voltage, and the 5 V rail for motors/servo (BEC/UBEC, never the Arduino 5 V pin).
- Emergency stop circuit: what does it cut? Recommendation: cut the motor power rail in
  hardware (relay/MOSFET), independent of any software.
Blocks: the Arduino firmware in `firmware/arduino/` (the host-side SerialActuator can be
written now).

## Q2 — Camera model and interface [ANSWERED 7 Nov 2026]
CSI, Raspberry Pi camera module. Exact module (v1/v2/v3/GS) to verify on the device.
No longer blocking: see the camera bring-up commands in docs/hardware.md.

## Q3 — Rear drivetrain [ANSWERED 7 Nov 2026]
The single rear motor drives both rear wheels through a mechanical differential.

## Q4 — Steering linkage [OPEN]
Is the steering linkage true Ackermann geometry? (Affects the Ackermann bonus, rule A1.)

## Q5 — Simulator [CLOSED 7 Nov 2026]
No external web simulator is used. Replaced by the in-repo **web dashboard for the
synthetic sim** (`src/mav/sim/dashboard/`), which serves the synthetic closed-loop
simulator's state, camera frame and controls over plain HTTP (stdlib only).
The recommendation above (a) is what got built: HTTP polling, no new dependency,
and the robot side needed no adapter at all because the sim already runs the real
`mav.app.step` loop. Blocks nothing any more.

## Q6 — Organizer answers (rules) [OPEN]
A1 (differential), A5 (extra sensors), A11 (start/finish and checkpoint colors),
plus checkpoint count and track map.

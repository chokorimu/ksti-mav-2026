# Open questions

Blocking questions for the user. Each entry names the task it blocks.
Ask all pending questions together at the end of a session, in one message.

Last updated: 7 November 2026.

## Q1 — Motor/servo driver and GPIO pins [PARTLY ANSWERED 7 Nov 2026]
Decided: Arduino over USB serial drives the motor driver and servo; the Jetson 40-pin
header is not used for actuators.
Answered 7 Nov 2026: **Arduino Uno**, motor driver **L298N**, battery **LiPo 3S**
(11.1 V nominal). Pin map chosen by us and written down in `docs/hardware.md`
(prototype: ENA=3, IN1=2, IN2=4 / ENB=11, IN3=7, IN4=8; final: servo=5, ENA=3,
IN1=2, IN2=4). If the robot is already wired differently, say so - the constants in
`firmware/arduino/src/*/` and the doc must both be corrected.
Still needed for the Arduino firmware and the power design:
- Servo model + its measured microsecond range (blocks the servo constants in
  `final/src/final.ino`).
- The servo 5 V rail: BEC/UBEC model and rating (never the L298N regulator, never
  the Arduino 5 V pin).
- Which supply feeds the Jetson itself (its own regulator/battery path - the motor
  rail must not brown it out).
- Emergency stop circuit: what does it cut? Recommendation: cut the motor power rail in
  hardware (relay/MOSFET), independent of any software.
Blocks: the servo constants + the wheels-lifted speed calibration in the firmware
(`firmware/arduino/`). The firmware itself compiles for the Uno already.

## Q2 — Camera model and interface [ANSWERED 7 Nov 2026]
CSI, Raspberry Pi camera module. Exact module (v1/v2/v3/GS) to verify on the device.
No longer blocking: see the camera bring-up commands in docs/hardware.md.

## Q3 — Rear drivetrain [ANSWERED 7 Nov 2026]
The single rear motor drives both rear wheels through a mechanical differential.

## Q4 — Steering linkage [ANSWERED 8 Nov 2026 — prototype]
**Prototype pakai differential (skid) steering**, final pakai **Ackermann** (servo on the
steering axle). Sudah sesuai desain: `DifferentialMapper` (prototype) dan `AckermannMapper`
(final) di `src/mav/control/mapping.py`, dipilih lewat `configs/vehicle.yaml` `drive:`.
Catatan: rule 5.1 mewajibkan steering axle untuk lomba, jadi `prototype` hanya untuk
pengembangan — bonus Ackermann [A1] hanya berlaku untuk `final`. Tidak lagi memblokir apa pun.

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

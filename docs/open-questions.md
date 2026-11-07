# Open questions

Blocking questions for the user. Each entry names the task it blocks.
Ask all pending questions together at the end of a session, in one message.

Last updated: 7 November 2026.

## Q1 — Motor/servo driver and GPIO pins [OPEN]
How are the motors and servo driven (driver on Jetson pins, PCA9685 over I2C, or a
microcontroller over USB serial)? Which pins?
Blocks: real motor/servo drivers, GPIO buttons, any actuator test with hardware.
Related: emergency stop circuit — what does it cut?

## Q2 — Camera model and interface [ANSWERED 7 Nov 2026]
CSI, Raspberry Pi camera module. Exact module (v1/v2/v3/GS) to verify on the device.
No longer blocking: see the camera bring-up commands in docs/hardware.md.

## Q3 — Rear drivetrain [ANSWERED 7 Nov 2026]
The single rear motor drives both rear wheels through a mechanical differential.

## Q4 — Steering linkage [OPEN]
Is the steering linkage true Ackermann geometry? (Affects the Ackermann bonus, rule A1.)

## Q5 — Simulator [OPEN, recommendation below]
What stack is the web simulator, and how does it exchange frames and commands?
Blocks: the simulator bridge (PROGRESS task 5).
Recommendation (7 Nov 2026): given the time pressure, do NOT adopt a heavyweight
simulator (Gazebo/Unity/Webots). Two efficient options, in order:
  a) If the web simulator's code can be touched: expose the camera view as an MJPEG
     stream and accept commands with one HTTP POST (or a single WebSocket carrying
     both). Robot side then needs only two thin adapters: `cv2.VideoCapture` on the
     MJPEG URL (reuses `open_video`'s path) and a small POST actuator. Perception,
     control and the state machine stay untouched.
  b) If it cannot be modified quickly: screen-record the simulator and run offline
     with `open_video` (already works). No closed loop, but zero integration work.
Either way the sim is secondary: the real camera is now known (CSI), so real
recordings + the synthetic generator cover algorithm testing.

## Q6 — Organizer answers (rules) [OPEN]
A1 (differential), A5 (extra sensors), A11 (start/finish and checkpoint colors),
plus checkpoint count and track map.

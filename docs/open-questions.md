# Open questions

Blocking questions for the user. Each entry names the task it blocks.
Ask all pending questions together at the end of a session, in one message.

Last updated: 7 November 2026.

## Q1 — Motor/servo driver and GPIO pins
How are the motors and servo driven (driver on Jetson pins, PCA9685 over I2C, or a
microcontroller over USB serial)? Which pins?
Blocks: real motor/servo drivers, GPIO buttons, any actuator test with hardware.
Related: emergency stop circuit — what does it cut?

## Q2 — Camera model and interface
CSI or USB? Resolution and FPS at which it will run?
Blocks: recording real video (PROGRESS task 1), detector calibration (task 2),
Jetson latency measurement (task 3).

## Q3 — Rear drivetrain
Does the single rear motor drive both rear wheels through a mechanical differential?

## Q4 — Steering linkage
Is the steering linkage true Ackermann geometry? (Affects the Ackermann bonus, rule A1.)

## Q5 — Simulator
What stack is the web simulator, and how does it exchange frames and commands?
Blocks: the simulator bridge (PROGRESS task 5).

## Q6 — Organizer answers (rules)
A1 (differential), A5 (extra sensors), A11 (start/finish and checkpoint colors),
plus checkpoint count and track map.

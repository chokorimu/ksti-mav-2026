# Research Plan – 1:10 Autonomous Vehicle for KSTI 2026

Status: initial draft. Items marked **(assumption)** must be confirmed by the user.

## 1. Team context and constraints

| Item | Decision |
|---|---|
| Robot computer | Jetson Orin Nano, SSH access only, no display. JetPack 6.2.3 (L4T R36.5) |
| Dev environment | Linux (priority), Windows (fallback) |
| Models | Must be able to switch between prototype and final versions |
| Team | Programmers with limited electrical engineering knowledge |
| Simulation | Web-based, but uses real computer vision. Stack and interface: **unknown** |

Competition on 12 November 2026, Technical Meeting on 8 November 2026. If these dates still hold, time is very short, so the work order in section 4 prioritizes a **robot that is reliable on the track** over elegant research.

## 2. Research questions

### RQ1 – Lane perception
The floor and markings change color (white or black markings), track width is not constant, and the dashed center line has gaps of at least 5 cm.
- Compare classical approaches (adaptive color threshold + line detection / sliding window) with learned approaches (lightweight segmentation).
- Measure: robustness to lighting, shadows, floor reflections, and changes in floor color; per-frame latency on the Jetson.
- Output: a method decision + a dataset of real recordings with labels as needed.

### RQ2 – Checkpoints, start/finish, obstacles
Green checkpoints (not necessarily marked), start/finish color uncertain, obstacles for ARC.
- Color detection must be parameterized, not hardcoded. Research fast color calibration at the venue (e.g. 1 minute during preparation).
- Obstacles: start with simple detection (free-space segmentation); use an object detection model only if needed.
- Count checkpoints correctly and in order, and tolerate duplicate detections.

### RQ3 – Planning and control
- Kinematic steering model (bicycle/Ackermann) for turning radius ≥ 1.5 m and track width 50 cm ± 10%.
- Two vehicle configs exist (prototype: 4 independently driven wheels; final: 1 steering servo + 1 rear motor). Control outputs a vehicle-agnostic `(speed, curvature)`; each actuator maps it to hardware. This keeps tuning on the prototype transferable.
- Candidate controllers: pure pursuit or Stanley for steering, PID for speed. Speed adapted to curvature.
- Trade-off from the scoring formula: `cp` matters more than the speed bonus, while leaving the track adds `0.8 * T_stage / N_cp` seconds. Choose a target speed that keeps the chance of leaving the track very low.
- A **state machine** is required: `BOOT → WAIT_START → RUN → STOPPED (emergency) → WAIT_START → RUN ...`. After a manual restart at the last checkpoint, the checkpoint count must not reset and the robot must find the lane again quickly (assumption: placed at the checkpoint, possibly at an angle).

### RQ4 – Headless Jetson
- Camera: CSI (GStreamer pipeline `nvarguscamerasrc`) vs USB (V4L2). The Jetson's OpenCV 4.8.0 has GStreamer support (so CSI capture via GStreamer works) but no CUDA in its build info, so classical CV runs on CPU; the GPU is used through TensorRT.
- No display: no `imshow`; use frame/video saving, logs, and an MJPEG stream for dev only. Working tools: SSH, tmux, VS Code Remote-SSH, rsync/git.
- Monitor performance and temperature: `tegrastats`, `nvpmodel`, `jetson_clocks`. Set the same power mode for testing and competition.
- Autostart at boot with a service (systemd) that waits for the physical start button, since no communication to the robot is allowed during a run.
- End-to-end latency (camera → actuator command) and jitter. Target measured values, not estimates.

### RQ5 – Development environment (Linux primary, Windows fallback)
- Two machine classes: x86_64 laptop and aarch64 Jetson. Separate dependency files (dev vs jetson). On the Jetson, packages such as PyTorch/TensorRT/OpenCV follow the JetPack version, which must be recorded first.
- Linux: a single setup script. Windows: use WSL2; set `.gitattributes` to LF line endings; do not hardcode serial port names or paths.
- Consider containers to standardize environments, noting that Jetson and laptop images differ.

### RQ6 – Prototype → final model
- Flow: train/test on laptop or sim → export (e.g. ONNX) → optimize for Jetson (e.g. TensorRT, lower precision if accuracy holds) → parity test.
- TensorRT 10.3 runtime and ONNX parser are installed; `trtexec` and the Python bindings were not seen in the package list and must be verified (see `docs/hardware.md`).
- Parity test: for the same set of frames, the output difference between prototype and final models must be below an agreed threshold; also measure FPS/latency.
- Profile selection via `configs/*.yaml` and one variable (e.g. `MAV_PROFILE`), not code branches. The `final` profile disables all network and debug components.

### RQ7 – Web simulation and sim-to-real gap
Suggested contract: the sim only replaces two interfaces.
```
Camera.read() -> frame           (SimCamera | RealCamera)
Actuator.set(steer, throttle)    (SimActuator | RealActuator)
```
All perception, planning, control, and state machine code is shared.
- Gaps to test: lighting/shadows, motion blur, lens distortion, camera height and angle, floor/marking colors, camera and actuator latency, real dynamics (traction, servo delay).
- Domain randomization in sim: floor color, marking color (white/black), varying track width, brightness.
- **Replay real recordings** through the same pipeline as regression tests without hardware.
- Physical robot parameters (wheel base, track width, steering limits) go into configuration and are used by both sim and control.

### RQ8 – Minimal electrical knowledge for programmers
See section 3. Output: `docs/hardware.md` with the real component list, power and signal diagrams, and verification by someone with electrical engineering knowledge.

## 3. Minimal electrical guide (for programmers)

This is a general guide; confirm with someone who knows electrical engineering before connecting hardware.

1. **Separate compute power from motor power.** Motors draw large currents and can cause voltage drops (brownout), causing the Jetson to reboot mid-track. Use separate regulators for the Jetson and the motors, and **connect the grounds together**.
2. **Do not power motors/servos from Jetson pins.** Pins are for signals only.
3. **Logic levels.** Jetson GPIO pins are 3.3 V and not 5 V tolerant. Signals from 5 V microcontrollers need a level shifter, or use USB serial to avoid the issue.
4. **Emergency stop.** The rules require a physical button. Design it to cut motor power in hardware, not only send a signal to software that may hang. Provide a separate start button.
5. **Batteries.** Use a fuse or current protection on the main path, watch polarity, and follow safe LiPo procedures. Check voltage before every attempt.
6. **Actuators.** Set steering angle and throttle limits in software so servos/chassis are not forced past their limits.
7. **Cables and connectors.** Loose connectors cause random resets. Lock cables and add strain relief. Plan so no part can detach (a detached part = attempt ends).
8. **Heat.** The Jetson and motor driver need airflow; test under sustained load for at least 5 minutes (the attempt length). The Jetson is currently in `MAXN_SUPER` (highest power mode, highest draw): size its regulator for it, or choose a lower mode, and use the same mode in tests and in competition.

Questions an electrical engineer must answer: which motor and driver are used, battery voltage and capacity, motor peak current, regulator for the Jetson, emergency stop circuit (what it cuts), pins used and their levels.

## 4. Work phases (priority order)

**P0 – Foundation (must come first)**
- Confirm open items (section 6) and the Technical Meeting results.
- Stable SSH access to the Jetson; camera readable; frames can be saved.
- Safe actuators: the robot can be stopped reliably via e-stop; steering/throttle can be controlled from a program with wheels lifted.

**P1 – Robot can complete the track**
- Lane perception → basic control pipeline; state machine with WAIT_START and resume.
- Run in sim and on real recordings with the same code; `prototype` profile on the Jetson.

**P2 – Reliability**
- Parameterized checkpoint and start/finish detection; adaptive speed; tests under different lighting and floor colors.
- Final: optimized model, autostart, fixed power mode, no network.
- Full dry run with the referee procedure (10 minutes preparation, start, violation, restart, emergency).

**P3 – Optional**
- ARC (obstacles), Ackermann if the platform supports it and the `STP = 0.25` bonus is worth it.

## 5. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Jetson brownout/reset when motors start | Attempt fails | Separate regulator, load test |
| Floor/marking color changes at venue | Perception fails | Color parameters, calibration during preparation, domain randomization |
| Rule ambiguities (A1, A5, A11) | Design violates rules | Ask the organizer before locking the design |
| Sim does not represent the real world | Misleading results | Replay real recordings, wheels-lifted tests |
| Final model behaves differently from prototype | Silent regression | Parity test |
| Time too short | Not enough testing | Follow P0 → P1 order first |
| Car part detaches | Attempt ends | Locking and inspection before start |

## 6. Open items

- How are the motors and servo driven (driver wired to Jetson pins, PCA9685 over I2C, or a microcontroller over USB serial)?
- Web simulator stack and how the sim exchanges frames/commands with the CV code (WebSocket/HTTP)?
- Camera and car platform used? Motor driver and steering controller?
- Is the robot Ackermann (0.25 bonus) or not?
- Organizer confirmation for A1, A5, A11.
- Test day dates and number of checkpoints/track map.

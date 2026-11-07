# AGENTS.md – 1:10 Autonomous Vehicle for KSTI 2026 (Mini Autonomous Vehicle)

Working language: English. Keep Indonesian names of official documents as they appear.

## Reference documents (read before working on the related topic)
- Full competition rules, scoring formulas, penalties, ambiguities: @docs/ksti-mav-rules.md
- Research plan, architecture, minimal electrical guide: @docs/research-plan.md

If a task touches an item marked **[A#]** in the rules, do not guess. Ask the user.

## Environment
- **Jetson Orin Nano: SSH only, no display.** Do not use `cv2.imshow`, GUI code, or tools that need a screen. To inspect results, save images/video or use an MJPEG stream in dev mode only.
- **JetPack 6.2.3 (L4T R36.5, Ubuntu 22.04 base).** Version-dependent facts (Python, CUDA, TensorRT, OpenCV build options) must be verified on the device and recorded in `docs/hardware.md`; do not assume them. Use NVIDIA aarch64 packages built for JetPack 6, never x86 wheels.
- Verified on the Jetson: Python 3.10.12, OpenCV 4.8.0 (GStreamer yes, CPU only), CUDA 12.6, cuDNN 9.3, TensorRT 10.3 runtime. TensorRT dev tools (`trtexec`, Python bindings) are not confirmed yet. Details in `docs/hardware.md`.
- Dev machines should match OpenCV 4.8.x and `numpy<2` so behavior is comparable to the Jetson.
- Check the power mode with `sudo nvpmodel -q`. It is currently `MAXN_SUPER`. Use the same mode for testing and competition, and measure performance with `tegrastats`.
- **Development: Linux is preferred**, Windows is supported as a fallback (ideally via WSL2). Write commands for bash. Use `pathlib`; do not hardcode paths or serial port names.
- CPU architectures differ (x86_64 laptop vs aarch64 Jetson). Separate dependencies per platform; do not assume the same pip wheels work on both.
- There is a **web-based simulator** that uses real computer vision. CV and control code must be identical between sim and the real robot; only the camera and actuator layers differ.

## Vehicle configurations
| Config | Drive | Steering | Notes |
|---|---|---|---|
| `prototype` | 4 motors, one per wheel | differential (skid) steering by wheel speeds | development only: rule 5.1 requires a steering axle |
| `deploy` | 1 motor on the rear wheel | 1 servo on the steering axle | competition hardware; Ackermann bonus depends on the steering linkage **[A1]** |

- Perception and control must output a **vehicle-agnostic command**: `(speed, curvature)` or `(speed, yaw_rate)`. Never output motor or servo values directly.
- Each actuator implementation maps that command to its hardware: differential config -> left/right wheel speeds; servo config -> steering angle (`atan(wheel_base * curvature)`) + motor speed.
- A single rear motor driving both rear wheels needs a mechanical differential, otherwise the wheels fight each other in turns. Confirm this with the user.

## Execution profiles
Select via configuration, not by editing code.

| Profile | Where | Model | Purpose |
|---|---|---|---|
| `sim` | laptop + web sim | prototype model | test algorithms without hardware |
| `prototype` | Jetson/laptop | prototype model (debug, overlay, full logging) | iteration and measurement |
| `deploy` | Jetson | optimized model (e.g. TensorRT), no debug | competition mode |

`deploy` rules: no network server, streaming, or SSH dependency while the robot runs. It starts automatically at boot and waits for the physical start button.

## Hard constraints from the competition (no design may violate these)
- Infrared sensors and line followers are banned. Do not add sensors other than the camera without organizer confirmation **[A5]**.
- Any communication to control the robot during a run is banned, wireless or wired.
- Omni and mecanum wheels are banned. A physical emergency stop button and a start button are required.
- Marking color (white/black) follows the floor. Do not hardcode checkpoint or start/finish colors **[A11]**.
- After start is pressed, no human intervention. After a violation, a human presses emergency stop, places the robot at the last checkpoint, and presses start again. **The program must resume (checkpoint count must not reset)**; verify this behavior on every state machine change.

## Working rules for agents
1. **Assume the programmers know little about electrical engineering.** When touching GPIO, motors, servos, batteries, or cables, explain briefly in plain language: voltage, current, logic levels, shared ground.
2. **Do not guess hardware facts** (Jetson model details, battery voltage, motor driver, pins). Ask, or request verification; record answers in `docs/hardware.md`.
3. **Give explicit warnings** before changes that could damage hardware: Jetson GPIO pins are 3.3 V, do not connect 5 V signals directly; do not power motors from Jetson pins; do not run actuators without a tested emergency stop path.
4. **Safe by default:** actuator code must start from a stopped state (throttle 0, steering neutral) and return there on error, frame loss, or exception.
5. **Test without hardware first:** sim or recorded real video, then Jetson with wheels lifted, then the track.
6. Any change to model or parameters that affects latency must include FPS/latency measured on the Jetson, not only on the laptop.
7. If a command is not defined below, do not invent it. Ask, or propose one and record it after approval.

## Repo layout (plan, adjust to reality)
```
AGENTS.md
docs/        ksti-mav-rules.md, research-plan.md, hardware.md
configs/     sim.yaml, prototype.yaml, deploy.yaml
src/         perception/, planning/, control/, io/ (camera, actuator: sim & real), sim/
models/      prototype/, deploy/
scripts/     setup_dev.sh, setup_jetson.sh, run.sh
tests/
```

## Implemented so far (no real hardware drivers yet)
- `src/mav/types.py`: `Command(speed m/s, curvature 1/m)`, `VehicleParams`, `load_vehicle_params()` (refuses to run while `configs/vehicle.yaml` has unmeasured `null` values).
- `src/mav/io/camera.py`: `Camera` protocol, `csi_pipeline()` (GStreamer, headless), `open_csi/open_usb/open_video`, `CallbackCamera` for the simulator.
- `src/mav/io/actuator.py`: `SafeActuator` (stopped by default, speed clamp, command timeout), `NullActuator`, `CallbackActuator` for the simulator.
- `src/mav/io/buttons.py`: `Event` and `ScriptedButtons`. **Real GPIO buttons are not written**: pins and wiring are unknown.
- `src/mav/control/mapping.py`: `DifferentialMapper` (prototype) and `AckermannMapper` (deploy).
- `src/mav/control/state_machine.py`: BOOT/WAIT_START/RUN/STOPPED; resume after emergency keeps the checkpoint count.
- `src/mav/app.py`: `step()`, one loop iteration shared by sim, prototype and deploy.
- `src/mav/perception/lane.py`: LaneDetector (numpy-only), offline runner
    `PYTHONPATH=src python -m mav.perception.lane <video> [--overlay out.mp4]`.
- `src/mav/planning/lane_policy.py`: LanePolicy (measurement -> Command, pure-pursuit). Both have tests on synthetic frames.
- `src/mav/sim/` (synthetic closed-loop simulator, no hardware): `world.py` (exact
    constant-curvature pose integration, lane polyline, camera model with taper),
    `render.py` (shared numpy-only frame renderer, also used by `tests/synthetic_lane.py`),
    `scenarios.py` (straight / offset / curve / s_curve), `runner.py` (`run_sim()` drives
    the real `mav.app.step` with the real detector and policy; only the camera and actuator
    layers are simulated). CLI: `PYTHONPATH=src python -m mav.sim.runner [scenario]
    [--steps N] [--dt S] [--seed S] [--overlay out.mp4] [--list]`. Tests: `tests/test_sim.py`.
- **Not written yet:** real motor/servo drivers, GPIO buttons, checkpoint detection, and the web-simulator bridge (the synthetic sim above is done). Do not invent them; ask for the hardware facts in `docs/hardware.md` first.

## Long-running work protocol (anti-drift)

State lives in files, not in memory. Context can be lost at any time.

1. Session start: read docs/PROGRESS.md first. Do not re-read the full
   rules/research docs; open only the section relevant to the current task.
2. Work on ONE task from PROGRESS.md at a time. Each task has a
   "done when" check (a test or command). Do not start the next task
   before the current one passes its check.
3. After each finished task (or every ~30 min of work):
   - run tests, then git commit with a clear message
   - update PROGRESS.md: mark done, write the exact next step
4. Never leave the repo broken at a stopping point. If you must stop
   mid-task, first commit work-in-progress (or stash), and write in
   PROGRESS.md: what is done, what is half-done, what to run next.
5. Blocked by an [A#] item or missing hardware fact:
   - append the question to docs/open-questions.md (with the task it blocks)
   - skip to the next unblocked task; do not stop the whole session
   - ask all pending questions together at the end, in one message
6. Before long exploration (reading many files, searching), keep notes short:
   write findings into PROGRESS.md instead of relying on the chat.
7. If a task needs more than ~10 files or several modules, split it into
   subtasks in PROGRESS.md before coding.
8. Re-read the "Hard constraints" section before any change to the
   state machine, actuators, or sensors.

## Commands
Run all commands from the repo root. Use bash.

- Dev setup (laptop, x86_64): `scripts/setup_dev.sh`
- Jetson setup (Orin Nano, aarch64): `scripts/setup_jetson.sh` (uses `--system-site-packages` so JetPack's OpenCV/TensorRT stay visible; do not pip-install x86 wheels there)
- Run a profile: `scripts/run.sh <sim|prototype|deploy>`. Currently this only loads and prints the profile config; the actual pipeline is not wired yet.
- Run sim: `scripts/run.sh sim [scenario]` (synthetic closed-loop sim; defaults from
  `configs/sim.yaml` section `sim:`). The web simulator's own launch command is _TBD_
  (its stack and frame/command interface are not yet known).
- Run on Jetson: `scripts/run.sh prototype` (debug on) or `scripts/run.sh deploy` (no debug, no network). Use the same `sudo nvpmodel` mode as the competition.
- Tests: `source .venv/bin/activate && pytest` (on the Jetson install pytest first: `pip install pytest`, or `sudo apt install python3-pytest`)
- Check JetPack (on the Jetson, before any version-dependent install): `cat /etc/nv_tegra_release` and `apt show nvidia-jetpack`

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
| `final` | 1 motor on the rear wheel | 1 servo on the steering axle | competition hardware; Ackermann bonus depends on the steering linkage **[A1]** |

- Perception and control must output a **vehicle-agnostic command**: `(speed, curvature)` or `(speed, yaw_rate)`. Never output motor or servo values directly.
- Each actuator implementation maps that command to its hardware: differential config -> left/right wheel speeds; servo config -> steering angle (`atan(wheel_base * curvature)`) + motor speed.
- A single rear motor driving both rear wheels needs a mechanical differential, otherwise the wheels fight each other in turns. Confirm this with the user.

## Execution profiles
Select via configuration, not by editing code.

| Profile | Where | Model | Purpose |
|---|---|---|---|
| `sim` | laptop + web sim | prototype model | test algorithms without hardware |
| `prototype` | Jetson/laptop | prototype model (debug, overlay, full logging) | iteration and measurement |
| `final` | Jetson | optimized model (e.g. TensorRT), no debug | competition mode |

`final` rules: no network server, streaming, or SSH dependency while the robot runs. It starts automatically at boot and waits for the physical start button.

## Perception config (per profile)

Perception parameters live in `configs/<profile>.yaml`, not in code:

| Section | Holds | Why it is config |
|---|---|---|
| `lane:` | `px_per_m`, `bottom_row_m`, `roi_top`, `max_band_span_m` | the camera mount is not fixed, so a camera move is a config edit |
| `checkpoint:` | hue bands + geometry | marking colours must not be hardcoded **[A11]** |

`src/mav/config.py` loads them (`load_lane_config(profile)`,
`load_checkpoint_config(profile)`); an omitted key keeps the dataclass default.
`python -m mav` validates both sections at boot, so a typo fails there with the
offending key named instead of silently mid-run. `sim.yaml` carries a `lane:`
section for reference only (the runner derives its own from `WorldConfig`) and
no `checkpoint:` section (hues come from the rendered scenario colours);
`final.yaml` has neither `dashboard:` nor `sim:`, so it cannot start a server.

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
configs/     sim.yaml, prototype.yaml, final.yaml
src/         perception/, planning/, control/, io/ (camera, actuator: sim & real), sim/
firmware/    arduino/ (MCU firmware: protocol.md + lib/MavCore shared core +
              prototype/ and final/ PlatformIO projects; compiles for the Uno,
              calibration + servo constants still open)
models/      prototype/, final/
scripts/     setup_dev.sh, setup_jetson.sh, run.sh
tests/
```

## Implemented so far (no real hardware drivers yet)
- `src/mav/types.py`: `Command(speed m/s, curvature 1/m)`, `VehicleParams`, `load_vehicle_params()` (refuses to run while `configs/vehicle.yaml` has unmeasured `null` values).
- `src/mav/io/camera.py`: `Camera` protocol, `csi_pipeline()` (GStreamer, headless), `open_csi/open_usb/open_video`, `CallbackCamera` for the simulator.
- `src/mav/io/actuator.py`: `SafeActuator` (stopped by default, speed clamp, command timeout), `NullActuator`, `CallbackActuator` for the simulator, and `SerialActuator` (mapper output -> ASCII lines over USB serial per `firmware/arduino/protocol.md`; the port is injected, so tests use a fake port and pyserial stays an optional, lazily-imported dependency).
- `src/mav/io/buttons.py`: `Event` and `ScriptedButtons`. **Real GPIO buttons are not written**: pins and wiring are unknown.
- `src/mav/control/mapping.py`: `DifferentialMapper` (prototype) and `AckermannMapper` (final).
- `src/mav/control/state_machine.py`: BOOT/WAIT_START/RUN/STOPPED; resume after emergency keeps the checkpoint count.
- `src/mav/app.py`: `step()`, one loop iteration shared by sim, prototype and final.
- `src/mav/config.py`: `load_lane_config()` / `load_checkpoint_config()` map the
    `lane:` and `checkpoint:` sections of `configs/<profile>.yaml` onto `LaneConfig` /
    `CheckpointConfig` (unknown keys, bad hue bands and out-of-range values raise;
    `null` disables an optional gate). `python -m mav` validates both sections at
    startup. Tests: `tests/test_config.py`.
- `src/mav/perception/lane.py`: LaneDetector (numpy-only), offline runner
    `PYTHONPATH=src python -m mav.perception.lane <video> [--overlay out.mp4]`.
- `src/mav/planning/lane_policy.py`: LanePolicy (measurement -> Command, pure-pursuit); `on_lost="hold"` default (was `"stop"`) in 8 Nov 2026. Both have tests on synthetic frames.
- `src/mav/sim/` (synthetic closed-loop simulator, no hardware): `world.py` (exact
    constant-curvature pose integration, lane polyline, camera model with taper),
    `render.py` (shared numpy-only frame renderer, also used by `tests/synthetic_lane.py`),
    `scenarios.py` (straight / offset / curve / s_curve), `runner.py` (`run_sim()` drives
    the real `mav.app.step` with the real detector and policy; only the camera and actuator
    layers are simulated). CLI: `PYTHONPATH=src python -m mav.sim.runner [scenario]
    [--steps N] [--dt S] [--seed S] [--overlay out.mp4] [--list]`. Tests: `tests/test_sim.py`.
- `src/mav/sim/dashboard/` (web dashboard for the sim, pure viewer + control channel):
    `state.py` (`SimState` -> JSON-safe dict, the browser contract), `bridge.py`
    (`Control`/`parse_control`, `Bridge`, `NullBridge`, `FakeBridge`), `server.py`
    (stdlib `http.server`: `/`, `/state.json`, `/lane.json`, `/frame.jpg`, `/scenarios`,
    `POST /control`), `static/` (vendored three.js + frontend). `lane.py`, `lane_policy.py`
    and `app.py` are untouched by it. Only the `sim` profile starts a server (`--dashboard`
    or `dashboard.enabled` in `configs/sim.yaml`); `final` has no such section and stays
    serverless. Tests: `tests/test_dashboard.py`, `tests/test_dashboard_server.py`.
- `src/mav/perception/obstacle.py`: cone detection (`ConeDetector`) + ARC scoring
    (`ConePassTracker`: a cone passed increments `cp`, because in the ARC the
    obstacles ARE the checkpoints, rules §8), and `src/mav/planning/avoid.py`:
    the pure swerve decision (`choose_offset`). Wired into `Driver`: the swerve
    target replaces `offset_m` before `LanePolicy`, and never leaves the lane
    (hit cone k=0.5 is cheaper than off-track k=0.8). Scope decision (user,
    9 Nov 2026): **ARC, not AR**. Tests: `tests/test_obstacle.py`.
- `firmware/arduino/`: `tests/test_firmware_sketches.py` guards the two sketches
    against constants that look calibrated but are not — `SERVO_US_NEUTRAL = 0`
    in `final.ino` used to boot the steering hard over (the Servo library clamps
    to 544 µs), and `MPS_TO_PWM = 0` made every speed command PWM 0. Both
    sketches now warn on serial at boot instead of failing silently.
- `mav::speed_to_pwm()` in `firmware/arduino/lib/MavCore/`: the m/s -> PWM
    conversion, in the board-agnostic core so it is unit-tested on the laptop
    (`scripts/test_firmware_core.sh`) rather than only ever compiled. Takes TWO
    measured constants — `MPS_TO_PWM` (slope) and `PWM_DEADBAND` (breakaway PWM)
    — because a DC motor does not turn below a breakaway PWM, and a single gain
    fitted at full throttle overstates the speed at the 0.15-0.3 m/s the policy
    actually uses. `speed == 0` returns 0 so "stop" really coasts.
- `src/mav/tools/calibrate_speed.py`: the measurement procedure in code. Streams
    a PWM sweep over serial (the 500 ms watchdog makes sweeping by hand
    impossible), asks for each distance, and least-squares fits both constants.
    `--dry-run` prints the sweep and a hand-checkable worked fit. `sweep()` needs
    hardware; `fit_speed_model()` is pure and tested in
    `tests/test_calibrate_speed.py`. Procedure: `docs/hardware.md`,
    "Speed calibration".
- **Not written yet:** the firmware's measured constants (m/s->PWM calibration,
    servo model + microsecond range, servo 5 V BEC - all Q1: measure, do not
    guess), GPIO buttons, the real cone hue, and the dashboard's browser
    visual check (frontend built in `8875d48`). The firmware itself compiles
    for the Uno; see `firmware/arduino/README.md`.

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
- Run a profile: `scripts/run.sh <sim|prototype|final>`. Currently this only loads and prints the profile config; the actual pipeline is not wired yet.
- Run sim: `scripts/run.sh sim [scenario]` (synthetic closed-loop sim; defaults from
  `configs/sim.yaml` section `sim:`).
- Run sim + web dashboard: `scripts/run.sh sim [scenario] --dashboard`, then open
  http://127.0.0.1:8000 (host/port from the `dashboard:` section of `configs/sim.yaml`;
  `enabled: true` there starts it without the flag). The dashboard is a viewer plus a
  control channel only — it never computes anything, and `final` never starts a server.
- Run on Jetson: `scripts/run.sh prototype` (debug on) or `scripts/run.sh final` (no debug, no network). Use the same `sudo nvpmodel` mode as the competition.
- Tests: `source .venv/bin/activate && pytest` (on the Jetson install pytest first: `pip install pytest`, or `sudo apt install python3-pytest`)
- Firmware core unit test (laptop, no Arduino needed): `scripts/test_firmware_core.sh` (g++ build + run of `firmware/arduino/lib/MavCore/test/test_protocol.cpp`)
- Flash firmware (laptop, Uno over USB): `scripts/flash_firmware.sh <prototype|final>` (port auto-detect; `--compile-only` / `--list-ports` / `--port` available)
- Firmware build/flash (Arduino Uno, needs `pip install platformio`): `cd firmware/arduino/prototype && pio run` (compile), `pio run -t upload` (flash), `pio device monitor` (serial 115200); same three in `final/`
- Check JetPack (on the Jetson, before any version-dependent install): `cat /etc/nv_tegra_release` and `apt show nvidia-jetpack`

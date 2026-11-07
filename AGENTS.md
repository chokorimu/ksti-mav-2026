# AGENTS.md – 1:10 Autonomous Vehicle for KSTI 2026 (Mini Autonomous Vehicle)

Working language: English. Keep Indonesian names of official documents as they appear.

## Reference documents (read before working on the related topic)
- Full competition rules, scoring formulas, penalties, ambiguities: @docs/ksti-mav-rules.md
- Research plan, architecture, minimal electrical guide: @docs/research-plan.md

If a task touches an item marked **[A#]** in the rules, do not guess. Ask the user.

## Environment
- **Jetson Orin Nano: SSH only, no display.** Do not use `cv2.imshow`, GUI code, or tools that need a screen. To inspect results, save images/video or use an MJPEG stream in dev mode only.
- **JetPack 6.2.3 (L4T R36.5, Ubuntu 22.04 base).** Version-dependent facts (Python, CUDA, TensorRT, OpenCV build options) must be verified on the device and recorded in `docs/hardware.md`; do not assume them. Use NVIDIA aarch64 packages built for JetPack 6, never x86 wheels.
- Check the power mode with `sudo nvpmodel -q`. Use the same mode for testing and competition, and measure performance with `tegrastats`.
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
src/         perception/, planning/, control/, io/ (camera, actuator: sim & real)
models/      prototype/, deploy/
scripts/     setup_dev.sh, setup_jetson.sh, run.sh
tests/
```

## Commands
Run all commands from the repo root. Use bash.

- Dev setup (laptop, x86_64): `scripts/setup_dev.sh`
- Jetson setup (Orin Nano, aarch64): `scripts/setup_jetson.sh` (uses `--system-site-packages` so JetPack's OpenCV/TensorRT stay visible; do not pip-install x86 wheels there)
- Run a profile: `scripts/run.sh <sim|prototype|deploy>`. Currently this only loads and prints the profile config; the actual pipeline is not wired yet.
- Run sim: `scripts/run.sh sim`. The web simulator's own launch command is _TBD_ (its stack and frame/command interface are not yet known).
- Run on Jetson: `scripts/run.sh prototype` (debug on) or `scripts/run.sh deploy` (no debug, no network). Use the same `sudo nvpmodel` mode as the competition.
- Tests: `source .venv/bin/activate && pytest`
- Check JetPack (on the Jetson, before any version-dependent install): `cat /etc/nv_tegra_release` and `apt show nvidia-jetpack`

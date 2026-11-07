# AGENTS.md – 1:10 Autonomous Vehicle for KSTI 2026 (Mini Autonomous Vehicle)

Working language: English. Keep Indonesian names of official documents as they appear; inline comments in `configs/` and requirements files are Indonesian — leave them as-is.

## Read first
- Competition rules, scoring, penalties, ambiguities: `docs/ksti-mav-rules.md` (transcription of the guidebook PDF — the organizer's word overrides it).
- Research plan, architecture, electrical guide, open items: `docs/research-plan.md`.
- If a task touches an item marked **[A#]** in the rules, do not guess. Ask the user.
- This is the only instruction file in the repo (no README, CI, or other agent config to satisfy).

## Environment
- **Jetson Orin Nano: SSH only, no display.** No `cv2.imshow` or GUI code; inspect via saved frames/video or an MJPEG stream (dev mode only).
- **JetPack version unknown.** Before any version-dependent install, ask the user to run `cat /etc/nv_tegra_release` and `apt show nvidia-jetpack` on the Jetson and record it in `docs/hardware.md` (does not exist yet — create it).
- Power mode: check with `sudo nvpmodel -q`; use the same mode for testing and competition; measure with `tegrastats`.
- Linux preferred, Windows via WSL2 fallback. Write commands for bash; use `pathlib`; never hardcode paths or serial port names. `.gitattributes` enforces LF.
- Two architectures: x86_64 laptop vs aarch64 Jetson. Dependencies are split (`requirements-dev.txt` vs `requirements-jetson.txt`); never pip-install x86 wheels on the Jetson.
- The web simulator uses real computer vision. Only the camera and actuator layers differ between sim and robot; all perception/planning/control/state-machine code is shared. Planned interfaces: `Camera.read() -> frame` and `Actuator.set(steer, throttle)`.

## Execution profiles (select via `MAV_PROFILE`, never code branches)

| Profile | Where | Model | Purpose |
|---|---|---|---|
| `sim` | laptop + web sim | prototype model | test algorithms without hardware |
| `prototype` | Jetson/laptop | prototype model (debug, overlay, full logging) | iteration and measurement |
| `deploy` | Jetson | optimized model (e.g. TensorRT), no debug | competition mode |

- `deploy` rules: no network server, streaming, or SSH dependency while the robot runs. Starts automatically at boot and waits for the physical start button.
- `prototype.yaml` and `deploy.yaml` have `camera.source` / `actuator.type` = `null` — fill in after hardware is confirmed, do not guess.

## Hard constraints from the competition (no design may violate these)
- Infrared sensors and line followers are banned. No sensors other than the camera without organizer confirmation **[A5]**.
- Any communication to control the robot during a run is banned, wireless or wired.
- Omni and mecanum wheels are banned. A physical emergency stop button and a start button are required.
- Marking color (white/black) follows the floor; checkpoint and start/finish colors are not guaranteed. Keep all colors configurable **[A11]**.
- After start is pressed, no human intervention. On violation, a human presses emergency stop, places the robot at the last checkpoint, and presses start again. **The program must resume (checkpoint count must not reset)**; verify this on every state machine change. Planned states: `BOOT → WAIT_START → RUN → STOPPED (emergency) → WAIT_START → RUN`.
- Scoring driver: checkpoint count outweighs the speed bonus, and leaving the track costs `0.8 * T_stage / N_cp` seconds — prefer conservative speed over fast-and-wrecked.

## Working rules for agents
1. **Assume the programmers know little about electrical engineering.** When touching GPIO, motors, servos, batteries, or cables, explain briefly in plain language: voltage, current, logic levels, shared ground (details: `research-plan.md` §3).
2. **Do not guess hardware facts** (Jetson model details, battery voltage, motor driver, pins). Ask, or request verification; record answers in `docs/hardware.md`.
3. **Give explicit warnings** before changes that could damage hardware: Jetson GPIO pins are 3.3 V and not 5 V tolerant — do not connect 5 V signals directly; do not power motors from Jetson pins; do not run actuators without a tested hardware e-stop path.
4. **Safe by default:** actuator code must start from a stopped state (throttle 0, steering neutral) and return there on error, frame loss, or exception.
5. **Test without hardware first:** sim or recorded real video (replay recordings through the same pipeline as regression tests), then Jetson with wheels lifted, then the track.
6. Any change to model or parameters that affects latency must include FPS/latency measured on the Jetson, not only on the laptop.
7. If a command is not defined below, do not invent it. Ask, or propose one and record it after approval.

## Repo layout
```
AGENTS.md
configs/     sim.yaml, prototype.yaml, deploy.yaml, vehicle.yaml
docs/        ksti-mav-rules.md, research-plan.md (hardware.md: to be created)
models/      prototype/, deploy/ (only .gitkeep — weights are gitignored)
scripts/     setup_dev.sh, setup_jetson.sh, run.sh
src/mav/     __main__.py (entry point); io/ control/ perception/ planning/ (empty packages)
tests/       only .gitkeep — no tests yet
data/        recordings/, calibration/ (contents gitignored)
```

## Commands (run from repo root, bash)
- Dev setup (laptop, x86_64): `scripts/setup_dev.sh` — creates `.venv`, installs `requirements-dev.txt`.
- Jetson setup (Orin Nano, aarch64): `scripts/setup_jetson.sh` — venv with `--system-site-packages` so JetPack's OpenCV/TensorRT stay visible; installs only `pyyaml` (numpy/CV/TensorRT/PyTorch come from JetPack).
- Run a profile: `scripts/run.sh <sim|prototype|deploy>` (default: `prototype`). Sets `MAV_PROFILE` and `PYTHONPATH=src`, runs `python -m mav`. Currently `__main__.py` only loads and prints the profile config; the pipeline is not wired yet.
- Config loading: `src/mav/__main__.py` resolves `configs/<profile>.yaml` relative to the package file (`parents[2]` = repo root), so it works from any cwd.
- Tests: `source .venv/bin/activate && pytest` (no tests exist yet — add them under `tests/`).
- JetPack check (on the Jetson, before version-dependent installs): `cat /etc/nv_tegra_release && apt show nvidia-jetpack`.
- The web simulator's own launch command is TBD (its stack and frame/command interface are not yet known).

## Gotchas
- `.gitignore` excludes `models/**/*.onnx`, `*.pt`, `*.engine` — engines are tied to device and JetPack version. Never commit model weights.
- `configs/vehicle.yaml` (`wheel_base_mm`, `track_width_mm`, `max_steer_deg`) is all `null` — fill from real measurements; it is shared by sim and control.
- Competition is 12 Nov 2026 (Technical Meeting 8 Nov). Time is short — follow the P0 → P1 order in `research-plan.md` §4; a reliable robot beats an elegant one.

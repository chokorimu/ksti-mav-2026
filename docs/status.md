# Project Status and Handoff

Last updated: 7 November 2026. Read `AGENTS.md` first, then this file, then `docs/hardware.md`.
Update this file at the end of every work session.

## Time pressure
Technical Meeting: 8 Nov 2026. Competition: 12 Nov 2026. Priority is a robot that completes the track reliably, not elegant research.

## Decisions made by the user
- Repo `ksti-mav-2026`; Linux preferred for development (Windows as fallback).
- Jetson Orin Nano, SSH only, no display. JetPack 6.2.3.
- Three profiles: `sim`, `prototype`, `deploy`. The model can be switched between prototype and deploy.
- Prototype robot: camera + 4 motors, one per wheel. Deploy robot (planned): 1 steering servo + 1 rear motor.
- A web simulator exists and uses real computer vision.
- Documentation and agent files are in English. The user writes in Indonesian; reply in the user's language.

## Assumptions made by the previous agent (NOT confirmed, verify before relying on them)
- A1: "differential drive" in the rules probably means a differential gear in the drive system, not differential steering.
- After an emergency stop and a manual restart at the last checkpoint, the run is a resume (checkpoint count kept).
- `T_stage = 300 s` (5 minutes); `S_ARC = 0` if the challenge is not taken; unfinished attempts have `T_total >= T_stage`.
- Cameras are allowed (the guidebook does not say so explicitly).
- `docs/ksti-mav-rules.md` is a transcription of the PDF and contains some interpretation. When a rule matters for a decision, check the original guidebook PDF.

## Verified facts
- On the Jetson: Python 3.10.12, OpenCV 4.8.0 (GStreamer yes, no CUDA line), CUDA 12.6, cuDNN 9.3, TensorRT 10.3 runtime. Power mode `MAXN_SUPER`.
- `scripts/setup_jetson.sh` ran successfully.
- The full `nvidia-jetpack` package is NOT installed (connection problems). TensorRT dev tools and Python bindings are unconfirmed. Not needed for classical CV.
- 48 unit tests passed on the development laptop (Python 3.14.8, numpy 2.5.4, OpenCV 5.0.0). **They have not been run on the Jetson (Python 3.10).** Run `pytest` there first and report failures.

## Code that exists (see `AGENTS.md` for the module list)
Command type, camera/actuator/button interfaces, SafeActuator, differential and Ackermann mappers, state machine, `step()`.
Classical lane detector (`src/mav/perception/lane.py`, numpy-only) and a conservative lane policy (`src/mav/planning/lane_policy.py`), both with tests on synthetic frames.

## Not done (do not invent these; ask for the facts first)
- Real motor/servo drivers and GPIO buttons: how the motors and servo are driven is unknown.
- Checkpoint and start/finish detection (lane detection v1 exists but is uncalibrated).
- Measured vehicle values in `configs/vehicle.yaml` (all `null`).
- Emergency stop circuit details (what it cuts is unknown).

## Lane detection v1 (done on synthetic frames only, 7 Nov 2026)
- `LaneDetector` (numpy, no cv2): grayscale -> ROI -> large-window box background -> residual threshold at ±3.5·robust sigma (3x3 high-pass MAD) -> per-band marking centroids -> weighted quadratic fit -> offset at the lookahead row, curvature, confidence. Auto polarity picks light or dark markings.
- `LanePolicy` maps the measurement to `Command(speed, curvature)`: pure-pursuit steering clamped to the 1.5 m minimum turn radius, speed scaled by curvature and confidence, STOP when the lane is lost.
- Tests (`tests/test_lane.py`, `tests/test_lane_policy.py`, generator `tests/synthetic_lane.py`): affine projection asserts near-exact numbers; a "taper" projection (scale shrinks with distance) checks the sign and rough magnitude under scale variation. 18 tests, all passing on the laptop.
- Offline runner smoke-tested on a synthetic video: 120/120 frames detected, mean 8.3 ms/frame on the laptop (Jetson latency NOT measured).
- **Uncalibrated:** `px_per_m=130` and `bottom_row_m=0.3` are guesses. Offset/curvature are only exact at the lookahead row. Known weakness: a low camera makes near markings thick relative to the 75 px background window. Validate on real recordings before trusting the numbers.

## Next task: lane detection from real recordings
1. Record real video from the robot camera on the Jetson, headless. Do not rely on hardware H.264 encoding; saving frames or MJPG via `cv2.VideoWriter` is safer. Camera is CSI (Raspberry Pi module); bring-up commands are in `docs/hardware.md`. **(user task)**
2. Record variety: both marking colors (white and black) and floor colors if possible, bright and dim light, straight sections, curves, dashed centre line, a checkpoint, the start/finish line. **(user task)**
3. ~~Build classical lane detection~~ done as v1 above; now: run it offline on the recordings and recalibrate `px_per_m` / `bottom_row_m` from them.
4. Run it offline on the recordings first. Measure per-frame latency on the Jetson.
5. Do not connect it to real actuators until the hardware questions below are answered.
- Proposed command (needs approval before it goes into `AGENTS.md`): `PYTHONPATH=src python -m mav.perception.lane <video> [--overlay out.mp4]` from the repo root.

## Open questions for the user
- Arduino details: model, motor driver model, pins, servo model + angle range, battery/BEC for the 5 V rail. Emergency stop: what does it cut? (Recommendation: cut the motor power rail in hardware.)
- Is the steering linkage true Ackermann geometry?
- Organizer answers for A1 (differential), A5 (sensors), A11 (start/finish and checkpoint colors), plus checkpoint count and track map.
- Simulator stack and how it exchanges frames and commands (recommendation recorded in `docs/open-questions.md` Q5).
- Answered 7 Nov 2026: camera = CSI Raspberry Pi module (alive on the Jetson); rear wheels = mechanical differential; motor/servo = Arduino over USB serial. See `docs/hardware.md`.

## Rules for any agent working here
- Do not guess hardware facts or competition rules. Ask, and record the answer.
- Warn explicitly before anything that could damage hardware (GPIO 3.3 V, motor power, e-stop).
- Keep changes small and tested. Report what was tested and where (laptop or Jetson).

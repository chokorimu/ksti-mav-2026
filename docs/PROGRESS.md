# Progress

Last updated: 7 November 2026 (laptop session). Read this file first at session start.
Time pressure: Technical Meeting 8 Nov 2026, Competition 12 Nov 2026.

## Done (with "done when" check)

- [x] Core control-loop scaffolding (types, camera/actuator/buttons interfaces, mappers,
      state machine, `step()`). Check: `pytest` — 30 tests passed (laptop).
- [x] Lane detector v1 (`src/mav/perception/lane.py`, numpy-only) + offline runner
      `PYTHONPATH=src python -m mav.perception.lane <video> [--overlay out.mp4]`.
      Check: `pytest tests/test_lane.py` — 12 tests passed (laptop, synthetic frames only).
- [x] Lane policy v1 (`src/mav/planning/lane_policy.py`, measurement -> Command).
      Check: `pytest tests/test_lane_policy.py` — 6 tests passed (laptop).
- [x] Full suite green: `pytest` — 57 passed (laptop, Python 3.14.8, numpy 2.5.4, cv2 5.0.0).
      NOT yet run on the Jetson (Python 3.10).
- [x] Closed-loop synthetic simulator (`src/mav/sim/`): world (exact constant-curvature
      integration), shared numpy renderer, 4 scenarios, runner that drives the REAL
      `mav.app.step()` with the real detector and policy — only the camera and actuator
      layers are simulated, so anything tuned here transfers unchanged.
      Check: `pytest tests/test_sim.py` — 9 passed (laptop).
      Smoke (`python -m mav.sim.runner <scenario>`): straight + offset finish the 12 m,
      curve + s_curve keep 100% detection and stay inside the 0.6 m corridor
      (mean |offset| 0.14 m / 0.18 m — pure-pursuit lag, no curvature feedforward yet).
- [x] Sim bug found and fixed by the closed loop: a path that wraps >180° put its far end
      back inside the camera's forward window, corrupting `centre_x_fn` (centre_x(2.0 m)
      read -0.32 m instead of -0.007 m) and collapsing detector confidence.
      `centre_x_fn` now selects the branch the vehicle is driving on by ARC length from
      the nearest polyline point; the `curve` scenario is a 90° turn, not a 286° loop.

## Current task

Recording real video on the Jetson (user task). Camera is alive (CSI, `nvarguscamerasrc`
works). Record the variety listed in task 1 below; MJPG via `cv2.VideoWriter`, no hardware
H.264. Done when: the clips exist on the Jetson.

## Next tasks (in order)

1. Record real video on the Jetson. Variety: straight, curves, white + black markings,
   dim + bright light, dashed centre line, a checkpoint, start/finish. Done when: the
   clips exist and play back.
2. Run the detector offline on the recordings; recalibrate `px_per_m` and `bottom_row_m`
   (currently uncalibrated guesses: 130.0 / 0.3). Blocked by task 1.
   Done when: offsets/curvatures on recordings look sane and the constants are measured.
3. Measure per-frame latency on the Jetson (`tegrastats` + the runner's latency printout).
   Blocked by task 1. Done when: mean/p95 ms per frame recorded in this file.
4. `SerialActuator` (host side): send mapped wheel/steering commands to the Arduino over
   USB serial, wrapped in the existing `SafeActuator` (stopped default, clamp, timeout).
   Needs only the protocol decision + a serial port config (no hardcoded port names).
   The Arduino firmware waits for the driver model (Q1). Done when: unit tests with a
   fake serial port pass.
5. Checkpoint / start-finish line detection. Not started. Split into subtasks before coding.
6. Web-simulator bridge. The synthetic closed-loop sim exists (`src/mav/sim/`); what is
   left is wiring the web simulator's camera frames and commands into the same io layers.
   Blocked on Q5.

## Blocked on hardware facts (details in docs/open-questions.md)

- Arduino model, motor driver model, pins, servo model, battery/BEC, e-stop wiring (Q1).
- Steering linkage Ackermann or not (Q4) — affects the bonus, not the v1 code.
- Simulator stack (Q5).

## Known limitations of lane detector v1

- Single constant `px_per_m`: exact only at the lookahead row. A low camera makes near
  markings thick relative to the 75 px background window; the sigma estimate then
  collapses and the mask saturates. Validate on real recordings.
- Centroid-based: with only one edge line visible the centre estimate is pulled toward it.
- Checkpoints / start-finish lines are not detected (they do cross the lane symmetrically,
  so centring survives, but this is untested).

## Environment notes

- Laptop: `.venv` at repo root; debug scripts run with `PYTHONPATH=src:tests`.
- pytest output gets truncated by the tool harness: redirect to `/tmp/opencode/pytest_out.txt`
  and read the file for full failure details. Same for scripts that print to stdout.
- Sim CLI: `PYTHONPATH=src python -m mav.sim.runner [scenario] [--steps N] [--dt S]
  [--seed S] [--overlay out.mp4] [--list]`, or `scripts/run.sh sim <scenario>`.

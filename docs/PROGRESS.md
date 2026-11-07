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
- [x] Full suite green: `pytest` — 48 passed (laptop, Python 3.14.8, numpy 2.5.4, cv2 5.0.0).
      NOT yet run on the Jetson (Python 3.10).

## Current task

None in progress. Everything below is either blocked on the user or not started.

## Next tasks (in order)

1. Record real video on the Jetson (user task). Blocked: camera model/interface unknown
   (see docs/open-questions.md Q2). Done when: a recording exists on the Jetson.
2. Run the detector offline on the recordings; recalibrate `px_per_m` and `bottom_row_m`
   (currently uncalibrated guesses: 130.0 / 0.3). Blocked by task 1.
   Done when: offsets/curvatures on recordings look sane and the constants are measured.
3. Measure per-frame latency on the Jetson (`tegrastats` + the runner's latency printout).
   Blocked by task 1. Done when: mean/p95 ms per frame recorded in this file.
4. Checkpoint / start-finish line detection. Not started. Split into subtasks before coding.
5. Simulator bridge (frame/command exchange with the web simulator). Not started; blocked
   on the simulator stack question (Q5).

## Blocked on hardware facts (details in docs/open-questions.md)

- Real motor/servo drivers + GPIO buttons: driver type and pins unknown (Q1).
- All of tasks 1-3 above: camera unknown (Q2).
- Mechanical differential on the single rear motor (Q3), steering linkage Ackermann (Q4).

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
  and read the file for full failure details.

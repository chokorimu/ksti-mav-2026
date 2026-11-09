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
- [x] Measured KSTI track as a simulator scenario (T1): `src/mav/sim/data/track_ksti.json`
      (1440 centreline samples in metres, scale anchored to the guidebook's 50 cm lane
      width — measured off the committee's own map, nothing estimated) +
      `src/mav/sim/track.py` (rigid transform so the vehicle starts at the origin heading
      +Y, `track_stats()`, `measured_radius_min()` as an independent curvature check),
      `LanePath.from_points()`, `Scenario.points`, and the track name accepted by
      `parse_control()` / `/scenarios`. Check: `pytest tests/test_track.py` — 16 passed
      (laptop); `run.sh sim --list` shows ksti; a live dashboard serves it in `/scenarios`
      and accepts it in `POST /control`. DONE 7 Nov 2026, commits `001cf30` + `8df1883`.

## T2 result: the current policy on the measured track (7 Nov 2026, laptop)

`run_sim("ksti", max_steps=1200)` = 60 s sim time. Seeds 0/1/2 agree to within noise:

| metric | value |
|---|---|
| lap covered | **1.26 m of 19.39 m (6.5%)** |
| mean speed | 0.021 m/s |
| ticks below 0.1 m/s | 1147 / 1200 |
| detection rate | 9% (109/1200 frames) |
| departures (>0.25 m) | 0 |
| steering clamp saturated | **0% of ticks** |

Failure chain, traced tick by tick (the robot dies at s≈1.26 m, ~11 m before the
0.46 m corner at s=12.59 m):

1. The track is tight from the start: median R = 1.33 m in the first metre, 0.47 m at
   s=2–3 m. There is no straight to accelerate on.
2. The detector finds the lane on every frame (12 band points), but a SINGLE quadratic
   cannot fit a track whose curvature changes inside the 2.2 m window: `resid` = 42 px,
   and `f_res = 1 - min(1, resid/25)` saturates to 0 at 25 px, so confidence is capped
   at ~0.31 (vs 0.90 on the synthetic scenarios, where curvature is constant).
3. `severity = max(|κ|, |measured curvature|)` = 1.71 > `curv_slow` = 1.0, so speed
   drops to `v_min` = 0.15 m/s, then `*= confidence` → 0.047 m/s.
4. Confidence fluctuates 0.30–0.32, right on `min_confidence` = 0.3. When it dips below,
   `meas is None` → STOP.
5. Deadlock: stopped = frozen pose = frozen frame = no recovery. The robot never moves again.

Track geometry (measured, not simulated): loop 19.39 m, width 0.4993–0.5027 m, tightest
radius 0.46 m at s=12.59 m, median 1.97 m, and **7.83 m (39.3%) of the lap is tighter
than the policy's 1.5 m steering clamp**.

The important nuance: the 1.5 m steering clamp is NOT the binding constraint (0%
saturation). The robot dies of low speed long before it reaches a corner tight enough
to need the clamp. Fixing the steering clamp alone would change nothing.

Candidate fixes (NOT decided — waiting on the user):
- Detector: piecewise fit, or fit only the near rows (where one quadratic is valid).
- Policy: `on_lost="hold"` or a slow creep instead of `"stop"`, so a lost frame is not
  a permanent stop.
- Policy: the speed law multiplying by confidence turns a detector problem into a
  motion problem — a detector artefact should not be able to stop the robot.

### T2 update 3: conf_min lowered, and the sim now reads `policy:` (9 Nov 2026)

`LanePolicyConfig.conf_min` 0.3 → **0.15** (also in `configs/prototype.yaml` and
`configs/sim.yaml`). At 0.3 the policy called a low-confidence frame "lane lost" and
stopped instead of steering; on the measured track that is most frames.

Found on the way: `runner.main()` never passed `policy_cfg`, so the new `policy:`
section in `configs/sim.yaml` was read by **nobody** — the CLI silently ran on code
defaults. Fixed with `_policy_config()`, the same top-level section `robot.py` reads,
so `run.sh sim` and `run.sh prototype` steer with the same numbers. Pinned by
`test_cli_reads_the_policy_section`.

Full suite 219 passed. Still not re-measured on the KSTI track.

### T2 update 2: the policy deadlock is IN (8 Nov 2026, commit `6f2d8c1`)

The user chose "fix deadlock policy dulu". One line in `src/mav/planning/lane_policy.py`:
`LanePolicyConfig.on_lost` default `"stop"` → **`"hold"`**.

Why it was a deadlock, not just a conservative stop: the robot is stationary when
`on_lost="stop"` fires, and a stationary pose keeps producing the same frame, so the
same lost measurement — the stop is self-sustaining. Measured on the KSTI track:
1101 of 1200 ticks below 0.1 m/s. With `"hold"` the last good command is kept, so the
robot keeps rolling through a bad frame and the detector gets a fresh view.

Full suite **211 passed** (laptop). No regression on the synthetic scenarios
(straight/offset/curve/s_curve unchanged). **Not yet re-measured on the KSTI track** —
that is the next step, and it needs the closed-loop script in `/tmp/opencode/t2_report.py`
re-run (or an equivalent) to say whether the robot now gets past s=1.75 m.

### T2 update: the detector fix is IN (7 Nov 2026, commit `fcccf2a`)

The user chose "detektor dulu". Two changes in `src/mav/perception/lane.py`:
`roi_top` 0.45 → **0.65** (a shallower integration window) and a new
`LaneConfig.max_band_span_m = 0.8` that drops any band whose marking is smeared
across the image (where the track is tight the lane runs ALONG a band instead of
across it, so its centroid is meaningless — errors up to 85 px measured).
6 new tests in `tests/test_lane.py`; full suite **147 passed** (laptop).

Same measurement, same 1200 ticks, seeds 0/1/2:

| metric | before | after |
|---|---|---|
| lap covered | 1.26 m (6.5%) | **1.73 m (8.9%)** |
| detection rate | 9.1% | 11.5% |
| mean \|offset\| | 0.081 m | 0.053 m |
| fit residual | mean 42 px | mean 10.4 / max 25.7 px |
| confidence | capped ~0.31 | mean 0.713 / min 0.308 |

The detector is healthy now — but **the robot still dies**, at s=1.75 m (t≈153).
The remaining cause is pure geometry, and it is NOT fixable by tuning:

- At s=1.75 m the local radius is **0.73 m**, so the 0.83 m lookahead is already
  past the 90° point of that corner: the lane wraps inside the ROI no matter how
  clean the mask is.
- Proof: a **noiseless camera changes nothing** (1.73 → 1.71 m). The smearing is
  geometric, not sensor noise.
- Sweeping `roi_top` 0.60 → 0.90 improves the measured track monotonically
  (1.60 → 2.24 m) but wrecks `s_curve` monotonically (0 → 271 departures).
  **No single fixed `roi_top` wins both** — that is the case for an adaptive
  lookahead, not for another constant.
- Even at the best setting the robot reaches only 11.6% of the lap, still ~10 m
  before the 0.46 m corner at s=12.59 m.

Still true and unchanged: the 1.5 m steering clamp is NOT the binding constraint
(0% saturation). The robot dies of low speed long before it needs the clamp.

**Next decision needed (asked 7 Nov 2026, not yet answered):** adaptive lookahead
(`roi_top`/`lookahead_frac` driven by the measured curvature — shallow in a tight
corner, deep on a straight) versus accepting the fixed 0.65 and moving to the
policy's deadlock (`on_lost="stop"` makes any single lost frame a permanent stop,
which is a robustness bug independent of the detector). Note the sim camera is
ideal (perfect markings, known geometry) — the real robot will be worse, so
detector tuning against this synthetic track has limited value.

## Current task: web dashboard for the sim (user request, 7 Nov 2026)

Goal: browser dashboard with a 3D top-view arena, a 3D car, a live camera panel with the
detector overlay, and debug numbers — plus start / emergency / scenario / reset controls.
Decisions taken: HTTP polling with stdlib `ThreadingHTTPServer` (no new dependency),
three.js vendored in-repo, this REPLACES the external web simulator (Q5 closed).
Hard rule: the dashboard is a pure viewer + control channel. `lane.py`, `lane_policy.py`
and `app.py` stay untouched; `final` stays serverless (gated by config, sim profile only).

Split into 6 subtasks (AGENTS.md rule 7). One at a time, commit after each.

- [x] 1. State + bridge + control channel (`src/mav/sim/dashboard/state.py`, `bridge.py`;
      runner gains `bridge=` + control handling + `realtime` pacing; `detect_with_debug()`
      in lane.py, additive). Done when: `pytest tests/test_dashboard.py` passes AND
      `run_sim(bridge=...)` returns a SimResult identical to `run_sim()` (same seed) AND
      scripted start/emergency/scenario/reset actions drive the state machine correctly.
      DONE 7 Nov 2026, commit `e7044bb`: 20 new tests, full suite 77 passed (laptop).
- [x] 2. HTTP server (`src/mav/sim/dashboard/server.py`, stdlib only): `/`, `/state.json`,
      `/frame.jpg`, `/lane.json`, `/scenarios`, `POST /control`. Done when: curl against a
      running sim returns valid JSON/JPEG and a POST start makes the robot move.
      DONE 7 Nov 2026, commit `19298c8`: 22 new tests, full suite 99 passed (laptop).
      Also fixed a real bug found on the way: `_sim_config()` resolved `parents[2]` == `src/`,
      so `configs/sim.yaml` was NEVER read and every documented default silently fell back to
      argparse values. Same bug in `__main__.py` (non-sim profiles). Guarded by
      `test_sim_config_is_read_from_the_repo`. `configs/sim.yaml` now has the `dashboard:`
      section (subtask 6's config part, done here).
- [x] 3. Arena 3D + car (`static/`): vendored `three.module.js`, floor, lane, car with
      steering front wheels, lookahead marker, trail. Done when: the user verifies visually
      on the laptop that the render matches the world.
- [x] 4. Camera panel + detector overlay: live frame + band centroids + fitted quadratic +
      lookahead row + ground-truth centre + mask thumbnail. Done when: overlay matches the
      numbers shown.
- [x] 5. Debug numbers + controls UI: buttons wired to `POST /control`, scenario dropdown,
      reset, run metrics, wall-clock FPS. Done when: the full loop works from the browser
      (select scenario -> start -> emergency -> start again (resume) -> reset).
      Subtasks 3-5 DONE 7 Nov 2026, commit `8875d48` (frontend built by a subagent, then
      audited and fixed by me: one duplicate `const isStr` broke the module). Verified
      mechanically only — see the commit message. **Visual verification in the browser is
      still the user's job**: open http://127.0.0.1:8000 while
      `scripts/run.sh sim curve --dashboard` runs and check that the arena matches the world,
      the overlay matches the numbers, and the full control loop works.
- [x] 6. Config gating + docs + commit: `dashboard:` section in `configs/sim.yaml`;
      `final.yaml` has none -> no server. Update AGENTS.md, status.md, close Q5.
      DONE 7 Nov 2026: config part in `19298c8`, docs in `e027c07`.

Frontend subtasks 3-5 are one block (same `app.js`/`index.html`) and may be delegated to a
subagent against the JSON contract fixed in subtask 1; visual verification stays with the user.
DONE 7 Nov 2026 (`8875d48`) — the whole dashboard task is finished except the user's visual
check. Next: the real-recordings task below.

## Task 6: checkpoint / start-finish line detection — DROPPED 9 Nov 2026

> **The user reported (9 Nov 2026) that checkpoints are only markers placed OUTSIDE
> the lane**, so the robot never needs to detect them; the count comes from the
> organisers, not from our camera. Subtasks 1, 2 and 4 are committed and stay
> (harmless, tested), subtask 3 (sim rendering of cross-lines) is not worth
> finishing. The real in-lane hazard is the **orange cone**: see "Obstacle
> avoidance" below. Do not spend more time on the text in this section.

Why it mattered: `cp` (checkpoints passed) is the biggest scoring term, and after a
violation the robot is restarted at the last checkpoint — the count must survive
that (the state machine already does; only the DETECTION is missing).

Rules that shape the design (docs/ksti-mav-rules.md):
- §6: checkpoints are shown in **green**; the start/finish line is "a colour
  different from the track markings, **not necessarily red**". **[A11]**
- [A11] resolution (from the rules doc itself): "Do not hardcode the start/finish
  or checkpoint colours without confirmation; **design colour parameters to be
  configurable**." So: hue ranges are config, defaults follow the guidebook text
  and are documented as unconfirmed until read off the arena on test day.
- §7.7: checkpoints must be passed in order (a counter enforces that by construction).
- Lane markings are white OR black (§6) — the lane detector is colour-blind on
  purpose; the cross-line detector is colour-FIRST (saturation + hue), which is
  what keeps it independent of the marking colour.

Split into 4 subtasks (one at a time, commit after each):

- [x] 1. Core detector `src/mav/perception/checkpoint.py` (numpy-only, no cv2,
      same contract as LaneDetector): HSV saturation mask (white/black/grey
      markings and most floors stay below it) + configurable hue bands
      (checkpoint/finish) + geometry (thin horizontal band spanning the lane).
      Done when: `pytest tests/test_checkpoint.py` passes — green band detected,
      white/black band rejected (unsaturated), red rejected while finish is
      disabled and detected when configured, patch/tall-region/narrow-band
      rejected, blank floor None, bad frame layout ValueError.
      DONE 7 Nov 2026: 15 tests, full suite 146 passed (1 pre-existing failure
      in the user's uncommitted lane.py WIP, unrelated). Defaults: checkpoint
      hue 90-150 deg (green, per guidebook, UNCONFIRMED [A11]), finish
      disabled. NOTE: lane.py's docstring still says "checkpoints not detected
      here (later task)" - update that cross-reference when the user's lane.py
      work lands (their file is uncommitted, do not mix).
- [x] 2. Event tracker + wiring: debounce (one event per physical crossing:
      line enters from far, exits near — never counts a line the robot starts
      sitting on), forward-only gating, feeds `RobotStateMachine.add_checkpoint()`;
      finish line ends the run (`sm.finish()`, START ignored until new_attempt).
      Done when: scripted frame sequences produce exactly one event per crossing,
      resume keeps the count, and the existing state-machine tests still pass.
      DONE 7 Nov 2026: `CrossLineTracker` (IDLE/TRACKING/AT_EXIT, re-arms when
      the line leaves the frame, 2-frame dropout tolerance), `RobotStateMachine
      .finish()` (STOPPED + not resumable until new_attempt, count kept),
      `run_sim(tracker=...)` wires events into the state machine and reports
      `SimResult.checkpoints`. Tracker without rendered lines leaves the run
      bit-identical (asserted by a test). 20 new tests; full suite 160 passed.
- [ ] 3. Sim integration: render cross-lines in the sim camera layer (scenario
      config: s-positions + colours), wire the tracker into `run_sim()`.
      Done when: a closed-loop sim run passes N checkpoints and the count is N,
      and emergency+resume mid-run keeps it.
- [x] 4. Config + docs: `checkpoint:` section in configs/ (colours + geometry,
      defaults marked unconfirmed), AGENTS.md/PROGRESS.md updates.
      Done when: every shipped profile parses, a bad key/value fails loudly, and
      the camera geometry is reachable without touching code.
      DONE 7 Nov 2026: new `src/mav/config.py` (`load_lane_config()`,
      `load_checkpoint_config()`; unknown keys / bad hue bands / out-of-range
      values raise, `null` disables an optional gate). `configs/prototype.yaml`
      and `configs/final.yaml` carry `lane:` (px_per_m 130, bottom_row_m 0.3,
      both UNCALIBRATED - the phone recording measures 964-1610 px/m, i.e. the
      default is ~10x off for that viewpoint) and `checkpoint:` (checkpoint hue
      90-150 deg green UNCONFIRMED [A11], finish disabled). `configs/sim.yaml`
      has a reference-only `lane:` (the runner derives its own from WorldConfig)
      and no `checkpoint:` (hues come from the rendered scenario colours).
      `python -m mav` validates both sections at boot. 14 new tests
      (`tests/test_config.py`); full suite 160 passed (laptop). Only subtask 3
      remains, and it is the user's in-progress work (render.py/lane.py/runner.py
      /scenarios.py/world.py uncommitted - not touched here).

## Current task: obstacle (orange cone) avoidance, rules §8

Why: the user's arena update (9 Nov 2026) — the thing ON the lane is cones, and the
penalty table decides the whole strategy: hitting a cone `k = 0.5`, leaving the
track `k = 0.8`. So a swerve is only worth it while it stays inside the lane.

- [x] Detection `src/mav/perception/obstacle.py`: `ConeDetector` -> list of blobs
      (nearest first) with lateral/distance in metres, reusing `_hsv`/`_hue_mask`/
      `_runs` from `checkpoint.py`. Colour-first like the cross-line detector, so
      white/black lane markings and the grey floor can never match. `hues: []`
      disables it (AR only).
- [x] Decision `src/mav/planning/avoid.py`: `choose_offset()` is a PURE function
      (no state, no camera) — pure pursuit's lane offset in, a clamped target out.
      Picks the side needing the smaller deviation; when neither side fits inside
      `max_offset_m`, it returns the lane offset unchanged and accepts the cone.
- [x] Wired into `Driver` (`src/mav/robot.py`): the target REPLACES `offset_m` in
      the measurement handed to `LanePolicy`, so the policy still steers at the
      lane, only at a shifted aim point. The lost-lane test uses the MEASURED
      offset, not the swerve target. No cone detector -> behaviour is unchanged.
- [x] Config: `obstacle:` (hue band UNCONFIRMED, saturation/value gates, `roi_top`)
      and `avoid:` (`clearance_m`, `max_offset_m`, `horizon_m`) in
      `configs/prototype.yaml` and `configs/final.yaml`; `load_obstacle_config()`
      INHERITS `px_per_m`/`bottom_row_m` from `lane:` so one camera move stays one
      edit. Tests: `tests/test_obstacle.py` (16), `tests/test_robot.py` (+3),
      `tests/test_config.py` (+6); full suite 244 passed (laptop).
- [ ] Not done: rendering cones in the simulator. It needs `render.py` + wiring in
      `runner.py` (which calls the detector directly, NOT through `Driver`), and
      both files are the user's active work. Add when the sim can draw a cone.
- [ ] Not measured: the real cone hue. Until it is read off the arena the detector
      is a guess (`obstacle.hues`, marked UNCONFIRMED [A11]).

## Next tasks (in order)

1. **T2 follow-up — BLOCKED on a user decision (asked 7 Nov 2026).** The policy cannot
   drive the measured track: it covers 6.5% of a lap and then deadlocks (see the T2
   result above). Two coupled decisions are needed before any code changes:
   (a) Stanley vs pure pursuit for the steering law, and (b) whether to fix the
   detector's single-quadratic model first (piecewise / near-rows-only fit), since the
   policy's confidence term is what actually stops the robot. Do not start this until
   the user answers; do not change `lane_policy.py` or `lane.py` unilaterally.
2. Record real video on the Jetson. Variety: straight, curves, white + black markings,
   dim + bright light, dashed centre line, a checkpoint, start/finish. Done when: the
   clips exist and play back.
3. Run the detector offline on the recordings; recalibrate `px_per_m` and `bottom_row_m`
   (currently uncalibrated guesses: 130.0 / 0.3, now per-profile config in
   `configs/<profile>.yaml` - edit there, not in code). Blocked by task 2.
   Done when: offsets/curvatures on recordings look sane and the constants are measured.
   Partial input already in hand: the phone recording in `data/recordings/` measures
   964-1610 px/m (lane 0.5 m wide = 482-805 px on screen) and gets 0% detection with
   the 130 default - the band gate correctly rejects every band as "smeared". That
   viewpoint is NOT the robot camera, so it proves the scale is wrong, not what the
   right value is.
4. Measure per-frame latency on the Jetson (`tegrastats` + the runner's latency printout).
   Blocked by task 2. Done when: mean/p95 ms per frame recorded in this file.
5. ~~`SerialActuator` (host side)~~ DONE 7 Nov 2026, commit `c75c7d0`: protocol v1 in
   `firmware/arduino/protocol.md` (ASCII lines `W`/`D`/`S`, 500 ms watchdog, boot-stopped),
   `SerialActuator` in `src/mav/io/actuator.py` (port injected, verb chosen by the mapper,
   pyserial imported lazily), `tests/test_serial_actuator.py` (11 tests, fake port).
   Firmware DONE 7 Nov 2026, commit `0553de5`: hardware confirmed (Arduino Uno,
   L298N, LiPo 3S - wiring notes in docs/hardware.md). `firmware/arduino/` is now
   two PlatformIO projects (`prototype/`, `final/`) sharing `lib/MavCore`
   (protocol.md parser + watchdog, laptop-tested with g++). Both variants
   COMPILE for the Uno (prototype 4966 B flash / 276 B RAM, final 5532 B /
   317 B); pin maps and L298N output stages written. Still open: servo model +
   range (final), the servo 5 V BEC, and the wheels-lifted m/s->PWM
   calibration (both). Flash tooling DONE 7 Nov 2026:
   `scripts/flash_firmware.sh <prototype|final>` (compile + upload, port
   auto-detect, `--compile-only`/`--list-ports`/`--port`); tested end-to-end
   except the physical upload (no Uno was plugged in at the time).
6. ~~Checkpoint / start-finish line detection~~ DONE 9 Nov 2026: `CrossLineDetector`
   in `src/mav/perception/checkpoint.py` (HSV colour-first, configurable hue bands
   per [A11]), `CrossLineTracker` (one event per physical crossing, conservative
   about lines first seen deep), integration in `Driver` (robot.py) and
   `run_sim`/`policy_fn` (sim/runner.py). 28 tests pass; sim "checkpoints"
   scenario counts 3 CPs + finish line correctly; emergency-resume preserves
   checkpoint count. Still needed: calibrate hue bands on the real arena
   (block: task 2 — real video recordings).
7. ~~Web-simulator bridge~~ replaced by the web dashboard for the synthetic sim (current
   task above). The external web simulator is no longer needed; Q5 closed.

## Blocked on hardware facts (details in docs/open-questions.md)

- Arduino model, motor driver model, pins, servo model, battery/BEC, e-stop wiring (Q1).
- Steering linkage Ackermann or not (Q4) — affects the bonus, not the v1 code.
- ~~Simulator stack (Q5)~~ closed 7 Nov 2026: replaced by the web dashboard for the
  synthetic sim (current task). No external web simulator needed.

## Known limitations of lane detector v1

- Single constant `px_per_m`: exact only at the lookahead row. A low camera makes near
  markings thick relative to the 75 px background window; the sigma estimate then
  collapses and the mask saturates. Validate on real recordings.
- Centroid-based: with only one edge line visible the centre estimate is pulled toward it.
- Checkpoints / start-finish lines are not detected (they do cross the lane symmetrically,
  so centring survives, but this is untested).
- **Measured on the real track (T2, 7 Nov 2026):** the single-quadratic model assumes
  constant curvature across the whole 2.2 m window. The measured track changes curvature
  inside that window, so `resid` = 42 px and the `f_res = 1 - min(1, resid/25)` term
  saturates, capping confidence at ~0.31 — right on `min_confidence`. This is the root
  cause of the T2 failure, not the steering clamp.

## Environment notes

- Laptop: `.venv` at repo root; debug scripts run with `PYTHONPATH=src:tests`.
- pytest output gets truncated by the tool harness: redirect to `/tmp/opencode/pytest_out.txt`
  and read the file for full failure details. Same for scripts that print to stdout.
- Sim CLI: `PYTHONPATH=src python -m mav.sim.runner [scenario] [--steps N] [--dt S]
  [--seed S] [--overlay out.mp4] [--list]`, or `scripts/run.sh sim <scenario>`.

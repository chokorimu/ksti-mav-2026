"""Closed-loop simulator runner.

Runs the REAL control loop (mav.app.step) with the real detector and policy;
only the camera and actuator layers are simulated. That is the whole point:
whatever is tuned here transfers to the robot unchanged.

Usage: `python -m mav.sim.runner [scenario] [--steps N] [--dt S] [--seed S]
[--overlay out.mp4] [--list]`. Also reachable via `scripts/run.sh sim`
(MAV_PROFILE=sim), which reads defaults from configs/sim.yaml.

`bridge=` attaches a viewer (web dashboard): the runner publishes a SimState and
the camera frame every tick and drains controls (start / emergency / scenario /
reset). Without a bridge the run is identical to the pre-dashboard behaviour,
which tests/test_dashboard.py asserts by comparing results with the same seed.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from mav.app import step
from mav.control.state_machine import RobotStateMachine
from mav.io.actuator import CallbackActuator, SafeActuator
from mav.io.camera import CallbackCamera
from mav.io.buttons import Event, QueueButtons
from mav.perception.lane import LaneConfig, LaneDetector, lookahead_row
from mav.planning.lane_policy import LanePolicy, LanePolicyConfig
from mav.sim.dashboard.bridge import Bridge, NullBridge
from mav.sim.dashboard.state import Pose, SimState
from mav.sim.scenarios import SCENARIOS, Scenario, get_scenario
from mav.sim.world import SimWorld, WorldConfig, build_path
from mav.types import Command, STOP

DEPARTURE_M = 0.25  # |offset| beyond this counts as leaving the lane centre corridor
TRAIL_MAX = 400     # poses kept for the arena trail
LANE_STEP_M = 0.25  # display sampling of the centreline polyline


@dataclass
class SimResult:
    scenario: str
    steps: int = 0
    distance_m: float = 0.0
    time_s: float = 0.0
    detected: int = 0
    lost: int = 0
    departures: int = 0
    max_abs_offset: float = 0.0
    mean_abs_offset: float = 0.0
    stop_frames: int = 0
    crashed: bool = False
    finished: bool = False
    history: list[tuple] = field(default_factory=list)

    @property
    def detection_rate(self) -> float:
        return self.detected / self.steps if self.steps else 0.0

    @property
    def mean_abs_offset_so_far(self) -> float:
        """Running mean. `mean_abs_offset` itself stays a SUM until the run ends."""
        return self.mean_abs_offset / max(1, self.detected)


def _default_detector(cfg: WorldConfig) -> LaneDetector:
    """Detector whose px_per_m matches the world's scale at the lookahead row."""
    base = LaneConfig(bottom_row_m=cfg.y_near_m)
    h = cfg.size[0]
    y_look_m = cfg.y_near_m + (h - 1 - lookahead_row(base, h)) / cfg.px_per_m
    return LaneDetector(
        LaneConfig(px_per_m=cfg.scale_at(y_look_m), bottom_row_m=cfg.y_near_m)
    )


def _pace(tick_start: float, dt: float) -> None:
    """Sleep so one tick costs about dt of wall time (dashboard viewing only)."""
    remaining = dt - (time.perf_counter() - tick_start)
    if remaining > 0:
        time.sleep(remaining)


def _lane_points(world: SimWorld, step_m: float = LANE_STEP_M) -> list:
    """Centreline polyline sampled for display (world.path is sampled every 1 cm)."""
    pts = world.path.points
    if len(pts) < 2:
        return []
    spacing = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).mean())
    stride = max(1, int(round(step_m / max(spacing, 1e-9))))
    idx = list(range(0, len(pts), stride))
    if idx[-1] != len(pts) - 1:
        idx.append(len(pts) - 1)
    return [[float(p[0]), float(p[1])] for p in pts[idx]]


def run_sim(
    scenario: str | Scenario,
    *,
    detector: LaneDetector | None = None,
    policy: LanePolicy | None = None,
    policy_cfg: LanePolicyConfig | None = None,
    dt: float = 0.05,
    max_steps: int = 1200,
    seed: int = 0,
    world_cfg: WorldConfig | None = None,
    overlay_path: str | None = None,
    record: bool = True,
    bridge: Bridge | None = None,
    realtime: bool = False,
) -> SimResult:
    """Drive one scenario through the real step() loop. Never touches hardware.

    max_steps <= 0 runs until the scenario ends or a control stops it (dashboard).
    bridge: viewer that receives a SimState + frame per tick and can push controls.
    realtime: pace the loop to ~dt of wall time per tick, for watching.
    """
    if isinstance(scenario, str):
        scenario = get_scenario(scenario)
    cfg = world_cfg or WorldConfig(seed=seed)
    det = detector or _default_detector(cfg)
    pol = policy or LanePolicy(policy_cfg)
    view = bridge if bridge is not None else NullBridge()

    sm = RobotStateMachine()
    buttons = QueueButtons()
    sim_time = [0.0]
    cell: dict = {"world": None, "res": None, "lane": [], "trail": [],
                  "lane_width_m": 0.0, "corridor_m": 0.0}
    last: dict = {"frame": None, "meas": None, "debug": {}}

    def load(sc: Scenario) -> None:
        """(Re)build world + metrics. The robot then waits for START (real flow)."""
        cell["world"] = SimWorld(build_path(sc.segments, sc.start_offset_m), cfg)
        cell["res"] = SimResult(scenario=sc.name)
        sm.new_attempt()
        sim_time[0] = 0.0
        w = cell["world"]
        cell["lane"] = _lane_points(w)
        cell["trail"] = [[w.x, w.y, w.theta]]
        cell["lane_width_m"] = w.cfg.lane_width_m
        cell["corridor_m"] = w.cfg.corridor_m

    load(scenario)

    def send(cmd: Command) -> None:
        cell["world"].apply(cmd, dt)
        sim_time[0] += dt

    actuator = SafeActuator(
        CallbackActuator(send),
        max_speed_mps=cfg.max_speed_mps,
        timeout_s=0.3,
        clock=lambda: sim_time[0],
    )

    def get_frame():
        f = cell["world"].frame()
        last["frame"] = f
        return f

    camera = CallbackCamera(get_frame)
    buttons.push(Event.START)

    def policy_fn(frame, _sm):
        res = cell["res"]
        world = cell["world"]
        meas = None
        debug: dict = {}
        if frame is not None:
            meas, debug = det.detect_with_debug(frame)
        last["meas"] = meas
        last["debug"] = debug
        res.steps += 1  # one driving tick (policy_fn only runs while RUN)
        if meas is None:
            res.lost += 1
        else:
            res.detected += 1
            res.departures += int(abs(meas.offset_m) > DEPARTURE_M)
            res.max_abs_offset = max(res.max_abs_offset, abs(meas.offset_m))
            res.mean_abs_offset += abs(meas.offset_m)
        cmd = pol.command(meas)
        if cmd == STOP:
            res.stop_frames += 1
        if record:
            res.history.append(
                (world.t, meas.offset_m if meas else None, cmd.speed, cmd.curvature,
                 world.true_offset_at(1.0))
            )
        return cmd

    def publish(cmd: Command, fps: float) -> None:
        res, world = cell["res"], cell["world"]
        h = cfg.size[0]
        look_row = lookahead_row(det.cfg, h)
        look_m = det.cfg.bottom_row_m + (h - 1 - look_row) / det.cfg.px_per_m
        meas = last["meas"]
        view.publish(
            SimState(
                step=res.steps,
                t=world.t,
                scenario=res.scenario,
                robot_state=sm.state.value,
                checkpoints=sm.checkpoints,
                pose=Pose(world.x, world.y, world.theta),
                lookahead_m=look_m,
                true_offset_m=world.true_offset_at(look_m),
                measurement=None
                if meas is None
                else {
                    "offset_m": meas.offset_m,
                    "curvature_inv_m": meas.curvature_inv_m,
                    "confidence": meas.confidence,
                    "n_points": meas.n_points,
                    "lookahead_m": meas.lookahead_m,
                },
                debug=last["debug"],
                command={"speed": cmd.speed, "curvature": cmd.curvature},
                lane=cell["lane"],
                trail=cell["trail"],
                lane_width_m=cell["lane_width_m"],
                corridor_m=cell["corridor_m"],
                metrics={
                    "detected": res.detected,
                    "lost": res.lost,
                    "departures": res.departures,
                    "max_abs_offset": res.max_abs_offset,
                    "mean_abs_offset": res.mean_abs_offset_so_far,
                    "stop_frames": res.stop_frames,
                    "detection_rate": res.detection_rate,
                    "distance_m": world.distance,
                    "time_s": world.t,
                    "crashed": world.crashed,
                    "finished": world.finished,
                },
                fps=fps,
            ),
            last["frame"],
        )

    writer = None
    ticks = 0
    fps = 0.0
    tick_start = time.perf_counter()
    try:
        while max_steps <= 0 or ticks < max_steps:
            ctrl = view.poll_control()
            if ctrl is not None:
                if ctrl.action == "start":
                    buttons.push(Event.START)
                elif ctrl.action == "emergency":
                    buttons.push(Event.EMERGENCY)
                elif ctrl.action == "scenario":
                    load(get_scenario(ctrl.scenario))
                    buttons.clear()  # wait for a fresh START
                elif ctrl.action == "reset":
                    load(scenario)
                    buttons.clear()
            world = cell["world"]
            if world.crashed or world.finished:
                break
            cmd = step(sm, camera, buttons, actuator, policy_fn)
            ticks += 1
            cell["trail"].append([world.x, world.y, world.theta])
            if len(cell["trail"]) > TRAIL_MAX:
                del cell["trail"][: len(cell["trail"]) - TRAIL_MAX]
            if overlay_path is not None and last["frame"] is not None:
                writer = _overlay_writer(writer, overlay_path, last["frame"])
                _draw_overlay(last["frame"], last["meas"], world, det.cfg)
                writer.write(last["frame"])
            now = time.perf_counter()
            instant = 1.0 / max(now - tick_start, 1e-9)
            fps = instant if fps == 0.0 else 0.9 * fps + 0.1 * instant
            tick_start = now
            publish(cmd, fps)
            if realtime:
                _pace(tick_start, dt)
    finally:
        camera.close()
        if writer is not None:
            writer.release()

    res = cell["res"]
    res.distance_m = cell["world"].distance
    res.time_s = cell["world"].t
    res.mean_abs_offset /= max(1, res.detected)
    res.crashed = cell["world"].crashed
    res.finished = cell["world"].finished
    return res


def _overlay_writer(writer, path, frame):
    import cv2

    if writer is None:
        fh, fw = frame.shape[:2]
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 1.0 / 0.05, (fw, fh))
    return writer


def _draw_overlay(frame, meas, world: SimWorld, det_cfg: LaneConfig) -> None:
    """Green = detected lane centre, blue = true lane centre, at the lookahead row."""
    import cv2

    h, w = frame.shape[:2]
    v = int(lookahead_row(det_cfg, h))
    true_off = world.true_offset_at(det_cfg.bottom_row_m + (h - 1 - v) / det_cfg.px_per_m)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
    if true_off is not None:
        cv2.circle(frame, (int(w / 2 + true_off * det_cfg.px_per_m), v), 4, (255, 0, 0), 1)
    if meas is not None:
        cv2.circle(frame, (int(w / 2 + meas.offset_m * det_cfg.px_per_m), v), 6, (0, 0, 255), 2)
        cv2.putText(
            frame,
            f"off {meas.offset_m:+.2f}m  curv {meas.curvature_inv_m:+.2f}  "
            f"conf {meas.confidence:.2f}",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )
    else:
        cv2.putText(frame, "lane lost", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


def _repo_root() -> Path:
    """Walk up from this file until a directory containing configs/sim.yaml."""
    for base in Path(__file__).resolve().parents:
        if (base / "configs" / "sim.yaml").is_file():
            return base
    return Path.cwd()


def _sim_config() -> dict:
    """Defaults from configs/sim.yaml (section `sim:`), if present."""
    path = _repo_root() / "configs" / "sim.yaml"
    if not path.exists():
        return {}
    import yaml

    return (yaml.safe_load(path.read_text()) or {}).get("sim") or {}


def _print_result(res: SimResult) -> None:
    status = "CRASHED" if res.crashed else ("finished" if res.finished else "stopped (max steps)")
    print(f"scenario : {res.scenario}  -> {status}")
    print(f"steps    : {res.steps}  ({res.time_s:.1f} s sim time, {res.distance_m:.1f} m)")
    print(f"detected : {res.detected} ({100 * res.detection_rate:.0f}%), lost {res.lost}")
    print(f"offset   : mean |{res.mean_abs_offset:.3f}| m, max |{res.max_abs_offset:.3f}| m, "
          f"departures (>{DEPARTURE_M} m) {res.departures}")
    print(f"stops    : {res.stop_frames} frames commanded STOP")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="mav.sim.runner")
    parser.add_argument("scenario", nargs="?", help=" / ".join(sorted(SCENARIOS)))
    parser.add_argument("--steps", type=int, help="max control ticks (0 = until the scenario ends)")
    parser.add_argument("--dt", type=float, help="seconds per control tick")
    parser.add_argument("--seed", type=int, help="noise seed")
    parser.add_argument("--overlay", help="write an overlay video to this path")
    parser.add_argument("--realtime", action="store_true", help="pace the loop to dt per tick")
    parser.add_argument("--dashboard", action="store_true",
                        help="serve the web dashboard while running (configs/sim.yaml)")
    parser.add_argument("--list", action="store_true", help="list scenarios and exit")
    args = parser.parse_args(argv)

    if args.list:
        for s in SCENARIOS.values():
            print(f"{s.name:10s} {s.description}")
        return 0

    cfg = _sim_config()
    name = args.scenario or cfg.get("scenario", "straight")
    dash_cfg = cfg.get("dashboard") or {}
    # Server only starts on request: --dashboard, or enabled: true in the config.
    # deploy.yaml has no dashboard section, so it can never start there.
    dash = dash_cfg if (args.dashboard or dash_cfg.get("enabled")) else {}
    bridge = None
    server = None
    stop_server = None
    if dash:
        from mav.sim.dashboard import server as dashboard_server

        server, bridge = dashboard_server.serve_in_thread(dash)
        stop_server = dashboard_server.stop

    res = run_sim(
        name,
        dt=args.dt if args.dt is not None else float(cfg.get("dt", 0.05)),
        max_steps=args.steps if args.steps is not None else int(cfg.get("max_steps", 1200)),
        seed=args.seed if args.seed is not None else int(cfg.get("seed", 0)),
        overlay_path=args.overlay or cfg.get("overlay"),
        realtime=args.realtime or bool(dash.get("realtime", False)),
        bridge=bridge,
    )
    _print_result(res)
    if server is not None:
        print(f"dashboard: http://{dash.get('host', '127.0.0.1')}:{dash.get('port', 8000)}")
        try:
            input("press Enter to stop the dashboard... ")
        except (EOFError, KeyboardInterrupt):
            pass
        stop_server(server)
    return 0 if not res.crashed else 1


if __name__ == "__main__":
    raise SystemExit(main())

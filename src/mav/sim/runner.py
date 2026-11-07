"""Closed-loop simulator runner.

Runs the REAL control loop (mav.app.step) with the real detector and policy;
only the camera and actuator layers are simulated. That is the whole point:
whatever is tuned here transfers to the robot unchanged.

Usage: `python -m mav.sim.runner [scenario] [--steps N] [--dt S] [--seed S]
[--overlay out.mp4] [--list]`. Also reachable via `scripts/run.sh sim`
(MAV_PROFILE=sim), which reads defaults from configs/sim.yaml.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

from mav.app import step
from mav.control.state_machine import RobotStateMachine
from mav.io.actuator import CallbackActuator, SafeActuator
from mav.io.camera import CallbackCamera
from mav.io.buttons import Event, ScriptedButtons
from mav.perception.lane import LaneConfig, LaneDetector, lookahead_row
from mav.planning.lane_policy import LanePolicy, LanePolicyConfig
from mav.sim.scenarios import SCENARIOS, Scenario, get_scenario
from mav.sim.world import LanePath, SimWorld, WorldConfig, build_path
from mav.types import Command, STOP

DEPARTURE_M = 0.25  # |offset| beyond this counts as leaving the lane centre corridor


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


def _default_detector(cfg: WorldConfig) -> LaneDetector:
    """Detector whose px_per_m matches the world's scale at the lookahead row."""
    base = LaneConfig(bottom_row_m=cfg.y_near_m)
    h = cfg.size[0]
    y_look_m = cfg.y_near_m + (h - 1 - lookahead_row(base, h)) / cfg.px_per_m
    return LaneDetector(
        LaneConfig(px_per_m=cfg.scale_at(y_look_m), bottom_row_m=cfg.y_near_m)
    )


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
) -> SimResult:
    """Drive one scenario through the real step() loop. Never touches hardware."""
    if isinstance(scenario, str):
        scenario = get_scenario(scenario)
    cfg = world_cfg or WorldConfig(seed=seed)
    world = SimWorld(build_path(scenario.segments, scenario.start_offset_m), cfg)
    det = detector or _default_detector(cfg)
    pol = policy or LanePolicy(policy_cfg)

    sm = RobotStateMachine()
    sm.boot_done()
    buttons = ScriptedButtons({0: [Event.START]})
    sim_time = [0.0]

    def send(cmd: Command) -> None:
        world.apply(cmd, dt)
        sim_time[0] += dt

    actuator = SafeActuator(
        CallbackActuator(send),
        max_speed_mps=cfg.max_speed_mps,
        timeout_s=0.3,
        clock=lambda: sim_time[0],
    )

    last: dict = {"frame": None, "meas": None}

    def get_frame():
        f = world.frame()
        last["frame"] = f
        return f

    camera = CallbackCamera(get_frame)
    res = SimResult(scenario=scenario.name)

    def policy_fn(frame, _sm):
        meas = det.detect(frame) if frame is not None else None
        last["meas"] = meas
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

    writer = None
    try:
        while res.steps < max_steps and not world.crashed and not world.finished:
            step(sm, camera, buttons, actuator, policy_fn)
            res.steps += 1
            if overlay_path is not None and last["frame"] is not None:
                writer = _overlay_writer(writer, overlay_path, last["frame"])
                _draw_overlay(last["frame"], last["meas"], world, det.cfg)
                writer.write(last["frame"])
    finally:
        camera.close()
        if writer is not None:
            writer.release()

    res.distance_m = world.distance
    res.time_s = world.t
    res.mean_abs_offset /= max(1, res.detected)
    res.crashed = world.crashed
    res.finished = world.finished
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


def _sim_config() -> dict:
    """Defaults from configs/sim.yaml (section `sim:`), if present."""
    path = Path(__file__).resolve().parents[2] / "configs" / "sim.yaml"
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
    parser.add_argument("--steps", type=int, help="max control ticks")
    parser.add_argument("--dt", type=float, help="seconds per control tick")
    parser.add_argument("--seed", type=int, help="noise seed")
    parser.add_argument("--overlay", help="write an overlay video to this path")
    parser.add_argument("--list", action="store_true", help="list scenarios and exit")
    args = parser.parse_args(argv)

    if args.list:
        for s in SCENARIOS.values():
            print(f"{s.name:10s} {s.description}")
        return 0

    cfg = _sim_config()
    name = args.scenario or cfg.get("scenario", "straight")
    res = run_sim(
        name,
        dt=args.dt if args.dt is not None else float(cfg.get("dt", 0.05)),
        max_steps=args.steps if args.steps is not None else int(cfg.get("max_steps", 1200)),
        seed=args.seed if args.seed is not None else int(cfg.get("seed", 0)),
        overlay_path=args.overlay or cfg.get("overlay"),
    )
    _print_result(res)
    return 0 if not res.crashed else 1


if __name__ == "__main__":
    raise SystemExit(main())

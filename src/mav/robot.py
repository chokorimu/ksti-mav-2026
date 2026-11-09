"""Real-robot runner for the prototype/final profiles.

camera -> lane detector -> cone detector -> avoid -> LanePolicy -> mapper -> SerialActuator -> Arduino

Same mav.app.step() loop as the simulator; only the camera and actuator are real.
Usage (via scripts/run.sh):
    run.sh prototype --dry-run --video data/recordings/x.mp4   # no hardware at all
    run.sh prototype --dry-run                                  # real camera, no serial
    run.sh prototype                                            # camera + Arduino (wheels lifted first!)
Controls (terminal): `s` + Enter = START, ANY other line (or just Enter) = EMERGENCY, Ctrl+C = stop and exit.
This is NOT the physical emergency stop.
"""
from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from dataclasses import fields, replace

from mav.planning.avoid import choose_offset
from mav.types import STOP, Command

_EXPECTED_DRIVE = {"prototype": "differential", "final": "ackermann"}
_ROBOT_KEYS = {"rate_hz", "max_hold_s"}
_UNO_RESET_S = 2.0  # the Uno reboots when the serial port is opened; commands sent earlier are lost


class Driver:
    """policy_fn for mav.app.step: detect -> avoid cones -> count passes -> policy, with a BOUNDED hold on a lost lane.

    LanePolicy(on_lost="hold") returns the `last` command it is given; the policy keeps no state, so this
    class passes it in and stops the robot if the lane stays lost longer than max_hold_s.

    Cone avoidance is applied to the measured lane offset, not to the command: the policy keeps steering
    at the lane, just at a shifted target. With no cone detector (or `avoid_cfg.enabled: false`) the
    measurement reaches the policy untouched, so behaviour is unchanged.

    `cone_tracker` produces the ARC score: in the ARC, checkpoints ARE the obstacles, so a cone that is
    physically passed calls `sm.add_checkpoint()`. That is a separate object from `cones` (which only
    answers "what is in front of me now") because counting needs history and detection must not.
    """

    def __init__(self, detector, policy, tracker=None, max_hold_s: float = 0.5,
                 clock=time.monotonic, cones=None, avoid_cfg=None, cone_tracker=None):
        self._detector, self._policy, self._tracker = detector, policy, tracker
        self._max_hold_s, self._clock = max_hold_s, clock
        self._cones, self._avoid_cfg = cones, avoid_cfg
        self._cone_tracker = cone_tracker
        self.last: Command = STOP
        self.meas = None
        self.lost = False
        self.avoid: dict = {}
        self._lost_since: float | None = None

    def reset(self) -> None:
        self.last, self._lost_since = STOP, None

    def __call__(self, frame, sm) -> Command:
        meas = None
        if frame is not None:
            meas = self._detector.detect_with_debug(frame)[0]
            if self._tracker is not None:
                ev = self._tracker.update(frame)
                if ev is not None:
                    sm.finish() if ev.kind == "finish" else sm.add_checkpoint()
            if self._cone_tracker is not None and self._cone_tracker.update(frame) is not None:
                # ARC §8: passing an obstacle counts as passing the checkpoint.
                sm.add_checkpoint()
        pc = self._policy.cfg
        # The lost test uses the MEASURED offset, not the swerved target: the lane
        # is either seen or it is not, regardless of which way we dodge.
        lost = meas is None or meas.confidence < pc.conf_min or abs(meas.offset_m) > pc.max_offset_m
        now = self._clock()
        if not lost:
            self._lost_since = None
        elif self._lost_since is None:
            self._lost_since = now
        steered = meas
        if meas is not None and self._cones is not None:
            target, self.avoid = choose_offset(meas.offset_m, self._cones.detect(frame), self._avoid_cfg)
            if target != meas.offset_m:
                steered = replace(meas, offset_m=target)
        cmd = self._policy.command(steered, last=self.last)
        if lost and now - self._lost_since > self._max_hold_s:
            cmd = STOP
        self.last, self.meas, self.lost = cmd, meas, lost
        return cmd


class _LinePort:
    """Fake serial port for --dry-run: keeps the last protocol line."""

    def __init__(self) -> None:
        self.last = ""

    def write(self, data: bytes) -> None:
        self.last = data.decode("ascii").strip()


def _check_keys(section: dict, allowed: set, where: str) -> dict:
    unknown = sorted(set(section) - allowed)
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {unknown}; valid: {sorted(allowed)}")
    return section


def _stdin_buttons(buttons, Event) -> None:
    for line in sys.stdin:
        buttons.push(Event.START if line.strip().lower() in ("s", "start") else Event.EMERGENCY)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="mav robot")
    ap.add_argument("--dry-run", action="store_true", help="no serial port; print the protocol lines")
    ap.add_argument("--video", help="replay a recording instead of the camera")
    ap.add_argument("--port", help="serial port, overrides configs/<profile>.yaml")
    ap.add_argument("--hz", type=float, help="control rate, overrides configs/<profile>.yaml")
    args = ap.parse_args(argv)

    # Heavy imports here so Driver stays importable (and testable) without the rest of the repo.
    from mav.app import step
    from mav.config import (load_avoid_config, load_checkpoint_config, load_lane_config,
                            load_obstacle_config, load_profile, profile_name, repo_root)
    from mav.control.mapping import AckermannMapper, DifferentialMapper
    from mav.control.state_machine import RobotStateMachine
    from mav.io.actuator import SafeActuator, SerialActuator, open_serial_port
    from mav.io.buttons import Event, QueueButtons
    from mav.io.camera import open_csi, open_usb, open_video
    from mav.perception.checkpoint import CrossLineDetector, CrossLineTracker
    from mav.perception.lane import LaneDetector
    from mav.perception.obstacle import ConeDetector, ConePassTracker
    from mav.planning.lane_policy import LanePolicy, LanePolicyConfig
    from mav.types import load_vehicle_params

    # ---- 1. validate ALL configuration before touching any hardware ----
    profile = profile_name()
    cfg = load_profile(profile)
    where = f"configs/{profile}.yaml"
    robot = _check_keys(cfg.get("robot") or {}, _ROBOT_KEYS, where + " robot")
    hz = args.hz or float(robot.get("rate_hz", 20.0))
    max_hold_s = float(robot.get("max_hold_s", 0.5))
    pol_sec = cfg.get("policy") or {}
    _check_keys(pol_sec, {f.name for f in fields(LanePolicyConfig)}, where + " policy")
    policy = LanePolicy(LanePolicyConfig(**pol_sec))
    params = load_vehicle_params(repo_root() / "configs" / "vehicle.yaml")
    expected = _EXPECTED_DRIVE.get(profile)
    if expected and params.drive != expected:
        raise SystemExit(f"profile {profile!r} needs drive={expected!r} in vehicle.yaml, got {params.drive!r}")
    mapper = DifferentialMapper(params) if params.drive == "differential" else AckermannMapper(params)
    lane_cfg, cp_cfg = load_lane_config(profile), load_checkpoint_config(profile)
    detector = LaneDetector(lane_cfg)
    tracker = (CrossLineTracker(CrossLineDetector(cp_cfg))
               if cp_cfg.checkpoint_hues or cp_cfg.finish_hues else None)
    obs_cfg, avoid_cfg = load_obstacle_config(profile), load_avoid_config(profile)
    # No cones expected or swerving turned off: skip the detectors, they are a
    # full-frame HSV pass each and would be pure cost.
    if obs_cfg.hues:
        cones = ConeDetector(obs_cfg) if avoid_cfg.enabled else None
        # Counting is independent of swerving: ARC scores cones passed, so the
        # tracker runs even with avoidance disabled (robot would hit them, but
        # the count stays honest about what it did).
        cone_tracker = ConePassTracker(ConeDetector(obs_cfg))
    else:
        cones = cone_tracker = None
    cam_sec = cfg.get("camera") or {}
    act_sec = cfg.get("actuator") or {}
    port_name = args.port or act_sec.get("port")
    if not args.dry_run and not port_name:
        raise SystemExit(f"{where}: actuator.port is not set (or use --dry-run / --port)")

    # ---- 2. open hardware: camera first, serial last ----
    if args.video:
        camera = open_video(args.video)
    elif cam_sec.get("kind") == "csi":
        camera = open_csi(sensor_id=int(cam_sec.get("sensor_id", 0)))
    elif cam_sec.get("kind") == "usb":
        camera = open_usb(int(cam_sec.get("index", 0)))
    else:
        raise SystemExit(f"{where}: camera.kind must be 'csi' or 'usb' (or use --video)")
    ser = _LinePort() if args.dry_run else open_serial_port(port_name)
    if not args.dry_run:
        time.sleep(_UNO_RESET_S)
    actuator = SafeActuator(SerialActuator(mapper, ser), max_speed_mps=params.max_speed_mps, timeout_s=0.3)

    driver = Driver(detector, policy, tracker, max_hold_s, cones=cones,
                    avoid_cfg=avoid_cfg, cone_tracker=cone_tracker)
    buttons = QueueButtons()
    sm = RobotStateMachine()
    sm.boot_done()
    threading.Thread(target=_stdin_buttons, args=(buttons, Event), daemon=True).start()

    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)

    mode = "DRY-RUN" if args.dry_run else f"serial {port_name}"
    print(f"profile={profile} drive={params.drive} {hz:g} Hz {mode}\n"
          "type `s` + Enter = START | Enter or anything else = EMERGENCY | Ctrl+C = exit")
    period, last_report, no_frame = 1.0 / hz, 0.0, 0
    try:
        while True:
            t0 = time.perf_counter()
            if not sm.is_running:
                driver.reset()
            if getattr(sm, "finished", False):
                actuator.disable()
            else:
                step(sm, camera, buttons, actuator, driver)
            if args.video and driver.meas is None and sm.is_running:
                no_frame += 1
                if no_frame > 20:
                    print("end of video")
                    break
            ms = (time.perf_counter() - t0) * 1000.0
            if t0 - last_report >= 1.0:
                last_report = t0
                m = driver.meas
                off = f"{m.offset_m:+.2f}" if m else "  -  "
                print(f"{sm.state.value:10s} cp={sm.checkpoints} lost={int(driver.lost)} off={off} "
                      f"cmd=({driver.last.speed:.2f},{driver.last.curvature:+.2f}) "
                      f"avo={driver.avoid.get('action', '-')} "
                      f"line={ser.last if args.dry_run else '-'} {ms:.0f}ms")
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\nstopping")
    except OSError as e:
        print(f"serial/camera error: {e}", file=sys.stderr)
        return 1
    finally:
        for fn in (actuator.disable, actuator.stop, camera.close):
            try:
                fn()
            except Exception:
                pass
        if not args.dry_run:
            try:
                ser.close()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

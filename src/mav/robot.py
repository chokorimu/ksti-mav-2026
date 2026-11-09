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

from mav.io.actuator import SerialUnavailable, list_serial_ports
from mav.planning.avoid import choose_offset
from mav.types import STOP, Command

_EXPECTED_DRIVE = {"prototype": "differential", "final": "ackermann"}
_ROBOT_KEYS = {"rate_hz", "max_hold_s"}
_BOARD_RESET_S = 2.0  # opening the port resets the board (DTR); commands sent earlier are lost


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


def _autodetect_port(where: str) -> str:
    """The single Arduino plugged in, or a message saying why that cannot be decided."""
    try:
        found = list_serial_ports()
    except SerialUnavailable as exc:  # pyserial missing
        raise SystemExit(f"{exc}") from None
    if len(found) == 1:
        return found[0]
    if not found:
        raise SystemExit(
            f"{where}: actuator.port is not set and no Arduino was found over USB.\n"
            "The Nano gets power from USB, so if it is connected it should appear.\n"
            "Check on this machine:\n"
            "  scripts/flash_firmware.sh prototype --list-ports   # what the runner can see\n"
            "  lsusb -t | grep -i 'ch34\\|cp210\\|ftdi'            # which driver is BOUND\n"
            "  lsusb | grep -i '1a86\\|2341'                       # 1a86 = CH340 clone, 2341 = genuine\n"
            "  dmesg | tail -5                                    # does the kernel see it at all?\n"
            "A CH340 is NOT always /dev/ttyUSB0: WCH's own driver (usb_ch341) makes\n"
            "/dev/ttyCH341USB0 instead, so check /dev/ttyCH341* too.\n"
            "If the device exists but cannot be OPENED, you may need: "
            "sudo usermod -aG dialout $USER (then log out and back in).\n"
            "Or pass --port <the device name>."
        )
    raise SystemExit(
        f"{where}: actuator.port is not set and {len(found)} serial devices are "
        f"plugged in: {', '.join(found)}.\nPass --port to choose one."
    )


def _open_serial_or_exit(port: str):
    """Open the Arduino port, or exit with a message an operator can act on."""
    try:
        return open_serial_port(port)
    except SerialUnavailable as exc:
        raise SystemExit(f"{exc}") from None


def _first_frame(camera, timeout_s: float = 5.0):
    """Read one frame, or None after `timeout_s`.

    Called once before the control loop. Two reasons: a camera that opens but
    never delivers is otherwise indistinguishable from a robot that ignores its
    commands, and the frame size is needed to sanity-check `lane.px_per_m`.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = camera.read()
        if frame is not None:
            return frame
        time.sleep(0.05)
    return None


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
    from mav.io.actuator import (SafeActuator, SerialActuator, open_serial_port,
                                 require_serial)
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
        # No port configured: use the one board that is plugged in. AGENTS.md
        # forbids hardcoding serial port names, and the split between a genuine
        # Nano (/dev/ttyACM*) and a CH340 clone (/dev/ttyUSB*) means a hardcoded
        # name is wrong on half the boards - including the one on this desk.
        port_name = _autodetect_port(where)

    # ---- 2. open hardware: preflight, camera, then serial ----
    # The serial preflight comes FIRST, before the camera. With pyserial missing
    # the traceback used to appear directly after the Argus/GStreamer log, so a
    # missing Python package looked exactly like a camera fault (Jetson, 10 Nov
    # 2026). The camera also takes a second to start, so nothing is lost.
    if not args.dry_run:
        try:
            require_serial()
        except SerialUnavailable as exc:
            raise SystemExit(f"{exc}") from None
    if args.video:
        camera, cam_desc = open_video(args.video), f"video {args.video}"
    elif cam_sec.get("kind") == "csi":
        camera = open_csi(sensor_id=int(cam_sec.get("sensor_id", 0)))
        cam_desc = f"csi sensor {cam_sec.get('sensor_id', 0)}"
    elif cam_sec.get("kind") == "usb":
        camera = open_usb(int(cam_sec.get("index", 0)))
        cam_desc = f"usb camera {cam_sec.get('index', 0)}"
    else:
        raise SystemExit(f"{where}: camera.kind must be 'csi' or 'usb' (or use --video)")

    # Prove the camera really delivers, and print the geometry. A camera that
    # opens but never produces a frame otherwise looks like a robot that simply
    # refuses to move; and `lane.px_per_m` is pixels per metre AT THIS FRAME
    # SIZE, so a resolution change rescales the entire lane model silently.
    first = _first_frame(camera)
    if first is None:
        camera.close()
        raise SystemExit(
            f"{cam_desc} opened but delivered no frame within 5 s. "
            "Check the ribbon cable and `ls /dev/video*`."
        )
    shape = getattr(first, "shape", None)
    if shape is not None and len(shape) >= 2:
        print(f"camera {cam_desc}: {shape[1]}x{shape[0]} - "
              f"lane.px_per_m is calibrated for THIS frame size")

    ser = _LinePort() if args.dry_run else _open_serial_or_exit(port_name)
    if not args.dry_run:
        time.sleep(_BOARD_RESET_S)
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

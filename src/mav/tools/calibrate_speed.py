"""Measure the speed calibration: PWM per m/s, and the breakaway PWM.

Why this exists. The robot has no encoders, so `mps -> PWM` is open loop and it is
the only link between a commanded speed and a real one. Both firmware sketches
ship with `MPS_TO_PWM = 0`, which makes every command PWM 0: the car receives
valid commands and never moves. That is the single blocker for motion.

Why TWO numbers and not one. A DC motor does not turn at all below a breakaway
PWM - the L298N's voltage drop plus static friction hold it still - and only
above that does speed rise roughly linearly. So

    pwm = deadband + speed * pwm_per_mps        (0 for speed == 0)

A single gain is what everyone reaches for, and it is wrong in the place that
matters: measuring the top speed at PWM 255 and setting `K = 255 / top` draws a
line through the ORIGIN, which OVERSTATES the speed at low PWM. The policy drives
at 0.15-0.3 m/s, i.e. the low end, so that error lands exactly on the speeds the
car actually uses - and it shows up as a car that either refuses to creep or
lurches.

Why a tool and not the serial monitor. `protocol.md` has a 500 ms watchdog: if no
valid line arrives, the firmware stops. Typing commands by hand cannot keep up,
so a sweep has to be streamed by a program.

Measuring. The car must be ON THE FLOOR. A lifted wheel spins faster than a loaded
one, so a wheels-lifted sweep measures the wrong thing (it is also unsafe to
extrapolate from: the car is calibrated to drive on the floor). For each PWM the
tool holds the command for `--seconds`, stops, and asks how far the car went.
Speed = distance / time, which is why TIMING is the operator's job: use the same
stopwatch, the same floor, and the same battery charge for every point.

Usage:
    PYTHONPATH=src python -m mav.tools.calibrate_speed --port /dev/ttyACM0 \\
        --k-current 300 --pwm 50 70 90 120 150 180

    # no hardware: print the sweep and the fit arithmetic for made-up data
    PYTHONPATH=src python -m mav.tools.calibrate_speed --dry-run

The tool prints the two constants to paste into the firmware. It does NOT write
them: a measured value should be recorded in docs/hardware.md on purpose.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

PWM_MAX = 255  # Uno/AVR analogWrite range (firmware/arduino/*/src/*.ino)


@dataclass(frozen=True)
class SpeedModel:
    """`pwm = deadband + speed * pwm_per_mps`, the firmware's own model."""

    pwm_per_mps: float
    deadband: float
    rms_mps: float  # residual of the fit, in m/s: how well it explains the data
    n_points: int
    deadband_clamped: bool = False

    def pwm_for(self, speed_mps: float) -> int:
        if speed_mps == 0.0:
            return 0
        mag = abs(speed_mps) * self.pwm_per_mps + self.deadband
        pwm = min(int(mag + 0.5), PWM_MAX)
        return pwm if speed_mps > 0 else -pwm

    def speed_for(self, pwm: int) -> float:
        """The speed the model predicts for a PWM. Below the deadband: 0."""
        mag = abs(pwm) - self.deadband
        if mag <= 0.0:
            return 0.0
        return mag / self.pwm_per_mps


def fit_speed_model(points: list[tuple[float, float]]) -> SpeedModel:
    """Least squares of speed on PWM, inverted into the firmware's form.

    Regressed as speed(PWM) and then inverted, not the other way round, because
    the quantity that has an error is the MEASURED speed; minimising PWM
    residuals would be minimising a variable we set exactly.

    Points that measured 0 m/s are excluded from the fit and not silently
    dropped: they are real information about the deadband (they say it is at
    least that PWM), so `deadband` is never allowed below the largest PWM that
    failed to move the car.
    """
    pts = [(float(p), float(v)) for p, v in points]
    if any(not math.isfinite(p) or not math.isfinite(v) for p, v in pts):
        raise ValueError("non-finite measurement")
    moved = [(p, v) for p, v in pts if v > 0.0]
    stalled = [p for p, v in pts if v <= 0.0]
    if len(moved) < 2:
        raise ValueError(
            f"need at least 2 PWM levels that actually moved the car, got {len(moved)}"
        )

    n = len(moved)
    mean_p = sum(p for p, _ in moved) / n
    mean_v = sum(v for _, v in moved) / n
    sxx = sum((p - mean_p) ** 2 for p, _ in moved)
    if sxx <= 1e-9:
        raise ValueError("every moving measurement used the same PWM")
    sxy = sum((p - mean_p) * (v - mean_v) for p, v in moved)
    slope = sxy / sxx  # m/s per PWM count
    if slope <= 0.0:
        raise ValueError(
            "a higher PWM did not give a higher speed; the run is invalid "
            "(wheel slip, a binding wheel, or the car was held)"
        )
    intercept = mean_v - slope * mean_p
    pwm_per_mps = 1.0 / slope
    deadband = -intercept / slope

    # A negative breakaway means the fitted line passes ABOVE the origin (the
    # motor would move at PWM 0), which cannot be inverted to a positive PWM, so
    # 0 is the only physical answer. A sub-count negative is floating-point noise
    # on a line that really does pass through the origin - not worth an alarm.
    clamped = deadband < -1.0
    if deadband < 0.0:
        deadband = 0.0
    # A car that sat still at PWM p proves the deadband is at least p.
    if stalled:
        floor = max(stalled)
        if deadband < floor:
            deadband = float(floor)

    rms = math.sqrt(
        sum((slope * p + intercept - v) ** 2 for p, v in moved) / n
    )
    return SpeedModel(
        pwm_per_mps=pwm_per_mps,
        deadband=deadband,
        rms_mps=rms,
        n_points=n,
        deadband_clamped=clamped,
    )


def _vehicle_max_speed() -> float | None:
    """`max_speed_mps` from configs/vehicle.yaml, or None when it cannot be read.

    Read, not hardcoded: the ceiling is a measured/safety value that lives in
    config, and duplicating it here would let the two drift apart.
    """
    try:
        from mav.config import repo_root

        import yaml

        path = repo_root() / "configs" / "vehicle.yaml"
        cfg = yaml.safe_load(path.read_text()) or {}
        value = cfg.get("max_speed_mps")
        return float(value) if value is not None else None
    except Exception:  # a missing/broken config must not block calibration
        return None


# ------------------------------------------------------------------ hardware ----


def _command_line(drive: str, speed_mps: float) -> str:
    """The protocol line for a straight run at `speed_mps` (protocol.md)."""
    if drive == "ackermann":
        return f"D 0.0000 {speed_mps:.3f}"  # steering 0 = straight
    return f"W {speed_mps:.3f} {speed_mps:.3f}"


def _open_serial(port: str, baud: int = 115200):
    try:
        import serial  # lazy: pyserial is optional on the laptop
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise SystemExit(
            "pyserial is required for a real sweep: pip install pyserial"
        ) from exc
    return serial.Serial(port, baud, timeout=1)


def _hold(ser, drive: str, speed_mps: float, seconds: float, hz: float = 20.0) -> None:
    """Stream one command for `seconds`.

    Streaming, not sending once: the firmware's watchdog stops the motors after
    500 ms without a valid line (protocol.md), so a single line would drive for
    half a second and then coast - which would be measured as a much lower speed.
    """
    line = (_command_line(drive, speed_mps) + "\n").encode("ascii")
    period = 1.0 / hz
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ser.write(line)
        ser.flush()
        time.sleep(period)
    ser.write(b"S\n")
    ser.flush()


def sweep(
    port: str,
    k_current: float,
    pwms: list[int],
    *,
    drive: str = "differential",
    seconds: float = 4.0,
    ask=input,
    max_speed_mps: float | None = None,
) -> list[tuple[float, float]]:
    """Run the sweep, returning [(pwm, speed_mps)] with the speed the operator read.

    `k_current` is the MPS_TO_PWM already flashed. It is only used to convert a
    PWM target into the m/s value the firmware expects: `pwm = mps * k_current`.
    The fit does not care whether `k_current` was a good number - it only has to
    match what the firmware actually holds, or the PWM printed here is not the
    PWM the motor received.
    """
    if not 0.0 < k_current:
        raise SystemExit("--k-current must be > 0 (the MPS_TO_PWM currently flashed)")
    # This function writes straight to the serial port, so it never passes
    # through SafeActuator and never sees its max_speed_mps clamp. Refusing here
    # is the only thing standing between a sweep and a car doing twice its
    # configured top speed on the operator's first run.
    for pwm in pwms:
        v = pwm / k_current
        if max_speed_mps is not None and v > max_speed_mps:
            raise SystemExit(
                f"PWM {pwm} means {v:.3f} m/s at --k-current {k_current:g}, above "
                f"max_speed_mps {max_speed_mps:g} in configs/vehicle.yaml.\n"
                "Lower the PWM list, or raise --max-speed deliberately."
            )
    ser = _open_serial(port)
    points: list[tuple[float, float]] = []
    try:
        time.sleep(2.0)  # opening the port resets the Uno; let it boot
        for pwm in pwms:
            speed_cmd = pwm / k_current
            print(f"\nPWM {pwm}  ->  commanding {speed_cmd:.3f} m/s for {seconds:g} s")
            ask("  press Enter to start the run (e-stop in hand)...")
            _hold(ser, drive, speed_cmd, seconds)
            print("  stopped. Measure how far the car travelled.")
            raw = ask("  distance in metres (blank = skip this point): ").strip()
            if not raw:
                print("  skipped")
                continue
            try:
                distance_m = float(raw)
            except ValueError:
                print(f"  not a number: {raw!r}, skipping")
                continue
            if distance_m < 0:
                print("  negative distance, skipping")
                continue
            points.append((float(pwm), distance_m / seconds))
            print(f"  = {distance_m / seconds:.3f} m/s")
    finally:
        try:
            ser.write(b"S\n")
            ser.flush()
        finally:
            ser.close()
    return points


# ------------------------------------------------------------------------ cli ----


def _report(model: SpeedModel, points: list[tuple[float, float]], drive: str) -> None:
    print("\n--- fit ---------------------------------------------------------")
    print(f"  points used     : {model.n_points}")
    print(f"  residual (rms)  : {model.rms_mps * 1000:.0f} mm/s")
    if model.deadband_clamped:
        print("  NOTE: the fitted line passed above the origin, so the breakaway")
        print("        was clamped to 0. Check the run before trusting this.")
    print()
    print("  Paste into the firmware (firmware/arduino/"
          + ("final" if drive == "ackermann" else "prototype") + "/src/):")
    print(f"    const float MPS_TO_PWM   = {model.pwm_per_mps:.1f}f;")
    print(f"    const float PWM_DEADBAND = {model.deadband:.1f}f;")
    print()
    print("  What that means in the speed range the policy uses:")
    for v in (0.15, 0.30):
        print(f"    {v:.2f} m/s -> PWM {model.pwm_for(v):3d}  "
              f"(model predicts {model.speed_for(model.pwm_for(v)):.3f} m/s)")
    print()
    print("  Check the model against the measurements:")
    for pwm, v in points:
        pred = model.speed_for(pwm)
        flag = "" if abs(pred - v) <= 0.05 else "   <-- off by >50 mm/s"
        print(f"    PWM {pwm:3.0f}: measured {v:.3f} m/s, model {pred:.3f} m/s{flag}")
    print()
    print("  Record both numbers in docs/hardware.md.")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="python -m mav.tools.calibrate_speed",
        description="Measure MPS_TO_PWM and PWM_DEADBAND for the Arduino firmware.",
    )
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyACM0")
    ap.add_argument(
        "--k-current",
        type=float,
        default=300.0,
        help="the MPS_TO_PWM currently flashed (used only to turn a PWM target "
        "into an m/s command). Default 300 is a starting point, not a measurement.",
    )
    ap.add_argument(
        "--pwm",
        type=int,
        nargs="+",
        default=[50, 70, 90, 120, 150, 180],
        help="PWM levels to sweep, low to high",
    )
    ap.add_argument(
        "--drive",
        choices=("differential", "ackermann"),
        default="differential",
        help="differential speaks W (prototype), ackermann speaks D (final)",
    )
    ap.add_argument("--seconds", type=float, default=4.0, help="run duration per point")
    ap.add_argument(
        "--max-speed",
        type=float,
        default=None,
        help="safety ceiling in m/s. Defaults to max_speed_mps from "
        "configs/vehicle.yaml, because this tool writes to the port directly and "
        "so bypasses SafeActuator's clamp.",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="print the sweep and a worked example without touching hardware",
    )
    args = ap.parse_args(argv)

    bad = [p for p in args.pwm if not 1 <= p <= PWM_MAX]
    if bad:
        print(f"PWM values out of range 1..{PWM_MAX}: {bad}")
        return 2
    if args.seconds <= 0:
        print("--seconds must be > 0")
        return 2

    if args.dry_run:
        print("dry run - no hardware is touched.\n")
        for pwm in args.pwm:
            print(f"  PWM {pwm:3d}  ->  {_command_line(args.drive, pwm / args.k_current)}")
        print("\nA worked fit, so the arithmetic is checkable by hand:")
        demo = [(60.0, 0.100), (100.0, 0.200), (140.0, 0.300)]
        model = fit_speed_model(demo)
        print(f"  points {demo}")
        print(f"  -> pwm_per_mps {model.pwm_per_mps:.1f}, deadband {model.deadband:.1f}")
        print("  (each point is pwm = 20 + speed * 400: 0.100 -> 60, "
              "0.200 -> 100, 0.300 -> 140)")
        return 0

    if not args.port:
        print("--port is required for a real sweep (or use --dry-run)")
        return 2

    ceiling = args.max_speed
    if ceiling is None:
        ceiling = _vehicle_max_speed()
        if ceiling is not None:
            print(f"safety ceiling from configs/vehicle.yaml: {ceiling:g} m/s")
    print("The car WILL move. Clear floor, e-stop in hand, battery charged,")
    print("and the same surface and charge for every point.")
    points = sweep(
        args.port,
        args.k_current,
        args.pwm,
        drive=args.drive,
        seconds=args.seconds,
        max_speed_mps=ceiling,
    )
    if not points:
        print("\nno measurements entered, nothing to fit")
        return 1
    try:
        model = fit_speed_model(points)
    except ValueError as exc:
        print(f"\ncannot fit: {exc}")
        return 1
    _report(model, points, args.drive)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

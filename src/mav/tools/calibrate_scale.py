"""Measure `px_per_m` from a recording by measuring the LANE, not the camera.

Why this exists: `px_per_m` (pixels per metre at the lookahead row) is the number
that turns every pixel measurement into metres, and it was a guess of 130. The
real recording measures ~10x that. Nobody can read pixels-per-metre off a ruler,
but everybody can measure a lane 0.50 m wide (rules §6, ±10%) in a picture:

    px_per_m = (pixels between the two edge lines at the lookahead row) / 0.50 m

So this script measures the edge-line span where the robot actually looks, calls
that the lane width, and solves for `px_per_m`. It needs no calibration target,
no tape measure, and no camera geometry: the lane is the ruler.

Run it on the robot's own camera recordings, NOT a phone clip - the value belongs
to the camera mount (height + pitch), so a different viewpoint gives a different
answer and that answer is wrong for the robot.

    PYTHONPATH=src python -m mav.tools.calibrate_scale <video> [--lane-m 0.50]

It prints the current value, the derived value, and the config line to paste.
It does NOT edit configs/ - a measured value should be committed on purpose.
"""
from __future__ import annotations

import sys

import numpy as np

from mav.config import load_lane_config, profile_name
from mav.io.camera import open_video
from mav.perception.lane import LaneConfig, LaneDetector


def _span_at_lookahead(detector: LaneDetector, frame: np.ndarray) -> tuple[float, float] | None:
    """(pixel span between the lane edges AT the lookahead row, confidence).

    Uses the detector's own band reports rather than re-finding the lines: if the
    detector cannot see the lane, this number is worthless, and AGREEING with the
    detector is the whole point - a scale derived from a different method would
    calibrate something the robot does not use.

    Two details that make or break this, both found by running it:

      1. `band_points` are per-band centroids spread over the WHOLE ROI (rows 7 to
         161 in the real clip). Taking max(col) - min(col) over all of them
         measures the span at the widest row - the bottom of the frame, nearest
         the robot - not at the lookahead row where the detector measures. With
         perspective in the picture that reported 4.7 px at p5 and 505 px at p95
         for the same lane. Only bands near the lookahead row may be used.
      2. The dashed centre line is a band too, so the OUTERMOST pair is the two
         edges and their span is the lane. Requiring at least two bands near the
         lookahead row is what keeps a single edge line from being read as a lane.
    """
    meas, dbg = detector.detect_with_debug(frame)
    if meas is None:
        return None
    pts = dbg.get("band_points") or []
    if len(pts) < 2:
        return None
    # `band_points` rows are ROI-relative; `lookahead_row` in the debug dict is
    # full-frame. Converting here (rather than changing lane.py) keeps the
    # detector's public contract alone.
    from mav.perception.lane import lookahead_row

    h = int(np.asarray(frame).shape[0])
    y_look_roi = lookahead_row(detector.cfg, h) - int(detector.cfg.roi_top * h)
    # Rows are ROI-relative and spaced ~14 px; take the ones within one band pitch.
    near = [p for p in pts if abs(float(p[0]) - y_look_roi) <= 16.0]
    if len(near) < 2:
        return None
    cols = sorted(float(p[1]) for p in near)
    return cols[-1] - cols[0], float(meas.confidence)


def calibrate(video_path: str, *, lane_m: float = 0.50, profile: str | None = None) -> int:
    name = profile_name(profile)
    try:
        current = load_lane_config(name)
    except FileNotFoundError:
        current = LaneConfig()
        print(f"note: no configs/{name}.yaml, using code defaults")

    # Detect with the CURRENT px_per_m (the pixel-side geometry barely depends on
    # it) and measure spans in pixels.
    detector = LaneDetector(current)
    cam = open_video(video_path)
    spans: list[float] = []
    frames = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            frames += 1
            got = _span_at_lookahead(detector, frame)
            if got is not None:
                span_px, conf = got
                if span_px > 10.0:  # a lane narrower than this is a mis-detection
                    spans.append(span_px)
    finally:
        cam.close()

    if frames == 0:
        print(f"no frames read from {video_path}")
        return 1
    if len(spans) < 20:
        print(
            f"only {len(spans)} usable frames out of {frames}: the detector is not "
            "seeing both edge lines at the lookahead row.\n"
            "Fix detection FIRST (lower roi_top, or recalibrate bottom_row_m), "
            "then re-run this. Do not guess a scale from this."
        )
        return 2

    arr = np.array(spans)
    p25, p50, p75 = (float(np.percentile(arr, q)) for q in (25, 50, 75))
    span_px = p50  # median, not mean: one badly-smoothed frame must not move it
    derived = span_px / lane_m

    print(f"video    : {video_path}")
    print(f"profile  : {name} (current px_per_m = {current.px_per_m:g})")
    print(f"frames   : {frames}, usable {len(spans)} ({100 * len(spans) / frames:.0f}%)")
    print(f"lane span: median {span_px:.1f} px  (p25 {p25:.1f}, p75 {p75:.1f})  "
          f"assumed {lane_m:g} m wide")

    # A lane does not change width by a factor of two between frames. If it looks
    # like it does, the BANDS are wrong (dashes merging, one edge line lost) and
    # the median is an average of garbage. Refusing here is the whole value of
    # this tool: a plausible-looking number from bad input would be baked into
    # the config and be indistinguishable from a measurement.
    spread = p75 / max(p25, 1e-9)
    if spread > 1.5:
        print()
        print(f"REFUSING to report a value: p75/p25 = {spread:.1f}x.")
        print("A 0.50 m lane cannot vary that much frame to frame, so the band")
        print("detection is unreliable on this clip (dashes merging into the edge")
        print("lines, or only one edge line seen). A number derived from this")
        print("would look like a measurement and not be one.")
        print()
        print("Fix detection first: check the overlay from")
        print(f"  python -m mav.perception.lane {video_path} --overlay /tmp/ov.mp4")
        print("and re-run this once BOTH edge lines are tracked cleanly.")
        return 2

    print()
    print(f"  derived px_per_m = {derived:.0f}")
    print(f"  current          = {current.px_per_m:g}")
    print(f"  ratio            = {derived / current.px_per_m:.2f}x")
    print()
    print("If that ratio is far from 1.0, every offset reported so far was wrong")
    print("by that factor - which is what 'the policy stops after 1.75 m' looks")
    print("like from the inside.")
    print()
    print(f"Paste into configs/{name}.yaml, then re-run the detector to confirm")
    print("the detection rate goes UP (a correct scale fixes the smear gate, which")
    print("is what rejects bands on a mis-scaled image):")
    print(f"  lane: {{ px_per_m: {derived:.0f} }}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    lane_m = 0.50
    if "--lane-m" in args:
        i = args.index("--lane-m")
        lane_m = float(args[i + 1])
        del args[i:i + 2]
    if len(args) != 1:
        print("usage: python -m mav.tools.calibrate_scale <video> [--lane-m 0.50]")
        return 2
    return calibrate(args[0], lane_m=lane_m)


if __name__ == "__main__":
    raise SystemExit(main())

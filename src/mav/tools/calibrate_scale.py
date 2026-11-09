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
    """(lane width in pixels AT the lookahead row, confidence), or None.

    The first version of this function was wrong twice over, and both mistakes
    are worth recording because both produced plausible-looking numbers.

    Mistake 1: it read `band_points` and took `max(col) - min(col)`, believing
    those were the two edge lines. They are not - `band_points` holds ONE
    centroid per horizontal band, and with both edge lines visible that centroid
    sits in the MIDDLE of the lane. On a straight lane all the columns are
    identical (max-min = 0); on a curve it measures how far the lane centre
    drifts across the frame. The real clip reported a 43.8 px "lane" on a 1280 px
    frame because of this.

    Mistake 2: the replacement used the ROI-median width, but `px_per_m` is
    defined at the LOOKAHEAD ROW, and under perspective the two differ a lot.
    The ROI median also swings frame to frame as each frame sees a different mix
    of near (wide) and far (narrow) rows - that spread is geometry, not noise,
    so averaging it away produces a number that describes no row in particular.

    So: the width must be measured at the lookahead row, by the detector, from
    bands that actually resolve two clusters. On the user's robot clip that
    constraint is what exposed the real problem - every two-cluster observation
    was on one row at the bottom of the frame, never at the lookahead row - and
    the honest answer there is "cannot measure yet", not a number.
    """
    meas, dbg = detector.detect_with_debug(frame)
    if meas is None:
        return None

    by_row = dbg.get("lane_width_by_row") or []
    if not by_row:
        # Older/stub debug dict: fall back to the ROI median. Coarser, same
        # quantity, better than refusing outright.
        width = dbg.get("lane_width_px")
        if width is None or not np.isfinite(width) or width <= 1.0:
            return None
        return float(width), float(meas.confidence)

    h = int(np.asarray(frame).shape[0])
    from mav.perception.lane import lookahead_row

    y_look = lookahead_row(detector.cfg, h)
    y0 = int(detector.cfg.roi_top * h)
    near = [(y + y0, w) for y, w in by_row if abs((y + y0) - y_look) <= 16.0]
    if not near:
        return None
    width = float(np.median([w for _, w in near]))
    if not np.isfinite(width) or width <= 1.0:
        return None
    return width, float(meas.confidence)


def _rel_spread(values) -> float:
    """p75/p25 of a sample, or inf when it is too small to say anything."""
    arr = np.asarray(list(values), dtype=float)
    if arr.size < 4:
        return float("inf")
    lo = float(np.percentile(arr, 25))
    hi = float(np.percentile(arr, 75))
    return hi / max(lo, 1e-9)


def _largest_window(vals: np.ndarray, tol: float) -> tuple[int, int]:
    """[start, end) of the longest run of sorted `vals` inside a relative tol."""
    if vals.size == 0:
        return 0, 0
    best_s, best_e = 0, 0
    j = 0
    for i in range(vals.size):
        j = max(j, i)
        limit = vals[i] * (1.0 + tol)
        while j < vals.size and vals[j] <= limit:
            j += 1
        if j - i > best_e - best_s:
            best_s, best_e = i, j
    return best_s, best_e


def _dominant_group(
    spans: np.ndarray, tol: float = 0.15
) -> tuple[float, float, np.ndarray, int]:
    """(median of the largest consistent group, its share, the group, runner-up).

    The failures this tool must survive are one-sided: a split edge line or a
    dash merging into an edge makes the lane look NARROWER, never wider. So the
    median of all frames lands between the true width and the broken ones, and
    p25/p75 reports "the lane is changing size" when actually a minority of
    frames are broken. The question actually worth asking is whether there is a
    width that MOST of this clip agrees on - so take the largest group of
    mutually consistent readings, and require that it both hold a majority and
    clearly beat the next group. Two equally popular answers means the clip does
    not contain one lane, and no number is better than a coin flip.

    Groups are formed by scanning the sorted values for the longest run inside a
    stated relative tolerance: deterministic, no clustering library.
    """
    vals = np.sort(np.asarray(spans, dtype=float))
    if vals.size == 0:
        return 0.0, 0.0, vals, 0
    bs, be = _largest_window(vals, tol)
    group = vals[bs:be]
    rest = np.concatenate([vals[:bs], vals[be:]])
    rs, re_ = _largest_window(rest, tol)
    runner_up = re_ - rs if rest.size else 0
    return float(np.median(group)), float(group.size / vals.size), group, runner_up


def _explain_where_bands_were(detector: LaneDetector, video_path: str) -> None:
    """Say WHERE the two-cluster bands appeared, not just that they were missing.

    "Detection is unreliable" is useless advice. The actionable fact is which
    rows actually resolve both edge lines: if they are all at the bottom of the
    frame, the lookahead row is seeing only one edge and no scale can be derived
    from it - the fix is to move the lookahead (or the camera), not to retune
    the threshold.

    Reads the clip ONCE (the first version opened it twice, which also made it
    immune to monkeypatching `open_video` in the tests) and remembers the frame
    height while it is already reading frames.
    """
    rows: dict[int, list[float]] = {}
    h: int | None = None
    try:
        cam = open_video(video_path)
    except RuntimeError:
        print(f"Could not reopen {video_path!r} to explain the failure.")
        return
    try:
        i = 0
        while True:
            frame = cam.read()
            if frame is None:
                break
            i += 1
            if h is None:
                h = int(np.asarray(frame).shape[0])
            if i % 2:  # every other frame is plenty for a distribution
                continue
            meas, dbg = detector.detect_with_debug(frame)
            if meas is None:
                continue
            y0 = int(detector.cfg.roi_top * h)
            for y, w in dbg.get("lane_width_by_row") or []:
                rows.setdefault(round(y + y0), []).append(float(w))
    finally:
        cam.close()

    if not rows:
        print("No band in this clip resolved two separate markings, so nothing")
        print("here can measure a lane width. Check the overlay to see whether")
        print("both edge lines appear at all:")
        print(f"  python -m mav.perception.lane {video_path} --overlay /tmp/ov.mp4")
        return

    from mav.perception.lane import lookahead_row

    y_look = round(lookahead_row(detector.cfg, h)) if h else None
    print(
        f"Two-cluster bands (both edge lines visible) appeared on "
        f"{len(rows)} row(s); the lookahead row is {y_look}:"
    )
    for r in sorted(rows):
        ws = np.array(rows[r])
        mark = "  <-- lookahead" if y_look is not None and abs(r - y_look) <= 16 else ""
        print(
            f"  row {r:4d}: {len(ws):5d} observations, median width "
            f"{float(np.median(ws)):6.1f} px{mark}"
        )
    print()
    if y_look is not None and not any(abs(r - y_look) <= 16 for r in rows):
        print("No observation is near the lookahead row, so no scale can be")
        print("derived that applies where the robot actually measures. Options:")
        print("  1. Point the camera so both edge lines are visible where the")
        print("     robot looks (higher, or less pitch).")
        print("  2. Derive px_per_m at the rows you CAN see and convert with the")
        print("     perspective model - more work, and it needs the camera height.")
        print("Do NOT take the median of the visible rows: it describes a")
        print("different distance and every offset would inherit the error.")


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
    frame_w = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            frames += 1
            frame_w = int(np.asarray(frame).shape[1])
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
            "seeing BOTH edge lines at the lookahead row.\n"
        )
        _explain_where_bands_were(detector, video_path)
        return 2

    arr = np.array(spans, dtype=float)
    p25, p50, p75 = (float(np.percentile(arr, q)) for q in (25, 50, 75))
    print(f"video    : {video_path}")
    print(f"profile  : {name} (current px_per_m = {current.px_per_m:g})")
    print(f"frames   : {frames}, usable {len(spans)} ({100 * len(spans) / frames:.0f}%)")
    print(f"frame    : {frame_w} px wide")
    print(f"lane span: median {p50:.1f} px  (p25 {p25:.1f}, p75 {p75:.1f})  "
          f"assumed {lane_m:g} m wide")

    # A median is not good enough here, because the failures are NOT symmetric:
    # the usual one makes the lane look NARROWER than it is. Two clusters 90 px
    # apart on a real 500 px lane is one painted line split by a smudge, not a
    # 90 px lane - and a handful of those drag p25 down far enough to make the
    # spread look like the lane is changing size (measured: p25 95 px on a clip
    # whose real edges are ~510 px apart). So find the DOMINANT GROUP of
    # consistent readings and ask how much of the clip supports it.
    band_px, support, group, runner_up = _dominant_group(arr)
    print(f"           dominant group: {band_px:.1f} px  "
          f"({support:.0%} of usable frames), spread within it "
          f"{_rel_spread(group):.2f}x")

    # Refusing is the whole value of this tool: a plausible-looking number from
    # bad input would be pasted into the config and be indistinguishable from a
    # measurement. Two independent things must hold - the clip must have a
    # consistent reading (spread), and enough of the clip must agree (support).
    clear_winner = runner_up == 0 or group.size >= 2 * runner_up
    if _rel_spread(group) > 1.25 or support <= 0.50 or not clear_winner:
        print()
        print("REFUSING to report a value.")
        print()
        if support <= 0.50 or not clear_winner:
            print(f"Only {support:.0%} of the usable frames agree on a width, and the")
            print(f"next group has {runner_up} frames against this one's {group.size},")
            print("so there is no clear answer in this clip. Common causes, in")
            print("order of how often they hit this project:")
            print("  - the clip shows more than one surface (a white mat and a dark")
            print("    mat need opposite polarity, so the same pixel width means")
            print("    different things in each);")
            print("  - the camera moves, so px_per_m genuinely changes frame to frame;")
            print("  - at the lookahead row only ONE edge line is inside the frame, and")
            print("    the 'second edge' is a dash or the frame border.")
        else:
            print(f"The frames that agree still differ by {_rel_spread(group):.2f}x, which a")
            print("0.50 m lane cannot do. The bands are unreliable.")
        print()
        print("px_per_m belongs to the CAMERA MOUNT (height and pitch). A phone held")
        print("by hand is not the robot's mount, so even a clean phone clip is not")
        print("the number the robot needs. Record with the robot's own CSI camera on")
        print("a real lane, then re-run this.")
        print()
        print("To see what the detector saw:")
        print(f"  python -m mav.perception.lane {video_path} --overlay /tmp/ov.mp4")
        return 2

    span_px = band_px
    derived = span_px / lane_m
    print()
    print(f"  derived px_per_m = {derived:.0f}")
    print(f"  current          = {current.px_per_m:g}")
    print(f"  ratio            = {derived / current.px_per_m:.2f}x")
    print()
    # Resolution trap: px_per_m is PIXELS per metre, so the same wall is a
    # different number of pixels at a different frame width. The robot's CSI
    # pipeline delivers 640x360; a phone clips at 854x480. A value measured on a
    # clip of a different width is off by exactly that ratio.
    robot_w = 640
    if frame_w and frame_w != robot_w:
        print(f"  NOTE: this clip is {frame_w} px wide, the robot camera is {robot_w}.")
        print(f"  px_per_m is PIXELS per metre, so a different frame width needs a")
        print(f"  different value. Rescaled to {robot_w} px wide:")
        print(f"    px_per_m for {robot_w} px frames = {derived * robot_w / frame_w:.0f}")
        print(f"  Use THIS only if the clip came from the same camera at the same")
        print(f"  angle - a phone in a different position is a different answer, not")
        print(f"  a rescaling. Prefer recording with the robot camera.")
        print()
    print(f"Paste into configs/{name}.yaml, then re-run the detector to confirm")
    print("the detection rate goes UP (a correct scale fixes the smear gate, which")
    print("is what rejects bands on a mis-scaled image):")
    value = derived * robot_w / frame_w if frame_w else derived
    print(f"  lane: {{ px_per_m: {value:.0f} }}")
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

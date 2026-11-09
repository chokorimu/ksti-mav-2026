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
from mav.perception.lane import LaneConfig, LaneDetector, _LUM


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


def _runs_in_gray_row(row: np.ndarray,
                      min_width_px: int = 8,
                      max_width_px: int = 150) -> list[tuple[int, int]]:
    """Marking-shaped bright runs in ONE already-converted grayscale row."""
    row = np.asarray(row, dtype=np.float32)
    lo, hi = float(row.min()), float(row.max())
    if hi - lo < 30.0:
        return []  # a featureless row: nothing painted here, either way
    # Halfway up the row's own range, with an absolute floor so a nearly flat row
    # cannot manufacture contrast out of sensor noise. Deliberately NOT the median:
    # a row where markings cover more than half the width (a wide bright surface)
    # puts the median ON the markings and then nothing clears it.
    thr = max(lo + 0.5 * (hi - lo), lo + 35.0)
    b = (row > thr).astype(np.uint8)
    d = np.diff(np.concatenate([[0], b, [0]]))
    starts, ends = np.where(d == 1)[0], np.where(d == -1)[0]
    return [(int(s), int(e)) for s, e in zip(starts, ends)
            if min_width_px <= e - s <= max_width_px]


def _markings_at_row(frame: np.ndarray, y: int,
                     min_width_px: int = 8,
                     max_width_px: int = 150) -> list[tuple[int, int]]:
    """Bright runs on ONE image row: the raw evidence for a lane width there.

    Deliberately independent of the detector's mask. The mask is what produced
    the suspect number in the first place, so asking it to justify itself is
    circular. This looks at the pixels: how many things on this row actually
    look like a painted marking?

    `max_width_px` matters on a shiny floor. A specular glare patch is one wide
    run (often 200+ px), while a painted line or dash is 20-60 px, so capping
    the width is what stops glare being counted as an edge line.
    """
    g = np.asarray(frame)[..., :3].astype(np.float32) @ _LUM
    return _runs_in_gray_row(g[int(y)], min_width_px, max_width_px)


def _explain_lookahead_evidence(detector: LaneDetector, video_path: str) -> None:
    """Say WHY no lane width could be measured, instead of listing three maybes.

    The refusal message used to offer three causes and leave the operator to
    guess. On the robot's first real clip (10 Nov 2026) the answer was decidable
    in one pass: at the lookahead row there was never more than ONE marking in
    the whole clip, so no lane width existed there to measure. The detector had
    still reported two clusters - it was pairing a dash with a glare patch, which
    is why the spans scattered 84-563 px (6.7x) instead of settling.

    Counting markings on the raw row separates "the camera cannot see both edges
    where we look" (a mounting problem) from "the clip is inconsistent" (a
    recording problem). Those need opposite fixes.
    """
    from mav.perception.lane import lookahead_row

    try:
        cam = open_video(video_path)
    except RuntimeError:
        return
    counts: dict[int, int] = {}
    spans: list[int] = []
    frame_w = 0
    frame_h = 0
    y_look = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            arr = np.asarray(frame)
            frame_h, frame_w = int(arr.shape[0]), int(arr.shape[1])
            y_look = int(round(lookahead_row(detector.cfg, frame_h)))
            runs = _markings_at_row(arr, y_look)
            counts[len(runs)] = counts.get(len(runs), 0) + 1
            if len(runs) >= 2:
                spans.append(runs[-1][1] - runs[0][0])
    finally:
        cam.close()
    total = sum(counts.values())
    if not total:
        return
    print(f"Evidence at the lookahead row {y_look} "
          f"(frame {frame_w}x{frame_h}, ROI starts at row "
          f"{int(detector.cfg.roi_top * frame_h)}):")
    for n in sorted(counts):
        what = {0: "nothing painted", 1: "ONE marking (a dash, or one edge)",
                2: "two markings"}.get(n, f"{n} markings")
        print(f"  {counts[n]:5d} frames ({counts[n] / total:4.0%}): {what}")
    print()
    one_or_fewer = counts.get(0, 0) + counts.get(1, 0)
    if one_or_fewer > 0.5 * total:
        print("So at the row where the robot measures, the frame does not contain")
        print("both lane edges - it cannot, because they are outside the picture.")
        print("The lane is WIDER than the frame that close to the car, so any")
        print("'lane width' the detector reports there is a dash paired with a")
        print("glare patch, not the lane. Nothing is wrong with the recording.")
        print()
        print("Fix the GEOMETRY, not the clip:")
        print("  - aim the camera further ahead (less pitch, or mounted higher),")
        print("    so the lookahead row lands where the whole lane fits;")
        print("  - or move the measurement point up the frame (a smaller")
        print("    `lane.lookahead_frac`, and `roi_top` to match).")
        print("Then record again. Do not paste a number derived from a row that")
        print("cannot see the lane.")
        return
    if spans:
        arr = np.array(spans, dtype=float)
        print(f"Two markings WERE found on {len(spans)} frames, spanning "
              f"{arr.min():.0f}-{arr.max():.0f} px "
              f"(p25 {np.percentile(arr, 25):.0f}, p75 {np.percentile(arr, 75):.0f}).")
        if float(np.percentile(arr, 75)) / max(1.0, float(np.percentile(arr, 25))) > 2.0:
            print("That range is too wide to be one lane: the pairing of which two")
            print("markings are being measured is changing from frame to frame.")
            print("A dashed centre line next to one edge line looks exactly like")
            print("this - the 'width' is then dash-to-edge, not edge-to-edge.")


def _scan_rows_for_a_usable_lane(detector: LaneDetector, video_path: str) -> None:
    """Which image rows can actually see BOTH lane edges, and how steadily.

    This is the answer to "why can I not calibrate": not a list of possible
    causes, but a table of where the lane is measurable and where it is not. The
    robot measures at ONE row (`lookahead_row`); if the lane does not fit in the
    frame there, `px_per_m` cannot be derived at all, and the fix is to move the
    camera or the row - not to average harder.

    Reported per row: how many frames show exactly two markings, the median
    distance between them, and p75/p25 of that distance. A stable lane gives a
    spread near 1.0; a dash paired with an edge line swings with the dash gaps.
    """
    from mav.perception.lane import lookahead_row

    try:
        cam = open_video(video_path)
    except RuntimeError:
        return
    per_row: dict[int, list[float]] = {}
    per_row_clipped: dict[int, int] = {}
    frame_h = frame_w = 0
    frames = 0
    step = 10
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            arr = np.asarray(frame)
            frame_h, frame_w = int(arr.shape[0]), int(arr.shape[1])
            g = arr[..., :3].astype(np.float32) @ _LUM
            frames += 1
            for y in range(step // 2, frame_h, step):
                runs = _runs_in_gray_row(g[y])
                if len(runs) == 2:
                    per_row.setdefault(y, []).append(float(runs[1][1] - runs[0][0]))
                    # A marking touching x=0 or x=w-1 is CUT OFF by the frame, so
                    # the distance to it is not the distance to its far edge. This
                    # is the direct evidence that the lane is wider than the
                    # picture, which beats inferring it from a scattered width.
                    if runs[0][0] <= 1 or runs[-1][1] >= frame_w - 1:
                        per_row_clipped[y] = per_row_clipped.get(y, 0) + 1
    finally:
        cam.close()
    if not frames:
        return
    y_look = int(round(lookahead_row(detector.cfg, frame_h)))
    print(f"Row scan ({frames} frames, {frame_w}x{frame_h}, "
          f"lookahead row is {y_look}):")
    usable: list[tuple[int, float, int]] = []
    seen_dashed: list[int] = []
    for y in sorted(per_row):
        spans = np.array(per_row[y], dtype=float)
        presence = spans.size / frames
        if spans.size < 0.4 * frames:
            continue
        p25, p50, p75 = (float(np.percentile(spans, q)) for q in (25, 50, 75))
        spread = p75 / max(1.0, p25)
        clipped = per_row_clipped.get(y, 0)
        cut_off = clipped > 0.25 * spans.size
        note = ""
        # A lane EDGE is painted along the whole track, so it is in nearly every
        # frame. A dashed centre line is not. That is the discriminator: on the
        # robot's first clip the "stable" row was present in only 42% of frames,
        # which means the leftmost marking was a DASH - so the distance measured
        # was dash-to-edge, and 331 px would have implied a plausible, wrong
        # px_per_m of 662. Stability alone is not enough; presence is what tells
        # an edge line from a dash.
        if cut_off:
            note = (f"  in {clipped} of those {spans.size} frames a marking TOUCHES "
                    f"the frame border, so the pair is cut off and this span is a "
                    f"lower bound - the lane is wider than the picture here")
        elif spread <= 1.25 and presence >= 0.80:
            usable.append((y, p50, spans.size))
            note = "  <-- stable and always present: a lane width can be measured here"
        elif spread <= 1.25:
            seen_dashed.append(y)
            note = (f"  present in only {presence:.0%} of frames: the marking "
                    f"comes and goes, so it is a DASH, not an edge")
        print(f"  row {y:4d}: {spans.size:4d}/{frames} frames show two markings, "
              f"median span {p50:6.1f} px, spread {spread:.2f}x{note}")
    print()
    if not usable:
        if seen_dashed:
            print("The only rows with a consistent distance measure a DASH against an")
            print("edge line, which is not the lane width - a dashed marking is")
            print("absent from roughly half the frames, while a lane edge is painted")
            print("continuously. Taking that number would give a plausible, wrong")
            print("px_per_m, so this tool will not report it.")
        else:
            print("NO row in this clip shows two markings with a stable distance, so")
            print("the lane never fits in the frame at any height.")
        print("That is a camera MOUNTING problem: raise the camera, or pitch it")
        print("further ahead, so both edges are inside the picture where the robot")
        print("measures.")
        return
    best_y, best_span, best_n = max(usable, key=lambda t: t[2])
    if abs(best_y - y_look) > 16:
        y0 = int(detector.cfg.roi_top * frame_h)
        span = max(1, frame_h - 1 - y0)
        frac = (best_y - y0) / span
        print(f"The robot measures at row {y_look}, but the lane is only measurable")
        print(f"around row {best_y} (span {best_span:.0f} px, {best_n}/{frames} frames).")
        if 0.0 <= frac <= 1.0:
            print(f"Set `lane.lookahead_frac: {frac:.2f}` so the two rows coincide,")
            print("then record again and re-run this tool.")
        else:
            print(f"That row is OUTSIDE the detector's ROI (rows {y0}-{frame_h - 1}),")
            print("so no setting of `lookahead_frac` can reach it. Raise the camera")
            print("or pitch it further ahead until both edges appear where the")
            print("detector already looks.")
    else:
        print(f"Row {best_y} is usable and is where the robot already measures:")
        print(f"  span {best_span:.0f} px / 0.50 m -> px_per_m = {best_span / 0.5:.0f}")
        print("  If that looks wrong, check the lane really is 0.50 m wide where")
        print("  the robot measures it (rules §6), not just where it is painted.")


def _camera_motion(video_path: str) -> tuple[float, float, float] | None:
    """(median px/frame, share of frames >5 px, net drift) or None.

    `px_per_m` is a property of the camera MOUNT - its height and pitch. If the
    camera moves while recording, the quantity being measured changes from frame
    to frame, and no single value can describe the clip. This is not a subtle
    effect: pushing the robot along the lane (which is the right way to record
    DETECTOR test material) swings the mount, and that invalidates the clip for
    a SCALE calibration.

    Measured with OpenCV's phase correlation, which is what actually detects it:
    a first attempt using a numpy row-brightness profile reported a median of
    0.0 px/frame on a clip that phase correlation shows moving 6 px/frame. The
    profile fingerprint is too insensitive because markings sit at similar rows
    even as the whole scene slides.
    """
    import cv2

    try:
        cam = open_video(video_path)
    except RuntimeError:
        return None
    prev = None
    shifts: list[float] = []
    dys: list[float] = []
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            g = cv2.cvtColor(np.asarray(frame), cv2.COLOR_BGR2GRAY).astype(np.float32)
            if prev is not None:
                win = cv2.createHanningWindow((g.shape[1], g.shape[0]), cv2.CV_32F)
                (dx, dy), _resp = cv2.phaseCorrelate(prev, g, win)
                shifts.append(float(np.hypot(dx, dy)))
                dys.append(float(dy))
            prev = g
    finally:
        cam.close()
    if len(shifts) < 4:
        return None
    arr = np.array(shifts, dtype=float)
    return (float(np.median(arr)), float((arr > 5.0).mean()), float(np.sum(dys)))


def _explain_camera_motion(video_path: str) -> bool:
    """Print the motion finding; True when the clip is too unstable to calibrate."""
    got = _camera_motion(video_path)
    if got is None:
        return False
    med, share, net = got
    if share < 0.30:
        return False
    print(f"The CAMERA MOVED during this clip: median {med:.1f} px/frame, "
          f"{share:.0%} of frames move more than 5 px, net drift {net:+.0f} px.")
    print()
    print("`px_per_m` is a property of the camera MOUNT (height and pitch), so it")
    print("only has a value while the mount is still. Pushing the robot along the")
    print("lane is the right way to record DETECTOR test material, but it swings")
    print("the camera, which changes the very quantity being measured here.")
    print()
    print("Record the scale clip with the robot STANDING STILL on the lane:")
    print("  - place it where the markings are representative,")
    print("  - do not touch or push it,")
    print("  - a few seconds is enough (5-10 frames).")
    print()
    return True


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
        _explain_camera_motion(video_path)
        print()
        _explain_lookahead_evidence(detector, video_path)
        print()
        _scan_rows_for_a_usable_lane(detector, video_path)
        print()
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
        # Both causes are reported, not just the first: they are independent
        # (the mount can be moving AND the lane can be too wide for the frame),
        # and fixing one while the other remains produces another refused clip.
        _explain_camera_motion(video_path)
        print()
        _explain_lookahead_evidence(detector, video_path)
        print()
        _scan_rows_for_a_usable_lane(detector, video_path)
        print()
        if support <= 0.50 or not clear_winner:
            print(f"Only {support:.0%} of the usable frames agree on a width, and the")
            print(f"next group has {runner_up} frames against this one's {group.size},")
            print("so there is no clear answer in this clip.")
        else:
            print(f"The frames that agree still differ by {_rel_spread(group):.2f}x, which a")
            print("0.50 m lane cannot do. The bands are unreliable.")
        print()
        print("px_per_m belongs to the CAMERA MOUNT (height and pitch), so it must")
        print("come from the robot's own CSI camera on a real lane.")
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
    # different number of pixels at a different frame width. A phone clip at
    # 854x480 gives a different answer than the robot camera at 640x360.
    #
    # Read from the pipeline rather than hardcoding 640: this is the number the
    # value will be applied at, so it must follow the pipeline if it changes.
    # (Confirmed on the robot 10 Nov 2026: csi_pipeline outputs 640x360, and the
    # runner prints the same size at startup.)
    from mav.io.camera import csi_pipeline

    robot_w = int(csi_pipeline().split("width=")[-1].split(",")[0])
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

"""Classical lane detection, v1. Numpy-only on purpose: no cv2 import here, so the
perception core runs anywhere and stays testable.

Pipeline per frame:
  1. grayscale (BGR luminance)
  2. region of interest (lower part of the frame, where the track is)
  3. background = large-window box mean (thin lines barely move it), then
     residual = gray - background, and a robust noise sigma from the residual's
     median absolute deviation. Markings are residual > +k*sigma (light) or
     < -k*sigma (dark), so white-on-dark and black-on-light both work and the
     floor's own noise stays out of the mask.
  4. split the ROI into horizontal bands; the marking-pixel centroid of each band
     approximates the lane centre there (the two edge lines are symmetric around it,
     and the dashed centre line reinforces it). A band whose marking is smeared
     across the image is dropped: see `max_band_span_m`. Where a band's marked
     columns split into separate clusters (gap > `cluster_gap_m`), the reported
     centre is the midpoint of the OUTERMOST clusters: the lane centre even when
     one edge line is thicker than the other. A single-cluster band is one edge
     line on its own and cannot locate the centre, so the fit prefers bands with
     two or more clusters and falls back to all bands only when too few survive.
  5. weighted quadratic fit x = a*y^2 + b*y + c over the band centroids, with
     iterative outlier rejection (`_robust_fit`)
  6. report offset at a lookahead row, curvature, and a confidence score

Known limitations (v1, deliberate):
  - `px_per_m` is a single constant: with a real (perspective) camera the scale
    varies with distance, so offset/curvature are approximate until the camera is
    calibrated. Signs and rough magnitudes are reliable; treat the numbers as
    uncalibrated until measured on the real robot.
  - The quadratic assumes the lane keeps one curvature across the whole ROI, so
    the ROI is deliberately shallow (`roi_top` 0.65 -> 0.96 m of forward distance
    at 130 px/m). On the measured KSTI track, whose tightest centreline radius is
    0.46 m, a deeper window wraps around the corner and no low-order polynomial
    fits it. Measured there: at `roi_top` 0.45 (a 1.52 m window) the fit residual
    is 25-46 px and confidence sits at 0.31-0.40, right on `min_confidence`; at
    0.65 the residual is 3-11 px and confidence 0.37-0.91.
  - When only ONE edge line is visible the centre cannot be measured: those
    single-cluster bands are excluded from the fit when better bands exist, and
    the all-single-cluster fallback is penalised (confidence halved), so the
    policy stops instead of steering half a lane width off. Measured on a phone
    recording of a practice track: without the cluster rule the offset saturates
    at +-2.5-3 m whenever one edge dominates the ROI.
  - `min_contrast` exists because of that same recording: a messaging-app codec
    puts +-10-20 gray levels of noise on a smooth floor, the robust sigma then
    bottoms out at its 1.0 floor, and a k*sigma-only threshold masks the noise
    instead of the markings (0% detection measured). The absolute floor keeps
    the mask on real contrast; clean sensor noise never reaches it. Broad
    shadows can still enter the dark mask and pull a centroid; RQ1 (lighting
    robustness) is still open.
  - Checkpoints / start-finish lines are NOT detected here (see
    `mav.perception.checkpoint`). They do not break centring: they cross the
    lane symmetrically.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

# BGR luminance weights (frames are BGR, see mav.io.camera).
_LUM = np.array([0.114, 0.587, 0.299], dtype=np.float32)


@dataclass(frozen=True)
class LaneConfig:
    polarity: str = "auto"          # "auto" | "light" | "dark" markings
    roi_top: float = 0.65           # search starts this far down (0.0 top, 1.0 bottom)
    bg_ksize: int = 75              # window for the background estimate (odd, pixels)
    thresh_k: float = 3.5           # marking threshold at +- k * robust sigma
    n_bands: int = 12               # horizontal slices from roi_top to the bottom
    min_band_pixels: int = 30       # ignore bands with fewer marking pixels
    min_points: int = 4             # bands needed before a fit is attempted
    px_per_m: float = 130.0         # UNCALIBRATED: pixels per metre at the lookahead row
    bottom_row_m: float = 0.3       # UNCALIBRATED: forward distance of the frame's bottom row
    lookahead_frac: float = 0.5     # offset is evaluated this far down the ROI
    min_confidence: float = 0.3
    min_contrast: float = 20.0       # absolute gray-level floor under the marking threshold
    cluster_gap_m: float = 0.15      # column gap that splits one band into separate markings
    border_margin_px: int = 2       # markings this close to the frame edge are truncated, not edges
    min_cluster_pixels: float = 3.0  # absolute floor on a cluster's mass (codec noise is 1-2 px)
    max_band_span_m: float = 0.8    # if a band's marking spans more than this, it is dropped
    mask_frac_band: tuple[float, float] = (0.01, 0.25)  # sane marking coverage (auto mode)


@dataclass(frozen=True)
class LaneMeasurement:
    offset_m: float          # lane centre X at the lookahead row, +right, -left
    lookahead_m: float       # forward distance of that row (approx, from px_per_m)
    curvature_inv_m: float   # +left, -right (curvature = 1 / turning radius)
    confidence: float        # 0..1
    n_points: int            # bands used in the fit


def _box_sum(img: np.ndarray, k: int) -> np.ndarray:
    """Sum over a k x k window (edge-padded), via integral image."""
    if k <= 1:
        return img.astype(np.float64)
    pad = k // 2
    p = np.pad(img, ((pad, pad + 1), (pad, pad + 1)), mode="edge")
    S = np.zeros((p.shape[0] + 1, p.shape[1] + 1), dtype=np.float64)
    S[1:, 1:] = np.cumsum(np.cumsum(p, axis=0), axis=1)
    h, w = img.shape
    return S[k:k + h, k:k + w] - S[0:h, k:k + w] - S[k:k + h, 0:w] + S[0:h, 0:w]


def _background(roi: np.ndarray, k: int) -> np.ndarray:
    """Local background level (large-window box mean; thin lines barely move it)."""
    return _box_sum(roi, k) / float(k * k)


def _noise_sigma(roi: np.ndarray) -> float:
    """Robust pixel-noise estimate from a 3x3 high-pass.

    A median over the whole ROI stays at the noise level even when thick
    markings or their background halos cover a large part of the frame,
    which a plain MAD of the residual does not.
    """
    local3 = _box_sum(roi, 3) / 9.0
    return max(1.4826 * float(np.median(np.abs(roi - local3))), 1.0)


def _structure_score_per_band(mask: np.ndarray, n_bands: int = 12) -> float:
    """Mean over horizontal bands of the fraction of the band's marking mass
    held by its busiest 10% of columns.

    A lane concentrates each band's mass in a handful of columns (two edge
    lines, plus dashes); sensor noise or a broad shadow spreads it over the
    whole width. Measured PER BAND, not over the whole ROI: a curving lane
    legitimately sweeps across columns as it runs down the frame (measured on
    the KSTI track: whole-ROI score 0.45 on a clean curve, 0.9+ on a
    straight), so a whole-ROI score punishes exactly the curves the robot must
    drive. Per band the score stays high on curves and low on noise.
    """
    h, w = mask.shape
    edges = np.linspace(0, h, n_bands + 1).astype(int)
    k = max(1, w // 10)
    scores: list[float] = []
    for i in range(n_bands):
        colsum = mask[edges[i]:edges[i + 1]].sum(axis=0).astype(np.float64)
        total = colsum.sum()
        if total <= 0:
            continue  # band without markings says nothing about structure
        scores.append(float(np.sort(colsum)[-k:].sum() / total))
    if not scores:
        return 0.0
    return float(np.mean(scores))


def _structure_score(mask: np.ndarray) -> float:
    """Legacy whole-ROI score (kept for comparison; not used by the fit)."""
    colsum = mask.sum(axis=0).astype(np.float64)
    total = colsum.sum()
    if total <= 0:
        return 0.0
    k = max(1, colsum.size // 10)
    return float(np.sort(colsum)[-k:].sum() / total)


def lookahead_row(cfg: LaneConfig, frame_height: int) -> float:
    """Row (in full-frame coordinates) where offset/curvature are evaluated."""
    y0 = int(cfg.roi_top * frame_height)
    return y0 + cfg.lookahead_frac * (frame_height - 1 - y0)


def _band_points(
    mask: np.ndarray,
    n_bands: int,
    min_px: int,
    max_span_px: float | None = None,
    dropped: dict[str, int] | None = None,
    max_gap_px: float | None = None,
) -> list[tuple[float, float, float]]:
    """(row, column, pixel-count) of the marking centroid per horizontal band.

    `max_span_px` drops bands whose marking is smeared across the image. Where
    the track is tight the lane runs ALONG a band instead of across it: the
    marked columns then spread over 150-280 px instead of the ~60 px a 0.5 m
    lane occupies, and the centroid of that smear is meaningless (errors up to
    85 px measured on the KSTI track). The span is measured between the 5th and
    95th percentile of the marked columns, so a few stray noise pixels cannot
    fake a wide band and get a good one thrown away.

    `dropped`, if given, is filled with why bands were left out: `thin` (too
    few marking pixels) and `smeared` (span over the limit). The two are
    different failures and the dashboard shows them separately.

    `max_gap_px`, if given, splits each band's marked columns into clusters
    separated by wider gaps; a band with two or more clusters then reports the
    midpoint of the outermost clusters instead of the plain centroid. See
    `_band_points_with_clusters`, which also returns the cluster count.
    """
    return [
        (y, x, n)
        for y, x, n, _ in _band_points_with_clusters(
            mask, n_bands, min_px, max_span_px, dropped, max_gap_px
        )
    ]


def _band_points_with_clusters(
    mask: np.ndarray,
    n_bands: int,
    min_px: int,
    max_span_px: float | None = None,
    dropped: dict[str, int] | None = None,
    max_gap_px: float | None = None,
    width_out: dict | None = None,
    border_margin_px: int = 0,
    min_cluster_px: float = 2.0,
) -> list[tuple[float, float, float, int]]:
    """`_band_points` plus a cluster count: (row, column, pixels, n_clusters).

    A band whose marked columns split into two or more clusters shows separate
    markings - the two edge lines, with or without the dashes between them -
    and the midpoint of the outermost clusters is the lane centre there, no
    matter how unevenly thick the individual markings are. A single cluster is
    one edge line on its own: its centroid sits ON that line, up to half a lane
    width off centre (measured on a phone recording: offsets saturating at
    +-2.5-3 m), so `_fit_debug` trusts such a band only when nothing better
    exists. Set `width_out["lane_width_px"]` to receive the frame's measured
    lane width in pixels, which the smear gate computes anyway.
    """
    h, w = mask.shape
    edges = np.linspace(0, h, n_bands + 1).astype(int)
    xs = np.arange(w, dtype=np.float64)
    clustered = max_gap_px is not None

    # Pass 1: per-band significant clusters. A cluster must carry real marking
    # mass: stray single pixels (sensor or codec noise) form their own
    # clusters, and as the OUTERMOST ones they would drag the midpoint off the
    # lane - on the sim's clean floor a 1-px cluster at column 145 moved a band
    # centre from 320 to 248. 5% of the band's marking pixels keeps every real
    # line (a thin edge next to a thick one is ~10% of the band) and drops the
    # noise.
    bands: list[tuple[int, np.ndarray, float, list[tuple[float, float, float]]]] = []
    n_thin = 0
    for i in range(n_bands):
        colsum = mask[edges[i]:edges[i + 1]].sum(axis=0)
        total = float(colsum.sum())
        if total < min_px:
            n_thin += 1
            continue
        sig: list[tuple[float, float, float]] = []
        if clustered:
            cols = np.nonzero(colsum)[0]
            if cols.size:
                splits = np.where(np.diff(cols) > max_gap_px)[0]
                groups = [g for g in np.split(cols, splits + 1) if g.size]
                min_cluster_px = max(min_cluster_px, 0.05 * total)
                # A cluster running into the frame edge is TRUNCATED: its outer
                # boundary is where the sensor stopped, not where the marking
                # ends, so it cannot be used as a lane edge. Without this, codec
                # noise on the sensor border (measured: 2 dark pixels at column
                # 0 and 1 at column 723 on a phone clip) reads as "two edge
                # lines" and reports the lane as 723 px wide - the full frame.
                sig = [
                    (float(g[0]), float(g[-1]), float(colsum[g].sum()))
                    for g in groups
                    if float(colsum[g].sum()) >= min_cluster_px
                    and g[0] >= border_margin_px
                    and g[-1] <= w - 1 - border_margin_px
                ]
        bands.append((i, colsum, total, sig))

    # The lane width as measured in THIS frame: the median outer width of the
    # bands that show two or more clusters. That is scale-invariant, so the
    # smear decision no longer depends on a calibrated px_per_m - a phone
    # camera at ~840 px/m and the robot at 130 px/m get the same answer. The
    # old fixed `max_span_px` gate (0.8 m at the fallback 130 px/m = 104 px)
    # threw away 8 of 12 good bands on the phone recording, whose lane is
    # ~420 px wide, and cut detection from 88% to 15%.
    multi_widths = [b[3][-1][1] - b[3][0][0] for b in bands if len(b[3]) >= 2]
    if multi_widths:
        lane_width_px: float | None = float(np.median(multi_widths))
    else:
        lane_width_px = max_span_px
    if width_out is not None:
        # Exposed for the scale calibrator: the lane width in pixels IS the
        # ruler for px_per_m, and the smear gate already measures it. Computing
        # it a second time elsewhere would let the two drift apart.
        #
        # `lane_width_px` is the median across ALL bands, which is what the smear
        # gate wants (one robust number for the whole ROI). It is NOT the width
        # at the lookahead row, and those differ under perspective: on a
        # synthetic lane with taper=0.4 the ROI median is 158 px while the
        # lookahead row is ~130 px. `px_per_m` is defined AT the lookahead row,
        # so the calibrator must use the per-row value, reported below.
        width_out["lane_width_px"] = lane_width_px
        width_out["n_width_samples"] = len(multi_widths)
        # `b[0]` is the band INDEX; using a leftover loop variable here made
        # every row report as the last band's row, which silenced the whole
        # per-row diagnostic and cost an hour chasing a nonexistent "all
        # observations land on one row" problem in the real clip.
        width_out["lane_width_by_row"] = [
            (float(0.5 * (edges[b[0]] + edges[b[0] + 1])),
             float(b[3][-1][1] - b[3][0][0]))
            for b in bands if len(b[3]) >= 2
        ]

    pts: list[tuple[float, float, float, int]] = []
    n_smeared = 0
    for i, colsum, total, sig in bands:
        if clustered:
            width = (sig[-1][1] - sig[0][0]) if sig else 0.0
            if lane_width_px is not None and width > 1.5 * lane_width_px:
                n_smeared += 1
                continue
            if len(sig) >= 2:
                n_clusters = len(sig)
                # Midpoint of the OUTER boundaries, not of the cluster
                # centres. Where a dash merges with an edge line into one
                # wide cluster, that cluster's centre is pulled off the line
                # (measured on the sim: column 276 for a line at 253, a 23 px
                # error), while its outer column is still the lane edge.
                x = 0.5 * (sig[0][0] + sig[-1][1])
            else:
                n_clusters = 1
                x = float((xs * colsum).sum() / total)
        else:
            # No cluster split requested: the legacy fixed span gate.
            cols = np.nonzero(colsum)[0]
            if max_span_px is not None and cols.size:
                cum = np.cumsum(colsum[cols])
                lo = int(np.searchsorted(cum, 0.05 * total, side="left"))
                hi = min(int(np.searchsorted(cum, 0.95 * total, side="left")), cols.size - 1)
                if float(cols[hi] - cols[lo]) > max_span_px:
                    n_smeared += 1
                    continue
            n_clusters = 1
            x = float((xs * colsum).sum() / total)
        y = 0.5 * (edges[i] + edges[i + 1])
        pts.append((y, x, total, n_clusters))
    if dropped is not None:
        dropped["thin"] = n_thin
        dropped["smeared"] = n_smeared
    return pts


def _fit_quadratic(pts: list[tuple[float, float, float]]) -> tuple[float, float, float, float]:
    """Weighted least squares x = a*y^2 + b*y + c. Returns (a, b, c, weighted rms px)."""
    ys = np.array([p[0] for p in pts])
    xs = np.array([p[1] for p in pts])
    ws = np.array([p[2] for p in pts])
    A = np.vstack([ys * ys, ys, np.ones_like(ys)]).T
    sw = np.sqrt(ws)
    coef, *_ = np.linalg.lstsq(A * sw[:, None], xs * sw, rcond=None)
    a, b, c = (float(v) for v in coef)
    resid = float(np.sqrt((sw * sw * (xs - A @ coef) ** 2).sum() / sw.sum()))
    return a, b, c, resid


def _robust_fit(
    pts: list[tuple[float, float, float]], min_points: int, iters: int = 3
) -> tuple[float, float, float, float]:
    """Weighted quadratic fit with iterative outlier rejection.

    A plain least-squares fit bends towards an outlying band instead of
    rejecting it: the polynomial spends its freedom accommodating the outlier
    and the good bands pay for it. Measured on a phone recording, one near-field
    edge line moved the lookahead offset by +1.5 m while its own residual stayed
    under 15 px - no per-point residual test can catch that, only a consensus
    can. Bands whose residual exceeds max(median + 3*MAD, 12 px) are dropped and
    the fit repeated; the loop stops when nothing changes or `min_points` bands
    are left, so a detector can never reject its way below a fit.
    """
    a = b = c = resid = 0.0
    for _ in range(iters):
        a, b, c, resid = _fit_quadratic(pts)
        rp = [abs(x - (a * y * y + b * y + c)) for y, x, _ in pts]
        med = float(np.median(rp))
        mad = 1.4826 * float(np.median(np.abs(np.array(rp) - med)))
        cut = max(med + 3.0 * mad, 12.0)
        keep = [p for p, v in zip(pts, rp) if v <= cut]
        if len(keep) == len(pts) or len(keep) < min_points:
            break
        pts = keep
    return a, b, c, resid


class LaneDetector:
    """Stateless per call: safe to reuse across frames, no history kept."""

    def __init__(self, cfg: LaneConfig | None = None) -> None:
        self.cfg = cfg or LaneConfig()

    def detect(self, frame: np.ndarray) -> LaneMeasurement | None:
        """LaneMeasurement or None. detect_with_debug() returns the same plus internals."""
        return self.detect_with_debug(frame)[0]

    def detect_with_debug(self, frame: np.ndarray) -> tuple[LaneMeasurement | None, dict]:
        """detect() plus a dict of the internals the dashboard overlay needs.

        The measurement is identical to detect(); only extra information comes
        back. The dict becomes JSON-safe through SimState.to_dict().
        """
        arr = np.asarray(frame)
        if arr.ndim != 3 or arr.shape[2] != 3 or arr.dtype != np.uint8:
            raise ValueError(f"expected an HxWx3 uint8 BGR frame, got {arr.dtype} {arr.shape}")

        cfg = self.cfg
        h, w = arr.shape[:2]
        gray = arr[..., :3].astype(np.float32) @ _LUM
        y0 = int(cfg.roi_top * h)
        debug: dict = {"h": h, "w": w, "roi_top": y0, "reason": None}
        if h - y0 < cfg.min_points * 4:
            debug["reason"] = "roi too small"
            return None, debug
        roi = gray[y0:]

        resid = roi - _background(roi, cfg.bg_ksize)
        sigma = _noise_sigma(roi)
        # k*sigma alone is not enough on coded footage: a messaging-app codec
        # leaves +-10-20 gray levels on a smooth floor, the robust sigma then
        # bottoms out at its 1.0 floor, and the mask fills with codec noise
        # (0% detection measured on a phone recording). The absolute floor
        # keeps the threshold on real contrast; clean sensor noise never
        # reaches it, so the sim and the Jetson camera are unaffected.
        thresh = max(cfg.thresh_k * sigma, cfg.min_contrast)
        masks = {
            "light": resid > thresh,
            "dark": resid < -thresh,
        }
        debug.update(
            sigma=sigma,
            threshold=thresh,
            light_frac=float(masks["light"].mean()),
            dark_frac=float(masks["dark"].mean()),
        )

        best: LaneMeasurement | None = None
        best_debug: dict = {}
        for name, mask in masks.items():
            if cfg.polarity != "auto":
                if name != cfg.polarity:
                    continue
            elif not (cfg.mask_frac_band[0] <= mask.mean() <= cfg.mask_frac_band[1]):
                continue  # implausible marking coverage for this polarity
            meas, mdbg = self._fit_debug(mask, name, y0, h, w)
            # Keep the debug of the strongest attempt even when it produced no
            # measurement: otherwise a failed fit reports "no candidate mask"
            # and hides why it failed (the dashboard then shows nothing).
            if best_debug.get("polarity") is None or (
                meas is not None and (best is None or meas.confidence > best.confidence)
            ):
                best_debug = mdbg
            if meas is not None and (best is None or meas.confidence > best.confidence):
                best, best_debug = meas, mdbg

        debug.update(best_debug)
        if best is None:
            debug["reason"] = "no fit" if best_debug.get("polarity") else "no candidate mask"
            return None, debug
        if best.confidence < cfg.min_confidence:
            debug["reason"] = "low confidence"
            return None, debug
        return best, debug

    def _fit_debug(
        self, mask: np.ndarray, polarity: str, y0: int, h: int, w: int
    ) -> tuple[LaneMeasurement | None, dict]:
        """Fit one polarity mask. Returns (measurement or None, debug dict)."""
        cfg = self.cfg
        max_span_px = None if cfg.max_band_span_m is None else cfg.max_band_span_m * cfg.px_per_m
        gap_px = None if cfg.cluster_gap_m is None else cfg.cluster_gap_m * cfg.px_per_m
        dropped: dict[str, int] = {}
        widths: dict = {}
        pts4 = _band_points_with_clusters(
            mask, cfg.n_bands, cfg.min_band_pixels, max_span_px, dropped, gap_px,
            width_out=widths,
            border_margin_px=cfg.border_margin_px,
            min_cluster_px=cfg.min_cluster_pixels,
        )
        debug: dict = {
            "polarity": polarity,
            "mask_pixels": int(mask.sum()),
            "n_bands_with_pixels": len(pts4),
            "n_bands_dropped_smeared": dropped.get("smeared", 0),
            "n_bands_dropped_thin": dropped.get("thin", 0),
            "band_points": [[p[0], p[1], p[2]] for p in pts4],
            # The lane width in pixels the smear gate already measured. This is
            # what turns the lane into a ruler for px_per_m.
            "lane_width_px": widths.get("lane_width_px"),
            "n_width_samples": widths.get("n_width_samples", 0),
            "lane_width_by_row": widths.get("lane_width_by_row") or [],
        }
        if len(pts4) < cfg.min_points:
            return None, debug

        # Prefer bands that show two or more clusters: those have both edge
        # lines in view, so their midpoint is the lane centre. A single-cluster
        # band is one edge line alone - its centroid sits ON that line, up to
        # half a lane width off centre (measured: offsets saturating at
        # +-2.5-3 m on a phone recording). Only when too few multi-cluster
        # bands survive do we fall back to all bands, and that fallback is
        # penalised below so the policy stops rather than steering blind.
        multi = [p for p in pts4 if p[3] >= 2]
        pts = [(y, x, n) for y, x, n, _ in (multi if len(multi) >= cfg.min_points else pts4)]
        fallback = len(multi) < cfg.min_points
        debug.update(
            n_multi_cluster_bands=len(multi),
            used_single_cluster_fallback=fallback,
        )
        a, b, c, resid = _robust_fit(pts, cfg.min_points)

        # The fit works in ROI-relative rows; convert the lookahead row to match.
        y_look_roi = lookahead_row(cfg, h) - y0
        x_look = a * y_look_roi * y_look_roi + b * y_look_roi + c
        debug.update(
            fit={"a": a, "b": b, "c": c},
            resid=resid,
            lookahead_row=lookahead_row(cfg, h),
            lookahead_x=x_look,
        )
        if not 0.0 <= x_look <= w:
            return None, debug  # fitted centre left the image: not a lane

        offset_m = (x_look - w / 2.0) / cfg.px_per_m
        curvature = -2.0 * a * cfg.px_per_m  # u(v) = U0 - (k/(2s))(v_near-v)^2 -> k = -2*a*s
        lookahead_m = cfg.bottom_row_m + (h - 1 - lookahead_row(cfg, h)) / cfg.px_per_m

        total_px = sum(p[2] for p in pts)
        f_pts = min(1.0, len(pts) / 6.0)
        f_res = 1.0 - min(1.0, resid / 25.0)
        f_mask = min(1.0, total_px / (cfg.min_band_pixels * cfg.n_bands * 0.5))
        c_struct = min(1.0, max(0.0, (_structure_score_per_band(mask) - 0.35) / 0.45))
        debug.update(
            f_pts=f_pts,
            f_res=f_res,
            f_mask=f_mask,
            c_struct=c_struct,
            structure_raw=_structure_score_per_band(mask),
        )
        confidence = f_pts * (0.4 + 0.6 * f_res) * f_mask * (0.15 + 0.85 * c_struct)
        if fallback:
            # No band showed both edge lines: the centre is a guess. Halve the
            # confidence so `min_confidence` rejects it unless the rest of the
            # evidence is strong, and the policy stops instead of steering at
            # half a lane width off.
            confidence *= 0.5

        return (
            LaneMeasurement(
                offset_m=offset_m,
                lookahead_m=lookahead_m,
                curvature_inv_m=curvature,
                confidence=confidence,
                n_points=len(pts),
            ),
            debug,
        )


def _main(argv: list[str]) -> int:
    """Offline runner: `python -m mav.perception.lane <video> [--overlay out.mp4]`.

    cv2 is imported here (lazily) so the module stays importable without it.
    """
    import cv2

    from mav.io.camera import open_video

    args = argv[1:]
    overlay_path = None
    if "--overlay" in args:
        i = args.index("--overlay")
        overlay_path = args[i + 1]
        del args[i:i + 2]
    # Validate a candidate px_per_m WITHOUT editing the config first. The
    # calibration tool says "paste it, then re-run the detector", which is the
    # wrong order: a wrong value has already been written to the file the robot
    # runs from, and reverting it is a separate step someone will forget. With
    # this flag the loop is: measure -> try -> look at the offsets -> paste.
    px_override = None
    if "--px-per-m" in args:
        i = args.index("--px-per-m")
        try:
            px_override = float(args[i + 1])
        except (IndexError, ValueError):
            print("--px-per-m needs a number, e.g. --px-per-m 597")
            return 2
        if px_override <= 0:
            print("--px-per-m must be > 0")
            return 2
        del args[i:i + 2]
    if len(args) != 1:
        print("usage: python -m mav.perception.lane <video> "
              "[--overlay out.mp4] [--px-per-m N]")
        return 2
    video_path = args[0]

    # Read the profile config, not the code defaults: the whole point of the
    # calibration loop is that `px_per_m`/`bottom_row_m` are measured values in
    # configs/<profile>.yaml. Without this the runner would ignore every edit
    # made during calibration and always report the old numbers.
    from dataclasses import replace as _replace

    from mav.config import load_lane_config, profile_name

    profile = profile_name()
    try:
        lane_cfg = load_lane_config(profile)
    except FileNotFoundError as e:
        print(f"note: {e}; using code defaults")
        lane_cfg = None
    if lane_cfg is not None and px_override is not None:
        lane_cfg = _replace(lane_cfg, px_per_m=px_override)
        print(f"px_per_m OVERRIDDEN to {px_override:g} for this run only "
              f"(configs/{profile}.yaml is unchanged)")
    if lane_cfg is not None:
        print(f"profile: {profile}  px_per_m={lane_cfg.px_per_m:g}  "
              f"bottom_row_m={lane_cfg.bottom_row_m:g}  roi_top={lane_cfg.roi_top:g}")
    detector = LaneDetector(lane_cfg)
    cam = open_video(video_path)
    writer = None
    latencies: list[float] = []
    n = detected = 0
    offsets: list[float] = []
    curvatures: list[float] = []
    try:
        while True:
            t0 = time.perf_counter()
            frame = cam.read()
            if frame is None:
                break
            meas = detector.detect(frame)
            latencies.append(time.perf_counter() - t0)
            n += 1
            if meas is not None:
                detected += 1
                offsets.append(meas.offset_m)
                curvatures.append(meas.curvature_inv_m)
                if overlay_path is not None:
                    fh, fw = frame.shape[:2]
                    if writer is None:
                        writer = cv2.VideoWriter(
                            overlay_path, cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (fw, fh)
                        )
                    u = int(fw / 2 + meas.offset_m * detector.cfg.px_per_m)
                    v = int(lookahead_row(detector.cfg, fh))
                    cv2.circle(frame, (u, v), 6, (0, 0, 255), 2)
                    cv2.putText(
                        frame,
                        f"off {meas.offset_m:+.2f}m  curv {meas.curvature_inv_m:+.2f}  "
                        f"conf {meas.confidence:.2f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                    )
                    writer.write(frame)
    finally:
        cam.close()
        if writer is not None:
            writer.release()

    if n == 0:
        print(f"no frames read from {video_path}")
        return 1
    arr = np.array(latencies) * 1000.0
    print(f"frames: {n}  detected: {detected} ({100.0 * detected / n:.0f}%)")
    if offsets:
        print(f"offset_m: min {min(offsets):+.2f}  max {max(offsets):+.2f}  "
              f"mean {sum(offsets) / len(offsets):+.2f}")
        # These two decide whether a px_per_m is usable, and neither is visible
        # from a min/max. A lane that fits in the arena has offsets within
        # max_offset_m (the policy declares the lane LOST beyond it) and a
        # curvature no tighter than the rules' minimum turning radius.
        lim = 0.35
        lost = sum(1 for o in offsets if abs(o) > lim)
        print(f"  over {lim:g} m (the policy calls the lane LOST): {lost}/{len(offsets)}"
              f" ({100.0 * lost / len(offsets):.0f}%)")
        if curvatures:
            k = max(abs(c) for c in curvatures)
            print(f"curvature_inv_m: |max| {k:.2f}  -> tightest radius "
                  f"{1.0 / k if k > 1e-6 else float('inf'):.1f} m"
                  f"  (rules allow 1.5 m)")
    print(f"latency ms/frame: mean {arr.mean():.2f}  p95 {np.percentile(arr, 95):.2f}  max {arr.max():.2f}")
    return 0


if __name__ == "__main__":
    import sys

    raise SystemExit(_main(sys.argv))

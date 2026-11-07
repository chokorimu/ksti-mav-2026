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
     and the dashed centre line reinforces it)
  5. weighted quadratic fit x = a*y^2 + b*y + c over the band centroids
  6. report offset at a lookahead row, curvature, and a confidence score

Known limitations (v1, deliberate):
  - `px_per_m` is a single constant: with a real (perspective) camera the scale
    varies with distance, so offset/curvature are approximate until the camera is
    calibrated. Signs and rough magnitudes are reliable; treat the numbers as
    uncalibrated until measured on the real robot.
  - Centroid-based: if only one edge line is visible the centre estimate sits
    between that edge and the dashed centre line. Broad shadows can enter the
    dark mask and pull the centroid; RQ1 (lighting robustness) is still open.
  - Checkpoints / start-finish lines are NOT detected here (later task). They do
    not break centring: they cross the lane symmetrically.
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
    roi_top: float = 0.45           # search starts at this fraction of frame height
    bg_ksize: int = 75              # window for the background estimate (odd, pixels)
    thresh_k: float = 3.5           # marking threshold at +- k * robust sigma
    n_bands: int = 12               # horizontal slices from roi_top to the bottom
    min_band_pixels: int = 30       # ignore bands with fewer marking pixels
    min_points: int = 4             # bands needed before a fit is attempted
    px_per_m: float = 130.0         # UNCALIBRATED: pixels per metre at the lookahead row
    bottom_row_m: float = 0.3       # UNCALIBRATED: forward distance of the frame's bottom row
    lookahead_frac: float = 0.5     # offset is evaluated this far down the ROI
    min_confidence: float = 0.3
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


def _structure_score(mask: np.ndarray) -> float:
    """Fraction of marking mass held by the busiest 10% of columns.

    A lane concentrates mass in a handful of columns (two edges + dashes);
    sensor noise or a broad shadow spreads it over the whole width.
    """
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


def _band_points(mask: np.ndarray, n_bands: int, min_px: int) -> list[tuple[float, float, float]]:
    """(row, column, pixel-count) of the marking centroid per horizontal band."""
    h, w = mask.shape
    edges = np.linspace(0, h, n_bands + 1).astype(int)
    xs = np.arange(w, dtype=np.float64)
    pts: list[tuple[float, float, float]] = []
    for i in range(n_bands):
        colsum = mask[edges[i]:edges[i + 1]].sum(axis=0)
        total = float(colsum.sum())
        if total < min_px:
            continue
        x = float((xs * colsum).sum() / total)
        y = 0.5 * (edges[i] + edges[i + 1])
        pts.append((y, x, total))
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
        masks = {
            "light": resid > cfg.thresh_k * sigma,
            "dark": resid < -cfg.thresh_k * sigma,
        }
        debug.update(
            sigma=sigma,
            threshold=cfg.thresh_k * sigma,
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
        pts = _band_points(mask, cfg.n_bands, cfg.min_band_pixels)
        debug: dict = {
            "polarity": polarity,
            "mask_pixels": int(mask.sum()),
            "n_bands_with_pixels": len(pts),
            "band_points": [[p[0], p[1], p[2]] for p in pts],
        }
        if len(pts) < cfg.min_points:
            return None, debug
        a, b, c, resid = _fit_quadratic(pts)

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
        c_struct = min(1.0, max(0.0, (_structure_score(mask) - 0.35) / 0.45))
        confidence = f_pts * (0.4 + 0.6 * f_res) * f_mask * (0.15 + 0.85 * c_struct)

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
    if len(args) != 1:
        print("usage: python -m mav.perception.lane <video> [--overlay out.mp4]")
        return 2
    video_path = args[0]

    detector = LaneDetector()
    cam = open_video(video_path)
    writer = None
    latencies: list[float] = []
    n = detected = 0
    offsets: list[float] = []
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
    print(f"latency ms/frame: mean {arr.mean():.2f}  p95 {np.percentile(arr, 95):.2f}  max {arr.max():.2f}")
    return 0


if __name__ == "__main__":
    import sys

    raise SystemExit(_main(sys.argv))

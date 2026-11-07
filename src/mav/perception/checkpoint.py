"""Cross-line detection: checkpoints and the start/finish line. Numpy-only,
no cv2 import here (same rule as mav.perception.lane: the perception core
runs anywhere and stays testable).

The lane detector is colour-blind on purpose: markings are white OR black
(rules §6), so luminance alone decides. Checkpoints and the start/finish
line are the opposite case. The guidebook says checkpoints are shown in
green and the start/finish line uses "a colour different from the track
markings, not necessarily red" [A11] - so this detector is colour-FIRST:

  1. HSV saturation: white / black / grey markings and most arena floors
     are unsaturated; a coloured line across the lane is not. That is what
     makes the detector independent of the actual marking colour.
  2. configurable hue bands (per [A11]: parameters, not hardcoded colours)
     decide checkpoint vs finish. The defaults follow the guidebook text
     and are documented as UNCONFIRMED - read the real colours off the
     arena on test day and put them in the config.
  3. geometry: the coloured pixels must form a thin horizontal band that
     spans the lane - a line ACROSS the track. A coloured patch of floor is
     too tall, a coloured post too narrow, and the edge lines (which run
     ALONG the track) never span it.

The output is a per-frame measurement, not an event. Counting crossings -
one event per physical line, and never counting a line the robot starts
sitting on - is CrossLineTracker's job, at the bottom of this module.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


@dataclass(frozen=True)
class CheckpointConfig:
    """Hue bands in degrees (0..360; a band may wrap, lo > hi). Empty = disabled.

    The defaults follow the guidebook text (checkpoints green) and are
    UNCONFIRMED [A11]: the finish colour is disabled entirely until someone
    reads it off the arena. Override from configs/ - never hardcode a colour
    in code.
    """

    checkpoint_hues: tuple[tuple[float, float], ...] = ((90.0, 150.0),)  # green
    finish_hues: tuple[tuple[float, float], ...] = ()                    # [A11] unknown
    min_saturation: float = 0.35    # white/black/grey markings stay below this
    min_value: float = 0.15         # near-black pixels are shadow, not colour
    min_span_frac: float = 0.06     # band must cover this much of the frame width
    target_span_frac: float = 0.15  # ...and this much is a full-confidence span
    max_band_frac: float = 0.30     # a line is thin; a coloured floor area is not
    min_band_pixels: int = 30       # coloured pixels per row for the row to count
    min_confidence: float = 0.35


@dataclass(frozen=True)
class CrossLineMeasurement:
    kind: str              # "checkpoint" | "finish"
    confidence: float      # 0..1
    row: float             # mean image row of the band (0 = top = far away)
    span_frac: float       # fraction of the frame width the band covers
    height_frac: float     # band height as a fraction of the frame height
    pixels: int            # coloured pixels inside the band


def _hsv(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(hue in degrees 0..360, saturation 0..1, value 0..1), vectorised."""
    b = frame[..., 0].astype(np.float32) / 255.0
    g = frame[..., 1].astype(np.float32) / 255.0
    r = frame[..., 2].astype(np.float32) / 255.0
    mx = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    d = mx - mn
    sat = np.where(mx > 0.0, d / np.where(mx > 0.0, mx, 1.0), 0.0).astype(np.float32)
    hue = np.zeros_like(mx)
    nz = d > 1e-6
    safe = np.where(nz, d, 1.0)
    is_r = nz & (mx == r)
    is_g = nz & (mx == g) & ~is_r
    is_b = nz & (mx == b) & ~is_r & ~is_g
    # Ties (r == g > b etc.) give the same hue from either branch, so the
    # priority order below is only there to keep the masks disjoint.
    hue = np.where(is_r, 60.0 * (((g - b) / safe) % 6.0), hue)
    hue = np.where(is_g, 60.0 * ((b - r) / safe + 2.0), hue)
    hue = np.where(is_b, 60.0 * ((r - g) / safe + 4.0), hue)
    return hue.astype(np.float32), sat, mx


def _hue_mask(hue: np.ndarray, ranges: tuple[tuple[float, float], ...]) -> np.ndarray:
    """True where the hue falls inside any band (bands may wrap around 360)."""
    m = np.zeros(hue.shape, dtype=bool)
    for lo, hi in ranges:
        if lo <= hi:
            m |= (hue >= lo) & (hue <= hi)
        else:  # wraps around 360 (e.g. red: 340..360 + 0..20)
            m |= (hue >= lo) | (hue <= hi)
    return m


def _runs(hot: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous [start, end) runs of True rows."""
    out: list[tuple[int, int]] = []
    start: int | None = None
    for y, v in enumerate(hot):
        if v and start is None:
            start = y
        elif not v and start is not None:
            out.append((start, y))
            start = None
    if start is not None:
        out.append((start, int(hot.size)))
    return out


def _band_span(mask: np.ndarray, y0: int, y1: int) -> tuple[int, int] | None:
    """(span in px between the 5th and 95th percentile column, pixel count)."""
    col_counts = mask[y0:y1].sum(axis=0)
    total = int(col_counts.sum())
    if total <= 0:
        return None
    cols = np.nonzero(col_counts)[0]
    cum = np.cumsum(col_counts[cols])
    lo_i = int(np.searchsorted(cum, 0.05 * total, side="left"))
    hi_i = min(int(np.searchsorted(cum, 0.95 * total, side="left")), cols.size - 1)
    return int(cols[hi_i]) - int(cols[lo_i]) + 1, total


class CrossLineDetector:
    """Stateless per call, like LaneDetector: safe to reuse, no history kept."""

    def __init__(self, cfg: CheckpointConfig | None = None) -> None:
        self.cfg = cfg or CheckpointConfig()

    def detect(self, frame: np.ndarray) -> CrossLineMeasurement | None:
        return self.detect_with_debug(frame)[0]

    def detect_with_debug(self, frame: np.ndarray) -> tuple[CrossLineMeasurement | None, dict]:
        """detect() plus the internals, for the dashboard overlay and tests."""
        arr = np.asarray(frame)
        if arr.ndim != 3 or arr.shape[2] != 3 or arr.dtype != np.uint8:
            raise ValueError(f"expected an HxWx3 uint8 BGR frame, got {arr.dtype} {arr.shape}")

        cfg = self.cfg
        h, w = arr.shape[:2]
        hue, sat, val = _hsv(arr)
        colored = (sat >= cfg.min_saturation) & (val >= cfg.min_value)
        debug: dict = {"h": h, "w": w, "colored_frac": float(colored.mean()), "reason": None}

        best: CrossLineMeasurement | None = None
        any_hue = False
        for kind, ranges in (
            ("checkpoint", cfg.checkpoint_hues),
            ("finish", cfg.finish_hues),
        ):
            if not ranges:
                debug[kind] = {"enabled": False}
                continue
            mask = colored & _hue_mask(hue, ranges)
            any_hue = any_hue or bool(mask.any())
            meas, kdbg = self._kind_debug(mask, kind, h, w)
            debug[kind] = kdbg
            if meas is not None and (best is None or meas.confidence > best.confidence):
                best = meas

        if best is None:
            debug["reason"] = "geometry" if any_hue else "no matching colour"
            return None, debug
        if best.confidence < cfg.min_confidence:
            debug["reason"] = "low confidence"
            return None, debug
        return best, debug

    def _kind_debug(
        self, mask: np.ndarray, kind: str, h: int, w: int
    ) -> tuple[CrossLineMeasurement | None, dict]:
        """Geometry test for one hue band. Returns (measurement or None, debug)."""
        cfg = self.cfg
        hot = mask.sum(axis=1) >= cfg.min_band_pixels
        runs = _runs(hot)
        max_rows = max(1, int(cfg.max_band_frac * h))
        thin = [(y0, y1) for (y0, y1) in runs if (y1 - y0) <= max_rows]
        dbg: dict = {"enabled": True, "pixels": int(mask.sum()), "n_runs": len(runs)}
        if not thin:
            dbg["reason"] = "no thin band"
            return None, dbg

        y0, y1 = max(thin, key=lambda r: int(mask[r[0]:r[1]].sum()))
        span = _band_span(mask, y0, y1)
        if span is None:
            dbg["reason"] = "empty band"
            return None, dbg
        span_px, pixels = span
        span_frac = span_px / float(w)
        height_frac = (y1 - y0) / float(h)
        dbg.update(
            band=[y0, y1],
            span_frac=span_frac,
            height_frac=height_frac,
            band_pixels=pixels,
        )
        if span_frac < cfg.min_span_frac:
            dbg["reason"] = "band too narrow"
            return None, dbg

        f_span = min(1.0, span_frac / max(cfg.target_span_frac, 1e-6))
        f_thin = 1.0 - height_frac / max(cfg.max_band_frac, 1e-6)
        f_fill = pixels / float(max(span_px * (y1 - y0), 1))
        conf = float(f_span * f_thin * min(1.0, f_fill))
        return (
            CrossLineMeasurement(
                kind=kind,
                confidence=conf,
                row=0.5 * (y0 + y1 - 1),
                span_frac=span_frac,
                height_frac=height_frac,
                pixels=pixels,
            ),
            dbg,
        )


@dataclass(frozen=True)
class CrossLineEvent:
    """One physical line was passed. Emitted once per crossing, never per frame."""

    kind: str              # "checkpoint" | "finish"
    row: float             # band row when the line was last seen
    confidence: float      # confidence of that last measurement


class _TrackState(Enum):
    IDLE = "idle"          # no line in view
    TRACKING = "tracking"  # line visible, still ahead of the robot
    AT_EXIT = "at_exit"    # line level with / behind the robot


class CrossLineTracker:
    """Turns per-frame measurements into crossing events.

    One event per physical line, with the two rules that matter on race day:

    - A line must be SEEN APPROACHING (in the upper part of the frame) before
      it counts. A line the robot starts sitting on - the start line at boot,
      or the checkpoint it was just restarted at after a violation - is first
      seen deep in the frame and never counts.
    - After a line leaves the frame the tracker re-arms, so one line can never
      fire twice, and rolling back over it does not fire either: it re-enters
      from the bottom, i.e. not approaching.

    A dropped detection frame does not end a crossing: the line must be gone
    for `min_gone_frames` consecutive frames. Losing the line while it is
    still approaching (state TRACKING) loses that count - deliberately
    conservative: better to undercount than to count a line never passed.

    The tracker is state-machine-agnostic on purpose: it emits events, and the
    caller decides what they mean (add_checkpoint / finish). That keeps
    perception free of control imports and the tracker testable on scripted
    frame sequences.
    """

    def __init__(
        self,
        detector: CrossLineDetector | None = None,
        *,
        exit_row_frac: float = 0.8,
        min_gone_frames: int = 2,
    ) -> None:
        self.detector = detector or CrossLineDetector()
        self.exit_row_frac = exit_row_frac
        self.min_gone_frames = max(1, int(min_gone_frames))
        self.state = _TrackState.IDLE
        self.approaching = False
        self.gone = 0
        self.last_row: float | None = None
        self.last_kind: str | None = None
        self.last_confidence = 0.0

    def reset(self) -> None:
        """Forget everything (new attempt / scenario change)."""
        self.state = _TrackState.IDLE
        self.approaching = False
        self.gone = 0
        self.last_row = None
        self.last_kind = None
        self.last_confidence = 0.0

    @property
    def debug(self) -> dict:
        """JSON-safe tracker state, for the dashboard overlay."""
        return {
            "state": self.state.value,
            "approaching": self.approaching,
            "gone": self.gone,
            "last_row": self.last_row,
            "last_kind": self.last_kind,
        }

    def update(self, frame: np.ndarray) -> CrossLineEvent | None:
        """Feed one frame. Returns an event the frame the crossing completed."""
        m = self.detector.detect(frame)
        if m is None:
            return self._on_gone()
        self.gone = 0
        self.last_row = m.row
        self.last_kind = m.kind
        self.last_confidence = m.confidence
        at_exit = m.row >= self.exit_row_frac * self._frame_rows(frame)
        if self.state is _TrackState.IDLE:
            # First sighting: deep in the frame means the robot is sitting on
            # (or just past) the line - the start line, or a restart checkpoint.
            self.state = _TrackState.AT_EXIT if at_exit else _TrackState.TRACKING
            self.approaching = self.state is _TrackState.TRACKING
        elif self.state is _TrackState.TRACKING and at_exit:
            self.state = _TrackState.AT_EXIT
        return None

    def _on_gone(self) -> CrossLineEvent | None:
        if self.state is _TrackState.IDLE:
            self.gone = 0
            return None
        self.gone += 1
        if self.gone < self.min_gone_frames:
            return None  # one dropped frame is not a crossing
        fired = (
            self.state is _TrackState.AT_EXIT
            and self.approaching
        )
        kind, row, conf = self.last_kind, self.last_row, self.last_confidence
        self.state = _TrackState.IDLE
        self.approaching = False
        self.gone = 0
        if fired and kind is not None and row is not None:
            return CrossLineEvent(kind=kind, row=row, confidence=conf)
        return None

    @staticmethod
    def _frame_rows(frame: np.ndarray) -> int:
        return int(np.asarray(frame).shape[0])

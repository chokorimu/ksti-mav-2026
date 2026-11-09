"""Obstacle (orange cone) detection + cone-pass counting for the ARC.

Checkpoints turned out to be markers OUTSIDE the lane (user, 9 Nov 2026), so
the cross-line detector is not what the robot needs on the arena floor. What
is on the floor is cones, and in the ARC rules §8 they ARE the checkpoints:
"Checkpoints are obstacles to avoid. Avoiding an obstacle counts as passing the
checkpoint." Two jobs follow, and they are deliberately separate:

  * `ConeDetector` - what is in front of us, in metres. Stateless, per frame.
  * `ConePassTracker` - how many we got past, one event per cone. This is the
    `cp` term in the score.

Colour-first, for the same reason `checkpoint.py` is: the lane markings are
white or black (rules §6) and the floor is grey, so an orange cone is the only
strongly saturated thing in the lower half of the frame. The hue band is a
parameter, not a constant - the rules forbid hardcoding arena colours [A11],
and nobody has measured the real cone yet.

Detection output is a per-frame LIST of blobs with their position in metres
relative to the camera axis, not an event. Deciding which way to steer around
them is the planner's job (`mav.planning.avoid`), not the detector's.

Reuses the HSV helpers from `mav.perception.checkpoint` rather than growing a
second colour implementation that can drift from the first.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from mav.perception.checkpoint import _hsv, _hue_mask, _runs


@dataclass(frozen=True)
class ObstacleConfig:
    """Cone colour and blob geometry. Defaults are UNCONFIRMED [A11]."""

    hues: tuple[tuple[float, float], ...] = ((5.0, 35.0),)  # orange, UNCONFIRMED
    min_saturation: float = 0.45   # grey floor and white/black markings stay below
    min_value: float = 0.25        # a shadowed cone is dark, not a colour
    roi_top: float = 0.35          # ignore the far field: a distant cone is a few pixels
    min_col_pixels: int = 2        # a column with fewer is noise, not a blob
    merge_gap_px: int = 8          # columns closer than this belong to one blob
    min_blob_pixels: int = 60
    min_width_frac: float = 0.01   # a cone is wider than a lane marking line
    max_width_frac: float = 0.45   # ...and narrower than half the frame
    min_height_frac: float = 0.02
    max_height_frac: float = 0.70
    px_per_m: float = 130.0        # UNCALIBRATED, same guess as LaneConfig
    bottom_row_m: float = 0.3      # UNCALIBRATED, same guess as LaneConfig


@dataclass(frozen=True)
class Obstacle:
    """One blob, positioned relative to the camera axis (not the lane centre)."""

    row: float          # image row of the blob centre; larger = closer
    col: float          # image column of the blob centre
    lateral_m: float    # + right, - left, relative to the camera axis
    distance_m: float   # forward distance of the blob centre (approx)
    width_frac: float   # blob width as a fraction of the frame width
    height_frac: float  # blob height as a fraction of the frame height
    pixels: int


class ConeDetector:
    """Stateless per call, like LaneDetector and CrossLineDetector."""

    def __init__(self, cfg: ObstacleConfig | None = None) -> None:
        self.cfg = cfg or ObstacleConfig()

    def detect(self, frame: np.ndarray) -> list[Obstacle]:
        return self.detect_with_debug(frame)[0]

    def detect_with_debug(self, frame: np.ndarray) -> tuple[list[Obstacle], dict]:
        """detect() plus the internals, for the dashboard overlay and tests."""
        arr = np.asarray(frame)
        if arr.ndim != 3 or arr.shape[2] != 3 or arr.dtype != np.uint8:
            raise ValueError(f"expected an HxWx3 uint8 BGR frame, got {arr.dtype} {arr.shape}")

        cfg = self.cfg
        h, w = arr.shape[:2]
        debug: dict = {"h": h, "w": w}
        if not cfg.hues:
            debug["reason"] = "disabled"
            return [], debug

        hue, sat, val = _hsv(arr)
        y0 = int(cfg.roi_top * h)
        mask = np.zeros((h, w), dtype=bool)
        mask[y0:] = (sat[y0:] >= cfg.min_saturation) & (val[y0:] >= cfg.min_value)
        mask &= _hue_mask(hue, cfg.hues)
        debug.update(
            roi_top_row=y0,
            mask_pixels=int(mask.sum()),
            mask_frac=float(mask.mean()),
        )

        blobs = self._blobs(mask, h, w)
        debug["n_blobs"] = len(blobs)
        debug["reason"] = None if blobs else ("no matching colour" if not mask.any() else "geometry")
        return blobs, debug

    def _blobs(self, mask: np.ndarray, h: int, w: int) -> list[Obstacle]:
        cfg = self.cfg
        counts = mask.sum(axis=0)
        hot = counts >= cfg.min_col_pixels
        if not hot.any():
            return []

        # Merge column runs separated by a small gap: a cone with a highlight or
        # a shadow can leave a 1-3 px hole, and splitting it in two would double
        # the obstacle count.
        spans: list[tuple[int, int]] = []
        for start, end in _runs(hot):
            if spans and start - spans[-1][1] <= cfg.merge_gap_px:
                spans[-1] = (spans[-1][0], end)
            else:
                spans.append((start, end))

        out: list[Obstacle] = []
        for c0, c1 in spans:
            cols = c1 - c0
            width_frac = cols / float(w)
            if not cfg.min_width_frac <= width_frac <= cfg.max_width_frac:
                continue
            block = mask[:, c0:c1]
            rows = np.nonzero(block.any(axis=1))[0]
            if rows.size == 0:
                continue
            height_frac = (rows[-1] - rows[0] + 1) / float(h)
            if not cfg.min_height_frac <= height_frac <= cfg.max_height_frac:
                continue
            pixels = int(block.sum())
            if pixels < cfg.min_blob_pixels:
                continue
            r_mid = 0.5 * (rows[0] + rows[-1])
            c_mid = 0.5 * (c0 + c1 - 1)
            out.append(
                Obstacle(
                    row=float(r_mid),
                    col=float(c_mid),
                    lateral_m=(c_mid - w / 2.0) / cfg.px_per_m,
                    distance_m=cfg.bottom_row_m + (h - 1 - r_mid) / cfg.px_per_m,
                    width_frac=width_frac,
                    height_frac=height_frac,
                    pixels=pixels,
                )
            )
        # Nearest first: the planner only has to care about the cone it will hit next.
        out.sort(key=lambda o: o.distance_m)
        return out


# ------------------------------------------------------------------ counting ----
# Why counting is separate from detecting (rules §8): "Avoiding an obstacle
# counts as passing the checkpoint", so the scoring term `cp` - the biggest
# term in the formula - is the number of cones PASSED, not lines crossed.
# `choose_offset` decides how to dodge; this decides whether the dodge worked.


class _PassState(Enum):
    IDLE = "idle"
    APPROACHING = "approaching"
    AT_EXIT = "at_exit"


@dataclass(frozen=True)
class ConePassEvent:
    """One cone passed. `lateral_m` is where it was relative to the camera axis."""

    lateral_m: float
    distance_m: float


class ConePassTracker:
    """One event per cone physically passed. Emits events; the caller scores them.

    Same two rules as `CrossLineTracker`, for the same reasons:

      - A cone must be SEEN APPROACHING (in the upper part of the frame) before
        it counts. A cone the robot starts next to - the one it was just
        restarted beside after a violation - is first seen deep in the frame and
        never counts.
      - It must then leave through the bottom. Leaving upward means the robot
        reversed away from it, which is not passing it.

    Deliberately does NOT judge "was it avoided". The car-to-cone distance that
    counts as a hit is a rule question nobody has answered, and the car is
    0.20 m wide while the cone's own diameter has not been measured. This counts
    passes; the referee applies the penalty for a hit.
    """

    def __init__(
        self,
        detector: ConeDetector | None = None,
        *,
        exit_row_frac: float = 0.8,
        min_gone_frames: int = 2,
        match_lateral_m: float = 0.25,
    ) -> None:
        self.detector = detector or ConeDetector()
        self.exit_row_frac = exit_row_frac
        self.min_gone_frames = max(1, int(min_gone_frames))
        # How far a cone may drift sideways between sightings and still be the
        # same cone. The camera is bolted to the robot, so a tracked cone moves
        # down the frame and only slightly sideways as the robot approaches.
        self.match_lateral_m = match_lateral_m
        self.state = _PassState.IDLE
        self.approaching = False
        self.gone = 0
        self.tracked: Obstacle | None = None
        self.passed = 0

    def reset(self) -> None:
        """Forget the cone being tracked (new attempt). Keeps `passed`: it is a score."""
        self.state = _PassState.IDLE
        self.approaching = False
        self.gone = 0
        self.tracked = None

    @property
    def debug(self) -> dict:
        """JSON-safe tracker state, for the dashboard overlay."""
        return {
            "state": self.state.value,
            "approaching": self.approaching,
            "gone": self.gone,
            "passed": self.passed,
            "lateral_m": None if self.tracked is None else self.tracked.lateral_m,
        }

    def update(self, frame: np.ndarray) -> ConePassEvent | None:
        """Feed one frame. Returns an event on the frame a cone is passed."""
        cone = self._nearest(self.detector.detect(frame))
        if cone is None:
            return self._on_gone()
        self.gone = 0
        at_exit = cone.row >= self.exit_row_frac * self._frame_rows(frame)
        if self.state is _PassState.IDLE:
            # First sighting. Deep in the frame means the robot is already
            # beside it, so it must not count.
            self.state = _PassState.AT_EXIT if at_exit else _PassState.APPROACHING
            self.approaching = self.state is _PassState.APPROACHING
        elif self.state is _PassState.APPROACHING and at_exit:
            self.state = _PassState.AT_EXIT
        self.tracked = cone
        return None

    def _nearest(self, blobs: list[Obstacle]) -> Obstacle | None:
        """The cone we are about to reach. The others are not reachable first."""
        if not blobs:
            return None
        if self.tracked is not None and self.state is not _PassState.IDLE:
            # Stay on the cone being followed rather than jumping to a different
            # one that happens to be nearer this frame.
            near = [
                b for b in blobs
                if abs(b.lateral_m - self.tracked.lateral_m) <= self.match_lateral_m
            ]
            if near:
                return min(near, key=lambda b: b.distance_m)
        return min(blobs, key=lambda b: b.distance_m)

    def _on_gone(self) -> ConePassEvent | None:
        if self.state is _PassState.IDLE:
            self.gone = 0
            return None
        self.gone += 1
        if self.gone < self.min_gone_frames:
            return None  # one dropped frame is not a pass
        fired = self.state is _PassState.AT_EXIT and self.approaching
        cone = self.tracked
        self.state, self.approaching, self.gone, self.tracked = _PassState.IDLE, False, 0, None
        if not fired or cone is None:
            return None
        self.passed += 1
        return ConePassEvent(lateral_m=cone.lateral_m, distance_m=cone.distance_m)

    @staticmethod
    def _frame_rows(frame: np.ndarray) -> int:
        return int(np.asarray(frame).shape[0])

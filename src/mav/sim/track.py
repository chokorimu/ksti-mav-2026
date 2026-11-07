"""The measured KSTI 2026 track, as a scenario the simulator can drive.

`data/track_ksti.json` comes from the arena-3d extraction tool: 1440 samples of
the centreline in metres, origin at the loop centre, x = map x, y = map z, scale
anchored to the guidebook's "lebar lintasan 50 cm" rule. Nothing here is
estimated -- it is measured off the committee's own drawing.

`track_scenario()` rotates and translates that closed loop so the vehicle starts
at the origin looking down +Y, exactly like every other scenario. The transform
is rigid, so curvature -- and therefore the tightest radius -- survives it.

Why this scenario exists: the measured tightest centreline radius is 0.46 m,
while `LanePolicy` clamps steering to the rules' 1.5 m minimum turning radius.
Roughly 39% of the lap is tighter than that clamp. The synthetic scenarios only
ever went down to R=2 m, so until now nothing had tested the policy against the
geometry it will actually meet.
"""
from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np

from mav.sim.scenarios import Scenario

DATA_DIR = Path(__file__).resolve().parent / "data"

# name -> {file, description}. Static on purpose: listing scenarios and
# validating a control from the browser must not read a 125 KB JSON to do it.
TRACKS: dict[str, dict[str, str]] = {
    "ksti": {
        "file": "track_ksti.json",
        "description": "official track measured from the map: 19.41 m loop, "
        "0.50 m wide, tightest radius 0.46 m",
    },
}

# The policy's steering clamp, mirrored here so the stats below can report how
# much of the lap the robot physically cannot follow. Keep in sync with
# mav.planning.lane_policy._MIN_TURN_RADIUS_M.
POLICY_MIN_RADIUS_M = 1.5


def _raw(name: str) -> dict[str, Any]:
    try:
        spec = TRACKS[name]
    except KeyError:
        raise KeyError(
            f"unknown track {name!r}; have: {', '.join(sorted(TRACKS))}"
        ) from None
    path = DATA_DIR / spec["file"]
    if not path.is_file():
        raise FileNotFoundError(f"track data missing: {path}")
    with path.open(encoding="utf-8") as fh:
        return json.load(fh)


def track_points(name: str = "ksti") -> np.ndarray:
    """Centreline as (N, 2) metres, rotated and translated to start at the
    origin heading +Y. The transform is rigid, so curvature is preserved."""
    pts = np.asarray(_raw(name)["centre"], dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 16:
        raise ValueError(
            f"track {name!r}: expected an Nx2 centreline, got shape {pts.shape}"
        )
    # Tangent averaged over the first few samples: one segment is noisy where
    # the polyline is dense and the corner is tight.
    d = pts[8] - pts[0]
    if not np.any(d):
        raise ValueError(f"track {name!r}: degenerate start (first 8 samples coincide)")
    # alpha = atan2(dx, dy) is the rotation that carries d onto +Y:
    #   R(alpha) @ d = (dx*cos - dy*sin, dx*sin + dy*cos) = (0, |d|)
    alpha = math.atan2(float(d[0]), float(d[1]))
    c, s = math.cos(alpha), math.sin(alpha)
    rot = np.array([[c, -s], [s, c]], dtype=np.float64)
    out = pts @ rot.T
    return out - out[0]


@lru_cache(maxsize=None)
def track_scenario(name: str = "ksti") -> Scenario:
    """The measured track as a `Scenario` (cached: the JSON is read once)."""
    pts = track_points(name)
    return Scenario(
        name=name,
        segments=(),
        start_offset_m=0.0,
        description=TRACKS[name]["description"],
        points=tuple((float(p[0]), float(p[1])) for p in pts),
    )


def track_stats(name: str = "ksti") -> dict[str, Any]:
    """Geometry facts, for reporting rather than for the control loop."""
    raw = _raw(name)
    radius = np.asarray(raw["radius"], dtype=np.float64)
    width = np.asarray(raw["width"], dtype=np.float64)
    s = np.asarray(raw["s"], dtype=np.float64)
    if not (radius.size == width.size == s.size) or s.size < 2:
        raise ValueError(f"track {name!r}: radius/width/s arrays disagree in length")
    seg = np.diff(s)
    tight = radius < POLICY_MIN_RADIUS_M
    # A segment counts as tight when either end is: the arc between two samples
    # is not straight just because its endpoints happen to be.
    tight_seg = tight[:-1] | tight[1:]
    meta = raw.get("meta") or {}
    return {
        "samples": int(radius.size),
        "length_m": float(s[-1]),
        "min_radius_m": float(radius.min()),
        "median_radius_m": float(np.median(radius)),
        "width_min_m": float(width.min()),
        "width_max_m": float(width.max()),
        "fraction_tighter_than_policy": float(tight.mean()),
        "arc_tighter_than_policy_m": float(seg[tight_seg].sum()),
        "source": str(meta.get("source", "")),
    }


def measured_radius_min(points: np.ndarray, stride: int = 20) -> float:
    """Tightest circumradius of the polyline, computed from the points alone.

    An independent check that the rigid transform really did preserve curvature:
    three points on a circle determine it exactly, so this should come back with
    the `min_radius_m` the extraction reported.

    `stride` sets the span of each triple (3*stride samples, ~27 cm at the
    default). It has to be that wide: the extracted polyline carries sub-
    millimetre wiggle, and at the native 1.35 cm spacing three consecutive
    points measure that noise instead of the corner -- stride=1 reports a
    0.055 m "corner" on a 7.9 m straight.
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 3 * stride:
        raise ValueError(f"need at least 3*stride={3 * stride} points, got {len(pts)}")
    a, b, c = pts[:-2 * stride], pts[stride:-stride], pts[2 * stride:]
    ab, ac = b - a, c - a
    cross = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
    lab = np.linalg.norm(ab, axis=1)
    lac = np.linalg.norm(ac, axis=1)
    lbc = np.linalg.norm(c - b, axis=1)
    area = 0.5 * np.abs(cross)
    with np.errstate(invalid="ignore", divide="ignore"):
        r = (lab * lac * lbc) / (4.0 * area)
    r = r[np.isfinite(r) & (r > 1e-6)]
    if r.size == 0:
        raise ValueError("no finite radius")
    return float(r.min())


__all__ = [
    "DATA_DIR",
    "TRACKS",
    "POLICY_MIN_RADIUS_M",
    "track_points",
    "track_scenario",
    "track_stats",
    "measured_radius_min",
]

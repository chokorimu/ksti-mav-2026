"""Simulated world: lane geometry, vehicle pose, camera frames. Numpy-only (no cv2),
so the simulator and the tests run anywhere.

Conventions (same as the detector and the policy): X right, Y forward, heading
theta measured left-positive, curvature > 0 = left turn. The vehicle starts at
the origin looking down +Y; the lane is a polyline in world coordinates.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mav.sim.render import render_frame
from mav.types import Command


@dataclass(frozen=True)
class Segment:
    """One piece of the lane centreline: straight (curvature=0) or a circular arc."""

    length_m: float
    curvature: float = 0.0  # 1/m, +left


# BGR for the cross-line kinds. Single source of truth for the sim camera
# layer; the runner derives the detector's hue bands from these, so what is
# rendered and what is detectable can never drift apart. Lives here (not in
# scenarios.py) because scenarios imports this module, not the other way.
CROSS_LINE_BGR: dict[str, tuple[int, int, int]] = {
    "checkpoint": (0, 200, 0),   # green
    "finish": (0, 0, 200),       # red
}


@dataclass(frozen=True)
class WorldConfig:
    px_per_m: float = 130.0        # scale at the bottom row (y_near_m)
    y_near_m: float = 0.35         # forward distance of the frame's bottom row
    y_far_m: float = 2.2           # forward distance covered by the frame
    taper: float = 0.25            # horizontal scale shrinks linearly with distance
    lane_width_m: float = 0.5
    line_width_m: float = 0.05
    marking: str = "white"         # "white" | "black"
    floor_bgr: tuple[int, int, int] = (115, 115, 115)
    brightness: float = 1.0
    noise: float = 4.0
    size: tuple[int, int] = (360, 640)  # (height, width)
    max_speed_mps: float = 1.5
    corridor_m: float = 0.6        # |lateral offset| beyond this = left the track
    seed: int = 0

    def scale_at(self, y_m: float) -> float:
        """Horizontal px/m at forward distance y_m (taper model)."""
        span = max(self.y_far_m - self.y_near_m, 1e-6)
        return self.px_per_m * (1.0 - self.taper * (y_m - self.y_near_m) / span)


class LanePath:
    """Dense polyline of the lane centreline in world coordinates."""

    def __init__(self, points: np.ndarray, length_m: float) -> None:
        self.points = points
        self.length_m = length_m
        # Cumulative arc length: lets the camera view follow the branch of the
        # path the vehicle is driving on instead of any point that happens to
        # fall inside the forward window.
        d = np.linalg.norm(np.diff(points, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(d)])

    @classmethod
    def from_points(cls, points: np.ndarray, length_m: float | None = None) -> "LanePath":
        """Wrap an already-sampled centreline (the measured track, a recording).

        `build_path` integrates straights and arcs; this takes a polyline as-is.
        `length_m` is the exact arc length when the caller knows it (an
        integrated path does); without it the chord sum is used, which is what a
        measured polyline wants anyway.
        """
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
            raise ValueError(f"expected an Nx2 polyline, got shape {pts.shape}")
        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        return cls(pts, float(d.sum()) if length_m is None else float(length_m))

    def vehicle_frame(self, x: float, y: float, theta: float) -> tuple[np.ndarray, np.ndarray]:
        """Centreline points as (lateral right+, forward) relative to a pose."""
        c, s = math.cos(theta), math.sin(theta)
        dx = self.points[:, 0] - x
        dy = self.points[:, 1] - y
        forward = -dx * s + dy * c   # vehicle forward axis is (-sin, cos)
        lateral = dx * c + dy * s    # vehicle right axis is (cos, sin)
        return lateral, forward

    def centre_x_fn(self, x: float, y: float, theta: float, y_near: float, y_far: float):
        """Callable forward_m -> lateral centre position (m), or None if no lane in view.

        Only the branch of the path the vehicle is driving on is used: the scan
        starts at the nearest polyline point and stops y_far metres of ARC past
        it. Selecting by forward distance alone lets a path that wraps around
        (a long curve) put its far end back inside the forward window, which
        corrupts the interpolation with points the vehicle will not reach.
        """
        lateral, forward = self.vehicle_frame(x, y, theta)
        i0 = int(np.argmin(forward * forward + lateral * lateral))
        m = (
            (self.s >= self.s[i0] - 0.5)
            & (self.s <= self.s[i0] + y_far + 0.5)
            & (forward >= y_near)
            & (forward <= y_far)
        )
        if int(m.sum()) < 2:
            return None
        f = forward[m]
        lat = lateral[m]
        order = np.argsort(f, kind="stable")
        f, lat = f[order], lat[order]
        keep = np.ones(f.shape, dtype=bool)
        keep[1:] = np.diff(f) > 1e-9  # one point per forward distance
        f, lat = f[keep], lat[keep]
        if f.size < 2:
            return None

        def centre_x(yy: float) -> float:
            return float(np.interp(yy, f, lat))

        return centre_x


def build_path(
    segments: list[Segment] | tuple[Segment, ...],
    start_offset_m: float = 0.0,
    step_m: float = 0.01,
) -> LanePath:
    """Integrate the centreline. Exact circular arcs, sampled every ~step_m."""
    x, y, psi = start_offset_m, 0.0, 0.0
    pts = [(x, y)]
    total = 0.0
    for seg in segments:
        s = 0.0
        while s < seg.length_m - 1e-9:
            ds = min(step_m, seg.length_m - s)
            k = seg.curvature
            if abs(k) < 1e-12:
                x += -ds * math.sin(psi)
                y += ds * math.cos(psi)
            else:
                psi_new = psi + k * ds
                x += (math.cos(psi_new) - math.cos(psi)) / k
                y += (math.sin(psi_new) - math.sin(psi)) / k
                psi = psi_new
            s += ds
            total += ds
            pts.append((x, y))
    return LanePath.from_points(np.array(pts, dtype=np.float64), length_m=total)


class SimWorld:
    """Vehicle + lane + camera. apply(cmd, dt) integrates the pose; frame() renders."""

    def __init__(self, path: LanePath, cfg: WorldConfig | None = None, cross_lines: list[tuple[float, tuple[int, int, int]]] | None = None) -> None:
        self.path = path
        self.cfg = cfg or WorldConfig()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t = 0.0
        self.distance = 0.0
        self._rng = np.random.default_rng(self.cfg.seed)
        self.cross_lines = list(cross_lines or [])

    def apply(self, cmd: Command, dt: float) -> None:
        """Integrate the pose under a constant-curvature command (exact for arcs)."""
        v = max(-self.cfg.max_speed_mps, min(self.cfg.max_speed_mps, cmd.speed))
        k = cmd.curvature
        dtheta = v * k * dt
        if abs(dtheta) < 1e-12:
            self.x += -v * dt * math.sin(self.theta)
            self.y += v * dt * math.cos(self.theta)
        else:
            theta_new = self.theta + dtheta
            self.x += (math.cos(theta_new) - math.cos(self.theta)) / k
            self.y += (math.sin(theta_new) - math.sin(self.theta)) / k
            self.theta = theta_new
        self.t += dt
        self.distance += abs(v) * dt

    def frame(self) -> np.ndarray | None:
        """Render the camera view at the current pose (blank floor if no lane in view)."""
        cfg = self.cfg
        fn = self.path.centre_x_fn(self.x, self.y, self.theta, cfg.y_near_m, cfg.y_far_m)
        cross = None
        if fn is not None and self.cross_lines:
            # Cross-lines live at arc-length s on the centreline; project each
            # one into the vehicle frame and keep the ones inside the view.
            lateral, forward = self.path.vehicle_frame(self.x, self.y, self.theta)
            cross = []
            for s_m, color in self.cross_lines:
                i = int(np.argmin(np.abs(self.path.s - s_m)))
                y_line = float(forward[i])
                if cfg.y_near_m - 0.1 <= y_line <= cfg.y_far_m + 0.1:
                    cross.append((y_line, color))
        return render_frame(
            fn if fn is not None else (lambda _y: 0.0),
            y_near_m=cfg.y_near_m,
            y_far_m=cfg.y_far_m,
            px_per_m=cfg.px_per_m,
            taper=cfg.taper,
            lane_width_m=cfg.lane_width_m,
            line_width_m=cfg.line_width_m,
            marking=cfg.marking,
            floor_bgr=cfg.floor_bgr,
            brightness=cfg.brightness,
            noise=cfg.noise,
            size=cfg.size,
            rng=self._rng,
            draw_lane=fn is not None,
            cross_lines=cross,
        )

    def true_offset_at(self, forward_m: float) -> float | None:
        """True lateral position (m, +right) of the lane centre at a forward distance."""
        fn = self.path.centre_x_fn(
            self.x, self.y, self.theta, forward_m - 0.5, forward_m + 0.5
        )
        return None if fn is None else fn(forward_m)

    @property
    def finished(self) -> bool:
        return self.distance >= self.path.length_m - 0.5

    @property
    def crashed(self) -> bool:
        off = self.true_offset_at(0.5)
        return off is not None and abs(off) > self.cfg.corridor_m

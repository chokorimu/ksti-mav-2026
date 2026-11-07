"""The measured KSTI 2026 track as a simulator scenario.

The track geometry is data we did not produce, so these tests assert the facts
the extraction reported and the properties the transform must not break -- not
numbers we tuned.
"""
from __future__ import annotations

import numpy as np
import pytest

from mav.sim.runner import run_sim
from mav.sim.scenarios import (
    SCENARIOS,
    get_scenario,
    scenario_info,
    scenario_names,
)
from mav.sim.track import (
    POLICY_MIN_RADIUS_M,
    measured_radius_min,
    track_points,
    track_scenario,
    track_stats,
)


# -- the data itself -------------------------------------------------------

def test_track_data_is_the_measured_official_track():
    st = track_stats("ksti")
    assert st["samples"] == 1440
    assert st["length_m"] == pytest.approx(19.41, abs=0.05)
    assert st["min_radius_m"] == pytest.approx(0.46, abs=0.02)
    assert st["median_radius_m"] == pytest.approx(1.97, abs=0.1)
    # The guidebook says 50 cm +-10%; the drawing holds it far tighter than that.
    assert st["width_min_m"] == pytest.approx(0.499, abs=0.005)
    assert st["width_max_m"] == pytest.approx(0.503, abs=0.005)
    assert "lintasan" in st["source"].lower()


def test_the_track_is_far_tighter_than_the_policy_can_steer():
    """The reason this scenario exists. If this ever stops being true, the
    synthetic scenarios are enough again and this file can be dropped."""
    st = track_stats("ksti")
    assert st["fraction_tighter_than_policy"] > 0.30
    assert st["arc_tighter_than_policy_m"] > 5.0
    assert st["min_radius_m"] < POLICY_MIN_RADIUS_M / 3.0


# -- the transform into simulator coordinates ------------------------------

def test_track_starts_at_the_origin_heading_forward():
    pts = track_points("ksti")
    assert pts.ndim == 2 and pts.shape[1] == 2
    assert np.allclose(pts[0], (0.0, 0.0), atol=1e-12)
    tangent = pts[8] - pts[0]
    tangent = tangent / np.linalg.norm(tangent)
    # +Y is forward in this codebase, and the vehicle starts with theta = 0.
    assert tangent[0] == pytest.approx(0.0, abs=1e-9)
    assert tangent[1] == pytest.approx(1.0, abs=1e-9)


def test_the_transform_is_rigid_so_curvature_survives():
    """Rotating/translating must not change the tightest radius. Three points on
    a circle determine it exactly, so recomputing from the points alone has to
    come back with the radius the extraction reported."""
    reported = track_stats("ksti")["min_radius_m"]
    assert measured_radius_min(track_points("ksti")) == pytest.approx(reported, rel=0.1)


def test_the_radius_check_needs_a_wide_window():
    """The raw polyline wiggles by sub-millimetre amounts; three adjacent points
    measure that noise, not the corner. Guard the stride the check relies on."""
    pts = track_points("ksti")
    assert measured_radius_min(pts, stride=1) < 0.2      # noise, not geometry
    assert measured_radius_min(pts, stride=20) > 0.4     # the real corner


def test_the_loop_closes():
    pts = track_points("ksti")
    assert np.linalg.norm(pts[0] - pts[-1]) < 0.05


def test_polyline_is_dense_enough_for_the_camera_model():
    """~1.3 cm spacing: the forward window is only ~2 m, so this is plenty."""
    pts = track_points("ksti")
    spacing = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert spacing.mean() < 0.02


# -- wiring into the scenario registry -------------------------------------

def test_get_scenario_returns_the_measured_track():
    sc = get_scenario("ksti")
    assert sc.name == "ksti"
    assert sc.points is not None and len(sc.points) == 1440
    assert sc.segments == ()
    assert sc.start_offset_m == 0.0
    # cached, so switching scenarios does not re-read the JSON every time
    assert get_scenario("ksti") is sc
    assert track_scenario("ksti") is sc


def test_scenario_names_and_info_cover_both_kinds():
    names = scenario_names()
    assert names == sorted(names)
    for synthetic in SCENARIOS:
        assert synthetic in names
    assert "ksti" in names
    name, desc = scenario_info("ksti")
    assert name == "ksti" and "0.46" in desc


def test_listing_scenarios_does_no_file_io():
    """The browser validates controls on every POST; that must stay cheap."""
    import inspect

    src = inspect.getsource(scenario_names) + inspect.getsource(scenario_info)
    assert "track_points" not in src and "track_scenario" not in src


def test_unknown_scenario_still_raises():
    with pytest.raises(KeyError, match="unknown scenario"):
        get_scenario("does-not-exist")


def test_a_control_can_select_the_track():
    from mav.sim.dashboard.bridge import parse_control

    c = parse_control({"action": "scenario", "scenario": "ksti"})
    assert c.action == "scenario" and c.scenario == "ksti"


# -- the simulator can actually drive it -----------------------------------

def test_the_sim_drives_the_measured_track():
    res = run_sim("ksti", max_steps=200, dt=0.05, seed=0)
    assert res.steps >= 150            # it ran, it did not crash on step 3
    assert res.detected > 100          # the detector found the lane
    assert res.distance_m > 1.0        # and the robot actually moved


def test_the_synthetic_scenarios_still_build_the_same_paths():
    """Regression guard for adding LanePath.from_points: wrapping an integrated
    path must reproduce its points and arc-length table exactly."""
    from mav.sim.world import LanePath, Segment, build_path

    segs = (Segment(6.0, 0.0), Segment(1.0, 0.5), Segment(4.0, 0.0))
    integrated = build_path(segs)
    wrapped = LanePath.from_points(integrated.points, length_m=integrated.length_m)
    assert wrapped.length_m == pytest.approx(integrated.length_m, rel=1e-12)
    assert np.array_equal(wrapped.points, integrated.points)
    assert np.array_equal(wrapped.s, integrated.s)


def test_from_points_defaults_to_the_chord_sum():
    """Without an exact length the chord sum is used -- correct for a measured
    polyline, and only ~1e-6 m short of the true arc at this sampling."""
    from mav.sim.world import LanePath, Segment, build_path

    integrated = build_path((Segment(6.0, 0.0), Segment(1.0, 0.5), Segment(4.0, 0.0)))
    wrapped = LanePath.from_points(integrated.points)
    assert wrapped.length_m == pytest.approx(integrated.length_m, abs=1e-4)
    assert np.array_equal(wrapped.points, integrated.points)
    assert np.array_equal(wrapped.s, integrated.s)


def test_from_points_rejects_junk():
    from mav.sim.world import LanePath

    for bad in (np.zeros(4), np.zeros((2, 3)), np.zeros((1, 2))):
        with pytest.raises(ValueError, match="Nx2 polyline"):
            LanePath.from_points(bad)

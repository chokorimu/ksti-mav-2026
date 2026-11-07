"""Simulator tests: kinematics first, then the closed loop (detector + policy)."""
import math

import pytest

from mav.sim.runner import run_sim
from mav.sim.world import Segment, SimWorld, WorldConfig, build_path
from mav.types import Command


def test_path_straight_stays_on_axis():
    path = build_path([Segment(5.0)])
    assert path.length_m == pytest.approx(5.0)
    assert abs(path.points[:, 0]).max() < 1e-9
    assert abs(path.points[-1, 1] - 5.0) < 1e-6


def test_path_left_curve_bends_left():
    # curvature > 0 = left = -X in this convention.
    path = build_path([Segment(5.0, 0.5)])
    assert path.points[-1, 0] < -0.5
    assert path.length_m == pytest.approx(5.0)


def test_world_straight_command_drives_forward():
    world = SimWorld(build_path([Segment(20.0)]), WorldConfig())
    for _ in range(10):
        world.apply(Command(0.5, 0.0), 0.1)
    assert abs(world.y - 0.5) < 1e-9
    assert abs(world.x) < 1e-9
    assert abs(world.theta) < 1e-9


def test_world_left_curvature_turns_left():
    world = SimWorld(build_path([Segment(20.0)]), WorldConfig())
    for _ in range(10):
        world.apply(Command(0.5, 0.5), 0.1)  # R = 2 m
    assert world.theta > 0.2
    assert world.x < 0  # moved left
    # Arc length 0.5 m at R=2 m -> heading change 0.25 rad.
    assert abs(world.theta - 0.25) < 1e-9


def test_world_frame_has_lane_pixels():
    world = SimWorld(build_path([Segment(20.0)]), WorldConfig(seed=1))
    frame = world.frame()
    assert frame.shape == (360, 640, 3)
    assert (frame > 200).mean() > 0.01  # white markings visible


def test_closed_loop_straight_centres():
    res = run_sim("straight", max_steps=250, seed=2)
    assert not res.crashed
    assert res.detection_rate > 0.9
    assert res.mean_abs_offset < 0.1
    assert res.departures == 0
    assert res.distance_m > 2.0  # actually moved


def test_closed_loop_recovers_from_offset():
    res = run_sim("offset", max_steps=250, seed=3)
    assert not res.crashed
    assert res.detection_rate > 0.9
    # offset shrinks from the initial 0.3 m towards the centre
    measured = [abs(o) for _, o, _, _, _ in res.history if o is not None]
    assert measured[-1] < 0.1
    assert measured[-1] < measured[0]


def test_closed_loop_follows_curve():
    # v1 policy has no curvature feedforward, so a steady-state lag is expected;
    # it must still stay inside the corridor and keep detecting.
    res = run_sim("curve", max_steps=250, seed=4)
    assert not res.crashed
    assert res.detection_rate > 0.8
    assert res.max_abs_offset < 0.4


def test_closed_loop_black_markings():
    cfg = WorldConfig(marking="black", seed=5)
    res = run_sim("straight", max_steps=120, seed=5, world_cfg=cfg)
    assert res.detection_rate > 0.8
    assert res.mean_abs_offset < 0.15

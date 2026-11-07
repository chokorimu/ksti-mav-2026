"""Dashboard contract tests: state serialisation, control parsing, bridge isolation.

The important one is `test_bridge_does_not_change_the_result`: the dashboard is a
viewer plus a control channel, never a participant, so attaching a bridge must
not move the robot by a single pixel.
"""
import json

import numpy as np
import pytest

from mav.sim.dashboard.bridge import Control, FakeBridge, NullBridge, parse_control
from mav.sim.dashboard.state import Pose, SimState, _plain
from mav.sim.runner import run_sim

# ---------------------------------------------------------------- state ----


def test_state_to_dict_is_json_safe():
    st = SimState(
        step=3, t=0.15, scenario="straight", robot_state="run", checkpoints=1,
        pose=Pose(0.1, 1.2, -0.05), lookahead_m=1.1, true_offset_m=0.02,
        measurement={"offset_m": 0.01, "confidence": 0.9, "n_points": np.int64(12)},
        debug={"sigma": np.float64(2.5), "band_points": [(8.0, 320.1, 340.0)],
               "fit": {"a": np.float32(0.001), "b": -0.2, "c": 320.0}},
        command={"speed": 0.8, "curvature": -0.02},
        metrics={"detected": 3, "detection_rate": 1.0}, fps=20.0,
    )
    d = st.to_dict()
    json.dumps(d)  # must not raise
    assert d["pose"] == {"x": 0.1, "y": 1.2, "theta": -0.05}
    assert d["measurement"]["n_points"] == 12
    assert d["debug"]["sigma"] == 2.5
    assert d["debug"]["band_points"] == [[8.0, 320.1, 340.0]]
    assert d["debug"]["fit"]["a"] == pytest.approx(0.001, abs=1e-6)


def test_plain_drops_non_finite_and_numpy():
    out = _plain({
        "nan": float("nan"), "inf": float("inf"), "f32": np.float32(1.5),
        "i64": np.int64(3), "arr": np.array([1.0, np.nan]), "none": None,
        "lst": [np.float64(2.0), "x", True], "b": np.bool_(True),
    })
    assert out == {"nan": None, "inf": None, "f32": 1.5, "i64": 3,
                   "arr": [1.0, None], "none": None, "lst": [2.0, "x", True], "b": True}
    json.dumps(out)


# -------------------------------------------------------------- control ----


def test_parse_control_accepts_known_actions():
    assert parse_control({"action": "start"}) == Control("start", None)
    assert parse_control({"action": "emergency"}) == Control("emergency", None)
    assert parse_control({"action": "reset"}) == Control("reset", None)
    assert parse_control({"action": "scenario", "scenario": "curve"}) == Control("scenario", "curve")


@pytest.mark.parametrize("payload", [
    {}, {"action": "nope"}, {"action": "scenario"}, {"action": "scenario", "scenario": "nope"},
    {"action": "start", "scenario": "curve"}, "start", None, 7,
])
def test_parse_control_rejects_bad_payloads(payload):
    with pytest.raises(ValueError):
        parse_control(payload)


# --------------------------------------------------------------- bridge ----


def test_null_bridge_is_inert():
    b = NullBridge()
    b.publish(SimState(), None)
    assert b.poll_control() is None


def test_fake_bridge_records_and_replays():
    b = FakeBridge([Control("start"), Control("emergency")])
    assert b.poll_control() == Control("start")
    assert b.poll_control() == Control("emergency")
    assert b.poll_control() is None
    b.publish(SimState(step=1), None)
    assert [s.step for s in b.published] == [1]


# ------------------------------------------------------- runner isolation ----


def test_bridge_does_not_change_the_result():
    """A viewer must not move the robot: identical seed -> identical history."""
    a = run_sim("straight", max_steps=120, seed=2)
    bridge = FakeBridge()
    b = run_sim("straight", max_steps=120, seed=2, bridge=bridge)
    assert a.history == b.history
    assert (a.steps, a.detected, a.lost, a.distance_m, a.max_abs_offset) == \
           (b.steps, b.detected, b.lost, b.distance_m, b.max_abs_offset)
    assert bridge.published, "the bridge should have received states"


def test_published_state_matches_the_documented_contract():
    b = FakeBridge()
    run_sim("straight", max_steps=25, seed=3, bridge=b)
    d = b.published[-1].to_dict()
    json.dumps(d)
    for key in ("step", "t", "scenario", "robot_state", "checkpoints", "pose", "lookahead_m",
                "true_offset_m", "measurement", "debug", "command", "metrics", "fps"):
        assert key in d, key
    assert d["robot_state"] == "run"
    assert d["pose"]["y"] > 0
    assert d["measurement"] is not None
    assert d["measurement"]["confidence"] > 0.3
    assert d["debug"]["polarity"] in ("light", "dark")
    assert len(d["debug"]["band_points"]) >= 4
    assert set(d["debug"]["fit"]) == {"a", "b", "c"}
    assert d["metrics"]["detection_rate"] > 0.9
    assert d["fps"] > 0


# ------------------------------------------------------ controls in loop ----


def test_emergency_on_the_first_tick_stops_everything():
    b = FakeBridge([Control("emergency")])
    res = run_sim("straight", max_steps=40, seed=1, bridge=b)
    assert res.steps == 0            # policy never ran
    assert res.distance_m == 0.0
    assert b.published[-1].robot_state == "stopped"


def test_emergency_then_start_resumes_the_run():
    b = FakeBridge([Control("emergency"), Control("start")])
    res = run_sim("straight", max_steps=60, seed=1, bridge=b)
    states = [s.robot_state for s in b.published]
    assert states[0] == "stopped"
    assert "run" in states
    assert res.steps > 0
    assert res.distance_m > 0


def test_scenario_switch_rebuilds_the_world_and_waits_for_start():
    b = FakeBridge([Control("scenario", "offset")])
    res = run_sim("straight", max_steps=30, seed=1, bridge=b)
    assert res.scenario == "offset"
    assert res.steps == 0                      # waits for START, like the real robot
    assert b.published[-1].scenario == "offset"
    assert b.published[-1].robot_state == "wait_start"


def test_reset_restarts_the_same_scenario():
    b = FakeBridge([Control("reset")])
    res = run_sim("curve", max_steps=20, seed=1, bridge=b)
    assert res.scenario == "curve"
    assert res.steps == 0
    assert b.published[-1].checkpoints == 0


def test_detect_with_debug_matches_detect():
    """The debug variant must not change the measurement it reports."""
    from mav.perception.lane import LaneDetector
    from synthetic_lane import make_frame

    det = LaneDetector()
    for seed in range(5):
        frame = make_frame(offset_m=0.1 * seed, curvature=0.0, seed=seed)
        m1 = det.detect(frame)
        m2, dbg = det.detect_with_debug(frame)
        assert (m1 is None) == (m2 is None)
        if m1 is not None:
            assert m1 == m2
            assert dbg["polarity"] in ("light", "dark")
            assert dbg["n_bands_with_pixels"] == m1.n_points
            assert dbg["reason"] is None

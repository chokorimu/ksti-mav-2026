"""Dashboard HTTP server tests (subtask 2).

Runs a REAL server on an ephemeral port and drives it over HTTP with urllib —
no mocks. The last test runs the real sim loop in a thread and checks that a
POST emergency freezes the robot and a POST start resumes it, which is the
competition resume rule seen through the browser.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pytest

from mav.sim.dashboard.bridge import Control
from mav.sim.dashboard.server import HttpBridge, serve_in_thread, stop
from mav.sim.dashboard.state import Pose, SimState
from mav.sim.runner import run_sim


def _get(port: int, path: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=5) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def _get_json(port: int, path: str) -> tuple[int, dict]:
    status, body = _get(port, path)
    return status, json.loads(body)


def _post(port: int, path: str, payload) -> tuple[int, dict]:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}", data=data,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def _wait_for(port: int, pred, timeout: float = 8.0) -> dict:
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            _, d = _get_json(port, "/state.json")
            last = d
            if d.get("ready") and pred(d):
                return d
        except Exception:
            pass
        time.sleep(0.02)
    raise AssertionError(f"condition not met within {timeout}s; last={last}")


@pytest.fixture()
def srv():
    server, bridge = serve_in_thread({"host": "127.0.0.1", "port": 0})
    yield server, bridge, server.server_address[1]
    stop(server)


def _sample_state() -> SimState:
    return SimState(
        step=7, t=0.35, scenario="curve", robot_state="run", checkpoints=2,
        pose=Pose(0.4, 1.9, 0.31), lookahead_m=1.05, true_offset_m=-0.03,
        measurement={"offset_m": -0.02, "curvature_inv_m": 0.48, "confidence": 0.91,
                     "n_points": 11, "lookahead_m": 1.05},
        debug={"polarity": "light", "band_points": [[10, 300, 40]], "fit": {"a": 0.001, "b": 0.2, "c": 310.0}},
        command={"speed": 0.7, "curvature": 0.4},
        metrics={"detected": 7, "detection_rate": 1.0, "distance_m": 1.9},
        fps=19.5,
        lane=[[0.0, 0.0], [0.0, 1.0], [0.5, 2.0]],
        trail=[[0.0, 0.0, 0.0], [0.1, 0.5, 0.05]],
        lane_width_m=0.5, corridor_m=0.6,
    )


# ------------------------------------------------------------- endpoints ----


def test_state_json_has_telemetry_and_no_geometry(srv):
    server, bridge, port = srv
    bridge.publish(_sample_state(), np.zeros((8, 8, 3), dtype=np.uint8))
    status, d = _get_json(port, "/state.json")
    assert status == 200
    assert d["ready"] is True
    assert d["step"] == 7 and d["scenario"] == "curve" and d["robot_state"] == "run"
    assert d["pose"] == {"x": 0.4, "y": 1.9, "theta": 0.31}
    assert d["measurement"]["confidence"] == 0.91
    assert d["debug"]["fit"]["c"] == 310.0
    assert d["metrics"]["distance_m"] == 1.9
    assert d["fps"] == 19.5
    for key in ("lane", "trail", "lane_width_m", "corridor_m"):
        assert key not in d, f"{key} belongs to /lane.json"


def test_lane_json_has_the_arena_geometry(srv):
    server, bridge, port = srv
    bridge.publish(_sample_state(), None)
    status, d = _get_json(port, "/lane.json")
    assert status == 200 and d["ready"] is True
    assert d["scenario"] == "curve"
    assert d["lane"] == [[0.0, 0.0], [0.0, 1.0], [0.5, 2.0]]
    assert d["trail"][1] == [0.1, 0.5, 0.05]
    assert d["lane_width_m"] == 0.5 and d["corridor_m"] == 0.6
    assert d["pose"]["y"] == 1.9 and d["lookahead_m"] == 1.05


def test_endpoints_before_the_first_tick(srv):
    server, bridge, port = srv
    for path in ("/state.json", "/lane.json"):
        status, d = _get_json(port, path)
        assert status == 200 and d["ready"] is False
    status, _ = _get(port, "/frame.jpg")
    assert status == 503


def test_frame_jpeg(srv):
    server, bridge, port = srv
    frame = (np.random.default_rng(0).random((90, 160, 3)) * 255).astype(np.uint8)
    bridge.publish(_sample_state(), frame)
    status, body = _get(port, "/frame.jpg")
    assert status == 200
    assert body[:3] == b"\xff\xd8\xff", "not a JPEG"
    import cv2

    img = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (90, 160)

    status, small = _get(port, "/frame.jpg?w=80")
    assert status == 200 and small[:3] == b"\xff\xd8\xff"
    img = cv2.imdecode(np.frombuffer(small, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[1] == 80 and img.shape[0] < 90


def test_scenarios_endpoint(srv):
    server, bridge, port = srv
    status, d = _get_json(port, "/scenarios")
    assert status == 200
    names = {s["name"] for s in d["scenarios"]}
    assert {"straight", "offset", "curve", "s_curve"} <= names
    assert all(s["description"] for s in d["scenarios"])


def test_index_html_is_served(srv):
    server, bridge, port = srv
    status, body = _get(port, "/")
    assert status == 200
    assert b"<html" in body.lower()


def test_static_traversal_is_rejected(srv):
    server, bridge, port = srv
    for path in ("/static/../server.py", "/static/../../configs/sim.yaml",
                 "/static/nope.js", "/bogus"):
        status, _ = _get(port, path)
        assert status == 404, path


# -------------------------------------------------------------- control ----


def test_post_control_queues_actions(srv):
    server, bridge, port = srv
    for payload, expected in (
        ({"action": "start"}, Control("start", None)),
        ({"action": "emergency"}, Control("emergency", None)),
        ({"action": "reset"}, Control("reset", None)),
        ({"action": "scenario", "scenario": "s_curve"}, Control("scenario", "s_curve")),
    ):
        status, resp = _post(port, "/control", payload)
        assert status == 200 and resp["ok"] is True, payload
        assert bridge.poll_control() == expected
    assert bridge.poll_control() is None


@pytest.mark.parametrize("payload", [
    {}, {"action": "nope"}, {"action": "scenario"}, {"action": "scenario", "scenario": "nope"},
    {"action": "start", "scenario": "curve"}, "start", None, 7, [],
])
def test_post_control_rejects_bad_payloads(srv, payload):
    server, bridge, port = srv
    status, resp = _post(port, "/control", payload)
    assert status == 400 and resp["ok"] is False
    assert resp["error"]
    assert bridge.poll_control() is None


def test_post_to_unknown_path_is_404(srv):
    server, bridge, port = srv
    status, _ = _post(port, "/nope", {"action": "start"})
    assert status == 404


# ------------------------------------------------------- loop integration ----


def test_emergency_then_start_over_http(srv):
    """The competition resume rule, driven from the browser."""
    server, bridge, port = srv
    out: dict = {}

    def worker():
        out["res"] = run_sim("straight", max_steps=250, seed=1, dt=0.02,
                             realtime=True, bridge=bridge)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        # the loop auto-starts at boot (START pushed once), like the real robot
        _wait_for(port, lambda d: d["robot_state"] == "run")

        status, resp = _post(port, "/control", {"action": "emergency"})
        assert status == 200 and resp["ok"]
        _wait_for(port, lambda d: d["robot_state"] == "stopped")
        d1 = _get_json(port, "/state.json")[1]["metrics"]["distance_m"]
        time.sleep(0.3)
        d2 = _get_json(port, "/state.json")[1]["metrics"]["distance_m"]
        assert d2 == d1, "emergency must freeze the robot"

        status, resp = _post(port, "/control", {"action": "start"})
        assert status == 200 and resp["ok"]
        _wait_for(port, lambda d: d["robot_state"] == "run")
        time.sleep(0.3)
        d3 = _get_json(port, "/state.json")[1]["metrics"]["distance_m"]
        assert d3 > d2, "START after emergency must resume the run"
    finally:
        stop(server)
        t.join(timeout=20)
    assert out["res"].distance_m > 0
    assert out["res"].steps > 0


def test_scenario_switch_over_http_waits_for_start(srv):
    server, bridge, port = srv
    out: dict = {}

    def worker():
        out["res"] = run_sim("straight", max_steps=250, seed=1, dt=0.02,
                             realtime=True, bridge=bridge)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        _wait_for(port, lambda d: d["robot_state"] == "run")
        status, resp = _post(port, "/control", {"action": "scenario", "scenario": "curve"})
        assert status == 200 and resp["ok"]
        d = _wait_for(port, lambda d: d["scenario"] == "curve")
        assert d["robot_state"] == "wait_start", "a new scenario waits for START"
        assert d["metrics"]["distance_m"] == 0.0
    finally:
        stop(server)
        t.join(timeout=20)
    assert out["res"].scenario == "curve"


def test_http_bridge_satisfies_the_bridge_protocol():
    b = HttpBridge()
    b.publish(SimState(step=1), None)
    assert b.state().step == 1
    assert b.frame() is None
    assert b.poll_control() is None
    b.push(Control("reset"))
    assert b.poll_control() == Control("reset")
    assert b.poll_control() is None


def test_sim_config_is_read_from_the_repo():
    """Regression: the old path resolved to src/, so configs/sim.yaml was never read."""
    from mav.sim.runner import _sim_config

    cfg = _sim_config()
    assert cfg.get("scenario") == "straight"
    assert "dt" in cfg and "max_steps" in cfg
    assert "dashboard" in cfg

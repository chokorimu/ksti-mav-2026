"""Cross-profile dashboard server tests (dashboard subtask 1).

Runs a REAL server on an ephemeral port and drives it over HTTP with urllib,
plus a hand-rolled minimal WebSocket client (the project adds no test
dependency, so the client is ~30 lines: handshake + one frame parse).

Covers the subtask-1 contract: static frontend, WS handshake + telemetry
broadcast, MJPEG stream (and its 503 before the first frame), the control
endpoints with their refuse-while-RUN rule, and the config hooks being
optional (501 until a later subtask wires them).
"""
from __future__ import annotations

import base64
import json
import os
import socket
import struct
import urllib.error
import urllib.request

import numpy as np
import pytest

from mav.dashboard.bridge import Control, DashboardState, parse_control  # noqa: F401
from mav.dashboard.server import serve_in_thread, stop
from mav.dashboard.state import STATE_LABELS, label_of


# ---------------------------------------------------------------- HTTP helpers
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


def _put(port: int, path: str, text: str) -> tuple[int, str]:
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}", data=text.encode("utf-8"),
        headers={"Content-Type": "text/plain"}, method="PUT",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, r.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode("utf-8")


# ---------------------------------------------------------------- WS helpers
def _ws_connect(port: int) -> tuple[socket.socket, bytes]:
    """Handshake only. Returns (socket, bytes already received after it)."""
    sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    sock.sendall(
        (
            f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n"
            f"Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        ).encode("ascii")
    )
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise AssertionError("connection closed during the WS handshake")
        buf += chunk
    head, rest = buf.split(b"\r\n\r\n", 1)
    assert b" 101 " in head.split(b"\r\n")[0], head
    assert b"Sec-WebSocket-Accept" in head
    return sock, rest


def _ws_read_frame(sock: socket.socket, pre: bytes = b"") -> tuple[int, bytes]:
    """Read one server->client frame; returns (opcode, payload)."""
    buf = pre

    def need(n: int) -> None:
        nonlocal buf
        while len(buf) < n:
            chunk = sock.recv(4096)
            if not chunk:
                raise AssertionError("connection closed mid-frame")
            buf += chunk

    need(2)
    opcode = buf[0] & 0x0F
    length = buf[1] & 0x7F
    idx = 2
    if length == 126:
        need(4)
        length = struct.unpack(">H", buf[2:4])[0]
        idx = 4
    elif length == 127:
        need(10)
        length = struct.unpack(">Q", buf[2:10])[0]
        idx = 10
    need(idx + length)
    return opcode, buf[idx : idx + length]


@pytest.fixture()
def srv():
    server, bridge = serve_in_thread({"host": "127.0.0.1", "port": 0, "profile": "sim"})
    yield server, bridge, server.server_address[1]
    stop(server)


# ---------------------------------------------------------------- static ----
def test_serves_the_frontend(srv):
    _server, _bridge, port = srv
    status, body = _get(port, "/")
    assert status == 200
    text = body.decode("utf-8")
    assert "MAV Control" in text
    # the contract the frontend documents must be the one the server serves
    assert "/stream.mjpg" in text and "/api/config/" in text
    assert "?mock=1" in text


def test_static_traversal_is_rejected(srv):
    _server, _bridge, port = srv
    for path in ("/static/../server.py", "/static/../../configs/sim.yaml"):
        status, _body = _get(port, path)
        assert status == 404, path


def test_healthz(srv):
    _server, _bridge, port = srv
    status, d = _get_json(port, "/healthz")
    assert status == 200 and d == {"ok": True, "profile": "sim"}


def test_unknown_path_is_404(srv):
    _server, _bridge, port = srv
    status, d = _get_json(port, "/nope")
    assert status == 404 and d["ok"] is False


# ---------------------------------------------------------------- websocket ----
def test_ws_handshake_and_boot_telemetry(srv):
    _server, bridge, port = srv
    sock, pre = _ws_connect(port)
    try:
        opcode, payload = _ws_read_frame(sock, pre)
        assert opcode == 0x1  # text frame
        d = json.loads(payload)
        assert d["state"] == "BOOT" and d["profile"] == "sim"
        assert d["recording"] == {"on": False, "frames": 0}
        assert d["log"] == []
        assert d["checkpoints"] == 0
    finally:
        sock.close()


def test_ws_broadcasts_published_state(srv):
    _server, bridge, port = srv
    bridge.publish(
        DashboardState(
            state="RUN", profile="prototype", checkpoints=3, violations=1,
            fps=28.5, latency_ms=12.25, cmd={"speed": 0.4, "curvature": 0.8},
            out={"left_mps": 0.41, "right_mps": 0.39},
            extra={"detected": 11},
        ),
        None,
    )
    sock, pre = _ws_connect(port)
    try:
        _opcode, payload = _ws_read_frame(sock, pre)
        d = json.loads(payload)
        assert d["state"] == "RUN" and d["checkpoints"] == 3
        assert d["violations"] == 1 and d["fps"] == 28.5
        assert d["cmd"] == {"speed": 0.4, "curvature": 0.8}
        assert d["out"]["left_mps"] == 0.41
        assert d["extra"]["detected"] == 11
    finally:
        sock.close()


def test_ws_pushes_at_about_10hz(srv):
    _server, bridge, port = srv
    sock, pre = _ws_connect(port)
    try:
        stamps = []
        for _ in range(3):
            _ws_read_frame(sock, pre)
            pre = b""
            stamps.append(__import__("time").monotonic())
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        assert all(0.05 <= g <= 0.5 for g in gaps), gaps
    finally:
        sock.close()


# ---------------------------------------------------------------- mjpeg ----
def test_mjpeg_503_before_the_first_frame(srv):
    _server, _bridge, port = srv
    status, d = _get_json(port, "/stream.mjpg")
    assert status == 503 and "no frame" in d["error"]


def test_mjpeg_streams_multipart(srv):
    _server, bridge, port = srv
    bridge.publish(DashboardState(), np.zeros((24, 32, 3), dtype=np.uint8))
    sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    try:
        sock.sendall(b"GET /stream.mjpg HTTP/1.1\r\nHost: x\r\n\r\n")
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += sock.recv(4096)
        head, rest = buf.split(b"\r\n\r\n", 1)
        assert b"200" in head.split(b"\r\n")[0]
        assert b"multipart/x-mixed-replace; boundary=frame" in head
        while len(rest) < 4096:
            chunk = sock.recv(4096)
            if not chunk:
                break
            rest += chunk
        assert b"--frame\r\n" in rest
        assert b"Content-Type: image/jpeg" in rest
        assert b"\xff\xd8" in rest  # JPEG SOI
    finally:
        sock.close()


# ---------------------------------------------------------------- controls ----
def test_estop_pushes_a_control(srv):
    _server, bridge, port = srv
    status, d = _post(port, "/api/estop", {})
    assert status == 200 and d == {"ok": True, "action": "emergency"}
    assert bridge.poll_control() == Control(action="emergency")


def test_start_pushes_a_control(srv):
    _server, bridge, port = srv
    status, d = _post(port, "/api/start", {})
    assert status == 200 and d["action"] == "start"
    assert bridge.poll_control() == Control(action="start")


def test_record_pushes_session_and_flag(srv):
    _server, bridge, port = srv
    status, d = _post(port, "/api/record", {"on": True, "session": "white-bright-01"})
    assert status == 200
    ctrl = bridge.poll_control()
    assert ctrl.action == "record" and ctrl.on is True
    assert ctrl.session == "white-bright-01"


def test_reset_and_profile_are_refused_while_run(srv):
    _server, bridge, port = srv
    bridge.publish(DashboardState(state="RUN"), None)
    assert bridge.running() is True
    status, d = _post(port, "/api/reset", {})
    assert status == 409 and "RUN" in d["error"]
    assert bridge.poll_control() is None
    status, d = _post(port, "/api/profile", {"profile": "final"})
    assert status == 409
    assert bridge.poll_control() is None
    # the software estop must stay reachable at all times
    status, _d = _post(port, "/api/estop", {})
    assert status == 200


def test_reset_and_profile_work_while_stopped(srv):
    _server, bridge, port = srv
    bridge.publish(DashboardState(state="STOPPED"), None)
    status, _d = _post(port, "/api/reset", {})
    assert status == 200
    assert bridge.poll_control() == Control(action="reset")


def test_profile_without_hook_is_501(srv):
    _server, bridge, port = srv
    bridge.publish(DashboardState(state="WAIT_START"), None)
    status, d = _post(port, "/api/profile", {"profile": "sim"})
    assert status == 501
    assert bridge.poll_control() is None  # nothing pushed: the hook owns it


def test_profile_hook_restarts_the_pipeline(srv):
    server, bridge, port = srv
    calls = []

    def hook(profile: str) -> str:
        calls.append(profile)
        return f"switched to {profile}"

    server.profile_hook = hook  # type: ignore[attr-defined]
    bridge.publish(DashboardState(state="WAIT_START"), None)
    status, d = _post(port, "/api/profile", {"profile": "final"})
    assert status == 200 and d["message"] == "switched to final"
    assert calls == ["final"]


def test_profile_hook_rejection_is_400(srv):
    server, _bridge, port = srv

    def hook(profile: str) -> str:
        raise ValueError("camera source is not configured")

    server.profile_hook = hook  # type: ignore[attr-defined]
    status, d = _post(port, "/api/profile", {"profile": "prototype"})
    assert status == 400 and "camera" in d["error"]


def test_bad_payloads_are_400(srv):
    _server, _bridge, port = srv
    data = b"{not json"
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/start", data=data,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            status, body = r.status, r.read()
    except urllib.error.HTTPError as exc:
        status, body = exc.code, exc.read()
    assert status == 400
    status, d = _post(port, "/api/record", {"on": "yes"})
    assert status == 200, d  # parse_control coerces truthy strings to bool
    status, d = _post(port, "/api/bogus", {})
    assert status == 404


# ---------------------------------------------------------------- config ----
def test_config_endpoints_are_501_until_wired(srv):
    _server, _bridge, port = srv
    status, d = _get_json(port, "/api/config/sim")
    assert status == 501
    status, body = _put(port, "/api/config/sim", "profile: sim\n")
    assert status == 501


def test_config_unknown_name_is_404(srv):
    _server, _bridge, port = srv
    status, _d = _get_json(port, "/api/config/nope")
    assert status == 404


def test_config_hook_reads_and_writes(srv):
    server, bridge, port = srv
    written = {}

    def hook(name: str, body: str | None = None) -> str:
        if body is None:
            return f"# {name}.yaml\nprofile: {name}\n"
        written[name] = body
        return f"saved {name}.yaml"

    server.config_hook = hook  # type: ignore[attr-defined]
    status, text = _get(port, "/api/config/final")
    assert status == 200 and "profile: final" in text.decode("utf-8")
    status, text = _put(port, "/api/config/final", "profile: final\nnetwork: {enabled: false}\n")
    assert status == 200 and text == "saved final.yaml"
    assert written["final"].startswith("profile: final")


def test_config_put_is_refused_while_run(srv):
    server, bridge, port = srv
    server.config_hook = lambda name, body=None: "ok"  # type: ignore[attr-defined]
    bridge.publish(DashboardState(state="RUN"), None)
    status, body = _put(port, "/api/config/sim", "profile: sim\n")
    assert status == 409
    # reading stays allowed: it cannot affect the run
    status, _body = _get(port, "/api/config/sim")
    assert status == 200


# ---------------------------------------------------------------- units ----
def test_label_of_maps_state_machine_values():
    from mav.control.state_machine import State

    assert label_of(State.RUN) == "RUN"
    assert label_of(State.WAIT_START) == "WAIT_START"
    assert label_of("stopped") == "STOPPED"
    assert label_of("weird_new_state") == "WEIRD_NEW_STATE"
    assert STATE_LABELS == ("BOOT", "WAIT_START", "RUN", "STOPPED")


def test_parse_control_validation():
    assert parse_control({"action": "start"}).action == "start"
    assert parse_control({"action": "profile", "profile": "final"}).profile == "final"
    rec = parse_control({"action": "record", "on": True, "session": "s1"})
    assert rec.on is True and rec.session == "s1"
    for bad in ({"action": "nope"}, {"action": "profile"}, {"action": "profile", "profile": "x"},
                {"action": "record", "on": True, "session": 5}, "notadict"):
        with pytest.raises(ValueError):
            parse_control(bad)


def test_state_to_dict_is_json_safe():
    st = DashboardState(
        state="RUN", profile="sim", fps=float("nan"), temp_c=float("inf"),
        cmd={"speed": np.float64(0.5)}, out={"n": np.int64(3)},
    )
    d = st.to_dict()
    json.dumps(d)  # must not raise
    assert d["fps"] is None and d["temp_c"] is None
    assert d["cmd"]["speed"] == 0.5 and d["out"]["n"] == 3

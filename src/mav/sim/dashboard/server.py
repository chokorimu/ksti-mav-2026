"""Stdlib HTTP server for the sim dashboard (sim profile only).

Endpoints:
  GET  /             static/index.html
  GET  /static/...   vendored frontend (three.module.js, app.js, style.css)
  GET  /state.json   telemetry — small, polled fast
  GET  /lane.json    arena geometry (centreline polyline + trail) — polled slower
  GET  /frame.jpg    newest camera frame, optional ?w=<width> to downscale
  GET  /scenarios    scenario list for the dropdown
  POST /control      {"action": "start"|"emergency"|"scenario"|"reset",
                      "scenario": "<name>"}   (scenario only with action=scenario)

No third-party dependency: `http.server` from the standard library. The server
only moves data between the browser and the running sim loop — it computes
nothing, so the CV and control code stays identical to the real robot. Only the
sim profile ever starts it (configs/sim.yaml `dashboard:`); final has no such
section and stays serverless.
"""
from __future__ import annotations

import json
import os
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

from mav.sim.dashboard.bridge import Control, parse_control
from mav.sim.dashboard.state import SimState
from mav.sim.scenarios import SCENARIOS

STATIC_DIR = Path(__file__).resolve().parent / "static"

_MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".jpg": "image/jpeg",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}

# Bulky / slow-changing fields live in /lane.json, not in the fast telemetry poll.
_GEOMETRY_KEYS = ("lane", "trail", "lane_width_m", "corridor_m")


class HttpBridge:
    """Bridge that serves the newest snapshot over HTTP and queues controls.

    Thread-safe: the sim loop calls publish()/poll_control(), the HTTP threads
    call state()/frame()/push(). A lock guards the snapshot; the control queue
    is a queue.Queue, which is already thread-safe.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: SimState | None = None
        self._frame: np.ndarray | None = None
        self._controls: queue.Queue = queue.Queue()

    # -- Bridge protocol (called by the sim loop) -------------------------
    def publish(self, state: SimState, frame: np.ndarray | None) -> None:
        with self._lock:
            self._state = state
            self._frame = frame

    def poll_control(self) -> Control | None:
        try:
            return self._controls.get_nowait()
        except queue.Empty:
            return None

    # -- called by the HTTP threads ---------------------------------------
    def push(self, control: Control) -> None:
        self._controls.put(control)

    def state(self) -> SimState | None:
        with self._lock:
            return self._state

    def frame(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    # -- views -------------------------------------------------------------
    def telemetry(self) -> dict:
        """Everything except the arena geometry (that is /lane.json's job)."""
        st = self.state()
        if st is None:
            return {"ready": False}
        d = st.to_dict()
        return {"ready": True, **{k: v for k, v in d.items() if k not in _GEOMETRY_KEYS}}

    def geometry(self) -> dict:
        st = self.state()
        if st is None:
            return {"ready": False, "lane": [], "trail": []}
        d = st.to_dict()
        return {
            "ready": True,
            "scenario": d["scenario"],
            "lane": d["lane"],
            "trail": d["trail"],
            "lane_width_m": d["lane_width_m"],
            "corridor_m": d["corridor_m"],
            "pose": d["pose"],
            "lookahead_m": d["lookahead_m"],
        }


def _encode_jpeg(frame: np.ndarray, width: int | None) -> bytes | None:
    """JPEG bytes for a BGR frame, optionally downscaled. None if cv2 is absent."""
    try:
        import cv2
    except ImportError:  # pragma: no cover - cv2 is a project dependency
        return None
    img = frame
    if width and 0 < width < img.shape[1]:
        h = max(1, int(round(img.shape[0] * width / img.shape[1])))
        img = cv2.resize(img, (int(width), h), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    return buf.tobytes() if ok else None


class DashboardHandler(BaseHTTPRequestHandler):
    """Serves the dashboard. The bridge arrives via `server.bridge`."""

    protocol_version = "HTTP/1.1"
    server_version = "MavDashboard/1"

    # -- helpers -----------------------------------------------------------
    @property
    def bridge(self) -> HttpBridge:
        return self.server.bridge  # type: ignore[attr-defined]

    def _send(self, status: int, body: bytes, ctype: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, obj: Any, status: int = 200) -> None:
        self._send(status, json.dumps(obj).encode("utf-8"), "application/json")

    def _not_found(self, msg: str = "not found") -> None:
        self._json({"ok": False, "error": msg}, 404)

    def log_message(self, fmt: str, *args) -> None:  # keep the console readable
        pass

    # -- GET ---------------------------------------------------------------
    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._static_file("index.html")
        elif path == "/state.json":
            self._json(self.bridge.telemetry())
        elif path == "/lane.json":
            self._json(self.bridge.geometry())
        elif path == "/frame.jpg":
            self._frame_jpeg()
        elif path == "/scenarios":
            self._json(
                {
                    "scenarios": [
                        {"name": s.name, "description": s.description}
                        for s in SCENARIOS.values()
                    ]
                }
            )
        elif path.startswith("/static/"):
            self._static_file(path[len("/static/"):])
        else:
            self._not_found(path)

    def do_HEAD(self) -> None:
        self.do_GET()

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        if path != "/control":
            self._not_found(path)
            return
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        raw = self.rfile.read(length) if length > 0 else b""
        try:
            payload = json.loads(raw.decode("utf-8")) if raw else None
            control = parse_control(payload)
        except (ValueError, UnicodeDecodeError) as exc:
            self._json({"ok": False, "error": str(exc)}, 400)
            return
        self.bridge.push(control)
        self._json({"ok": True, "action": control.action,
                    "scenario": control.scenario})

    # -- pieces ------------------------------------------------------------
    def _frame_jpeg(self) -> None:
        frame = self.bridge.frame()
        if frame is None:
            self._json({"ok": False, "error": "no frame yet"}, 503)
            return
        width = None
        qs = self.path.split("?", 1)[1] if "?" in self.path else ""
        for part in qs.split("&"):
            if part.startswith("w="):
                try:
                    width = int(part[2:])
                except ValueError:
                    width = None
        body = _encode_jpeg(frame, width)
        if body is None:
            self._json({"ok": False, "error": "jpeg encode failed"}, 500)
            return
        self._send(200, body, "image/jpeg")

    def _static_file(self, rel: str) -> None:
        try:
            base = STATIC_DIR.resolve()
            target = (base / rel).resolve()
            target.relative_to(base)  # rejects ../ traversal
        except (ValueError, OSError):
            self._not_found(rel)
            return
        if not target.is_file():
            self._not_found(rel)
            return
        body = target.read_bytes()
        self._send(200, body, _MIME.get(target.suffix.lower(), "application/octet-stream"))


def serve_in_thread(cfg: dict | None = None) -> tuple[ThreadingHTTPServer, HttpBridge]:
    """Start the dashboard in a daemon thread. Returns (server, bridge).

    cfg keys (all optional): host (default 127.0.0.1), port (default 8000).
    """
    cfg = cfg or {}
    host = str(cfg.get("host", "127.0.0.1"))
    port = int(cfg.get("port", 8000))
    bridge = HttpBridge()
    server = ThreadingHTTPServer((host, port), DashboardHandler)
    server.daemon_threads = True
    server.bridge = bridge  # type: ignore[attr-defined]
    thread = threading.Thread(target=server.serve_forever, name="mav-dashboard", daemon=True)
    thread.start()
    return server, bridge


def stop(server: ThreadingHTTPServer) -> None:
    """Shut the server down and close the listening socket."""
    server.shutdown()
    server.server_close()


__all__ = ["HttpBridge", "DashboardHandler", "serve_in_thread", "stop", "STATIC_DIR"]

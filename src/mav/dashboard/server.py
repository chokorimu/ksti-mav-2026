"""Stdlib HTTP + WebSocket server for the cross-profile dashboard.

Endpoints (the frontend contract lives in ``static/index.html``):

  GET  /                     static/index.html
  GET  /stream.mjpg          MJPEG camera stream (?overlay=1 -> debug overlay)
  GET  /ws                   WebSocket telemetry, ~10 Hz (one DashboardState per tick)
  POST /api/start            push START (the state machine gates WAIT_START/STOPPED)
  POST /api/estop            push EMERGENCY (always allowed; software stop only)
  POST /api/reset            new_attempt; refused while RUN
  POST /api/profile          switch profile; refused while RUN
  POST /api/record           start/stop dataset recording
  GET  /api/config/{name}    YAML text of configs/<name>.yaml
  PUT  /api/config/{name}    validate + write; refused while RUN

The server only moves data, exactly like the sim dashboard: the bridge holds
the newest snapshot and frame, the run loop drains controls. Telemetry is
broadcast by ONE sender thread so a slow browser cannot stall the loop, and
the WebSocket is hand-rolled (handshake + text frames, ~100 lines) so the
project keeps its "stdlib only" rule.
"""
from __future__ import annotations

import base64
import hashlib
import json
import queue
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

from mav.dashboard.bridge import Control, parse_control
from mav.dashboard.state import DashboardState

STATIC_DIR = Path(__file__).resolve().parent / "static"

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"  # RFC 6455 fixed GUID
_WS_INTERVAL_S = 0.1          # ~10 Hz telemetry push
_WS_IDLE_TIMEOUT_S = 30.0     # drop a silent peer after this (bounded thread)
_MJPEG_INTERVAL_S = 1.0 / 15  # stream cap; the loop usually runs faster

_MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".jpg": "image/jpeg",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}

_CONFIG_NAMES = ("sim", "prototype", "final", "vehicle")


# --------------------------------------------------------------------------
# WebSocket frame helpers (RFC 6455, the subset this dashboard needs)
# --------------------------------------------------------------------------

def _ws_accept(key: str) -> str:
    """Sec-WebSocket-Accept value for a client handshake key."""
    digest = hashlib.sha1((key + _WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def _ws_frame(message: str) -> bytes:
    """Encode one unmasked server->client TEXT frame."""
    data = message.encode("utf-8")
    header = bytearray([0x81])  # FIN + opcode 1 (text)
    n = len(data)
    if n < 126:
        header.append(n)
    elif n < 65536:
        header.append(126)
        header += struct.pack(">H", n)
    else:
        header.append(127)
        header += struct.pack(">Q", n)
    return bytes(header) + data


def _ws_opcodes(data: bytes) -> list[int]:
    """Opcodes of the complete client frames in a buffer.

    Client frames are always masked; we only care about close (0x8) and
    ping (0x9), so the payload is skipped, not decoded.
    """
    opcodes: list[int] = []
    i = 0
    while i + 2 <= len(data):
        opcode = data[i] & 0x0F
        length = data[i + 1] & 0x7F
        j = i + 2
        if length == 126:
            if j + 2 > len(data):
                break
            length = struct.unpack(">H", data[j : j + 2])[0]
            j += 2
        elif length == 127:
            if j + 8 > len(data):
                break
            length = struct.unpack(">Q", data[j : j + 8])[0]
            j += 8
        j += 4  # masking key
        j += length
        if j > len(data):
            break  # truncated frame: wait for more bytes
        opcodes.append(opcode)
        i = j
    return opcodes


def _ws_pong() -> bytes:
    """Empty PONG frame. Browsers never ping a server; this only answers
    intermediaries that do, so echoing the payload is not worth the parser."""
    return b"\x8a\x00"


# --------------------------------------------------------------------------
# Bridge: newest snapshot + frame, control queue, WS client registry
# --------------------------------------------------------------------------

class DashboardBridge:
    """Holds the newest snapshot/frame, queues controls, tracks WS clients.

    Thread-safe: the run loop calls ``publish()``/``poll_control()``, the HTTP
    threads call ``state()``/``frame()``/``push()``, and the sender thread
    broadcasts to the client registry.
    """

    def __init__(self, profile: str = "prototype") -> None:
        self.profile = profile  # reported while no run loop has published yet
        self._lock = threading.Lock()
        self._state: DashboardState | None = None
        self._frame: np.ndarray | None = None
        self._controls: queue.Queue = queue.Queue()
        self._clients: list[socket.socket] = []
        self._client_lock = threading.Lock()

    # -- run-loop side --------------------------------------------------------
    def publish(self, state: DashboardState, frame: np.ndarray | None) -> None:
        with self._lock:
            self._state = state
            self._frame = frame

    def poll_control(self) -> Control | None:
        try:
            return self._controls.get_nowait()
        except queue.Empty:
            return None

    # -- HTTP-thread side ------------------------------------------------------
    def push(self, control: Control) -> None:
        self._controls.put(control)

    def state(self) -> DashboardState | None:
        with self._lock:
            return self._state

    def frame(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    def running(self) -> bool:
        """True while a run is in progress. Gates the refuse-while-RUN rules."""
        st = self.state()
        return st is not None and st.state == "RUN"

    # -- WebSocket clients ------------------------------------------------------
    def add_client(self, sock: socket.socket) -> None:
        with self._client_lock:
            self._clients.append(sock)

    def remove_client(self, sock: socket.socket) -> None:
        with self._client_lock:
            if sock in self._clients:
                self._clients.remove(sock)

    def has_clients(self) -> bool:
        with self._client_lock:
            return bool(self._clients)

    def broadcast(self, message: str) -> None:
        """Send one text frame to every client; drop the ones that fail."""
        data = _ws_frame(message)
        with self._client_lock:
            dead: list[socket.socket] = []
            for sock in self._clients:
                try:
                    sock.sendall(data)
                except OSError:
                    dead.append(sock)
            for sock in dead:
                if sock in self._clients:
                    self._clients.remove(sock)


def _compose(bridge: DashboardBridge) -> str:
    """The WS payload: the newest state, or a BOOT placeholder before the
    first tick (so the browser flips to "live" even with no run loop yet)."""
    st = bridge.state()
    if st is None:
        st = DashboardState(state="BOOT", profile=bridge.profile)
    return json.dumps(st.to_dict())


def _push_loop(bridge: DashboardBridge, stop_event: threading.Event) -> None:
    while not stop_event.wait(_WS_INTERVAL_S):
        if bridge.has_clients():
            bridge.broadcast(_compose(bridge))


# --------------------------------------------------------------------------
# HTTP handler
# --------------------------------------------------------------------------

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
    """Serves the dashboard. The bridge and hooks arrive via `server`."""

    protocol_version = "HTTP/1.1"
    server_version = "MavDashboard/2"

    # -- helpers ---------------------------------------------------------------
    @property
    def bridge(self) -> DashboardBridge:
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

    def _text(self, text: str, status: int = 200) -> None:
        self._send(status, text.encode("utf-8"), "text/plain; charset=utf-8")

    def _not_found(self, msg: str = "not found") -> None:
        self._json({"ok": False, "error": msg}, 404)

    def _refused(self, msg: str) -> None:
        self._json({"ok": False, "error": msg}, 409)

    def log_message(self, fmt: str, *args) -> None:  # keep the console readable
        pass

    # -- GET ---------------------------------------------------------------------
    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._static_file("index.html")
        elif path == "/healthz":
            self._json({"ok": True, "profile": self.bridge.profile})
        elif path == "/stream.mjpg":
            self._mjpeg()
        elif path == "/ws":
            self._websocket()
        elif path.startswith("/api/config/"):
            self._config_get(path[len("/api/config/") :])
        elif path.startswith("/static/"):
            self._static_file(path[len("/static/") :])
        else:
            self._not_found(path)

    def do_HEAD(self) -> None:
        self.do_GET()

    # -- POST / PUT -----------------------------------------------------------------
    def _payload(self) -> tuple[dict | None, str | None]:
        """Read + parse the JSON body. Returns (payload, error)."""
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        raw = self.rfile.read(length) if length > 0 else b""
        if not raw:
            return {}, None
        try:
            data = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as exc:
            return None, str(exc)
        if not isinstance(data, dict):
            return None, "payload must be a JSON object"
        return data, None

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        if path not in ("/api/start", "/api/estop", "/api/reset",
                        "/api/profile", "/api/record"):
            self._not_found(path)
            return
        payload, err = self._payload()
        if err is not None:
            self._json({"ok": False, "error": err}, 400)
            return
        # URL name -> control action ("estop" is the friendlier URL spelling).
        action = {"estop": "emergency"}.get(path[len("/api/") :], path[len("/api/") :])

        if action in ("reset", "profile") and self.bridge.running():
            # Reference contract: both restart the pipeline, so neither may
            # fire mid-run. The physical e-stop stays the way to stop a run.
            self._refused(f"{action} refused while RUN")
            return

        try:
            control = parse_control({"action": action, **(payload or {})})
        except ValueError as exc:
            self._json({"ok": False, "error": str(exc)}, 400)
            return

        if action == "profile":
            hook = getattr(self.server, "profile_hook", None)
            if hook is None:
                self._json({"ok": False,
                            "error": "profile switching is not wired for this run"}, 501)
                return
            try:
                message = hook(control.profile or "")
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, 400)
                return
            self._json({"ok": True, "action": action, "message": message})
            return

        self.bridge.push(control)
        self._json({"ok": True, "action": action})

    def do_PUT(self) -> None:
        path = self.path.split("?", 1)[0]
        if not path.startswith("/api/config/"):
            self._not_found(path)
            return
        name = path[len("/api/config/") :]
        if name not in _CONFIG_NAMES:
            self._json({"ok": False, "error": f"unknown config {name!r}"}, 404)
            return
        if self.bridge.running():
            self._refused("config edits are locked while RUN")
            return
        hook = getattr(self.server, "config_hook", None)
        if hook is None:
            self._json({"ok": False, "error": "config editing is not wired yet"}, 501)
            return
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        raw = self.rfile.read(length) if length > 0 else b""
        try:
            message = hook(name, raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as exc:
            self._json({"ok": False, "error": str(exc)}, 400)
            return
        self._text(message)

    # -- pieces ----------------------------------------------------------------------
    def _config_get(self, name: str) -> None:
        if name not in _CONFIG_NAMES:
            self._json({"ok": False, "error": f"unknown config {name!r}"}, 404)
            return
        hook = getattr(self.server, "config_hook", None)
        if hook is None:
            self._json({"ok": False, "error": "config editing is not wired yet"}, 501)
            return
        try:
            text = hook(name)  # read mode: no body
        except (ValueError, OSError) as exc:
            self._json({"ok": False, "error": str(exc)}, 400)
            return
        self._text(text)

    def _mjpeg(self) -> None:
        """multipart/x-mixed-replace stream of the newest frame."""
        # Wait for the first frame BEFORE the headers: once the multipart
        # response starts there is no way back to a 503.
        deadline = time.monotonic() + 2.0
        while self.bridge.frame() is None:
            if time.monotonic() > deadline:
                self._json({"ok": False, "error": "no frame yet"}, 503)
                return
            time.sleep(0.05)
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-store, private, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        self.close_connection = True  # this connection never returns to HTTP
        try:
            while True:
                frame = self.bridge.frame()
                if frame is not None:
                    body = _encode_jpeg(frame, None)
                    if body:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(body)).encode("ascii")
                            + b"\r\n\r\n"
                            + body
                            + b"\r\n"
                        )
                        self.wfile.flush()
                time.sleep(_MJPEG_INTERVAL_S)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # browser navigated away / closed the stream

    def _websocket(self) -> None:
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self._json({"ok": False, "error": "missing Sec-WebSocket-Key"}, 400)
            return
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", _ws_accept(key))
        self.end_headers()
        self.close_connection = True  # the socket now speaks WS, not HTTP
        sock = self.connection
        sock.settimeout(_WS_IDLE_TIMEOUT_S)
        self.bridge.add_client(sock)
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break  # peer closed
                if 0x8 in _ws_opcodes(data):
                    break  # close frame
                if 0x9 in _ws_opcodes(data):
                    sock.sendall(_ws_pong())
        except OSError:
            pass  # timeout, reset, or the sender dropped us
        finally:
            self.bridge.remove_client(sock)

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


# --------------------------------------------------------------------------
# Server lifecycle
# --------------------------------------------------------------------------

def serve_in_thread(cfg: dict | None = None) -> tuple[ThreadingHTTPServer, DashboardBridge]:
    """Start the dashboard in daemon threads. Returns (server, bridge).

    cfg keys (all optional): host (default 127.0.0.1), port (default 8000),
    profile (the profile to report before the first tick).
    """
    cfg = cfg or {}
    host = str(cfg.get("host", "127.0.0.1"))
    port = int(cfg.get("port", 8000))
    bridge = DashboardBridge(profile=str(cfg.get("profile", "prototype")))
    server = ThreadingHTTPServer((host, port), DashboardHandler)
    server.daemon_threads = True
    server.bridge = bridge  # type: ignore[attr-defined]
    server.profile_hook = None  # type: ignore[attr-defined]  # set by the run loop
    server.config_hook = None  # type: ignore[attr-defined]   # set by the run loop
    server.stop_event = threading.Event()  # type: ignore[attr-defined]
    threading.Thread(
        target=server.serve_forever, name="mav-dashboard-http", daemon=True
    ).start()
    threading.Thread(
        target=_push_loop, args=(bridge, server.stop_event),  # type: ignore[attr-defined]
        name="mav-dashboard-ws",
        daemon=True,
    ).start()
    return server, bridge


def stop(server: ThreadingHTTPServer) -> None:
    """Stop the pusher, shut the listener down and close its socket."""
    event = getattr(server, "stop_event", None)
    if event is not None:
        event.set()
    server.shutdown()
    server.server_close()


__all__ = [
    "DashboardBridge",
    "DashboardHandler",
    "serve_in_thread",
    "stop",
    "STATIC_DIR",
]

"""Web dashboard for the synthetic simulator.

Pure data + plumbing: the simulator loop publishes a `SimState` (and the camera
frame) every tick through a `Bridge`, and drains `Control` actions back. The
browser is a viewer plus a control channel — it never computes anything, so the
CV and control code stays identical to the real robot.

Modules:
  state.py   SimState -> JSON-safe dict (the browser contract, in one place)
  bridge.py  Bridge protocol, NullBridge (default), FakeBridge (tests)
  server.py  stdlib HTTP server (subtask 2, not written yet)
"""

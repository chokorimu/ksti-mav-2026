import tempfile
from pathlib import Path

from mav.io.camera import CallbackCamera, csi_pipeline
from mav.types import load_vehicle_params


def test_csi_pipeline_is_headless_and_ends_in_appsink():
    p = csi_pipeline(sensor_id=1, capture_size=(1280, 720), output_size=(640, 360), fps=30)
    assert "nvarguscamerasrc sensor-id=1" in p
    assert "width=640, height=360" in p
    assert p.rstrip().endswith("appsink drop=1 max-buffers=1")
    assert "nvoverlaysink" not in p and "nveglglessink" not in p


def test_callback_camera_returns_latest():
    frames = iter([1, 2, None])
    cam = CallbackCamera(lambda: next(frames))
    assert [cam.read(), cam.read(), cam.read()] == [1, 2, None]


def _write(text):
    d = tempfile.mkdtemp()
    p = Path(d) / "vehicle.yaml"
    p.write_text(text)
    return p


def test_vehicle_params_refuses_unmeasured_values():
    p = _write("drive: null\nwheel_base_mm: null\ntrack_width_mm: null\nmax_steer_deg: null\nmax_speed_mps: null\n")
    try:
        load_vehicle_params(p)
        raised = False
    except ValueError as e:
        raised = "wheel_base_mm" in str(e)
    assert raised


def test_vehicle_params_converts_units():
    p = _write("drive: ackermann\nwheel_base_mm: 300\ntrack_width_mm: 200\nmax_steer_deg: 30\nmax_speed_mps: 1.5\n")
    v = load_vehicle_params(p)
    assert v.wheel_base_m == 0.3 and v.track_width_m == 0.2 and v.max_speed_mps == 1.5
    assert abs(v.max_steer_rad - 0.5235987755982988) < 1e-9


def test_vehicle_params_rejects_unknown_drive():
    p = _write("drive: omni\nwheel_base_mm: 300\ntrack_width_mm: 200\nmax_steer_deg: 30\nmax_speed_mps: 1\n")
    try:
        load_vehicle_params(p)
        raised = False
    except ValueError:
        raised = True
    assert raised

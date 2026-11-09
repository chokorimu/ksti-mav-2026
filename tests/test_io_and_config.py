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


# --------------------------------------------------------- recording a clip --
def test_recording_writes_the_frames_the_detector_sees(tmp_path):
    """`--record` must capture the DETECTOR's input, not a second pipeline.

    lane.px_per_m is pixels per metre at a specific frame size, so a recording
    made through a different pipeline (gst-launch, a phone) can differ in size or
    colour conversion and silently mis-scale every offset. Wrapping the camera and
    writing read()'s return value makes that impossible.
    """
    import numpy as np

    from mav.io.camera import RecordingCamera

    class Cam:
        def __init__(self):
            self.n = 4
            self.closed = False

        def read(self):
            if self.n <= 0:
                return None
            self.n -= 1
            return np.full((360, 640, 3), self.n * 40, dtype=np.uint8)

        def close(self):
            self.closed = True

    inner = Cam()
    logs = []
    cam = RecordingCamera(inner, str(tmp_path / "clip.mp4"), fps=20, log=logs.append)
    seen = 0
    while cam.read() is not None:
        seen += 1
    cam.close()

    assert seen == 4
    assert cam.frames == 4
    assert cam.size == (640, 360)
    assert inner.closed, "the wrapped camera must still be closed"
    assert any("4 frames at 640x360" in line for line in logs)


def test_recording_is_readable_back(tmp_path):
    """The clip must be a real video file, or calibration has nothing to read."""
    import cv2
    import numpy as np

    from mav.io.camera import RecordingCamera

    class Cam:
        def __init__(self):
            self.n = 3

        def read(self):
            if self.n <= 0:
                return None
            self.n -= 1
            return np.full((360, 640, 3), 90, dtype=np.uint8)

        def close(self):
            pass

    cam = RecordingCamera(Cam(), str(tmp_path / "clip.mp4"), fps=20, log=lambda _s: None)
    while cam.read() is not None:
        pass
    cam.close()

    cap = cv2.VideoCapture(cam.path)
    assert cap.isOpened()
    ok, frame = cap.read()
    cap.release()
    assert ok
    assert frame.shape[:2] == (360, 640), "recorded size must survive the round trip"


def test_recording_size_is_taken_from_the_first_frame():
    """The writer cannot be opened before a frame exists - the size is unknown."""
    import numpy as np

    from mav.io.camera import RecordingCamera

    class Cam:
        def read(self):
            return np.zeros((240, 320, 3), dtype=np.uint8)

        def close(self):
            pass

    cam = RecordingCamera(Cam(), "/tmp/opencode/size_probe.mp4", log=lambda _s: None)
    assert cam.size is None, "no size before the first frame"
    cam.read()
    assert cam.size == (320, 240)
    cam.close()


def test_calibrate_scale_reads_the_robot_width_from_the_pipeline():
    """The width px_per_m is applied at must follow the pipeline, not a literal.

    Confirmed on the robot (10 Nov 2026): the CSI pipeline outputs 640x360 while
    Argus logs 1280x720 (the sensor side). Hardcoding 640 in two places would let
    the two drift apart.
    """
    from mav.io.camera import csi_pipeline

    pipeline = csi_pipeline()
    width = int(pipeline.split("width=")[-1].split(",")[0])
    assert width == 640, (
        "the robot camera frame width changed; px_per_m must be re-measured at the "
        "new size"
    )

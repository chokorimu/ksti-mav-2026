"""Camera interface. Same `Camera` protocol for real, replay and simulator sources."""
from __future__ import annotations

from typing import Any, Callable, Optional, Protocol

Frame = Any  # numpy array, BGR, uint8, HxWx3


class Camera(Protocol):
    def read(self) -> Optional[Frame]:
        """Return the newest frame, or None if none is available."""

    def close(self) -> None: ...


def csi_pipeline(
    sensor_id: int = 0,
    capture_size: tuple[int, int] = (1280, 720),
    output_size: tuple[int, int] = (640, 360),
    fps: int = 30,
    flip_method: int = 0,
) -> str:
    """GStreamer pipeline string for a CSI camera on Jetson (headless safe, ends in appsink)."""
    cw, ch = capture_size
    ow, oh = output_size
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={cw}, height={ch}, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={ow}, height={oh}, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=1 max-buffers=1"
    )


class OpenCVCamera:
    """Wraps cv2.VideoCapture. cv2 is imported lazily so tests run without it."""

    def __init__(self, source: Any, use_gstreamer: bool = False) -> None:
        import cv2

        self._cap = cv2.VideoCapture(source, cv2.CAP_GSTREAMER) if use_gstreamer else cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera source: {source!r}")

    def read(self) -> Optional[Frame]:
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        self._cap.release()


def open_csi(**kwargs: Any) -> OpenCVCamera:
    return OpenCVCamera(csi_pipeline(**kwargs), use_gstreamer=True)


def open_usb(index: int = 0) -> OpenCVCamera:
    return OpenCVCamera(index)


def open_video(path: str) -> OpenCVCamera:
    """Replay a recorded video through the same pipeline. read() returns None at the end."""
    return OpenCVCamera(path)


class CallbackCamera:
    """Simulator camera: you provide a function that returns the latest simulated frame."""

    def __init__(self, get_frame: Callable[[], Optional[Frame]]) -> None:
        self._get_frame = get_frame

    def read(self) -> Optional[Frame]:
        return self._get_frame()

    def close(self) -> None:
        pass

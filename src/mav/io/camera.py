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
    """GStreamer pipeline string for a CSI camera on Jetson (headless safe, ends in appsink).

    `output_size` is the frame size Python actually receives - confirmed on the
    robot as 640x360 on 10 Nov 2026. Argus logs `capture_size` (1280x720), which is
    the sensor side and NOT what the detector sees. `lane.px_per_m` is pixels per
    metre at the OUTPUT size, so changing this rescales the whole lane model.
    """
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


class RecordingCamera:
    """Wraps a Camera and writes every frame it hands out to a video file.

    It wraps rather than sits beside the pipeline on purpose: `read()` returns the
    frame the detector sees, so recording the return value guarantees the clip and
    the live run are the same geometry. A separate `gst-launch` recording would be
    a second pipeline that can differ in size or colour conversion, and
    `lane.px_per_m` is pixels per metre *at a specific frame size* - a mismatch
    silently mis-scales every offset instead of failing.

    The writer is opened lazily from the first frame, because the frame size is
    only known once a frame exists.
    """

    def __init__(self, inner: Camera, path: str, fps: float = 20.0,
                 log: Callable[[str], None] = print) -> None:
        self._inner = inner
        self._path = path
        self._fps = fps
        self._log = log
        self._writer = None
        self._size: tuple[int, int] | None = None
        self.frames = 0
        self.path = path

    @property
    def size(self) -> tuple[int, int] | None:
        """(width, height) actually written, or None before the first frame."""
        return self._size

    def _open_writer(self, frame) -> None:
        import cv2

        h, w = frame.shape[:2]
        # mp4v for .mp4; MJPG/.avi is the fallback because some Jetson OpenCV
        # builds ship without the MPEG-4 encoder, and failing here would abort a
        # recording session that the camera is already delivering to.
        attempts = [("mp4v", self._path)]
        if self._path.lower().endswith(".mp4"):
            attempts.append(("MJPG", str(self._path) + ".avi"))
        last = None
        for fourcc_name, target in attempts:
            writer = cv2.VideoWriter(
                target, cv2.VideoWriter_fourcc(*fourcc_name), self._fps, (w, h)
            )
            if writer.isOpened():
                self._writer, self.path = writer, target
                self._size = (w, h)
                if target != self._path:
                    self._log(
                        f"recording: no mp4v encoder, writing MJPG to {target} instead"
                    )
                return
            writer.release()
            last = fourcc_name
        raise RuntimeError(
            f"cannot open a video writer for {self._path!r} (tried {attempts}). "
            "Is OpenCV built with video I/O? Check: python -c \"import cv2; "
            "print(cv2.VideoWriter_fourcc)\""
        )

    def read(self):
        frame = self._inner.read()
        if frame is not None:
            if self._writer is None:
                self._open_writer(frame)
            self._writer.write(frame)
            self.frames += 1
        return frame

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            h, w = self._size[1], self._size[0]
            self._log(f"recording: {self.frames} frames at {w}x{h} -> {self.path}")
        self._inner.close()


class CallbackCamera:
    """Simulator camera: you provide a function that returns the latest simulated frame."""

    def __init__(self, get_frame: Callable[[], Optional[Frame]]) -> None:
        self._get_frame = get_frame

    def read(self) -> Optional[Frame]:
        return self._get_frame()

    def close(self) -> None:
        pass

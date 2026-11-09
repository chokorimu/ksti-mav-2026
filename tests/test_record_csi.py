import sys
import types

from scripts.record_csi import _size, record


class FakeFrame:
    shape = (360, 640, 3)


class FakeCapture:
    def __init__(self, interrupt=False):
        self.released = False
        self.interrupt = interrupt
        self.frames = [FakeFrame(), FakeFrame(), None]

    def isOpened(self):
        return True

    def read(self):
        if self.interrupt and len(self.frames) == 1:
            raise KeyboardInterrupt
        frame = self.frames.pop(0)
        return (False, None) if frame is None else (True, frame)

    def release(self):
        self.released = True


class FakeWriter:
    def __init__(self):
        self.released = False
        self.writes = 0

    def isOpened(self):
        return True

    def write(self, frame):
        self.writes += 1

    def release(self):
        self.released = True


def test_size_parser():
    assert _size("640x360") == (640, 360)


def test_record_releases_camera_and_writer(monkeypatch, tmp_path):
    capture = FakeCapture()
    writer = FakeWriter()
    fake_cv2 = types.SimpleNamespace(
        CAP_GSTREAMER=1800,
        VideoCapture=lambda pipeline, backend: capture,
        VideoWriter=lambda *args: writer,
        VideoWriter_fourcc=lambda *chars: 1,
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    frames, actual_fps = record(tmp_path / "nested" / "recording.mp4", fps=20)

    assert frames == 2
    assert actual_fps >= 0.0
    assert writer.writes == 2
    assert capture.released
    assert writer.released


def test_record_finalizes_on_keyboard_interrupt(monkeypatch, tmp_path):
    capture = FakeCapture(interrupt=True)
    writer = FakeWriter()
    fake_cv2 = types.SimpleNamespace(
        CAP_GSTREAMER=1800,
        VideoCapture=lambda pipeline, backend: capture,
        VideoWriter=lambda *args: writer,
        VideoWriter_fourcc=lambda *chars: 1,
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    frames, _ = record(tmp_path / "interrupted.mp4", fps=20)

    assert frames == 2
    assert capture.released
    assert writer.released

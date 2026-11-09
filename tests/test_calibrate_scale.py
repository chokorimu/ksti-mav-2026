"""The calibration tool must refuse bad data, not average it.

`calibrate_scale` solves `px_per_m` from the LANE WIDTH, which is the only length
in the arena we know (rules §6: 0.50 m ±10%). That is a good idea, but it has one
failure mode that matters more than any other: on a clip where the band detector
is confused, the per-frame spans are garbage, and their MEDIAN is a
plausible-looking number that is not a measurement. Baked into the config, that
is indistinguishable from a real calibration - and every offset, the swerve
decision, and the cone lateral position silently inherit the error.

So the refusal path is the feature. These tests pin it, plus the two geometry
bugs that were found by actually running the tool on the real recording.
"""
from pathlib import Path

import numpy as np
import pytest

from mav.tools import calibrate_scale as cs

REPO = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------- the span rule ----
class _StubDetector:
    """Fake LaneDetector: detect_with_debug returns whatever we script."""

    def __init__(self, cfg, payload):
        self.cfg = cfg
        self.payload = payload

    def detect_with_debug(self, frame):
        return self.payload


def _frame(h=360, w=640):
    return np.zeros((h, w, 3), dtype=np.uint8)


def _cfg(roi_top=0.65):
    from mav.perception.lane import LaneConfig

    return LaneConfig(roi_top=roi_top)


def test_span_uses_only_bands_at_the_lookahead_row():
    """The bug: bands span the whole ROI, so max-min measures the WIDEST row.

    On the real clip the ROI covered rows 7..161 while the lookahead row sits
    near the bottom of it, and the same lane measured 4.7 px at p5 versus 505 px
    at p95 - because the nearest row is much wider on screen than the lookahead
    row. Using only nearby bands is what makes the number mean anything.
    """
    cfg = _cfg()
    from mav.perception.lane import lookahead_row

    h = 360
    y_look_roi = lookahead_row(cfg, h) - int(cfg.roi_top * h)
    payload = (
        type("M", (), {"confidence": 0.9})(),
        {
            "band_points": [
                # A far row with a tiny apparent span...
                [7.0, 300.0, 500.0],
                [7.0, 340.0, 500.0],
                # ...and the lookahead row, where the lane is genuinely wide.
                [y_look_roi, 200.0, 500.0],
                [y_look_roi, 440.0, 500.0],
            ],
            "lookahead_row": lookahead_row(cfg, h),
        },
    )
    span, conf = cs._span_at_lookahead(_StubDetector(cfg, payload), _frame())
    assert span == pytest.approx(240.0)  # the lookahead row, not 40 px from row 7
    assert conf == pytest.approx(0.9)


def test_span_needs_two_bands_at_the_lookahead_row():
    """One edge line alone is not a lane: it would be read as a 0 px span."""
    cfg = _cfg()
    from mav.perception.lane import lookahead_row

    h = 360
    y_look_roi = lookahead_row(cfg, h) - int(cfg.roi_top * h)
    payload = (
        type("M", (), {"confidence": 0.9})(),
        {"band_points": [[y_look_roi, 300.0, 500.0]], "lookahead_row": lookahead_row(cfg, h)},
    )
    assert cs._span_at_lookahead(_StubDetector(cfg, payload), _frame()) is None


def test_span_requires_a_measurement_at_all():
    cfg = _cfg()
    assert cs._span_at_lookahead(_StubDetector(cfg, (None, {})), _frame()) is None


# ---------------------------------------------------------------- the refusal ----
def test_refuses_to_report_a_scale_when_the_spans_are_wide(monkeypatch, capsys):
    """p75/p25 = 9.7x on the real clip: refuse, do not average.

    A 0.50 m lane cannot change width by a factor of ten between frames. If the
    spans say it does, the bands are wrong and the median is an average of
    garbage. This is the one behaviour that stops a fake calibration from being
    committed with real-looking precision.
    """
    spans = iter([20.0, 20.5, 199.0, 199.2] * 40)
    monkeypatch.setattr(cs, "open_video", lambda _p: _FakeCam(spans))
    monkeypatch.setattr(cs, "_span_at_lookahead", lambda _d, _f: (next(spans), 0.9))
    monkeypatch.setattr(cs, "load_lane_config", lambda _n: _cfg_for_scale())
    monkeypatch.setattr(cs, "profile_name", lambda _p=None: "prototype")

    rc = cs.calibrate("fake.mp4")
    out = capsys.readouterr().out
    assert rc == 2
    assert "REFUSING" in out
    assert "px_per_m =" not in out.replace("current px_per_m =", "")  # no value offered


def test_reports_a_scale_when_the_spans_agree(monkeypatch, capsys):
    # Exactly 100 px across a 0.50 m lane -> 200 px/m. Constant, so the median is
    # unambiguous; the point of the test is the accept path, not the statistics.
    spans = iter([100.0] * 160)
    monkeypatch.setattr(cs, "open_video", lambda _p: _FakeCam(spans))
    monkeypatch.setattr(cs, "_span_at_lookahead", lambda _d, _f: (next(spans), 0.9))
    monkeypatch.setattr(cs, "load_lane_config", lambda _n: _cfg_for_scale())
    monkeypatch.setattr(cs, "profile_name", lambda _p=None: "prototype")

    rc = cs.calibrate("fake.mp4")
    out = capsys.readouterr().out
    assert rc == 0
    assert "derived px_per_m = 200" in out  # 100 px / 0.50 m


def test_too_few_usable_frames_is_refused(monkeypatch, capsys):
    monkeypatch.setattr(cs, "open_video", lambda _p: _FakeCam(iter([100.0] * 5)))
    monkeypatch.setattr(cs, "_span_at_lookahead", lambda _d, _f: (100.0, 0.9))
    monkeypatch.setattr(cs, "load_lane_config", lambda _n: _cfg_for_scale())
    monkeypatch.setattr(cs, "profile_name", lambda _p=None: "prototype")

    assert cs.calibrate("fake.mp4") == 2
    assert "usable frames" in capsys.readouterr().out


class _FakeCam:
    def __init__(self, spans):
        self.spans = spans

    def read(self):
        try:
            next(self.spans)
        except StopIteration:
            return None
        return _frame()

    def close(self):
        pass


def test_a_clip_of_a_different_width_is_rescaled(monkeypatch, capsys):
    """px_per_m is PIXELS per metre, so frame width matters.

    The robot's CSI pipeline outputs 640x360; a phone clip is 854x480. A value
    measured on a clip of a different width is wrong for the robot by exactly
    that ratio, and nothing in the detector would notice - it would just report
    every offset ~33% too large.
    """
    spans = iter([100.0] * 160)
    monkeypatch.setattr(cs, "open_video", lambda _p: _WideCam(spans, width=854))
    monkeypatch.setattr(cs, "_span_at_lookahead", lambda _d, _f: (next(spans), 0.9))
    monkeypatch.setattr(cs, "load_lane_config", lambda _n: _cfg_for_scale())
    monkeypatch.setattr(cs, "profile_name", lambda _p=None: "prototype")

    assert cs.calibrate("fake.mp4") == 0
    out = capsys.readouterr().out
    # 100 px / 0.50 m = 200 at 854 px wide -> 200 * 640/854 = 150 at robot width.
    assert "150" in out
    assert "px_per_m: 150" in out


class _WideCam(_FakeCam):
    def __init__(self, spans, width):
        super().__init__(spans)
        self.width = width

    def read(self):
        f = super().read()
        return None if f is None else np.zeros((360, self.width, 3), dtype=np.uint8)


def _cfg_for_scale():
    from mav.perception.lane import LaneConfig

    return LaneConfig(px_per_m=130.0)

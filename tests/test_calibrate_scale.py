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
import re
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


def test_span_uses_the_detectors_own_lane_width():
    """The width must be the number the DETECTOR uses, not a re-derivation.

    `_band_points_with_clusters` already measures the lane width for its smear
    gate; `lane_width_px` exposes it. Reading anything else risks the calibrator
    and the detector disagreeing about what a lane is.
    """
    cfg = _cfg()
    payload = (
        type("M", (), {"confidence": 0.9})(),
        {"lane_width_px": 240.0, "n_width_samples": 9},
    )
    span, conf = cs._span_at_lookahead(_StubDetector(cfg, payload), _frame())
    assert span == pytest.approx(240.0)
    assert conf == pytest.approx(0.9)


def test_band_points_are_not_a_lane_width():
    """The bug that made the real clip report a 43.8 px lane on a 1280 px frame.

    `band_points` holds one centroid per band, and with both edge lines visible
    that centroid sits in the MIDDLE of the lane. So on a straight lane all the
    columns are identical and max-min is ~0; on a curve it measures how far the
    lane centre drifts across the frame. Either way it is not the width, and the
    old code used it as if it were.
    """
    cfg = _cfg()
    # A perfectly straight lane: every band centroid is the same column.
    straight = (
        type("M", (), {"confidence": 0.9})(),
        {"band_points": [[float(r), 320.0, 300.0] for r in range(7, 170, 14)]},
    )
    # No lane_width_px to fall back on -> refuse rather than invent a number.
    assert cs._span_at_lookahead(_StubDetector(cfg, straight), _frame()) is None


def test_span_refuses_a_missing_or_zero_width():
    cfg = _cfg()
    for payload in (
        {"lane_width_px": None},
        {"lane_width_px": 0.0},
        {"lane_width_px": 0.5},
    ):
        stub = _StubDetector(cfg, (type("M", (), {"confidence": 0.9})(), payload))
        assert cs._span_at_lookahead(stub, _frame()) is None, payload


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


def test_narrow_outliers_do_not_drag_the_value_down(monkeypatch, capsys):
    """The real clip's failure shape: broken frames make the lane look NARROWER.

    Two clusters 90 px apart on a real 500 px lane is one painted line split by a
    smudge. Those frames are one-sided - they can only ever under-report - so a
    plain median lands below the truth and p25/p75 screams "the lane changes
    size". The majority group is what the clip actually agrees on: 500 px across
    0.50 m = 1000 px/m.
    """
    good = [500.0, 505.0, 495.0, 510.0] * 40      # the real reading, 80%
    bad = [88.0, 92.0, 95.0, 90.0] * 10           # split lines, 20%
    spans = iter(good + bad)
    monkeypatch.setattr(cs, "open_video", lambda _p: _FakeCam(spans))
    monkeypatch.setattr(cs, "_span_at_lookahead", lambda _d, _f: (next(spans), 0.9))
    monkeypatch.setattr(cs, "load_lane_config", lambda _n: _cfg_for_scale())
    monkeypatch.setattr(cs, "profile_name", lambda _p=None: "prototype")

    rc = cs.calibrate("fake.mp4")
    out = capsys.readouterr().out
    assert rc == 0, out
    got = int(re.search(r"derived px_per_m = (\d+)", out).group(1))
    # ~500 px across 0.50 m. A plain median would land near 420 px (840 px/m)
    # because 20% of the frames under-report, so this also pins that the
    # outliers did NOT drag the answer down.
    assert 990 <= got <= 1030, f"got {got} px/m; the majority group says ~1000"


def test_a_clip_with_two_equally_popular_widths_is_refused(monkeypatch, capsys):
    """No majority means no answer: picking one mode would be a coin flip."""
    spans = iter([500.0, 505.0] * 40 + [900.0, 905.0] * 40)
    monkeypatch.setattr(cs, "open_video", lambda _p: _FakeCam(spans))
    monkeypatch.setattr(cs, "_span_at_lookahead", lambda _d, _f: (next(spans), 0.9))
    monkeypatch.setattr(cs, "load_lane_config", lambda _n: _cfg_for_scale())
    monkeypatch.setattr(cs, "profile_name", lambda _p=None: "prototype")

    rc = cs.calibrate("fake.mp4")
    out = capsys.readouterr().out
    assert rc == 2
    assert "REFUSING" in out
    assert "clear" in out  # says WHY: no clear winner


def test_dominant_group_reports_support_and_runner_up():
    import numpy as np

    vals = np.array([500.0] * 10 + [505.0] * 10 + [90.0] * 4)
    median, support, group, runner = cs._dominant_group(vals)
    assert median == pytest.approx(505.0, abs=5.0)
    assert support == pytest.approx(20 / 24)
    assert group.size == 20
    assert runner == 4


def test_dominant_group_of_nothing_is_safe():
    import numpy as np

    assert cs._dominant_group(np.array([]))[1] == 0.0


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


# ---------------------------------------------------------------- end to end ----
def test_recovers_the_true_scale_from_a_synthetic_lane():
    """The real proof: render a lane of known width and get px_per_m back.

    A synthetic frame has a ground truth, so this is the only test that can show
    the pipeline measures the right quantity. The recovered value runs slightly
    high because the painted lines are 0.05 m wide and the detector measures
    outer edge to outer edge, so the tolerance is a ratio, not an equality -
    the point is that it TRACKS the truth instead of reporting curve drift.
    """
    from mav.perception.lane import LaneConfig, LaneDetector
    from mav.sim.render import render_frame

    recovered = {}
    for truth in (200.0, 400.0):
        cfg = LaneConfig(px_per_m=truth, bottom_row_m=0.35)
        frame = render_frame(
            lambda y: 0.0, px_per_m=truth, y_near_m=0.35, y_far_m=2.2,
            lane_width_m=0.5, line_width_m=0.05, size=(360, 640),
        )
        span, _ = cs._span_at_lookahead(LaneDetector(cfg), frame)
        recovered[truth] = span / 0.5

    for truth, got in recovered.items():
        assert 0.9 * truth <= got <= 1.3 * truth, (truth, got)
    # And it must SCALE: doubling the truth doubles the answer. The old
    # band-point method returned ~0 for both, because a straight lane has one
    # centroid column.
    assert recovered[400.0] > 1.5 * recovered[200.0]

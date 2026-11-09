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


# ------------------------------------- saying WHY, not listing three maybes ----
def test_markings_at_row_finds_two_painted_lines():
    """The raw evidence: how many marking-shaped runs are on this row?"""
    frame = np.zeros((40, 200, 3), dtype=np.uint8)
    frame[:, 40:70] = 255      # a 30 px line
    frame[:, 120:150] = 255    # another 30 px line
    runs = cs._markings_at_row(frame, 20)
    assert len(runs) == 2
    assert runs[0] == (40, 70) and runs[1] == (120, 150)


def test_markings_at_row_ignores_a_wide_glare_patch():
    """A specular glare blob is ONE wide run; a painted line is 20-60 px.

    On a shiny floor the detector's two 'edges' were a dash paired with glare,
    which is why spans scattered 6.7x. Capping the run width is what separates
    them - without it, glare counts as a lane edge.
    """
    frame = np.zeros((40, 400, 3), dtype=np.uint8)
    frame[:, 20:50] = 255      # a real line
    frame[:, 150:380] = 255    # a 230 px glare patch
    runs = cs._markings_at_row(frame, 20)
    assert runs == [(20, 50)], f"glare must not count as a marking, got {runs}"


def test_markings_at_row_ignores_codec_speckle():
    frame = np.zeros((40, 200, 3), dtype=np.uint8)
    frame[:, 100:102] = 255    # 2 px: noise, not paint
    assert cs._markings_at_row(frame, 20) == []


def test_markings_at_row_reports_nothing_on_a_featureless_row():
    """A blank row must not be turned into a marking by an arbitrary threshold."""
    assert cs._markings_at_row(np.full((40, 200, 3), 90, np.uint8), 20) == []


def test_explanation_names_the_geometry_when_the_lane_is_wider_than_the_frame(
        monkeypatch, capsys):
    """The real first clip: 89% of frames showed 0 or 1 marking at the lookahead row.

    The refusal message used to list three possible causes and leave the operator
    to guess. On the robot's own clip the answer was decidable in one pass: both
    edges were outside the picture at that distance, because a 0.50 m lane seen
    0.4-0.5 m ahead is wider than a 640 px frame. That is a MOUNTING problem, and
    its fix (aim higher / look further) is the opposite of a recording problem's
    (record again), so guessing was expensive.
    """
    one_marking = np.zeros((360, 640, 3), dtype=np.uint8)
    one_marking[:, 300:330] = 255          # a single dash, like the real clip

    class Cam:
        def __init__(self):
            self.n = 0

        def read(self):
            self.n += 1
            return None if self.n > 20 else one_marking

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    cs._explain_lookahead_evidence(_StubDetector(_cfg(), (None, {})), "clip.mp4")
    out = capsys.readouterr().out
    assert "ONE marking" in out, out
    assert "outside the picture" in out, out
    assert "GEOMETRY" in out, "the fix must be named, not implied"
    # It must NOT send the operator back to re-record a clip that is fine.
    assert "Nothing is wrong with the recording" in out


def test_explanation_reports_two_visible_edges_as_a_pairing_problem(
        monkeypatch, capsys):
    """Both edges visible but the span swings 2x+: a dash is being paired with an edge."""
    wide = np.zeros((360, 640, 3), dtype=np.uint8)
    wide[:, 60:90] = 255       # left edge
    wide[:, 540:570] = 255     # right edge -> 510 px span
    narrow = np.zeros((360, 640, 3), dtype=np.uint8)
    narrow[:, 60:90] = 255     # left edge
    narrow[:, 240:270] = 255   # a dash -> 210 px "span", not the lane

    class Cam:
        def __init__(self):
            self.n = 0

        def read(self):
            self.n += 1
            if self.n > 20:
                return None
            return wide if self.n % 2 else narrow

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    cs._explain_lookahead_evidence(_StubDetector(_cfg(), (None, {})), "clip.mp4")
    out = capsys.readouterr().out
    assert "two markings" in out, out
    assert "dash-to-edge" in out, out


# ------------------------------------------- diagnosing the refusal, not listing it
def test_a_dashed_line_is_not_a_lane_edge():
    """The clip that produced a plausible WRONG answer.

    On the robot's first real clip the leftmost marking at row 155 was present in
    only 42% of frames. A lane EDGE is painted along the whole track, so it is in
    nearly every frame; a dashed centre line is not. The tool's row scan found a
    "stable" 331 px span there and was about to recommend px_per_m = 662 - a
    number that would look perfectly reasonable in the config and be dash-to-edge.
    Presence, not just stability, is what tells an edge line from a dash.
    """
    import mav.perception.lane as lane_mod
    from mav.tools import calibrate_scale as cs

    # A dash that appears in 4 of 10 frames, plus an edge line in all of them.
    frames = []
    for i in range(10):
        f = np.zeros((360, 640, 3), dtype=np.uint8)
        f[:, 540:570] = 255                      # the edge: always there
        if i < 4:
            f[:, 210:240] = 255                  # the dash: intermittent
        frames.append(f)

    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return frames[self.i - 1] if self.i <= len(frames) else None

        def close(self):
            pass

    detector = _StubDetector(_cfg(), (None, {}))
    orig = cs.open_video
    cs.open_video = lambda _p: Cam()
    try:
        cs._scan_rows_for_a_usable_lane(detector, "clip.mp4")
    finally:
        cs.open_video = orig
    # The point is the report, which we capture via capsys in the sibling test;
    # here assert the internal decision: no row may be called usable.
    # Re-derive it the way the function does.
    per_row = {}
    for f in frames:
        g = f[..., :3].astype(np.float32) @ lane_mod._LUM
        for y in range(5, 360, 10):
            runs = cs._runs_in_gray_row(g[y])
            if len(runs) == 2:
                per_row.setdefault(y, []).append(runs[1][1] - runs[0][0])
    assert per_row, "the fixture must produce two-marking rows"
    for y, spans in per_row.items():
        presence = len(spans) / len(frames)
        assert presence < 0.80, (
            f"row {y} has presence {presence:.0%}; a dash must never reach the "
            "80% an edge line requires")


def test_the_row_scan_refuses_to_recommend_a_dash(capsys, monkeypatch):
    import mav.perception.lane as lane_mod
    from mav.tools import calibrate_scale as cs

    frames = []
    for i in range(10):
        f = np.zeros((360, 640, 3), dtype=np.uint8)
        f[:, 540:570] = 255
        if i < 4:
            f[:, 210:240] = 255
        frames.append(f)

    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return frames[self.i - 1] if self.i <= len(frames) else None

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    cs._scan_rows_for_a_usable_lane(_StubDetector(_cfg(), (None, {})), "clip.mp4")
    out = capsys.readouterr().out
    assert "DASH" in out, out
    assert "px_per_m" not in out or "will not report it" in out, (
        f"a dash span must never be offered as a px_per_m:\n{out}")
    assert "662" not in out, "331 px / 0.5 m would have implied 662: never print it"


def test_the_row_scan_does_recommend_a_real_edge_pair(capsys, monkeypatch):
    """A continuously painted pair IS measurable - the tool must still work."""
    from mav.tools import calibrate_scale as cs

    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    frame[:, 100:130] = 255     # left edge
    frame[:, 420:450] = 255     # right edge -> 350 px, both present always

    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return frame if self.i <= 10 else None

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    # roi_top 0.0 so the measurable rows are inside the detector's ROI.
    cs._scan_rows_for_a_usable_lane(_StubDetector(_cfg(roi_top=0.0), (None, {})), "clip.mp4")
    out = capsys.readouterr().out
    assert "always present" in out, out
    # The span is 350 px, so 350/0.5 = 700 px/m - and the row is far from the
    # lookahead row, so it must say how to move the measurement point instead of
    # handing over a number that belongs to another distance.
    assert "350" in out, out
    assert "lookahead_frac" in out, out
    frac = re.search(r"lookahead_frac: ([0-9.]+)", out)
    assert frac and 0.0 <= float(frac.group(1)) <= 1.0, f"fraction out of range:\n{out}"


# ------------------------------------------------ the camera must be STILL ----
def test_camera_motion_is_detected(monkeypatch):
    """A moving mount cannot calibrate px_per_m: it is a property of the mount.

    The tool's own `--record` procedure said "push the robot along the lane",
    which is right for recording DETECTOR test material and wrong for a SCALE
    calibration. The first real clip moved 6 px/frame with a net drift of
    1220 px, so the lane's apparent width changed for reasons that have nothing
    to do with the camera geometry - and no single px_per_m describes it.
    """
    from mav.tools import calibrate_scale as cs

    frames = []
    base = np.zeros((180, 320, 3), dtype=np.uint8)
    base[40:60, 40:80] = 255
    base[100:130, 200:260] = 255
    for i in range(12):
        frames.append(np.roll(base, i * 7, axis=0))   # 7 px/frame: clearly moving

    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return frames[self.i - 1] if self.i <= len(frames) else None

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    got = cs._camera_motion("clip.mp4")
    assert got is not None
    med, share, _net = got
    assert med > 3.0, f"median shift should be about 7 px, got {med}"
    assert share > 0.5, f"most frames move, got {share:.0%}"


def test_a_still_clip_is_not_accused_of_moving(monkeypatch):
    """The check must not fire on a still clip, or every recording is refused."""
    from mav.tools import calibrate_scale as cs

    base = np.zeros((180, 320, 3), dtype=np.uint8)
    base[40:60, 40:80] = 255
    base[100:130, 200:260] = 255

    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return base.copy() if self.i <= 12 else None

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    assert cs._explain_camera_motion("clip.mp4") is False


def test_motion_message_says_to_record_with_the_robot_still(capsys, monkeypatch):
    from mav.tools import calibrate_scale as cs

    frames = []
    base = np.zeros((180, 320, 3), dtype=np.uint8)
    base[40:60, 40:80] = 255
    base[100:130, 200:260] = 255
    for i in range(12):
        frames.append(np.roll(base, i * 7, axis=0))

    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return frames[self.i - 1] if self.i <= len(frames) else None

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    assert cs._explain_camera_motion("clip.mp4") is True
    out = capsys.readouterr().out
    assert "STANDING STILL" in out, out
    assert "MOUNT" in out, "it must explain WHY stillness matters"


def test_a_cut_off_marking_invalidates_a_row(capsys, monkeypatch):
    """A marking touching the frame border means the lane is wider than the frame.

    This is direct evidence, unlike inferring it from a scattered width: if the
    pair is cut off, the distance measured is to the CUT edge, not the real one,
    so the span is a lower bound and px_per_m would come out too small.
    """
    from mav.tools import calibrate_scale as cs

    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    frame[:, 0:40] = 255        # left marking CUT by the frame border
    frame[:, 560:590] = 255     # right marking, fully visible

    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return frame if self.i <= 20 else None

        def close(self):
            pass

    monkeypatch.setattr(cs, "open_video", lambda _p: Cam())
    cs._scan_rows_for_a_usable_lane(_StubDetector(_cfg(), (None, {})), "clip.mp4")
    out = capsys.readouterr().out
    assert "TOUCHES the frame border" in out, out
    assert "always present" not in out, (
        f"a cut-off pair must never be called a usable lane width:\n{out}")


# ------------------------------ an ACCEPTED value must be verified too ----
def _still_cam(frames):
    class Cam:
        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return frames[self.i - 1] if self.i <= len(frames) else None

        def close(self):
            pass

    return Cam()


def test_an_accepted_value_is_still_checked_for_a_dash(monkeypatch):
    """The gap that let a plausible-but-wrong number through.

    The dash/presence and border-cutoff tests lived only in the REFUSAL
    diagnostics, so a value that passed the main gate was never examined for
    them. A STILL clip reports high support trivially - consecutive frames barely
    differ - so the main gate is weakest exactly where it looks strongest.
    """
    from mav.tools import calibrate_scale as cs

    # An edge that is always there, plus a dash that appears in only half the
    # frames: 50% presence, well below the 80% an edge line must reach.
    frames = []
    for i in range(20):
        f = np.zeros((360, 640, 3), dtype=np.uint8)
        f[:, 500:530] = 255
        if i % 2 == 0:
            f[:, 220:250] = 255
        frames.append(f)

    monkeypatch.setattr(cs, "open_video", lambda _p: _still_cam(frames))
    ok, notes = cs._verify_accepted_value(_StubDetector(_cfg(), (None, {})), "c.mp4", 280.0)
    assert ok is False, f"a 60% presence pair must not pass, notes: {notes}"
    assert any("DASH" in n for n in notes), notes


def test_an_accepted_value_is_still_checked_for_a_cut_off_marking(monkeypatch):
    from mav.tools import calibrate_scale as cs

    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    frame[:, 0:40] = 255        # touches the left border: cut off
    frame[:, 560:590] = 255

    monkeypatch.setattr(cs, "open_video", lambda _p: _still_cam([frame] * 20))
    ok, notes = cs._verify_accepted_value(_StubDetector(_cfg(), (None, {})), "c.mp4", 550.0)
    assert ok is False, f"a cut-off pair must not pass, notes: {notes}"
    assert any("TOUCHES the frame border" in n for n in notes), notes


def test_a_genuinely_good_row_passes_verification(monkeypatch):
    """The check must not reject a real edge pair, or nothing can ever calibrate."""
    from mav.tools import calibrate_scale as cs

    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    frame[:, 120:150] = 255     # left edge, fully inside
    frame[:, 430:460] = 255     # right edge, fully inside

    monkeypatch.setattr(cs, "open_video", lambda _p: _still_cam([frame] * 20))
    ok, notes = cs._verify_accepted_value(_StubDetector(_cfg(), (None, {})), "c.mp4", 310.0)
    assert ok is True, f"a clean pair must pass, notes: {notes}"

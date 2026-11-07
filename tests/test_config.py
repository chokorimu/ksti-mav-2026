"""Profile config loading: configs/<profile>.yaml -> LaneConfig / CheckpointConfig.

The point of mav.config is that a camera move or an arena colour reading is a
CONFIG edit, so these tests pin the mapping (YAML lists -> tuples, wrapping
hue bands, unknown keys, out-of-range values) and that every shipped profile
still parses.
"""
import tempfile
from pathlib import Path

import pytest

from mav.config import (
    load_checkpoint_config,
    load_lane_config,
    load_profile,
    profile_name,
)
from mav.perception.checkpoint import CheckpointConfig
from mav.perception.lane import LaneConfig


def _root(text: str, profile: str = "prototype") -> Path:
    """A throwaway repo root holding configs/<profile>.yaml with `text`."""
    d = Path(tempfile.mkdtemp())
    (d / "configs").mkdir()
    (d / "configs" / f"{profile}.yaml").write_text(text)
    return d


def test_missing_sections_keep_the_code_defaults():
    cfg = load_lane_config("prototype", _root("profile: prototype\n"))
    assert cfg == LaneConfig()
    cp = load_checkpoint_config("prototype", _root("profile: prototype\n"))
    assert cp == CheckpointConfig()


def test_lane_geometry_comes_from_config():
    root = _root("lane:\n  px_per_m: 1050.0\n  bottom_row_m: 0.45\n  roi_top: 0.7\n")
    cfg = load_lane_config("prototype", root)
    assert cfg.px_per_m == 1050.0
    assert cfg.bottom_row_m == 0.45
    assert cfg.roi_top == 0.7


def test_null_disables_the_band_gate():
    # The one scalar where `null` is meaningful: it turns the gate off.
    root = _root("lane:\n  max_band_span_m: null\n")
    assert load_lane_config("prototype", root).max_band_span_m is None


def test_pair_field_becomes_a_tuple():
    root = _root("lane:\n  mask_frac_band: [0.02, 0.2]\n")
    assert load_lane_config("prototype", root).mask_frac_band == (0.02, 0.2)


def test_hue_bands_become_tuples_and_may_wrap():
    root = _root(
        "checkpoint:\n"
        "  checkpoint_hues: [[90, 150]]\n"
        "  finish_hues: [[340, 20]]\n"
    )
    cfg = load_checkpoint_config("prototype", root)
    assert cfg.checkpoint_hues == ((90.0, 150.0),)
    # lo > hi is a band that wraps around 360 (red), not an error.
    assert cfg.finish_hues == ((340.0, 20.0),)


def test_empty_hue_list_disables_that_kind():
    root = _root("checkpoint:\n  checkpoint_hues: []\n")
    cfg = load_checkpoint_config("prototype", root)
    assert cfg.checkpoint_hues == ()
    assert cfg.finish_hues == ()  # still the default (disabled) [A11]


def test_unknown_key_names_itself():
    root = _root("lane:\n  px_per_mm: 130\n")
    with pytest.raises(ValueError, match="px_per_mm"):
        load_lane_config("prototype", root)


def test_hue_outside_0_360_is_refused():
    root = _root("checkpoint:\n  checkpoint_hues: [[90, 400]]\n")
    with pytest.raises(ValueError, match="outside 0..360"):
        load_checkpoint_config("prototype", root)


def test_out_of_range_value_is_refused():
    root = _root("lane:\n  roi_top: 1.5\n")
    with pytest.raises(ValueError, match="roi_top"):
        load_lane_config("prototype", root)


def test_section_that_is_not_a_mapping_is_refused():
    root = _root("lane: 130\n")
    with pytest.raises(ValueError, match="expected a mapping"):
        load_lane_config("prototype", root)


def test_missing_profile_file_is_an_error_not_silent_defaults():
    with pytest.raises(FileNotFoundError):
        load_lane_config("no-such-profile", _root("lane: {}\n"))


def test_profile_name_follows_the_env(monkeypatch):
    monkeypatch.setenv("MAV_PROFILE", "final")
    assert profile_name() == "final"
    assert profile_name("sim") == "sim"  # an explicit name wins


def test_every_shipped_profile_parses():
    for name in ("sim", "prototype", "final"):
        data = load_profile(name)
        assert data["profile"] == name
        load_lane_config(name)
        load_checkpoint_config(name)


def test_final_stays_serverless():
    # final: no network server, streaming, or SSH dependency while it runs.
    data = load_profile("final")
    assert data["network"]["enabled"] is False
    assert "dashboard" not in data
    assert "sim" not in data

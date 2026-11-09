"""Profile configuration: configs/<profile>.yaml -> perception dataclasses.

Two things in this repo are per-profile CONFIG rather than constants in code:

  * lane geometry (`px_per_m`, `bottom_row_m`, `roi_top`, `max_band_span_m`).
    The camera mount is not fixed, so a camera move is a config edit here,
    never a code edit.
  * cross-line colours [A11]. The rules forbid hardcoding the checkpoint or
    start/finish colour without confirmation, so the hue bands are parameters
    whose defaults follow the guidebook text and stay UNCONFIRMED until they
    are read off the arena on test day.

The shape follows `load_vehicle_params()` in mav/types.py: read the YAML, map
the section onto the dataclass, and fail loudly on an unknown key - a typo in
a config file must never silently fall back to a default.
"""
from __future__ import annotations

import os
import subprocess
import sys
from dataclasses import fields
from pathlib import Path

import pytest
import yaml

from mav.config import (
    DEFAULT_PROFILE,
    load_checkpoint_config,
    load_lane_config,
    load_profile,
    profile_path,
)
from mav.perception.checkpoint import CheckpointConfig
from mav.perception.lane import LaneConfig

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture()
def tmp_root(tmp_path):
    """A fake repo: root/configs/<profile>.yaml, like the real layout."""
    (tmp_path / "configs").mkdir()
    return tmp_path


def write_cfg(root, profile, text):
    (root / "configs" / f"{profile}.yaml").write_text(text)
    return root


# ---------------------------------------------------------------- defaults ----
def test_lane_config_defaults_when_no_section(tmp_root):
    write_cfg(tmp_root, "p", "profile: p\n")
    cfg = load_lane_config("p", root=tmp_root)
    assert cfg == LaneConfig()  # every field keeps its code default


def test_checkpoint_config_defaults_when_no_section(tmp_root):
    write_cfg(tmp_root, "p", "profile: p\n")
    cfg = load_checkpoint_config("p", root=tmp_root)
    assert cfg == CheckpointConfig()
    assert cfg.finish_hues == ()  # [A11] unknown colour: disabled


# ---------------------------------------------------------------- section read ----
def test_lane_config_reads_the_section(tmp_root):
    write_cfg(tmp_root, "p", """
lane:
  px_per_m: 964.0
  bottom_row_m: 0.45
  roi_top: 0.7
  max_band_span_m: 0.6
""")
    cfg = load_lane_config("p", root=tmp_root)
    assert (cfg.px_per_m, cfg.bottom_row_m) == (964.0, 0.45)
    assert (cfg.roi_top, cfg.max_band_span_m) == (0.7, 0.6)


def test_checkpoint_config_reads_the_section(tmp_root):
    write_cfg(tmp_root, "p", """
checkpoint:
  checkpoint_hues: [[90.0, 150.0]]
  finish_hues: [[340.0, 20.0]]
  min_saturation: 0.5
""")
    cfg = load_checkpoint_config("p", root=tmp_root)
    assert cfg.checkpoint_hues == ((90.0, 150.0),)
    assert cfg.finish_hues == ((340.0, 20.0),)  # wraps around 360
    assert cfg.min_saturation == 0.5


def test_pair_fields_accept_a_two_item_list(tmp_root):
    write_cfg(tmp_root, "p", "lane: {mask_frac_band: [0.02, 0.2]}\n")
    cfg = load_lane_config("p", root=tmp_root)
    assert cfg.mask_frac_band == (0.02, 0.2)


# ---------------------------------------------------------------- bad input ----
def test_unknown_lane_key_raises(tmp_root):
    write_cfg(tmp_root, "p", "lane: {px_per_m: 130.0, bogus: 1}\n")
    with pytest.raises(ValueError) as exc:
        load_lane_config("p", root=tmp_root)
    assert "unknown key(s)" in str(exc.value) and "bogus" in str(exc.value)


def test_unknown_checkpoint_key_raises(tmp_root):
    write_cfg(tmp_root, "p", "checkpoint: {finish_hues: [], nope: 3}\n")
    with pytest.raises(ValueError) as exc:
        load_checkpoint_config("p", root=tmp_root)
    assert "unknown key(s)" in str(exc.value) and "nope" in str(exc.value)


def test_bad_hue_band_shape_raises(tmp_root):
    write_cfg(tmp_root, "p", "checkpoint: {checkpoint_hues: [[90.0]]}\n")
    with pytest.raises(ValueError) as exc:
        load_checkpoint_config("p", root=tmp_root)
    assert "each hue band must be [lo, hi]" in str(exc.value)


def test_hue_outside_0_360_raises(tmp_root):
    write_cfg(tmp_root, "p", "checkpoint: {finish_hues: [[400.0, 10.0]]}\n")
    with pytest.raises(ValueError) as exc:
        load_checkpoint_config("p", root=tmp_root)
    assert "outside 0..360" in str(exc.value)


def test_out_of_range_lane_value_raises(tmp_root):
    write_cfg(tmp_root, "p", "lane: {roi_top: 1.5}\n")
    with pytest.raises(ValueError) as exc:
        load_lane_config("p", root=tmp_root)
    assert "roi_top=1.5" in str(exc.value) and "above" in str(exc.value)


def test_out_of_range_checkpoint_value_raises(tmp_root):
    write_cfg(tmp_root, "p", "checkpoint: {min_saturation: 2.0}\n")
    with pytest.raises(ValueError) as exc:
        load_checkpoint_config("p", root=tmp_root)
    assert "min_saturation=2" in str(exc.value) and "above" in str(exc.value)


def test_section_must_be_a_mapping(tmp_root):
    write_cfg(tmp_root, "p", "lane: 42\n")
    with pytest.raises(ValueError) as exc:
        load_lane_config("p", root=tmp_root)
    assert "expected a mapping" in str(exc.value)


def test_missing_profile_file_raises(tmp_root):
    with pytest.raises(FileNotFoundError) as exc:
        load_profile("nope", root=tmp_root)
    assert "no config for profile 'nope'" in str(exc.value)


# ---------------------------------------------------------------- null gates ----
def test_null_disables_an_optional_gate(tmp_root):
    write_cfg(tmp_root, "p", "lane: {max_band_span_m: null}\n")
    cfg = load_lane_config("p", root=tmp_root)
    assert cfg.max_band_span_m is None  # the band gate is off


def test_null_on_a_tuple_field_keeps_the_default(tmp_root):
    write_cfg(tmp_root, "p", "checkpoint: {checkpoint_hues: null}\n")
    cfg = load_checkpoint_config("p", root=tmp_root)
    assert cfg.checkpoint_hues == ((90.0, 150.0),)  # the code default


# ---------------------------------------------------------------- env / paths ----
def test_profile_env_var_picks_the_profile(tmp_root, monkeypatch):
    write_cfg(tmp_root, "final", "lane: {px_per_m: 964.0}\n")
    monkeypatch.setenv("MAV_PROFILE", "final")
    assert profile_path(root=tmp_root) == tmp_root / "configs" / "final.yaml"
    assert load_lane_config(root=tmp_root).px_per_m == 964.0


def test_default_profile_is_prototype():
    assert DEFAULT_PROFILE == "prototype"


# ---------------------------------------------------------------- shipped configs ----
@pytest.mark.parametrize("profile", ["sim", "prototype", "final"])
def test_every_shipped_profile_parses(profile):
    lane = load_lane_config(profile)
    cp = load_checkpoint_config(profile)
    assert lane.px_per_m > 0.0
    assert cp.checkpoint_hues  # green band present...
    assert cp.finish_hues == ()  # ...finish disabled until read off the arena [A11]


def test_repo_profiles_are_the_real_files():
    for profile in ("sim", "prototype", "final"):
        assert (REPO / "configs" / f"{profile}.yaml").is_file()


# ---------------------------------------------------------------- boot validation ----
def test_boot_validates_both_sections():
    for profile in ("prototype", "final"):
        env = {**os.environ, "PYTHONPATH": str(REPO / "src"), "MAV_PROFILE": profile}
        out = subprocess.run(
            [sys.executable, "-m", "mav", "--check"], env=env, cwd=REPO,
            capture_output=True, text=True, timeout=60,
        )
        assert out.returncode == 0, out.stderr
        assert "lane      :" in out.stdout
        assert "checkpoint:" in out.stdout
        assert "px_per_m=130.0" in out.stdout


def test_boot_refuses_unmeasured_values_per_drive():
    """The real-robot path must refuse to start while ITS drive is unmeasured.

    `prototype` is differential and its `track_width_mm` is measured (250 mm,
    9 Nov 2026), so it boots. `final` is ackermann and needs `wheel_base_mm` and
    `max_steer_deg`, both still null (Q1): that is the intended behaviour
    (AGENTS.md rule 2: never guess hardware facts), so boot exits non-zero and
    names a missing key instead of silently driving an unknown robot.
    """
    def boot(profile):
        env = {**os.environ, "PYTHONPATH": str(REPO / "src"), "MAV_PROFILE": profile}
        return subprocess.run(
            [sys.executable, "-m", "mav"], env=env, cwd=REPO,
            capture_output=True, text=True, timeout=60,
        )

    proto = boot("prototype")
    # It boots past validation and then fails on the CSI camera, which does not
    # exist on a laptop. What matters is that it did NOT refuse on the params.
    assert "measure the robot" not in proto.stderr

    final = boot("final")
    # Two-step refusal, both loud:
    #   1. vehicle.yaml still says drive: differential -> the profile check fires;
    #   2. after switching drive: ackermann, load_vehicle_params() demands
    #      wheel_base_mm + max_steer_deg, which are still null.
    # Either message is acceptable here; both name what to measure.
    assert final.returncode != 0
    assert "ackermann" in final.stderr or "measure the robot" in final.stderr


def test_boot_fails_loudly_on_a_bad_key(tmp_root):
    write_cfg(tmp_root, "prototype", "lane: {px_per_metre: 130.0}\n")
    # __main__ resolves configs/ from its own file, so a bad key is tested
    # through the loader with the same "must not fall back silently" rule.
    with pytest.raises(ValueError) as exc:
        load_lane_config("prototype", root=tmp_root)
    assert "unknown key(s)" in str(exc.value)

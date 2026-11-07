"""Profile configuration: configs/<profile>.yaml -> perception dataclasses.

Two things in this repo are per-profile CONFIG rather than constants in code:

  * lane geometry (`px_per_m`, `bottom_row_m`, ...). The camera mount is not
    fixed - the recording in data/recordings/ was shot from a phone and
    measures 964-1610 px/m, roughly ten times the 130 the detector assumes -
    so moving the camera is a config edit, never a code edit.
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
from dataclasses import fields
from pathlib import Path

import yaml

from mav.perception.checkpoint import CheckpointConfig
from mav.perception.lane import LaneConfig

DEFAULT_PROFILE = "prototype"

# YAML has no tuples: these fields arrive as lists and are converted.
_HUE_FIELDS = frozenset({"checkpoint_hues", "finish_hues"})
_PAIR_FIELDS = frozenset({"mask_frac_band"})

# Inclusive (low, high) sanity ranges; None = unbounded on that side. A value
# of `null` skips the check - that is how an optional gate is turned off.
_RANGES: dict[str, tuple[float | None, float | None]] = {
    "roi_top": (0.0, 1.0),
    "lookahead_frac": (0.0, 1.0),
    "px_per_m": (0.0, None),
    "bottom_row_m": (0.0, None),
    "bg_ksize": (1, None),
    "thresh_k": (0.0, None),
    "n_bands": (1, None),
    "min_band_pixels": (1, None),
    "min_points": (1, None),
    "min_contrast": (0.0, None),
    "cluster_gap_m": (0.0, None),
    "max_band_span_m": (0.0, None),
    "min_saturation": (0.0, 1.0),
    "min_value": (0.0, 1.0),
    "min_span_frac": (0.0, 1.0),
    "target_span_frac": (0.0, 1.0),
    "max_band_frac": (0.0, 1.0),
    "min_confidence": (0.0, 1.0),
}


def repo_root() -> Path:
    """Walk up from this file until a directory that holds a configs/ folder."""
    for base in Path(__file__).resolve().parents:
        if (base / "configs").is_dir():
            return base
    return Path.cwd()


def profile_name(profile: str | None = None) -> str:
    """MAV_PROFILE picks the profile when the caller does not name one."""
    return profile or os.environ.get("MAV_PROFILE") or DEFAULT_PROFILE


def profile_path(profile: str | None = None, root: Path | None = None) -> Path:
    return (root or repo_root()) / "configs" / f"{profile_name(profile)}.yaml"


def load_profile(profile: str | None = None, root: Path | None = None) -> dict:
    """The whole configs/<profile>.yaml as a dict."""
    path = profile_path(profile, root)
    if not path.is_file():
        raise FileNotFoundError(
            f"no config for profile {profile_name(profile)!r}: {path}"
        )
    return yaml.safe_load(path.read_text()) or {}


def load_lane_config(
    profile: str | None = None, root: Path | None = None
) -> LaneConfig:
    """LaneConfig for a profile: code defaults, overridden by its `lane:` section."""
    name = profile_name(profile)
    where = f"configs/{name}.yaml lane"
    data = load_profile(name, root)
    return _build(LaneConfig, _section(data, "lane", where), where)


def load_checkpoint_config(
    profile: str | None = None, root: Path | None = None
) -> CheckpointConfig:
    """CheckpointConfig for a profile: code defaults, overridden by `checkpoint:`."""
    name = profile_name(profile)
    where = f"configs/{name}.yaml checkpoint"
    data = load_profile(name, root)
    return _build(CheckpointConfig, _section(data, "checkpoint", where), where)


def _section(data: dict, key: str, where: str) -> dict:
    sec = data.get(key)
    if sec is None:
        return {}
    if not isinstance(sec, dict):
        raise ValueError(f"{where}: expected a mapping, got {type(sec).__name__}")
    return sec


def _build(cls, section: dict, where: str):
    valid = {f.name: f for f in fields(cls)}
    unknown = sorted(set(section) - set(valid))
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {unknown}; valid: {sorted(valid)}")
    kwargs: dict = {}
    for key, value in section.items():
        if value is None:
            # `null` on a scalar means "disabled" (`max_band_span_m: null` turns
            # the band gate off); on a tuple field it means "keep the default",
            # because the disabled spelling there is an empty list.
            default = valid[key].default
            kwargs[key] = None if not isinstance(default, tuple) else default
            continue
        kwargs[key] = _coerce(key, value)
        _check_range(key, kwargs[key], where)
    return cls(**kwargs)


def _coerce(key: str, value):
    """YAML lists -> the tuples the dataclasses expect."""
    if key in _HUE_FIELDS:
        bands = []
        for band in value:
            if not isinstance(band, (list, tuple)) or len(band) != 2:
                raise ValueError(f"{key}: each hue band must be [lo, hi], got {band!r}")
            lo, hi = float(band[0]), float(band[1])
            for hue in (lo, hi):
                if not 0.0 <= hue <= 360.0:
                    raise ValueError(f"{key}: hue {hue:g} outside 0..360")
            bands.append((lo, hi))
        return tuple(bands)
    if key in _PAIR_FIELDS:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"{key}: expected [low, high], got {value!r}")
        return tuple(float(v) for v in value)
    return value


def _check_range(key: str, value, where: str) -> None:
    lo, hi = _RANGES.get(key, (None, None))
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return
    if lo is not None and value < lo:
        raise ValueError(f"{where}: {key}={value:g} is below {lo:g}")
    if hi is not None and value > hi:
        raise ValueError(f"{where}: {key}={value:g} is above {hi:g}")

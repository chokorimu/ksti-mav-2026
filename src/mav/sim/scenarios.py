"""Scripted lane layouts for the simulator, plus the measured track."""
from __future__ import annotations

import math
from dataclasses import dataclass

from mav.sim.world import Segment


@dataclass(frozen=True)
class Scenario:
    name: str
    segments: tuple[Segment, ...] = ()
    start_offset_m: float = 0.0   # vehicle starts this far right of the centreline
    description: str = ""
    # Raw centreline in world metres. When set, this polyline IS the lane and
    # `segments` is ignored -- the measured track is not a sequence of straights
    # and arcs, so it cannot be expressed as Segments.
    points: tuple[tuple[float, float], ...] | None = None


SCENARIOS: dict[str, Scenario] = {
    "straight": Scenario(
        "straight", (Segment(12.0),), 0.0, "12 m straight, centred"
    ),
    "offset": Scenario(
        "offset", (Segment(12.0),), 0.3, "12 m straight, start 0.3 m right of centre"
    ),
    "curve": Scenario(
        "curve",
        (Segment(6.0, 0.0), Segment(math.pi, 0.5), Segment(4.0, 0.0)),
        0.0,
        "6 m straight, 90 deg left at R=2 m, 4 m straight",
    ),
    "s_curve": Scenario(
        "s_curve",
        (Segment(5.0, 0.4), Segment(5.0, -0.4), Segment(5.0, 0.4)),
        0.0,
        "alternating R=2.5 m left/right curves",
    ),
}


def scenario_names() -> list[str]:
    """Every scenario name, synthetic and measured. No file I/O."""
    from mav.sim.track import TRACKS  # lazy: track imports this module

    return sorted(list(SCENARIOS) + list(TRACKS))


def scenario_info(name: str) -> tuple[str, str]:
    """(name, description) for dropdowns and --list. No file I/O."""
    if name in SCENARIOS:
        return name, SCENARIOS[name].description
    from mav.sim.track import TRACKS

    if name in TRACKS:
        return name, TRACKS[name]["description"]
    raise KeyError(f"unknown scenario {name!r}; have: {', '.join(scenario_names())}")


def get_scenario(name: str) -> Scenario:
    if name in SCENARIOS:
        return SCENARIOS[name]
    from mav.sim.track import TRACKS, track_scenario  # lazy: reads the JSON

    if name in TRACKS:
        return track_scenario(name)
    raise KeyError(f"unknown scenario {name!r}; have: {', '.join(scenario_names())}") from None

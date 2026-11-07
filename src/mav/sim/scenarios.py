"""Scripted lane layouts for the simulator."""
from __future__ import annotations

import math
from dataclasses import dataclass

from mav.sim.world import Segment


@dataclass(frozen=True)
class Scenario:
    name: str
    segments: tuple[Segment, ...]
    start_offset_m: float = 0.0   # vehicle starts this far right of the centreline
    description: str = ""


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


def get_scenario(name: str) -> Scenario:
    try:
        return SCENARIOS[name]
    except KeyError:
        raise KeyError(f"unknown scenario {name!r}; have: {', '.join(sorted(SCENARIOS))}") from None

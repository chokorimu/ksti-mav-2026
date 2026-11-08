import os
import sys
from pathlib import Path

import yaml


def _repo_root() -> Path:
    """Walk up from this file until a directory that holds a configs/ folder."""
    for base in Path(__file__).resolve().parents:
        if (base / "configs").is_dir():
            return base
    return Path.cwd()


def main() -> None:
    profile = os.environ.get("MAV_PROFILE", "prototype")
    if profile == "sim":
        # The sim profile runs the real control loop against the simulated world.
        from mav.sim.runner import main as sim_main

        raise SystemExit(sim_main())

    if profile in ("prototype", "final") and "--check" not in sys.argv[1:]:
        # Real robot: camera -> detector -> policy -> serial -> Arduino. `--check` keeps the old behaviour.
        from mav.robot import main as robot_main

        raise SystemExit(robot_main())

    path = _repo_root() / "configs" / f"{profile}.yaml"
    cfg = yaml.safe_load(path.read_text())
    print(f"profil: {profile}")
    print(cfg)
    # Validate the perception sections here: a typo in configs/ must fail at
    # boot with a clear message, not silently halfway around the track.
    from mav.config import load_checkpoint_config, load_lane_config

    lane = load_lane_config(profile)
    cp = load_checkpoint_config(profile)
    print(f"lane      : px_per_m={lane.px_per_m} bottom_row_m={lane.bottom_row_m} "
          f"roi_top={lane.roi_top}")
    print(f"checkpoint: hues={cp.checkpoint_hues} "
          f"finish={cp.finish_hues or 'disabled [A11]'}")


if __name__ == "__main__":
    main()

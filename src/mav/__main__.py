import os
from pathlib import Path

import yaml


def main() -> None:
    profile = os.environ.get("MAV_PROFILE", "prototype")
    if profile == "sim":
        # The sim profile runs the real control loop against the simulated world.
        from mav.sim.runner import main as sim_main

        raise SystemExit(sim_main())

    path = Path(__file__).resolve().parents[2] / "configs" / f"{profile}.yaml"
    cfg = yaml.safe_load(path.read_text())
    print(f"profil: {profile}")
    print(cfg)


if __name__ == "__main__":
    main()

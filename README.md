# KSTI MAV 2026 Prototype

Minimal setup + all commands. Prototype = differential steering (skid). Final = Ackermann.

## Quick Start

```bash
# 1. Laptop (Python 3.14)
source .venv/bin/activate
scripts/setup_dev.sh

# 2. Jetson (Orin Nano, JetPack 6.2)
scripts/setup_jetson.sh

# 3. Run (all profiles)
scripts/run.sh sim     # synthetic closed-loop
scripts/run.sh prototype # real camera + Arduino (wheels lifted!)
scripts/run.sh final    # competition mode
```

## Key Commands

```bash
# Sim + Dashboard
scripts/run.sh sim curve --dashboard   # open http://127.0.0.1:8000
scripts/run.sh sim ksti                # measured track

# Real Robot (prototype)
scripts/run.sh prototype --dry-run     # no hardware, just camera
scripts/run.sh prototype               # real camera + Arduino

# Offline detector
PYTHONPATH=src python -m mav.perception.lane data/recordings/*.mp4

# Test everything
pytest

# Firmware (Arduino)
cd firmware/arduino/prototype
pio run
pio run -t upload
pio device monitor
```

## Profiles

| Profile   | Purpose                     | Config          | Hardware |
|-----------|-----------------------------|-----------------|----------|
| `sim`     | Synthetic closed-loop       | `configs/sim.yaml` | None     |
| `prototype` | Development (differential) | `configs/prototype.yaml` | Camera + Arduino |
| `final`   | Competition (Ackermann)     | `configs/final.yaml` | Camera + Arduino |

## Important Files

- `configs/*.yaml`: lane geometry, checkpoint colours, vehicle params
- `src/mav/robot.py`: real-robot driver (policy + tracker + bounded lane hold)
- `src/mav/perception/lane.py`: detector (improved cluster + robust fit)
- `src/mav/planning/lane_policy.py`: pure-pursuit with `on_lost="hold"`
- `src/mav/sim/`: synthetic simulator + web dashboard

All tests pass: `pytest`

Run `python -m mav --check` to validate config at boot.
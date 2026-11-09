#!/usr/bin/env bash
# Flash the Arduino firmware. First argument selects the variant (same names
# as the repo profiles): prototype (4 motors, skid steer) or final (rear
# motor + steering servo).
#
#   scripts/flash_firmware.sh <prototype|final> [--port /dev/ttyACM0]
#   scripts/flash_firmware.sh prototype --compile-only
#   scripts/flash_firmware.sh --list-ports
#
# Options:
#   --port PORT     upload port (default: auto-detect when exactly one board
#                   is plugged in)
#   --compile-only  build only, do not upload
#   --list-ports    list candidate serial ports and exit
#   -- <args>       everything after -- is passed to `pio run` (e.g. -v)
#
# Run this on the LAPTOP with the board plugged in over USB. PlatformIO's AVR
# toolchain is x86_64 and is not verified on the Jetson (aarch64) - see AGENTS.md.
#
# The board is an Arduino NANO (10 Nov 2026), so a clone appears as /dev/ttyUSB*
# (CH340) rather than /dev/ttyACM*. Both are handled below.
#
# NOTE: flashing a sketch with MPS_TO_PWM = 0 leaves the car unable to move at
# all. That is the shipped state; the speed calibration procedure in
# docs/hardware.md starts by flashing a PROVISIONAL value on purpose.

set -euo pipefail
cd "$(dirname "$0")/.."
source .venv/bin/activate

variant="${1:-}"
if [ $# -gt 0 ]; then shift; fi

port=""
compile_only=0
list_ports=0
pio_extra=()
while [ $# -gt 0 ]; do
  case "$1" in
    --port) port="${2:?--port needs a value}"; shift 2 ;;
    --port=*) port="${1#*=}"; shift ;;
    --compile-only) compile_only=1; shift ;;
    --list-ports) list_ports=1; shift ;;
    --) shift; pio_extra=("$@"); break ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

case "$variant" in
  prototype|final) ;;
  *)
    echo "Usage: scripts/flash_firmware.sh <prototype|final> [--port PORT] [--compile-only] [--list-ports] [-- <pio args>]" >&2
    exit 2
    ;;
esac

# Candidate ports. Use the SAME enumeration as the runner (pyserial via
# mav.io.actuator.list_serial_ports) so the two can never disagree, then fall
# back to globbing if python/pyserial is unavailable.
#
# A hardcoded /dev/ttyACM* /dev/ttyUSB* glob is not enough: WCH's own out-of-tree
# driver (usb_ch341, present on some Jetson images) creates /dev/ttyCH341USB0
# instead of /dev/ttyUSB0, and the glob silently misses the board while it sits
# there bound to a driver. That cost a debugging session on 10 Nov 2026.
ports=()
while IFS= read -r p; do
  [ -n "$p" ] && ports+=("$p")
done < <(
  PYTHONPATH=src python -c '
from mav.io.actuator import list_serial_ports
for p in list_serial_ports():
    print(p)
' 2>/dev/null || true
)
if [ "${#ports[@]}" -eq 0 ]; then
  for p in /dev/ttyACM* /dev/ttyUSB* /dev/ttyCH341*; do
    [ -e "$p" ] && ports+=("$p")
  done
fi

# Listing ports must work BEFORE the pio check: the Jetson has no pio (the AVR
# toolchain is x86-only) and is exactly where you need to find out which port
# the Nano landed on.
if [ "$list_ports" -eq 1 ]; then
  if [ "${#ports[@]}" -eq 0 ]; then
    echo "No serial ports found - is the Arduino plugged in?"
    echo "The chip may still be visible on USB: lsusb | grep -i '1a86\\|2341'"
  else
    printf '%s\n' "${ports[@]}"
  fi
  exit 0
fi

command -v pio >/dev/null 2>&1 || {
  echo "pio not found - run scripts/setup_dev.sh first (pip install platformio)." >&2
  echo "Listing ports needs no pio: scripts/flash_firmware.sh prototype --list-ports" >&2
  exit 1
}

if [ "$compile_only" -eq 0 ]; then
  if [ "${#ports[@]}" -eq 0 ]; then
    echo "No Arduino detected." >&2
    echo "Plug it in, then check: scripts/flash_firmware.sh prototype --list-ports" >&2
    echo "If lsusb sees it but there is no device node, the driver is not binding:" >&2
    echo "  lsusb -t          # which driver owns it?" >&2
    echo "  sudo dmesg | tail -20" >&2
    exit 1
  fi
  if [ -z "$port" ]; then
    if [ "${#ports[@]}" -gt 1 ]; then
      echo "Multiple serial ports found:" >&2
      printf '  %s\n' "${ports[@]}" >&2
      echo "Re-run with --port <one of them>" >&2
      exit 1
    fi
    port="${ports[0]}"
  fi
fi

project="firmware/arduino/$variant"

echo "==> $variant: compile"
pio run --project-dir "$project" ${pio_extra[@]+"${pio_extra[@]}"}

if [ "$compile_only" -eq 1 ]; then
  echo "==> compile only, nothing uploaded"
  exit 0
fi

echo "==> $variant: upload to $port"
pio run --project-dir "$project" -t upload --upload-port "$port"
echo "==> done. Monitor with: pio device monitor --port $port -b 115200"

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

command -v pio >/dev/null 2>&1 || {
  echo "pio not found - run scripts/setup_dev.sh first (pip install platformio)" >&2
  exit 1
}

# Candidate ports: ttyACM* (native USB, e.g. a genuine Nano/ATmega16U2) and
# ttyUSB* (USB-serial bridge, e.g. a CH340 Nano clone). Never hardcode a name.
ports=()
for p in /dev/ttyACM* /dev/ttyUSB*; do
  [ -e "$p" ] && ports+=("$p")
done

if [ "$list_ports" -eq 1 ]; then
  if [ "${#ports[@]}" -eq 0 ]; then
    echo "No serial ports found"
  else
    printf '%s\n' "${ports[@]}"
  fi
  exit 0
fi

if [ "$compile_only" -eq 0 ]; then
  if [ "${#ports[@]}" -eq 0 ]; then
    echo "No Arduino detected (no /dev/ttyACM* or /dev/ttyUSB*)." >&2
    echo "Plug in the Arduino over USB, then re-run. Check with: ls /dev/ttyACM* /dev/ttyUSB*" >&2
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

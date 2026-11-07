#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Unit-test the shared firmware core (firmware/arduino/MavCore) on the laptop.
# No Arduino and no toolchain needed: the core is board-agnostic C++ that
# implements firmware/arduino/protocol.md (parser, verb dispatch, watchdog).
g++ -std=c++11 -Wall -Wextra \
    -I firmware/arduino/MavCore/src \
    firmware/arduino/MavCore/test/test_protocol.cpp \
    firmware/arduino/MavCore/src/MavCore.cpp \
    -o /tmp/mav_proto_test
/tmp/mav_proto_test

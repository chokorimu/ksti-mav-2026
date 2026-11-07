#include "MavCore.h"

// C headers, not <cstdlib>/<cstring>: the PlatformIO AVR toolchain ships
// avr-libc's C headers but not the C++ standard library headers, and on a
// 2 KB-RAM microcontroller the C ones are the idiomatic choice anyway.
#include <stdlib.h>
#include <string.h>

namespace mav {

LineProtocol::LineProtocol()
    : cmd_(), last_verb_(0), timed_out_(true), bad_lines_(0), last_valid_ms_(0) {}

bool LineProtocol::is_space(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

bool LineProtocol::feed(const char* line, unsigned long now_ms) {
  char buf[64];
  size_t n = strlen(line);
  while (n > 0 && is_space(line[n - 1])) --n;
  if (n == 0 || n >= sizeof(buf)) {
    ++bad_lines_;
    return false;
  }
  memcpy(buf, line, n);
  buf[n] = '\0';

  char* verb_end = strchr(buf, ' ');
  if (verb_end == nullptr) {
    // Single-token line: only "S" is valid.
    if (strcmp(buf, "S") == 0) {
      last_verb_ = 'S';
      timed_out_ = false;  // a valid line re-arms the watchdog
      last_valid_ms_ = now_ms;
      return true;
    }
    ++bad_lines_;
    return false;
  }
  *verb_end = '\0';
  char* rest = verb_end + 1;
  while (*rest == ' ') ++rest;

  const bool is_w = strcmp(buf, "W") == 0;
  const bool is_d = strcmp(buf, "D") == 0;
  if (!is_w && !is_d) {
    ++bad_lines_;
    return false;
  }

  if (!parse_two_floats(rest, cmd_.a, cmd_.b)) {
    ++bad_lines_;
    return false;
  }

  last_verb_ = is_w ? 'W' : 'D';
  timed_out_ = false;
  last_valid_ms_ = now_ms;
  return true;
}

bool LineProtocol::parse_two_floats(char* rest, float& a, float& b) {
  char* end1 = nullptr;
  const double v1 = strtod(rest, &end1);
  if (end1 == rest) return false;  // no number at all
  while (*end1 == ' ') ++end1;

  char* end2 = nullptr;
  const double v2 = strtod(end1, &end2);
  if (end2 == end1) return false;  // second number missing
  while (*end2 == ' ') ++end2;
  if (*end2 != '\0') return false;  // trailing garbage

  a = static_cast<float>(v1);
  b = static_cast<float>(v2);
  return true;
}

bool LineProtocol::tick(unsigned long now_ms) {
  if (timed_out_) return false;
  // Unsigned subtraction: correct across the millis() rollover too.
  if (now_ms - last_valid_ms_ > MAV_WATCHDOG_MS) {
    timed_out_ = true;
    return true;
  }
  return false;
}

bool LineProtocol::safe_required() const {
  return timed_out_ || last_verb_ == 'S';
}

}  // namespace mav

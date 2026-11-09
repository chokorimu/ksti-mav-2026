// Host-side unit test for MavCore: no Arduino, no toolchain needed.
//
// Run from firmware/arduino/MavCore/:
//   g++ -std=c++11 -Wall -Wextra -I src test/test_protocol.cpp src/MavCore.cpp -o /tmp/mav_proto_test
//   /tmp/mav_proto_test

#include <cstdio>

#include "MavCore.h"

static int failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                 \
    }                                                             \
  } while (0)

int main() {
  using mav::LineProtocol;

  {  // boot state is safe
    LineProtocol p;
    CHECK(p.safe_required());
    CHECK(p.last_verb() == 0);
    CHECK(p.bad_lines() == 0);
  }
  {  // W line (prototype)
    LineProtocol p;
    CHECK(p.feed("W 0.400 0.600", 1000));
    CHECK(!p.safe_required());
    CHECK(p.last_verb() == 'W');
    CHECK(p.command().a > 0.399f && p.command().a < 0.401f);
    CHECK(p.command().b > 0.599f && p.command().b < 0.601f);
    CHECK(p.bad_lines() == 0);
  }
  {  // negative speeds (reverse)
    LineProtocol p;
    CHECK(p.feed("W -0.500 -0.500", 1000));
    CHECK(p.command().a < 0.0f && p.command().b < 0.0f);
  }
  {  // D line (final)
    LineProtocol p;
    CHECK(p.feed("D 0.2915 0.500", 1000));
    CHECK(p.last_verb() == 'D');
    CHECK(p.command().a > 0.2914f && p.command().a < 0.2916f);
    CHECK(p.command().b > 0.499f && p.command().b < 0.501f);
  }
  {  // S stops
    LineProtocol p;
    CHECK(p.feed("W 0.5 0.5", 1000));
    CHECK(!p.safe_required());
    CHECK(p.feed("S", 1100));
    CHECK(p.safe_required());
  }
  {  // CRLF tolerated (the Serial Monitor's default line ending)
    LineProtocol p;
    CHECK(p.feed("W 0.500 0.500\r", 1000));
    CHECK(p.last_verb() == 'W');
  }
  {  // extra spaces tolerated
    LineProtocol p;
    CHECK(p.feed("W  0.500   0.500", 1000));
    CHECK(p.last_verb() == 'W');
  }
  {  // malformed lines are counted and ignored
    LineProtocol p;
    CHECK(!p.feed("W 0.5", 1000));      // one field
    CHECK(!p.feed("X 1 2", 1000));      // unknown verb
    CHECK(!p.feed("W abc def", 1000));  // not numbers
    CHECK(!p.feed("W 1 2 x", 1000));    // trailing garbage
    CHECK(!p.feed("", 1000));           // empty
    CHECK(!p.feed("S 1", 1000));        // S with fields
    CHECK(p.bad_lines() == 6);
    CHECK(p.safe_required());  // never received a valid line
  }
  {  // watchdog expires once, then a valid line re-arms
    LineProtocol p;
    CHECK(p.feed("W 0.5 0.5", 1000));
    CHECK(!p.tick(1400));
    CHECK(p.tick(1501));         // > 500 ms without a valid line
    CHECK(!p.tick(1600));        // fires exactly once
    CHECK(p.safe_required());
    CHECK(p.feed("W 0.2 0.2", 1700));
    CHECK(!p.safe_required());
    CHECK(!p.tick(2000));
  }
  {  // bad lines do NOT reset the watchdog (protocol.md)
    LineProtocol p;
    CHECK(p.feed("W 0.5 0.5", 1000));
    CHECK(!p.feed("garbage", 1400));
    CHECK(p.tick(1501));
  }
  {  // overlong lines are rejected, not overflowed
    LineProtocol p;
    char long_line[128];
    for (int i = 0; i < 127; ++i) long_line[i] = 'W';
    long_line[127] = '\0';
    CHECK(!p.feed(long_line, 1000));
    CHECK(p.bad_lines() == 1);
  }

  {  // speed_to_pwm: uncalibrated and stop are BOTH 0, never a moving state
    CHECK(mav::speed_to_pwm(0.3f, 0.0f, 40.0f, 255) == 0);   // not calibrated
    CHECK(mav::speed_to_pwm(0.3f, -5.0f, 40.0f, 255) == 0);  // nonsense gain
    CHECK(mav::speed_to_pwm(0.0f, 300.0f, 40.0f, 255) == 0);  // stop, not deadband
  }
  {  // speed_to_pwm: the deadband is applied ABOVE zero, both directions
    // 200 PWM per m/s, 40 counts of breakaway, 0.25 m/s -> 40 + 50 = 90.
    CHECK(mav::speed_to_pwm(0.25f, 200.0f, 40.0f, 255) == 90);
    CHECK(mav::speed_to_pwm(-0.25f, 200.0f, 40.0f, 255) == -90);
    // Zero deadband must reproduce the old single-gain behaviour exactly, so an
    // unmeasured deadband cannot silently change how the robot drives.
    CHECK(mav::speed_to_pwm(0.25f, 200.0f, 0.0f, 255) == 50);
    // A negative deadband is a measurement error, not a direction change.
    CHECK(mav::speed_to_pwm(0.25f, 200.0f, -40.0f, 255) == 50);
  }
  {  // speed_to_pwm: clamps, and keeps the sign through the clamp
    CHECK(mav::speed_to_pwm(5.0f, 200.0f, 40.0f, 255) == 255);
    CHECK(mav::speed_to_pwm(-5.0f, 200.0f, 40.0f, 255) == -255);
  }
  {  // the EXACT bytes SerialActuator writes must parse (host<->firmware contract)
    // SerialActuator builds `f"W {left:.3f} {right:.3f}\n"`. If that stops
    // matching the parser, the robot receives valid-looking traffic and never
    // moves - which is exactly the failure a mismatched reference sketch caused
    // on 10 Nov 2026 (`sscanf("M %d %d")` against "W 0.277 0.263").
    LineProtocol p;
    CHECK(p.feed("W 0.277 0.263", 1000));
    CHECK(p.last_verb() == 'W');
    CHECK(!p.safe_required());
    const float l = p.command().a;
    const float r = p.command().b;
    CHECK(l > 0.2765f && l < 0.2775f);
    CHECK(r > 0.2625f && r < 0.2635f);
    // Signed, and the sign must survive: reverse is a real command.
    CHECK(p.feed("W -0.300 -0.300", 1010));
    CHECK(p.command().a < -0.2995f && p.command().a > -0.3005f);
    // "S" must stop immediately rather than wait for the watchdog, because the
    // host's SafeActuator.stop() writes it and expects the car to be stopped.
    CHECK(p.feed("S", 1020));
    CHECK(p.safe_required());
  }
  if (failures == 0) {
    std::printf("all MavCore protocol tests passed\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", failures);
  return 1;
}

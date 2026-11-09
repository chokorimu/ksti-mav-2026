// MavCore - shared firmware core for the KSTI MAV Arduino.
//
// Implements firmware/arduino/protocol.md: line buffering is done by the
// sketch, this class parses one line, dispatches the verb, and runs the
// watchdog. Board-agnostic on purpose: no Arduino headers, so the whole
// core is unit-tested on the laptop with g++ (see test/test_protocol.cpp).
// The hardware layer (pins, PWM, servo) lives in the .ino sketches.

#ifndef MAV_CORE_H
#define MAV_CORE_H

namespace mav {

// Watchdog window from protocol.md. A sketch may override before including.
#ifndef MAV_WATCHDOG_MS
#define MAV_WATCHDOG_MS 500UL
#endif
// Wheel speed (m/s) -> PWM counts. Open loop on purpose: the hardware has no
// encoders, so this is the ONLY link between "m/s" and "PWM", and it is a
// physical measurement, never a guess.
//
// Why it takes two numbers and not one. A DC motor does not turn at all below a
// breakaway PWM: the L298N's drop plus static friction hold it still, and only
// above that threshold does speed rise roughly linearly with PWM. So
//
//     pwm = deadband + |speed| * pwm_per_mps          for speed != 0
//     pwm = 0                                         for speed == 0
//
// A single gain fitted at full throttle OVERSTATES the speed at low PWM (it
// assumes the line passes through the origin), and the KSTI policy drives at
// 0.15-0.3 m/s - the low end - so that error lands exactly where it hurts.
//
// speed == 0 returns 0 rather than `deadband`, so "stop" really means coast.
// pwm_per_mps <= 0 means "not calibrated" (still 0) and returns 0, which is the
// safe state rather than a moving one. Result is clamped to +-pwm_max.
int speed_to_pwm(float speed_mps, float pwm_per_mps, float deadband, int pwm_max);

// A parsed drive command. Field meaning depends on the verb (protocol.md):
//   'W' (prototype): a = left wheel speed m/s,  b = right wheel speed m/s
//   'D' (final):     a = steering angle rad,    b = speed m/s
// Values are NOT clamped here: the host already clamps, and the sketch
// clamps again when converting to PWM/servo (defense in depth).
struct DriveCommand {
  float a;
  float b;
};

class LineProtocol {
 public:
  LineProtocol();

  // One received line; the terminator is optional (trailing \r, \n, spaces
  // and tabs are stripped). now_ms is the current millis(). Returns true
  // when the line was valid. Malformed lines are counted and ignored, and
  // they do NOT reset the watchdog (protocol.md).
  bool feed(const char* line, unsigned long now_ms);

  // Call every loop with millis(). Returns true exactly once, when the
  // watchdog expires. A later valid line re-arms the outputs.
  bool tick(unsigned long now_ms);

  // True when the outputs must be in the safe state: at boot, after "S",
  // or while the watchdog has expired.
  bool safe_required() const;

  char last_verb() const { return last_verb_; }  // 'W', 'D', 'S', or 0 at boot
  const DriveCommand& command() const { return cmd_; }
  unsigned long bad_lines() const { return bad_lines_; }

 private:
  static bool is_space(char c);
  bool parse_two_floats(char* rest, float& a, float& b);

  DriveCommand cmd_;
  char last_verb_;
  bool timed_out_;
  unsigned long bad_lines_;
  unsigned long last_valid_ms_;
};

}  // namespace mav

#endif  // MAV_CORE_H

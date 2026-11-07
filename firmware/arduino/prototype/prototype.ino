// prototype.ino - KSTI MAV prototype firmware: 4 motors, skid steering.
// Speaks "W <left> <right>" (m/s) per firmware/arduino/protocol.md.
//
// STATUS: SCAFFOLD (7 Nov 2026). The control flow below is final; every
// hardware constant is EMPTY on purpose. Fill them from Q1
// (docs/open-questions.md) and record the measured calibration in
// docs/hardware.md - do not guess. This sketch does not compile until the
// Q1 block is filled and the toolchain is installed, and that is
// deliberate: a scaffold must not be able to move hardware it knows
// nothing about. Flashed as-is it cannot move anything (all constants 0/-1).

#include <MavCore.h>  // shared core: protocol.md parser + watchdog.
                     // Install once: copy firmware/arduino/MavCore into the
                     // Arduino sketchbook libraries/ folder (see README.md).

// ---------------- Q1: fill these (empty on purpose) ---------------------
// Arduino model: TODO(Q1) (Uno / Nano / ESP32 - decides the toolchain and
//                whether analogWrite or LEDC drives the motors)
// Motor driver:  TODO(Q1) (L298N / DRV8833 / TB6612 / 4WD shield)
// The 4 motors wire as 2 channels: both left wheels on the left channel,
// both right wheels on the right channel (skid steering by speed difference).
const int PIN_LEFT_PWM  = -1;    // TODO(Q1)
const int PIN_LEFT_DIR  = -1;    // TODO(Q1) -1 if the driver takes signed
const int PIN_RIGHT_PWM = -1;    //      PWM or a 2-pin (IN1/IN2) channel
const int PIN_RIGHT_DIR = -1;    // TODO(Q1)
const int PWM_MAX       = 0;     // TODO(Q1) 255 on AVR, 4095 on ESP32 LEDC
const float MPS_TO_PWM  = 0.0f;  // TODO(Q1) measured wheels-lifted -> docs/hardware.md
// ------------------------------------------------------------------------

static const unsigned long SERIAL_BAUD = 115200UL;  // protocol.md
static const size_t LINE_BUF = 64;                   // matches MavCore's buffer

mav::LineProtocol proto;
char line_buf[LINE_BUF];
size_t line_len = 0;

void apply_outputs();
int mps_to_pwm(float mps);
void stop_motors();
void set_motor(int pwm_pin, int dir_pin, int pwm);

void setup() {
  Serial.begin(SERIAL_BAUD);
  stop_motors();  // boot state: everything off (protocol.md)
}

void loop() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n') {
      line_buf[line_len] = '\0';
      proto.feed(line_buf, millis());  // the core strips the trailing \r
      line_len = 0;
    } else if (line_len < LINE_BUF - 1) {
      line_buf[line_len++] = c;  // overlong lines are dropped and counted
    }                            // as bad by the core, never overflowed
  }
  proto.tick(millis());  // true = watchdog just expired
  apply_outputs();
}

// ---- hardware layer: everything below is Q1 ----------------------------

void apply_outputs() {
  if (proto.safe_required()) {  // boot, "S", or watchdog
    stop_motors();
    return;
  }
  const mav::DriveCommand& cmd = proto.command();
  set_motor(PIN_LEFT_PWM, PIN_LEFT_DIR, mps_to_pwm(cmd.a));
  set_motor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, mps_to_pwm(cmd.b));
}

int mps_to_pwm(float mps) {
  int pwm = (int)(mps * MPS_TO_PWM + (mps >= 0.0f ? 0.5f : -0.5f));
  if (pwm > PWM_MAX) pwm = PWM_MAX;  // defense in depth: the host clamps too
  if (pwm < -PWM_MAX) pwm = -PWM_MAX;
  return pwm;
}

void stop_motors() {
  set_motor(PIN_LEFT_PWM, PIN_LEFT_DIR, 0);
  set_motor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, 0);
}

// TODO(Q1): implement for the real driver.
//   AVR:   analogWrite(pwm_pin, abs(pwm)); digitalWrite(dir_pin, pwm >= 0);
//   ESP32: LEDC channel per motor instead of analogWrite.
// A no-op body is the correct fail-safe for a scaffold.
void set_motor(int pwm_pin, int dir_pin, int pwm) {
  (void)pwm_pin;
  (void)dir_pin;
  (void)pwm;
}

// final.ino - KSTI MAV final firmware: 1 rear motor + 1 steering servo.
// Speaks "D <steer_rad> <speed>" (rad, m/s) per firmware/arduino/protocol.md.
//
// STATUS: SCAFFOLD (7 Nov 2026). The control flow below is final; every
// hardware constant is EMPTY on purpose. Fill them from Q1
// (docs/open-questions.md) and record the measured calibration in
// docs/hardware.md - do not guess. This sketch does not compile until the
// Q1 block is filled and the toolchain is installed, and that is
// deliberate: a scaffold must not be able to move hardware it knows
// nothing about. Flashed as-is it cannot move anything (all constants 0/-1).

#include <Servo.h>    // TODO(Q1): confirm the servo API for the chosen board
#include <MavCore.h>  // shared core: protocol.md parser + watchdog.
                     // Install once: copy firmware/arduino/MavCore into the
                     // Arduino sketchbook libraries/ folder (see README.md).

// ---------------- Q1: fill these (empty on purpose) ---------------------
// Arduino model: TODO(Q1) (Uno / Nano / ESP32)
// Motor driver:  TODO(Q1) (L298N / DRV8833 / TB6612 / ...)
// Servo model:   TODO(Q1) + its measured microsecond range
const int PIN_SERVO     = -1;  // TODO(Q1)
const int PIN_MOTOR_PWM = -1;  // TODO(Q1)
const int PIN_MOTOR_DIR = -1;  // TODO(Q1) -1 if the driver takes signed PWM
const int PWM_MAX       = 0;   // TODO(Q1) 255 on AVR, 4095 on ESP32 LEDC
const float MPS_TO_PWM  = 0.0f;  // TODO(Q1) measured wheels-lifted -> docs/hardware.md
// Servo range in microseconds, measured not guessed. steer_rad > 0 is a
// LEFT turn (same convention as src/mav/control/mapping.py).
const int SERVO_US_MIN     = 0;  // TODO(Q1) full left
const int SERVO_US_MAX     = 0;  // TODO(Q1) full right
const int SERVO_US_NEUTRAL = 0;  // TODO(Q1) wheels straight ahead
const float RAD_TO_US = 0.0f;    // TODO(Q1) = (SERVO_US_MAX - SERVO_US_MIN) / 2
                                 //                / max_steer_rad (configs/vehicle.yaml)
// ------------------------------------------------------------------------

static const unsigned long SERIAL_BAUD = 115200UL;  // protocol.md
static const size_t LINE_BUF = 64;                   // matches MavCore's buffer

mav::LineProtocol proto;
Servo steering;
char line_buf[LINE_BUF];
size_t line_len = 0;

void apply_outputs();
int mps_to_pwm(float mps);
int clamp_int(int v, int lo, int hi);
void stop_motor();
void set_motor(int pwm);

void setup() {
  Serial.begin(SERIAL_BAUD);
  steering.attach(PIN_SERVO);
  steering.writeMicroseconds(SERVO_US_NEUTRAL);  // boot = neutral (protocol.md)
  stop_motor();
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
    steering.writeMicroseconds(SERVO_US_NEUTRAL);
    stop_motor();
    return;
  }
  const mav::DriveCommand& cmd = proto.command();
  // cmd.a = steering angle (rad), cmd.b = speed (m/s)
  const int us = SERVO_US_NEUTRAL + (int)(cmd.a * RAD_TO_US + (cmd.a >= 0.0f ? 0.5f : -0.5f));
  steering.writeMicroseconds(clamp_int(us, SERVO_US_MIN, SERVO_US_MAX));
  set_motor(mps_to_pwm(cmd.b));
}

int mps_to_pwm(float mps) {
  int pwm = (int)(mps * MPS_TO_PWM + (mps >= 0.0f ? 0.5f : -0.5f));
  if (pwm > PWM_MAX) pwm = PWM_MAX;  // defense in depth: the host clamps too
  if (pwm < -PWM_MAX) pwm = -PWM_MAX;
  return pwm;
}

int clamp_int(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void stop_motor() {
  set_motor(0);
}

// TODO(Q1): implement for the real driver.
//   AVR:   analogWrite(PIN_MOTOR_PWM, abs(pwm)); digitalWrite(PIN_MOTOR_DIR, pwm >= 0);
//   ESP32: LEDC channel instead of analogWrite.
// A no-op body is the correct fail-safe for a scaffold.
void set_motor(int pwm) {
  (void)PIN_MOTOR_PWM;
  (void)PIN_MOTOR_DIR;
  (void)pwm;
}

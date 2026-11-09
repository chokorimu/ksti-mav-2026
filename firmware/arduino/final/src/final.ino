// final.ino - KSTI MAV final firmware: 1 rear motor + 1 steering servo.
// Speaks "D <steer_rad> <speed>" (rad, m/s) per firmware/arduino/protocol.md.
//
// Hardware (confirmed 7 Nov 2026, docs/hardware.md):
//   Arduino Uno (5 V logic, AVR), motor driver L298N, battery LiPo 3S.
//   The rear motor uses one L298N channel; the steering servo is on a
//   separate 5 V rail.
//
// STATUS: the pin map and the L298N motor stage are final. Still TODO(Q1):
// the servo model + its measured microsecond range, and the servo 5 V rail
// (BEC/UBEC - NOT the L298N's onboard regulator, NOT the Uno 5 V pin:
// servo stalls pull amps and brown out logic). Record answers in
// docs/hardware.md - do not guess.
//
// Wiring notes (see docs/hardware.md for the full list):
//   - L298N GND, Arduino GND and battery - MUST share one ground.
//   - Remove the ENA jumper on the L298N, or PWM speed control is off.
//   - The L298N logic is 5 V, same as the Uno: connect IN/EN directly.

#include <Arduino.h>
#include <Servo.h>    // Uno: the Servo library takes over Timer1 (PWM on
                     // 9/10 stops working - not needed on this robot).
#include <MavCore.h>  // shared core: protocol.md parser + watchdog.
                     // PlatformIO finds it in lib/; in the Arduino IDE,
                     // install lib/MavCore into the sketchbook libraries/.

// ---------------- pin map (our choice, 7 Nov 2026) ----------------------
// Wire exactly like this. If the robot is already wired differently, fix
// the constants here AND docs/hardware.md - never just one of them.
const int PIN_SERVO      = 5;   // any digital pin works for Servo on the Uno
const int PIN_MOTOR_ENA  = 3;   // PWM ~490 Hz, L298N channel A = rear motor
const int PIN_MOTOR_IN1  = 2;
const int PIN_MOTOR_IN2  = 4;
// ------------------------------------------------------------------------

const int PWM_MAX       = 255;    // Uno/AVR: analogWrite range

// ---------------- speed calibration (MEASURE - do not guess) -------------
// MPS_TO_PWM is PWM counts per m/s ABOVE the breakaway point; PWM_DEADBAND is
// the breakaway PWM itself, below which the motor does not turn at all.
//     pwm = PWM_DEADBAND + mps * MPS_TO_PWM      (and 0 for mps == 0)
//
// Two numbers, not one, because a DC motor is not linear from zero: the L298N's
// voltage drop and static friction hold the car still until the PWM crosses a
// threshold. A single gain measured at full throttle therefore OVERSTATES the
// speed at low PWM, and the policy drives at 0.15-0.3 m/s - the low end.
//
// Measure with the car ON THE FLOOR (a lifted wheel spins faster than a loaded
// one, so it is the wrong number) using
// `python -m mav.tools.calibrate_speed --drive ackermann`, then record both
// values in docs/hardware.md.
const float MPS_TO_PWM   = 0.0f;  // TODO: measured -> docs/hardware.md
const float PWM_DEADBAND = 0.0f;  // TODO: measured -> docs/hardware.md

// MPS_TO_PWM == 0 means "not measured" (see the boot warning in setup()).
const bool SPEED_CALIBRATED = MPS_TO_PWM > 0.0f;
// Servo range in microseconds, measured not guessed. steer_rad > 0 is a
// LEFT turn (same convention as src/mav/control/mapping.py).
//
// SERVO_US_NEUTRAL defaults to 1500, the Servo library's own DEFAULT_PULSE_WIDTH
// and the centre of virtually every hobby servo. That is not a measurement, but
// it IS the safe value: the previous version had 0 here, and Servo::writeMicroseconds()
// clamps to MIN_PULSE_WIDTH = 544, so "boot = neutral" actually booted the steering
// HARD OVER to one side. Until MIN/MAX are measured the car cannot steer, but it
// must at least sit straight.
const int SERVO_US_NEUTRAL = 1500;
const int SERVO_US_MIN     = 0;  // TODO(Q1) full left
const int SERVO_US_MAX     = 0;  // TODO(Q1) full right
const float RAD_TO_US = 0.0f;    // TODO(Q1) = (SERVO_US_MAX - SERVO_US_MIN) / 2
                                 //                / max_steer_rad (configs/vehicle.yaml)
// A range of 0 means "not measured". The firmware must refuse to steer with it
// rather than compute nonsense: with SERVO_US_MIN == SERVO_US_MAX == 0, clamp_int()
// pins the servo to 0 uS (-> 544 uS, hard lock) and the car would veer into the
// wall the moment a "D" line arrived. Boot stays neutral and the motor stays
// stopped until somebody fills the three values in.
const bool SERVO_CALIBRATED = SERVO_US_MIN > 0 && SERVO_US_MAX > SERVO_US_MIN && RAD_TO_US > 0.0f;
static const unsigned long SERIAL_BAUD = 115200UL;  // protocol.md (matters on
                                                    // the Uno: ATmega16U2, not
                                                    // native USB)
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
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  steering.attach(PIN_SERVO);
  steering.writeMicroseconds(SERVO_US_NEUTRAL);  // boot = neutral (protocol.md)
  stop_motor();
  if (!SPEED_CALIBRATED) {
    Serial.println(F("WARN: MPS_TO_PWM uncalibrated - motor will not turn."));
    Serial.println(F("      Run: python -m mav.tools.calibrate_speed --drive ackermann"));
  }
  if (!SERVO_CALIBRATED) {
    // Not fatal: the car holds neutral and stays stopped, which is safe. But it
    // must say so, because otherwise a valid "D" line looks like it did nothing.
    Serial.println(F("WARN: servo range uncalibrated - steering held neutral, motor stopped."));
    Serial.println(F("      Set SERVO_US_MIN / SERVO_US_MAX / RAD_TO_US."));
  }
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

// ---- hardware layer ----------------------------------------------------

void apply_outputs() {
  if (proto.safe_required()) {  // boot, "S", or watchdog
    steering.writeMicroseconds(SERVO_US_NEUTRAL);
    stop_motor();
    return;
  }
  const mav::DriveCommand& cmd = proto.command();
  // cmd.a = steering angle (rad), cmd.b = speed (m/s)
  if (!SERVO_CALIBRATED) {
    // No measured servo range: hold neutral and stop. Steering on an
    // uncalibrated range would command 0 uS -> 544 uS = hard lock. Failing
    // still is the safe behaviour; the dashboard/log shows why.
    steering.writeMicroseconds(SERVO_US_NEUTRAL);
    stop_motor();
    return;
  }
  const int us = SERVO_US_NEUTRAL + (int)(cmd.a * RAD_TO_US + (cmd.a >= 0.0f ? 0.5f : -0.5f));
  steering.writeMicroseconds(clamp_int(us, SERVO_US_MIN, SERVO_US_MAX));
  set_motor(mps_to_pwm(cmd.b));
}

// The conversion itself lives in MavCore so it is unit-tested on the laptop
// (scripts/test_firmware_core.sh) instead of only ever being compiled for the
// Uno. This wrapper just binds the measured constants.
int mps_to_pwm(float mps) {
  return mav::speed_to_pwm(mps, MPS_TO_PWM, PWM_DEADBAND, PWM_MAX);
}

int clamp_int(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void stop_motor() {
  set_motor(0);
}

// L298N output stage (channel A = rear motor). pwm > 0 forward, pwm < 0
// reverse, 0 = coast. Braking (IN1 = IN2 = HIGH) is possible but not
// needed at v1.
void set_motor(int pwm) {
  if (pwm > 0) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
  }
  analogWrite(PIN_MOTOR_ENA, (uint8_t)abs(pwm));  // Uno: 0..255
}

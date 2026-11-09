// prototype.ino - KSTI MAV prototype firmware: 4 motors, skid steering.
// Speaks "W <left> <right>" (m/s) per firmware/arduino/protocol.md.
//
// Hardware (confirmed 7 Nov 2026, docs/hardware.md):
//   Arduino Uno (5 V logic, AVR), motor driver L298N (dual H-bridge),
//   battery LiPo 3S (11.1 V nominal) on the L298N +12 V input.
// The 4 motors wire as 2 channels: both left wheels on L298N channel A,
// both right wheels on channel B (skid steering by speed difference).
//
// STATUS: the pin map and the L298N output stage are final. The only
// remaining unknown is the speed calibration (MPS_TO_PWM + PWM_DEADBAND),
// which must be MEASURED ON THE FLOOR and recorded in docs/hardware.md - do
// not guess, and do not use a wheels-lifted measurement (a lifted wheel spins
// faster than a loaded one, so it is the wrong number).
//
// Wiring notes (see docs/hardware.md for the full list):
//   - L298N GND, Arduino GND and battery - MUST share one ground.
//   - Remove the ENA/ENB jumpers on the L298N, or PWM speed control is off.
//   - The L298N logic is 5 V, same as the Uno: connect IN/EN directly.
//   - The L298N drops ~2-3 V, so the motors see ~8-9 V from a 3S pack.

#include <Arduino.h>
#include <MavCore.h>  // shared core: protocol.md parser + watchdog.
                     // PlatformIO finds it in lib/; in the Arduino IDE,
                     // install lib/MavCore into the sketchbook libraries/.

// ---------------- pin map (our choice, 7 Nov 2026) ----------------------
// Wire exactly like this. If the robot is already wired differently, fix
// the constants here AND docs/hardware.md - never just one of them.
// ENA/ENB use the Uno's ~490 Hz PWM pins (3, 9, 10, 11); pins 5/6 run at
// ~980 Hz and are deliberately left alone.
const int PIN_ENA = 3;   // PWM, L298N channel A = LEFT motors
const int PIN_IN1 = 2;
const int PIN_IN2 = 4;
const int PIN_ENB = 11;  // PWM, L298N channel B = RIGHT motors
const int PIN_IN3 = 7;
const int PIN_IN4 = 8;
// ------------------------------------------------------------------------

const int PWM_MAX       = 255;    // Uno/AVR: analogWrite range

// ---------------- speed calibration (MEASURE - do not guess) -------------
// MPS_TO_PWM is PWM counts per m/s ABOVE the breakaway point; PWM_DEADBAND is
// the breakaway PWM itself, below which this motor does not turn at all.
//     pwm = PWM_DEADBAND + mps * MPS_TO_PWM      (and 0 for mps == 0)
//
// Two numbers, not one, because a DC motor is not linear from zero: the L298N's
// voltage drop and static friction hold the car still until the PWM crosses a
// threshold. A single gain measured at full throttle therefore OVERSTATES the
// speed at low PWM, and the policy drives at 0.15-0.3 m/s - the low end. That
// is what makes a one-number calibration either refuse to creep or lurch.
//
// Measure with the car ON THE FLOOR, because a lifted wheel spins faster than a
// loaded one and would give the wrong number. Use
// `python -m mav.tools.calibrate_speed`, then record both values in
// docs/hardware.md.
const float MPS_TO_PWM   = 0.0f;  // TODO: measured -> docs/hardware.md
const float PWM_DEADBAND = 0.0f;  // TODO: measured -> docs/hardware.md

// MPS_TO_PWM == 0 means "not measured". Without this flag the robot fails
// SILENTLY: every speed command becomes PWM 0, the wheels never turn, and it
// looks like a software bug rather than an uncalibrated robot.
const bool SPEED_CALIBRATED = MPS_TO_PWM > 0.0f;
static const unsigned long SERIAL_BAUD = 115200UL;  // protocol.md (matters on
                                                    // the Uno: ATmega16U2, not
                                                    // native USB)
static const size_t LINE_BUF = 64;                   // matches MavCore's buffer

mav::LineProtocol proto;
char line_buf[LINE_BUF];
size_t line_len = 0;

void apply_outputs();
int mps_to_pwm(float mps);
void stop_motors();
void set_motor(int en_pin, int in1_pin, int in2_pin, int pwm);

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  stop_motors();  // boot state: everything off (protocol.md)
  if (!SPEED_CALIBRATED) {
    // Not fatal, but never silent: with MPS_TO_PWM == 0 every speed command
    // becomes PWM 0, so the robot receives valid commands and never moves.
    Serial.println(F("WARN: MPS_TO_PWM uncalibrated - wheels will not turn."));
    Serial.println(F("      Run: python -m mav.tools.calibrate_speed"));
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
    stop_motors();
    return;
  }
  const mav::DriveCommand& cmd = proto.command();
  set_motor(PIN_ENA, PIN_IN1, PIN_IN2, mps_to_pwm(cmd.a));
  set_motor(PIN_ENB, PIN_IN3, PIN_IN4, mps_to_pwm(cmd.b));
}

// The conversion itself lives in MavCore so it is unit-tested on the laptop
// (scripts/test_firmware_core.sh) instead of only ever being compiled for the
// Uno. This wrapper just binds the measured constants.
int mps_to_pwm(float mps) {
  return mav::speed_to_pwm(mps, MPS_TO_PWM, PWM_DEADBAND, PWM_MAX);
}

void stop_motors() {
  set_motor(PIN_ENA, PIN_IN1, PIN_IN2, 0);
  set_motor(PIN_ENB, PIN_IN3, PIN_IN4, 0);
}

// L298N output stage. pwm > 0 forward, pwm < 0 reverse, 0 = coast (both IN
// pins low; the motor freewheels). Braking (IN1 = IN2 = HIGH) is possible
// but not needed at v1.
void set_motor(int en_pin, int in1_pin, int in2_pin, int pwm) {
  if (pwm > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }
  analogWrite(en_pin, (uint8_t)abs(pwm));  // Uno: 0..255
}

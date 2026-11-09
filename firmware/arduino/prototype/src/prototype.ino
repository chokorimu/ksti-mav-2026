// prototype.ino - KSTI MAV prototype firmware: 4 motors, skid steering.
// Speaks "W <left> <right>" (m/s) per firmware/arduino/protocol.md.
//
// Hardware (confirmed 10 Nov 2026, docs/hardware.md):
//   Arduino **Nano** (ATmega328P, 5 V logic), **2x L298N** (one per axle pair),
//   4 motors one per wheel, battery LiPo 3S (11.1 V nominal) on the L298N +12 V
//   input. The two L298Ns are driven in parallel: driver 1 channel A + driver 2
//   channel A are the LEFT pair, channel B of each is the RIGHT pair.
//
//   The pin map below is the one on the actual robot (supplied 10 Nov 2026).
//   It is NOT the Uno map this file used to carry: that version drove only two
//   channels on pins 2/4/7/8, i.e. the wrong pins on this hardware.
//
// Wiring notes (see docs/hardware.md for the full list):
//   - L298N GND, Arduino GND and battery - MUST share one ground.
//   - Remove the ENA/ENB jumpers on BOTH L298Ns, or PWM speed control is off.
//   - The L298N logic is 5 V, same as the Nano: connect IN/EN directly.
//   - The L298N drops ~2-3 V, so the motors see ~8-9 V from a 3S pack.
#include <Arduino.h>
#include <MavCore.h>  // shared core: protocol.md parser + watchdog + speed map.
                     // PlatformIO finds it in lib/; in the Arduino IDE,
                     // install lib/MavCore into the sketchbook libraries/.

// ---------------- pin map (the robot's wiring, 10 Nov 2026) -------------
// Wire exactly like this. If the robot is wired differently, fix the constants
// here AND docs/hardware.md - never just one of them.
//
// A2/A0/A1/A3 are the Nano's analog pins used as plain digital outputs: legal
// on the ATmega328P, and they are what the harness already uses.
//
// ENA/ENB/ENA2/ENB2 are all on the Nano's hardware-PWM pins (3, 11, 9, 10), so
// analogWrite() really varies the duty cycle. Putting an EN pin on a non-PWM pin
// would make analogWrite() act as a plain on/off and quietly lose speed control.
const int PIN_ENA   = 3;    // driver 1, channel A = LEFT pair
const int PIN_IN1   = 4;
const int PIN_IN2   = 5;
const int PIN_ENB   = 11;   // driver 1, channel B = RIGHT pair
const int PIN_IN3   = A2;
const int PIN_IN4   = A3;

const int PIN_ENA2  = 9;    // driver 2, channel A = LEFT pair
const int PIN_IN1_2 = A0;
const int PIN_IN2_2 = 7;
const int PIN_ENB2  = 10;   // driver 2, channel B = RIGHT pair
const int PIN_IN3_2 = 8;
const int PIN_IN4_2 = A1;

// Fan: wired but not used in a run. Driven LOW at boot so it can never be left
// on by a reset mid-test.
const int PIN_FAN   = 6;
// ------------------------------------------------------------------------

const int PWM_MAX = 255;  // Uno/Nano AVR: analogWrite range

// Per-motor direction. Flip a flag if that wheel spins the wrong way when the
// robot is told to go forward. This is the software alternative to swapping the
// motor leads, and it keeps the wiring harness untouched.
// Set from the robot as supplied (10 Nov 2026): all four inverted, which means
// the motors are mounted mirrored. If one wheel alone is wrong, flip only that
// one - do not flip all four to "fix" a single wheel.
const bool INV_1A = true;   // driver 1 - motor A (left)
const bool INV_1B = true;   // driver 1 - motor B (right)
const bool INV_2A = true;   // driver 2 - motor A (left)
const bool INV_2B = true;   // driver 2 - motor B (right)

// ---------------- speed calibration (MEASURE - do not guess) -------------
// MPS_TO_PWM is PWM counts per m/s ABOVE the breakaway point; PWM_DEADBAND is
// the breakaway PWM itself, below which the motors do not turn at all.
//     pwm = PWM_DEADBAND + mps * MPS_TO_PWM      (and 0 for mps == 0)
//
// Two numbers, not one, because a DC motor is not linear from zero: the L298N's
// voltage drop and static friction hold the car still until the PWM crosses a
// threshold. A single gain measured at full throttle therefore OVERSTATES the
// speed at low PWM, and the policy drives at 0.15-0.3 m/s - the low end. That
// is what makes a one-number calibration either refuse to creep or lurch.
//
// The previous revision of this file pinned every command to FULL_PWM (255),
// which threw away speed control entirely: the policy's corner speed schedule,
// the max_speed_mps ceiling and this calibration all had no effect. Speed is now
// proportional, which is the only way the lane follower can slow for a curve.
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
                                                    // the Nano: ATmega16U2/CH340,
                                                    // not native USB)
static const size_t LINE_BUF = 64;                   // matches MavCore's buffer

mav::LineProtocol proto;
char line_buf[LINE_BUF];
size_t line_len = 0;

void apply_outputs();
int mps_to_pwm(float mps);
void stop_motors();
void set_motor(int en_pin, int in1_pin, int in2_pin, int pwm, bool invert);

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  pinMode(PIN_ENA2, OUTPUT);
  pinMode(PIN_IN1_2, OUTPUT);
  pinMode(PIN_IN2_2, OUTPUT);
  pinMode(PIN_ENB2, OUTPUT);
  pinMode(PIN_IN3_2, OUTPUT);
  pinMode(PIN_IN4_2, OUTPUT);

  pinMode(PIN_FAN, OUTPUT);
  digitalWrite(PIN_FAN, LOW);

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
  // `cmd.a` is the LEFT wheel speed, `cmd.b` the RIGHT, both in m/s. Each L298N
  // gets the same pair: driver 1 A + driver 2 A = left, B of each = right.
  set_motor(PIN_ENA,  PIN_IN1,   PIN_IN2,   mps_to_pwm(cmd.a), INV_1A);
  set_motor(PIN_ENB,  PIN_IN3,   PIN_IN4,   mps_to_pwm(cmd.b), INV_1B);
  set_motor(PIN_ENA2, PIN_IN1_2, PIN_IN2_2, mps_to_pwm(cmd.a), INV_2A);
  set_motor(PIN_ENB2, PIN_IN3_2, PIN_IN4_2, mps_to_pwm(cmd.b), INV_2B);
}

// The conversion itself lives in MavCore so it is unit-tested on the laptop
// (scripts/test_firmware_core.sh) instead of only ever being compiled for the
// board. This wrapper just binds the measured constants.
int mps_to_pwm(float mps) {
  return mav::speed_to_pwm(mps, MPS_TO_PWM, PWM_DEADBAND, PWM_MAX);
}

void stop_motors() {
  set_motor(PIN_ENA,  PIN_IN1,   PIN_IN2,   0, INV_1A);
  set_motor(PIN_ENB,  PIN_IN3,   PIN_IN4,   0, INV_1B);
  set_motor(PIN_ENA2, PIN_IN1_2, PIN_IN2_2, 0, INV_2A);
  set_motor(PIN_ENB2, PIN_IN3_2, PIN_IN4_2, 0, INV_2B);
}

// L298N output stage. pwm > 0 forward, pwm < 0 reverse, 0 = coast (both IN
// pins low; the motor freewheels). Braking (IN1 = IN2 = HIGH) is possible
// but not needed at v1.
//
// `invert` flips the DIRECTION only. It must not touch the magnitude, or a
// motor flagged as reversed would also be a motor flagged as slower.
void set_motor(int en_pin, int in1_pin, int in2_pin, int pwm, bool invert) {
  if (invert) pwm = -pwm;
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
  analogWrite(en_pin, (uint8_t)abs(pwm));  // Nano: 0..255
}

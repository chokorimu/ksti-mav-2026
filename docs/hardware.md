# Hardware Notes

Only facts confirmed by the user are in "Confirmed". Everything else is a question or an unverified expectation.

## Confirmed
- Computer: Jetson Orin Nano, headless, SSH only.
- JetPack 6.2.3 (L4T R36.5, from `/etc/nv_tegra_release` and `apt show nvidia-jetpack`).
- Current power mode: `MAXN_SUPER`.
- Prototype robot: camera + 4 motors, one per wheel.
- Final robot (planned): 1 servo for steering + 1 motor for the rear wheel.
- Camera: CSI interface, Raspberry Pi camera module (user, 7 Nov 2026). Exact module
  (v1/v2/v3/GS) still to verify on the device; `nvarguscamerasrc` normally handles all of them.
- **Prototype track width: 250 mm** (user, 9 Nov 2026) — left wheel centre to right
  wheel centre, measured. It feeds `DifferentialMapper`, so it must be re-measured for
  `final`: `configs/vehicle.yaml` is one file for both drives, and switching `drive:` to
  `ackermann` silently stops using `track_width_mm` while requiring `wheel_base_mm` and
  `max_steer_deg` (both still `null`, so `load_vehicle_params()` will refuse to start
  `final` until they are measured — that refusal is the reminder).
- **Emergency stop cuts the MOTOR rail only; the Jetson stays powered** (user,
  9 Nov 2026). That is the right split and it constrains the design: the e-stop must
  interrupt the L298N supply (or its ENA/ENB enables) in hardware, independent of any
  software, while the Jetson keeps running so the checkpoint count and the state
  machine survive the stop. A software-only stop is not acceptable — if the Jetson
  hangs, nothing would stop the motors.
- **Competition scope: ARC (Autonomous Race Challenge), rules §8** (user, 9 Nov 2026).
  Not AR. Consequences: in the ARC the cones ARE the checkpoints ("avoiding an
  obstacle counts as passing the checkpoint"), so `cp` is the number of cones passed
  and `ConePassTracker` is what produces the score; hitting a cone is a violation
  (k=0.5); and the off-track threshold is **50%** of the body outside the edge line
  (AR is 30%), which sets the lateral budget. `S_ARC` can roughly double the total.
  Note `STP = 0.25` is Ackermann-only, so the skid-steering prototype scores 0 there
  and `final` must be Ackermann to get it.
- **Final drivetrain: the single rear motor drives both rear wheels through a mechanical
  differential (user, 7 Nov 2026).
- Motor/servo control path: **Arduino over USB serial** (user decision, 7 Nov 2026).
  Jetson stays the CV brain; the Arduino drives the motor driver and servo. The Jetson
  40-pin header is NOT used for actuators (3.3 V logic, mA-level current, not real-time).
  The Arduino firmware goes in `firmware/arduino/` (compiles for the Uno; the speed
  calibration and the servo range are still open).
- **MCU: Arduino Uno** (user, 7 Nov 2026). ATmega328P, 16 MHz, 5 V logic, 2 KB RAM,
  31.5 KB flash. USB serial through the ATmega16U2, so the 115200 baud in
  `firmware/arduino/protocol.md` matters on this board. Verified firmware size:
  prototype 4966 B flash / 276 B RAM, final 5532 B / 317 B.
- **Motor driver: L298N** (user, 7 Nov 2026). Dual H-bridge, two channels (A and B),
  each channel IN1/IN2 + ENA. Logic is 5 V, same as the Uno: connect IN/EN directly,
  no level shifter. The L298N drops ~2-3 V across its outputs, so the motors see
  ~8-9 V from a 3S pack.
- **Battery: LiPo 3S** (user, 7 Nov 2026) = 11.1 V nominal, 12.6 V full, ~9.9-10.5 V
  empty (3.3-3.5 V per cell). Feeds the L298N +12 V input through a fuse.

## Electrical notes for the Arduino + L298N wiring (plain language)

- **Common ground is mandatory.** L298N GND, Arduino GND and battery - must all be one
  node. Without it nothing works, and it is the number one wiring bug.
- **Remove the ENA/ENB jumpers** on the L298N, otherwise both channels run at full
  speed and PWM speed control does nothing.
- **Prototype wiring (4 motors as 2 channels):** both left wheels on L298N channel A,
  both right wheels on channel B; skid steering is the speed difference between them.
  Pin map (our choice, 7 Nov 2026, wire exactly like this):
  ENA=3, IN1=2, IN2=4 (left) / ENB=11, IN3=7, IN4=8 (right). ENA/ENB use the Uno's
  ~490 Hz PWM pins; pins 5/6 (~980 Hz) deliberately unused.
- **Final wiring:** rear motor on channel A (ENA=3, IN1=2, IN2=4), steering servo
  signal on pin 5. The Servo library takes over Timer1, so PWM on pins 9/10 stops
  working - not needed on this robot.
- **Powering the Uno:** simplest is USB from the Jetson (it also carries the serial
  link). Alternative: the L298N's own 5 V output into the Uno's **5V pin** - never
  into VIN (the Uno's regulator needs ~6.5 V or more), and never with USB connected
  at the same time (that backfeeds the port).
- **Servo 5 V rail - still open (Q1):** NOT from the L298N's onboard regulator and NOT
  from the Uno 5 V pin. A stalling servo pulls hundreds of mA to amps and browns out
  the logic. Use a separate BEC/UBEC (5-6 V, >=2 A) from the same 3S pack.
- **LiPo handling:** fuse on the main path, low-voltage alarm, never below 3.0 V per
  cell, balance charge only, inspect the pack before every attempt.
- **Heat/current:** the L298N gets hot at high current. Watch it during the
  wheels-lifted test; the Jetson keeps its own separate power path (brownout rule in
  docs/research-plan.md).
- `scripts/setup_jetson.sh` runs on the Jetson (PyYAML 5.4.1 comes from the system packages).

## Verified on the Jetson (from the user's command output)
- Python 3.10.12.
- OpenCV 4.8.0 with GStreamer 1.20.3. The build info had no CUDA line, so assume OpenCV runs on CPU only.
- CUDA toolkit config 12.6, cuDNN 9.3, TensorRT 10.3.0.30 runtime libraries and ONNX parser (`libnvonnxparsers10`).
- Power mode: `MAXN_SUPER` (mode id 2), the highest power mode.
- CSI camera alive: `nvarguscamerasrc` works (user, 7 Nov 2026). Exact module still to identify.
- Not seen in the package list (may be missing, verify): TensorRT development packages, `python3-libnvinfer`, `trtexec`, `nvcc`.

## To verify next
```bash
python3 -c "import tensorrt; print(tensorrt.__version__)"
ls /usr/src/tensorrt/bin/trtexec
ls /usr/local/cuda/bin/nvcc
dpkg -l | grep -E "nvidia-jetpack|python3-libnvinfer|libnvinfer-dev"
```
If these are missing, `sudo apt update && sudo apt install nvidia-jetpack` normally adds them. It is a large download, so do it on a good connection before competition day.

## Camera bring-up (CSI, Raspberry Pi module) — verify on the Jetson
```bash
# 1. Does the sensor come up at all? (needs a display or SSH with X for the preview window;
#    on a headless box use the fakesink variant and check for errors only)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 ! fakesink

# 2. Save a short clip headless (what we actually need for the detector):
python3 - <<'EOF'
import cv2
cap = cv2.VideoCapture(
    "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1",
    cv2.CAP_GSTREAMER)
out = cv2.VideoWriter("/tmp/csi_test.avi", cv2.VideoWriter_fourcc(*"MJPG"), 20.0, (640, 360))
n = 0
while n < 60:
    ok, frame = cap.read()
    if not ok:
        break
    out.write(cv2.resize(frame, (640, 360)))
    n += 1
cap.release(); out.release()
print("frames:", n)
EOF
```
If `nvarguscamerasrc` fails, check the camera ribbon seating and `dmesg | grep -i cam`; the exact
module (v1/v2/v3/GS) decides whether a driver/DTB update is needed on JetPack 6.2.3.


## Speed calibration: `MPS_TO_PWM` and `PWM_DEADBAND` (MEASURE, do not guess)
There are no encoders, so `m/s -> PWM` is open loop: it is the only link between a
commanded speed and a real one, and a wrong value is never noticed by the car — it
simply drives at the wrong speed, and the lane offsets and cone distances inherit
the error.

**Why two numbers.** A DC motor does not turn at all below a breakaway PWM: the
L298N's voltage drop plus static friction hold it still, and only above that does
speed rise roughly linearly with PWM.

```
pwm = PWM_DEADBAND + speed_mps * MPS_TO_PWM        (and 0 when the speed is 0)
```

The obvious one-number calibration — measure the top speed and set
`K = 255 / top` — draws that line through the ORIGIN and so overstates the speed at
low PWM. The policy drives at 0.15-0.3 m/s, the low end, which is exactly where that
error lands: the car either refuses to creep or lurches. With the two-number model
the low end is correct by construction.

**Measure on the floor, not wheels-lifted.** A lifted wheel spins faster than a
loaded one, so a wheels-lifted sweep measures the wrong quantity. The device has to
drive.

### Procedure
Safety first: e-stop in hand and tested, motors running only on the floor, a clear
straight run of at least 4 m, the same surface for every point, and the battery
charged the same way each time (a sagging pack changes the speed).

1. Flash the firmware with a **provisional** `MPS_TO_PWM` (300 is a fine starting
   point) and `PWM_DEADBAND = 0`. The provisional value only has to let the car move;
   the sweep converts each PWM target into the m/s command the firmware needs, so the
   fit does not depend on the provisional value being right.
2. Tape a start line and put a tape measure along the run.
3. Run:
   ```
   PYTHONPATH=src python -m mav.tools.calibrate_speed --port /dev/ttyACM0 \
       --k-current 300 --pwm 50 70 90 120 150 180
   ```
   (`--drive ackermann` for the `final` build, which speaks `D` instead of `W`.)
4. For each PWM: press Enter, the car runs for `--seconds`, it stops, you enter how
   far it travelled in metres. The tool streams the command at 20 Hz on purpose — the
   firmware's watchdog stops the motors after 500 ms without a valid line, so a
   one-off command would coast after half a second and be measured as far too slow.
5. The tool fits both constants, prints the residual, and shows the model against
   every measurement. Paste the two lines into the sketch, recompile, and record them
   here:

   ```
   MPS_TO_PWM   = ______   # measured <date>, <floor surface>, battery <V>
   PWM_DEADBAND = ______
   ```

### Reading the result
- **Any PWM that did not move the car** is evidence about the deadband, not a failed
  point: the tool raises the deadband to at least that PWM. Include two low levels for
  this reason.
- **Residual above ~50 mm/s** means the motor is not behaving linearly in that range
  (wheel slip, a binding wheel, or PWM 50 was under the breakaway). Re-run the points
  that disagree.
- **A negative breakaway** means the fitted line passed above the origin, which cannot
  be inverted; the tool clamps it to 0 and says so. Do not trust that run.
- **`PWM_DEADBAND` near or above the PWMs you drive at** means the car has almost no
  usable low range: raise `policy.v_min` in `configs/<profile>.yaml` rather than
  pretending the car can creep.
- Keep `max_speed_mps` in `configs/vehicle.yaml` below the speed at `PWM 255`, or the
  host will command a speed the motor cannot reach and the car will simply saturate.

## Still unknown (needed before writing the real actuator/camera code)
- Motor driver model(s), motor type, gearbox.
- Battery type, voltage, capacity; regulator(s) feeding the Jetson and the motors.
- Start button: which pin, which input.
- Is the steering linkage true Ackermann geometry (affects the 0.25 bonus **[A1]**)?
- Servo model, steering angle range, wheel base and track width (measured, in mm).
- **Servo pulse range** (`SERVO_US_MIN` / `SERVO_US_MAX` / `RAD_TO_US` in
  `final.ino`). Until measured, `SERVO_CALIBRATED` is false and the firmware holds
  the servo neutral and keeps the motor stopped — the car cannot steer. That is
  deliberate: the previous placeholder (0) commanded 544 µs, i.e. hard lock.
- **`MPS_TO_PWM` and `PWM_DEADBAND`** in both sketches. Until measured they are 0, and
  `MPS_TO_PWM = 0` means every speed command becomes PWM 0: the robot accepts valid
  commands and never moves. Both sketches now print a warning at boot instead of
  failing silently. Procedure in "Speed calibration" above.
- **Cone colour** (`obstacle.hues` in `configs/*.yaml`) — the detector's default is
  an unconfirmed orange band. Measure on the real cone [A11].
- **Cone diameter** — the swerve clearance (`avoid.clearance_m = 0.16`) assumes
  car half-width 0.10 m + cone radius ~0.04 m + margin. Re-check against the real
  cone, since hitting one is a violation in the ARC.

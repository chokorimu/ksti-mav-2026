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
- Final drivetrain: the single rear motor drives both rear wheels through a mechanical
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

## Still unknown (needed before writing the real actuator/camera code)
- Motor driver model(s), motor type, gearbox.
- Battery type, voltage, capacity; regulator(s) feeding the Jetson and the motors.
- Emergency stop circuit: what it cuts (motor power, or only a signal?).
- Start button: which pin, which input.
- Is the steering linkage true Ackermann geometry (affects the 0.25 bonus **[A1]**)?
- Servo model, steering angle range, wheel base and track width (measured, in mm).

# Hardware Notes

Only facts confirmed by the user are in "Confirmed". Everything else is a question or an unverified expectation.

## Confirmed
- Computer: Jetson Orin Nano, headless, SSH only.
- JetPack 6.2.3 (L4T R36.5, from `/etc/nv_tegra_release` and `apt show nvidia-jetpack`).
- Current power mode: `MAXN_SUPER`.
- Prototype robot: camera + 4 motors, one per wheel.
- Deploy robot (planned): 1 servo for steering + 1 motor for the rear wheel.
- Camera: CSI interface, Raspberry Pi camera module (user, 7 Nov 2026). Exact module
  (v1/v2/v3/GS) still to verify on the device; `nvarguscamerasrc` normally handles all of them.
- Deploy drivetrain: the single rear motor drives both rear wheels through a mechanical
  differential (user, 7 Nov 2026).
- Motor/servo control path: **Arduino over USB serial** (user decision, 7 Nov 2026).
  Jetson stays the CV brain; the Arduino drives the motor driver and servo. The Jetson
  40-pin header is NOT used for actuators (3.3 V logic, mA-level current, not real-time).
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

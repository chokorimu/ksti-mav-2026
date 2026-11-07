# Hardware Notes

Only facts confirmed by the user are in "Confirmed". Everything else is a question or an unverified expectation.

## Confirmed
- Computer: Jetson Orin Nano, headless, SSH only.
- JetPack 6.2.3 (L4T R36.5, from `/etc/nv_tegra_release` and `apt show nvidia-jetpack`).
- Current power mode: `MAXN_SUPER`.
- Prototype robot: camera + 4 motors, one per wheel.
- Deploy robot (planned): 1 servo for steering + 1 motor for the rear wheel.
- `scripts/setup_jetson.sh` runs on the Jetson (PyYAML 5.4.1 comes from the system packages).

## Verified on the Jetson (from the user's command output)
- Python 3.10.12.
- OpenCV 4.8.0 with GStreamer 1.20.3. The build info had no CUDA line, so assume OpenCV runs on CPU only.
- CUDA toolkit config 12.6, cuDNN 9.3, TensorRT 10.3.0.30 runtime libraries and ONNX parser (`libnvonnxparsers10`).
- Power mode: `MAXN_SUPER` (mode id 2), the highest power mode.
- Not seen in the package list (may be missing, verify): TensorRT development packages, `python3-libnvinfer`, `trtexec`, `nvcc`.

## To verify next
```bash
python3 -c "import tensorrt; print(tensorrt.__version__)"
ls /usr/src/tensorrt/bin/trtexec
ls /usr/local/cuda/bin/nvcc
dpkg -l | grep -E "nvidia-jetpack|python3-libnvinfer|libnvinfer-dev"
```
If these are missing, `sudo apt update && sudo apt install nvidia-jetpack` normally adds them. It is a large download, so do it on a good connection before competition day.

## Still unknown (needed before writing the real actuator/camera code)
- Camera model and interface (CSI or USB).
- What drives the motors and servo: driver wired to Jetson pins, PCA9685 over I2C, or a microcontroller (Arduino/ESP32) over USB serial.
- Motor driver model(s), motor type, gearbox.
- Battery type, voltage, capacity; regulator(s) feeding the Jetson and the motors.
- Emergency stop circuit: what it cuts (motor power, or only a signal?).
- Start button: which pin, which input.
- Does the single rear motor drive both rear wheels through a mechanical differential?
- Is the steering linkage true Ackermann geometry (affects the 0.25 bonus **[A1]**)?
- Servo model, steering angle range, wheel base and track width (measured, in mm).

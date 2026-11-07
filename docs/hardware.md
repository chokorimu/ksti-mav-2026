# Hardware Notes

Only facts confirmed by the user are in "Confirmed". Everything else is a question or an unverified expectation.

## Confirmed
- Computer: Jetson Orin Nano, headless, SSH only.
- JetPack 6.2.3 (L4T R36.5, from `/etc/nv_tegra_release` and `apt show nvidia-jetpack`).
- Prototype robot: camera + 4 motors, one per wheel.
- Deploy robot (planned): 1 servo for steering + 1 motor for the rear wheel.
- `scripts/setup_jetson.sh` runs on the Jetson (PyYAML 5.4.1 comes from the system packages).

## To verify on the Jetson (run and paste the output here)
```bash
python3 --version
python3 -c "import cv2; print(cv2.__version__)"
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -E "GStreamer|CUDA"
dpkg -l | grep -i -E "tensorrt|cuda-toolkit|libcudnn"
sudo nvpmodel -q
```

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

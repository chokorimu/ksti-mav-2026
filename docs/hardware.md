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
  The Arduino firmware goes in `firmware/arduino/` (the speed calibration and the
  servo range are still open).
- **MCU: Arduino NANO** (corrected 10 Nov 2026 - the robot's own sketch says Nano;
  it was recorded as Uno on 7 Nov 2026). ATmega328P, 16 MHz, 5 V logic, 2 KB RAM,
  30 KB flash usable. USB serial through a bridge chip (ATmega16U2 on a genuine
  board, CH340 on most clones), so the 115200 baud in
  `firmware/arduino/protocol.md` matters. `platformio.ini` now builds
  `board = nanoatmega328new`; if an upload times out on sync, the board has the
  OLD bootloader (57600 baud) - switch to `board = nanoatmega328`. Only the upload
  changes: both are the same chip. Verified firmware size: prototype 4830 B flash /
  280 B RAM.
- **Motor drivers: 2x L298N** (10 Nov 2026). The robot carries two, one per axle
  pair, driven in parallel: channel A of BOTH is the left pair, channel B of BOTH
  is the right pair. Four motors, one per wheel, so the prototype really is skid
  steering by speed difference. The old note below (one L298N, two channels)
  described the plan, not the robot.
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
- **Prototype wiring (2x L298N, 4 motors) - THE ROBOT'S ACTUAL PIN MAP (10 Nov 2026):**

  | Driver 1 | pin | Driver 2 | pin |
  |---|---|---|---|
  | ENA (left) | 3 | ENA2 (left) | 9 |
  | IN1 | 4 | IN1_2 | A0 |
  | IN2 | 5 | IN2_2 | 7 |
  | ENB (right) | 11 | ENB2 (right) | 10 |
  | IN3 | A2 | IN3_2 | 8 |
  | IN4 | A3 | IN4_2 | A1 |

  Fan on pin 6 (wired, unused, driven LOW at boot).
  All four enable pins (3, 11, 9, 10) are hardware-PWM pins on the ATmega328P; an EN
  pin on a non-PWM pin would make `analogWrite()` a plain on/off and silently lose
  speed control. A0/A1/A2/A3 are analog pins used as digital outputs, which is legal.
  The earlier map (ENA=3, IN1=2, IN2=4 / ENB=11, IN3=7, IN4=8) was a design choice
  that never matched this harness: it drove only two channels and used pins 2/4/7/8.
  **Speed is proportional** (`MPS_TO_PWM` + `PWM_DEADBAND`), not pinned to 255: the
  reference sketch this replaced sent full PWM for any non-zero command, which made
  the policy's corner speed schedule, the `max_speed_mps` ceiling and the calibration
  all do nothing.
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

## The CH340 Nano clone: its port is NOT /dev/ttyUSB0 (confirmed 10 Nov 2026)

**Symptom.** The board is on USB and bound to a driver, but the expected device
node is missing:

```
$ lsusb | grep -i 1a86
Bus 001 Device 010: ID 1a86:7523 QinHeng Electronics CH340 serial converter
$ ls /dev/ttyUSB*
No such file or directory
$ lsusb -t | grep -i ch34
|__ Port 4: Dev 13, If 0, Class=Vendor Specific Class, Driver=usb_ch341, 12M
```

**Answer: the port is `/dev/ttyCH341USB0`.** `usb_ch341` is not the kernel's
`ch341` driver - it is **WCH's own out-of-tree driver** (the chip's manufacturer,
`ch341ser_linux`), which names its device `ttyCH341USB0` instead of `ttyUSB0`.
`lsmod` showing `ch341 28672 0` (refcount 0) is consistent: the in-tree module is
loaded but has nothing bound, because the vendor driver got there first.

```
$ ls /dev/ttyCH341*
/dev/ttyCH341USB0
```

```bash
# Either pass it explicitly:
scripts/run.sh prototype --port /dev/ttyCH341USB0
# or let auto-detection find it (list_serial_ports covers this name):
scripts/run.sh prototype
```

**Two wrong diagnoses, recorded so they are not repeated.**
1. "L4T R36 does not ship `ch341`" - wrong. `ch341.ko` is present and loaded on
   kernel 5.15.185-tegra.
2. "`brltty` is claiming the device" - wrong on this machine. `brltty` was already
   removed. It *is* a real cause of CH340 flapping on Ubuntu 22.04 in general
   (its udev rule matches `1a86:7523`; `dmesg` then shows
   `ch341-uart converter now attached to ttyUSB0` immediately followed by
   `ch341-uart disconnected from ttyUSB0`), so it stays documented - but it was not
   this case.

**The lesson that generalises:** `lsusb -t` names the driver that is actually
bound, and that name is the fastest way to tell the kernel driver (`ch341`) from
the vendor one (`usb_ch341`). Checking it first would have skipped both wrong
turns. Do not assume a CH340 always means `/dev/ttyUSB0`.

**Why this was missed three times, and the real trap:** pyserial's own
`comports()` globs a **hardcoded list of names** -
`/dev/ttyS*`, `ttyUSB*`, `ttyXRUSB*`, `ttyACM*`, `ttyAMA*`, `rfcomm*`, `ttyAP*`
(see `serial/tools/list_ports_linux.py`) - and `/dev/ttyCH341*` is **not** in it.
So the device was invisible to pyserial before any filtering ran:
`list_serial_ports()` returned `[]` while `ls /dev/ttyCH341*` showed the port.
Filtering the results could never have fixed it.

`list_serial_ports()` therefore **globs `/dev` itself** and unions that with
pyserial's output (pyserial still contributes vid/pid where it can see the
device). `flash_firmware.sh` calls the same helper. Both now find a CH340 under
the WCH driver.

**Symptom to recognise:** `--list-ports` finds a port but the runner says none
was found. That mismatch means the two are enumerating differently - which is
exactly what a hardcoded name list in a third-party library causes.

## Startup troubleshooting on the Jetson

**`ModuleNotFoundError: No module named 'serial'`** (hit 10 Nov 2026). `pyserial`
drives the Arduino link and is imported lazily, so nothing fails at install time -
the robot dies at startup instead. It now fails with an explicit message, but the
fix is:
```bash
.venv/bin/pip install pyserial          # or: sudo apt install python3-serial
```
It was missing because it was never listed in `requirements-jetson.txt`; the laptop
only had it as a transitive dependency of `platformio`, which is dev-only. Both
requirements files declare it now. **If you add a new runtime dependency, add it to
`requirements-jetson.txt` too** - the Jetson venv installs only that file.

The check runs *before* the camera opens on purpose: the first version crashed
after the Argus/GStreamer log, so a missing Python package looked like a camera
fault.

**`no Arduino was found over USB`.** The Nano is powered from USB, so if it is
connected it should appear. On the Jetson:
```bash
scripts/flash_firmware.sh prototype --list-ports   # /dev/ttyACM* or /dev/ttyUSB*
lsusb | grep -i '1a86\|2341'                       # 1a86 = CH340 clone, 2341 = genuine
dmesg | tail -5                                    # does the kernel see it at all?
```
- **Nothing in `lsusb`**: the cable is charge-only, or the port is dead. Swap the
  cable first - this is the most common cause and costs nothing to test.
- **In `lsusb` but no `/dev/ttyUSB*`**: the CH340 driver is not loaded. Check
  `lsmod | grep ch34x`; if it is missing, `sudo modprobe ch341`.
- **Device present but `permission denied`**: your user is not in `dialout`, which
  is the default on a fresh JetPack install. `sudo usermod -aG dialout $USER`, then
  **log out and back in** - group changes only apply to a new login.

**Finding the port on the Jetson.** `scripts/flash_firmware.sh prototype --list-ports`
works there even though `pio` does not (the AVR toolchain is x86-only): listing
ports is pure bash and deliberately sits before the `pio` check.

**Wrong serial port.** `configs/prototype.yaml` no longer hardcodes one
(`actuator: { type: serial }`). The runner uses the single board that is plugged in
and refuses to guess when several are. Override with `--port /dev/ttyUSB0` or add
`port:` to the config. A **CH340 clone** is `/dev/ttyUSB0`; a genuine board is
`/dev/ttyACM0`. List them with `scripts/flash_firmware.sh --list-ports`.

**The camera prints its frame size at startup** (`camera csi sensor 0: 640x360`).
That number matters: `lane.px_per_m` is pixels per metre **at that frame size**, so
changing the resolution silently rescales the whole lane model. The CSI pipeline
captures 1280x720 and scales down to 640x360 (`mav.io.camera.csi_pipeline`); Argus
reporting `1280 x 720` in its log is the capture side, not the frame Python sees.

## Verified on the Jetson (from the user's command output)
- Python 3.10.12.
- OpenCV 4.8.0 with GStreamer 1.20.3. The build info had no CUDA line, so assume OpenCV runs on CPU only.
- CUDA toolkit config 12.6, cuDNN 9.3, TensorRT 10.3.0.30 runtime libraries and ONNX parser (`libnvonnxparsers10`).
- Power mode: `MAXN_SUPER` (mode id 2), the highest power mode.
- CSI camera alive: `nvarguscamerasrc` works (user, 7 Nov 2026). Exact module still to identify.
- Not seen in the package list (may be missing, verify): TensorRT development packages, `python3-libnvinfer`, `trtexec`, `nvcc`.
- **OS: L4T R36.5.0 (JetPack 6.2.3)**, confirmed 10 Nov 2026 from `/etc/nv_tegra_release`.
- **`trtexec` and `nvcc` are absent** (confirmed 10 Nov 2026). This is FINE for now: no
  code in `src/` uses TensorRT, CUDA or a model - `models/` holds only `.gitkeep`, and
  the `model: { path: ... }` keys in the configs are read by nobody. Lane, cone and
  checkpoint detection are numpy-only. JetPack's dev tools are only needed if a neural
  network is added for `final` later. **The `nvidia-jetpack` metapackage is a multi-GB
  download and is NOT a prerequisite for anything currently working.**
- The user is in the `dialout` group (confirmed 10 Nov 2026), so serial access rights
  are not the problem.

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

### Recording a clip for `px_per_m` (Q7)

`calibrate_scale` needs a video file, and the file has to come from the ROBOT
camera, because `px_per_m` belongs to the camera mount. The runner can write one:

```bash
scripts/run.sh prototype --dry-run --record data/recordings/lane.mp4
# press `s` + Enter, push the robot slowly along a real 0.50 m lane,
# then Ctrl+C
PYTHONPATH=src python -m mav.tools.calibrate_scale data/recordings/lane.mp4
```

`--dry-run` keeps the motors dead, so the robot can be pushed by hand and the clip
is safe to take. Frames are only written while RUN, which is why `s` is needed -
`app.step` reads the camera only in that state.

`--record` wraps the camera and writes `read()`'s return value, so the clip is
**exactly** what the detector sees. That matters: `px_per_m` is pixels per metre at
a specific frame size, so a clip from a different pipeline (gst-launch, a phone)
could differ in size or colour conversion and mis-scale every offset silently. The
runner prints the frame size at startup (`camera csi sensor 0: 640x360`) and
`RecordingCamera` prints it again on close.

If OpenCV lacks the MPEG-4 encoder on this image, the recorder falls back to
MJPG in a `.avi` beside the requested path and says so - it does not abort a
session the camera is already delivering to.

### Symptom: the motor HUMS but the car does not move

This is the **deadband**, and it is the most likely first symptom on a fresh
build. A DC gearmotor does not turn below a breakaway PWM: the windings buzz at the
PWM frequency while the rotor stays put.

With `MPS_TO_PWM = 300` and `PWM_DEADBAND = 0`, the policy's 0.15-0.3 m/s maps to:

| command | PWM | duty | volts into the motor (3S through an L298N, ~8.5 V) |
|---|---|---|---|
| 0.15 m/s | 45 | 18 % | ~1.5 V |
| 0.30 m/s | 90 | 35 % | ~3.0 V |

1.5 V is far below what a geared motor needs to break away, so it hums and holds.
**Measure the deadband first:**

```bash
PYTHONPATH=src python -m mav.tools.calibrate_speed --port /dev/ttyCH341USB0 \
    --k-current 300 --probe --seconds 1.5
```

It ramps PWM from 40 to 255 in steps and asks "did the wheels turn?" after each,
stopping at the first yes. Then set that value:

```c
const float PWM_DEADBAND = 120.0f;   // measured, not guessed
```

and reflash. **Only then** run the normal sweep for `MPS_TO_PWM`, using PWM values
above the deadband.

`--probe` deliberately does **not** apply the `max_speed_mps` ceiling: while the
deadband is unknown, every m/s value is computed from an unmeasured model, so a
speed ceiling would block the very PWMs needed to find the breakaway point. The
physical `--max-pwm` limit (default 255) is what protects the motor there.

**If the motor is SILENT rather than humming**, PWM is not reaching it at all -
that is wiring, not calibration. Check, in this order:
- the **ENA/ENB jumpers are removed** on both L298Ns (with them in, the channel is
  permanently full-on and PWM does nothing);
- IN1/IN2 (and IN3/IN4) are actually wired to the pins in the table above;
- the battery is connected to the L298N **+12V** terminal, and all grounds share
  one node.

### Symptom: it moves sometimes, mostly it does not

Two independent causes, and the status line tells them apart. Read it once per
second:

```
run  cp=0 lost=0 off=+0.06 conf=0.65 cmd=(0.26,-0.12) avo=- line=W 0.260 0.253 31ms
```

| field | means | what a bad value causes |
|---|---|---|
| `lost=1` | no usable measurement, or `|off| > 0.35 m` | `STOP` after `max_hold_s` (0.5 s) - and a stopped robot keeps the same bad view, so it can stay stopped forever |
| `off=` | lateral error the policy believes | too large because `px_per_m` is wrong (see below) |
| `conf=` | detection confidence, 0.3..1.0 | the policy **multiplies speed by it**, so a weak frame commands a crawl, not a stop |
| `cmd=` | the `(speed, curvature)` sent | 0.00 means the policy chose to stop; small means it is crawling |
| `line=` | the exact protocol line written | `S` = stop reached the board; `W ...` = a drive command did |

**Cause 1: `PWM_DEADBAND` unmeasured -> the PWM never reaches breakaway.** The
policy's realistic output is *lower* than its `v_min`/`v_max` suggest, because
speed is scaled by confidence and by curvature. Confidence on the phone recording
in `data/recordings/` ranges 0.33-0.96 (median 0.65). Feeding those through the
policy:

| offset | conf | speed | PWM (k=300, deadband 0) |
|---|---|---|---|
| 0.05 m | 0.35 | 0.100 | 30 |
| 0.15 m | 0.65 | 0.166 | 50 |
| 0.25 m | 0.90 | 0.202 | 61 |

Every combination lands at PWM 24-77, and a geared motor typically needs 100-150
to break away. So **no** command the policy can produce at this calibration moves
the car. Fix: `--probe` above, then re-measure `MPS_TO_PWM`.

**Cause 2: `px_per_m` is wrong, so the lane counts as LOST.** With
`px_per_m = 130` instead of the real ~650, every offset is inflated 5x. A real
deviation of 7 cm reads as 0.35 m, which is exactly `max_offset_m`, so the robot
declares the lane lost after 7 cm and stops. On the phone recording in
`data/recordings/`, **46 of 76 detected frames** exceeded 0.35 m at this setting.
Fix: record with `--record` and run `calibrate_scale` (Q7 below).

On the robot's own first clip (`data/recordings/lane.mp4`) the picture is worse
than "wrong scale": **both lane edges are never in the frame** at the lookahead
row, so no scale exists there to measure. The detector still reports confident
numbers (-1.85..+1.98 m offsets, curvature to -5.62 1/m), which is why
`calibrate_scale` refuses rather than reporting them. See
`docs/PROGRESS.md`, "the first real robot clip".

**Cause 3 (not a fault): `s` vs Enter.** `s` + Enter starts. **Any other input,
including a bare Enter, is an EMERGENCY stop.** After that the robot stays in
`stopped` until `s` is typed again.

Also note: `policy.conf_min` (0.15) is **dead config** while
`lane.min_confidence` (0.30) is higher - the detector returns `None` first, so the
policy never sees a measurement below 0.30. The runner prints a note at startup
when this happens. Lower `lane.min_confidence` to actually loosen it.

### Procedure
Safety first: e-stop in hand and tested, motors running only on the floor, a clear
straight run of at least 4 m, the same surface for every point, and the battery
charged the same way each time (a sagging pack changes the speed).

1. **Flash a PROVISIONAL value first — this is not optional.** With
   `MPS_TO_PWM = 0` the firmware turns every command into PWM 0, so the car will not
   move even for the calibration itself. Edit
   `firmware/arduino/prototype/src/prototype.ino`:
   ```c
   const float MPS_TO_PWM   = 300.0f;  // PROVISIONAL, replaced in step 5
   const float PWM_DEADBAND = 0.0f;
   ```
   then `scripts/flash_firmware.sh prototype`. The provisional value only has to let
   the car move: the sweep converts each PWM target into the m/s command the firmware
   needs, so the fit does not depend on the provisional value being correct.
   **This is the only step where the car is calibrated against a number you have not
   measured — do it with the wheels off the ground first to confirm direction.**
2. Tape a start line and put a tape measure along the run.
3. Run:
   ```
   PYTHONPATH=src python -m mav.tools.calibrate_speed --port /dev/ttyUSB0 \
       --k-current 300 --pwm 40 60 80 100 130 160 --seconds 3
   ```
   (`--drive ackermann` for the `final` build, which speaks `D` instead of `W`.)
   A Nano clone shows up as `/dev/ttyUSB*` (CH340); a genuine one as `/dev/ttyACM*`.
   Check with `scripts/flash_firmware.sh --list-ports`.
   The tool reads `max_speed_mps` from `configs/vehicle.yaml` and **refuses** any PWM
   that would exceed it: it writes to the port directly, so it never passes through
   `SafeActuator`'s clamp. Raise `--max-speed` only on purpose.
4. For each PWM: press Enter, the car runs for `--seconds`, it stops, you enter how
   far it travelled in metres. The tool streams the command at 20 Hz on purpose — the
   firmware's watchdog stops the motors after 500 ms without a valid line, so a
   one-off command would coast after half a second and be measured as far too slow.
5. The tool fits both constants, prints the residual, and shows the model against
   every measurement. Paste the two lines into the sketch (replacing the provisional
   300), **reflash**, and record them here:

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

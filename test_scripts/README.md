# Hardware Test Scripts

Standalone scripts for verifying individual hardware components **without** building the full ROS 2 package. Run these directly on the target machine (Raspberry Pi, NUC, etc.) to confirm wiring, communication, and basic functionality before integrating with the robot stack.

---

## Scripts

### `ili9341_spi_test.py` — 2.8" ILI9341 SPI Display

Tests the ILI9341 TFT display over SPI: colour fills, text rendering, and a mock status screen layout.

**Dependencies:**

```bash
pip install Pillow adafruit-circuitpython-rgb-display
```

**Usage:**

```bash
# Full test with hardware connected (default pins: CS=8, DC=24, RST=25, LED=18)
python3 ili9341_spi_test.py

# Custom GPIO pins
python3 ili9341_spi_test.py --dc 24 --rst 25 --cs 8 --led 18

# Skip colour fills, just show the mock status screen
python3 ili9341_spi_test.py --mock-only

# Measure full-screen refresh rate
python3 ili9341_spi_test.py --fps

# No hardware — renders in memory only (good for checking the code runs)
python3 ili9341_spi_test.py --no-hw
```

**Pinout (ILI9341 → Raspberry Pi, BCM numbering):**

```
ILI9341 Pin    Pi Pin (BCM)    Pi Physical Pin   Notes
───────────    ────────────    ───────────────   ─────
VCC            3.3V            Pin 1             3.3V only — not 5V
GND            GND             Pin 6             Any ground pin
CS             GPIO 8 (CE0)    Pin 24            SPI0 chip select
RESET          GPIO 25         Pin 22
DC / RS        GPIO 24         Pin 18            Data / Command
SDI / MOSI     GPIO 10         Pin 19            SPI0 MOSI
SCK            GPIO 11         Pin 23            SPI0 SCLK
LED            GPIO 18         Pin 12            Backlight (PWM-capable)
SDO / MISO     GPIO 9          Pin 21            SPI0 MISO (optional)
```

Default GPIO assignments match `params.yaml` and the driver constructor. Override with `--cs`, `--dc`, `--rst`, `--led` flags if your wiring differs.

**What to look for:**
- Red/green/blue/white/black fill screens cycle cleanly
- Text is readable at all three sizes
- Mock status screen layout matches the expected dashboard
- FPS benchmark: ~10-15 FPS is typical over SPI on a Pi

---

### `datafrog_controller_test.py` — Data Frog Bluetooth Gamepad

Tests a Bluetooth gamepad (Data Frog or similar Xbox-style controller) via `evdev`. Verifies axes, buttons, and rumble/vibration.

**Dependencies:**

```bash
pip install evdev
```

**Pairing (first time):**

```bash
bluetoothctl
# > scan on
# > pair XX:XX:XX:XX:XX:XX
# > trust XX:XX:XX:XX:XX:XX
# > connect XX:XX:XX:XX:XX:XX
```

**Usage:**

```bash
# Full test suite (axes → buttons → vibration, 45s per phase)
python3 datafrog_controller_test.py

# List all detected input devices
python3 datafrog_controller_test.py --list

# Live event monitor (prints every input event, Ctrl+C to stop)
python3 datafrog_controller_test.py --monitor

# Test only axes or buttons
python3 datafrog_controller_test.py --axes-only
python3 datafrog_controller_test.py --buttons-only

# Specify device path manually
python3 datafrog_controller_test.py -d /dev/input/event5

# Shorter timeout per phase (e.g. 20 seconds)
python3 datafrog_controller_test.py -t 20
```

**What to look for:**
- Both sticks report full range of motion (>30% coverage = PASS)
- All physical buttons register (A/B/X/Y, bumpers, start, d-pad)
- Phantom buttons (C, Z, etc.) are expected to show SKIP
- Vibration test may show SKIP on controllers without rumble motors

---

### `odriveSerialTest.py` — ODrive Motor Controller (Serial ASCII)

Interactive test console for ODrive motor controllers over serial (ASCII protocol). Supports calibration, velocity/position control, encoder diagnostics, and full parameter dumps.

**Dependencies:**

```bash
pip install pyserial
```

**Usage:**

```bash
# Interactive mode (default port COM18 / 115200 baud)
python3 odriveSerialTest.py

# Specify port and axis
python3 odriveSerialTest.py -p /dev/ttyACM0 -a 0

# Quick info dump and exit
python3 odriveSerialTest.py -p /dev/ttyACM0 --info

# Run calibration non-interactively
python3 odriveSerialTest.py -p /dev/ttyACM0 --calibrate

# Automated velocity or position test
python3 odriveSerialTest.py -p /dev/ttyACM0 --test-velocity
python3 odriveSerialTest.py -p /dev/ttyACM0 --test-position
```

**Interactive menu highlights:**
| Option | What it does |
|--------|-------------|
| 4 | Setup incremental encoder (MT6816 ABZ) |
| 5 | Setup SPI absolute encoder (MT6816 SPI) |
| 6 | Live encoder readout (verify counts change when shaft rotates) |
| 7 | Measure actual CPR by hand-turning one revolution |
| 10 | **Start motor** — full routine: calibrate → encoder offset → closed loop |
| 15 | Manual velocity control (type speeds, see measured feedback) |
| 17 | Full diagnostic dump (copy-paste for troubleshooting) |
| a | Switch between axis 0 and axis 1 |

**What to look for:**
- VBus voltage reads ~24V (or whatever your PSU provides)
- Motor calibration completes without errors (phase resistance/inductance printed)
- Encoder shows changing position when shaft is rotated
- Velocity test: measured velocity tracks commanded velocity within ~10%

---

### `test_realsense_bag.sh` — Intel RealSense D415 + Rosbag Recording

Launches the RealSense D415 camera node via ROS 2 and records colour + aligned depth streams to an MCAP rosbag.

**Dependencies:**

```bash
# Add the ROS 2 apt repo first (see main README), then:
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-realsense2-camera \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-mcap \
  librealsense2-utils
```

**Usage:**

```bash
chmod +x test_realsense_bag.sh

# Record for 30 seconds (default)
./test_realsense_bag.sh

# Record for 60 seconds
./test_realsense_bag.sh 60

# Record until Ctrl+C
./test_realsense_bag.sh 0
```

**What it does:**
1. Checks ROS 2 Humble is sourced and all packages are installed
2. Detects a connected RealSense device via `rs-enumerate-devices`
3. Launches `realsense2_camera` at 424x240 @ 6fps (colour + aligned depth)
4. Waits for topics to appear, samples the frame rate
5. Records 4 topics to `~/rosbags/realsense_test_<timestamp>/` in MCAP format
6. Prints bag info summary when done

**Recorded topics:**
- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/color/camera_info`
- `/camera/aligned_depth_to_color/camera_info`

**What to look for:**
- Camera detected on USB 3.x (won't work reliably on USB 2)
- Topic Hz reports ~6 FPS (matching the configured profile)
- Bag info shows non-zero message counts on all 4 topics
- Replay: `ros2 bag play ~/rosbags/realsense_test_*`

---

## General Tips

- All scripts are **standalone** — no `colcon build` needed
- Run with `--help` (Python scripts) to see all options
- On a Pi, make sure SPI and I2C are enabled in `raspi-config` for the display test
- For serial devices, check permissions: `sudo usermod -aG dialout $USER` (log out/in after)
- For input devices: `sudo usermod -aG input $USER`

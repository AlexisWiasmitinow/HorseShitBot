# Hardware Test Scripts

Standalone scripts for verifying individual hardware components **without** building the full ROS 2 package. Run these directly on the target machine (Raspberry Pi, NUC, etc.) to confirm wiring, communication, and basic functionality before integrating with the robot stack.

---

## Scripts

### `ili9341_spi_test.py` — 2.8" ILI9341 SPI Display

Tests the ILI9341 TFT display over SPI: colour fills, text rendering, and a mock status screen layout.

**Dependencies:**

```bash
pip install Pillow adafruit-blinka adafruit-circuitpython-rgb-display
```

**Usage:**

```bash
# Full test with hardware (defaults follow the SBC: Jetson physical pins vs Pi BCM)
python3 ili9341_spi_test.py

# Custom pins (same numbering rules as your platform — see pin tables below)
python3 ili9341_spi_test.py --cs 24 --dc 22 --rst 18 --led 12

# If D/C and RESET might be swapped on the harness, override both (Jetson defaults: 22 / 18)
python3 ili9341_spi_test.py --raw-test --spi-bus 0 --dc 18 --rst 22

# After init: green fill, then toggles INVOFF/INVON (0x20/0x21) — if the image never inverts, D/C may be wrong
python3 ili9341_spi_test.py --raw-test --spi-bus 0 --raw-invert-cmd-test

# Raw SPI (drives --led HIGH by default; use --raw-no-backlight to skip)
python3 ili9341_spi_test.py --raw-test

# Same, but many 2.8" boards use ST7789; slow SPI helps long jumpers
python3 ili9341_spi_test.py --raw-test --chip st7789 --raw-baud 500000

# Jumper MOSI to MISO on the header (TFT disconnected from those pins); verifies /dev/spidev
python3 ili9341_spi_test.py --spi-loopback --spi-bus 1 --spi-device 0

# Very slow SPI (50 kHz) or explicit Hz if wiring is long / noisy
python3 ili9341_spi_test.py --raw-test --spi-bus 0 --raw-slow
python3 ili9341_spi_test.py --raw-test --spi-bus 0 --raw-baud 10000

# Skip colour fills, just show the mock status screen
python3 ili9341_spi_test.py --mock-only

# Measure full-screen refresh rate
python3 ili9341_spi_test.py --fps

# No hardware — renders in memory only (good for checking the code runs)
python3 ili9341_spi_test.py --no-hw
```

### `luma_lcd_test.py` — 2.8" ILI9341 / ST7789 SPI Display (luma.lcd)

Same tests as `ili9341_spi_test.py` (colour fills, text, mock status screen, FPS benchmark, SPI loopback) but uses the **luma.lcd** library instead of the Adafruit stack. luma.lcd talks to `spidev` directly (no Blinka layer) and uses `RPi.GPIO` / `Jetson.GPIO` for D/C, RST, and backlight.

**Dependencies:**

```bash
pip install luma.lcd Pillow
# Jetson: Jetson.GPIO is usually pre-installed
# Pi:     RPi.GPIO is usually pre-installed
# For --spi-loopback:  sudo apt install python3-spidev
```

**Usage:**

```bash
# Full test with hardware (defaults follow the SBC: Jetson physical pins vs Pi BCM)
python3 luma_lcd_test.py

# Many 2.8" breakouts are actually ST7789, not ILI9341
python3 luma_lcd_test.py --chip st7789

# Custom pins (same numbering rules as your platform — see pin tables below)
python3 luma_lcd_test.py --dc 22 --rst 18 --led 12

# Explicit SPI port/device (try --spi-port 0 vs 1 on Jetson)
python3 luma_lcd_test.py --spi-port 1 --spi-device 0

# Skip colour fills, just show the mock status screen
python3 luma_lcd_test.py --mock-only

# Measure full-screen refresh rate
python3 luma_lcd_test.py --fps

# No hardware — luma dummy device, renders in memory only
python3 luma_lcd_test.py --no-hw

# SPI loopback (pure spidev, same as ili9341_spi_test.py --spi-loopback)
python3 luma_lcd_test.py --spi-loopback --spi-bus 1 --spi-device 0
```

**luma.lcd vs Adafruit stack:**
| | luma.lcd | Adafruit |
|---|---|---|
| SPI transport | `spidev` (kernel) directly | Blinka → `busio` → `spidev` |
| GPIO for D/C, RST | `RPi.GPIO` / `Jetson.GPIO` | Blinka `digitalio` |
| Init sequence | Built into `luma.lcd.device` | Built into `adafruit_rgb_display` |
| Drawing API | `canvas(device)` context manager (auto-flush) | Manual `Image.new()` + `disp.image()` |
| Backlight | `device.backlight(True/False)` | Manual GPIO |
| Install | `pip install luma.lcd` | `pip install adafruit-blinka adafruit-circuitpython-rgb-display` |

**What to look for:** Same as `ili9341_spi_test.py` — clean colour fills, readable text at three sizes, correct mock dashboard layout, ~10-15 FPS over SPI.

---

### `jetson_gpio_wiggle.py` — prove a header GPIO toggles

If SPI loopback works and even **10 kHz** still misbehaves, confirm **D/C** and **RST** really reach the pins (multimeter should see ~0 V / ~3.3 V alternating).

```bash
python3 test_scripts/jetson_gpio_wiggle.py 22
python3 test_scripts/jetson_gpio_wiggle.py 18
```

**Jetson Nano (40-pin header)** — enable **SPI1** in `sudo /opt/nvidia/jetson-io/jetson-io.py` (pins 19/21/23/24/26). For the defaults below, set **`spi3` (13,16,18,22,37)** and **`i2s2` (12,35,38,40)** groups to **GPIO** so pins 12, 18, and 22 are usable. Reboot after saving.

```
ILI9341 Pin    Jetson physical pin   Notes
───────────    ───────────────────   ─────
VCC            3.3V (e.g. pin 1)     3.3V only unless your module is 5V-tolerant
GND            GND                   Any ground
CS             24                    SPI1 CS0 (hardware CS)
DC / RS        22                    GPIO (configure group as GPIO in jetson-io)
RESET          18                    GPIO
SDI / MOSI     19                    SPI1 MOSI
SCK            23                    SPI1 SCLK
LED            12                    Backlight GPIO (or tie to 3.3V)
```

**Jetson pinmux (why Pi “just works” but Nano can look dead)**  
- **SPI1 MISO is physical pin 21**, not 22. D/C on **22** is not “the same net” as SPI1 MISO.  
- Pin **22** is in the **`spi3` (13,16,18,22,37)** group in `jetson-io`. That pin must be muxed as **GPIO**, not as an SPI3 function, or D/C will not behave (backlight on, black glass, software still prints `ok`).  
- If D/C is unreliable after checking mux, move **D/C** to another header pin that is **only** GPIO in your config (and update `--dc` / `params.yaml`).

**Raspberry Pi (BCM GPIO + SPI0)**

```
ILI9341 Pin    BCM    Physical   Notes
───────────    ───    ────────   ─────
VCC            —      Pin 1      3.3V only — not 5V
GND            —      Pin 6      Any ground
CS             8      Pin 24     SPI0 CE0
RESET          25     Pin 22
DC / RS        24     Pin 18
SDI / MOSI     10     Pin 19
SCK            11     Pin 23
LED            18     Pin 12     Backlight (PWM-capable)
```

Defaults match `params.yaml` / `ILI9341Display` for each platform. Override with `--cs`, `--dc`, `--rst`, `--led` if your wiring differs.

**If `--raw-test` shows no colours**

- **`--raw-test` turns the backlight on by default** (GPIO `--led` HIGH). Use **`--raw-no-backlight`** only if you tie LED to 3.3V or use a different backlight circuit.
- Run `ls /dev/spidev*` — you should see at least one device after SPI is enabled.
- On **Jetson**, which header SPI maps to **`/dev/spidev0.*` vs `1.*`** depends on the device tree — use **`--spi-loopback`** (MOSI jumpered to MISO) to see which bus matches your pins. Then use that **`--spi-bus`** with **`--raw-test`**.  
  `sudo apt install python3-spidev`  
  `python3 ili9341_spi_test.py --spi-loopback --spi-bus 0` and `--spi-bus 1`  
  With `--spi-bus`, the script uses **kernel chip select** on that `spidev` device (normal `mode=0`). Tegra usually rejects `SPI_NO_CS`, so GPIO bit-bang CS is not the default; use **`--spi-device 1`** if your CS line is CS1 (e.g. pin 26). Only use **`--spi-gpio-cs`** on hosts where the `SPI_NO_CS` ioctl succeeds.
- Lower **`--raw-baud`** (for example `500000`) if you use long jumpers, or **`--raw-slow`** (50 kHz), or **`--raw-baud 10000`**.  
- **If even the lowest baud still fails**, speed is not the bottleneck. Check in order: **(1)** TFT **`VCC` on 3.3 V (pin 1)**, not 5 V. **(2)** **`CS`** = **GND only** *or* **pin 24 only**, never both, and no accidental short. **(3)** **D/C** really on **`--dc`** (default 22) — run `python3 test_scripts/jetson_gpio_wiggle.py 22` and probe the pin; it must toggle. **(4)** **`--chip st7789`**. **(5)** Try the same panel on a Pi to rule out a dead flex.
- **Solid white** with backlight and SPI OK often means **GRAM or MADCTL/BGR** — try  
  `./test_scripts/ili9341_spi_test.py --raw-test --spi-bus 0 --raw-madctl 0xE8` then `0x28`, `0x88`, `0xA8`.  
- If the image is still blank, try **`--raw-dc-invert`**, confirm **`--dc` / `--rst`** and **MOSI/SCK** match **`--spi-loopback`** pins. Optionally **`--raw-init-minimal`** or **`--chip st7789`**.  
  `./test_scripts/ili9341_spi_test.py --raw-test --spi-bus 0 --raw-dc-invert`  
  `./test_scripts/ili9341_spi_test.py --raw-test --spi-bus N --raw-diagnose`  
  and check the readback lines. **All `ff`** usually means floating MISO or the wrong **`/dev/spidev`**; **all `00`** often means SDO not wired back to the Jetson, MISO held low, or reads not routed on that breakout. **Non-trivial bytes** mean the panel is answering on MISO. Then try **`--spi-device` (0 vs 1)** and **`--spi-mode 3`** if needed.

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
- Raspberry Pi: enable SPI (and I2C if needed) in `raspi-config` for the display test
- Jetson Nano: use `jetson-io` to enable SPI1 and free GPIO lines for DC/RST/LED as above
- For serial devices, check permissions: `sudo usermod -aG dialout $USER` (log out/in after)
- For input devices: `sudo usermod -aG input $USER`

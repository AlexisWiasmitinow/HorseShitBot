# HorseShitBot

## Overview

ROS 2 robot controller with switchable wheel backends (MKS steppers / ODrive BLDC),
generic actuator nodes (lift, brush, bin door) with stall-based referencing,
Bluetooth gamepad teleop, a web dashboard, and an on-robot ILI9341 SPI status screen.

## Training — Perception & Autonomous Navigation

The one-day training plan (1 day, 9am–5pm) is available in **[PLAN.md](PLAN.md)**.
Technical details: see `docs/training_perception_navigation.md` and `docs/slam_guide.md`.

### Nodes

| Node | Purpose |
|------|---------|
| `mks_bus_node` | Owns the Modbus RTU serial port, exposes motor command services |
| `wheel_driver_node` | Subscribes `/cmd_vel`, ramping loop, switchable MKS / ODrive backend |
| `lift_node` | Dual-motor lift actuator (motors 3+5, counter-rotating) |
| `brush_node` | Single-motor brush actuator (motor 4) |
| `bin_door_node` | Single-motor bin door actuator (motor 6) |
| `gamepad_teleop_node` | Data Frog BT controller via evdev, publishes `/cmd_vel` + actuator commands |
| `web_dashboard_node` | FastAPI web UI for settings, control, and live diagnostics |
| `status_screen_node` | Renders live status on the on-robot ILI9341 2.8" TFT (SPI) |
| `bag_recorder_node` | Programmatic rosbag2 recording of camera topics for ML training |
| `battery_modbus_node` | Battery voltage via the shared Modbus bus, low-voltage warning and shutdown |
| `realsense2_camera` | Official Intel RealSense wrapper (colour + depth, D415) |

### Hardware

- **Wheels (MKS):** MKS SERVO57D steppers on Modbus RTU (`/dev/ttyUSB0`, 38400 baud)
- **Wheels (ODrive):** ODrive dual BLDC via serial ASCII (`/dev/ttyACM0`, 115200 baud)
- **Lift/Brush/Bin Door:** MKS SERVO57D steppers on the same Modbus bus
- **Gamepad:** Data Frog Bluetooth controller (evdev)
- **Status Screen:** 2.8" ILI9341 SPI TFT, 240x320 (JC2432S028 V1.2) — Raspberry Pi uses SPI0 + BCM GPIO defaults; Jetson Nano uses SPI1 on pins 19/21/23/24 with DC/RST/LED on physical pins 22/18/12 (see `test_scripts/README.md`)
- **Camera:** Intel RealSense D415 (USB 3.0, colour + depth)

## Prerequisites

- Raspberry Pi, Jetson Nano, or similar with ROS 2 Humble/Iron/Jazzy installed
- Python 3.10+
- System packages: `python3-colcon-common-extensions`

## Build

```bash
cd ~/HorseShitBot
./scripts/build_workspace.sh
```

The build helper uses `colcon build --symlink-install` and records a fingerprint
of `src/`. It rebuilds all workspace packages when build-relevant source
content changes, including launch files, web static files, and interface
definitions. Use `./scripts/build_workspace.sh --force` for an unconditional
rebuild.

## Configure

Edit parameters in `src/horseshitbot/config/params.yaml` before launching,
or update them at runtime via the web dashboard or `ros2 param set`.

Key parameters:

- `wheel_driver_node.wheel_backend`: `"mks"` or `"odrive"` (switchable at runtime)
- `mks_bus_node.port`: Modbus serial port (default `/dev/ttyUSB0`)
- `wheel_driver_node.odrive_port`: ODrive serial port (default `/dev/ttyACM0`)
- Per-actuator open/close speeds, accelerations, and stall referencing thresholds

## Run

The supported startup command builds when needed, sources ROS 2 Humble and the
workspace install, and launches `robot_launch.py`:

```bash
cd ~/HorseShitBot
./scripts/start.sh
```

Source changes under `src/` therefore affect the next normal start without
rebuilding unnecessarily. To force a rebuild and then start, use:

```bash
./scripts/buildstart.sh
# Equivalent:
./scripts/start.sh --rebuild
```

Startup options:

- `--no-camera` skips the RealSense node. The two bag recorder nodes remain
  available because recording profiles are independently selected in the
  dashboard and the mapping recorder does not require a camera.
- `--no-mks` skips `mks_bus_node` and the MKS lift/brush/bin-door nodes, and
  explicitly selects the existing ODrive wheel backend.
- `--no-lidar` skips the lidar node and its static transform.
- `--drive-only` starts the wheel dependency, wheel driver, and gamepad only.

The current supported web UI is `web_dashboard_node`, available at
`http://<ROBOT_IP>:8080`.

### Start at boot with systemd

`systemd/robot-web.service` is a per-user service that calls the same supported
`scripts/start.sh` path. It expects the checkout at `~/HorseShitBot`; `%h`
resolves to the service user's home directory.

```bash
mkdir -p ~/.config/systemd/user
cp ~/HorseShitBot/systemd/robot-web.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now robot-web.service

# Allow the user service to start at boot before interactive login:
sudo loginctl enable-linger "$USER"
```

Inspect it with `systemctl --user status robot-web.service` and
`journalctl --user -u robot-web.service`. The service starts the complete ROS 2
stack, including `web_dashboard_node` on port 8080; it does not start the
legacy port-8000 application.

## Battery Monitoring and Low-Voltage Shutdown

`battery_modbus_node` reads channel 1 of the N43IC04 acquisition module
(Modbus ID 33, 19200 baud) through `mks_bus_node`'s
`/modbus/read_holding_register` service. It never opens a serial port itself,
so it cannot contend with the motor bus. It is configured in
`src/horseshitbot/config/battery_modbus.yaml` and is started with
`enable_battery:=true`:

```bash
ros2 launch horseshitbot battery_modbus_launch.py   # battery node alone
ros2 launch horseshitbot robot_launch.py enable_battery:=true
```

### Calibration

Measured on the assembled robot (2026-09-25):

| Battery | Raw count |
|---------|-----------|
| 20 V | 970 |
| 22 V | 1068 |
| 24 V | 1167 |
| 26 V | 1265 |
| 28 V | 1364 |

The response is linear, so the conversion is

```
battery_voltage = (raw + 15.2) / 49.25
```

which reproduces every measured point to better than 0.005 V. The constants are
the `raw_offset_counts` and `counts_per_volt` parameters.

### Thresholds

| Voltage | Behaviour |
|---------|-----------|
| above 22.0 V | normal |
| at or below 22.0 V | low-battery warning (`/battery/low`, dashboard warning, `WARN` diagnostic) |
| at or below 20.0 V | critical (`/battery/critical`, `ERROR` diagnostic) |
| at or below 20.0 V for 5 s | safe motor stop, then Jetson shutdown |

The critical timer is wall-clock based (`critical_hold_sec`), so brief sag under
motor load does not trigger a shutdown, and a slow poll loop cannot shorten the
confirmation window. A recovery above 20.0 V or a failed ADC read restarts the
timer. The shutdown request is latched and runs at most once per boot.

The shutdown sequence calls the existing safe-stop services in
`safe_stop_services` — `/wheel_driver_node/stop_fast` (which fires the
hardware-level MKS `REG_EMERGENCY_STOP`) plus the lift, brush, and bin-door
stops — waits `safe_stop_grace_sec` for the stop frames to reach the motors,
and then runs `shutdown_command`, which defaults to
`sudo -n /usr/sbin/shutdown -h now`.

### Deployment prerequisite: passwordless shutdown

The stack does not run as root, so powering off must be the one command it may
run without a password. Grant exactly that and nothing more:

```bash
sudo tee /etc/sudoers.d/horseshitbot-shutdown >/dev/null <<'EOF'
hsb ALL=(root) NOPASSWD: /usr/sbin/shutdown
EOF
sudo chmod 0440 /etc/sudoers.d/horseshitbot-shutdown
sudo visudo -cf /etc/sudoers.d/horseshitbot-shutdown
```

Check it without powering the robot off:

```bash
sudo -n /usr/sbin/shutdown --help
```

`shutdown_command` uses the absolute `/usr/sbin/shutdown` so the sudoers rule
matches a fixed path rather than whatever `PATH` happens to resolve to. Without
this file the safe motor stop still runs, but the shutdown command fails and
the node logs an error telling you to power the robot down manually.

### Observing the battery

```bash
# Voltage only
ros2 topic echo /battery/voltage

# Voltage, state, thresholds and the critical hold timer
ros2 topic echo /battery/status_json

# ROS diagnostics (OK / WARN / ERROR with raw count and thresholds)
ros2 topic echo /diagnostics

# One-shot human-readable summary
ros2 service call /battery_modbus_node/get_battery_voltage std_srvs/srv/Trigger
```

`/battery/status_json` also carries the state, both thresholds, the critical
hold timer and whether the shutdown has been requested, so a consumer such as
the web dashboard never has to hard-code the thresholds.

## Test Scripts

Standalone scripts in `test_scripts/` for testing hardware outside of ROS:

```bash
# Test the ODrive motors
python3 test_scripts/odriveSerialTest.py -p /dev/ttyACM0

# Test the Bluetooth gamepad
python3 test_scripts/datafrog_controller_test.py

# Test the ILI9341 SPI display
python3 test_scripts/ili9341_spi_test.py
python3 test_scripts/ili9341_spi_test.py --mock-only   # status screen mockup
python3 test_scripts/ili9341_spi_test.py --fps          # benchmark refresh rate
python3 test_scripts/ili9341_spi_test.py --no-hw        # run without hardware
```

## RealSense D415 Camera & Data Recording

### Install the RealSense ROS 2 wrapper

```bash
sudo apt install ros-${ROS_DISTRO}-realsense2-camera
```

The launch file automatically starts the camera node with 640x480@30fps colour + depth
(aligned depth enabled, point cloud disabled).

### Recording rosbag data

Recording captures colour frames, aligned depth frames, and both camera-info topics into
timestamped `.mcap` bag files under `~/rosbags/`.

**Start / stop recording:**

| Method | How |
|--------|-----|
| Gamepad | Press **Select** to toggle recording on/off |
| Web dashboard | Click **Start Recording** / **Stop Recording** in the Camera card |
| CLI | `ros2 service call /bag_recorder_node/start_recording std_srvs/srv/Trigger` |

### Replay a bag

```bash
ros2 bag play ~/rosbags/horseshitbot_2026-04-03_14-30-00
```

### Extract frames from a bag

```bash
# List topics
ros2 bag info ~/rosbags/horseshitbot_2026-04-03_14-30-00

# Play back and subscribe with your own script, or use ros2 bag export tools
ros2 bag play ~/rosbags/<bag_name> --topics /camera/color/image_raw
```

## Gamepad Button Mapping

| Input | Action |
|-------|--------|
| Left Stick | Drive (X = steer, Y = forward/back) |
| D-Pad Up/Down | Lift open/close |
| Right Bumper (hold) | Brush open |
| Left Bumper (hold) | Bin door open |
| A | Emergency stop |
| B | Normal stop |
| X | Stop all actuators |
| Y | Switch wheel backend (MKS / ODrive) |
| Start | Reference all actuators |
| Select | Toggle rosbag recording |

## Legacy FastAPI App

The original FastAPI web controller on port 8000 remains in `robot_web/` for
reference only. It is not part of `scripts/start.sh`, `robot_launch.py`, or the
systemd service. If it is deliberately needed for legacy testing, start it
manually:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn robot_web.main:app --host 0.0.0.0 --port 8000
```

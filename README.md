# HorseShitBot

## Overview

ROS 2 robot controller with switchable wheel backends (MKS steppers / ODrive BLDC),
generic actuator nodes (lift, brush, bin door) with stall-based referencing,
Bluetooth gamepad teleop, a web dashboard, and an on-robot ILI9341 SPI status screen.

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

### Hardware

- **Wheels (MKS):** MKS SERVO57D steppers on Modbus RTU (`/dev/ttyUSB0`, 38400 baud)
- **Wheels (ODrive):** ODrive dual BLDC via serial ASCII (`/dev/ttyACM0`, 115200 baud)
- **Lift/Brush/Bin Door:** MKS SERVO57D steppers on the same Modbus bus
- **Gamepad:** Data Frog Bluetooth controller (evdev)
- **Status Screen:** 2.8" ILI9341 SPI TFT, 240x320 (JC2432S028 V1.2)

## Prerequisites

- Raspberry Pi (or similar) with ROS 2 Humble/Iron/Jazzy installed
- Python 3.10+
- System packages: `python3-colcon-common-extensions`

## Build

```bash
cd ~/HorseShitBot
colcon build --packages-select horseshitbot
source install/setup.bash
```

## Configure

Edit parameters in `src/horseshitbot/config/params.yaml` before launching,
or update them at runtime via the web dashboard or `ros2 param set`.

Key parameters:

- `wheel_driver_node.wheel_backend`: `"mks"` or `"odrive"` (switchable at runtime)
- `mks_bus_node.port`: Modbus serial port (default `/dev/ttyUSB0`)
- `wheel_driver_node.odrive_port`: ODrive serial port (default `/dev/ttyACM0`)
- Per-actuator open/close speeds, accelerations, and stall referencing thresholds

## Run

Launch the full system:

```bash
ros2 launch horseshitbot robot_launch.py
```

The web dashboard will be available at `http://<PI_IP>:8080`.

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

## Legacy FastAPI App

The original FastAPI web controller is still in `robot_web/` for reference:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn robot_web.main:app --host 0.0.0.0 --port 8000
```

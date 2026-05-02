#!/usr/bin/env python3
"""
Standalone gamepad → ODrive drive test.
No ROS required. Reads the Data Frog controller via evdev and sends
velocity commands to both ODrive axes over serial ASCII.

Left stick Y  = forward / backward (both wheels)
Right stick X = turn left / right (differential)

Usage:
    python3 gamepad_drive_test.py -p /dev/ttyACM0
    python3 gamepad_drive_test.py -p /dev/ttyACM1 --max-speed 5
    python3 gamepad_drive_test.py -p /dev/ttyACM0 -d /dev/input/event3
"""

import argparse
import sys
import time
import signal
import threading

try:
    import serial
except ImportError:
    print("Missing: pip install pyserial")
    sys.exit(1)

try:
    import evdev
    from evdev import InputDevice, ecodes, list_devices
except ImportError:
    print("Missing: pip install evdev")
    sys.exit(1)


CONTROLLER_NAMES = [
    "data frog", "datafrog", "wireless controller",
    "gamepad", "game controller", "bluetooth gamepad",
    "xbox", "gamesir",
]


def find_controller(device_path=None):
    if device_path:
        return InputDevice(device_path)

    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        name_lower = dev.name.lower()
        if any(kw in name_lower for kw in CONTROLLER_NAMES):
            return dev

    for dev in devices:
        caps = dev.capabilities(verbose=False)
        if ecodes.EV_ABS in caps and ecodes.EV_KEY in caps:
            abs_codes = [c[0] if isinstance(c, tuple) else c for c in caps[ecodes.EV_ABS]]
            if ecodes.ABS_X in abs_codes and ecodes.ABS_Y in abs_codes:
                return dev
    return None


class ODriveSimple:
    """Minimal ODrive serial driver for velocity control."""

    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(
            port=port, baudrate=baudrate, timeout=1.0,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        )
        time.sleep(0.5)
        self.ser.reset_input_buffer()

    def cmd(self, command):
        self.ser.reset_input_buffer()
        self.ser.write((command.strip() + "\n").encode("ascii"))
        time.sleep(0.1)
        resp = ""
        while self.ser.in_waiting > 0:
            resp += self.ser.readline().decode("ascii", errors="ignore").strip() + "\n"
            time.sleep(0.02)
        return resp.strip()

    def read(self, prop):
        return self.cmd(f"r {prop}")

    def write(self, prop, val):
        self.cmd(f"w {prop} {val}")

    def set_velocity(self, axis, vel):
        self.write(f"axis{axis}.controller.input_vel", f"{vel:.4f}")

    def set_idle(self, axis):
        self.write(f"axis{axis}.requested_state", "1")

    def enter_closed_loop(self, axis):
        self.write(f"axis{axis}.error", "0")
        self.write(f"axis{axis}.motor.error", "0")
        self.write(f"axis{axis}.encoder.error", "0")
        time.sleep(0.1)
        self.write(f"axis{axis}.requested_state", "8")
        time.sleep(0.5)
        state = self.read(f"axis{axis}.current_state").strip()
        return state == "8"

    def close(self):
        self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="Gamepad → ODrive drive test")
    parser.add_argument("-p", "--port", required=True, help="ODrive serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-d", "--device", help="Gamepad input device path (auto-detect if omitted)")
    parser.add_argument("--max-speed", type=float, default=3.0, help="Max velocity in turns/sec (default 3.0)")
    parser.add_argument("--deadzone", type=float, default=0.08, help="Stick deadzone (default 0.08)")
    parser.add_argument("--invert-left", action="store_true", help="Invert left motor direction")
    parser.add_argument("--invert-right", action="store_true", help="Invert right motor direction")
    parser.add_argument("--skip-calibration", action="store_true",
                        help="Skip entering closed loop (if already in closed loop from odriveSerialTest)")
    args = parser.parse_args()

    # ── find gamepad ──────────────────────────────────────────────
    print("Looking for gamepad...")
    pad = find_controller(args.device)
    if not pad:
        print("No gamepad found. Pair it first or specify with -d")
        print("Available devices:")
        for path in list_devices():
            dev = InputDevice(path)
            print(f"  {path}: {dev.name}")
        return 1
    print(f"  Gamepad: {pad.name} ({pad.path})")

    # figure out axis ranges
    caps = pad.capabilities(verbose=False)
    abs_info = {}
    if ecodes.EV_ABS in caps:
        for item in caps[ecodes.EV_ABS]:
            code = item[0] if isinstance(item, tuple) else item
            info = item[1] if isinstance(item, tuple) else None
            abs_info[code] = info

    def axis_range(code):
        info = abs_info.get(code)
        if info:
            return info.min, info.max
        return -32768, 32767

    ly_min, ly_max = axis_range(ecodes.ABS_Y)
    rx_code = ecodes.ABS_Z if ecodes.ABS_Z in abs_info else ecodes.ABS_RX
    rx_min, rx_max = axis_range(rx_code)

    # ── connect ODrive ────────────────────────────────────────────
    print(f"Connecting to ODrive on {args.port}...")
    try:
        odrv = ODriveSimple(args.port, args.baud)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return 1

    vbus = odrv.read("vbus_voltage")
    print(f"  VBus: {vbus}V")

    if not args.skip_calibration:
        for ax in (0, 1):
            print(f"  Entering closed loop on axis {ax}...")
            if not odrv.enter_closed_loop(ax):
                print(f"  Failed on axis {ax}. Run odriveSerialTest.py first to calibrate.")
                print(f"  Or use --skip-calibration if already in closed loop.")
                odrv.close()
                return 1
        print("  Both axes in closed loop.")
    else:
        print("  Skipping calibration (assuming already in closed loop).")

    # ── control loop ──────────────────────────────────────────────
    running = True

    def handle_sig(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sig)

    raw_linear = 0.0
    raw_angular = 0.0
    lock = threading.Lock()

    def normalize(val, vmin, vmax):
        center = (vmin + vmax) / 2.0
        half = (vmax - vmin) / 2.0
        if half == 0:
            return 0.0
        n = (val - center) / half
        if abs(n) < args.deadzone:
            return 0.0
        return max(-1.0, min(1.0, n))

    def gamepad_reader():
        nonlocal raw_linear, raw_angular
        for event in pad.read_loop():
            if not running:
                break
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_Y:
                    with lock:
                        raw_linear = -normalize(event.value, ly_min, ly_max)
                elif event.code == rx_code:
                    with lock:
                        raw_angular = normalize(event.value, rx_min, rx_max)

    reader = threading.Thread(target=gamepad_reader, daemon=True)
    reader.start()

    print()
    print(f"=== DRIVING ===")
    print(f"  Left stick Y  = forward/back")
    print(f"  Right stick X = turn")
    print(f"  Max speed: {args.max_speed} turns/sec")
    print(f"  Press Ctrl+C to stop")
    print()

    try:
        while running:
            with lock:
                lin = raw_linear
                ang = raw_angular

            left_vel = (lin - ang) * args.max_speed
            right_vel = (lin + ang) * args.max_speed

            left_vel = max(-args.max_speed, min(args.max_speed, left_vel))
            right_vel = max(-args.max_speed, min(args.max_speed, right_vel))

            if args.invert_left:
                left_vel = -left_vel
            if args.invert_right:
                right_vel = -right_vel

            odrv.set_velocity(0, left_vel)
            odrv.set_velocity(1, right_vel)

            print(f"\r  L: {left_vel:+6.2f}  R: {right_vel:+6.2f}  "
                  f"(stick: fwd={lin:+.2f} turn={ang:+.2f})   ",
                  end="", flush=True)

            time.sleep(0.05)

    except Exception as e:
        print(f"\nError: {e}")

    print("\n\nStopping motors...")
    odrv.set_velocity(0, 0)
    odrv.set_velocity(1, 0)
    time.sleep(0.3)
    odrv.set_idle(0)
    odrv.set_idle(1)
    odrv.close()
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

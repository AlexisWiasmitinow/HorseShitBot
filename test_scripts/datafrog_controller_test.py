#!/usr/bin/env python3
"""
Test program for Data Frog Game Controller via Bluetooth
Run on Raspberry Pi after pairing the controller.
"""

import sys
import time
import struct
import signal

try:
    import evdev
    from evdev import InputDevice, categorize, ecodes, list_devices
except ImportError:
    print("ERROR: evdev not installed. Run: pip3 install evdev")
    sys.exit(1)


CONTROLLER_NAMES = [
    "data frog",
    "datafrog",
    "wireless controller",
    "gamepad",
    "game controller",
    "bluetooth gamepad",
    "xbox",
    "gamesir",
]

ABS_AXIS_NAMES = {
    ecodes.ABS_X: "Left Stick X",
    ecodes.ABS_Y: "Left Stick Y",
    ecodes.ABS_Z: "Right Stick X",
    ecodes.ABS_RX: "Right Stick X (alt)",
    ecodes.ABS_RY: "Right Stick Y (alt)",
    ecodes.ABS_RZ: "Right Stick Y",
    ecodes.ABS_GAS: "Right Trigger",
    ecodes.ABS_BRAKE: "Left Trigger",
    ecodes.ABS_HAT0X: "D-Pad X",
    ecodes.ABS_HAT0Y: "D-Pad Y",
}

BUTTON_NAMES = {
    ecodes.BTN_A: "A (South)",
    ecodes.BTN_B: "B (East)",
    ecodes.BTN_C: "C (extra)",
    ecodes.BTN_X: "X (West)",
    ecodes.BTN_Y: "Y (North)",
    ecodes.BTN_Z: "Z (extra)",
    ecodes.BTN_TL: "Left Bumper (L1)",
    ecodes.BTN_TR: "Right Bumper (R1)",
    ecodes.BTN_TL2: "Left Trigger Click (L2)",
    ecodes.BTN_TR2: "Right Trigger Click (R2)",
    ecodes.BTN_SELECT: "Select",
    ecodes.BTN_START: "Start / Menu",
    ecodes.BTN_MODE: "Mode",
    ecodes.BTN_THUMBL: "Left Stick Press (L3)",
    ecodes.BTN_THUMBR: "Right Stick Press (R3)",
    ecodes.KEY_BACK: "Back / Select",
    ecodes.KEY_HOMEPAGE: "Home",
}

# Buttons that are phantom/unmapped on most Data Frog controllers
PHANTOM_BUTTONS = {
    ecodes.BTN_C,
    ecodes.BTN_Z,
    ecodes.BTN_TL2,
    ecodes.BTN_TR2,
    ecodes.BTN_SELECT,
    ecodes.BTN_MODE,
    ecodes.BTN_THUMBL,
    ecodes.BTN_THUMBR,
}


class ControllerTester:
    def __init__(self, device_path=None):
        self.device = None
        self.running = True
        self.axis_results = {}
        self.button_results = {}
        signal.signal(signal.SIGINT, self._signal_handler)

        if device_path:
            self.device = InputDevice(device_path)
        else:
            self.device = self._find_controller()

        if not self.device:
            print("ERROR: No controller found.")
            print("Make sure your Data Frog controller is paired and connected.")
            print("\nAvailable input devices:")
            for path in list_devices():
                dev = InputDevice(path)
                print(f"  {path}: {dev.name}")
            sys.exit(1)

        print(f"Using controller: {self.device.name}")
        print(f"Device path:      {self.device.path}")
        print(f"Phys:             {self.device.phys}")

    def _signal_handler(self, sig, frame):
        self.running = False

    def _find_controller(self):
        """Auto-detect the Data Frog controller from available input devices."""
        devices = [InputDevice(path) for path in list_devices()]
        for dev in devices:
            name_lower = dev.name.lower()
            if any(keyword in name_lower for keyword in CONTROLLER_NAMES):
                return dev

        # Fallback: pick the first device that has gamepad-like capabilities
        for dev in devices:
            caps = dev.capabilities(verbose=False)
            has_abs = ecodes.EV_ABS in caps
            has_key = ecodes.EV_KEY in caps
            if has_abs and has_key:
                abs_codes = [c[0] if isinstance(c, tuple) else c for c in caps[ecodes.EV_ABS]]
                if ecodes.ABS_X in abs_codes and ecodes.ABS_Y in abs_codes:
                    return dev
        return None

    def print_device_capabilities(self):
        """Print all capabilities the controller reports."""
        print("\n" + "=" * 60)
        print("CONTROLLER CAPABILITIES")
        print("=" * 60)

        caps = self.device.capabilities(verbose=True)

        if ("EV_ABS", ecodes.EV_ABS) in caps:
            print("\n--- Absolute Axes ---")
            for abs_info in caps[("EV_ABS", ecodes.EV_ABS)]:
                code_tuple, info = abs_info
                code_name = code_tuple[0] if isinstance(code_tuple, tuple) else code_tuple
                code_val = code_tuple[1] if isinstance(code_tuple, tuple) else code_tuple
                print(f"  {code_name}: min={info.min}, max={info.max}, "
                      f"fuzz={info.fuzz}, flat={info.flat}, resolution={info.resolution}")

        if ("EV_KEY", ecodes.EV_KEY) in caps:
            print("\n--- Buttons ---")
            for btn in caps[("EV_KEY", ecodes.EV_KEY)]:
                name = btn[0] if isinstance(btn, tuple) else btn
                print(f"  {name}")

        print()

    def test_axes(self, duration=45):
        """Test all analog axes - move every stick and trigger."""
        print("\n" + "=" * 60)
        print(f"AXIS TEST (move all sticks and triggers within {duration}s)")
        print("=" * 60)
        print("  Move left stick, right stick, triggers, and d-pad.")
        print("  Press Ctrl+C to skip.\n")

        caps = self.device.capabilities(verbose=False)
        expected_axes = {}
        if ecodes.EV_ABS in caps:
            for item in caps[ecodes.EV_ABS]:
                code = item[0] if isinstance(item, tuple) else item
                info = item[1] if isinstance(item, tuple) else None
                name = ABS_AXIS_NAMES.get(code, f"ABS_{code}")
                expected_axes[code] = {
                    "name": name,
                    "info": info,
                    "min_seen": None,
                    "max_seen": None,
                    "center": info.value if info else 0,
                    "moved": False,
                }

        start = time.time()
        try:
            while self.running and (time.time() - start) < duration:
                event = self.device.read_one()
                if event is None:
                    time.sleep(0.005)
                    continue
                if event.type == ecodes.EV_ABS:
                    code = event.code
                    if code in expected_axes:
                        ax = expected_axes[code]
                        val = event.value
                        if ax["min_seen"] is None or val < ax["min_seen"]:
                            ax["min_seen"] = val
                        if ax["max_seen"] is None or val > ax["max_seen"]:
                            ax["max_seen"] = val
                        ax["moved"] = True
                        remaining = duration - (time.time() - start)
                        print(f"\r  {ax['name']:20s} = {val:6d}  "
                              f"(range seen: {ax['min_seen']}..{ax['max_seen']})  "
                              f"[{remaining:.0f}s left]   ", end="", flush=True)
        except KeyboardInterrupt:
            self.running = True

        print("\n\n--- Axis Test Results ---")
        all_pass = True
        for code, ax in expected_axes.items():
            info = ax["info"]
            hw_range = info.max - info.min if info else 0
            if ax["moved"] and ax["min_seen"] is not None:
                seen_range = ax["max_seen"] - ax["min_seen"]
                coverage = (seen_range / hw_range * 100) if hw_range > 0 else 0
                status = "PASS" if coverage > 30 else "PARTIAL"
                if status != "PASS":
                    all_pass = False
                print(f"  [{status:7s}] {ax['name']:20s}  "
                      f"hw=[{info.min}..{info.max}]  "
                      f"seen=[{ax['min_seen']}..{ax['max_seen']}]  "
                      f"coverage={coverage:.0f}%")
            else:
                all_pass = False
                hw = f"hw=[{info.min}..{info.max}]" if info else ""
                print(f"  [NOT HIT] {ax['name']:20s}  {hw}")

        self.axis_results = expected_axes
        return all_pass

    def test_buttons(self, duration=45):
        """Test all buttons - press each one."""
        print("\n" + "=" * 60)
        print(f"BUTTON TEST (press every button within {duration}s)")
        print("=" * 60)
        print("  Press each button at least once.")
        print("  Phantom buttons (C, Z) are skipped if not physically present.")
        print("  Press Ctrl+C to skip.\n")

        caps = self.device.capabilities(verbose=False)
        expected_buttons = {}
        if ecodes.EV_KEY in caps:
            for code in caps[ecodes.EV_KEY]:
                name = BUTTON_NAMES.get(code, f"BTN_{code}")
                is_phantom = code in PHANTOM_BUTTONS
                expected_buttons[code] = {
                    "name": name,
                    "pressed": False,
                    "phantom": is_phantom,
                }

        real_buttons = {c: b for c, b in expected_buttons.items() if not b["phantom"]}
        start = time.time()
        try:
            while self.running and (time.time() - start) < duration:
                event = self.device.read_one()
                if event is None:
                    time.sleep(0.005)
                    continue
                if event.type == ecodes.EV_KEY and event.value == 1:
                    code = event.code
                    if code in expected_buttons:
                        expected_buttons[code]["pressed"] = True
                        remaining = duration - (time.time() - start)
                        hit = sum(1 for b in real_buttons.values() if b["pressed"])
                        total = len(real_buttons)
                        print(f"\r  Button: {expected_buttons[code]['name']:20s}  "
                              f"[{hit}/{total} tested]  "
                              f"[{remaining:.0f}s left]   ", end="", flush=True)
                        if hit == total:
                            break
        except KeyboardInterrupt:
            self.running = True

        print("\n\n--- Button Test Results ---")
        all_pass = True
        for code, btn in expected_buttons.items():
            if btn["phantom"]:
                if btn["pressed"]:
                    print(f"  [PASS   ] {btn['name']}  (phantom — but it fired!)")
                else:
                    print(f"  [SKIP   ] {btn['name']}  (phantom — no physical button)")
                continue
            status = "PASS" if btn["pressed"] else "NOT HIT"
            if not btn["pressed"]:
                all_pass = False
            print(f"  [{status:7s}] {btn['name']}")

        self.button_results = expected_buttons
        return all_pass

    def _find_hidraw_device(self):
        """Try to find the hidraw device that corresponds to our controller."""
        import os
        import glob as globmod
        phys = self.device.phys or ""
        name_lower = self.device.name.lower()

        for hidraw_path in sorted(globmod.glob("/dev/hidraw*")):
            try:
                sysfs = f"/sys/class/hidraw/{os.path.basename(hidraw_path)}/device/uevent"
                if os.path.exists(sysfs):
                    with open(sysfs) as f:
                        uevent = f.read().lower()
                    if any(kw in uevent for kw in ["data frog", "gamepad", "game controller"]):
                        return hidraw_path
                    if phys and phys.split("/")[0] in uevent:
                        return hidraw_path
            except (OSError, PermissionError):
                continue

        for hidraw_path in sorted(globmod.glob("/dev/hidraw*")):
            try:
                with open(hidraw_path, "rb") as f:
                    pass
                return hidraw_path
            except (OSError, PermissionError):
                continue
        return None

    def _try_hidraw_rumble(self):
        """Send rumble via hidraw using common gamepad report formats."""
        hidraw = self._find_hidraw_device()
        if not hidraw:
            return False

        # Common rumble report formats used by Xbox-style and generic controllers
        rumble_reports = [
            bytes([0x05, 0xFF, 0x04, 0x00, 0xFF, 0xFF, 0x00, 0x00]),  # Xbox-style
            bytes([0x01, 0x00, 0xFF, 0xFF, 0x00]),                      # Generic HID
            bytes([0x05, 0x01, 0x00, 0x00, 0xFF, 0xFF]),                # Alternate
        ]

        for i, report in enumerate(rumble_reports):
            try:
                with open(hidraw, "wb") as f:
                    f.write(report)
                print(f"  Sent rumble via hidraw ({hidraw}, format {i+1})...")
                time.sleep(0.7)

                stop_reports = [
                    bytes([0x05, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00]),
                    bytes([0x01, 0x00, 0x00, 0x00, 0x00]),
                    bytes([0x05, 0x01, 0x00, 0x00, 0x00, 0x00]),
                ]
                try:
                    with open(hidraw, "wb") as f:
                        f.write(stop_reports[i])
                except OSError:
                    pass
                return True
            except (OSError, PermissionError):
                continue
        return False

    def test_vibration(self):
        """Attempt to test rumble/vibration if supported."""
        print("\n" + "=" * 60)
        print("VIBRATION TEST")
        print("=" * 60)

        # Method 1: standard Linux force-feedback via evdev
        try:
            effect = evdev.ff.Effect(
                ecodes.FF_RUMBLE, -1, 0,
                evdev.ff.Trigger(0, 0),
                evdev.ff.Replay(500, 0),
                evdev.ff.EffectType(ff_rumble_effect=evdev.ff.Rumble(
                    strong_magnitude=0xFFFF, weak_magnitude=0xFFFF
                ))
            )
            effect_id = self.device.upload_effect(effect)
            self.device.write(ecodes.EV_FF, effect_id, 1)
            print("  Vibration ON via evdev FF for 0.5s ...")
            time.sleep(0.6)
            self.device.erase_effect(effect_id)
            print("  [PASS] Force feedback works (evdev).")
            return True
        except (OSError, AttributeError) as e:
            print(f"  evdev FF not available: {e}")

        # Method 2: direct hidraw rumble command
        print("  Trying hidraw rumble fallback...")
        if self._try_hidraw_rumble():
            print("  [INFO] Sent hidraw rumble — did the controller vibrate?")
            return True

        print("  [SKIP] No rumble method worked for this controller.")
        return None

    def live_monitor(self):
        """Live-print all events from the controller until Ctrl+C."""
        print("\n" + "=" * 60)
        print("LIVE EVENT MONITOR (press Ctrl+C to stop)")
        print("=" * 60 + "\n")

        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                if event.type == ecodes.EV_SYN:
                    continue
                if event.type == ecodes.EV_ABS:
                    name = ABS_AXIS_NAMES.get(event.code, f"ABS_{event.code}")
                    print(f"  ABS  {name:20s} = {event.value}")
                elif event.type == ecodes.EV_KEY:
                    name = BUTTON_NAMES.get(event.code, f"BTN_{event.code}")
                    state = "PRESSED" if event.value == 1 else ("RELEASED" if event.value == 0 else "REPEAT")
                    print(f"  KEY  {name:20s}   {state}")
                else:
                    print(f"  type={event.type} code={event.code} value={event.value}")
        except KeyboardInterrupt:
            self.running = True
        print("\nMonitor stopped.")

    def run_all_tests(self, duration=45):
        """Run the full test suite."""
        print("\n" + "=" * 60)
        print("  DATA FROG CONTROLLER TEST SUITE")
        print("=" * 60)

        self.print_device_capabilities()

        axes_ok = self.test_axes(duration=duration)
        buttons_ok = self.test_buttons(duration=duration)
        vib_ok = self.test_vibration()

        print("\n" + "=" * 60)
        print("  SUMMARY")
        print("=" * 60)
        print(f"  Axes:      {'PASS' if axes_ok else 'FAIL / INCOMPLETE'}")
        print(f"  Buttons:   {'PASS' if buttons_ok else 'FAIL / INCOMPLETE'}")
        print(f"  Vibration: {'PASS' if vib_ok else ('SKIP' if vib_ok is None else 'FAIL')}")
        print("=" * 60 + "\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Data Frog Bluetooth Controller Tester")
    parser.add_argument("-d", "--device", help="Input device path, e.g. /dev/input/event0")
    parser.add_argument("-l", "--list", action="store_true", help="List all input devices and exit")
    parser.add_argument("-m", "--monitor", action="store_true", help="Live event monitor mode")
    parser.add_argument("--axes-only", action="store_true", help="Run only the axis test")
    parser.add_argument("--buttons-only", action="store_true", help="Run only the button test")
    parser.add_argument("-t", "--timeout", type=int, default=15, help="Seconds per test phase (default: 15)")
    args = parser.parse_args()

    if args.list:
        print("Available input devices:")
        for path in list_devices():
            dev = InputDevice(path)
            print(f"  {path}: {dev.name} [{dev.phys}]")
        return

    tester = ControllerTester(device_path=args.device)

    if args.monitor:
        tester.live_monitor()
    elif args.axes_only:
        tester.test_axes(duration=args.timeout)
    elif args.buttons_only:
        tester.test_buttons(duration=args.timeout)
    else:
        tester.run_all_tests(duration=args.timeout)


if __name__ == "__main__":
    main()

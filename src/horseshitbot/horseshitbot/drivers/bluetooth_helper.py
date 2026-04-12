"""
Bluetooth helper — query paired/connected devices, battery level, and
reconnect via bluetoothctl.

All subprocess calls use stdin=DEVNULL to prevent bluetoothctl from
entering interactive mode.
"""

from __future__ import annotations

import logging
import re
import subprocess
import time
from pathlib import Path

logger = logging.getLogger(__name__)

_DEVNULL = subprocess.DEVNULL


def _run(cmd: list[str], timeout: int = 5) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout, stdin=_DEVNULL,
        )
    except subprocess.TimeoutExpired:
        logger.warning("Command timed out: %s", " ".join(cmd))
        return subprocess.CompletedProcess(cmd, 1, "", "timeout")
    except Exception as e:
        logger.warning("Command failed: %s — %s", " ".join(cmd), e)
        return subprocess.CompletedProcess(cmd, 1, "", str(e))


def get_battery_level(mac: str = "") -> int | None:
    """Read battery percentage for a Bluetooth device.

    Tries multiple sources in order:
      1. /sys/class/power_supply (kernel HID path)
      2. bluetoothctl info
      3. upower --dump (Xbox controllers often only report here)
    Returns 0-100 or None.
    """
    mac_upper = mac.upper().replace(":", "") if mac else ""

    # 1) /sys/class/power_supply
    ps_base = Path("/sys/class/power_supply")
    if ps_base.is_dir():
        for ps in ps_base.iterdir():
            try:
                cap_file = ps / "capacity"
                if not cap_file.exists():
                    continue
                scope_file = ps / "scope"
                if scope_file.exists() and scope_file.read_text().strip() != "Device":
                    continue
                if mac_upper:
                    ident = ""
                    uevent = ps / "uevent"
                    if uevent.exists():
                        ident = uevent.read_text().upper().replace(":", "")
                    ident += str(ps.resolve()).upper().replace(":", "")
                    if mac_upper not in ident:
                        continue
                return int(cap_file.read_text().strip())
            except (ValueError, OSError):
                continue

    # 2) bluetoothctl info
    if mac:
        r = _run(["bluetoothctl", "info", mac])
        if r.returncode == 0:
            m = re.search(r"Battery Percentage:.*\((\d+)\)", r.stdout)
            if m:
                return int(m.group(1))

    # 3) upower --dump (catches Xbox controllers via input-device paths)
    if mac:
        r = _run(["upower", "--dump"], timeout=3)
        if r.returncode == 0:
            mac_variants = [mac.upper(), mac.upper().replace(":", "_")]
            blocks = r.stdout.split("\n\n")
            for block in blocks:
                block_upper = block.upper()
                if any(v in block_upper for v in mac_variants):
                    m = re.search(r"percentage:\s*(\d+)", block, re.IGNORECASE)
                    if m:
                        return int(m.group(1))

    return None


def get_paired_devices() -> list[dict]:
    """Return list of paired Bluetooth devices [{mac, name, connected}].

    Uses a single `bluetoothctl info` call per device to get both
    connection status and battery.
    """
    r = _run(["bluetoothctl", "devices", "Paired"])
    if r.returncode != 0:
        r = _run(["bluetoothctl", "paired-devices"])
        if r.returncode != 0:
            return []

    devices = []
    for line in r.stdout.strip().splitlines():
        m = re.match(r"Device\s+([\dA-Fa-f:]{17})\s+(.*)", line)
        if not m:
            continue
        mac = m.group(1)
        name = m.group(2).strip()

        connected = False
        battery = None
        info = _run(["bluetoothctl", "info", mac])
        if info.returncode == 0:
            connected = bool(re.search(r"Connected:\s*yes", info.stdout, re.IGNORECASE))
            bm = re.search(r"Battery Percentage:.*\((\d+)\)", info.stdout)
            if bm:
                battery = int(bm.group(1))

        devices.append({
            "mac": mac, "name": name,
            "connected": connected, "battery": battery,
        })
    return devices


def get_last_connected_gamepad() -> dict | None:
    """Find the most recently paired gamepad-like device."""
    devices = get_paired_devices()
    gamepad_keywords = [
        "controller", "gamepad", "game pad", "xbox", "data frog",
        "datafrog", "gamesir", "wireless", "joystick", "pro controller",
    ]
    for dev in devices:
        name_lower = dev["name"].lower()
        if any(kw in name_lower for kw in gamepad_keywords):
            return dev
    return devices[0] if devices else None


def connect_device(mac: str) -> dict:
    """Connect to a Bluetooth device by MAC address.

    If the connection fails with ``br-connection-create-socket``, the
    bluetooth service is restarted automatically and one retry is made.
    """
    r = _run(["bluetoothctl", "connect", mac], timeout=15)
    combined = r.stdout + r.stderr
    ok = r.returncode == 0 or "successful" in combined.lower()

    if not ok and "br-connection-create-socket" in combined:
        logger.warning("BT socket error — restarting bluetooth service and retrying")
        _run(["sudo", "systemctl", "restart", "bluetooth"], timeout=15)
        time.sleep(2)
        r = _run(["bluetoothctl", "connect", mac], timeout=15)
        combined = r.stdout + r.stderr
        ok = r.returncode == 0 or "successful" in combined.lower()

    msg = r.stdout.strip() or r.stderr.strip()
    return {"success": ok, "message": msg}


def disconnect_device(mac: str) -> dict:
    r = _run(["bluetoothctl", "disconnect", mac])
    ok = r.returncode == 0
    msg = r.stdout.strip() or r.stderr.strip()
    return {"success": ok, "message": msg}

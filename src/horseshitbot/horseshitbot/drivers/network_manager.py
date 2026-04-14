"""
Network Manager — helpers for querying and configuring network interfaces
via NetworkManager (nmcli).  Also manages dnsmasq for DHCP server on
static-IP interfaces.

Designed to run on Jetson / Ubuntu where NetworkManager is the default
network stack.
"""

from __future__ import annotations

import json
import logging
import re
import subprocess
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DNSMASQ_CONF_DIR = Path("/etc/dnsmasq.d")


def _run(cmd: list[str], timeout: int = 15) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


# ── Data classes ──────────────────────────────────────────────────

_AP_CONN_PREFIX = "hsb-ap-"


@dataclass
class InterfaceInfo:
    name: str
    type: str                          # "ethernet" | "wifi"
    connected: bool = False
    ip4: str = ""
    gateway: str = ""
    dns: str = ""
    method: str = ""                   # "auto" (DHCP) | "manual" (static) | "shared" (AP)
    connection_name: str = ""
    mac: str = ""
    wifi_ssid: str = ""
    wifi_signal: int = 0
    dhcp_server_active: bool = False
    dhcp_range_start: str = ""
    dhcp_range_end: str = ""
    ap_mode: bool = False
    ap_ssid: str = ""
    ap_password: str = ""
    ap_band: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class WifiNetwork:
    ssid: str
    signal: int = 0
    security: str = ""
    in_use: bool = False
    bssid: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


# ── Query functions ───────────────────────────────────────────────

def get_interfaces() -> list[InterfaceInfo]:
    """Return info for every non-loopback interface managed by NM."""
    result = _run(["nmcli", "-t", "-f",
                    "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
    if result.returncode != 0:
        logger.warning("nmcli device status failed: %s", result.stderr.strip())
        return []

    ifaces: list[InterfaceInfo] = []
    for line in result.stdout.strip().splitlines():
        parts = line.split(":")
        if len(parts) < 4:
            continue
        dev, dtype, state, conn = parts[0], parts[1], parts[2], parts[3]
        if dtype not in ("ethernet", "wifi"):
            continue

        itype = "ethernet" if dtype == "ethernet" else "wifi"
        connected = state == "connected"

        info = InterfaceInfo(
            name=dev,
            type=itype,
            connected=connected,
            connection_name=conn if conn != "--" else "",
        )

        if connected and conn and conn != "--":
            _fill_connection_details(info, conn)

        info.dhcp_server_active = _is_dnsmasq_active(dev)
        if info.dhcp_server_active:
            s, e = _read_dnsmasq_range(dev)
            info.dhcp_range_start = s
            info.dhcp_range_end = e

        ifaces.append(info)
    return ifaces


def _fill_connection_details(info: InterfaceInfo, conn_name: str):
    """Populate IP, gateway, DNS, method, MAC, WiFi from nmcli."""
    # device show gives live IP/gateway/DNS for the interface
    dev_result = _run(["nmcli", "-t", "device", "show", info.name])
    if dev_result.returncode == 0:
        for line in dev_result.stdout.strip().splitlines():
            if ":" not in line:
                continue
            key, _, val = line.partition(":")
            key = key.strip()
            val = val.strip()
            if key.startswith("IP4.ADDRESS") and not info.ip4:
                info.ip4 = val.split("/")[0] if val else ""
            elif key.startswith("IP4.GATEWAY") and not info.gateway:
                info.gateway = val if val and val != "--" else ""
            elif key.startswith("IP4.DNS") and not info.dns:
                info.dns = val
            elif key == "GENERAL.HWADDR":
                info.mac = val

    # connection show gives method, wifi mode, SSID, password
    conn_result = _run(["nmcli", "-t", "-f",
                         "ipv4.method,802-11-wireless.mode,"
                         "802-11-wireless.ssid,802-11-wireless.band,"
                         "802-11-wireless-security.psk",
                         "connection", "show", conn_name])
    if conn_result.returncode == 0:
        for line in conn_result.stdout.strip().splitlines():
            if ":" not in line:
                continue
            key, _, val = line.partition(":")
            key = key.strip()
            val = val.strip()
            if key == "ipv4.method":
                info.method = val
            elif key == "802-11-wireless.mode":
                if val == "ap":
                    info.ap_mode = True
            elif key == "802-11-wireless.ssid":
                info.wifi_ssid = val
                if info.ap_mode:
                    info.ap_ssid = val
            elif key == "802-11-wireless.band":
                info.ap_band = val
            elif key == "802-11-wireless-security.psk":
                info.ap_password = val

    # WiFi signal from device-level info (only in client mode)
    if info.type == "wifi" and not info.ap_mode:
        wifi_result = _run(["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL",
                             "device", "wifi", "list", "ifname", info.name])
        if wifi_result.returncode == 0:
            for line in wifi_result.stdout.strip().splitlines():
                parts = line.split(":")
                if len(parts) >= 3 and parts[0] == "yes":
                    info.wifi_ssid = info.wifi_ssid or parts[1]
                    try:
                        info.wifi_signal = int(parts[2])
                    except ValueError:
                        pass
                    break


def get_ip_summary() -> dict[str, str]:
    """Quick IP lookup for display purposes — {iface_name: ip_or_empty}."""
    result = _run(["hostname", "-I"])
    all_ips = result.stdout.strip().split() if result.returncode == 0 else []

    summary: dict[str, str] = {}
    ifaces = get_interfaces()
    for iface in ifaces:
        summary[iface.name] = iface.ip4
    return summary


def get_ip_for_display() -> list[dict]:
    """Lightweight version for the TFT status screen."""
    ifaces = get_interfaces()
    out = []
    for iface in ifaces:
        out.append({
            "name": iface.name,
            "type": iface.type,
            "connected": iface.connected,
            "ip": iface.ip4,
            "ssid": iface.wifi_ssid,
        })
    return out


# ── WiFi scan / connect ──────────────────────────────────────────

def wifi_scan(iface: str = "") -> list[WifiNetwork]:
    """Scan for WiFi networks. Optionally specify the wifi interface."""
    cmd = ["nmcli", "-t", "-f", "BSSID,SSID,SIGNAL,SECURITY,IN-USE", "device", "wifi", "list"]
    if iface:
        cmd += ["ifname", iface]
    cmd += ["--rescan", "yes"]

    result = _run(cmd, timeout=30)
    if result.returncode != 0:
        logger.warning("wifi scan failed: %s", result.stderr.strip())
        return []

    seen: dict[str, WifiNetwork] = {}
    for line in result.stdout.strip().splitlines():
        # nmcli -t uses \: for escaped colons in BSSIDs
        parts = line.replace("\\:", "_").split(":")
        if len(parts) < 5:
            continue
        bssid = parts[0].replace("_", ":")
        ssid = parts[1]
        if not ssid:
            continue
        try:
            signal = int(parts[2])
        except ValueError:
            signal = 0
        security = parts[3]
        in_use = parts[4].strip() == "*"

        if ssid not in seen or signal > seen[ssid].signal:
            seen[ssid] = WifiNetwork(
                ssid=ssid, signal=signal, security=security,
                in_use=in_use, bssid=bssid,
            )
    nets = sorted(seen.values(), key=lambda n: (-n.in_use, -n.signal))
    return nets


def wifi_connect(ssid: str, password: str = "", iface: str = "") -> dict:
    """Connect to a WiFi network."""
    cmd = ["nmcli", "device", "wifi", "connect", ssid]
    if password:
        cmd += ["password", password]
    if iface:
        cmd += ["ifname", iface]

    result = _run(cmd, timeout=30)
    ok = result.returncode == 0
    msg = result.stdout.strip() or result.stderr.strip()
    return {"success": ok, "message": msg}


def wifi_disconnect(iface: str) -> dict:
    result = _run(["nmcli", "device", "disconnect", iface])
    ok = result.returncode == 0
    msg = result.stdout.strip() or result.stderr.strip()
    return {"success": ok, "message": msg}


# ── Interface configuration ──────────────────────────────────────

def set_dhcp(iface: str) -> dict:
    """Switch an interface to DHCP (auto)."""
    conn = _get_connection_for_device(iface)
    if not conn:
        conn = _ensure_connection(iface)
        if not conn:
            return {"success": False, "message": f"No NM connection for {iface}"}

    cmds = [
        ["nmcli", "connection", "modify", conn,
         "ipv4.method", "auto", "ipv4.addresses", "", "ipv4.gateway", ""],
        ["nmcli", "connection", "up", conn],
    ]
    for cmd in cmds:
        r = _run(cmd)
        if r.returncode != 0:
            return {"success": False, "message": r.stderr.strip() or r.stdout.strip()}
    return {"success": True, "message": f"{iface} set to DHCP"}


def set_static(iface: str, ip: str, prefix: int = 24,
               gateway: str = "", dns: str = "") -> dict:
    """Switch an interface to a static IP."""
    conn = _get_connection_for_device(iface)
    if not conn:
        conn = _ensure_connection(iface)
        if not conn:
            return {"success": False, "message": f"No NM connection for {iface}"}

    addr = f"{ip}/{prefix}"
    cmd = ["nmcli", "connection", "modify", conn,
           "ipv4.method", "manual", "ipv4.addresses", addr]
    if gateway:
        cmd += ["ipv4.gateway", gateway]
    if dns:
        cmd += ["ipv4.dns", dns]

    r = _run(cmd)
    if r.returncode != 0:
        return {"success": False, "message": r.stderr.strip() or r.stdout.strip()}

    r = _run(["nmcli", "connection", "up", conn])
    if r.returncode != 0:
        return {"success": False, "message": r.stderr.strip() or r.stdout.strip()}
    return {"success": True, "message": f"{iface} set to {addr}"}


def _get_connection_for_device(device: str) -> str:
    """Return the NM connection name currently active on *device*."""
    r = _run(["nmcli", "-t", "-f", "NAME,DEVICE", "connection", "show", "--active"])
    if r.returncode != 0:
        return ""
    for line in r.stdout.strip().splitlines():
        parts = line.split(":")
        if len(parts) >= 2 and parts[1] == device:
            return parts[0]
    # Try inactive connections too
    r = _run(["nmcli", "-t", "-f", "NAME,DEVICE", "connection", "show"])
    if r.returncode != 0:
        return ""
    for line in r.stdout.strip().splitlines():
        parts = line.split(":")
        if len(parts) >= 2 and parts[1] == device:
            return parts[0]
    return ""


def _ensure_connection(iface: str) -> str:
    """Create a NM connection for a device that has none."""
    conn_name = f"hsb-{iface}"
    r = _run(["nmcli", "connection", "add", "type", "ethernet",
              "con-name", conn_name, "ifname", iface])
    if r.returncode != 0:
        logger.error("Failed to create connection: %s", r.stderr.strip())
        return ""
    return conn_name


# ── DHCP server (dnsmasq) ────────────────────────────────────────

def _dnsmasq_conf_path(iface: str) -> Path:
    return _DNSMASQ_CONF_DIR / f"hsb-{iface}.conf"


def _is_dnsmasq_active(iface: str) -> bool:
    if not _dnsmasq_conf_path(iface).exists():
        return False
    r = _run(["systemctl", "is-active", "--quiet", "dnsmasq"])
    return r.returncode == 0


def _read_dnsmasq_range(iface: str) -> tuple[str, str]:
    conf = _dnsmasq_conf_path(iface)
    if not conf.exists():
        return ("", "")
    try:
        text = conf.read_text()
        m = re.search(r"dhcp-range=([^,]+),([^,]+)", text)
        if m:
            return (m.group(1), m.group(2))
    except Exception:
        pass
    return ("", "")


def enable_dhcp_server(iface: str, range_start: str, range_end: str,
                       lease_time: str = "12h") -> dict:
    """Write a per-interface dnsmasq config and restart dnsmasq.

    Expects /etc/dnsmasq.d to be writable by the current user
    (set up by install.sh) and passwordless sudo for systemctl restart dnsmasq.
    """
    conf = _dnsmasq_conf_path(iface)
    content = (
        f"interface={iface}\n"
        f"bind-interfaces\n"
        f"dhcp-range={range_start},{range_end},{lease_time}\n"
    )
    try:
        _DNSMASQ_CONF_DIR.mkdir(parents=True, exist_ok=True)
        conf.write_text(content)
    except PermissionError as e:
        return {"success": False,
                "message": f"Cannot write {conf} — run install.sh to fix permissions: {e}"}

    r = _run(["sudo", "systemctl", "restart", "dnsmasq"])
    if r.returncode != 0:
        return {"success": False,
                "message": f"Config written but dnsmasq restart failed: {r.stderr.strip()}"}
    return {"success": True, "message": f"DHCP server enabled on {iface} ({range_start}-{range_end})"}


def disable_dhcp_server(iface: str) -> dict:
    """Remove the dnsmasq config for this interface and restart."""
    conf = _dnsmasq_conf_path(iface)
    if conf.exists():
        try:
            conf.unlink()
        except PermissionError as e:
            return {"success": False,
                    "message": f"Cannot remove {conf} — run install.sh to fix permissions: {e}"}

    r = _run(["sudo", "systemctl", "restart", "dnsmasq"])
    ok = r.returncode == 0
    return {"success": ok,
            "message": f"DHCP server disabled on {iface}" if ok else r.stderr.strip()}


# ── WiFi Access Point (hostapd via NetworkManager) ───────────────

def _ap_conn_name(iface: str) -> str:
    return f"{_AP_CONN_PREFIX}{iface}"


_AP_ADDRESS = "10.0.0.1/24"


def enable_ap(iface: str, ssid: str, password: str = "",
              band: str = "bg", channel: int = 0) -> dict:
    """Create a WiFi access point using NetworkManager.

    NM internally manages hostapd + dnsmasq when 802-11-wireless.mode=ap
    and ipv4.method=shared.  We override the default 10.42.x.x subnet
    with 10.0.0.1/24 so clients get 10.0.0.x addresses.
    """
    if not ssid:
        return {"success": False, "message": "SSID is required"}

    conn = _ap_conn_name(iface)

    # Tear down any existing client connection on this interface
    _run(["nmcli", "device", "disconnect", iface])
    # Remove old AP connection if it exists
    _run(["nmcli", "connection", "delete", conn])

    # Build the connection
    add_cmd = [
        "nmcli", "connection", "add",
        "type", "wifi",
        "ifname", iface,
        "con-name", conn,
        "autoconnect", "no",
        "ssid", ssid,
        "802-11-wireless.mode", "ap",
        "802-11-wireless.band", band,
        "ipv4.method", "shared",
        "ipv4.addresses", _AP_ADDRESS,
    ]
    if channel > 0:
        add_cmd += ["802-11-wireless.channel", str(channel)]

    r = _run(add_cmd)
    if r.returncode != 0:
        return {"success": False, "message": r.stderr.strip() or r.stdout.strip()}

    # Set security if password given (WPA-PSK, minimum 8 chars)
    if password and len(password) >= 8:
        r = _run(["nmcli", "connection", "modify", conn,
                  "wifi-sec.key-mgmt", "wpa-psk",
                  "wifi-sec.psk", password])
        if r.returncode != 0:
            _run(["nmcli", "connection", "delete", conn])
            return {"success": False, "message": r.stderr.strip() or r.stdout.strip()}

    # Activate
    r = _run(["nmcli", "connection", "up", conn], timeout=30)
    if r.returncode != 0:
        msg = r.stderr.strip() or r.stdout.strip()
        _run(["nmcli", "connection", "delete", conn])
        return {"success": False, "message": f"Failed to start AP: {msg}"}

    return {"success": True, "message": f"Access point '{ssid}' active on {iface}"}


def disable_ap(iface: str) -> dict:
    """Stop the access point and return WiFi to client mode."""
    conn = _ap_conn_name(iface)

    r = _run(["nmcli", "connection", "down", conn])
    _run(["nmcli", "connection", "delete", conn])

    # Reconnect to a known WiFi network if available
    _run(["nmcli", "device", "set", iface, "autoconnect", "yes"])
    _run(["nmcli", "device", "connect", iface])

    return {"success": True, "message": f"Access point disabled on {iface}"}


def get_ap_status(iface: str) -> dict:
    """Check if AP mode is active on the interface."""
    conn = _ap_conn_name(iface)
    r = _run(["nmcli", "-t", "-f", "NAME,DEVICE",
              "connection", "show", "--active"])
    if r.returncode != 0:
        return {"active": False}

    for line in r.stdout.strip().splitlines():
        parts = line.split(":")
        if len(parts) >= 2 and parts[0] == conn and parts[1] == iface:
            return {"active": True, "connection": conn}
    return {"active": False}

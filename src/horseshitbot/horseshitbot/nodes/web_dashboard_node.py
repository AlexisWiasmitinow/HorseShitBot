"""
Web Dashboard Node — runs a FastAPI server alongside an rclpy node,
providing a REST + WebSocket API for live status, parameter tuning,
and robot control from a browser.
"""

from __future__ import annotations

import asyncio
from typing import Optional
import json
import os
import shutil
import tarfile
import tempfile
import threading
import time
from datetime import datetime
from io import BytesIO
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import String

from horseshitbot_interfaces.msg import ActuatorState as ActuatorStateMsg
from horseshitbot_interfaces.srv import MksSetSpeed, ActuatorCommand, SwitchBackend
from std_srvs.srv import Trigger

try:
    from sensor_msgs.msg import Image as RosImage
    _HAS_SENSOR_MSGS = True
except ImportError:
    _HAS_SENSOR_MSGS = False

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

try:
    import uvicorn
    from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
    from fastapi.staticfiles import StaticFiles
    from pydantic import BaseModel

    _HAS_FASTAPI = True
except ImportError:
    _HAS_FASTAPI = False

from ..drivers import network_manager as netmgr
from ..drivers import bluetooth_helper as bth


def _read_thermal_zones() -> list[dict]:
    """Read all thermal zones from sysfs (Jetson / any Linux board)."""
    zones = []
    thermal_base = Path("/sys/class/thermal")
    if not thermal_base.is_dir():
        return zones
    for zdir in sorted(thermal_base.iterdir()):
        if not zdir.name.startswith("thermal_zone"):
            continue
        try:
            zone_type = (zdir / "type").read_text().strip()
            temp_mc = int((zdir / "temp").read_text().strip())
            temp_c = temp_mc / 1000.0
        except Exception:
            continue
        zones.append({"zone": zdir.name, "type": zone_type, "temp_c": round(temp_c, 1)})
    return zones



STATIC_DIR = Path(__file__).parent.parent / "web" / "static"

_CONFIG_DIR = Path.home() / ".config" / "horseshitbot"
_CONFIG_FILE = _CONFIG_DIR / "controller_config.json"

def _find_repo_config() -> Path | None:
    """Walk up from __file__ to find the git repo root, then return the
    source config path.  Works whether running from source or install/."""
    p = Path(__file__).resolve()
    for parent in p.parents:
        candidate = parent / "src" / "horseshitbot" / "config" / "controller_config.json"
        if (parent / ".git").is_dir() and (parent / "src" / "horseshitbot").is_dir():
            return candidate
    return None

_REPO_CONFIG_FILE = _find_repo_config()

DEFAULT_BUTTON_MAP = {
    "A": "e_stop",
    "B": "stop_wheels",
    "X": "stop_actuators",
    "Y": "stop_wheels",
    "LB": "bin_door",
    "RB": "brush",
    "LT": "none",
    "RT": "none",
    "L3": "e_stop",
    "R3": "e_stop",
    "D-Pad Up": "lift_up",
    "D-Pad Down": "lift_down",
    "D-Pad Left": "none",
    "D-Pad Right": "none",
    "Options": "camera_bag_recording",
    "Menu": "reference_all",
    "Logo": "toggle_controls",
}

AVAILABLE_ACTIONS = {
    "e_stop": "E-STOP (fast stop)",
    "stop_wheels": "Stop wheels (gentle)",
    "stop_actuators": "Stop all actuators",
    "brush": "Brush deploy (hold)",
    "bin_door": "Bin door open (hold)",
    "lift_up": "Lift up (hold)",
    "lift_down": "Lift down (hold)",
    "reference_all": "Reference all actuators",
    "camera_bag_recording": "Camera bag recording",
    "toggle_controls": "Toggle TFT controls",
    "none": "Unassigned",
}

BUTTON_NAMES = [
    "A", "B", "X", "Y",
    "LB", "RB", "LT", "RT",
    "L3", "R3",
    "D-Pad Up", "D-Pad Down", "D-Pad Left", "D-Pad Right",
    "Options", "Menu", "Logo",
]

AXIS_INFO = {
    "L Stick": "Drive (fwd/back + turn)",
    "R Stick": "Not assigned",
}


class ParamUpdate(BaseModel):
    params: dict


class ControllerConfigUpdate(BaseModel):
    buttons: dict[str, str]


class WifiConnectBody(BaseModel):
    ssid: str
    password: str = ""
    iface: str = ""


class StaticIpBody(BaseModel):
    ip: str
    prefix: int = 24
    gateway: str = ""
    dns: str = ""


class DhcpServerBody(BaseModel):
    enabled: bool
    range_start: str = ""
    range_end: str = ""
    lease_time: str = "12h"


class ApModeBody(BaseModel):
    enabled: bool
    ssid: str = "HorseShitBot"
    password: str = ""
    band: str = "bg"
    channel: int = 0


def _load_config() -> dict:
    if _CONFIG_FILE.exists():
        try:
            with open(_CONFIG_FILE) as f:
                return json.load(f)
        except Exception:
            pass
    return {"buttons": dict(DEFAULT_BUTTON_MAP)}


def _save_config(cfg: dict) -> None:
    _CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(_CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


_DEPTH_MIN_MM = 200   # 0.2 m — closer is clipped
_DEPTH_MAX_MM = 3000  # 3.0 m — farther is clipped


class _RosBridge:
    """Thin glue between rclpy subscriptions and the FastAPI async world."""

    def __init__(self, ros_node: Node):
        self.node = ros_node
        self._lock = threading.Lock()
        self._state: dict = {
            "wheel_status": {},
            "lift": {},
            "brush": {},
            "bin_door": {},
            "bag_recorder": {},
            "gamepad": {},
            "network": [],
            "thermals": [],
        }
        self._ws_clients: list[WebSocket] = []

        self._frame_lock = threading.Lock()
        self._color_frame: np.ndarray | None = None
        self._depth_frame: np.ndarray | None = None

        self._start_network_poller()
        self._start_thermal_poller()

        ros_node.create_subscription(String, "/wheel_status", self._cb_wheel, 10)
        ros_node.create_subscription(ActuatorStateMsg, "/lift/state", lambda m: self._cb_actuator("lift", m), 10)
        ros_node.create_subscription(ActuatorStateMsg, "/brush/state", lambda m: self._cb_actuator("brush", m), 10)
        ros_node.create_subscription(ActuatorStateMsg, "/bin_door/state", lambda m: self._cb_actuator("bin_door", m), 10)
        ros_node.create_subscription(String, "/bag_recorder_node/status", self._cb_bag_recorder, 10)
        ros_node.create_subscription(String, "/gamepad/status", self._cb_gamepad, 10)

        if _HAS_SENSOR_MSGS:
            qos_be = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            )
            qos_rel = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            )
            for qos in (qos_be, qos_rel):
                ros_node.create_subscription(RosImage, "/camera/color/image_raw", self._cb_color, qos)
                ros_node.create_subscription(RosImage, "/camera/aligned_depth_to_color/image_raw", self._cb_depth, qos)

    def _start_network_poller(self):
        def _loop():
            while True:
                try:
                    ifaces = [i.to_dict() for i in netmgr.get_interfaces()]
                except Exception:
                    ifaces = []
                with self._lock:
                    self._state["network"] = ifaces
                time.sleep(10)
        t = threading.Thread(target=_loop, daemon=True)
        t.start()

    def _start_thermal_poller(self):
        def _loop():
            while True:
                try:
                    zones = _read_thermal_zones()
                except Exception:
                    zones = []
                with self._lock:
                    self._state["thermals"] = zones
                time.sleep(5)
        t = threading.Thread(target=_loop, daemon=True)
        t.start()

    def _cb_wheel(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        with self._lock:
            self._state["wheel_status"] = data

    def _cb_actuator(self, name: str, msg: ActuatorStateMsg):
        data = {
            "state": int(msg.state),
            "is_referenced": msg.is_referenced,
            "direction": msg.direction,
            "speed_rpm": msg.speed_rpm,
            "error_message": msg.error_message,
        }
        with self._lock:
            self._state[name] = data

    def _cb_bag_recorder(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        with self._lock:
            self._state["bag_recorder"] = data

    def _cb_gamepad(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        with self._lock:
            self._state["gamepad"] = data

    def _cb_color(self, msg: RosImage):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            with self._frame_lock:
                self._color_frame = frame
        except Exception:
            pass

    def _cb_depth(self, msg: RosImage):
        try:
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            invalid = depth == 0
            clamped = np.clip(depth.astype(np.float32), _DEPTH_MIN_MM, _DEPTH_MAX_MM)
            depth_8u = ((clamped - _DEPTH_MIN_MM) * (255.0 / (_DEPTH_MAX_MM - _DEPTH_MIN_MM))).astype(np.uint8)
            if _HAS_CV2:
                colorized = cv2.applyColorMap(depth_8u, cv2.COLORMAP_TURBO)
                colorized[invalid] = 0
                colorized = cv2.cvtColor(colorized, cv2.COLOR_BGR2RGB)
            else:
                depth_8u[invalid] = 0
                colorized = np.stack([depth_8u, depth_8u, depth_8u], axis=-1)
            with self._frame_lock:
                self._depth_frame = colorized
        except Exception:
            pass

    def get_color_frame(self) -> np.ndarray | None:
        with self._frame_lock:
            return self._color_frame.copy() if self._color_frame is not None else None

    def get_depth_frame(self) -> np.ndarray | None:
        with self._frame_lock:
            return self._depth_frame.copy() if self._depth_frame is not None else None

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._state)





class WebDashboardNode(Node):
    def __init__(self):
        super().__init__("web_dashboard_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("bag_output_dir", "~/rosbags")

        self._host = self.get_parameter("host").get_parameter_value().string_value
        self._port = self.get_parameter("port").get_parameter_value().integer_value
        self._bag_dir = Path(os.path.expanduser(
            self.get_parameter("bag_output_dir").get_parameter_value().string_value
        ))

        if not _HAS_FASTAPI:
            self.get_logger().error("fastapi/uvicorn not installed — dashboard disabled")
            return

        self._bridge = _RosBridge(self)

        self._rec_start_cli = self.create_client(Trigger, "/bag_recorder_node/start_recording")
        self._rec_stop_cli = self.create_client(Trigger, "/bag_recorder_node/stop_recording")
        self._config_pub = self.create_publisher(String, "/gamepad/config", 10)
        self._bag_topics_file = _CONFIG_DIR / "bag_topics.json"

        self._app = self._build_app()

        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()

        self.get_logger().info(f"Web dashboard at http://{self._host}:{self._port}")

    def _build_app(self) -> FastAPI:
        app = FastAPI(title="HorseShitBot Dashboard")

        if STATIC_DIR.exists():
            app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

        bridge = self._bridge
        ros_node = self

        @app.exception_handler(Exception)
        async def _global_exception_handler(request: Request, exc: Exception):
            return JSONResponse(
                {"success": False, "message": f"Internal error: {exc}"},
                status_code=500,
            )

        @app.get("/")
        async def index():
            index_path = STATIC_DIR / "index.html"
            if index_path.exists():
                return FileResponse(str(index_path))
            return {"message": "HorseShitBot Dashboard — no index.html found"}

        @app.get("/api/status")
        async def status():
            return bridge.snapshot()

        @app.get("/api/thermals")
        async def thermals():
            with bridge._lock:
                return bridge._state.get("thermals", [])

        @app.get("/api/controller-config")
        async def get_controller_config():
            cfg = _load_config()
            return {
                "buttons": cfg.get("buttons", DEFAULT_BUTTON_MAP),
                "available_actions": AVAILABLE_ACTIONS,
                "button_names": BUTTON_NAMES,
                "axes": AXIS_INFO,
                "defaults": DEFAULT_BUTTON_MAP,
            }

        @app.put("/api/controller-config")
        async def set_controller_config(body: ControllerConfigUpdate):
            valid_actions = set(AVAILABLE_ACTIONS.keys())
            valid_buttons = set(BUTTON_NAMES)

            cleaned = {}
            for btn, action in body.buttons.items():
                if btn not in valid_buttons:
                    continue
                if action not in valid_actions:
                    action = "none"
                cleaned[btn] = action

            for btn in BUTTON_NAMES:
                if btn not in cleaned:
                    cleaned[btn] = DEFAULT_BUTTON_MAP.get(btn, "none")

            cfg = {"buttons": cleaned}
            _save_config(cfg)

            msg = String()
            msg.data = json.dumps(cfg)
            ros_node._config_pub.publish(msg)

            return {"success": True, "buttons": cleaned}

        @app.put("/api/controller-config/defaults")
        async def save_controller_defaults(body: ControllerConfigUpdate):
            if _REPO_CONFIG_FILE is None:
                ros_node.get_logger().error("Cannot find repo root (no .git found)")
                return {"success": False, "message": "Cannot find repo root — is this a git checkout?"}

            valid_actions = set(AVAILABLE_ACTIONS.keys())
            valid_buttons = set(BUTTON_NAMES)

            cleaned = {}
            for btn, action in body.buttons.items():
                if btn not in valid_buttons:
                    continue
                if action not in valid_actions:
                    action = "none"
                cleaned[btn] = action

            for btn in BUTTON_NAMES:
                if btn not in cleaned:
                    cleaned[btn] = DEFAULT_BUTTON_MAP.get(btn, "none")

            cfg = {"buttons": cleaned}
            try:
                _REPO_CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
                with open(_REPO_CONFIG_FILE, "w") as f:
                    json.dump(cfg, f, indent=2)
                    f.write("\n")
                ros_node.get_logger().info(f"Saved repo defaults to {_REPO_CONFIG_FILE}")
                return {"success": True, "path": str(_REPO_CONFIG_FILE)}
            except Exception as e:
                ros_node.get_logger().error(f"Failed to save repo defaults: {e}")
                return {"success": False, "message": str(e)}

        @app.get("/api/params/{node_name}")
        async def get_params(node_name: str):
            return {"node": node_name, "note": "Use ros2 param list/get for now"}

        @app.put("/api/params/{node_name}")
        async def set_params(node_name: str, body: ParamUpdate):
            return {"node": node_name, "updated": list(body.params.keys()), "note": "Parameter update via rcl_interfaces pending"}

        @app.post("/api/command/{node_name}/{action}")
        async def command(node_name: str, action: str):
            service_map = {
                "stop": f"/{node_name}/stop",
                "stop_fast": f"/{node_name}/stop_fast",
                "reference": f"/{node_name}/reference",
                "open": f"/{node_name}/open",
                "close": f"/{node_name}/close",
            }
            srv_name = service_map.get(action)
            if not srv_name:
                return {"success": False, "message": f"unknown action: {action}"}
            return {"success": True, "message": f"called {srv_name}"}

        @app.post("/api/recording/start")
        async def start_recording():
            if not ros_node._rec_start_cli.service_is_ready():
                return {"success": False, "message": "bag_recorder_node not available"}
            future = ros_node._rec_start_cli.call_async(Trigger.Request())
            return {"success": True, "message": "start_recording called"}

        @app.post("/api/recording/stop")
        async def stop_recording():
            if not ros_node._rec_stop_cli.service_is_ready():
                return {"success": False, "message": "bag_recorder_node not available"}
            future = ros_node._rec_stop_cli.call_async(Trigger.Request())
            return {"success": True, "message": "stop_recording called"}

        @app.get("/api/bag-topics")
        async def get_bag_topics():
            """Return available/selected topics from the bag recorder status."""
            with bridge._lock:
                rec = bridge._state.get("bag_recorder", {})
            return {
                "available_topics": rec.get("available_topics", {}),
                "selected_topics": rec.get("selected_topics", []),
                "topic_groups": rec.get("topic_groups", {}),
                "recording": rec.get("recording", False),
            }

        @app.put("/api/bag-topics")
        async def set_bag_topics(body: dict):
            """Save selected topics to config file (bag recorder polls it)."""
            topics = body.get("topics", [])
            if not isinstance(topics, list) or not topics:
                return JSONResponse({"error": "topics must be a non-empty list"}, status_code=400)
            try:
                ros_node._bag_topics_file.parent.mkdir(parents=True, exist_ok=True)
                ros_node._bag_topics_file.write_text(json.dumps(topics, indent=2))
                ros_node.get_logger().info(f"Bag topics saved: {topics}")
                return {"success": True, "topics": topics}
            except Exception as e:
                return JSONResponse({"error": str(e)}, status_code=500)

        # ── Bag management ────────────────────────────────────────

        def _dir_size(p: Path) -> int:
            """Total bytes of all files under *p*."""
            if p.is_file():
                return p.stat().st_size
            return sum(f.stat().st_size for f in p.rglob("*") if f.is_file())

        @app.get("/api/bags")
        async def list_bags():
            bag_dir = ros_node._bag_dir
            if not bag_dir.is_dir():
                bag_dir.mkdir(parents=True, exist_ok=True)

            disk = shutil.disk_usage(str(bag_dir))

            bags = []
            for entry in sorted(bag_dir.iterdir(), reverse=True):
                if not entry.is_dir():
                    continue
                try:
                    size = _dir_size(entry)
                    mtime = max(f.stat().st_mtime for f in entry.rglob("*") if f.is_file())
                    files = [f.name for f in entry.iterdir() if f.is_file()]
                    bags.append({
                        "name": entry.name,
                        "size_bytes": size,
                        "modified": datetime.fromtimestamp(mtime).isoformat(),
                        "files": files,
                    })
                except Exception:
                    continue

            total = sum(b["size_bytes"] for b in bags)
            return {
                "bags": bags,
                "total_bytes": total,
                "disk_total": disk.total,
                "disk_used": disk.used,
                "disk_free": disk.free,
            }

        @app.get("/api/bags/{bag_name}/download")
        async def download_bag(bag_name: str):
            bag_path = ros_node._bag_dir / bag_name
            if not bag_path.is_dir() or ".." in bag_name:
                return JSONResponse({"error": "not found"}, status_code=404)

            all_files = [f for f in bag_path.iterdir() if f.is_file()]

            # Single file: serve directly (fast, no overhead)
            if len(all_files) == 1:
                f = all_files[0]
                return FileResponse(str(f), media_type="application/octet-stream", filename=f.name)

            # Multiple files: stream as uncompressed tar (no RAM spike)
            def _tar_stream():
                read_size = 1024 * 1024  # 1 MB chunks
                r_fd, w_fd = os.pipe()
                r_file = os.fdopen(r_fd, "rb")

                def _writer():
                    with os.fdopen(w_fd, "wb") as wf:
                        with tarfile.open(fileobj=wf, mode="w|") as tar:
                            tar.add(str(bag_path), arcname=bag_name)

                writer_thread = threading.Thread(target=_writer, daemon=True)
                writer_thread.start()
                try:
                    while True:
                        chunk = r_file.read(read_size)
                        if not chunk:
                            break
                        yield chunk
                finally:
                    r_file.close()
                    writer_thread.join(timeout=5)

            filename = f"{bag_name}.tar"
            return StreamingResponse(
                _tar_stream(),
                media_type="application/x-tar",
                headers={"Content-Disposition": f'attachment; filename="{filename}"'},
            )

        @app.delete("/api/bags/{bag_name}")
        async def delete_bag(bag_name: str):
            bag_path = ros_node._bag_dir / bag_name
            if not bag_path.is_dir() or ".." in bag_name:
                return JSONResponse({"error": "not found"}, status_code=404)
            try:
                shutil.rmtree(bag_path)
                ros_node.get_logger().info(f"Deleted bag: {bag_path}")
                return {"success": True}
            except Exception as e:
                ros_node.get_logger().error(f"Failed to delete bag {bag_path}: {e}")
                return JSONResponse({"error": str(e)}, status_code=500)

        # ── Network management ─────────────────────────────────────

        def _run_in_thread(fn, *args):
            """Run a blocking function in a thread so we don't stall the event loop."""
            loop = asyncio.get_event_loop()
            return loop.run_in_executor(None, fn, *args)

        @app.get("/api/network")
        async def get_network():
            ifaces = await _run_in_thread(netmgr.get_interfaces)
            return {"interfaces": [i.to_dict() for i in ifaces]}

        @app.get("/api/network/wifi/scan")
        async def wifi_scan(iface: str = ""):
            networks = await _run_in_thread(netmgr.wifi_scan, iface)
            return {"networks": [n.to_dict() for n in networks]}

        @app.post("/api/network/wifi/connect")
        async def wifi_connect(body: WifiConnectBody):
            return await _run_in_thread(
                netmgr.wifi_connect, body.ssid, body.password, body.iface
            )

        @app.post("/api/network/wifi/disconnect")
        async def wifi_disconnect(body: dict):
            iface = body.get("iface", "")
            if not iface:
                return JSONResponse({"success": False, "message": "iface required"}, status_code=400)
            return await _run_in_thread(netmgr.wifi_disconnect, iface)

        @app.put("/api/network/interface/{iface}/static")
        async def set_static_ip(iface: str, body: StaticIpBody):
            try:
                return await _run_in_thread(
                    netmgr.set_static, iface, body.ip, body.prefix, body.gateway, body.dns
                )
            except Exception as e:
                return JSONResponse(
                    {"success": False, "message": f"Failed to set static IP: {e}"},
                    status_code=500,
                )

        @app.put("/api/network/interface/{iface}/dhcp")
        async def set_dhcp_ip(iface: str):
            try:
                return await _run_in_thread(netmgr.set_dhcp, iface)
            except Exception as e:
                return JSONResponse(
                    {"success": False, "message": f"Failed to set DHCP: {e}"},
                    status_code=500,
                )

        @app.put("/api/network/interface/{iface}/dhcp-server")
        async def set_dhcp_server(iface: str, body: DhcpServerBody):
            if body.enabled:
                if not body.range_start or not body.range_end:
                    return JSONResponse(
                        {"success": False, "message": "range_start and range_end required"},
                        status_code=400,
                    )
                try:
                    return await _run_in_thread(
                        netmgr.enable_dhcp_server,
                        iface, body.range_start, body.range_end, body.lease_time,
                    )
                except Exception as e:
                    return JSONResponse(
                        {"success": False, "message": f"Failed to enable DHCP server: {e}"},
                        status_code=500,
                    )
            else:
                try:
                    return await _run_in_thread(netmgr.disable_dhcp_server, iface)
                except Exception as e:
                    return JSONResponse(
                        {"success": False, "message": f"Failed to disable DHCP server: {e}"},
                        status_code=500,
                    )

        @app.put("/api/network/interface/{iface}/ap")
        async def set_ap_mode(iface: str, body: ApModeBody):
            if body.enabled:
                if not body.ssid:
                    return JSONResponse(
                        {"success": False, "message": "SSID is required"},
                        status_code=400,
                    )
                if body.password and len(body.password) < 8:
                    return JSONResponse(
                        {"success": False, "message": "Password must be at least 8 characters"},
                        status_code=400,
                    )
                return await _run_in_thread(
                    netmgr.enable_ap, iface,
                    body.ssid, body.password, body.band, body.channel,
                )
            else:
                return await _run_in_thread(netmgr.disable_ap, iface)

        # ── Bluetooth ─────────────────────────────────────────────

        @app.post("/api/bluetooth/reconnect")
        async def bluetooth_reconnect(body: dict = {}):
            mac = body.get("mac", "")
            if not mac:
                gp = await _run_in_thread(bth.get_last_connected_gamepad)
                if not gp:
                    return JSONResponse(
                        {"success": False, "message": "No paired gamepad found"},
                        status_code=404,
                    )
                mac = gp["mac"]
            return await _run_in_thread(bth.connect_device, mac)

        @app.get("/api/bluetooth/gamepad")
        async def bluetooth_gamepad():
            gp = await _run_in_thread(bth.get_last_connected_gamepad)
            if not gp:
                return {"found": False}
            battery = gp.get("battery")
            if battery is None:
                battery = await _run_in_thread(bth.get_battery_level, gp["mac"])
            return {
                "found": True,
                "mac": gp["mac"],
                "name": gp["name"],
                "connected": gp["connected"],
                "battery": battery,
            }

        # ── MJPEG camera stream ───────────────────────────────────

        def _mjpeg_gen(get_frame, fps: int, quality: int, width: Optional[int]):
            """Yield JPEG frames as multipart chunks."""
            import time as _time
            interval = 1.0 / max(1, min(30, fps))
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality] if _HAS_CV2 else []

            while True:
                frame = get_frame()
                if frame is None:
                    _time.sleep(interval)
                    continue

                if width and _HAS_CV2 and frame.shape[1] != width:
                    h = int(frame.shape[0] * width / frame.shape[1])
                    frame = cv2.resize(frame, (width, h))

                if _HAS_CV2:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    ok, buf = cv2.imencode(".jpg", frame_bgr, encode_params)
                    if not ok:
                        _time.sleep(interval)
                        continue
                    jpg = buf.tobytes()
                else:
                    from PIL import Image as PilImage
                    pil_img = PilImage.fromarray(frame)
                    bio = BytesIO()
                    pil_img.save(bio, format="JPEG", quality=quality)
                    jpg = bio.getvalue()

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpg)).encode() + b"\r\n"
                    b"\r\n" + jpg + b"\r\n"
                )
                _time.sleep(interval)

        @app.get("/api/stream/color")
        async def stream_color(fps: int = 10, quality: int = 50, width: Optional[int] = None):
            return StreamingResponse(
                _mjpeg_gen(bridge.get_color_frame, fps, quality, width),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        @app.get("/api/stream/depth")
        async def stream_depth(fps: int = 10, quality: int = 50, width: Optional[int] = None):
            return StreamingResponse(
                _mjpeg_gen(bridge.get_depth_frame, fps, quality, width),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        @app.websocket("/ws")
        async def websocket_endpoint(ws: WebSocket):
            await ws.accept()
            try:
                while True:
                    data = bridge.snapshot()
                    await ws.send_json(data)
                    await asyncio.sleep(0.1)
            except WebSocketDisconnect:
                pass

        return app

    def _run_server(self):
        uvicorn.run(self._app, host=self._host, port=self._port, log_level="warning")


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

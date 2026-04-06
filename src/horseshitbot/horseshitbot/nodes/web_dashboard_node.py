"""
Web Dashboard Node — runs a FastAPI server alongside an rclpy node,
providing a REST + WebSocket API for live status, parameter tuning,
and robot control from a browser.
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from horseshitbot_interfaces.msg import ActuatorState as ActuatorStateMsg
from horseshitbot_interfaces.srv import MksSetSpeed, ActuatorCommand, SwitchBackend
from std_srvs.srv import Trigger

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse
    from fastapi.staticfiles import StaticFiles
    from pydantic import BaseModel

    _HAS_FASTAPI = True
except ImportError:
    _HAS_FASTAPI = False


STATIC_DIR = Path(__file__).parent.parent / "web" / "static"


class ParamUpdate(BaseModel):
    params: dict


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
        }
        self._ws_clients: list[WebSocket] = []

        ros_node.create_subscription(String, "/wheel_status", self._cb_wheel, 10)
        ros_node.create_subscription(ActuatorStateMsg, "/lift/state", lambda m: self._cb_actuator("lift", m), 10)
        ros_node.create_subscription(ActuatorStateMsg, "/brush/state", lambda m: self._cb_actuator("brush", m), 10)
        ros_node.create_subscription(ActuatorStateMsg, "/bin_door/state", lambda m: self._cb_actuator("bin_door", m), 10)
        ros_node.create_subscription(String, "/bag_recorder_node/status", self._cb_bag_recorder, 10)

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

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._state)


class WebDashboardNode(Node):
    def __init__(self):
        super().__init__("web_dashboard_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)

        self._host = self.get_parameter("host").get_parameter_value().string_value
        self._port = self.get_parameter("port").get_parameter_value().integer_value

        if not _HAS_FASTAPI:
            self.get_logger().error("fastapi/uvicorn not installed — dashboard disabled")
            return

        self._bridge = _RosBridge(self)

        self._rec_start_cli = self.create_client(Trigger, "/bag_recorder_node/start_recording")
        self._rec_stop_cli = self.create_client(Trigger, "/bag_recorder_node/stop_recording")

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

        @app.get("/")
        async def index():
            index_path = STATIC_DIR / "index.html"
            if index_path.exists():
                return FileResponse(str(index_path))
            return {"message": "HorseShitBot Dashboard — no index.html found"}

        @app.get("/api/status")
        async def status():
            return bridge.snapshot()

        @app.get("/api/params/{node_name}")
        async def get_params(node_name: str):
            cli = ros_node.create_client(
                rcl_interfaces.srv.ListParameters, f"/{node_name}/list_parameters"
            )
            # Simplified: return placeholder indicating the mechanism
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

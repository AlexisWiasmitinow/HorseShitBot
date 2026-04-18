"""
MKS Bus Node — owns the Modbus RTU serial connection and exposes ROS 2
services so other nodes can command MKS servo motors without serial
port contention.  Publishes per-motor connectivity on /mks_bus/status.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from horseshitbot_interfaces.srv import MksSetSpeed, MksMoveTurns, MksSetCurrent
from std_srvs.srv import Trigger

from ..drivers.mks_bus import BusCfg, MksBus, MODE_SR_CLOSE

_DEFAULTS_FILE = Path.home() / ".config" / "horseshitbot" / "motor_defaults.json"


class MksBusNode(Node):
    def __init__(self):
        super().__init__("mks_bus_node")

        self.declare_parameter("port", "/dev/mksbus")
        self.declare_parameter("baud", 38400)
        self.declare_parameter("timeout", 0.35)
        self.declare_parameter("retries", 3)
        self.declare_parameter("motor_ids", [3, 4, 5, 6])
        self.declare_parameter("health_hz", 0.5)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        timeout = self.get_parameter("timeout").get_parameter_value().double_value
        retries = self.get_parameter("retries").get_parameter_value().integer_value
        self._motor_ids = list(
            self.get_parameter("motor_ids").get_parameter_value().integer_array_value
        )
        health_hz = self.get_parameter("health_hz").get_parameter_value().double_value

        self._bus = MksBus(BusCfg(port=port, baud=baud, timeout=timeout, retries=retries))
        self._bus_connected = False

        self.create_service(MksSetSpeed, "/mks/set_speed", self._srv_set_speed)
        self.create_service(MksMoveTurns, "/mks/move_turns", self._srv_move_turns)
        self.create_service(MksSetCurrent, "/mks/set_current", self._srv_set_current)
        self.create_service(Trigger, "/mks/save_current_defaults", self._srv_save_defaults)
        self.create_service(Trigger, "/mks/init_servo", self._srv_init_servo)
        self.create_service(Trigger, "/mks/clear_errors", self._srv_clear_errors)

        self._status_pub = self.create_publisher(String, "/mks_bus/status", 10)
        self._motor_online: dict[int, bool] = {m: False for m in self._motor_ids}
        self._motor_current: dict[int, dict] = {}

        try:
            self._bus.connect()
            self._bus_connected = True
            self.get_logger().info(f"MKS bus connected on {port} @ {baud}")

            for mid in self._motor_ids:
                try:
                    self._bus.init_servo(mid, mode=MODE_SR_CLOSE, enable=True)
                    self._motor_online[mid] = True
                    self.get_logger().info(f"Motor {mid}: init + enabled")
                except Exception as e:
                    self.get_logger().warning(f"Motor {mid}: init failed: {e}")

            online_ids = [m for m in self._motor_ids if self._motor_online[m]]
            if online_ids:
                import time
                time.sleep(0.1)
                self._apply_saved_defaults(online_ids)
        except Exception as e:
            self.get_logger().error(f"MKS bus connection failed: {e}")

        period = 1.0 / max(0.1, health_hz)
        self.create_timer(period, self._health_tick)

    def _load_defaults(self) -> dict:
        try:
            if _DEFAULTS_FILE.exists():
                return json.loads(_DEFAULTS_FILE.read_text())
        except Exception:
            pass
        return {}

    def _save_defaults(self, defaults: dict):
        _DEFAULTS_FILE.parent.mkdir(parents=True, exist_ok=True)
        _DEFAULTS_FILE.write_text(json.dumps(defaults, indent=2))

    def _apply_saved_defaults(self, motor_ids: list[int] | None = None):
        defaults = self._load_defaults()
        if not defaults:
            return
        for mid in (motor_ids or self._motor_ids):
            cfg = defaults.get(str(mid))
            if not cfg:
                continue
            try:
                if cfg.get("run_current_ma"):
                    self._bus.set_run_current(mid, cfg["run_current_ma"])
                if cfg.get("hold_current_pct"):
                    self._bus.set_hold_current_pct(mid, cfg["hold_current_pct"])
                self._motor_current[mid] = dict(cfg)
                self.get_logger().info(
                    f"Motor {mid}: applied defaults run={cfg.get('run_current_ma')}mA "
                    f"hold={cfg.get('hold_current_pct')}%"
                )
            except Exception as e:
                self.get_logger().warning(f"Motor {mid}: apply defaults failed: {e}")

    def _srv_save_defaults(self, request, response):
        defaults = {}
        for mid in self._motor_ids:
            cached = self._motor_current.get(mid)
            if cached:
                defaults[str(mid)] = dict(cached)
        try:
            self._save_defaults(defaults)
            response.success = True
            response.message = f"saved defaults for motors {list(defaults.keys())}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _health_tick(self):
        motor_info = {}
        for mid in self._motor_ids:
            online = self._bus.probe(mid)
            self._motor_online[mid] = online
            info = {"online": online}
            cached = self._motor_current.get(mid, {})
            if cached:
                info.update(cached)
            motor_info[str(mid)] = info

        msg = String()
        msg.data = json.dumps({
            "bus_connected": self._bus_connected,
            "motors": {str(m): self._motor_online[m] for m in self._motor_ids},
            "motor_info": motor_info,
        })
        self._status_pub.publish(msg)

    def _srv_set_speed(self, request, response):
        try:
            self._bus.set_speed_signed(
                unit_id=int(request.motor_id),
                rpm_signed=float(request.rpm),
                acc=int(request.accel),
                invert_dir=bool(request.invert_dir),
            )
            response.success = True
        except Exception as e:
            self.get_logger().warning(f"set_speed failed motor={request.motor_id}: {e}")
            response.success = False
        return response

    def _srv_move_turns(self, request, response):
        try:
            self._bus.move_turns(
                unit_id=int(request.motor_id),
                turns=float(request.turns),
                speed_rpm=int(request.speed_rpm) if request.speed_rpm > 0 else 300,
                acc=int(request.accel) if request.accel > 0 else 3,
                invert_dir=bool(request.invert_dir),
            )
            response.success = True
            response.message = f"moving {request.turns} turns"
        except Exception as e:
            self.get_logger().warning(f"move_turns failed motor={request.motor_id}: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _srv_set_current(self, request, response):
        try:
            mid = int(request.motor_id)
            if mid not in self._motor_current:
                self._motor_current[mid] = {}
            if request.run_current_ma > 0:
                self._bus.set_run_current(mid, int(request.run_current_ma))
                self._motor_current[mid]["run_current_ma"] = int(request.run_current_ma)
            if request.hold_current_pct > 0:
                self._bus.set_hold_current_pct(mid, int(request.hold_current_pct))
                self._motor_current[mid]["hold_current_pct"] = int(request.hold_current_pct)
            response.success = True
            response.message = f"motor {mid}: run={request.run_current_ma}mA hold={request.hold_current_pct}%"
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().warning(f"set_current failed motor={request.motor_id}: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _srv_init_servo(self, request, response):
        try:
            for mid in range(1, 7):
                try:
                    self._bus.init_servo(mid, mode=MODE_SR_CLOSE, enable=True)
                except Exception:
                    pass
            response.success = True
            response.message = "init_servo done"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _srv_clear_errors(self, request, response):
        try:
            for mid in range(1, 7):
                try:
                    self._bus.clear_error_state(mid, mode=MODE_SR_CLOSE)
                except Exception:
                    pass
            response.success = True
            response.message = "errors cleared"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MksBusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

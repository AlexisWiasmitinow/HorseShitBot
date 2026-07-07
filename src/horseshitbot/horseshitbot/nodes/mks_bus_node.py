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
        self.declare_parameter("microsteps", 16)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        timeout = self.get_parameter("timeout").get_parameter_value().double_value
        retries = self.get_parameter("retries").get_parameter_value().integer_value
        self._motor_ids = list(
            self.get_parameter("motor_ids").get_parameter_value().integer_array_value
        )
        health_hz = self.get_parameter("health_hz").get_parameter_value().double_value
        self._microsteps = self.get_parameter("microsteps").get_parameter_value().integer_value

        self._bus = MksBus(BusCfg(port=port, baud=baud, timeout=timeout, retries=retries))
        self._bus_connected = False

        self.create_service(MksSetSpeed, "/mks/set_speed", self._srv_set_speed)
        self.create_service(MksMoveTurns, "/mks/move_turns", self._srv_move_turns)
        self.create_service(MksSetCurrent, "/mks/set_current", self._srv_set_current)
        self.create_service(Trigger, "/mks/save_current_defaults", self._srv_save_defaults)
        self.create_service(Trigger, "/mks/init_servo", self._srv_init_servo)
        self.create_service(Trigger, "/mks/clear_errors", self._srv_clear_errors)
        # Triggers the MKS REG_EMERGENCY_STOP (0xF7) on the wheel motors for
        # an instant hardware-level halt, independent of any ROS-side ramp.
        self.create_service(Trigger, "/mks/emergency_stop_wheels",
                            self._srv_emergency_stop_wheels)
        # Clears the latch left behind by emergency_stop_wheels — required
        # to actually drive again after an e-stop. Wheel driver calls this
        # on resume; can also be invoked manually for recovery.
        self.create_service(Trigger, "/mks/release_emergency_stop_wheels",
                            self._srv_release_emergency_stop_wheels)
        self.declare_parameter("wheel_motor_ids", [1, 2])
        self._wheel_motor_ids = list(
            self.get_parameter("wheel_motor_ids").get_parameter_value().integer_array_value
        )

        self._status_pub = self.create_publisher(String, "/mks_bus/status", 10)
        self._motor_online: dict[int, bool] = {m: False for m in self._motor_ids}
        self._motor_current: dict[int, dict] = {}

        try:
            self._bus.connect()
            self._bus_connected = True
            self.get_logger().info(f"MKS bus connected on {port} @ {baud}")
            self._scan_motors(reinit_new=True)
        except Exception as e:
            self.get_logger().error(f"MKS bus connection failed: {e}")

        # No periodic probing — the bus is otherwise idle when the robot
        # isn't moving, so polling just burns CPU and adds latency to
        # /mks/set_speed calls on the single-threaded executor. We still
        # republish the last known status periodically so newly-connected
        # WebSocket clients in the dashboard don't have to wait for the
        # next manual scan to see anything.
        period = 1.0 / max(0.1, health_hz)
        self.create_timer(period, self._publish_status)
        self.create_service(Trigger, "/mks/scan", self._srv_scan)

    def _scan_motors(self, reinit_new: bool = False):
        """Probe every motor_id and update _motor_online. If reinit_new is
        True, run init_servo for motors that have just come online (or that
        were online at start-up and we're scanning for the first time)."""
        for mid in self._motor_ids:
            was_online = self._motor_online.get(mid, False)
            try:
                online = self._bus.probe(mid)
            except Exception:
                online = False
            self._motor_online[mid] = online

            if not online:
                if was_online:
                    self.get_logger().warning(f"Motor {mid}: went offline")
                else:
                    self.get_logger().info(f"Motor {mid}: not detected (skipping init)")
                continue

            if not was_online or reinit_new:
                try:
                    self._bus.init_servo(
                        mid, mode=MODE_SR_CLOSE,
                        microsteps=self._microsteps, enable=True,
                    )
                    if was_online:
                        self.get_logger().info(f"Motor {mid}: re-initialised")
                    else:
                        self.get_logger().info(
                            f"Motor {mid}: detected, init mode=4 subdiv={self._microsteps}"
                        )
                except Exception as e:
                    self.get_logger().warning(f"Motor {mid}: init failed: {e}")
                    self._motor_online[mid] = False

        online_ids = [m for m in self._motor_ids if self._motor_online[m]]
        if online_ids:
            import time
            time.sleep(0.05)
            self._apply_saved_defaults(online_ids)
        self._publish_status()

    def _srv_scan(self, request, response):
        """Manually re-probe every motor on the bus. Use this from the web UI
        (or via `ros2 service call /mks/scan std_srvs/srv/Trigger`) after
        plugging in a previously-missing motor — no need to restart the
        stack."""
        try:
            self._scan_motors(reinit_new=True)
            online = [m for m in self._motor_ids if self._motor_online[m]]
            offline = [m for m in self._motor_ids if not self._motor_online[m]]
            response.success = True
            response.message = f"scan complete — online={online} offline={offline}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"scan failed: {e}"
            self.get_logger().warning(response.message)
        return response

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
        self.get_logger().info(f"Defaults file: {_DEFAULTS_FILE} (exists={_DEFAULTS_FILE.exists()})")
        if not defaults:
            self.get_logger().info("No saved defaults found")
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

    def _publish_status(self):
        """Republish the last-known motor status. Does NOT touch the Modbus
        bus — call _scan_motors first if you want fresh data. Runs on the
        ROS timer so that late WebSocket subscribers still see something."""
        motor_info = {}
        for mid in self._motor_ids:
            info = {"online": self._motor_online.get(mid, False)}
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
        mid = int(request.motor_id)
        turns = float(request.turns)
        self.get_logger().info(
            f"move_turns: motor={mid} turns={turns} speed={request.speed_rpm} acc={request.accel}"
        )
        try:
            self._bus.move_turns(
                unit_id=mid,
                turns=turns,
                speed_rpm=int(request.speed_rpm) if request.speed_rpm > 0 else 300,
                acc=int(request.accel) if request.accel > 0 else 3,
                invert_dir=bool(request.invert_dir),
                closed_loop=True,
                microsteps=self._microsteps,
            )
            response.success = True
            response.message = f"moving {turns} turns"
            self.get_logger().info(f"move_turns: motor={mid} command sent")
        except Exception as e:
            self.get_logger().warning(f"move_turns failed motor={mid}: {e}")
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
                    self._bus.init_servo(mid, mode=MODE_SR_CLOSE,
                                        microsteps=self._microsteps, enable=True)
                except Exception:
                    pass
            response.success = True
            response.message = "init_servo done"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _srv_emergency_stop_wheels(self, request, response):
        """Slam the wheel motors via MKS REG_EMERGENCY_STOP. Returns success
        even if some motors didn't acknowledge — at e-stop time we'd rather
        send the command on the others than abort the whole thing."""
        failed = []
        for mid in self._wheel_motor_ids:
            try:
                self._bus.emergency_stop(mid)
            except Exception as exc:
                failed.append(f"{mid}:{exc}")
        response.success = not failed
        response.message = (
            "wheel e-stop sent"
            if not failed
            else f"wheel e-stop partial fail: {', '.join(failed)}"
        )
        if failed:
            self.get_logger().warning(response.message)
        return response

    def _srv_release_emergency_stop_wheels(self, request, response):
        """Release the e-stop latch on the wheel motors so they accept
        set_speed commands again. Without this, REG_SPEED writes are
        silently ignored even though they return success."""
        failed = []
        for mid in self._wheel_motor_ids:
            try:
                self._bus.release_emergency_stop(mid)
            except Exception as exc:
                failed.append(f"{mid}:{exc}")
        response.success = not failed
        response.message = (
            "wheel e-stop released"
            if not failed
            else f"wheel e-stop release partial fail: {', '.join(failed)}"
        )
        if failed:
            self.get_logger().warning(response.message)
        else:
            self.get_logger().info(response.message)
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

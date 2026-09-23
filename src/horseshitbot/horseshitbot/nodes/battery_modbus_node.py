"""Battery monitor using the shared Modbus bus owner's read service."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from horseshitbot_interfaces.srv import ModbusReadHoldingRegister

from ..drivers.modbus_adc import AdcRegisterMap


class BatteryModbusNode(Node):
    """Poll battery voltage without ever opening a serial device."""

    def __init__(self):
        super().__init__("battery_modbus_node")

        self.declare_parameter("slave_id", 33)
        self.declare_parameter("channel", 1)
        self.declare_parameter("read_period_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 0.2)
        self.declare_parameter("channel_base_register", 0)
        self.declare_parameter("raw_scale", 100.0)
        self.declare_parameter("voltage_divider_factor", 0.5)
        self.declare_parameter("voltage_offset", 0.0)
        self.declare_parameter("voltage_multiplier", 1.0)
        self.declare_parameter("low_voltage", 22.0)
        self.declare_parameter("critical_voltage", 21.0)
        self.declare_parameter("critical_confirmations", 3)
        self.declare_parameter("shutdown_enabled", False)

        self._slave_id = int(self.get_parameter("slave_id").value)
        self._channel = int(self.get_parameter("channel").value)
        self._period = max(0.1, float(self.get_parameter("read_period_sec").value))
        self._request_timeout = max(
            0.05, float(self.get_parameter("request_timeout_sec").value)
        )
        self._map = AdcRegisterMap(
            channel_base_register=int(
                self.get_parameter("channel_base_register").value
            ),
            raw_scale=float(self.get_parameter("raw_scale").value),
            voltage_divider_factor=float(
                self.get_parameter("voltage_divider_factor").value
            ),
            voltage_offset=float(self.get_parameter("voltage_offset").value),
            voltage_multiplier=float(
                self.get_parameter("voltage_multiplier").value
            ),
        )
        self._register = self._map.channel_register(self._channel)
        self._low_voltage = float(self.get_parameter("low_voltage").value)
        self._critical_voltage = float(
            self.get_parameter("critical_voltage").value
        )
        self._critical_confirmations = max(
            1, int(self.get_parameter("critical_confirmations").value)
        )

        if bool(self.get_parameter("shutdown_enabled").value):
            self.get_logger().warning(
                "shutdown_enabled is ignored; OS shutdown is not implemented"
            )

        self._lock = threading.Lock()
        self._pending = None
        self._pending_deadline = 0.0
        self._last_reading: dict | None = None
        self._last_error: str | None = None
        self._critical_counter = 0
        self._last_error_log = 0.0

        self._read_client = self.create_client(
            ModbusReadHoldingRegister, "/modbus/read_holding_register"
        )
        self._voltage_pub = self.create_publisher(
            Float32, "/battery/voltage", 10
        )
        self._low_pub = self.create_publisher(Bool, "/battery/low", 10)
        self._critical_pub = self.create_publisher(
            Bool, "/battery/critical", 10
        )
        self._state_pub = self.create_publisher(
            BatteryState, "/battery/status", 10
        )
        self._json_pub = self.create_publisher(
            String, "/battery/status_json", 10
        )
        self.create_service(
            Trigger, "~/get_battery_voltage", self._srv_get_voltage
        )
        self.create_timer(self._period, self._poll)

        self.get_logger().info(
            "Battery monitor started: "
            f"shared_service=/modbus/read_holding_register "
            f"slave={self._slave_id} channel={self._channel} "
            f"register=0x{self._register:04X} period={self._period:.3f}s"
        )

    def _poll(self):
        now = time.monotonic()
        with self._lock:
            pending = self._pending
            deadline = self._pending_deadline

        if pending is not None:
            if now < deadline:
                return
            try:
                self._read_client.remove_pending_request(pending)
            except Exception:
                pass
            pending.cancel()
            with self._lock:
                if self._pending is pending:
                    self._pending = None
                    self._last_error = "shared-bus service request timed out"
            self._log_read_error("shared-bus service request timed out")

        if not self._read_client.service_is_ready():
            self._set_error("shared-bus read service unavailable")
            return

        request = ModbusReadHoldingRegister.Request()
        request.device_id = self._slave_id
        request.address = self._register
        future = self._read_client.call_async(request)
        with self._lock:
            self._pending = future
            self._pending_deadline = now + self._request_timeout
        future.add_done_callback(self._read_done)

    def _read_done(self, future):
        with self._lock:
            if self._pending is not future:
                return
            self._pending = None

        try:
            response = future.result()
            if response is None:
                raise RuntimeError("empty shared-bus service response")
            if response.busy:
                return
            if not response.success:
                raise RuntimeError(response.message or "ADC read failed")
            reading = self._map.convert(response.value, self._channel)
        except Exception as exc:
            self._set_error(str(exc))
            return

        voltage = float(reading["battery_voltage"])
        low = voltage <= self._low_voltage
        critical_sample = voltage <= self._critical_voltage
        if critical_sample:
            self._critical_counter += 1
        else:
            self._critical_counter = 0
        critical = self._critical_counter >= self._critical_confirmations

        with self._lock:
            self._last_reading = reading
            self._last_error = None
        self._publish(reading, low, critical)

    def _set_error(self, message: str):
        with self._lock:
            self._last_error = message
        self._log_read_error(message)

    def _log_read_error(self, message: str):
        now = time.monotonic()
        if now - self._last_error_log >= 30.0:
            self._last_error_log = now
            self.get_logger().warning(f"ADC read failed: {message}")

    def _publish(self, reading: dict, low: bool, critical: bool):
        voltage = float(reading["battery_voltage"])
        self._voltage_pub.publish(Float32(data=voltage))
        self._low_pub.publish(Bool(data=low))
        self._critical_pub.publish(Bool(data=critical))

        state = BatteryState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.voltage = voltage
        state.percentage = math.nan
        state.present = True
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if critical:
            state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif low:
            state.power_supply_health = (
                BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            )
        else:
            state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self._state_pub.publish(state)

        status = dict(reading)
        status.update(
            available=True,
            low=bool(low),
            critical=bool(critical),
            slave_id=self._slave_id,
        )
        self._json_pub.publish(String(data=json.dumps(status)))

    def _srv_get_voltage(self, request, response):
        del request
        with self._lock:
            reading = self._last_reading
            error = self._last_error
        if reading is None:
            response.success = False
            response.message = f"No battery voltage yet. Last error: {error}"
            return response
        response.success = True
        response.message = (
            f"battery_voltage={reading['battery_voltage']:.3f} V, "
            f"adc_voltage={reading['adc_voltage']:.3f} V, "
            f"raw={reading['raw']}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BatteryModbusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

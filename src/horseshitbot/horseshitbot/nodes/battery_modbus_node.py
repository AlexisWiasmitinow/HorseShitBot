"""Battery monitor using the shared Modbus bus owner's read service."""

from __future__ import annotations

import json
import subprocess
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from horseshitbot_interfaces.srv import ModbusReadHoldingRegister

from ..drivers.modbus_adc import (
    DEFAULT_SOC_PERCENT_POINTS,
    DEFAULT_SOC_VOLTAGE_POINTS,
    AdcRegisterMap,
    CRITICAL,
    LOW,
    LowVoltageMonitor,
    MedianVoltageFilter,
    state_of_charge_percent,
    validate_soc_curve,
    voltage_to_raw,
)


class BatteryModbusNode(Node):
    """Poll battery voltage without ever opening a serial device."""

    def __init__(self):
        super().__init__("battery_modbus_node")

        self.declare_parameter("slave_id", 33)
        self.declare_parameter("channel", 1)
        self.declare_parameter("read_period_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 0.2)
        self.declare_parameter("channel_base_register", 0)
        self.declare_parameter("raw_offset_counts", 15.2)
        self.declare_parameter("counts_per_volt", 49.25)
        self.declare_parameter("low_voltage", 22.0)
        self.declare_parameter("critical_voltage", 20.0)
        self.declare_parameter("critical_hold_sec", 5.0)
        self.declare_parameter("shutdown_enabled", True)
        self.declare_parameter(
            "safe_stop_services",
            [
                "/wheel_driver_node/stop_fast",
                "/lift/stop",
                "/brush/stop",
                "/bin_door/stop",
            ],
        )
        self.declare_parameter("safe_stop_grace_sec", 3.0)
        self.declare_parameter(
            "shutdown_command",
            ["sudo", "-n", "/usr/sbin/shutdown", "-h", "now"],
        )
        self.declare_parameter(
            "soc_voltage_points", list(DEFAULT_SOC_VOLTAGE_POINTS)
        )
        self.declare_parameter(
            "soc_percent_points", list(DEFAULT_SOC_PERCENT_POINTS)
        )
        self.declare_parameter("soc_median_window", 30)

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
            raw_offset_counts=float(
                self.get_parameter("raw_offset_counts").value
            ),
            counts_per_volt=float(self.get_parameter("counts_per_volt").value),
        )
        self._register = self._map.channel_register(self._channel)
        self._monitor = LowVoltageMonitor(
            low_voltage=float(self.get_parameter("low_voltage").value),
            critical_voltage=float(self.get_parameter("critical_voltage").value),
            critical_hold_sec=float(
                self.get_parameter("critical_hold_sec").value
            ),
        )
        self._shutdown_enabled = bool(
            self.get_parameter("shutdown_enabled").value
        )
        self._safe_stop_services = [
            str(name)
            for name in self.get_parameter("safe_stop_services").value
            if str(name).strip()
        ]
        self._safe_stop_grace = max(
            0.0, float(self.get_parameter("safe_stop_grace_sec").value)
        )
        self._shutdown_command = [
            str(part) for part in self.get_parameter("shutdown_command").value
        ]

        try:
            self._soc_voltages, self._soc_percents = validate_soc_curve(
                self.get_parameter("soc_voltage_points").value,
                self.get_parameter("soc_percent_points").value,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"invalid soc curve ({exc}); using built-in 2026-09-25 defaults"
            )
            self._soc_voltages = DEFAULT_SOC_VOLTAGE_POINTS
            self._soc_percents = DEFAULT_SOC_PERCENT_POINTS

        soc_window = int(self.get_parameter("soc_median_window").value)
        if soc_window < 1:
            self.get_logger().warning(
                f"soc_median_window={soc_window} is invalid; "
                "using 1 (unfiltered)"
            )
            soc_window = 1
        # Display smoothing only. LowVoltageMonitor keeps receiving the
        # instantaneous voltage so the hold timer and the shutdown decision
        # are never delayed by this window.
        self._soc_filter = MedianVoltageFilter(soc_window)

        self._lock = threading.Lock()
        self._pending = None
        self._pending_deadline = 0.0
        self._last_reading: dict | None = None
        self._last_status = None
        self._last_error: str | None = None
        self._last_error_log = 0.0
        self._last_state = None
        self._last_critical_log = 0.0
        self._shutdown_thread: threading.Thread | None = None

        self._read_client = self.create_client(
            ModbusReadHoldingRegister, "/modbus/read_holding_register"
        )
        self._safe_stop_clients = {
            name: self.create_client(Trigger, name)
            for name in self._safe_stop_services
        }
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
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
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
        self.get_logger().info(
            "Battery calibration: voltage = (raw + "
            f"{self._map.raw_offset_counts}) / {self._map.counts_per_volt} "
            f"| low <= {self._monitor.low_voltage:.1f} V "
            f"(raw {voltage_to_raw(self._monitor.low_voltage, self._map.raw_offset_counts, self._map.counts_per_volt):.0f}) "
            f"| critical <= {self._monitor.critical_voltage:.1f} V "
            f"(raw {voltage_to_raw(self._monitor.critical_voltage, self._map.raw_offset_counts, self._map.counts_per_volt):.0f})"
        )
        if self._shutdown_enabled:
            self.get_logger().info(
                "Shutdown armed: critical voltage held for "
                f"{self._monitor.critical_hold_sec:.1f}s triggers safe stop "
                f"then {' '.join(self._shutdown_command)}"
            )
        else:
            self.get_logger().warning(
                "shutdown_enabled is false; low battery will only be reported"
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
        status = self._monitor.update(voltage, time.monotonic())

        # The safety decision above is already settled from the instantaneous
        # voltage. Everything from here is display telemetry and must not be
        # able to influence it.
        smoothed = self._soc_filter.update(voltage)
        reading["battery_voltage_filtered"] = smoothed
        reading["percentage"] = state_of_charge_percent(
            smoothed, self._soc_voltages, self._soc_percents
        ) / 100.0

        with self._lock:
            self._last_reading = reading
            self._last_status = status
            self._last_error = None
        self._log_state(reading, status)
        self._publish(reading, status)

        if status.shutdown_due:
            self._request_shutdown(reading, status)

    def _set_error(self, message: str):
        with self._lock:
            self._last_error = message
        # A failed read is not evidence of a healthy battery, but it is also
        # not a critical sample. Drop the hold timer so only contiguous
        # confirmed readings can add up to a shutdown.
        self._monitor.reset()
        self._log_read_error(message)

    def _log_read_error(self, message: str):
        now = time.monotonic()
        if now - self._last_error_log >= 30.0:
            self._last_error_log = now
            self.get_logger().warning(f"ADC read failed: {message}")

    def _log_state(self, reading: dict, status):
        voltage = float(reading["battery_voltage"])
        if status.state != self._last_state:
            self._last_state = status.state
            message = (
                f"Battery {status.state}: {voltage:.3f} V "
                f"(raw={reading['raw']})"
            )
            if status.state == CRITICAL:
                self.get_logger().error(message)
            elif status.state == LOW:
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
        elif status.state == CRITICAL:
            if self._monitor.shutdown_latched:
                # The shutdown sequence already ran; throttle so a Jetson that
                # stays powered does not flood the log at the poll rate.
                now = time.monotonic()
                if now - self._last_critical_log < 30.0:
                    return
                self._last_critical_log = now
            self.get_logger().error(
                f"Battery critical for {status.critical_for_sec:.1f}s / "
                f"{self._monitor.critical_hold_sec:.1f}s: {voltage:.3f} V"
            )

    def _publish(self, reading: dict, status):
        voltage = float(reading["battery_voltage"])
        self._voltage_pub.publish(Float32(data=voltage))
        self._low_pub.publish(Bool(data=status.low))
        self._critical_pub.publish(Bool(data=status.critical))

        state = BatteryState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.voltage = voltage
        # BatteryState.percentage is a 0..1 fraction, not 0..100.
        state.percentage = float(reading["percentage"])
        state.present = True
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if status.critical:
            state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif status.low:
            state.power_supply_health = (
                BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            )
        else:
            state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self._state_pub.publish(state)

        payload = dict(reading)
        payload.update(
            available=True,
            state=status.state,
            low=bool(status.low),
            critical=bool(status.critical),
            critical_for_sec=round(status.critical_for_sec, 2),
            critical_hold_sec=self._monitor.critical_hold_sec,
            low_voltage=self._monitor.low_voltage,
            critical_voltage=self._monitor.critical_voltage,
            shutdown_enabled=self._shutdown_enabled,
            shutdown_requested=self._monitor.shutdown_latched,
            slave_id=self._slave_id,
        )
        self._json_pub.publish(String(data=json.dumps(payload)))
        self._publish_diagnostics(payload, status)

    def _publish_diagnostics(self, payload: dict, status):
        diag = DiagnosticStatus()
        diag.name = "battery: pack voltage"
        diag.hardware_id = f"n43ic04:{self._slave_id}:ch{self._channel}"
        if status.critical:
            diag.level = DiagnosticStatus.ERROR
        elif status.low:
            diag.level = DiagnosticStatus.WARN
        else:
            diag.level = DiagnosticStatus.OK
        diag.message = (
            f"{payload['battery_voltage']:.3f} V ({status.state})"
        )
        diag.values = [
            KeyValue(key=key, value=str(value))
            for key, value in (
                ("voltage_v", round(payload["battery_voltage"], 3)),
                ("raw", payload["raw"]),
                ("state", status.state),
                ("low_voltage_v", self._monitor.low_voltage),
                ("critical_voltage_v", self._monitor.critical_voltage),
                ("critical_for_sec", round(status.critical_for_sec, 2)),
                ("critical_hold_sec", self._monitor.critical_hold_sec),
                ("shutdown_enabled", self._shutdown_enabled),
                ("shutdown_requested", self._monitor.shutdown_latched),
            )
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diag]
        self._diag_pub.publish(array)

    def _request_shutdown(self, reading: dict, status):
        """Run the safe-stop then power-off sequence exactly once."""
        voltage = float(reading["battery_voltage"])
        with self._lock:
            if self._shutdown_thread is not None:
                return
            thread = threading.Thread(
                target=self._shutdown_sequence,
                args=(voltage,),
                name="battery_shutdown",
                daemon=True,
            )
            self._shutdown_thread = thread

        self.get_logger().error(
            f"Battery critical at {voltage:.3f} V for "
            f"{status.critical_for_sec:.1f}s — stopping motors and shutting down"
        )
        thread.start()

    def _shutdown_sequence(self, voltage: float):
        self._call_safe_stop()

        if not self._shutdown_enabled:
            self.get_logger().error(
                "Motors stopped. Jetson shutdown skipped "
                "(shutdown_enabled is false)."
            )
            return

        # Give the MKS bus node time to put the stop frames on the wire
        # before systemd starts tearing the process tree down.
        time.sleep(self._safe_stop_grace)

        self.get_logger().error(
            f"Shutting down Jetson: battery at {voltage:.3f} V"
        )
        try:
            subprocess.run(self._shutdown_command, timeout=30, check=True)
        except Exception as exc:
            self.get_logger().error(
                f"Shutdown command {self._shutdown_command} failed: {exc}. "
                "Power the robot down manually."
            )

    def _call_safe_stop(self):
        """Fire every configured Trigger stop service and wait briefly."""
        futures = {}
        for name, client in self._safe_stop_clients.items():
            if not client.service_is_ready():
                self.get_logger().warning(
                    f"Safe-stop service {name} unavailable; skipping"
                )
                continue
            futures[name] = client.call_async(Trigger.Request())

        if not futures:
            self.get_logger().error(
                "No safe-stop service was reachable before shutdown"
            )
            return

        deadline = time.monotonic() + self._safe_stop_grace
        while time.monotonic() < deadline:
            if all(future.done() for future in futures.values()):
                break
            time.sleep(0.05)

        for name, future in futures.items():
            if not future.done():
                self.get_logger().warning(
                    f"Safe-stop service {name} did not acknowledge in time"
                )
                continue
            try:
                result = future.result()
            except Exception as exc:
                self.get_logger().warning(f"Safe-stop {name} raised: {exc}")
                continue
            if getattr(result, "success", False):
                self.get_logger().info(f"Safe-stop {name} acknowledged")
            else:
                self.get_logger().warning(
                    f"Safe-stop {name} rejected: "
                    f"{getattr(result, 'message', '')}"
                )

    def _srv_get_voltage(self, request, response):
        del request
        with self._lock:
            reading = self._last_reading
            status = self._last_status
            error = self._last_error
        if reading is None:
            response.success = False
            response.message = f"No battery voltage yet. Last error: {error}"
            return response
        response.success = True
        response.message = (
            f"battery_voltage={reading['battery_voltage']:.3f} V, "
            f"display_voltage={reading['battery_voltage_filtered']:.3f} V, "
            f"soc={reading['percentage'] * 100.0:.1f} %, "
            f"raw={reading['raw']}, state={status.state}, "
            f"low<={self._monitor.low_voltage:.1f} V, "
            f"critical<={self._monitor.critical_voltage:.1f} V, "
            f"critical_for={status.critical_for_sec:.1f}s/"
            f"{self._monitor.critical_hold_sec:.1f}s, "
            f"shutdown_enabled={self._shutdown_enabled}"
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
        # Ctrl-C already shuts the context down via rclpy's signal handler;
        # calling shutdown again raises RCLError and exits 1.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

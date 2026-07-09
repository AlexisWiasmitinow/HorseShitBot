#!/usr/bin/env python3

import shlex
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from example_interfaces.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32

from horseshitbot.horseshitbot.modbus_adc import AdcRegisterMap, ModbusRtuClient, N43VD04


class BatteryModbusNode(Node):
    def __init__(self):
        super().__init__("battery_modbus_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("channel", 1)
        self.declare_parameter("read_period_sec", 1.0)
        self.declare_parameter("serial_timeout_sec", 0.25)

        self.declare_parameter("channel_base_register", 0)
        self.declare_parameter("address_register", 0x000E)
        self.declare_parameter("baudrate_register", 0x000F)
        self.declare_parameter("raw_scale", 100.0)

        # ADC_voltage = real_battery_voltage * voltage_divider_factor.
        # For the final HSB voltage divider, set this to 0.5.
        self.declare_parameter("voltage_divider_factor", 1.0)

        self.declare_parameter("low_voltage", 22.0)
        self.declare_parameter("critical_voltage", 21.0)
        self.declare_parameter("critical_confirmations", 3)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("critical_action_enabled", False)

        # This is intentionally configurable because we need to know the actual
        # HSB motor-disable interface before hardcoding anything.
        self.declare_parameter("disable_motors_command", "")

        self.declare_parameter("shutdown_enabled", False)
        self.declare_parameter("shutdown_command", "sudo /sbin/shutdown -h now")

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.slave_id = int(self.get_parameter("slave_id").value)
        self.channel = int(self.get_parameter("channel").value)
        self.read_period = float(self.get_parameter("read_period_sec").value)
        self.timeout = float(self.get_parameter("serial_timeout_sec").value)

        reg_map = AdcRegisterMap(
            channel_base_register=int(
                self.get_parameter("channel_base_register").value
            ),
            address_register=int(
                self.get_parameter("address_register").value
            ),
            baudrate_register=int(
                self.get_parameter("baudrate_register").value
            ),
            raw_scale=float(
                self.get_parameter("raw_scale").value
            ),
            voltage_divider_factor=float(
                self.get_parameter("voltage_divider_factor").value
            ),
        )

        self.low_voltage = float(self.get_parameter("low_voltage").value)
        self.critical_voltage = float(self.get_parameter("critical_voltage").value)
        self.critical_confirmations = int(
            self.get_parameter("critical_confirmations").value
        )

        self.critical_action_enabled = bool(
            self.get_parameter("critical_action_enabled").value
        )
        self.disable_motors_command = str(
            self.get_parameter("disable_motors_command").value
        )
        self.shutdown_enabled = bool(
            self.get_parameter("shutdown_enabled").value
        )
        self.shutdown_command = str(
            self.get_parameter("shutdown_command").value
        )

        self.client = ModbusRtuClient(
            port=self.port,
            baudrate=self.baudrate,
            slave_id=self.slave_id,
            timeout=self.timeout,
        )

        self.adc = N43VD04(self.client, reg_map)

        self.last_reading = None
        self.last_error = None

        self.critical_counter = 0
        self.critical_action_started = False
        self.last_low_warn_wall = 0.0

        self.lock = threading.Lock()

        self.voltage_pub = self.create_publisher(
            Float32,
            "battery_voltage",
            10,
        )
        self.low_pub = self.create_publisher(
            Bool,
            "battery_low",
            10,
        )
        self.critical_pub = self.create_publisher(
            Bool,
            "battery_critical",
            10,
        )
        self.battery_state_pub = self.create_publisher(
            BatteryState,
            "battery_state",
            10,
        )

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        self.srv = self.create_service(
            Trigger,
            "~/get_battery_voltage",
            self.handle_get_voltage,
        )

        self.timer = self.create_timer(
            self.read_period,
            self.timer_cb,
        )

        self.get_logger().info(
            f"Battery Modbus node started: "
            f"port={self.port}, "
            f"baud={self.baudrate}, "
            f"slave={self.slave_id}, "
            f"channel={self.channel}, "
            f"divider_factor={reg_map.voltage_divider_factor}"
        )

    def timer_cb(self):
        try:
            reading = self.adc.read_voltage(self.channel)
        except Exception as exc:
            self.last_error = repr(exc)
            self.get_logger().warn(f"ADC read failed: {self.last_error}")
            return

        with self.lock:
            self.last_reading = reading
            self.last_error = None

        voltage = float(reading["battery_voltage"])

        low = voltage <= self.low_voltage
        critical = voltage <= self.critical_voltage

        if critical:
            self.critical_counter += 1
        else:
            self.critical_counter = 0

        self.publish_reading(
            reading=reading,
            low=low,
            critical=critical,
        )

        if low and not critical:
            if (time.monotonic() - self.last_low_warn_wall) > 30.0:
                self.last_low_warn_wall = time.monotonic()
                self.get_logger().warn(
                    f"Battery LOW: "
                    f"{voltage:.2f} V <= {self.low_voltage:.2f} V"
                )

        if (
            self.critical_counter >= self.critical_confirmations
            and not self.critical_action_started
        ):
            self.critical_action_started = True

            self.get_logger().error(
                f"Battery CRITICAL: "
                f"{voltage:.2f} V <= {self.critical_voltage:.2f} V "
                f"for {self.critical_counter} consecutive readings"
            )

            self.start_critical_action()

    def publish_reading(
        self,
        reading: dict,
        low: bool,
        critical: bool,
    ):
        voltage = float(reading["battery_voltage"])

        msg = Float32()
        msg.data = voltage
        self.voltage_pub.publish(msg)

        low_msg = Bool()
        low_msg.data = low
        self.low_pub.publish(low_msg)

        critical_msg = Bool()
        critical_msg.data = critical
        self.critical_pub.publish(critical_msg)

        state = BatteryState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.voltage = voltage
        state.present = True

        if critical:
            state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif low:
            state.power_supply_health = (
                BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            )
        else:
            state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        self.battery_state_pub.publish(state)

    def handle_get_voltage(self, request, response):
        del request

        with self.lock:
            reading = self.last_reading
            error = self.last_error

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

    def start_critical_action(self):
        if not self.critical_action_enabled:
            self.get_logger().error(
                "Critical action is disabled. "
                "Set critical_action_enabled:=true only after bench testing."
            )
            return

        thread = threading.Thread(
            target=self.critical_action_worker,
            daemon=True,
        )
        thread.start()

    def critical_action_worker(self):
        self.get_logger().error("Publishing zero cmd_vel before motor shutdown")

        zero = Twist()

        for _ in range(10):
            self.cmd_vel_pub.publish(zero)
            time.sleep(0.1)

        if self.disable_motors_command:
            self.run_command(
                "disable_motors_command",
                self.disable_motors_command,
            )
        else:
            self.get_logger().error(
                "No disable_motors_command configured. "
                "Only zero cmd_vel was published."
            )

        if self.shutdown_enabled:
            self.run_command(
                "shutdown_command",
                self.shutdown_command,
            )
        else:
            self.get_logger().error(
                "shutdown_enabled is false; Jetson shutdown skipped."
            )

    def run_command(
        self,
        label: str,
        command: str,
    ):
        try:
            self.get_logger().error(f"Running {label}: {command}")
            subprocess.Popen(shlex.split(command))
        except Exception as exc:
            self.get_logger().error(
                f"Failed to run {label}: {repr(exc)}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = BatteryModbusNode()

    try:
        rclpy.spin(node)
    finally:
        node.client.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
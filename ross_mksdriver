import inspect

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


# ================== TO ADAPT ==================
SERIAL_PORT = "/dev/ttyUSB0"   # RS-485 adapter port on the Pi
BAUDRATE    = 38400            # must match UartBaud on the motor
SLAVE_ID    = 1                # must match UartAddr on the motor

DEFAULT_DIR = 0                # 0 = CCW, 1 = CW
DEFAULT_ACC = 10               # acceleration 0..255
# ========================================================

# Modbus registers for the MKS SERVO57D
REG_WORKMODE   = 0x0082   # work mode (5 = SR_vFOC)
REG_ENABLE     = 0x00F3   # enable motor in serial mode
REG_SPEED_BASE = 0x00F6   # base for speed command (two registers)


def _id_kw(method):
    """
    Return the correct keyword name for the device/slave ID, depending
    on the pymodbus version: device_id, slave, or unit.
    """
    try:
        params = inspect.signature(method).parameters
        for key in ("device_id", "slave", "unit"):
            if key in params:
                return key
    except Exception:
        pass
    return None


def _wr(client, address, value, dev_id):
    """Write a single holding register (version-agnostic helper)."""
    kw = _id_kw(client.write_register)
    kwargs = {kw: dev_id} if kw else {}
    return client.write_register(address=address, value=value, **kwargs)


def _wrs(client, address, values, dev_id):
    """Write multiple holding registers (version-agnostic helper)."""
    kw = _id_kw(client.write_registers)
    kwargs = {kw: dev_id} if kw else {}
    return client.write_registers(address=address, values=values, **kwargs)


class Servo57DDriver:
    """
    Low-level driver for the MKS SERVO57D using Modbus RTU.

    Provides:
      - connect(): open serial port, set mode, enable motor
      - set_speed(rpm): set speed in RPM (0..3000)
      - stop(): stop the motor (speed = 0)
    """

    def __init__(self,
                 port=SERIAL_PORT,
                 baudrate=BAUDRATE,
                 slave_id=SLAVE_ID,
                 default_dir=DEFAULT_DIR,
                 default_acc=DEFAULT_ACC,
                 timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.default_dir = default_dir
        self.default_acc = default_acc
        self.timeout = timeout

        self.client = None
        self.current_speed = 0

    def connect(self):
        """Open serial port and configure motor in SR_vFOC serial mode."""
        self.client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout,
        )

        if not self.client.connect():
            raise RuntimeError(f"Could not open serial port {self.port}")

        # Set work mode to SR_vFOC (5)
        res1 = _wr(self.client, REG_WORKMODE, 5, self.slave_id)
        if res1.isError():
            raise RuntimeError("Error setting work mode (REG_WORKMODE).")

        # Enable motor in serial mode
        res2 = _wr(self.client, REG_ENABLE, 1, self.slave_id)
        if res2.isError():
            raise RuntimeError("Error enabling motor (REG_ENABLE).")

    def set_speed(self, rpm, direction=None, acc=None):
        """
        Set motor speed in RPM (0..3000). 0 = stop.

        Modbus layout:
          - REG_SPEED_BASE (0x00F6): [dir (high byte), acc (low byte)]
          - REG_SPEED_BASE + 1     : speed (16 bits, RPM)
        """
        if rpm < 0:
            rpm = 0
        if rpm > 3000:
            rpm = 3000

        if direction is None:
            direction = self.default_dir
        if acc is None:
            acc = self.default_acc

        direction = 1 if direction else 0
        acc = max(0, min(255, int(acc)))

        reg_f6 = (direction << 8) | (acc & 0xFF)
        reg_f7 = int(rpm) & 0xFFFF

        res = _wrs(self.client, REG_SPEED_BASE, [reg_f6, reg_f7], self.slave_id)
        if res.isError():
            raise RuntimeError(f"Modbus error while sending speed command: {res}")

        self.current_speed = rpm

    def stop(self):
        """Stop the motor by sending speed = 0."""
        self.set_speed(0)

    def close(self):
        """Stop the motor and close the serial connection."""
        if self.client is not None:
            try:
                self.stop()
            except Exception:
                pass
            self.client.close()
            self.client = None


class Servo57DNode(Node):
    """
    ROS 2 node that exposes the SERVO57D driver via a speed command topic.

    Subscribes:
      - cmd_speed (std_msgs/Int32): desired speed in RPM (0..3000).
        * 0 => stop
        * >0 => run at that speed
    """

    def __init__(self):
        super().__init__("servo57d_node")

        self.get_logger().info(
            f"Connecting to SERVO57D on {SERIAL_PORT} (baud={BAUDRATE}, slave_id={SLAVE_ID})"
        )

        self.driver = Servo57DDriver()

        try:
            self.driver.connect()
        except Exception as e:
            self.get_logger().error(f"Failed to connect/enable motor: {e}")
            raise

        # Subscribe to speed command topic
        self.speed_sub = self.create_subscription(
            Int32,
            "cmd_speed",
            self.speed_callback,
            10,
        )

        self.get_logger().info(
            "Servo57D node started. Publish Int32 on 'cmd_speed' to set RPM (0..3000)."
        )

    def speed_callback(self, msg: Int32):
        rpm = msg.data
        try:
            self.driver.set_speed(rpm)
            self.get_logger().info(f"Set motor speed to {rpm} RPM")
        except Exception as e:
            self.get_logger().error(f"Failed to set speed: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down Servo57D node, stopping motor.")
        try:
            self.driver.close()
        except Exception as e:
            self.get_logger().warn(f"Error while closing driver: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Servo57DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


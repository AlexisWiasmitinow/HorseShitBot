"""
ODrive ASCII serial protocol driver.

Ported from test_scripts/odriveSerialTest.py — clean reusable class
for controlling an ODrive via its serial ASCII interface.
"""

from __future__ import annotations

import time

import serial


MOTOR_DEFAULTS = {
    "pole_pairs": 7,
    "current_lim": 30,
    "calibration_current": 20,
    "motor_type": 0,
    "pre_calibrated": 0,
}

ENCODER_DEFAULTS = {
    "mode": 0,
    "cpr": 2400,
    "use_index": 0,
    "calib_range": 0.05,
    "bandwidth": 500,
}

CONTROLLER_DEFAULTS = {
    "control_mode": 2,
    "vel_limit": 100,
    "vel_gain": 0.167,
    "vel_integrator_gain": 0.333,
    "pos_gain": 20,
    "input_mode": 2,
    "vel_ramp_rate": 20,
}

TRAP_TRAJ_DEFAULTS = {
    "vel_limit": 10,
    "accel_limit": 5,
    "decel_limit": 5,
}


class ODriveSerial:
    """ODrive ASCII protocol communication handler."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: serial.Serial | None = None

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return True
        except Exception:
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    @property
    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send_command(self, command: str) -> str | None:
        try:
            self.ser.reset_input_buffer()
            cmd = command.strip() + "\n"
            self.ser.write(cmd.encode("ascii"))

            response = ""
            time.sleep(0.15)
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
                if line:
                    response += line + "\n"
                time.sleep(0.05)
            return response.strip()
        except Exception:
            return None

    def read_property(self, property_path: str) -> str | None:
        response = self.send_command(f"r {property_path}")
        if response:
            lines = [ln.strip() for ln in response.split("\n") if ln.strip()]
            value = lines[-1] if lines else response
            value = "".join(c for c in value if c.isprintable())
            return value
        return response

    def read_property_fast(self, property_path: str) -> str | None:
        try:
            self.ser.reset_input_buffer()
            self.ser.write(f"r {property_path}\n".encode("ascii"))
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            return line
        except Exception:
            return None

    def write_property(self, property_path: str, value) -> str | None:
        return self.send_command(f"w {property_path} {value}")

    def write_property_fast(self, property_path: str, value) -> None:
        """Fire-and-forget write — no response wait. Use for real-time control."""
        try:
            self.ser.write(f"w {property_path} {value}\n".encode("ascii"))
        except Exception:
            pass

    def apply_defaults(self):
        for axis in range(2):
            for key, val in MOTOR_DEFAULTS.items():
                self.write_property(f"axis{axis}.motor.config.{key}", str(val))
            for key, val in ENCODER_DEFAULTS.items():
                self.write_property(f"axis{axis}.encoder.config.{key}", str(val))
            for key, val in CONTROLLER_DEFAULTS.items():
                self.write_property(f"axis{axis}.controller.config.{key}", str(val))
            for key, val in TRAP_TRAJ_DEFAULTS.items():
                self.write_property(f"axis{axis}.trap_traj.config.{key}", str(val))
        time.sleep(0.1)

    def clear_errors(self, axis: int = 0):
        for sub in ("error", "motor.error", "encoder.error", "controller.error"):
            self.write_property(f"axis{axis}.{sub}", "0")
        time.sleep(0.1)

    def set_idle(self, axis: int = 0):
        self.write_property(f"axis{axis}.requested_state", "1")
        time.sleep(0.3)

    def set_closed_loop(self, axis: int = 0) -> bool:
        self.ser.reset_input_buffer()
        time.sleep(0.05)
        self.clear_errors(axis)

        calibrated = self.read_property(f"axis{axis}.motor.is_calibrated")
        encoder_ready = self.read_property(f"axis{axis}.encoder.is_ready")

        cal_val = "".join(c for c in str(calibrated) if c.isdigit())
        enc_val = "".join(c for c in str(encoder_ready) if c.isdigit())

        if cal_val != "1" or enc_val != "1":
            return False

        self.write_property(f"axis{axis}.requested_state", "8")
        time.sleep(0.5)

        state = self.read_property(f"axis{axis}.current_state")
        err = self.read_property(f"axis{axis}.error")
        state_val = "".join(c for c in str(state) if c.isdigit())
        err_val = "".join(c for c in str(err) if c.isdigit())

        return state_val == "8" and err_val == "0"

    def _parse_digit(self, raw) -> str:
        return "".join(c for c in str(raw) if c.isdigit())

    def is_ready_for_closed_loop(self, axis: int = 0) -> bool:
        """Check if motor is calibrated and encoder is ready (no recalibration needed)."""
        cal = self._parse_digit(self.read_property(f"axis{axis}.motor.is_calibrated"))
        enc = self._parse_digit(self.read_property(f"axis{axis}.encoder.is_ready"))
        err = self._parse_digit(self.read_property(f"axis{axis}.error"))
        return cal == "1" and enc == "1" and err == "0"

    def start_motor(self, axis: int = 0, force_calibration: bool = False) -> bool:
        """Start motor into closed loop. Skips calibration if already calibrated."""
        self.clear_errors(axis)

        # Already calibrated and error-free? Just enter closed loop.
        if not force_calibration and self.is_ready_for_closed_loop(axis):
            self.write_property(f"axis{axis}.requested_state", "8")
            time.sleep(0.5)
            state = self.read_property(f"axis{axis}.current_state")
            return state == "8"

        # Full calibration needed
        self.write_property(f"axis{axis}.motor.config.pre_calibrated", "0")
        self.write_property(f"axis{axis}.encoder.config.pre_calibrated", "0")
        time.sleep(0.2)

        self.clear_errors(axis)

        # Motor calibration
        self.write_property(f"axis{axis}.requested_state", "4")
        time.sleep(1)
        for _ in range(15):
            time.sleep(1)
            state = self.read_property(f"axis{axis}.current_state")
            if state == "1":
                break
        time.sleep(0.5)

        motor_error = self.read_property(f"axis{axis}.motor.error")
        calibrated = self.read_property(f"axis{axis}.motor.is_calibrated")
        if self._parse_digit(motor_error) != "0" or self._parse_digit(calibrated) != "1":
            return False

        # Check hall sensors if in hall mode
        enc_mode = self.read_property(f"axis{axis}.encoder.config.mode")
        if enc_mode == "1":
            for _ in range(20):
                self.clear_errors(axis)
                raw = self.read_property(f"axis{axis}.encoder.hall_state")
                try:
                    hall_state = int(self._parse_digit(raw))
                except ValueError:
                    hall_state = -1
                if hall_state in (1, 2, 3, 4, 5, 6):
                    break
                time.sleep(0.5)
            else:
                return False

        # Encoder offset calibration
        self.clear_errors(axis)
        self.write_property(f"axis{axis}.requested_state", "7")
        time.sleep(1)
        for _ in range(20):
            time.sleep(1)
            state = self.read_property(f"axis{axis}.current_state")
            if state == "1":
                break
        time.sleep(0.5)

        encoder_error = self.read_property(f"axis{axis}.encoder.error")
        encoder_ready = self.read_property(f"axis{axis}.encoder.is_ready")
        if self._parse_digit(encoder_error) != "0" or self._parse_digit(encoder_ready) != "1":
            return False

        # Enter closed loop
        self.write_property(f"axis{axis}.requested_state", "8")
        time.sleep(1)
        state = self.read_property(f"axis{axis}.current_state")
        return state == "8"

    def set_velocity(self, axis: int, velocity: float):
        """Set velocity using fast fire-and-forget write."""
        self.write_property_fast(f"axis{axis}.controller.input_vel", f"{velocity:.4f}")

    def set_velocity_mode(self, axis: int, passthrough: bool = True):
        """Set axis to velocity control. passthrough=True disables ODrive's own ramp."""
        self.write_property(f"axis{axis}.controller.config.control_mode", "2")
        mode = "1" if passthrough else "2"
        self.write_property(f"axis{axis}.controller.config.input_mode", mode)
        time.sleep(0.05)

    def get_velocity(self, axis: int) -> float:
        raw = self.read_property_fast(f"axis{axis}.encoder.vel_estimate")
        try:
            return float(raw)
        except (ValueError, TypeError):
            return 0.0

    def get_vbus_voltage(self) -> float:
        raw = self.read_property("vbus_voltage")
        try:
            return float(raw)
        except (ValueError, TypeError):
            return 0.0

    def get_temperature(self, axis: int) -> float | None:
        """Read FET thermistor temperature (°C) for the given axis.

        Returns None if the ODrive firmware doesn't expose this property.
        """
        raw = self.read_property(f"axis{axis}.fet_thermistor.temperature")
        try:
            val = float(raw)
            if -40.0 < val < 200.0:
                return val
        except (ValueError, TypeError):
            pass
        return None

    def get_motor_temperature(self, axis: int) -> float | None:
        """Read motor thermistor temperature (°C), if a thermistor is wired."""
        raw = self.read_property(f"axis{axis}.motor_thermistor.temperature")
        try:
            val = float(raw)
            if -40.0 < val < 200.0:
                return val
        except (ValueError, TypeError):
            pass
        return None

    def save_config(self):
        self.send_command("ss")
        time.sleep(1)

    def reboot(self):
        self.send_command("sr")

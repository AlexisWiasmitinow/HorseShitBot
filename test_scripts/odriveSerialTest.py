#!/bin/python3
"""
ODrive Serial ASCII Protocol Test Script
This script connects to ODrive via serial port (COM) using ASCII protocol.
For better performance, use native USB with the main odriveTest.py script.
"""

import serial
import time
import sys
import argparse
import re


# =====================================================
# DEFAULT CONFIGURATION
# =====================================================
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 2.0

# =====================================================
# MOTOR / ENCODER / CONTROLLER DEFAULTS
# =====================================================
MOTOR_DEFAULTS = {
    "pole_pairs": 7,
    "current_lim": 20,
    "calibration_current": 10,
    "motor_type": 0,
    "pre_calibrated": 1,
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


_current_axis = 0


class ODriveSerial:
    """ODrive ASCII protocol communication handler"""

    def __init__(self, port, baudrate=115200, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        """Connect to ODrive via serial port"""
        try:
            print(f"Connecting to ODrive on {self.port} @ {self.baudrate} baud...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            time.sleep(0.5)  # Wait for connection to stabilize

            # Clear any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            print("✓ Connected successfully!")
            return True

        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from ODrive"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected")

    def send_command(self, command):
        """Send a command and read response"""
        try:
            # Clear buffers
            self.ser.reset_input_buffer()

            # Send command
            cmd = command.strip() + "\n"
            self.ser.write(cmd.encode("ascii"))

            # Read response - wait for data to arrive
            response = ""
            time.sleep(0.15)

            # Read all available data
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
                if line:
                    response += line + "\n"
                time.sleep(0.05)  # Small delay between reads

            return response.strip()

        except Exception as e:
            print(f"✗ Command error: {e}")
            return None

    def read_property_fast(self, property_path):
        """Read a property with minimal latency"""
        try:
            self.ser.reset_input_buffer()
            self.ser.write(f"r {property_path}\n".encode("ascii"))
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            return line
        except Exception as e:
            return None

    def read_property(self, property_path):
        """Read a property value"""
        response = self.send_command(f"r {property_path}")
        if response:
            # Parse the response - take the last non-empty line
            lines = [l.strip() for l in response.split("\n") if l.strip()]
            value = lines[-1] if lines else response
            # Strip any non-printable characters or serial artifacts
            value = "".join(c for c in value if c.isprintable())
            return value
        return response

    def write_property(self, property_path, value):
        """Write a property value"""
        command = f"w {property_path} {value}"
        response = self.send_command(command)
        return response


# =====================================================
# ODrive Control Functions
# =====================================================


def apply_defaults(odrv):
    """Apply default motor/encoder/controller settings to both axes on startup"""
    print("Applying default configuration to both axes...")
    for axis in range(2):
        for key, val in MOTOR_DEFAULTS.items():
            odrv.write_property(f"axis{axis}.motor.config.{key}", str(val))
        for key, val in ENCODER_DEFAULTS.items():
            odrv.write_property(f"axis{axis}.encoder.config.{key}", str(val))
        for key, val in CONTROLLER_DEFAULTS.items():
            odrv.write_property(f"axis{axis}.controller.config.{key}", str(val))
        for key, val in TRAP_TRAJ_DEFAULTS.items():
            odrv.write_property(f"axis{axis}.trap_traj.config.{key}", str(val))
        print(f"  ✓ Axis {axis} configured")
    time.sleep(0.1)
    print("✓ Defaults applied to both axes")


def get_info(odrv):
    """Get ODrive information"""
    print(f"\n{'='*50}")
    print("ODRIVE INFORMATION")
    print(f"{'='*50}")

    # Read basic info
    vbus = odrv.read_property("vbus_voltage")
    print(f"VBus Voltage: {vbus}V")


def get_axis_info(odrv, axis_num=0):
    """Get axis information"""
    print(f"\n{'='*50}")
    print(f"AXIS {axis_num} INFORMATION")
    print(f"{'='*50}")

    # Read axis state
    state = odrv.read_property(f"axis{axis_num}.current_state")
    error = odrv.read_property(f"axis{axis_num}.error")

    state_names = {
        "0": "UNDEFINED",
        "1": "IDLE",
        "2": "STARTUP_SEQUENCE",
        "3": "FULL_CALIBRATION_SEQUENCE",
        "4": "MOTOR_CALIBRATION",
        "6": "ENCODER_OFFSET_CALIBRATION",
        "8": "CLOSED_LOOP_CONTROL",
    }

    print(f"State: {state_names.get(state, 'UNKNOWN')} ({state})")
    print(f"Error: {error}")

    # Read position and velocity
    pos = odrv.read_property(f"axis{axis_num}.encoder.pos_estimate")
    vel = odrv.read_property(f"axis{axis_num}.encoder.vel_estimate")

    print(f"\nPosition: {pos} turns")
    print(f"Velocity: {vel} turns/sec")

    # Motor info
    calibrated = odrv.read_property(f"axis{axis_num}.motor.is_calibrated")
    motor_error = odrv.read_property(f"axis{axis_num}.motor.error")

    print(f"\nMotor:")
    print(f"  Is Calibrated: {calibrated}")
    print(f"  Error: {motor_error}")

    # Encoder info
    encoder_ready = odrv.read_property(f"axis{axis_num}.encoder.is_ready")
    encoder_error = odrv.read_property(f"axis{axis_num}.encoder.error")

    print(f"\nEncoder:")
    print(f"  Is Ready: {encoder_ready}")
    print(f"  Error: {encoder_error}")


def set_idle(odrv, axis_num=0):
    """Set axis to idle state"""
    print(f"Setting Axis {axis_num} to IDLE...")
    odrv.write_property(f"axis{axis_num}.requested_state", "1")
    time.sleep(0.5)
    print("✓ Motor disabled (idle)")


def calibrate(odrv, axis_num=0):
    """Calibration menu"""
    print(f"\n{'='*50}")
    print(f"CALIBRATION MENU - AXIS {axis_num}")
    print(f"{'='*50}")
    print("  1. Full calibration (motor + encoder)")
    print("  2. Motor calibration only")
    print("  3. Encoder offset calibration only")
    print("  4. Hall sensor setup (motor cal + skip encoder cal)")
    print("  [Enter] Cancel")

    choice = input("\nSelect calibration type: ").strip()

    if choice == "1":
        return run_calibration(odrv, axis_num, state=3, name="Full Calibration")
    elif choice == "2":
        return run_calibration(odrv, axis_num, state=4, name="Motor Calibration")
    elif choice == "3":
        return run_calibration(odrv, axis_num, state=7, name="Encoder Offset Calibration")
    elif choice == "4":
        return hall_sensor_setup(odrv, axis_num)
    else:
        print("Cancelled.")
        return False


def run_calibration(odrv, axis_num, state, name):
    """Run a specific calibration step"""
    print(f"\n{'='*50}")
    print(f"{name.upper()} - AXIS {axis_num}")
    print(f"{'='*50}")
    print("⚠ Warning: Motor will spin during calibration!")

    # Print current settings
    pole_pairs = odrv.read_property(f"axis{axis_num}.motor.config.pole_pairs")
    enc_mode = odrv.read_property(f"axis{axis_num}.encoder.config.mode")
    cpr = odrv.read_property(f"axis{axis_num}.encoder.config.cpr")
    calib_range = odrv.read_property(f"axis{axis_num}.encoder.config.calib_range")
    use_index = odrv.read_property(f"axis{axis_num}.encoder.config.use_index")
    pre_cal_m = odrv.read_property(f"axis{axis_num}.motor.config.pre_calibrated")
    pre_cal_e = odrv.read_property(f"axis{axis_num}.encoder.config.pre_calibrated")
    print(f"\n  Pole pairs:     {pole_pairs}")
    print(f"  Encoder mode:   {enc_mode}")
    print(f"  CPR:            {cpr}")
    print(f"  Calib range:    {calib_range}")
    print(f"  Use index:      {use_index}")
    print(f"  Motor pre-cal:  {pre_cal_m}")
    print(f"  Enc pre-cal:    {pre_cal_e}")

    # Clear errors first
    print("\nClearing errors...")
    odrv.write_property(f"axis{axis_num}.error", "0")
    odrv.write_property(f"axis{axis_num}.motor.error", "0")
    odrv.write_property(f"axis{axis_num}.encoder.error", "0")
    odrv.write_property(f"axis{axis_num}.controller.error", "0")
    time.sleep(0.2)
    print("✓ Errors cleared")

    # Start calibration
    print(f"\nStarting {name} (requested_state = {state})...")
    odrv.write_property(f"axis{axis_num}.requested_state", str(state))

    # Wait for calibration to begin
    time.sleep(1)

    # Monitor progress
    print("⟳ Calibration in progress...")
    for i in range(30):
        time.sleep(1)
        current = odrv.read_property(f"axis{axis_num}.current_state")

        state_names = {
            "0": "UNDEFINED",
            "1": "IDLE",
            "3": "FULL_CALIBRATION",
            "4": "MOTOR_CALIBRATION",
            "6": "ENCODER_OFFSET_CAL",
            "7": "ENCODER_OFFSET_CAL",
            "8": "CLOSED_LOOP",
        }
        state_name = state_names.get(current, f"UNKNOWN({current})")
        print(f"  State: {state_name} ({current})")

        if current == "1":  # IDLE - done
            break

    # Check results
    time.sleep(0.5)
    error = odrv.read_property(f"axis{axis_num}.error")
    motor_error = odrv.read_property(f"axis{axis_num}.motor.error")
    encoder_error = odrv.read_property(f"axis{axis_num}.encoder.error")
    calibrated = odrv.read_property(f"axis{axis_num}.motor.is_calibrated")
    encoder_ready = odrv.read_property(f"axis{axis_num}.encoder.is_ready")

    phase_r = odrv.read_property(f"axis{axis_num}.motor.config.phase_resistance")
    phase_l = odrv.read_property(f"axis{axis_num}.motor.config.phase_inductance")

    print(f"\nResults:")
    print(f"  Axis Error: {error}")
    print(f"  Motor Error: {motor_error}")
    print(f"  Encoder Error: {encoder_error}")
    print(f"  Motor Calibrated: {calibrated}")
    print(f"  Encoder Ready: {encoder_ready}")
    print(f"  Phase Resistance: {phase_r} ohm")
    print(f"  Phase Inductance: {phase_l} H")

    error_val = "".join(c for c in str(error) if c.isdigit())
    motor_err_val = "".join(c for c in str(motor_error) if c.isdigit())
    encoder_err_val = "".join(c for c in str(encoder_error) if c.isdigit())

    if error_val == "0" and motor_err_val == "0" and encoder_err_val == "0":
        print(f"\n✓ {name} successful!")
        return True
    else:
        print(f"\n✗ {name} failed")
        return False


def hall_sensor_setup(odrv, axis_num):
    """
    Setup for Hall sensor motors:
    1. Configure encoder for Hall mode
    2. Run motor calibration only
    3. Mark encoder as pre-calibrated (Hall sensors don't need offset cal)
    """
    print(f"\n{'='*50}")
    print(f"HALL SENSOR SETUP - AXIS {axis_num}")
    print(f"{'='*50}")

    # Step 1: Configure encoder for Hall mode
    print("\nStep 1: Configuring encoder for Hall sensors...")
    odrv.write_property(f"axis{axis_num}.encoder.config.mode", "1")

    pole_pairs = input("Enter motor pole pairs (e.g., 7, 14, 20): ").strip()
    if not pole_pairs:
        print("✗ Pole pairs required!")
        return False

    cpr = 6 * int(pole_pairs)
    odrv.write_property(f"axis{axis_num}.encoder.config.cpr", str(cpr))
    print(f"  ✓ Encoder mode: Hall (1)")
    print(f"  ✓ CPR: {cpr} (6 x {pole_pairs} pole pairs)")

    # Also set motor pole pairs
    odrv.write_property(f"axis{axis_num}.motor.config.pole_pairs", pole_pairs)
    print(f"  ✓ Motor pole pairs: {pole_pairs}")

    # Step 2: Clear errors
    print("\nStep 2: Clearing errors...")
    odrv.write_property(f"axis{axis_num}.error", "0")
    odrv.write_property(f"axis{axis_num}.motor.error", "0")
    odrv.write_property(f"axis{axis_num}.encoder.error", "0")
    odrv.write_property(f"axis{axis_num}.controller.error", "0")
    time.sleep(0.2)
    print("  ✓ Errors cleared")

    # Step 3: Run motor calibration only (state 4)
    print("\nStep 3: Running motor calibration...")
    print("⚠ Motor will beep/spin briefly!")

    odrv.write_property(f"axis{axis_num}.requested_state", "4")
    time.sleep(1)

    print("⟳ Motor calibration in progress...")
    for i in range(15):
        time.sleep(1)
        state = odrv.read_property(f"axis{axis_num}.current_state")
        print(f"  State: {state}")
        if state == "1":
            break

    time.sleep(0.5)
    motor_error = odrv.read_property(f"axis{axis_num}.motor.error")
    calibrated = odrv.read_property(f"axis{axis_num}.motor.is_calibrated")

    me_val = "".join(c for c in str(motor_error) if c.isdigit())
    cal_val = "".join(c for c in str(calibrated) if c.isdigit())
    if me_val != "0" or cal_val != "1":
        print(f"\n✗ Motor calibration failed")
        print(f"  Motor Error: {motor_error}")
        print(f"  Motor Calibrated: {calibrated}")
        return False

    print("  ✓ Motor calibration complete!")

    # Step 4: Mark motor and encoder as pre-calibrated
    print("\nStep 4: Marking as pre-calibrated (skip encoder offset cal)...")
    odrv.write_property(f"axis{axis_num}.motor.config.pre_calibrated", "1")
    odrv.write_property(f"axis{axis_num}.encoder.config.pre_calibrated", "1")
    print("  ✓ Motor pre-calibrated: True")
    print("  ✓ Encoder pre-calibrated: True")

    # Step 5: Save configuration
    print("\nStep 5: Saving configuration...")
    odrv.send_command("ss")
    time.sleep(1)
    print("  ✓ Configuration saved!")

    # Step 6: Try closed loop
    print(f"\n{'='*50}")
    print("Setup complete! Attempting closed loop control...")

    # Clear any leftover errors
    odrv.write_property(f"axis{axis_num}.error", "0")
    odrv.write_property(f"axis{axis_num}.motor.error", "0")
    odrv.write_property(f"axis{axis_num}.encoder.error", "0")
    time.sleep(0.2)

    odrv.write_property(f"axis{axis_num}.requested_state", "8")
    time.sleep(1)

    state = odrv.read_property(f"axis{axis_num}.current_state")
    if state == "8":
        print("✓ Closed loop control enabled! Motor is ready.")
        print("  Use option 10 (manual velocity) to test.")

        # Set back to idle for safety
        odrv.write_property(f"axis{axis_num}.requested_state", "1")
        time.sleep(0.2)
        print("  (Set back to idle for safety)")
    else:
        error = odrv.read_property(f"axis{axis_num}.error")
        print(f"⚠ Could not enter closed loop (state: {state}, error: {error})")
        print("  You may need to reboot the ODrive and try again.")

    return True


def set_closed_loop(odrv, axis_num=0):
    """Enable closed loop control"""
    print(f"Enabling closed loop control on Axis {axis_num}...")

    # Flush serial buffer and clear leftover errors
    odrv.ser.reset_input_buffer()
    time.sleep(0.05)
    odrv.write_property(f"axis{axis_num}.error", "0")
    odrv.write_property(f"axis{axis_num}.motor.error", "0")
    odrv.write_property(f"axis{axis_num}.encoder.error", "0")
    odrv.write_property(f"axis{axis_num}.controller.error", "0")
    time.sleep(0.1)

    # Check calibration
    calibrated = odrv.read_property(f"axis{axis_num}.motor.is_calibrated")
    encoder_ready = odrv.read_property(f"axis{axis_num}.encoder.is_ready")

    cal_val = "".join(c for c in str(calibrated) if c.isdigit())
    enc_val = "".join(c for c in str(encoder_ready) if c.isdigit())

    if cal_val != "1":
        print("✗ Motor not calibrated! Run calibration first.")
        return False

    if enc_val != "1":
        print("✗ Encoder not ready! Run calibration first.")
        return False

    # Enable closed loop
    odrv.write_property(f"axis{axis_num}.requested_state", "8")
    time.sleep(0.5)

    state = odrv.read_property(f"axis{axis_num}.current_state")
    state_val = "".join(c for c in str(state) if c.isdigit())
    err = odrv.read_property(f"axis{axis_num}.error")
    err_val = "".join(c for c in str(err) if c.isdigit())

    if state_val == "8" and err_val == "0":
        print("✓ Closed loop control enabled")
        return True
    else:
        motor_err = odrv.read_property(f"axis{axis_num}.motor.error")
        enc_err = odrv.read_property(f"axis{axis_num}.encoder.error")
        ctrl_err = odrv.read_property(f"axis{axis_num}.controller.error")
        print(f"✗ Failed to enable closed loop:")
        print(f"  State: {state}  Axis error: {err}")
        print(f"  Motor error: {motor_err}")
        print(f"  Encoder error: {enc_err}")
        print(f"  Controller error: {ctrl_err}")
        return False


def start_motor(odrv, axis_num=0):
    """
    Full start-up routine for Hall sensor motor:
    1. Reset pre_calibrated so calibrations actually run
    2. Clear all errors
    3. Run motor calibration
    4. Wait for valid Hall state
    5. Run encoder offset calibration (needed for Hall sensors!)
    6. Enter closed loop
    """
    print(f"\n{'='*50}")
    print(f"STARTING MOTOR - AXIS {axis_num}")
    print(f"{'='*50}")

    # Step 1: Reset pre_calibrated flags so calibrations actually run
    print("\nStep 1: Resetting pre-calibrated flags...")
    odrv.write_property(f"axis{axis_num}.motor.config.pre_calibrated", "0")
    odrv.write_property(f"axis{axis_num}.encoder.config.pre_calibrated", "0")
    time.sleep(0.2)
    print("  ✓ Flags reset")

    # Step 2: Clear all errors
    print("\nStep 2: Clearing all errors...")
    odrv.write_property(f"axis{axis_num}.error", "0")
    odrv.write_property(f"axis{axis_num}.motor.error", "0")
    odrv.write_property(f"axis{axis_num}.encoder.error", "0")
    odrv.write_property(f"axis{axis_num}.controller.error", "0")
    time.sleep(0.2)
    print("  ✓ Errors cleared")

    # Step 3: Run motor calibration (beep + resistance/inductance measurement)
    print("\nStep 3: Running motor calibration...")
    print("  ⚠ Motor will beep/spin briefly!")
    odrv.write_property(f"axis{axis_num}.requested_state", "4")
    time.sleep(1)

    print("  ⟳ Calibrating...")
    for i in range(15):
        time.sleep(1)
        state = odrv.read_property(f"axis{axis_num}.current_state")
        if state == "1":
            break
        print(f"    State: {state}")

    time.sleep(0.5)
    motor_error = odrv.read_property(f"axis{axis_num}.motor.error")
    calibrated = odrv.read_property(f"axis{axis_num}.motor.is_calibrated")

    me_val = "".join(c for c in str(motor_error) if c.isdigit())
    cal_val = "".join(c for c in str(calibrated) if c.isdigit())
    if me_val != "0" or cal_val != "1":
        print(f"  ✗ Motor calibration failed (error: {motor_error}, calibrated: {calibrated})")
        return False

    phase_r = odrv.read_property(f"axis{axis_num}.motor.config.phase_resistance")
    phase_l = odrv.read_property(f"axis{axis_num}.motor.config.phase_inductance")
    print(f"  ✓ Motor calibrated (R={phase_r}Ω, L={phase_l}H)")

    # Step 4: Check encoder type and prepare
    enc_mode = odrv.read_property(f"axis{axis_num}.encoder.config.mode")
    is_hall = enc_mode == "1"

    if is_hall:
        print("\nStep 4: Checking Hall sensors...")
        max_attempts = 20
        for attempt in range(max_attempts):
            odrv.write_property(f"axis{axis_num}.error", "0")
            odrv.write_property(f"axis{axis_num}.encoder.error", "0")
            time.sleep(0.1)

            raw = odrv.read_property(f"axis{axis_num}.encoder.hall_state")
            try:
                hall_state = int("".join(c for c in str(raw) if c.isdigit()))
            except ValueError:
                hall_state = -1

            if hall_state in [1, 2, 3, 4, 5, 6]:
                print(f"  ✓ Hall state: {hall_state} (valid)")
                break
            else:
                if attempt == 0:
                    print(f"  ⚠ Hall state: {hall_state} (invalid)")
                    print("  Nudge the motor shaft slightly to move off the Hall boundary...")
                time.sleep(0.5)
        else:
            print(f"  ✗ Hall state stuck at invalid value")
            print("    Manually rotate the shaft slightly and try again")
            return False
    else:
        print(f"\nStep 4: Incremental encoder (mode {enc_mode}), skipping Hall check")

    # Step 5: Run encoder offset calibration
    print("\nStep 5: Running encoder offset calibration...")
    print("  ⚠ Motor will spin slowly!")

    odrv.write_property(f"axis{axis_num}.error", "0")
    odrv.write_property(f"axis{axis_num}.encoder.error", "0")
    time.sleep(0.2)

    odrv.write_property(f"axis{axis_num}.requested_state", "7")
    time.sleep(1)

    print("  ⟳ Calibrating encoder offset...")
    for i in range(20):
        time.sleep(1)
        state = odrv.read_property(f"axis{axis_num}.current_state")
        if state == "1":
            break
        print(f"    State: {state}")

    time.sleep(0.5)
    encoder_error = odrv.read_property(f"axis{axis_num}.encoder.error")
    encoder_ready = odrv.read_property(f"axis{axis_num}.encoder.is_ready")

    ee_val = "".join(c for c in str(encoder_error) if c.isdigit())
    er_val = "".join(c for c in str(encoder_ready) if c.isdigit())
    if ee_val != "0" or er_val != "1":
        print(f"  ✗ Encoder offset calibration failed")
        print(f"    Encoder Error: {encoder_error}")
        print(f"    Encoder Ready: {encoder_ready}")
        axis_error = odrv.read_property(f"axis{axis_num}.error")
        print(f"    Axis Error: {axis_error}")
        return False

    offset = odrv.read_property(f"axis{axis_num}.encoder.config.offset")
    print(f"  ✓ Encoder offset calibrated (offset: {offset})")

    # Step 6: Enter closed loop
    print("\nStep 6: Entering closed loop control...")
    odrv.write_property(f"axis{axis_num}.requested_state", "8")
    time.sleep(1)

    state = odrv.read_property(f"axis{axis_num}.current_state")
    if state == "8":
        print("\n✓ Motor started successfully! Closed loop control active.")
        print("  Use option 11 (manual velocity) or 12 (manual position) to control.")
        return True
    else:
        error = odrv.read_property(f"axis{axis_num}.error")
        encoder_error = odrv.read_property(f"axis{axis_num}.encoder.error")
        motor_error = odrv.read_property(f"axis{axis_num}.motor.error")
        print(f"\n✗ Failed to enter closed loop")
        print(f"  State: {state}")
        print(f"  Axis Error: {error}")
        print(f"  Motor Error: {motor_error}")
        print(f"  Encoder Error: {encoder_error}")
        return False


def set_velocity(odrv, axis_num, velocity):
    """Set velocity control mode and command velocity"""
    odrv.write_property(f"axis{axis_num}.controller.config.control_mode", "2")
    odrv.write_property(f"axis{axis_num}.controller.config.input_mode", "2")
    time.sleep(0.05)
    odrv.write_property(f"axis{axis_num}.controller.input_vel", str(velocity))
    print(f"✓ Velocity set to {velocity} turns/sec")


def set_position(odrv, axis_num, position):
    """Set position control mode and command position"""
    odrv.write_property(f"axis{axis_num}.controller.config.control_mode", "3")
    odrv.write_property(f"axis{axis_num}.controller.config.input_mode", "5")
    time.sleep(0.05)
    odrv.write_property(f"axis{axis_num}.controller.input_pos", str(position))
    print(f"✓ Position set to {position:.3f} turns")


def read_encoder_inputs(odrv, axis_num=0):
    """Read and display raw encoder input pin states continuously"""
    print(f"\n{'='*50}")
    print(f"ENCODER INPUT TEST - AXIS {axis_num}")
    print(f"{'='*50}")

    shadow = odrv.read_property(f"axis{axis_num}.encoder.shadow_count")
    print(f"  Shadow count: {shadow}")
    print(f"\nRotate shaft slowly. Ctrl+C to stop.")
    print("State |  A  B  Z  | Shadow Count | Position")
    print("------|-----------|--------------|----------")

    last_state = None
    try:
        while True:
            raw = odrv.read_property(f"axis{axis_num}.encoder.hall_state")
            shadow = odrv.read_property_fast(f"axis{axis_num}.encoder.shadow_count")
            pos = odrv.read_property_fast(f"axis{axis_num}.encoder.pos_estimate")

            try:
                state = int("".join(c for c in str(raw) if c.isdigit()))
            except ValueError:
                state = -1

            a = (state >> 0) & 1
            b = (state >> 1) & 1
            z = (state >> 2) & 1

            if state != last_state:
                print(f"  {state:^3}  |  {a}  {b}  {z}  | {str(shadow):>12} | {str(pos):>8}")
                last_state = state

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nStopped.")


def test_encoder_live(odrv, axis_num=0):
    """Continuously read encoder position as fast as serial allows"""
    print(f"\n{'='*50}")
    print(f"LIVE ENCODER TEST - AXIS {axis_num}")
    print(f"{'='*50}")

    mode = odrv.read_property(f"axis{axis_num}.encoder.config.mode")
    cpr = odrv.read_property(f"axis{axis_num}.encoder.config.cpr")
    mode_names = {"0": "INCREMENTAL", "1": "HALL", "256": "SPI_ABS_AMS"}
    print(f"  Mode: {mode_names.get(mode, mode)}, CPR: {cpr}")
    print(f"\nRotate the shaft and watch the values change.")
    print(f"Press Ctrl+C to stop.\n")

    samples = 0
    t_start = time.perf_counter()

    try:
        while True:
            pos = odrv.read_property_fast(f"axis{axis_num}.encoder.pos_estimate")
            samples += 1
            elapsed = time.perf_counter() - t_start
            rate = samples / elapsed if elapsed > 0 else 0
            print(f"\r  pos: {str(pos):>14}  |  {rate:.0f} samples/sec", end="", flush=True)
    except KeyboardInterrupt:
        elapsed = time.perf_counter() - t_start
        print(f"\n\nStopped. {samples} samples in {elapsed:.1f}s ({samples/elapsed:.0f} samples/sec)")


def measure_cpr(odrv, axis_num=0):
    """Measure actual encoder counts per revolution by hand-turning the shaft"""
    print(f"\n{'='*50}")
    print(f"MEASURE COUNTS PER REVOLUTION - AXIS {axis_num}")
    print(f"{'='*50}")

    odrv.write_property(f"axis{axis_num}.encoder.error", "0")
    time.sleep(0.1)

    print("\n1. Mark the shaft position (use tape or a marker)")
    print("2. Press Enter to start counting")
    print("3. Slowly rotate EXACTLY one full turn")
    print("4. Press Enter when back to the mark")
    input("\nPress Enter to start...")

    start = odrv.read_property_fast(f"axis{axis_num}.encoder.shadow_count")
    try:
        start_count = int(start)
    except (ValueError, TypeError):
        print(f"Could not read shadow_count: {start}")
        return

    print(f"Start count: {start_count}")
    print("Rotate exactly one full turn now...")
    print("(Live count shown below, press Enter when done)\n")

    import threading

    stop_flag = threading.Event()

    def show_live():
        while not stop_flag.is_set():
            raw = odrv.read_property_fast(f"axis{axis_num}.encoder.shadow_count")
            try:
                current = int(raw)
                delta = current - start_count
                print(f"\r  Counts: {delta:>8}", end="", flush=True)
            except (ValueError, TypeError):
                pass
            time.sleep(0.05)

    t = threading.Thread(target=show_live, daemon=True)
    t.start()
    input()
    stop_flag.set()
    t.join()

    end = odrv.read_property_fast(f"axis{axis_num}.encoder.shadow_count")
    try:
        end_count = int(end)
    except (ValueError, TypeError):
        print(f"Could not read shadow_count: {end}")
        return

    delta = abs(end_count - start_count)
    print(f"\n\nEnd count: {end_count}")
    print(f"Counts in one revolution: {delta}")
    print(f"Equivalent lines: {delta // 4} (÷4 for quadrature)")
    print(f"\nTo use this, set CPR = {delta}")


def configure_encoder(odrv, axis_num=0):
    """Configure encoder settings"""
    print(f"\n{'='*50}")
    print(f"ENCODER CONFIGURATION - AXIS {axis_num}")
    print(f"{'='*50}")

    # Show current settings
    print("\nCurrent Encoder Configuration:")
    mode = odrv.read_property(f"axis{axis_num}.encoder.config.mode")
    cpr = odrv.read_property(f"axis{axis_num}.encoder.config.cpr")
    bandwidth = odrv.read_property(f"axis{axis_num}.encoder.config.bandwidth")
    is_ready = odrv.read_property(f"axis{axis_num}.encoder.is_ready")

    mode_names = {
        "0": "INCREMENTAL (Quadrature)",
        "1": "HALL (Hall Effect Sensors)",
        "2": "SINCOS (Sin/Cos Encoder)",
        "256": "SPI_ABS_AMS",
        "257": "SPI_ABS_CUI",
        "258": "SPI_ABS_AEAT",
    }

    print(f"  Mode: {mode_names.get(mode, 'UNKNOWN')} ({mode})")
    print(f"  CPR (Counts Per Revolution): {cpr}")
    print(f"  Bandwidth: {bandwidth}")
    print(f"  Is Ready: {is_ready}")

    # Ask if user wants to change settings
    print(f"\n{'─'*50}")
    print("Encoder Mode Options:")
    print("  0. Incremental (Quadrature encoder)")
    print("  1. Hall (Hall effect sensors)")
    print("  2. Sin/Cos encoder")
    print("  [Enter] to skip")
    print(f"{'─'*50}")

    choice = input("\nSelect encoder mode (or Enter to skip): ").strip()

    if choice in ["0", "1", "2"]:
        # Set encoder mode
        odrv.write_property(f"axis{axis_num}.encoder.config.mode", choice)
        print(f"✓ Encoder mode set to {mode_names.get(choice, choice)}")

        # Set CPR based on mode
        if choice == "0":  # Incremental
            print("\nCommon encoder line counts:")
            print("  1. 2048 lines")
            print("  2. 4096 lines")
            print("  3. 8192 lines")
            print("  4. Custom")
            line_choice = input("Select (1-4): ").strip()
            lines_map = {"1": 2048, "2": 4096, "3": 8192}
            if line_choice in lines_map:
                lines = lines_map[line_choice]
            elif line_choice == "4":
                lines = int(input("Enter line count: ").strip())
            else:
                lines = None
            if lines:
                cpr_value = lines * 4
                odrv.write_property(f"axis{axis_num}.encoder.config.cpr", str(cpr_value))
                print(f"✓ CPR set to {cpr_value} ({lines} lines × 4 quadrature)")

        elif choice == "1":  # Hall
            print("\nHall sensor CPR = 6 * pole_pairs")
            pole_pairs = input("Enter motor pole pairs (e.g., 7): ").strip()
            if pole_pairs:
                cpr_value = 6 * int(pole_pairs)
                odrv.write_property(f"axis{axis_num}.encoder.config.cpr", str(cpr_value))
                print(f"✓ CPR set to {cpr_value} (6 × {pole_pairs})")

        elif choice == "2":  # Sin/Cos
            cpr_input = input("Enter CPR value for Sin/Cos encoder: ").strip()
            if cpr_input:
                odrv.write_property(f"axis{axis_num}.encoder.config.cpr", cpr_input)
                print(f"✓ CPR set to {cpr_input}")

        # Ask about bandwidth
        bw_input = input("\nEnter bandwidth (default 1000, or Enter to skip): ").strip()
        if bw_input:
            odrv.write_property(f"axis{axis_num}.encoder.config.bandwidth", bw_input)
            print(f"✓ Bandwidth set to {bw_input}")

        # Save configuration
        print("\nSaving configuration...")
        odrv.send_command("ss")
        time.sleep(0.5)
        print("✓ Configuration saved!")

        # Show updated settings
        print("\nUpdated Encoder Configuration:")
        mode = odrv.read_property(f"axis{axis_num}.encoder.config.mode")
        cpr = odrv.read_property(f"axis{axis_num}.encoder.config.cpr")
        bandwidth = odrv.read_property(f"axis{axis_num}.encoder.config.bandwidth")
        print(f"  Mode: {mode_names.get(mode, 'UNKNOWN')} ({mode})")
        print(f"  CPR: {cpr}")
        print(f"  Bandwidth: {bandwidth}")
    else:
        print("No changes made.")


def setup_incremental_encoder(odrv, axis_num=0):
    """Quick setup for incremental (ABZ) encoder like MT6816"""
    print(f"\n{'='*50}")
    print(f"INCREMENTAL ENCODER SETUP - AXIS {axis_num}")
    print(f"{'='*50}")

    print("\nCommon MT6816 ABZ line counts:")
    print("  1. 1024 lines  → CPR  4096")
    print("  2. 2048 lines  → CPR  8192")
    print("  3. 4096 lines  → CPR 16384  (typical default)")
    print("  4. Custom")

    choice = input("\nSelect (1-4): ").strip()
    cpr_map = {"1": 4096, "2": 8192, "3": 16384}
    if choice in cpr_map:
        cpr = cpr_map[choice]
    elif choice == "4":
        lines = int(input("Enter line count: ").strip())
        cpr = lines * 4
    else:
        print("Cancelled.")
        return False

    pole_pairs = input("Enter motor pole pairs (e.g. 7, 14, 20): ").strip()
    if not pole_pairs:
        print("Pole pairs required!")
        return False

    print(f"\nConfiguring: mode=INCREMENTAL, CPR={cpr}, pole_pairs={pole_pairs}")

    odrv.write_property(f"axis{axis_num}.encoder.config.mode", "0")
    odrv.write_property(f"axis{axis_num}.encoder.config.cpr", str(cpr))
    odrv.write_property(f"axis{axis_num}.motor.config.pole_pairs", pole_pairs)
    odrv.write_property(f"axis{axis_num}.encoder.config.bandwidth", "1000")
    time.sleep(0.2)

    print("  ✓ Encoder mode: INCREMENTAL (0)")
    print(f"  ✓ CPR: {cpr}")
    print(f"  ✓ Pole pairs: {pole_pairs}")
    print(f"  ✓ Bandwidth: 1000")

    save = input("\nSave config? (y/n): ").strip().lower()
    if save == "y":
        odrv.send_command("ss")
        time.sleep(1)
        print("  ✓ Configuration saved!")

    print("\nNext steps:")
    print("  - Use 'Live encoder test' to verify the encoder counts when you rotate the shaft")
    print("  - Use 'Start motor' to calibrate and enter closed loop")
    return True


def setup_spi_encoder(odrv, axis_num=0):
    """Quick setup for SPI absolute encoder (MT6816 via SPI)"""
    print(f"\n{'='*50}")
    print(f"SPI ABSOLUTE ENCODER SETUP - AXIS {axis_num}")
    print(f"{'='*50}")

    print("\nWiring required (MT6816 → ODrive left connector):")
    print("  SCK  → SCK")
    print("  MISO → MISO")
    print("  MOSI → MOSI")
    print("  CSN  → GPIO3 (default for axis 0)")
    print("  GND  → GND")
    print("  VDD  → 3.3V")

    confirm = input("\nWiring correct? (y/n): ").strip().lower()
    if confirm != "y":
        print("Cancelled. Wire it up first.")
        return False

    cs_pin = input("CS GPIO pin (default 3 for axis0, 4 for axis1) [3]: ").strip()
    if not cs_pin:
        cs_pin = "3"

    pole_pairs = input("Enter motor pole pairs (e.g. 7, 14, 20): ").strip()
    if not pole_pairs:
        print("Pole pairs required!")
        return False

    print(f"\nConfiguring: mode=SPI_ABS_AMS (256), CS=GPIO{cs_pin}, pole_pairs={pole_pairs}")

    odrv.write_property(f"axis{axis_num}.encoder.config.mode", "256")
    odrv.write_property(f"axis{axis_num}.encoder.config.abs_spi_cs_gpio_pin", cs_pin)
    odrv.write_property(f"axis{axis_num}.encoder.config.cpr", "16384")
    odrv.write_property(f"axis{axis_num}.motor.config.pole_pairs", pole_pairs)
    odrv.write_property(f"axis{axis_num}.encoder.config.bandwidth", "1000")
    time.sleep(0.2)

    print("  ✓ Encoder mode: SPI_ABS_AMS (256)")
    print(f"  ✓ CS GPIO pin: {cs_pin}")
    print("  ✓ CPR: 16384 (14-bit)")
    print(f"  ✓ Pole pairs: {pole_pairs}")

    print("\nSaving and rebooting...")
    odrv.send_command("ss")
    time.sleep(1)
    print("  ✓ Configuration saved!")

    reboot = input("\nReboot ODrive now? (y/n): ").strip().lower()
    if reboot == "y":
        print("  Rebooting...")
        odrv.send_command("sr")
        time.sleep(3)
        print("  ✓ Rebooted. Reconnecting...")
        odrv.disconnect()
        time.sleep(1)
        if odrv.connect():
            print("  ✓ Reconnected!")
        else:
            print("  ✗ Reconnect failed. Restart the script.")
            return False

    print("\nChecking encoder...")
    time.sleep(0.5)
    enc_error = odrv.read_property(f"axis{axis_num}.encoder.error")
    pos = odrv.read_property(f"axis{axis_num}.encoder.pos_estimate")
    shadow = odrv.read_property(f"axis{axis_num}.encoder.shadow_count")

    print(f"  Encoder error: {enc_error}")
    print(f"  Position: {pos}")
    print(f"  Shadow count: {shadow}")

    if enc_error == "0":
        print("\n  ✓ No errors! Encoder appears to be communicating.")
        print("  Use 'Live encoder test' to verify angle changes when you rotate.")
    else:
        error_int = int(enc_error) if enc_error.isdigit() else -1
        if error_int & 0x40:
            print("\n  ✗ ABS_SPI_TIMEOUT — no response from encoder.")
            print("    Check wiring: SCK, MISO, MOSI, CSN, GND, 3.3V")
        elif error_int & 0x80:
            print("\n  ✗ ABS_SPI_COM_FAIL — communication error.")
            print("    Check wiring and that MT6816 is powered.")
        elif error_int & 0x100:
            print("\n  ✗ ABS_SPI_NOT_READY — encoder not ready.")
            print("    May need a magnet or the MT6816 is still initializing.")
        else:
            print(f"\n  ✗ Encoder error: {enc_error}")

    return True


def test_velocity_control(odrv, axis_num=0):
    """Test velocity control"""
    print(f"\n{'='*50}")
    print(f"VELOCITY CONTROL TEST - AXIS {axis_num}")
    print(f"{'='*50}")

    if not set_closed_loop(odrv, axis_num):
        return False

    # Test forward
    print("\nTest 1: Forward rotation (2 turns/sec)")
    set_velocity(odrv, axis_num, 2.0)
    time.sleep(3)
    vel = odrv.read_property(f"axis{axis_num}.encoder.vel_estimate")
    print(f"  Measured velocity: {vel} turns/sec")

    # Test reverse
    print("\nTest 2: Reverse rotation (-2 turns/sec)")
    set_velocity(odrv, axis_num, -2.0)
    time.sleep(3)
    vel = odrv.read_property(f"axis{axis_num}.encoder.vel_estimate")
    print(f"  Measured velocity: {vel} turns/sec")

    # Stop
    print("\nTest 3: Stop")
    set_velocity(odrv, axis_num, 0)
    time.sleep(1)
    vel = odrv.read_property(f"axis{axis_num}.encoder.vel_estimate")
    print(f"  Measured velocity: {vel} turns/sec")

    print("\n✓ Velocity control test complete")
    return True


def test_position_control(odrv, axis_num=0):
    """Test position control"""
    print(f"\n{'='*50}")
    print(f"POSITION CONTROL TEST - AXIS {axis_num}")
    print(f"{'='*50}")

    if not set_closed_loop(odrv, axis_num):
        return False

    # Get starting position
    start_pos = float(odrv.read_property(f"axis{axis_num}.encoder.pos_estimate"))
    print(f"\nStarting position: {start_pos:.3f} turns")

    # Test 1: Move forward
    print("\nTest 1: Move forward 2 turns")
    target = start_pos + 2.0
    set_position(odrv, axis_num, target)
    time.sleep(3)
    pos = odrv.read_property(f"axis{axis_num}.encoder.pos_estimate")
    print(f"  Current position: {pos} turns")

    # Test 2: Move back
    print("\nTest 2: Move back 2 turns")
    target = start_pos
    set_position(odrv, axis_num, target)
    time.sleep(3)
    pos = odrv.read_property(f"axis{axis_num}.encoder.pos_estimate")
    print(f"  Current position: {pos} turns")

    print("\n✓ Position control test complete")
    return True


# =====================================================
# Interactive Menu
# =====================================================


def dump_diagnostics(odrv, axis_num=0):
    """Read and display all important ODrive parameters for debugging"""
    print(f"\n{'='*50}")
    print(f"FULL DIAGNOSTIC DUMP - AXIS {axis_num}")
    print(f"{'='*50}")

    properties = [
        # Firmware
        ("FW Version Major", "fw_version_major"),
        ("FW Version Minor", "fw_version_minor"),
        ("FW Version Revision", "fw_version_revision"),
        ("HW Version Major", "hw_version_major"),
        ("HW Version Minor", "hw_version_minor"),
        # System
        ("VBus Voltage", "vbus_voltage"),
        # Axis
        ("Axis State", f"axis{axis_num}.current_state"),
        ("Axis Error", f"axis{axis_num}.error"),
        # Motor
        ("Motor Error", f"axis{axis_num}.motor.error"),
        ("Motor Is Calibrated", f"axis{axis_num}.motor.is_calibrated"),
        ("Motor Pre-Calibrated", f"axis{axis_num}.motor.config.pre_calibrated"),
        ("Motor Pole Pairs", f"axis{axis_num}.motor.config.pole_pairs"),
        ("Motor Current Lim", f"axis{axis_num}.motor.config.current_lim"),
        ("Motor Calibration Current", f"axis{axis_num}.motor.config.calibration_current"),
        ("Motor Type", f"axis{axis_num}.motor.config.motor_type"),
        ("Motor Resistance", f"axis{axis_num}.motor.config.phase_resistance"),
        ("Motor Inductance", f"axis{axis_num}.motor.config.phase_inductance"),
        # Encoder
        ("Encoder Error", f"axis{axis_num}.encoder.error"),
        ("Encoder Is Ready", f"axis{axis_num}.encoder.is_ready"),
        ("Encoder Pre-Calibrated", f"axis{axis_num}.encoder.config.pre_calibrated"),
        ("Encoder Mode", f"axis{axis_num}.encoder.config.mode"),
        ("Encoder CPR", f"axis{axis_num}.encoder.config.cpr"),
        ("Encoder Use Index", f"axis{axis_num}.encoder.config.use_index"),
        ("Encoder Calib Range", f"axis{axis_num}.encoder.config.calib_range"),
        ("Encoder SPI CS Pin", f"axis{axis_num}.encoder.config.abs_spi_cs_gpio_pin"),
        ("Encoder Bandwidth", f"axis{axis_num}.encoder.config.bandwidth"),
        ("Encoder Hall State", f"axis{axis_num}.encoder.hall_state"),
        ("Encoder Position", f"axis{axis_num}.encoder.pos_estimate"),
        ("Encoder Velocity", f"axis{axis_num}.encoder.vel_estimate"),
        ("Encoder Shadow Count", f"axis{axis_num}.encoder.shadow_count"),
        # Controller
        ("Controller Error", f"axis{axis_num}.controller.error"),
        ("Control Mode", f"axis{axis_num}.controller.config.control_mode"),
        ("Velocity Limit", f"axis{axis_num}.controller.config.vel_limit"),
        ("Vel Gain", f"axis{axis_num}.controller.config.vel_gain"),
        ("Vel Integrator Gain", f"axis{axis_num}.controller.config.vel_integrator_gain"),
        ("Pos Gain", f"axis{axis_num}.controller.config.pos_gain"),
        ("Input Mode", f"axis{axis_num}.controller.config.input_mode"),
        ("Vel Ramp Rate", f"axis{axis_num}.controller.config.vel_ramp_rate"),
        ("Trap Vel Limit", f"axis{axis_num}.trap_traj.config.vel_limit"),
        ("Trap Accel Limit", f"axis{axis_num}.trap_traj.config.accel_limit"),
        ("Trap Decel Limit", f"axis{axis_num}.trap_traj.config.decel_limit"),
    ]

    print(f"\n{'Property':<30} {'Value':>15}   Path")
    print(f"{'─'*30} {'─'*15}   {'─'*40}")

    for label, prop in properties:
        value = odrv.read_property(prop)
        print(f"{label:<30} {str(value):>15}   {prop}")

    print(f"\n{'='*50}")
    print("Copy and paste this output for troubleshooting.")
    print(f"{'='*50}")


def print_menu():
    """Display main menu"""
    print(f"\n{'─'*50}")
    print(f"ODrive Serial Test Menu [AXIS {_current_axis}]:")
    print("  1. Show ODrive info")
    print("  2. Show axis info")
    print("  3. Configure encoder (general)")
    print("  4. Setup incremental encoder (MT6816 ABZ)")
    print("  5. Setup SPI encoder (MT6816 SPI)")
    print("  6. Live encoder test")
    print("  7. Measure counts per revolution")
    print("  8. Encoder input test")
    print("  9. Calibrate motor")
    print("  10. ★ Start motor (full startup routine)")
    print("  11. Enable closed loop control")
    print("  12. Disable motor (idle)")
    print("  13. Test velocity control")
    print("  14. Test position control")
    print("  15. Manual velocity control")
    print("  16. Manual position control")
    print("  17. Full diagnostic dump")
    print("  18. Send raw command")
    print("  a. Switch axis (currently axis {})".format(_current_axis))
    print("  0. Exit")
    print(f"{'─'*50}")


def interactive_mode(odrv, axis_num):
    """Interactive menu"""
    global _current_axis
    _current_axis = axis_num

    print(f"\n{'='*50}")
    print(f"INTERACTIVE MODE - AXIS {_current_axis}")
    print(f"{'='*50}")

    apply_defaults(odrv)

    try:
        while True:
            print_menu()
            choice = input("\nEnter choice: ").strip().lower()

            if choice == "1":
                get_info(odrv)

            elif choice == "2":
                get_axis_info(odrv, axis_num)

            elif choice == "3":
                configure_encoder(odrv, axis_num)

            elif choice == "4":
                setup_incremental_encoder(odrv, axis_num)

            elif choice == "5":
                setup_spi_encoder(odrv, axis_num)

            elif choice == "6":
                test_encoder_live(odrv, axis_num)

            elif choice == "7":
                measure_cpr(odrv, axis_num)

            elif choice == "8":
                read_encoder_inputs(odrv, axis_num)

            elif choice == "9":
                calibrate(odrv, axis_num)

            elif choice == "10":
                start_motor(odrv, axis_num)

            elif choice == "11":
                set_closed_loop(odrv, axis_num)

            elif choice == "12":
                set_idle(odrv, axis_num)

            elif choice == "13":
                test_velocity_control(odrv, axis_num)
                set_idle(odrv, axis_num)

            elif choice == "14":
                test_position_control(odrv, axis_num)
                set_idle(odrv, axis_num)

            elif choice == "15":
                cur_limit = odrv.read_property(f"axis{axis_num}.controller.config.vel_limit")
                cur_ramp = odrv.read_property(f"axis{axis_num}.controller.config.vel_ramp_rate")
                cur_mode = odrv.read_property(f"axis{axis_num}.controller.config.input_mode")
                print(f"\n  Current vel_limit:    {cur_limit} turns/sec")
                print(f"  Current vel_ramp_rate: {cur_ramp} turns/sec²")
                print(f"  Current input_mode:    {cur_mode} (1=passthrough, 2=ramp)")

                try:
                    lim_input = input(f"\n  Set vel_limit [{cur_limit}]: ").strip()
                    if lim_input:
                        odrv.write_property(f"axis{axis_num}.controller.config.vel_limit", lim_input)
                        print(f"  vel_limit = {lim_input}")

                    ramp_input = input(f"  Set vel_ramp_rate [{cur_ramp}]: ").strip()
                    if ramp_input:
                        odrv.write_property(f"axis{axis_num}.controller.config.vel_ramp_rate", ramp_input)
                        odrv.write_property(f"axis{axis_num}.controller.config.input_mode", "2")
                        print(f"  vel_ramp_rate = {ramp_input}, input_mode = 2 (ramp)")
                except ValueError:
                    pass

                odrv.write_property(f"axis{axis_num}.controller.input_vel", "0")
                odrv.write_property(f"axis{axis_num}.controller.config.control_mode", "2")
                if set_closed_loop(odrv, axis_num):
                    print("\nManual velocity control. Enter speed in turns/sec.")
                    print("Type 'q' to stop.\n")
                    while True:
                        vel_input = input("  Speed (turns/sec): ").strip()
                        if vel_input.lower() == "q":
                            break
                        try:
                            vel = float(vel_input)
                            set_velocity(odrv, axis_num, vel)
                            time.sleep(3)

                            err = odrv.read_property(f"axis{axis_num}.error")
                            err_val = "".join(c for c in str(err) if c.isdigit())
                            motor_err = odrv.read_property(f"axis{axis_num}.motor.error")
                            motor_err_val = "".join(c for c in str(motor_err) if c.isdigit())
                            enc_err = odrv.read_property(f"axis{axis_num}.encoder.error")
                            cur_vel_raw = odrv.read_property(f"axis{axis_num}.encoder.vel_estimate")
                            iq = odrv.read_property(f"axis{axis_num}.motor.current_control.Iq_measured")

                            try:
                                cur_vel = float(cur_vel_raw)
                            except (ValueError, TypeError):
                                cur_vel = 0.0

                            print(f"  Measured: {cur_vel:.2f} turns/sec  Iq: {iq}A")

                            if err_val != "0" or motor_err_val != "0":
                                print(f"  ✗ ERROR! axis={err} motor={motor_err} encoder={enc_err}")
                                print("  Clearing and returning to idle...")
                                odrv.write_property(f"axis{axis_num}.error", "0")
                                odrv.write_property(f"axis{axis_num}.motor.error", "0")
                                odrv.write_property(f"axis{axis_num}.encoder.error", "0")
                                break

                            if vel != 0 and abs(cur_vel - vel) / max(abs(vel), 0.1) > 0.5:
                                print(f"  ⚠ Velocity mismatch! Commanded {vel:.1f} but got {cur_vel:.2f}")
                                print(f"    Motor may be current-limited (Iq={iq}A)")

                        except ValueError:
                            print("  Invalid number, try again")
                    set_velocity(odrv, axis_num, 0)
                    set_idle(odrv, axis_num)

            elif choice == "16":
                if set_closed_loop(odrv, axis_num):
                    origin = float(odrv.read_property(f"axis{axis_num}.encoder.pos_estimate"))
                    current_target = origin

                    cur_vel = odrv.read_property(f"axis{axis_num}.trap_traj.config.vel_limit")
                    cur_accel = odrv.read_property(f"axis{axis_num}.trap_traj.config.accel_limit")
                    cur_decel = odrv.read_property(f"axis{axis_num}.trap_traj.config.decel_limit")
                    print(f"  Current: vel={cur_vel} turns/s, accel={cur_accel} turns/s², decel={cur_decel} turns/s²")

                    vel_in = input(f"  Move speed [{cur_vel}]: ").strip()
                    if vel_in:
                        odrv.write_property(f"axis{axis_num}.trap_traj.config.vel_limit", vel_in)
                        print(f"  vel_limit = {vel_in}")

                    accel_in = input(f"  Acceleration [{cur_accel}]: ").strip()
                    if accel_in:
                        odrv.write_property(f"axis{axis_num}.trap_traj.config.accel_limit", accel_in)
                        odrv.write_property(f"axis{axis_num}.trap_traj.config.decel_limit", accel_in)
                        print(f"  accel/decel = {accel_in}")

                    print(f"\n  Origin: {origin:.3f} turns")
                    print("  Commands: number=relative move, 'z'=re-zero, 'q'=quit")
                    while True:
                        try:
                            cmd = input("  Move (turns): ").strip().lower()
                            if cmd == "q":
                                break
                            if cmd == "z":
                                origin = float(odrv.read_property(f"axis{axis_num}.encoder.pos_estimate"))
                                current_target = origin
                                print(f"  Origin re-zeroed at {origin:.3f}")
                                continue
                            delta = float(cmd)
                            current_target += delta
                            set_position(odrv, axis_num, current_target)
                            wait = max(
                                1.0,
                                abs(delta) / float(odrv.read_property(f"axis{axis_num}.trap_traj.config.vel_limit")) + 1.0,
                            )
                            time.sleep(wait)
                            pos = float(odrv.read_property(f"axis{axis_num}.encoder.pos_estimate"))
                            rel = pos - origin
                            err = odrv.read_property(f"axis{axis_num}.error")
                            err_val = "".join(c for c in str(err) if c.isdigit())
                            print(
                                f"  Position: {pos:.3f} (relative: {rel:+.3f})  target: {current_target:.3f}  error: {err}"
                            )
                            if err_val != "0":
                                print("  ✗ Axis error detected, stopping")
                                break
                        except ValueError:
                            print("  Invalid input, enter a number, 'z', or 'q'")
                    set_idle(odrv, axis_num)

            elif choice == "17":
                dump_diagnostics(odrv, axis_num)

            elif choice == "18":
                print("Raw command mode (Ctrl+C to exit)")
                try:
                    while True:
                        cmd = input("> ").strip()
                        if not cmd:
                            continue
                        response = odrv.send_command(cmd)
                        if response:
                            print(response)
                except KeyboardInterrupt:
                    print("\nExited raw command mode.")

            elif choice == "a":
                _current_axis = 1 - _current_axis
                axis_num = _current_axis
                print(f"\n  ★ Switched to AXIS {_current_axis}")

            elif choice == "0":
                print("\nExiting...")
                set_idle(odrv, 0)
                set_idle(odrv, 1)
                break

            else:
                print("✗ Invalid choice")

    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
        set_idle(odrv, axis_num)


# =====================================================
# Main Function
# =====================================================


def main():
    """Main application"""
    parser = argparse.ArgumentParser(description="ODrive Serial ASCII Protocol Test Script")
    parser.add_argument("-p", "--port", type=str, default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUDRATE, help=f"Baud rate (default: {DEFAULT_BAUDRATE})")
    parser.add_argument("-a", "--axis", type=int, default=0, choices=[0, 1], help="Axis number (0 or 1)")
    parser.add_argument("--info", action="store_true", help="Show axis info and exit")
    parser.add_argument("--calibrate", action="store_true", help="Run calibration sequence")
    parser.add_argument("--test-velocity", action="store_true", help="Run velocity control test")
    parser.add_argument("--test-position", action="store_true", help="Run position control test")
    args = parser.parse_args()

    # Create ODrive serial connection
    odrv = ODriveSerial(args.port, args.baud)

    if not odrv.connect():
        print("\n✗ Failed to connect to ODrive")
        print("  Check port name, wiring, and that ODrive is powered on.")
        return 1

    try:
        # Handle command-line options
        if args.info:
            get_axis_info(odrv, args.axis)
            return 0

        if args.calibrate:
            if not calibrate(odrv, args.axis):
                return 1

        if args.test_velocity:
            test_velocity_control(odrv, args.axis)
            set_idle(odrv, args.axis)

        if args.test_position:
            test_position_control(odrv, args.axis)
            set_idle(odrv, args.axis)

        # If no specific command, start interactive mode
        if not (args.info or args.calibrate or args.test_velocity or args.test_position):
            interactive_mode(odrv, args.axis)

    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
        set_idle(odrv, args.axis)

    finally:
        odrv.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
import argparse
import threading
import time
from dataclasses import dataclass

import minimalmodbus
import serial


REG_AXIS_ADDITION = 0x31   
REG_MOTOR_STATUS  = 0xF1   

REG_POSMODE3_F4   = 0xF4   
REG_EMSTOP_F7     = 0xF7   
REG_ENABLE_F3     = 0xF3   
COUNTS_PER_REV_AXIS = 0x4000     
ERR_COUNTS_PER_REV  = 51200      


def int_from_regs_be(regs, bits: int) -> int:
    """Combine 16-bit registers big-endian into a signed integer of 'bits'."""
    val = 0
    for r in regs:
        val = (val << 16) | (r & 0xFFFF)
    signbit = 1 << (bits - 1)
    if val & signbit:
        val -= 1 << bits
    return val


def int32_to_regs_be(x: int):
    """Signed int32 -> two 16-bit regs (big-endian)."""
    u = x & 0xFFFFFFFF
    return [(u >> 16) & 0xFFFF, u & 0xFFFF]


def make_instrument(port: str, slave_id: int, baud: int, timeout: float) -> minimalmodbus.Instrument:
    inst = minimalmodbus.Instrument(port, slave_id, mode=minimalmodbus.MODE_RTU)
    inst.serial.baudrate = baud
    inst.serial.bytesize = 8
    inst.serial.parity = serial.PARITY_NONE
    inst.serial.stopbits = 1
    inst.serial.timeout = timeout
    inst.clear_buffers_before_each_transaction = True
    inst.close_port_after_each_call = False
    return inst


@dataclass
class MotorState:
    axis_counts: int | None = None
    angle_deg: float | None = None
    err_counts: int | None = None
    err_deg: float | None = None
    status: int | None = None
    ok: bool = True
    last_error: str = ""


def read_state(inst: minimalmodbus.Instrument) -> MotorState:
    st = MotorState()
    try:
        regs_axis = inst.read_registers(REG_AXIS_ADDITION, 3, functioncode=4)
        axis = int_from_regs_be(regs_axis, 48)

        regs_err = inst.read_registers(REG_ANGLE_ERROR, 2, functioncode=4)
        err = int_from_regs_be(regs_err, 32)

        regs_status = inst.read_registers(REG_MOTOR_STATUS, 1, functioncode=4)
        status = regs_status[0] & 0xFF

        st.axis_counts = axis
        st.angle_deg = axis * 360.0 / COUNTS_PER_REV_AXIS
        st.err_counts = err
        st.err_deg = err * 360.0 / ERR_COUNTS_PER_REV
        st.status = status
        st.ok = True
        return st
    except Exception as e:
        st.ok = False
        st.last_error = str(e)
        return st


def emergency_stop(inst: minimalmodbus.Instrument):
    inst.write_register(REG_EMSTOP_F7, 1, number_of_decimals=0, functioncode=6, signed=False)


def enable_motor(inst: minimalmodbus.Instrument, enable: bool = True):
    inst.write_register(REG_ENABLE_F3, 1 if enable else 0, number_of_decimals=0, functioncode=6, signed=False)


def posmode3_relative_axis(inst: minimalmodbus.Instrument, acc: int, speed: int, rel_axis: int):
    hi, lo = int32_to_regs_be(rel_axis)
    inst.write_registers(REG_POSMODE3_F4, [acc & 0xFFFF, speed & 0xFFFF, hi, lo])


def main():
    ap = argparse.ArgumentParser(description="Dual MKS SERVO57D lift control (Modbus-RTU) - opposite directions + safety stop.")
    ap.add_argument("--port", required=True, help="Serial port (e.g. COM5 on Windows, /dev/ttyUSB0 on Linux)")
    ap.add_argument("--baud", type=int, default=38400, help="Baudrate (must match motor setting)")
    ap.add_argument("--id-a", type=int, required=True, help="Slave ID of motor A")
    ap.add_argument("--id-b", type=int, required=True, help="Slave ID of motor B")
    ap.add_argument("--speed", type=int, default=600, help="Speed 0..3000 (RPM units per manual)")
    ap.add_argument("--acc", type=int, default=20, help="Acceleration 0..255")
    ap.add_argument("--error-limit-deg", type=float, default=200.0, help="If |error| exceeds this, emergency stop both motors")
    ap.add_argument("--poll", type=float, default=0.2, help="Status refresh period in seconds")
    ap.add_argument("--timeout", type=float, default=0.25, help="Serial timeout seconds")
    ap.add_argument("--auto-enable", action="store_true", help="Send motor enable (F3=1) at startup")
    args = ap.parse_args()

    inst_a = make_instrument(args.port, args.id_a, args.baud, args.timeout)
    inst_b = make_instrument(args.port, args.id_b, args.baud, args.timeout)

    comm_lock = threading.Lock()
    stop_event = threading.Event()
    fault_event = threading.Event()
    pause_print = threading.Event()  

    latest_a = MotorState()
    latest_b = MotorState()

    def monitor_loop():
        nonlocal latest_a, latest_b
        while not stop_event.is_set():
            with comm_lock:
                latest_a = read_state(inst_a)
                latest_b = read_state(inst_b)

                if latest_a.ok and latest_a.err_deg is not None and abs(latest_a.err_deg) > args.error_limit_deg:
                    try:
                        emergency_stop(inst_a)
                        emergency_stop(inst_b)
                    except Exception:
                        pass
                    fault_event.set()

                if latest_b.ok and latest_b.err_deg is not None and abs(latest_b.err_deg) > args.error_limit_deg:
                    try:
                        emergency_stop(inst_a)
                        emergency_stop(inst_b)
                    except Exception:
                        pass
                    fault_event.set()

            if not pause_print.is_set():
                def fmt(st: MotorState):
                    if not st.ok:
                        return "ANG=---   ERR=---   ST=--"
                    return f"ANG={st.angle_deg:9.2f}°  ERR={st.err_deg:8.2f}°  ST={st.status:02d}"

                line = f"A({args.id_a}) {fmt(latest_a)} | B({args.id_b}) {fmt(latest_b)}"
                if fault_event.is_set():
                    line += "  >>> EMERGENCY STOP (error limit exceeded) <<<"
                print("\r" + line + " " * 10, end="", flush=True)

            time.sleep(args.poll)

    t = threading.Thread(target=monitor_loop, daemon=True)
    t.start()

    if args.auto_enable:
        with comm_lock:
            try:
                enable_motor(inst_a, True)
                enable_motor(inst_b, True)
            except Exception as e:
                print(f"\n[WARN] Could not auto-enable motors: {e}")

    print("\nType an angle in degrees (e.g. 30, -15). Motor A moves +angle, Motor B moves -angle.")
    print("Commands: stop | q")
    try:
        while True:
            if fault_event.is_set():
                print("\nFault latched: error limit exceeded. Fix the cause, then restart the script.")
                break

            pause_print.set()
            print("\n")
            if latest_a.ok and latest_b.ok:
                print(f"Status snapshot: A={latest_a.angle_deg:.2f}° err={latest_a.err_deg:.2f}° | "
                      f"B={latest_b.angle_deg:.2f}° err={latest_b.err_deg:.2f}°")
            else:
                print("Status snapshot: (communication error on at least one motor)")

            s = input("angle> ").strip().lower()
            pause_print.clear()

            if s in ("q", "quit", "exit"):
                break
            if s == "stop":
                with comm_lock:
                    emergency_stop(inst_a)
                    emergency_stop(inst_b)
                continue
            if not s:
                continue

            try:
                angle_deg = float(s)
            except ValueError:
                print("Please enter a number (degrees), or 'stop', or 'q'.")
                continue

            # Convert degrees -> axis counts (1 rev = 0x4000 counts)
            rel_axis = int(round(angle_deg * COUNTS_PER_REV_AXIS / 360.0))

            with comm_lock:
                # Position mode3: relative motion by axis (F4)
                posmode3_relative_axis(inst_a, args.acc, args.speed, rel_axis)
                posmode3_relative_axis(inst_b, args.acc, args.speed, -rel_axis)

    finally:
        stop_event.set()
        time.sleep(args.poll * 1.2)
        print("\nStopping...")
        try:
            inst_a.serial.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()

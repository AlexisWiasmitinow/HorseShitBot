#!/usr/bin/env python3
import argparse
import time
import re

import pymodbus
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
from pymodbus.exceptions import ModbusException, ModbusIOException

# ----------------- Registers (MKS SERVO42/57D RS485) -----------------
REG_WORKMODE       = 0x0082  # set work mode
MODE_SR_VFOC       = 5

REG_WORK_CURRENT   = 0x0083  # "Ma" work current in mA (SERVO57D max 5200mA)W


REG_ENABLE         = 0x00F3  # serial enable (1=enable, 0=disable)

REG_SPEED_MODE     = 0
6  # speed mode (write 2 regs)
REG_POSMODE3_RELAX = 0x00F4  # position mode3: relative motion by axis (write 4 regs)

REG_SET_AXIS_ZERO  = 0x0092  # set current axis to zero

# Input registers (FC04)
REG_ENC_ADD        = 0x0031  # encoder value (addition), int48 (read 3 regs)
REG_ANGLE_ERROR    = 0x0039  # angle error int32, 0..51200 <-> 0..360°
REG_PROTECT        = 0x003E  # protection state: 1 protected, 0 not

COUNTS_PER_REV = 0x4000      # per manual: +/-0x4000 per revolution


def _to_i32(u32: int) -> int:
    return u32 - 0x100000000 if (u32 & 0x80000000) else u32

def _split_i32(v: int):
    v &= 0xFFFFFFFF
    return (v >> 16) & 0xFFFF, v & 0xFFFF

def _sign_extend(value: int, bits: int) -> int:
    sign_bit = 1 << (bits - 1)
    return (value ^ sign_bit) - sign_bit


class MKSServo:
    def __init__(self, port: str, baud: int, dev_id: int, timeout: float, retries: int, local_echo: bool):
        self.dev_id = dev_id
        self.client = ModbusSerialClient(
            port=port,
            framer=FramerType.RTU,
            baudrate=baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=timeout,
            retries=retries,
            handle_local_echo=local_echo,
        )
        try:
            self.client.set_max_no_responses(100)
        except Exception:
            pass

    def connect(self):
        if not self.client.connect():
            raise RuntimeError("Could not open COM port / connect.")

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    # -------- safe wrappers (return None/False instead of crashing) --------
    def _safe(self, fn, *args, **kwargs):
        try:
            return fn(*args, device_id=self.dev_id, **kwargs)
        except (ModbusIOException, ModbusException):
            return None

    def write_reg(self, addr: int, value: int) -> bool:
        r = self._safe(self.client.write_register, address=addr, value=value)
        return (r is not None) and (not r.isError())

    def write_regs(self, addr: int, values) -> bool:
        r = self._safe(self.client.write_registers, address=addr, values=list(values))
        return (r is not None) and (not r.isError())

    def read_in(self, addr: int, count: int):
        r = self._safe(self.client.read_input_registers, address=addr, count=count)
        if r is None or r.isError():
            return None
        return r.registers

    # ---------------- high-level ----------------
    def set_workmode_vfoc(self):
        return self.write_reg(REG_WORKMODE, MODE_SR_VFOC)

    def set_work_current_ma(self, ma: int) -> bool:
        ma = max(0, min(5200, int(ma)))
        return self.write_reg(REG_WORK_CURRENT, ma)

    def enable(self, on=True):
        return self.write_reg(REG_ENABLE, 1 if on else 0)

    def speed_mode(self, cw=True, acc=8, rpm=60):
        acc = max(0, min(255, int(acc)))
        rpm = max(0, min(3000, int(rpm)))
        dir_bit = 1 if cw else 0
        reg0 = (dir_bit << 8) | acc
        return self.write_regs(REG_SPEED_MODE, [reg0, rpm])

    def stop_speed(self, decel=10):
        decel = max(0, min(255, int(decel)))
        reg0 = (0 << 8) | decel
        return self.write_regs(REG_SPEED_MODE, [reg0, 0])

    def set_axis_zero(self):
        return self.write_reg(REG_SET_AXIS_ZERO, 1)

    def read_protect_status(self):
        regs = self.read_in(REG_PROTECT, 1)
        if regs is None:
            return None
        return regs[0] & 0xFF  # 1 protected, 0 not

    def read_angle_error_deg(self):
        regs = self.read_in(REG_ANGLE_ERROR, 2)
        if regs is None:
            return None
        raw = ((regs[0] & 0xFFFF) << 16) | (regs[1] & 0xFFFF)
        err_counts = abs(_to_i32(raw))
        return err_counts * 360.0 / 51200.0  # manual mapping

    def read_encoder_addition_i48(self):
        regs = self.read_in(REG_ENC_ADD, 3)
        if regs is None:
            return None
        u48 = ((regs[0] & 0xFFFF) << 32) | ((regs[1] & 0xFFFF) << 16) | (regs[2] & 0xFFFF)
        return _sign_extend(u48, 48)

    def move_relative_deg_user(self, deg_user: float, acc=200, rpm=200) -> bool:
        """
        User convention:
          +deg => CCW
          -deg => CW
        """
        counts = int(round((abs(deg_user) / 360.0) * COUNTS_PER_REV))
        rel_axis = -counts if deg_user >= 0 else +counts  # +deg(CCW) => negative axis
        hi, lo = _split_i32(rel_axis)
        acc = max(0, min(255, int(acc)))
        rpm = max(0, min(3000, int(rpm)))
        return self.write_regs(REG_POSMODE3_RELAX, [acc, rpm, hi, lo])


def link_test(m: MKSServo, n=20, delay_s=0.1):
    ok = 0
    for _ in range(n):
        if m.read_in(REG_ENC_ADD, 3) is not None:
            ok += 1
        time.sleep(delay_s)
    return ok, n


def sensorless_home(m: MKSServo,
                    home_ma: int,
                    run_ma: int,
                    home_rpm: int,
                    home_acc: int,
                    stop_decel: int,
                    err_deg_threshold: float,
                    consecutive: int,
                    poll_ms: int,
                    timeout_s: float):
    print("Set SR_vFOC...")
    if not m.set_workmode_vfoc():
        raise RuntimeError("No reply on setting SR_vFOC (check ID/baud/wiring).")

    print(f"Set LOW current for homing: {home_ma} mA")
    if not m.set_work_current_ma(home_ma):
        raise RuntimeError("No reply on setting homing current (0x0083).")

    if not m.enable(True):
        raise RuntimeError("No reply on enable (0x00F3).")

    time.sleep(0.1)

    print(f"Homing: rotate CW {home_rpm} rpm (acc={home_acc})")
    if not m.speed_mode(cw=True, acc=home_acc, rpm=home_rpm):
        raise RuntimeError("No reply on speed command (0x00F6).")

    t0 = time.time()
    hits = 0
    missed = 0

    while True:
        if time.time() - t0 > timeout_s:
            m.stop_speed(stop_decel)
            raise TimeoutError("Homing timeout: never detected a hit (or comm unstable).")

        prot = m.read_protect_status()
        err = m.read_angle_error_deg()

        if prot is None and err is None:
            missed += 1
            if missed >= 10:
                m.stop_speed(stop_decel)
                raise RuntimeError("Too many consecutive timeouts while homing.")
        else:
            missed = 0

        hit_now = False
        if prot == 1:
            hit_now = True
        elif err is not None and err >= err_deg_threshold:
            hit_now = True

        hits = hits + 1 if hit_now else 0
        print(f"\rprotect={prot}  angle_err={err if err is not None else '---'}  hits={hits}/{consecutive}  ", end="")

        if hits >= consecutive:
            break

        time.sleep(poll_ms / 1000.0)

    print("\nHit detected -> stop.")
    m.stop_speed(stop_decel)
    time.sleep(0.2)

    enc0 = m.read_encoder_addition_i48()
    if enc0 is None:
        raise RuntimeError("Could not read encoder addition (0x0031) after homing.")

    # Set axis zero in the drive
    m.set_axis_zero()

    print(f"Set HIGH current for normal work: {run_ma} mA")
    m.set_work_current_ma(run_ma)

    return enc0


def pos_deg_user_from_enc(enc_now_i48: int, enc0_i48: int) -> float:
    # encoder: CW positive, CCW negative; user wants CCW positive
    # so pos_user = -(enc_now - enc0) * 360/0x4000
    delta = enc_now_i48 - enc0_i48
    return -(delta * 360.0 / COUNTS_PER_REV)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--home", action="store_true")

    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--retries", type=int, default=1)
    ap.add_argument("--local-echo", action="store_true")

    # currents
    ap.add_argument("--home-ma", type=int, default=1200, help="LOW work current (mA) during homing")
    ap.add_argument("--run-ma", type=int, default=3200, help="HIGH work current (mA) after homing")

    # homing tuning
    ap.add_argument("--home-rpm", type=int, default=60)
    ap.add_argument("--home-acc", type=int, default=8)
    ap.add_argument("--stop-decel", type=int, default=10)
    ap.add_argument("--err-deg", type=float, default=8.0)
    ap.add_argument("--consecutive", type=int, default=4)
    ap.add_argument("--poll-ms", type=int, default=150)
    ap.add_argument("--home-timeout", type=float, default=25.0)

    # move tuning
    ap.add_argument("--move-acc", type=int, default=200)
    ap.add_argument("--move-rpm", type=int, default=200)

    args = ap.parse_args()

    print(f"pymodbus {pymodbus.__version__}")
    m = MKSServo(args.port, args.baud, args.id, args.timeout, args.retries, args.local_echo)

    enc0 = None  # encoder baseline at zero-stop (home)

    # Soft limit: never allow position < 0
    EPS = 0.25  # degrees of tolerance (noise)

    try:
        print(f"Connecting {args.port} @ {args.baud}, id={args.id} ...")
        m.connect()
        print("Connected.")

        ok, n = link_test(m, n=10, delay_s=0.1)
        print(f"Link test (encoder add read): {ok}/{n} good replies")

        if args.home:
            enc0 = sensorless_home(
                m,
                home_ma=args.home_ma,
                run_ma=args.run_ma,
                home_rpm=args.home_rpm,
                home_acc=args.home_acc,
                stop_decel=args.stop_decel,
                err_deg_threshold=args.err_deg,
                consecutive=args.consecutive,
                poll_ms=args.poll_ms,
                timeout_s=args.home_timeout,
            )

            print("\nCommands:")
            print("  +180   -> move 180° CCW")
            print("  -180   -> move back CW (allowed only down to 0°)")
            print("  pos    -> print current position (deg, CCW positive)")
            print("  home   -> run homing again")
            print("  zero   -> set current position as zero")
            print("  q      -> quit\n")

            while True:
                s = input("cmd> ").strip().lower()
                if s in ("q", "quit", "exit"):
                    break

                if s == "pos":
                    if enc0 is None:
                        print("Not homed yet.")
                        continue
                    enc_now = m.read_encoder_addition_i48()
                    if enc_now is None:
                        print("No reply reading position.")
                        continue
                    pos = pos_deg_user_from_enc(enc_now, enc0)
                    if pos < 0 and abs(pos) <= EPS:
                        pos = 0.0
                    print(f"Position: {pos:.2f} deg (CCW positive, 0 = stop)")
                    continue

                if s == "home":
                    enc0 = sensorless_home(
                        m,
                        home_ma=args.home_ma,
                        run_ma=args.run_ma,
                        home_rpm=args.home_rpm,
                        home_acc=args.home_acc,
                        stop_decel=args.stop_decel,
                        err_deg_threshold=args.err_deg,
                        consecutive=args.consecutive,
                        poll_ms=args.poll_ms,
                        timeout_s=args.home_timeout,
                    )
                    continue

                if s == "zero":
                    enc_now = m.read_encoder_addition_i48()
                    if enc_now is None:
                        print("No reply reading position -> cannot zero.")
                        continue
                    enc0 = enc_now
                    m.set_axis_zero()
                    print("OK: set current position as zero.")
                    continue

                # numeric commands
                if re.fullmatch(r"[+-]?\d+(\.\d+)?", s):
                    if enc0 is None:
                        print("You must home first (--home or type 'home').")
                        continue

                    deg_cmd = float(s)

                    enc_now = m.read_encoder_addition_i48()
                    if enc_now is None:
                        print("No reply reading current position -> move cancelled.")
                        continue

                    pos_now = pos_deg_user_from_enc(enc_now, enc0)
                    if pos_now < 0 and abs(pos_now) <= EPS:
                        pos_now = 0.0

                    target = pos_now + deg_cmd  # because deg_cmd is relative in user coordinates

                    # ---- SOFT LIMIT: target must be >= 0 ----
                    if target < -EPS:
                        max_negative = -pos_now  # this is the most negative command that returns exactly to 0
                        if pos_now <= EPS:
                            print("BLOCKED: you are already at 0°. Can't go negative, you will hit something.")
                        else:
                            print(
                                f"BLOCKED: would go below 0° (hit stop).\n"
                                f"Current position: {pos_now:.2f}°.\n"
                                f"Most negative move allowed right now: {max_negative:.2f}° (to go back to zero)."
                            )
                        continue

                    # If we're very close to 0 due to noise, clamp to exactly zero move for safety
                    if deg_cmd < 0 and target < 0:
                        # within EPS, clamp to stop exactly at 0
                        deg_cmd = -pos_now
                        target = 0.0
                        print(f"Clamping: moving {deg_cmd:.2f}° to stop at 0° (to avoid going negative).")

                    ok = m.move_relative_deg_user(deg_cmd, acc=args.move_acc, rpm=args.move_rpm)
                    if not ok:
                        print("Move command sent but no reply (link drop).")
                    else:
                        print(f"OK: moved {deg_cmd:.2f}° -> target ~ {target:.2f}°")
                    continue

                print("Unknown command.")

        else:
            print("Run with --home to do homing + soft-limit mode.")

    finally:
        try:
            m.stop_speed(10)
            m.enable(False)
        except Exception:
            pass
        m.close()


if __name__ == "__main__":
    main()

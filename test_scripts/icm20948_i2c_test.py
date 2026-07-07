#!/usr/bin/env python3
"""
Standalone test script for the ICM-20948 9-DOF IMU ("20948") over I2C.
No ROS dependency — run on the Jetson to verify wiring/address, read the
WHO_AM_I registers, and stream accel/gyro/mag/temperature data.

The ICM-20948 uses a banked register map (REG_BANK_SEL selects bank 0-3) and
exposes its onboard AK09916 magnetometer on an internal aux I2C bus. This
script switches banks as needed, and reads the magnetometer via the chip's
I2C bypass mode (AK09916 then appears directly at 0x0C on the main bus).

Dependencies:
    pip install smbus2

Jetson I2C wiring (Jetson.GPIO-style 40-pin header):
    VCC  -> 3.3V (pin 1)      IMU boards are almost always 3.3V logic
    GND  -> GND (pin 6/9/...)
    SDA  -> pin 3  (I2C1, usually /dev/i2c-1 on Nano; 7/8/9 on Xavier/Orin)
    SCL  -> pin 5
    AD0  -> GND for address 0x68 (default), 3.3V for 0x69
    nCS  -> 3.3V (REQUIRED for I2C mode!)
             The ICM-20948 is an I2C/SPI combo chip. Leaving nCS floating or
             low can latch it into SPI mode, and I2C reads will just time out
             / NACK with no obvious reason. Tie it to VDD (3.3V) for I2C.
    FSYNC -> GND (or leave unconnected if the breakout ties it off already;
             not used by this script, but floating inputs can be noisy)

Usage:
    python3 icm20948_i2c_test.py                       # single reading, bus 1, addr 0x68
    python3 icm20948_i2c_test.py --scan                 # i2cdetect-style bus scan
    python3 icm20948_i2c_test.py --list-buses            # show available /dev/i2c-* devices
    python3 icm20948_i2c_test.py --bus 8 --addr 0x69     # Xavier/Orin often use bus 7/8/9
    python3 icm20948_i2c_test.py -c                      # stream continuously (Ctrl+C to stop)
    python3 icm20948_i2c_test.py -c -n 20 -r 5           # 20 samples at ~5 Hz
    python3 icm20948_i2c_test.py --no-mag                # skip AK09916 magnetometer test
    python3 icm20948_i2c_test.py --accel-fs 4 --gyro-fs 500
"""

import argparse
import glob
import sys
import time

# ── ICM-20948 register map (bank-relative addresses) ────────────
_ICM_WHO_AM_I = 0xEA  # expected value of Bank0.WHO_AM_I

_REG_BANK_SEL = 0x7F  # same physical register in every bank

# Bank 0
_B0_WHO_AM_I = 0x00
_B0_USER_CTRL = 0x03
_B0_PWR_MGMT_1 = 0x06
_B0_PWR_MGMT_2 = 0x07
_B0_INT_PIN_CFG = 0x0F
_B0_ACCEL_XOUT_H = 0x2D  # 12 bytes: accel xyz, gyro xyz (big-endian int16 each)
_B0_TEMP_OUT_H = 0x39    # 2 bytes

# Bank 2
_B2_GYRO_CONFIG_1 = 0x01
_B2_ACCEL_CONFIG = 0x14

# AK09916 magnetometer (accessed via I2C bypass at its own address)
_AK_ADDR = 0x0C
_AK_WHO_AM_I = 0x09  # expected value of WIA2
_AK_WIA2 = 0x01
_AK_ST1 = 0x10
_AK_HXL = 0x11  # 6 bytes: X,Y,Z little-endian int16
_AK_ST2 = 0x18
_AK_CNTL2 = 0x31
_AK_CNTL3 = 0x32

_ACCEL_FS_MAP = {2: (0x00, 16384.0), 4: (0x01, 8192.0), 8: (0x02, 4096.0), 16: (0x03, 2048.0)}
_GYRO_FS_MAP = {250: (0x00, 131.0), 500: (0x01, 65.5), 1000: (0x02, 32.8), 2000: (0x03, 16.4)}

_MAG_UT_PER_LSB = 0.15  # AK09916 sensitivity


class ICM20948:
    """Minimal ICM-20948 driver: bank switching, init, and raw sensor reads."""

    def __init__(self, bus: int, addr: int = 0x68):
        import smbus2
        self.bus_num = bus
        self.addr = addr
        self.bus = smbus2.SMBus(bus)
        self._bank = -1

    def close(self):
        self.bus.close()

    def _set_bank(self, bank: int, addr=None):
        if addr is None:
            addr = self.addr
        if addr == self.addr and self._bank == bank:
            return
        self.bus.write_byte_data(addr, _REG_BANK_SEL, (bank & 0x03) << 4)
        if addr == self.addr:
            self._bank = bank
        time.sleep(0.001)

    def read_reg(self, bank: int, reg: int, addr=None) -> int:
        if addr is None:
            addr = self.addr
        self._set_bank(bank, addr)
        return self.bus.read_byte_data(addr, reg)

    def read_block(self, bank: int, reg: int, length: int, addr=None) -> bytes:
        if addr is None:
            addr = self.addr
        self._set_bank(bank, addr)
        return bytes(self.bus.read_i2c_block_data(addr, reg, length))

    def write_reg(self, bank: int, reg: int, value: int, addr=None):
        if addr is None:
            addr = self.addr
        self._set_bank(bank, addr)
        self.bus.write_byte_data(addr, reg, value & 0xFF)

    def who_am_i(self) -> int:
        return self.read_reg(0, _B0_WHO_AM_I)

    def init(self, accel_fs: int = 2, gyro_fs: int = 250):
        # Reset, wait for the chip to come back, then wake it and pick a real clock source.
        self.write_reg(0, _B0_PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self._bank = -1
        self.write_reg(0, _B0_PWR_MGMT_1, 0x01)  # auto-select clock, exit sleep
        time.sleep(0.05)
        self.write_reg(0, _B0_PWR_MGMT_2, 0x00)  # enable accel + gyro axes

        accel_bits, self.accel_lsb_per_g = _ACCEL_FS_MAP[accel_fs]
        gyro_bits, self.gyro_lsb_per_dps = _GYRO_FS_MAP[gyro_fs]
        # bit0 (*_FCHOICE) = 1 enables the DLPF; FS_SEL occupies bits[2:1].
        self.write_reg(2, _B2_ACCEL_CONFIG, (accel_bits << 1) | 0x01)
        self.write_reg(2, _B2_GYRO_CONFIG_1, (gyro_bits << 1) | 0x01)
        time.sleep(0.01)

    def read_accel_gyro_temp(self) -> dict:
        data = self.read_block(0, _B0_ACCEL_XOUT_H, 14)
        raw = [self._s16(data[i], data[i + 1]) for i in range(0, 12, 2)]
        temp_raw = self._s16(data[12], data[13])
        ax, ay, az, gx, gy, gz = raw
        return {
            "accel_g": (ax / self.accel_lsb_per_g, ay / self.accel_lsb_per_g, az / self.accel_lsb_per_g),
            "gyro_dps": (gx / self.gyro_lsb_per_dps, gy / self.gyro_lsb_per_dps, gz / self.gyro_lsb_per_dps),
            "temp_c": temp_raw / 333.87 + 21.0,
        }

    @staticmethod
    def _s16(hi: int, lo: int) -> int:
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    # ── magnetometer (AK09916) via I2C bypass ──
    def enable_bypass(self):
        self.write_reg(0, _B0_USER_CTRL, 0x00)     # disable aux I2C master
        self.write_reg(0, _B0_INT_PIN_CFG, 0x02)   # BYPASS_EN: AK09916 now visible at 0x0C
        time.sleep(0.01)

    def mag_who_am_i(self) -> int:
        return self.bus.read_byte_data(_AK_ADDR, _AK_WIA2)

    def mag_init(self):
        self.bus.write_byte_data(_AK_ADDR, _AK_CNTL3, 0x01)  # soft reset
        time.sleep(0.01)
        self.bus.write_byte_data(_AK_ADDR, _AK_CNTL2, 0x08)  # continuous mode 100 Hz

    def read_mag(self, timeout: float = 0.2) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            st1 = self.bus.read_byte_data(_AK_ADDR, _AK_ST1)
            if st1 & 0x01:  # DRDY
                break
            time.sleep(0.005)
        else:
            raise TimeoutError("AK09916 data-ready timeout")

        data = self.bus.read_i2c_block_data(_AK_ADDR, _AK_HXL, 6)
        st2 = self.bus.read_byte_data(_AK_ADDR, _AK_ST2)  # must read ST2 to latch/release the next sample
        overflow = bool(st2 & 0x08)

        def s16le(lo, hi):
            v = (hi << 8) | lo
            return v - 65536 if v & 0x8000 else v

        mx = s16le(data[0], data[1]) * _MAG_UT_PER_LSB
        my = s16le(data[2], data[3]) * _MAG_UT_PER_LSB
        mz = s16le(data[4], data[5]) * _MAG_UT_PER_LSB
        return {"mag_ut": (mx, my, mz), "overflow": overflow}


# ── I2C bus discovery / scan (no smbus2 dependency needed) ──────

def list_buses():
    devs = sorted(glob.glob("/dev/i2c-*"))
    print("=== I2C buses ===")
    if not devs:
        print("  (none found — enable I2C, e.g. via jetson-io, or check wiring/kernel modules)")
    for d in devs:
        print(f"  {d}")
    return devs


def scan_bus(bus: int, quiet: bool = False) -> list:
    import smbus2
    if not quiet:
        print(f"=== I2C scan: bus {bus} (like `i2cdetect -y {bus}`) ===")
    try:
        smb = smbus2.SMBus(bus)
    except FileNotFoundError:
        print(f"  ERROR: /dev/i2c-{bus} not found. Try --list-buses to see what exists.")
        return []
    except PermissionError:
        print(f"  ERROR: permission denied opening /dev/i2c-{bus}. "
              f"Try: sudo usermod -aG i2c $USER (log out/in), or run with sudo.")
        return []
    found = []
    if not quiet:
        print("     " + " ".join(f"{c:02x}" for c in range(16)))
    for row in range(0, 0x78, 16):
        line = f"{row:02x}: "
        for col in range(16):
            addr = row + col
            if addr < 0x03 or addr > 0x77:
                line += "   "
                continue
            try:
                smb.read_byte(addr)
                line += f"{addr:02x} "
                found.append(addr)
            except OSError:
                line += "-- "
        if not quiet:
            print(line)
    smb.close()
    if not quiet:
        if found:
            print(f"\n  Found device(s) at: {', '.join(hex(a) for a in found)}")
            if 0x68 in found or 0x69 in found:
                print("  0x68/0x69 matches the ICM-20948's default I2C address(es) (AD0 low/high).")
        else:
            print("\n  No devices found. Check wiring (SDA/SCL not swapped), pull-ups, 3.3V power, and AD0 level.")
    return found


def scan_all_buses():
    devs = list_buses()
    if not devs:
        return
    print("\n=== Scanning every bus for 0x68 / 0x69 (ICM-20948 candidates) ===")
    hits = []
    for d in devs:
        bus_num = int(d.rsplit("-", 1)[-1])
        found = scan_bus(bus_num, quiet=True)
        candidates = [a for a in found if a in (0x68, 0x69)]
        status = f"candidates: {[hex(a) for a in candidates]}" if candidates else (
            f"other device(s): {[hex(a) for a in found]}" if found else "no devices")
        print(f"  {d}: {status}")
        if candidates:
            hits.extend((bus_num, a) for a in candidates)
    if hits:
        print("\n  Likely match(es):")
        for bus_num, addr in hits:
            print(f"    python3 {sys.argv[0]} --bus {bus_num} --addr 0x{addr:02X}")
    else:
        print("\n  No 0x68/0x69 device found on any bus. This points to a wiring/power issue")
        print("  rather than a bus-number mistake — see the README troubleshooting section.")


# ── printing helpers ─────────────────────────────────────────────

def print_reading(sample: dict, mag: dict | None):
    ax, ay, az = sample["accel_g"]
    gx, gy, gz = sample["gyro_dps"]
    line = (
        f"accel(g)=[{ax:+6.3f} {ay:+6.3f} {az:+6.3f}]  "
        f"gyro(dps)=[{gx:+7.2f} {gy:+7.2f} {gz:+7.2f}]  "
        f"temp={sample['temp_c']:5.1f}C"
    )
    if mag is not None:
        mx, my, mz = mag["mag_ut"]
        line += f"  mag(uT)=[{mx:+7.2f} {my:+7.2f} {mz:+7.2f}]"
        if mag["overflow"]:
            line += " (OVERFLOW)"
    print(f"  {line}")


def main():
    parser = argparse.ArgumentParser(
        description="ICM-20948 9-DOF IMU I2C test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  python3 icm20948_i2c_test.py                    # single reading, bus 1, addr 0x68
  python3 icm20948_i2c_test.py --scan              # find the device's address on a bus
  python3 icm20948_i2c_test.py --scan-all          # not sure which bus? check all of them
  python3 icm20948_i2c_test.py --list-buses        # see which /dev/i2c-N exist
  python3 icm20948_i2c_test.py -c -n 20 -r 5       # stream 20 samples at ~5 Hz
  python3 icm20948_i2c_test.py --bus 8 --addr 0x69 # Xavier/Orin often use bus 7/8/9
""",
    )
    parser.add_argument("--bus", type=int, default=1,
        help="I2C bus number, i.e. /dev/i2c-N (default 1; Jetson Nano=1, Xavier/Orin often 7/8/9)")
    parser.add_argument("--addr", type=lambda s: int(s, 0), default=0x68,
        help="I2C address (default 0x68; 0x69 if AD0 is tied high)")
    parser.add_argument("--scan", action="store_true",
        help="Scan the bus for device addresses (like i2cdetect -y) and exit")
    parser.add_argument("--scan-all", action="store_true",
        help="Scan every available /dev/i2c-* bus for a 0x68/0x69 device and exit "
             "(use this if you're not sure which bus the header maps to)")
    parser.add_argument("--list-buses", action="store_true",
        help="List available /dev/i2c-* devices and exit")
    parser.add_argument("--no-mag", action="store_true",
        help="Skip the AK09916 magnetometer test (accel/gyro only)")
    parser.add_argument("--accel-fs", type=int, choices=sorted(_ACCEL_FS_MAP), default=2,
        help="Accelerometer full-scale range in g (default 2)")
    parser.add_argument("--gyro-fs", type=int, choices=sorted(_GYRO_FS_MAP), default=250,
        help="Gyroscope full-scale range in dps (default 250)")
    parser.add_argument("-c", "--continuous", action="store_true",
        help="Stream readings continuously (Ctrl+C to stop)")
    parser.add_argument("-n", "--samples", type=int, default=0,
        help="With --continuous: stop after N samples (default: run until Ctrl+C)")
    parser.add_argument("-r", "--rate", type=float, default=10.0,
        help="With --continuous: sample rate in Hz (default 10)")
    args = parser.parse_args()

    if args.list_buses:
        list_buses()
        return

    if args.scan_all:
        scan_all_buses()
        return

    if args.scan:
        scan_bus(args.bus)
        return

    try:
        import smbus2  # noqa: F401
    except ImportError:
        print("ERROR: smbus2 not installed. Run: pip install smbus2", file=sys.stderr)
        sys.exit(1)

    print(f"Bus: /dev/i2c-{args.bus}  Address: 0x{args.addr:02X}")
    try:
        imu = ICM20948(args.bus, args.addr)
    except FileNotFoundError:
        print(f"ERROR: /dev/i2c-{args.bus} not found.", file=sys.stderr)
        print("Enable I2C for your platform (e.g. jetson-io on Jetson) or check --bus.", file=sys.stderr)
        sys.exit(1)

    try:
        print("\n=== WHO_AM_I check ===")
        try:
            whoami = imu.who_am_i()
        except OSError as e:
            print(f"  ERROR: no ACK from 0x{args.addr:02X} on bus {args.bus}: {e}", file=sys.stderr)
            print("  Check wiring (SDA/SCL), pull-ups, power, and AD0 level (0x68 vs 0x69).", file=sys.stderr)
            print("  Also check nCS: it MUST be tied to 3.3V for I2C mode (floating/low", file=sys.stderr)
            print("  can select SPI mode instead, which looks identical to a wiring fault).", file=sys.stderr)
            print(f"  Try: python3 {sys.argv[0]} --scan --bus {args.bus}", file=sys.stderr)
            sys.exit(1)

        if whoami == _ICM_WHO_AM_I:
            print(f"  0x{whoami:02X} == 0x{_ICM_WHO_AM_I:02X} -- OK, this is an ICM-20948")
        else:
            print(f"  WARNING: got 0x{whoami:02X}, expected 0x{_ICM_WHO_AM_I:02X}. "
                  f"Wrong chip/address, or device not initialised.")

        print("\n=== Initialising (accel +/-%dg, gyro +/-%ddps) ===" % (args.accel_fs, args.gyro_fs))
        imu.init(accel_fs=args.accel_fs, gyro_fs=args.gyro_fs)
        print("  done")

        mag_ok = False
        if not args.no_mag:
            print("\n=== Magnetometer (AK09916) WHO_AM_I check (via I2C bypass) ===")
            try:
                imu.enable_bypass()
                mag_whoami = imu.mag_who_am_i()
                if mag_whoami == _AK_WHO_AM_I:
                    print(f"  0x{mag_whoami:02X} == 0x{_AK_WHO_AM_I:02X} -- OK, AK09916 found at 0x{_AK_ADDR:02X}")
                    imu.mag_init()
                    mag_ok = True
                else:
                    print(f"  WARNING: got 0x{mag_whoami:02X}, expected 0x{_AK_WHO_AM_I:02X}. Skipping mag reads.")
            except OSError as e:
                print(f"  WARNING: could not reach AK09916 at 0x{_AK_ADDR:02X}: {e}")
                print("  Continuing without magnetometer data.")

        print("\n=== Sensor reading" + (" (streaming)" if args.continuous else "") + " ===")
        count = 0
        period = 1.0 / args.rate if args.rate > 0 else 0.0
        try:
            while True:
                sample = imu.read_accel_gyro_temp()
                mag = None
                if mag_ok:
                    try:
                        mag = imu.read_mag()
                    except (OSError, TimeoutError) as e:
                        print(f"  (mag read failed: {e})")
                print_reading(sample, mag)
                count += 1
                if not args.continuous:
                    break
                if args.samples and count >= args.samples:
                    break
                time.sleep(period)
        except KeyboardInterrupt:
            print("\n  Stopped by user.")

        print(f"\n{count} sample(s) read successfully.")
        print("Look for: accel Z ~= 1g at rest (whichever axis is vertical), gyro ~0dps at rest,")
        print("temp near room temperature, and mag values changing smoothly as you rotate the board.")
        print("\nIMU test complete.")

    finally:
        imu.close()


if __name__ == "__main__":
    main()

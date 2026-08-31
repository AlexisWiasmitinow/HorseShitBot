#!/usr/bin/env python3

import argparse
import glob
import os
import sys
import time

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient


# ============================================================
# ADC SETTINGS
# ============================================================

# Taken from our existing modbus_adc_tool.py
DEFAULT_BAUDS = [9600, 19200, 4800, 2400, 1200]

# Try the most likely Modbus addresses first.
DEFAULT_SLAVES = range(1, 11)

# N43VD04 input values
CHANNEL_START_REGISTER = 0x0000
CHANNEL_COUNT = 4

# Existing HorseShitBot ADC code assumes:
# raw / 100 = voltage measured by ADC
RAW_SCALE = 100.0


# ============================================================
# MODBUS
# ============================================================

def make_client(port, baudrate, timeout=0.4):
    return ModbusSerialClient(
        port=port,
        framer=FramerType.RTU,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=timeout,
        retries=0,
    )


def read_raw_channels(client, slave):
    """
    Read the four ADC channels.

    Modbus function 03:
        Read Holding Registers

    Registers:
        0x0000 = CH1
        0x0001 = CH2
        0x0002 = CH3
        0x0003 = CH4
    """

    try:
        result = client.read_holding_registers(
            address=CHANNEL_START_REGISTER,
            count=CHANNEL_COUNT,
            device_id=slave,
        )

        if result is None or result.isError():
            return None

        if not hasattr(result, "registers"):
            return None

        if len(result.registers) < CHANNEL_COUNT:
            return None

        return result.registers[:CHANNEL_COUNT]

    except Exception:
        return None


# ============================================================
# CONVERSION
# ============================================================

def convert_voltage(raw, divider_factor):
    adc_voltage = raw / RAW_SCALE

    # Existing HorseShitBot definition:
    #
    # ADC_voltage = real_voltage * divider_factor
    #
    # therefore:
    #
    # real_voltage = ADC_voltage / divider_factor

    real_voltage = adc_voltage / divider_factor

    return adc_voltage, real_voltage


# ============================================================
# USB PORT DETECTION
# ============================================================

def find_serial_ports():
    ports = []

    for pattern in (
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    ):
        ports.extend(glob.glob(pattern))

    return sorted(set(ports))


# ============================================================
# DEVICE SEARCH
# ============================================================

def probe(port, baudrate, slave):
    client = make_client(port, baudrate)

    try:
        if not client.connect():
            return None

        return read_raw_channels(client, slave)

    except Exception:
        return None

    finally:
        client.close()


def scan_device(port=None):
    if port:
        ports = [port]
    else:
        ports = find_serial_ports()

    if not ports:
        print()
        print("ERROR: No USB serial device found.")
        print()
        print("Expected something like:")
        print("  /dev/ttyUSB0")
        print()
        print("Check:")
        print("  1. USB-RS485 adapter is plugged in")
        print("  2. ADC has power")
        print("  3. USB cable/adapter works")
        return None

    print()
    print("USB serial ports:")
    for p in ports:
        print(f"  {p}")

    print()
    print("Scanning for ADC...")
    print()

    for current_port in ports:

        if not os.access(current_port, os.R_OK | os.W_OK):
            print(
                f"WARNING: No read/write permission for {current_port}"
            )

        for baud in DEFAULT_BAUDS:

            print(
                f"Trying {current_port} @ {baud} baud...",
                flush=True,
            )

            client = make_client(current_port, baud)

            try:
                if not client.connect():
                    continue

                for slave in DEFAULT_SLAVES:

                    values = read_raw_channels(
                        client,
                        slave,
                    )

                    if values is not None:

                        print()
                        print("=" * 60)
                        print("ADC FOUND")
                        print("=" * 60)
                        print(f"Port:   {current_port}")
                        print(f"Baud:   {baud}")
                        print(f"Slave:  {slave}")
                        print("=" * 60)

                        return (
                            current_port,
                            baud,
                            slave,
                            values,
                        )

            except PermissionError:
                print()
                print(
                    f"PERMISSION ERROR opening {current_port}"
                )
                print()
                print("Your user probably needs the 'dialout' group:")
                print()
                print("  sudo usermod -aG dialout $USER")
                print()
                print("Then log out and back in.")
                return None

            except Exception:
                pass

            finally:
                client.close()

    return None


# ============================================================
# DISPLAY
# ============================================================

def show_readings(values, divider_factor):
    print()

    for index, raw in enumerate(values, start=1):

        adc_voltage, real_voltage = convert_voltage(
            raw,
            divider_factor,
        )

        print(
            f"CH{index}: "
            f"raw={raw:5d}   "
            f"ADC={adc_voltage:7.3f} V   "
            f"real={real_voltage:7.3f} V"
        )


# ============================================================
# CONTINUOUS READING
# ============================================================

def watch(port, baudrate, slave, divider_factor):
    client = make_client(
        port,
        baudrate,
        timeout=0.5,
    )

    if not client.connect():
        print(f"Could not open {port}")
        return 1

    print()
    print("Continuous ADC reading")
    print("Press Ctrl+C to stop.")
    print()

    try:

        while True:

            values = read_raw_channels(
                client,
                slave,
            )

            if values is None:
                print("READ ERROR")
            else:

                line = []

                for channel, raw in enumerate(
                    values,
                    start=1,
                ):
                    _, voltage = convert_voltage(
                        raw,
                        divider_factor,
                    )

                    line.append(
                        f"CH{channel}={voltage:.3f}V"
                    )

                print(
                    "   ".join(line),
                    flush=True,
                )

            time.sleep(1)

    except KeyboardInterrupt:
        print()
        print("Stopped.")

    finally:
        client.close()

    return 0


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="HorseShitBot N43VD04 ADC test"
    )

    parser.add_argument(
        "--port",
        default=None,
        help="Serial port, e.g. /dev/ttyUSB0",
    )

    parser.add_argument(
        "--divider-factor",
        type=float,
        default=1.0,
        help=(
            "Voltage divider factor. "
            "Use 1.0 for direct ADC voltage. "
            "Existing HSB code notes 0.5 for final robot."
        ),
    )

    parser.add_argument(
        "--watch",
        action="store_true",
        help="Continuously print ADC voltages",
    )

    args = parser.parse_args()

    print("=" * 60)
    print("HORSESHITBOT ADC TEST")
    print("=" * 60)

    print()
    print("Searching for USB -> RS485 -> ADC...")

    result = scan_device(args.port)

    if result is None:

        print()
        print("=" * 60)
        print("ADC NOT FOUND")
        print("=" * 60)

        print()
        print("Check this chain:")
        print()
        print("PowerAIBox")
        print("   |")
        print("   +-- USB-RS485 adapter")
        print("            |")
        print("            +-- RS485 A/B")
        print("                    |")
        print("                    +-- ADC")
        print()
        print("Also check that the ADC itself has power.")

        return 1

    port, baud, slave, values = result

    show_readings(
        values,
        args.divider_factor,
    )

    print()
    print("=" * 60)
    print("SUCCESS")
    print("=" * 60)

    print()
    print("The complete communication chain works:")
    print()
    print(
        f"PowerAIBox -> {port} -> RS485 -> "
        f"ADC slave {slave}"
    )

    if args.watch:
        return watch(
            port,
            baud,
            slave,
            args.divider_factor,
        )

    print()
    print("For continuous measurements run:")
    print()
    print(
        f"python3 adc_test.py "
        f"--port {port} "
        f"--divider-factor {args.divider_factor} "
        f"--watch"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

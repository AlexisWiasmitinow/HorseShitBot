#!/usr/bin/env python3

import time
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

PORT = "/dev/ttyUSB0"
BAUDRATES = [19200, 9600, 4800, 2400, 1200]
SLAVES = range(33, 34)


def client_for(baud):
    return ModbusSerialClient(
        port=PORT,
        framer=FramerType.RTU,
        baudrate=baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.3,
        retries=0,
    )


def set_baudrate_19200(client, slave):
    """
    Sets the module baud rate to 19200 (value 4 at register 0x00FE).
    Note: The new baud rate will take effect only after powering the module off and on again.
    """
    try:
        response = client.write_register(
            address=0x00FE,
            value=4,  # 0: 1200, 1: 2400, 2: 4800, 3: 9600, 4: 19200
            device_id=slave,
        )
        if response is not None and not response.isError():
            print(f"Successfully set baud rate to 19200 on slave {slave}.")
            print("POWER CYCLE THE ADC MODULE FOR THE CHANGE TO TAKE EFFECT.")
            return True
        else:
            print(f"Failed to set baud rate: {response}")
            return False
    except Exception as e:
        print(f"Error setting baud rate: {e}")
        return False

def read_channels(client, slave):
    # Try holding registers first
    try:
        r = client.read_holding_registers(
            address=0x0000,
            count=4,
            device_id=slave,
        )
        if r is not None and not r.isError():
            return r.registers[:4]
    except Exception:
        pass

    # Some ADC variants expose them as input registers
    try:
        r = client.read_input_registers(
            address=0x0000,
            count=4,
            device_id=slave,
        )
        if r is not None and not r.isError():
            return r.registers[:4]
    except Exception:
        pass

    return None

def set_slave_address(client, current_slave, new_slave):
    """
    Changes the RS485 slave ID by writing to register 0x00FD.
    Valid new_slave values: 1 to 247.
    """
    if not (1 <= new_slave <= 247):
        print("Error: Address must be between 1 and 247.")
        return False

    try:
        response = client.write_register(
            address=0x00FD,
            value=new_slave,
            device_id=current_slave,
        )
        if response is not None and not response.isError():
            print(f"Successfully changed RS485 address from {current_slave} to {new_slave}.")
            return True
        else:
            print(f"Failed to set RS485 address: {response}")
            return False
    except Exception as e:
        print(f"Error changing RS485 address: {e}")
        return False


print("=" * 60)
print("HORSESHITBOT ADC TEST")
print("=" * 60)
print("USB-RS485:", PORT)
print()
print("Searching for ADC...")

found = None

for baud in BAUDRATES:
    print(f"Trying {baud} baud...")

    client = client_for(baud)
    #set_baudrate_19200(client, 1)
    #set_slave_address(client, current_slave=1, new_slave=33)
    try:
        if not client.connect():
            continue

        for slave in SLAVES:
            values = read_channels(client, slave)

            if values is not None:
                found = (baud, slave)
                break

    finally:
        client.close()

    if found:
        break


if not found:
    print()
    print("ADC NOT FOUND")
    print()
    print("USB adapter is detected correctly.")
    print("Check:")
    print("  - ADC has power")
    print("  - RS485 A/B wires")
    print("  - if necessary swap A and B")
    raise SystemExit(1)


baud, slave = found

print()
print("=" * 60)
print("ADC FOUND")
print("=" * 60)
print("Port :", PORT)
print("Baud :", baud)
print("Slave:", slave)
print()
print("TURN THE BLACK KNOB NOW.")
print("A channel value should change.")
print("Ctrl+C stops the test.")
print()

client = client_for(baud)

if not client.connect():
    raise SystemExit("Could not reopen serial port")

try:
    previous = None

    while True:
        values = read_channels(client, slave)

        if values is None:
            print("READ ERROR")
            time.sleep(0.5)
            continue

        # Existing HorseShitBot ADC code assumes raw / 100 = volts
        volts = [v / 100.0 for v in values]

        changed = previous is not None and values != previous

        print(
            "   ".join(
                f"CH{i+1}={volts[i]:7.2f}V"
                for i in range(4)
            )
            + ("   <-- CHANGED" if changed else ""),
            flush=True,
        )

        previous = list(values)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    client.close()

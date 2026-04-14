#!/bin/python3
import serial, pymodbus, time, logging
from platform import python_version

from pymodbus.client import ModbusSerialClient
from pymodbus import ModbusException, pymodbus_apply_logging_config
import argparse
import logging.config

modbusVersion = pymodbus.__version__
pythonVersion = python_version()
suggestedPythonVersion = "3.12.1"
suggestedModbusVersion = "3.10"
baudRate = 38400
# baudRate = 9600
slaveAddress = 1
if not modbusVersion.startswith("3.") or tuple(int(x) for x in modbusVersion.split(".")[:2]) < (3, 10):
    print(f"Warning: pymodbus version is {modbusVersion}, but >={suggestedModbusVersion} is required")
else:
    print(f"pymodbus version: {modbusVersion}")
if pythonVersion != suggestedPythonVersion:
    print(f"Warning: Python version is {pythonVersion}, but {suggestedPythonVersion} is suggested")
else:
    print(f"Python version: {pythonVersion}")

# client = ModbusSerialClient(port="COM9", baudrate=baudRate, parity="N", stopbits=1, bytesize=8, timeout=0.1, retries=1)
# client = ModbusSerialClient(port="/dev/ttyUSB0", baudrate=115200, parity="N", stopbits=1, bytesize=8)
# if not client.connect():
#   print("Error connecting to RS485 port")

print("=====================================================")
print("Starting tests")
print("=====================================================")

logging_config = {
    "version": 1,
    "formatters": {"verbose": {"format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"}},
    "handlers": {
        "console": {
            "class": "logging.StreamHandler",
            "formatter": "verbose",
            "level": "DEBUG",
        },
    },
    "root": {"level": "DEBUG", "handlers": ["console"]},
    "loggers": {
        "pymodbus": {  # Explicit configuration for pymodbus logger
            "level": "DEBUG",
            "handlers": ["console"],
            "propagate": False,
        },
        "pymodbus.transaction": {  # Focus on transaction details
            "level": "DEBUG",
            "handlers": ["console"],
            "propagate": False,
        },
    },
}
logging.config.dictConfig(logging_config)
# Log.setLevel(logging.DEBUG)
# commandTimeOut = 0.1

TRIGGER_COMMAND = {
    "IDLE": 0,
    "START_ABS_MOVE": 1,
    "START_REL_MOVE": 2,  # <--- This is the new one for relative moves
    "START_HOMING": 3,
    "STOP_MOVE": 4,
    "PAUSE_MOVE": 5,
    "RESUME_MOVE": 6,
}


def setBaudRate(setrate=1):
    rates = {1: 9600, 2: 19200, 3: 25000, 4: 38400, 5: 57600, 6: 115200, 7: 230400}
    print(f"Set Baud Rate to {rates[setrate]}")
    client.write_register(address=0x008A, value=setrate, device_id=slaveAddress, no_response_expected=True)


def setAddress(address):
    print(f"Set Address to {address}")
    client.write_register(address=0x008B, value=address, device_id=slaveAddress, no_response_expected=True)


# result = client.read_input_registers(address=0x1147, count=19, slave=0x01)
def enableMotor():
    print("enabling motor")
    result = client.write_register(address=0x00F3, value=1, device_id=slaveAddress)
    print(f"Result: {result}")
    time.sleep(1)
    print("read enable motor")
    result = client.read_input_registers(address=0x3A, count=1, device_id=slaveAddress)
    print(f"Enable motor result: {result.registers}")
    # time.sleep(commandTimeOut)


def disableMotor():
    print("disabling motor")
    result = client.write_register(address=0x00F3, value=0, device_id=slaveAddress)
    print(f"Result: {result}")
    print("read disable motor")
    result = client.read_input_registers(address=0x3A, count=1, device_id=slaveAddress)
    print(f"Enable disable result: {result.registers}")
    # time.sleep(commandTimeOut)


def readPulses():
    print("read number of pulses")
    result = client.read_input_registers(address=0x32, count=2, device_id=slaveAddress)
    print(f"Number of pulses result: {result.registers}")


def scanBus():
    print("Scanning Modbus bus...")
    for i in range(1, 255):
        print(f"Checking slave {i}...")
        try:
            # slave_id = struct.pack("B", i)
            slave_id = int(i)
            response = client.read_input_registers(address=0x32, count=2, device_id=slave_id)
            print(f"Found Modbus device at address {i} aka {hex(i)} response: {response}")
        except Exception as e:
            pass


def calibrateMotor():
    print("calibrate motor")
    result = client.write_register(address=0x0080, value=1, device_id=slaveAddress)
    print(f"Result: {result}")
    time.sleep(3)


def setZero():
    print("set motor zero")
    result = client.write_register(address=0x0092, value=1, device_id=slaveAddress)
    print(f"Result: {result}")


def readMotorStatus():
    print("read motor status")
    result = client.read_input_registers(address=0xF1, count=1, device_id=slaveAddress)
    case = result.registers[0]
    if case == 0:
        print("Motor status: Read Failed")
    elif case == 1:
        print("Motor status: Stopped")
    elif case == 2:
        print("Motor status: Accelerating")
    elif case == 3:
        print("Motor status: Decelerating")
    elif case == 4:
        print("Motor status: Constant Speed")
    elif case == 5:
        print("Motor status: Homing")
    elif case == 6:
        print("Motor status: Calibrating")
    # print(f"Motor status result: {result.registers}")
    # time.sleep(commandTimeOut)


def readEncoderCarry():
    print("read encoder carry")
    result = client.read_input_registers(address=0x30, count=3, device_id=slaveAddress)
    print(f"Encoder carry result: {result.registers}")


def readIO():
    print("read IO")
    result = client.read_input_registers(address=0x34, count=1, device_id=slaveAddress)
    print(f"IO result: {result.registers}")


def readIOloop(reads):
    if reads > 0:
        for i in range(reads):
            readIO()
            time.sleep(0.1)
    else:
        while True:
            readIO()
            time.sleep(0.1)


def readEndstopSetting():
    print("read endstop setting")
    result = client.read_input_registers(address=0x3B, count=1, device_id=slaveAddress)
    print(f"Endstop setting result: {result.registers}")


def remapIOhoming():
    print("remap endstop IO")
    endstop_active_level = 0  # 1 for active low, 0 for active high
    print(f"set endstop active level to {endstop_active_level}")

    result = client.write_register(address=0x85, value=endstop_active_level, device_id=slaveAddress)
    print(f"Result: {result}")
    # set direction and trigger level
    dir = 0
    trigger_level = 1
    value = (dir << 8) | trigger_level
    print(f"Remapping IO with value: {value} (dir: {dir}, trigger_level: {trigger_level})")
    result = client.write_register(address=0x90, value=value, device_id=slaveAddress)
    print(f"Result: {result}")
    # enable endstop
    result = client.write_register(address=0x92, value=1, device_id=slaveAddress)
    print(f"Result: {result}")

    # time.sleep(1)


def readEN():
    print("read EN")
    result = client.read_input_registers(address=0x3A, count=1, device_id=slaveAddress)
    print(f"EN result: {result.registers}")


def readENloop(reads):
    if reads > 0:
        for i in range(reads):
            readEN()
            time.sleep(0.1)
    else:
        while True:
            readEN()
            time.sleep(0.1)


def readEncoderAddition():
    print("read encoder addition")
    result = client.read_input_registers(address=0x31, count=3, device_id=slaveAddress)
    print(f"Encoder addition result: {result.registers}")


def restartMotor():
    print("restart motor")
    result = client.write_register(address=0x41, value=1, device_id=slaveAddress)
    print(f"Result: {result}")
    # time.sleep(commandTimeOut)


def setMotorHome(stopOnEndstop, direction, speed, enableEndstop):
    print(
        f"set motor home paramteters to: stopOnEndstop: {stopOnEndstop}, direction: {direction}, speed: {speed}, enableEndstop {enableEndstop}"
    )
    data = [stopOnEndstop, direction, speed, enableEndstop]
    # first parameter means stopping when endstop pulled low
    # second  means homing direction
    # third is speed
    # last means enable endstop
    payload = client.convert_to_registers(data, client.DATATYPE.INT16)
    result = client.write_registers(0x90, payload, device_id=slaveAddress)
    print(f"Result: {result}")


def moveMotorHome():
    print("move motor home")
    response = client.write_register(0x91, 1, device_id=slaveAddress)
    print(f"Response: {response}")


def moveMotorSpeed(direction, acceleration, speed):
    print(f"----move motor speed using direction: {direction}, acceleration: {acceleration}, speed: {speed}----")
    dir_acc = (int(direction) & 0xFF) << 8 | (int(acceleration) & 0xFF)
    speed_high = (int(speed) >> 8) & 0xFF  # High byte of speed
    speed_low = int(speed) & 0xFF  # Low byte of speed
    data = [dir_acc, (speed_high << 8) | speed_low]

    print(f"Packed data to send (register values): {data}")
    try:
        # Write data to the motor's registers
        response = client.write_registers(0xF6, data, device_id=slaveAddress)
        print(f"Response: {response}")
    except ModbusException as e:
        print(f"Error: {e}")


def stopMotorSpeed():
    print("stop motor")
    direction = 0
    acceleration = 100
    speed = 0
    dir_acc = (int(direction) & 0xFF) << 8 | (int(acceleration) & 0xFF)
    speed_high = (int(speed) >> 8) & 0xFF  # High byte of speed
    speed_low = int(speed) & 0xFF  # Low byte of speed
    data = [dir_acc, (speed_high << 8) | speed_low]
    payload = client.convert_to_registers(data, client.DATATYPE.INT16)
    response = client.write_registers(0xF6, payload, device_id=slaveAddress)
    print(f"Response: {response}")


def eStop():
    print("E stop")
    response = client.write_register(0x00F7, 0, device_id=slaveAddress)
    print(f"Response: {response}")


def setMicrosteps(microsteps):
    print("set microsteps to: ", microsteps)
    response = client.write_register(0x84, microsteps, device_id=slaveAddress)
    print(f"Response: {response}")
    # time.sleep(commandTimeOut)


def setHoldCurrent(holdCurrent):
    print("set hold current to: ", holdCurrent)
    if holdCurrent < 10:
        print("hold current too low, setting to 10")
        holdCurrent = 10
    if holdCurrent > 100:
        print("hold current too high, setting to 100")
        holdCurrent = 100
    holdCurrent = holdCurrent - 10
    writeCurrent = int(holdCurrent / 10)
    print("write hold current to: ", writeCurrent)
    response = client.write_register(0x9B, writeCurrent, device_id=slaveAddress)
    print(f"Response: {response}")


def setRunCurrent(runCurrent):
    print("set run current to: ", runCurrent)
    if runCurrent < 10:
        print("run current too low, setting to 10")
        runCurrent = 10
    if runCurrent > 5200:
        print("run current too high, setting to 5200")
        runCurrent = 5200
    response = client.write_register(0x83, runCurrent, device_id=slaveAddress)
    print(f"Response: {response}")


def moveMotorPosition(direction, acceleration, speed, steps):
    print(
        f"----move motor to position using direction: {direction}, acceleration: {acceleration}, speed: {speed}, steps: {steps}----"
    )
    direction_bit_value = 1 if direction != 0 else 0  # Force direction to be 0 or 1
    control_config_reg = (direction_bit_value << 8) | (int(acceleration) & 0xFF)
    data = [
        control_config_reg,  # 1 register: direction and acceleration
        int(speed) & 0xFFFF,  # 1 register: speed
        (int(steps) >> 16) & 0xFFFF,  # 1 register: steps (high word)
        int(steps) & 0xFFFF,  # 1 register: steps (low word)
    ]
    print("Data:", data)
    try:
        response = client.write_registers(0xFD, data, device_id=slaveAddress)
        print(f"Response: {response}")
        print(f"Response code: {response.function_code}")
    except ModbusException as e:
        print(f"Error: {e}")


def moveMotorRelative(direction, acceleration, speed, distance):
    """
    Moves the motor a relative distance.

    Args:
        client: pymodbus client object.
        slaveAddress: Modbus slave address of the motor.
        direction: 0 for positive relative move, 1 for negative relative move.
        acceleration: 0-200.
        speed: 0-200000 pulses/s.
        distance: Relative distance to move in pulses (always a positive value).
    """
    print(
        f"---- Moving motor a relative distance ----\n"
        f"  Direction: {'Negative' if direction == 1 else 'Positive'}\n"
        f"  Acceleration: {acceleration}\n"
        f"  Speed: {speed} pulses/s\n"
        f"  Relative Distance: {distance} pulses"
    )

    # 1. Register 0x00FC (Control/Config)
    # Bit 8: Direction (0=positive, 1=negative)
    # Bits 0-7: Acceleration (0-200)
    REG_RELATIVE_MOVE_START = 0x00FC
    REG_TRIGGER_COMMAND = 0x0113
    direction_bit_value = 1 if direction != 0 else 0  # Force direction to be 0 or 1
    control_config_reg = (direction_bit_value << 8) | (int(acceleration) & 0xFF)

    # 2. Register 0x00FD (Speed) - Note: This register is 0x00FD when starting from 0x00FC
    speed_reg = int(speed) & 0xFFFF  # Ensure it fits in 16 bits

    # 3. Registers 0x00FE and 0x00FF (Relative Distance - 32-bit unsigned value)
    # The manual shows 0x00FC (control), 0x00FD (speed), 0x00FE (distance high), 0x00FF (distance low)
    # For relative distance, the value itself should typically be positive.
    distance_val = abs(int(distance))  # Ensure distance is positive for relative moves

    distance_high_word = (distance_val >> 16) & 0xFFFF
    distance_low_word = distance_val & 0xFFFF

    # Data list for write_registers
    data = [
        control_config_reg,  # 0x00FC
        speed_reg,  # 0x00FD (This is the speed register relative to 0x00FC)
        distance_high_word,  # 0x00FE
        distance_low_word,  # 0x00FF
    ]

    print(f"Data to write (decimal): {data}")
    print(f"Data to write (hex): {[f'0x{val:04X}' for val in data]}")

    try:
        # Write 4 registers starting from 0x00FC
        response = client.write_registers(REG_RELATIVE_MOVE_START, data, device_id=slaveAddress)

        if not response.is_exception():
            print("Relative move parameters sent successfully.")
            # Now trigger the relative move
            trigger_response = client.write_register(
                REG_TRIGGER_COMMAND, TRIGGER_COMMAND["START_REL_MOVE"], device_id=slaveAddress
            )
            if not trigger_response.is_exception():
                print("Relative move command triggered successfully. Motor should now be moving.")
                return True
            else:
                print(f"Modbus Error triggering relative move: {trigger_response}")
                return False
        else:
            print(f"Modbus Error sending relative move parameters: {response}")
            return False
    except ModbusException as e:
        print(f"Communication error during relative motor move: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred during relative motor move: {e}")
        return False


def setWorkMode(workMode=5):
    print("set work mode to: ", workMode)
    if workMode == 0:
        print("setting work mode to 0, open loop pulse mode")
    elif workMode == 1:
        print("setting work mode to 1, closed loop pulse mode")
    elif workMode == 2:
        print("setting work mode to 2, closed loop FOC mode")
    elif workMode == 3:
        print("setting work mode to 3, open loop serial mode")
    elif workMode == 4:
        print("setting work mode to 4, closed loop serial mode")
    elif workMode == 5:
        print("setting work mode to 5, closed loop FOC serial mode")
    response = client.write_register(0x82, workMode, device_id=slaveAddress)
    print(f"Response: {response}")
    # time.sleep(commandTimeOut)


def sleepFor(seconds):
    for i in range(seconds):
        print(f"Sleeping for {seconds} seconds")
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description="Modbus Motor Test Utility")
    parser.add_argument("-p", "--port", type=str, default="COM9", help="Serial port (e.g., COM9 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("-a", "--address", type=int, default=1, help="Modbus slave address (default: 1)")
    args = parser.parse_args()

    global client, slaveAddress, baudRate
    baudRate = args.baud
    slaveAddress = args.address

    client = ModbusSerialClient(
        port=args.port,
        baudrate=baudRate,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=0.1,
        retries=1,
    )
    if not client.connect():
        print("Error connecting to RS485 port")
        return
    # setAddress(1)
    # setBaudRate(6)
    # exit()

    microsteps = 16
    enableMotor()
    setWorkMode(5)
    setZero()
    setMicrosteps(microsteps)
    setRunCurrent(3000)
    readEncoderCarry()
    moveMotorPosition(direction=0, acceleration=1, speed=50, steps=200 * microsteps)
    # moveMotorRelative(direction=1, acceleration=1, speed=100, distance=300 * microsteps)
    time.sleep(2)
    disableMotor()
    # setAddress(6)
    exit()
    readEncoderAddition()
    enableMotor()
    moveMotorPosition(direction=1, acceleration=1, speed=100, steps=300 * microsteps)
    # time.sleep(2)
    # moveMotorSpeed(direction=0, acceleration=127, speed=1000)
    time.sleep(20)
    # moveMotorSpeed(direction=0, acceleration=127, speed=0)
    # time.sleep(20)
    readEncoderCarry()
    readEncoderAddition()
    time.sleep(0.1)
    # disableMotor()
    # Additional tests can be added here as needed


if __name__ == "__main__":
    main()

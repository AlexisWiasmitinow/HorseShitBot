from dataclasses import dataclass
import struct
import time
from typing import Dict, List, Optional

try:
    import serial
except ImportError:
    serial = None


class ModbusError(RuntimeError):
    pass


class ModbusTimeout(ModbusError):
    pass


class ModbusCrcError(ModbusError):
    pass


class ModbusExceptionResponse(ModbusError):
    def __init__(self, function: int, exception_code: int):
        super().__init__(
            f"Modbus exception response: "
            f"function=0x{function:02X}, exception=0x{exception_code:02X}"
        )
        self.function = function
        self.exception_code = exception_code


def crc16_modbus(data: bytes) -> int:
    """
    CRC-16/MODBUS.

    Modbus RTU transmits CRC low byte first.
    """
    crc = 0xFFFF

    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def add_crc(frame_without_crc: bytes) -> bytes:
    crc = crc16_modbus(frame_without_crc)
    return frame_without_crc + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False

    received_crc = frame[-2] | (frame[-1] << 8)
    calculated_crc = crc16_modbus(frame[:-2])

    return received_crc == calculated_crc


@dataclass
class AdcRegisterMap:
    # According to common N43VD04-style Modbus ADC documentation:
    # CH1..CH4 are usually holding registers 0x0000..0x0003.
    channel_base_register: int = 0x0000

    # Device configuration registers.
    address_register: int = 0x000E
    baudrate_register: int = 0x000F

    # Usually raw / 100 = voltage at ADC input.
    # Example: raw=1234 -> 12.34 V.
    raw_scale: float = 100.0

    # ADC_voltage = real_battery_voltage * voltage_divider_factor.
    #
    # For the final robot:
    # voltage_divider_factor = 0.5
    #
    # Therefore:
    # battery_voltage = adc_voltage / 0.5 = adc_voltage * 2.
    voltage_divider_factor: float = 1.0

    # Optional calibration hooks.
    voltage_offset: float = 0.0
    voltage_multiplier: float = 1.0


BAUD_TO_CODE: Dict[int, int] = {
    1200: 0,
    2400: 1,
    4800: 2,
    9600: 3,
    19200: 4,
}

CODE_TO_BAUD: Dict[int, int] = {v: k for k, v in BAUD_TO_CODE.items()}


class ModbusRtuClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        slave_id: int = 1,
        timeout: float = 0.25,
        parity: str = "N",
        bytesize: int = 8,
        stopbits: int = 1,
    ):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.parity = parity
        self.bytesize = bytesize
        self.stopbits = stopbits
        self._ser = None

    def connect(self) -> None:
        if serial is None:
            raise RuntimeError(
                "pyserial is not installed. Install it with: "
                "sudo apt install python3-serial"
            )

        if self._ser is not None and self._ser.is_open:
            return

        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )

        time.sleep(0.05)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def reopen(
        self,
        *,
        baudrate: Optional[int] = None,
        slave_id: Optional[int] = None,
    ) -> None:
        self.close()

        if baudrate is not None:
            self.baudrate = baudrate

        if slave_id is not None:
            self.slave_id = slave_id

        self.connect()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _rtu_silent_interval(self) -> float:
        # Modbus RTU requires at least 3.5 character times between frames.
        # Typical RTU frame: 11 bits per character.
        return max(0.004, 3.5 * 11.0 / float(self.baudrate))

    def _txrx(
        self,
        function: int,
        payload: bytes,
        expected_len: int,
        *,
        slave_id: Optional[int] = None,
        expect_response: bool = True,
    ) -> bytes:
        self.connect()

        slave = self.slave_id if slave_id is None else slave_id

        if not 0 <= slave <= 255:
            raise ValueError("slave_id must be 0..255")

        request = add_crc(bytes((slave, function)) + payload)

        self._ser.reset_input_buffer()

        time.sleep(self._rtu_silent_interval())
        self._ser.write(request)
        self._ser.flush()

        if not expect_response:
            time.sleep(self._rtu_silent_interval())
            return b""

        response = self._ser.read(expected_len)

        if len(response) < 5:
            raise ModbusTimeout(
                f"No/short response on {self.port} at {self.baudrate} baud, "
                f"slave {slave}. Got {len(response)} bytes: {response.hex(' ')}"
            )

        if not check_crc(response):
            raise ModbusCrcError(f"Bad CRC in response: {response.hex(' ')}")

        if response[0] != slave:
            raise ModbusError(
                f"Response slave id mismatch: expected {slave}, got {response[0]}"
            )

        if response[1] == (function | 0x80):
            raise ModbusExceptionResponse(function, response[2])

        if response[1] != function:
            raise ModbusError(
                f"Function mismatch: expected 0x{function:02X}, "
                f"got 0x{response[1]:02X}"
            )

        return response

    def read_holding_registers(
        self,
        address: int,
        count: int,
        *,
        slave_id: Optional[int] = None,
    ) -> List[int]:
        if not 1 <= count <= 125:
            raise ValueError("count must be 1..125")

        payload = struct.pack(">HH", address, count)
        response = self._txrx(
            0x03,
            payload,
            5 + 2 * count,
            slave_id=slave_id,
        )

        byte_count = response[2]

        if byte_count != 2 * count:
            raise ModbusError(
                f"Unexpected byte count: expected {2 * count}, got {byte_count}"
            )

        data = response[3:3 + byte_count]

        return list(struct.unpack(">" + "H" * count, data))

    def read_input_registers(
        self,
        address: int,
        count: int,
        *,
        slave_id: Optional[int] = None,
    ) -> List[int]:
        if not 1 <= count <= 125:
            raise ValueError("count must be 1..125")

        payload = struct.pack(">HH", address, count)
        response = self._txrx(
            0x04,
            payload,
            5 + 2 * count,
            slave_id=slave_id,
        )

        byte_count = response[2]

        if byte_count != 2 * count:
            raise ModbusError(
                f"Unexpected byte count: expected {2 * count}, got {byte_count}"
            )

        data = response[3:3 + byte_count]

        return list(struct.unpack(">" + "H" * count, data))

    def write_single_register(
        self,
        address: int,
        value: int,
        *,
        slave_id: Optional[int] = None,
        expect_response: bool = True,
    ) -> bool:
        if not 0 <= value <= 0xFFFF:
            raise ValueError("register value must fit in uint16")

        payload = struct.pack(">HH", address, value)

        response = self._txrx(
            0x06,
            payload,
            8,
            slave_id=slave_id,
            expect_response=expect_response,
        )

        if not expect_response:
            return True

        echoed_address, echoed_value = struct.unpack(">HH", response[2:6])

        if echoed_address != address or echoed_value != value:
            raise ModbusError(
                f"Write echo mismatch: wrote register "
                f"0x{address:04X}=0x{value:04X}, got "
                f"0x{echoed_address:04X}=0x{echoed_value:04X}"
            )

        return True


class N43VD04:
    def __init__(
        self,
        client: ModbusRtuClient,
        register_map: Optional[AdcRegisterMap] = None,
    ):
        self.client = client
        self.map = register_map or AdcRegisterMap()

    def read_channel_raw(self, channel: int) -> int:
        if channel not in (1, 2, 3, 4):
            raise ValueError("channel must be 1, 2, 3, or 4")

        register = self.map.channel_base_register + (channel - 1)

        return self.client.read_holding_registers(register, 1)[0]

    def raw_to_adc_voltage(self, raw: int) -> float:
        adc_voltage = raw / self.map.raw_scale
        adc_voltage = adc_voltage * self.map.voltage_multiplier
        adc_voltage = adc_voltage + self.map.voltage_offset

        return adc_voltage

    def adc_to_battery_voltage(self, adc_voltage: float) -> float:
        if self.map.voltage_divider_factor <= 0:
            raise ValueError("voltage_divider_factor must be > 0")

        return adc_voltage / self.map.voltage_divider_factor

    def read_voltage(self, channel: int) -> dict:
        raw = self.read_channel_raw(channel)
        adc_voltage = self.raw_to_adc_voltage(raw)
        battery_voltage = self.adc_to_battery_voltage(adc_voltage)

        return {
            "channel": channel,
            "raw": raw,
            "adc_voltage": adc_voltage,
            "battery_voltage": battery_voltage,
        }

    def read_all_voltages(self) -> List[dict]:
        return [self.read_voltage(ch) for ch in (1, 2, 3, 4)]

    def read_slave_id(self, *, use_broadcast: bool = False) -> int:
        # Some of these modules document 0xFF as broadcast/read-any address.
        # Use only with exactly one ADC connected.
        sid = 0xFF if use_broadcast else self.client.slave_id

        return self.client.read_holding_registers(
            self.map.address_register,
            1,
            slave_id=sid,
        )[0]

    def set_slave_id(self, new_slave_id: int) -> None:
        if not 1 <= new_slave_id <= 247:
            raise ValueError("new_slave_id must be 1..247")

        self.client.write_single_register(
            self.map.address_register,
            new_slave_id,
        )

        # Many modules apply the new slave ID immediately.
        self.client.slave_id = new_slave_id

    def read_baudrate_code(self) -> int:
        return self.client.read_holding_registers(
            self.map.baudrate_register,
            1,
        )[0]

    def read_baudrate(self) -> Optional[int]:
        return CODE_TO_BAUD.get(self.read_baudrate_code())

    def set_baudrate(self, new_baudrate: int) -> int:
        if new_baudrate not in BAUD_TO_CODE:
            raise ValueError(
                f"Unsupported baudrate {new_baudrate}; "
                f"supported: {sorted(BAUD_TO_CODE)}"
            )

        code = BAUD_TO_CODE[new_baudrate]

        self.client.write_single_register(
            self.map.baudrate_register,
            code,
        )

        return code
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.pymodbus_compat import (  # noqa: E402
    RTU_FRAMER,
    call_with_device,
    select_rtu_framer,
)
from horseshitbot.drivers import mks_bus  # noqa: E402
from horseshitbot.drivers.mks_bus import validate_modbus_response  # noqa: E402
from robot_web.core import bus as legacy_bus  # noqa: E402
from robot_web.core import pymodbus_compat as legacy_compat  # noqa: E402


class OldFramer:
    RTU = "old-rtu"


class NewFramer:
    RTU = "new-rtu"


class PymodbusCompatTest(unittest.TestCase):
    def test_pymodbus_36_uses_framer_rtu(self):
        module = SimpleNamespace(Framer=OldFramer)
        self.assertEqual(select_rtu_framer(module), "old-rtu")

    def test_new_pymodbus_uses_framer_type_rtu(self):
        module = SimpleNamespace(FramerType=NewFramer, Framer=OldFramer)
        self.assertEqual(select_rtu_framer(module), "new-rtu")

    def test_pymodbus_36_call_uses_slave_without_changing_payload(self):
        received = {}

        def write_register(*, address, value, slave):
            received.update(address=address, value=value, slave=slave)
            return "response"

        result = call_with_device(
            write_register,
            7,
            address=0x00F6,
            value=0x1234,
        )

        self.assertEqual(result, "response")
        self.assertEqual(
            received,
            {"address": 0x00F6, "value": 0x1234, "slave": 7},
        )

    def test_new_pymodbus_call_uses_device_id_without_changing_payload(self):
        received = {}
        values = [0x00FF, 1200]

        def write_registers(*, address, values, device_id):
            received.update(
                address=address,
                values=values,
                device_id=device_id,
            )
            return "response"

        result = call_with_device(
            write_registers,
            2,
            address=0x00F6,
            values=values,
        )

        self.assertEqual(result, "response")
        self.assertEqual(
            received,
            {"address": 0x00F6, "values": values, "device_id": 2},
        )

    def test_read_arguments_are_unchanged(self):
        received = {}

        def read_input_registers(*, address, count, slave):
            received.update(address=address, count=count, slave=slave)
            return "response"

        result = call_with_device(
            read_input_registers,
            4,
            address=0x003A,
            count=3,
        )

        self.assertEqual(result, "response")
        self.assertEqual(
            received,
            {"address": 0x003A, "count": 3, "slave": 4},
        )

    def test_mks_bus_constructor_preserves_serial_configuration(self):
        received = {}

        class FakeClient:
            def __init__(self, **kwargs):
                received.update(kwargs)

        original = mks_bus.ModbusSerialClient
        mks_bus.ModbusSerialClient = FakeClient
        try:
            mks_bus.MksBus(
                mks_bus.BusCfg(
                    port="/dev/test-mks",
                    baud=115200,
                    timeout=0.1,
                    retries=2,
                )
            )
        finally:
            mks_bus.ModbusSerialClient = original

        self.assertEqual(
            received,
            {
                "port": "/dev/test-mks",
                "framer": RTU_FRAMER,
                "baudrate": 115200,
                "bytesize": 8,
                "parity": "N",
                "stopbits": 1,
                "timeout": 0.1,
                "retries": 2,
            },
        )

    def test_all_mks_bus_register_calls_use_compatible_addressing(self):
        calls = []

        class Response:
            def __init__(
                self,
                registers=None,
                address=None,
                value=None,
                count=None,
            ):
                self.registers = registers or []
                self.address = address
                self.value = value
                self.count = count

            @staticmethod
            def isError():
                return False

        class FakeClient:
            connected = True
            retries = 2
            timeout = 0.1

            def write_register(self, *, address, value, slave):
                calls.append(("write_register", address, value, slave))
                return Response(address=address, value=value)

            def write_registers(self, *, address, values, slave):
                calls.append(("write_registers", address, values, slave))
                return Response(address=address, count=len(values))

            def read_holding_registers(self, *, address, count, slave):
                calls.append(("read_holding", address, count, slave))
                return Response(registers=[123, 456])

            def read_input_registers(self, *, address, count, slave):
                calls.append(("read_input", address, count, slave))
                return Response(registers=[123, 456])

        bus = object.__new__(mks_bus.MksBus)
        bus.cfg = mks_bus.BusCfg(port="/dev/not-opened", inter_delay=0.0)
        bus.lock = threading.Lock()
        bus.client = FakeClient()

        bus.write_reg(1, 0x0082, 4)
        bus.write_regs(2, 0x00F6, [0x00FF, 1200])
        self.assertEqual(bus.read_regs(3, 0x0083, 1), [123, 456])
        self.assertEqual(bus.read_input_regs(4, 0x003A, 2), [123, 456])
        self.assertTrue(bus.probe(5))

        self.assertEqual(
            calls,
            [
                ("write_register", 0x0082, 4, 1),
                ("write_registers", 0x00F6, [0x00FF, 1200], 2),
                ("read_holding", 0x0083, 1, 3),
                ("read_input", 0x003A, 2, 4),
                ("read_input", 0x003A, 1, 5),
            ],
        )

    def test_modbus_response_validation_accepts_success(self):
        response = SimpleNamespace(
            isError=lambda: False,
            registers=[10, 20],
        )
        self.assertIs(
            validate_modbus_response(response, "read", expected_registers=2),
            response,
        )

    def test_modbus_response_validation_rejects_error_response(self):
        response = SimpleNamespace(isError=lambda: True)
        with self.assertRaisesRegex(RuntimeError, "write"):
            validate_modbus_response(response, "write")

    def test_modbus_response_validation_rejects_is_error_exception(self):
        def raise_error():
            raise RuntimeError("cannot inspect response")

        response = SimpleNamespace(isError=raise_error)
        with self.assertRaisesRegex(RuntimeError, "cannot inspect response"):
            validate_modbus_response(response, "write")

    def test_modbus_response_validation_rejects_none(self):
        with self.assertRaisesRegex(RuntimeError, "no response"):
            validate_modbus_response(None, "write")

    def test_modbus_response_validation_rejects_invalid_response(self):
        with self.assertRaisesRegex(RuntimeError, "without isError"):
            validate_modbus_response(object(), "write")

    def test_modbus_response_validation_requires_read_registers(self):
        missing = SimpleNamespace(isError=lambda: False)
        short = SimpleNamespace(isError=lambda: False, registers=[1])
        with self.assertRaisesRegex(RuntimeError, "no valid registers"):
            validate_modbus_response(missing, "read", expected_registers=1)
        with self.assertRaisesRegex(RuntimeError, "expected 2 registers"):
            validate_modbus_response(short, "read", expected_registers=2)

    def test_modbus_response_validation_rejects_wrong_write_echo(self):
        single = SimpleNamespace(
            isError=lambda: False,
            address=0x0082,
            value=4,
        )
        multiple = SimpleNamespace(
            isError=lambda: False,
            address=0x00F6,
            count=1,
        )
        with self.assertRaisesRegex(RuntimeError, "address mismatch"):
            validate_modbus_response(
                single,
                "write",
                expected_address=0x0083,
                expected_value=4,
            )
        with self.assertRaisesRegex(RuntimeError, "count mismatch"):
            validate_modbus_response(
                multiple,
                "write",
                expected_address=0x00F6,
                expected_write_count=2,
            )

    def test_mks_bus_rejects_exception_from_modbus_call(self):
        class FakeClient:
            connected = True

            @staticmethod
            def close():
                pass

            @staticmethod
            def connect():
                return True

        bus = object.__new__(mks_bus.MksBus)
        bus.cfg = mks_bus.BusCfg(port="/dev/not-opened", inter_delay=0.0)
        bus.lock = threading.Lock()
        bus.client = FakeClient()

        def failing_call():
            raise OSError("serial failure")

        with self.assertRaisesRegex(RuntimeError, "serial failure"):
            bus._retry(failing_call, "write")

    def test_speed_write_rejects_modbus_error_and_none(self):
        class Response:
            def __init__(self, error):
                self.error = error

            def isError(self):
                return self.error

        class FakeClient:
            connected = True

            def __init__(self, responses):
                self.responses = list(responses)

            def write_registers(self, *, address, values, slave):
                return self.responses.pop(0)

            @staticmethod
            def close():
                pass

            @staticmethod
            def connect():
                return True

        for responses in (
            [Response(True), Response(True)],
            [None, None],
        ):
            with self.subTest(responses=responses):
                bus = object.__new__(mks_bus.MksBus)
                bus.cfg = mks_bus.BusCfg(
                    port="/dev/not-opened",
                    inter_delay=0.0,
                )
                bus.lock = threading.Lock()
                bus.client = FakeClient(responses)

                with self.assertRaises(RuntimeError):
                    bus.set_speed_signed(1, 500, acc=255)

    def test_speed_write_accepts_valid_response_and_preserves_payload(self):
        calls = []

        class Response:
            address = mks_bus.REG_SPEED
            count = 2

            @staticmethod
            def isError():
                return False

        class FakeClient:
            connected = True

            def write_registers(self, *, address, values, slave):
                calls.append((address, values, slave))
                return Response()

        bus = object.__new__(mks_bus.MksBus)
        bus.cfg = mks_bus.BusCfg(port="/dev/not-opened", inter_delay=0.0)
        bus.lock = threading.Lock()
        bus.client = FakeClient()

        bus.set_speed_signed(2, -500, acc=255, invert_dir=False)

        self.assertEqual(calls, [(mks_bus.REG_SPEED, [0x01FF, 500], 2)])

    def test_legacy_bus_defers_serial_client_creation_until_connect(self):
        received = {}

        class FakeClient:
            def __init__(self, **kwargs):
                received.update(kwargs)

        original = legacy_bus.ModbusSerialClient
        legacy_bus.ModbusSerialClient = FakeClient
        try:
            bus = legacy_bus.MksBus(
                legacy_bus.BusCfg(
                    port="/dev/legacy-test",
                    baud=38400,
                    timeout=0.35,
                )
            )
            self.assertIsNone(bus.client)
            bus.client = bus._new_client()
        finally:
            legacy_bus.ModbusSerialClient = original

        self.assertEqual(
            received,
            {
                "port": "/dev/legacy-test",
                "framer": RTU_FRAMER,
                "baudrate": 38400,
                "bytesize": 8,
                "parity": "N",
                "stopbits": 1,
                "timeout": 0.35,
                "retries": 0,
            },
        )

    def test_legacy_compat_uses_pymodbus_36_api(self):
        received = {}

        def write_register(*, address, value, slave):
            received.update(address=address, value=value, slave=slave)

        module = SimpleNamespace(Framer=SimpleNamespace(RTU="legacy-rtu"))
        self.assertEqual(legacy_compat.select_rtu_framer(module), "legacy-rtu")
        legacy_compat.call_with_device(
            write_register,
            7,
            address=0x00F3,
            value=1,
        )
        self.assertEqual(
            received,
            {"address": 0x00F3, "value": 1, "slave": 7},
        )

    def test_legacy_compat_uses_new_pymodbus_api(self):
        received = {}
        values = [0x00FF, 1200]

        def write_registers(*, address, values, device_id):
            received.update(
                address=address,
                values=values,
                device_id=device_id,
            )

        module = SimpleNamespace(FramerType=SimpleNamespace(RTU="new-rtu"))
        self.assertEqual(legacy_compat.select_rtu_framer(module), "new-rtu")
        legacy_compat.call_with_device(
            write_registers,
            8,
            address=0x00F6,
            values=values,
        )
        self.assertEqual(
            received,
            {
                "address": 0x00F6,
                "values": [0x00FF, 1200],
                "device_id": 8,
            },
        )


if __name__ == "__main__":
    unittest.main()

import threading
import unittest
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.mks_bus import (  # noqa: E402
    BusCfg,
    MksBus,
    ModbusBusBusy,
)
from horseshitbot.drivers.modbus_adc import (  # noqa: E402
    AdcRegisterMap,
    BAUD_TO_CODE,
)


class Response:
    address = 0

    def __init__(self, registers=None, error=False):
        self.registers = registers or []
        self._error = error

    def isError(self):
        return self._error


class SharedModbusTest(unittest.TestCase):
    def make_bus(self, client):
        bus = object.__new__(MksBus)
        bus.cfg = BusCfg(
            port="/dev/not-opened",
            baud=19200,
            timeout=0.1,
            retries=2,
            inter_delay=0.0,
        )
        bus.lock = threading.Lock()
        bus.client = client
        return bus

    def test_low_priority_read_fails_fast_when_motor_owns_lock(self):
        class Client:
            connected = True
            retries = 2
            timeout = 0.1

        bus = self.make_bus(Client())
        bus.lock.acquire()
        try:
            with self.assertRaises(ModbusBusBusy):
                bus.read_regs_once_if_idle(7, 0, timeout=0.03)
        finally:
            bus.lock.release()

    def test_low_priority_read_has_no_retry_and_restores_client_settings(self):
        observations = []

        class Client:
            connected = True
            retries = 2
            timeout = 0.1

            def read_holding_registers(self, *, address, count, slave):
                observations.append(
                    (address, count, slave, self.retries, self.timeout)
                )
                return Response([1234])

        client = Client()
        bus = self.make_bus(client)
        self.assertEqual(
            bus.read_regs_once_if_idle(7, 0, timeout=0.03),
            [1234],
        )
        self.assertEqual(observations, [(0, 1, 7, 0, 0.03)])
        self.assertEqual(client.retries, 2)
        self.assertEqual(client.timeout, 0.1)

    def test_low_priority_read_does_not_reconnect_or_retry_on_error(self):
        class Client:
            connected = True
            retries = 2
            timeout = 0.1

            def __init__(self):
                self.calls = 0
                self.connect_calls = 0

            def read_holding_registers(self, *, address, count, device_id):
                self.calls += 1
                return Response(error=True)

            def connect(self):
                self.connect_calls += 1

        client = Client()
        bus = self.make_bus(client)
        with self.assertRaises(RuntimeError):
            bus.read_regs_once_if_idle(7, 0, timeout=0.03)
        self.assertEqual(client.calls, 1)
        self.assertEqual(client.connect_calls, 0)
        self.assertEqual(client.retries, 2)
        self.assertEqual(client.timeout, 0.1)

    def test_adc_conversion_and_final_defaults(self):
        mapping = AdcRegisterMap(voltage_divider_factor=0.5)
        reading = mapping.convert(1234, channel=1)
        self.assertEqual(mapping.channel_register(1), 0)
        self.assertAlmostEqual(reading["adc_voltage"], 12.34)
        self.assertAlmostEqual(reading["battery_voltage"], 24.68)
        self.assertEqual(BAUD_TO_CODE[19200], 4)
        self.assertEqual(BusCfg("/dev/mksbus").baud, 19200)

    def test_battery_runtime_has_no_serial_client(self):
        source = (
            PACKAGE_ROOT
            / "horseshitbot"
            / "nodes"
            / "battery_modbus_node.py"
        ).read_text()
        self.assertNotIn("ModbusSerialClient", source)
        self.assertNotIn("serial.Serial", source)
        self.assertIn('"/modbus/read_holding_register"', source)


if __name__ == "__main__":
    unittest.main()

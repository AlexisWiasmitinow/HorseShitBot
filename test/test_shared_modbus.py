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
    LowVoltageMonitor,
    raw_to_voltage,
    voltage_to_raw,
)

# Bench measurements taken on the robot's battery divider (2026-09-25).
CALIBRATION_POINTS = (
    (20.0, 970),
    (22.0, 1068),
    (24.0, 1167),
    (26.0, 1265),
    (28.0, 1364),
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
        mapping = AdcRegisterMap()
        reading = mapping.convert(1364, channel=1)
        self.assertEqual(mapping.channel_register(1), 0)
        self.assertEqual(reading["raw"], 1364)
        self.assertAlmostEqual(reading["battery_voltage"], 28.004, places=3)
        self.assertEqual(BAUD_TO_CODE[19200], 4)
        self.assertEqual(BusCfg("/dev/mksbus").baud, 19200)

    def test_calibration_reproduces_every_measured_point(self):
        for expected_voltage, raw in CALIBRATION_POINTS:
            self.assertAlmostEqual(
                raw_to_voltage(raw), expected_voltage, delta=0.01
            )
            self.assertAlmostEqual(
                voltage_to_raw(expected_voltage), raw, delta=1.0
            )

    def test_low_warning_starts_at_threshold_without_shutdown(self):
        monitor = LowVoltageMonitor()
        self.assertEqual(monitor.update(22.1, 0.0).state, "normal")
        status = monitor.update(22.0, 1.0)
        self.assertEqual(status.state, "low")
        self.assertTrue(status.low)
        self.assertFalse(status.critical)
        self.assertFalse(status.shutdown_due)

    def test_shutdown_requires_the_full_critical_hold_time(self):
        monitor = LowVoltageMonitor(critical_hold_sec=5.0)
        self.assertFalse(monitor.update(20.0, 100.0).shutdown_due)
        self.assertFalse(monitor.update(19.5, 104.9).shutdown_due)
        status = monitor.update(19.5, 105.0)
        self.assertTrue(status.shutdown_due)
        self.assertAlmostEqual(status.critical_for_sec, 5.0)

    def test_recovery_and_read_failure_restart_the_critical_hold(self):
        monitor = LowVoltageMonitor(critical_hold_sec=5.0)
        monitor.update(19.8, 0.0)
        # Load sag that recovers must not accumulate toward a shutdown.
        self.assertEqual(monitor.update(23.0, 3.0).state, "normal")
        self.assertFalse(monitor.update(19.8, 4.0).shutdown_due)

        monitor.update(19.8, 5.0)
        # A dropped ADC read is not a critical sample either.
        monitor.reset()
        self.assertFalse(monitor.update(19.8, 8.0).shutdown_due)
        self.assertTrue(monitor.update(19.8, 13.0).shutdown_due)

    def test_shutdown_is_requested_only_once(self):
        monitor = LowVoltageMonitor(critical_hold_sec=1.0)
        monitor.update(19.0, 0.0)
        self.assertTrue(monitor.update(19.0, 1.0).shutdown_due)
        self.assertTrue(monitor.shutdown_latched)
        self.assertFalse(monitor.update(19.0, 2.0).shutdown_due)
        self.assertFalse(monitor.update(19.0, 60.0).shutdown_due)
        # Even a full recovery followed by another critical spell stays latched.
        monitor.update(26.0, 61.0)
        monitor.update(19.0, 62.0)
        self.assertFalse(monitor.update(19.0, 70.0).shutdown_due)

    def test_monitor_rejects_inconsistent_thresholds(self):
        with self.assertRaises(ValueError):
            LowVoltageMonitor(low_voltage=20.0, critical_voltage=22.0)

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

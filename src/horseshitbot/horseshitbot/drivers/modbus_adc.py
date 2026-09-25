"""Transport-free helpers for the N43IC04 Modbus acquisition module.

ROS serial ownership deliberately does not live here. The bus node reads one
raw register and battery_modbus_node uses this module only for conversion.
"""

from __future__ import annotations

from dataclasses import dataclass


BAUD_TO_CODE = {
    1200: 0,
    2400: 1,
    4800: 2,
    9600: 3,
    19200: 4,
}
CODE_TO_BAUD = {code: baud for baud, code in BAUD_TO_CODE.items()}

# Battery divider calibration measured on the assembled robot (2026-09-25):
#
#   20 V -> 970    22 V -> 1068    24 V -> 1167    26 V -> 1265    28 V -> 1364
#
# The response is linear to within a fraction of a count, so the channel is
# converted with battery_voltage = (raw + RAW_OFFSET) / COUNTS_PER_VOLT.
# Every measured point reproduces to better than 0.005 V.
RAW_OFFSET_COUNTS = 15.2
COUNTS_PER_VOLT = 49.25


@dataclass(frozen=True)
class AdcRegisterMap:
    channel_base_register: int = 0x0000
    address_register: int = 0x00FD
    baudrate_register: int = 0x00FE
    raw_offset_counts: float = RAW_OFFSET_COUNTS
    counts_per_volt: float = COUNTS_PER_VOLT

    def channel_register(self, channel: int) -> int:
        if channel not in (1, 2, 3, 4):
            raise ValueError("channel must be 1, 2, 3, or 4")
        return self.channel_base_register + channel - 1

    def convert(self, raw: int, channel: int) -> dict:
        if not 0 <= int(raw) <= 0xFFFF:
            raise ValueError("raw ADC value must fit in uint16")
        if self.counts_per_volt <= 0:
            raise ValueError("counts_per_volt must be positive")

        return {
            "channel": int(channel),
            "raw": int(raw),
            "battery_voltage": raw_to_voltage(
                raw,
                raw_offset_counts=self.raw_offset_counts,
                counts_per_volt=self.counts_per_volt,
            ),
        }


def raw_to_voltage(
    raw: int,
    raw_offset_counts: float = RAW_OFFSET_COUNTS,
    counts_per_volt: float = COUNTS_PER_VOLT,
) -> float:
    """Convert a raw N43IC04 count into battery volts."""
    if counts_per_volt <= 0:
        raise ValueError("counts_per_volt must be positive")
    return (int(raw) + float(raw_offset_counts)) / float(counts_per_volt)


def voltage_to_raw(
    voltage: float,
    raw_offset_counts: float = RAW_OFFSET_COUNTS,
    counts_per_volt: float = COUNTS_PER_VOLT,
) -> float:
    """Inverse of raw_to_voltage, useful for logging thresholds in counts."""
    return float(voltage) * float(counts_per_volt) - float(raw_offset_counts)


NORMAL = "normal"
LOW = "low"
CRITICAL = "critical"


@dataclass
class LowVoltageStatus:
    state: str
    low: bool
    critical: bool
    critical_for_sec: float
    shutdown_due: bool


class LowVoltageMonitor:
    """Threshold/hold-time state machine for battery voltage.

    Kept free of ROS types so the shutdown decision can be unit tested. The
    critical timer is wall-clock based rather than sample based, so a slow or
    stalled poll loop cannot shorten the confirmation window.
    """

    def __init__(
        self,
        low_voltage: float = 22.0,
        critical_voltage: float = 20.0,
        critical_hold_sec: float = 5.0,
    ):
        if critical_voltage > low_voltage:
            raise ValueError("critical_voltage must not exceed low_voltage")
        if critical_hold_sec < 0:
            raise ValueError("critical_hold_sec must be non-negative")
        self.low_voltage = float(low_voltage)
        self.critical_voltage = float(critical_voltage)
        self.critical_hold_sec = float(critical_hold_sec)
        self._critical_since: float | None = None
        self._shutdown_latched = False

    @property
    def shutdown_latched(self) -> bool:
        return self._shutdown_latched

    def reset(self) -> None:
        """Forget the critical timer. Does not clear the shutdown latch."""
        self._critical_since = None

    def update(self, voltage: float, now: float) -> LowVoltageStatus:
        low = voltage <= self.low_voltage
        critical_sample = voltage <= self.critical_voltage

        if critical_sample:
            if self._critical_since is None:
                self._critical_since = now
            critical_for = max(0.0, now - self._critical_since)
        else:
            self._critical_since = None
            critical_for = 0.0

        confirmed = critical_sample and critical_for >= self.critical_hold_sec

        # Latch on the first confirmed shutdown so a recovering battery, a
        # reconnecting ADC or a bouncing reading can never re-request it.
        shutdown_due = confirmed and not self._shutdown_latched
        if shutdown_due:
            self._shutdown_latched = True

        if critical_sample:
            state = CRITICAL
        elif low:
            state = LOW
        else:
            state = NORMAL

        return LowVoltageStatus(
            state=state,
            low=low,
            critical=critical_sample,
            critical_for_sec=critical_for,
            shutdown_due=shutdown_due,
        )

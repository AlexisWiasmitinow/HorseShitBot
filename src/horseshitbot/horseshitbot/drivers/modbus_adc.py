"""Transport-free helpers for the N43IC04 Modbus acquisition module.

ROS serial ownership deliberately does not live here. The bus node reads one
raw register and battery_modbus_node uses this module only for conversion.

State of charge is an estimate derived from voltage alone; see
DEFAULT_SOC_VOLTAGE_POINTS. It is display telemetry. Low-voltage and shutdown
decisions are made by LowVoltageMonitor from the instantaneous calibrated
voltage and never from the smoothed value or the percentage.
"""

from __future__ import annotations

import math
from collections import deque
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


# Empirical HorseShitBot discharge calibration, 2026-09-25: a single 21.4 h
# run down to the BMS cutoff, sampled at 1 Hz. Percentages are the share of
# that run still remaining, so they assume a comparable load. Ordered from the
# top of the usable range downwards. Above the first voltage the pack is still
# shedding surface charge, which lasted only ~16 minutes, so that reads 100%;
# at or below the last voltage it reads 0%.
#
# No battery current or coulomb counting is available, so this cannot be
# corrected for load. The curve is very flat through its middle, where one ADC
# count is worth roughly two percentage points, which is why callers should
# smooth the voltage before looking it up.
DEFAULT_SOC_VOLTAGE_POINTS = (
    26.40, 26.20, 26.05, 25.95, 25.85, 25.70, 25.55,
    25.40, 25.20, 24.80, 24.00, 23.00, 22.00, 20.00,
)
DEFAULT_SOC_PERCENT_POINTS = (
    100.0, 70.8, 52.1, 39.8, 32.2, 24.4, 20.2,
    15.5, 7.9, 5.3, 3.0, 1.5, 0.6, 0.0,
)


def validate_soc_curve(voltage_points, percent_points):
    """Return the curve normalised to descending voltage order.

    Raises ValueError if the pair cannot describe a monotonic curve, so a
    caller can fall back to the defaults rather than display nonsense.
    """
    voltages = [float(v) for v in voltage_points]
    percents = [float(p) for p in percent_points]

    if len(voltages) != len(percents):
        raise ValueError(
            f"soc curve needs matching lengths, got {len(voltages)} voltages "
            f"and {len(percents)} percentages"
        )
    if len(voltages) < 2:
        raise ValueError("soc curve needs at least two points")
    if not all(math.isfinite(v) for v in voltages):
        raise ValueError("soc curve voltages must all be finite")
    if not all(math.isfinite(p) for p in percents):
        raise ValueError("soc curve percentages must all be finite")
    if any(p < 0.0 or p > 100.0 for p in percents):
        raise ValueError("soc curve percentages must lie within 0..100")

    ascending = all(a < b for a, b in zip(voltages, voltages[1:]))
    if ascending:
        voltages.reverse()
        percents.reverse()
    elif not all(a > b for a, b in zip(voltages, voltages[1:])):
        raise ValueError("soc curve voltages must be strictly monotonic")

    if not all(a >= b for a, b in zip(percents, percents[1:])):
        raise ValueError(
            "soc curve percentages must not rise as voltage falls"
        )
    return tuple(voltages), tuple(percents)


def state_of_charge_percent(voltage, voltage_points=None, percent_points=None):
    """Interpolate state of charge in percent (0..100) from pack voltage.

    Points are expected in descending voltage order, as produced by
    validate_soc_curve.
    """
    voltages = (
        DEFAULT_SOC_VOLTAGE_POINTS if voltage_points is None else voltage_points
    )
    percents = (
        DEFAULT_SOC_PERCENT_POINTS if percent_points is None else percent_points
    )
    value = float(voltage)
    if not math.isfinite(value):
        raise ValueError("voltage must be finite")

    if value >= voltages[0]:
        result = percents[0]
    elif value <= voltages[-1]:
        result = percents[-1]
    else:
        result = percents[-1]
        for i in range(len(voltages) - 1):
            hi_v, lo_v = voltages[i], voltages[i + 1]
            if lo_v <= value <= hi_v:
                span = hi_v - lo_v
                frac = 1.0 if span == 0 else (value - lo_v) / span
                result = percents[i + 1] + frac * (percents[i] - percents[i + 1])
                break
    return max(0.0, min(100.0, float(result)))


class MedianVoltageFilter:
    """Rolling median over the last ``window`` samples.

    Display smoothing only. Feeding this to LowVoltageMonitor would delay a
    genuine collapse by most of the window, so the node keeps the two paths
    apart. Reports a median from the first sample onwards rather than waiting
    for a full window, so the percentage appears immediately after startup.
    """

    def __init__(self, window: int = 30):
        size = int(window)
        if size < 1:
            raise ValueError("median window must be at least 1")
        self._samples: deque[float] = deque(maxlen=size)

    @property
    def window(self) -> int:
        return self._samples.maxlen

    @property
    def count(self) -> int:
        return len(self._samples)

    def reset(self) -> None:
        self._samples.clear()

    def update(self, value: float) -> float:
        sample = float(value)
        if not math.isfinite(sample):
            raise ValueError("median filter needs a finite sample")
        self._samples.append(sample)
        ordered = sorted(self._samples)
        mid = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[mid]
        return (ordered[mid - 1] + ordered[mid]) / 2.0

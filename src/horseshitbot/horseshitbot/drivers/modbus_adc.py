"""Transport-free helpers for the N43IC04 Modbus acquisition module.

ROS serial ownership deliberately does not live here. The bus node reads one
raw register and battery_modbus_node uses this module only for conversion.
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


@dataclass(frozen=True)
class AdcRegisterMap:
    channel_base_register: int = 0x0000
    address_register: int = 0x00FD
    baudrate_register: int = 0x00FE
    # Legacy conversion placeholders, not verified N43IC04 protocol scaling.
    raw_scale: float = 100.0
    voltage_divider_factor: float = 1.0
    voltage_offset: float = 0.0
    voltage_multiplier: float = 1.0

    def channel_register(self, channel: int) -> int:
        if channel not in (1, 2, 3, 4):
            raise ValueError("channel must be 1, 2, 3, or 4")
        return self.channel_base_register + channel - 1

    def convert(self, raw: int, channel: int) -> dict:
        if not 0 <= int(raw) <= 0xFFFF:
            raise ValueError("raw ADC value must fit in uint16")
        if self.raw_scale <= 0:
            raise ValueError("raw_scale must be positive")
        if self.voltage_divider_factor <= 0:
            raise ValueError("voltage_divider_factor must be positive")

        adc_voltage = (
            int(raw) / self.raw_scale * self.voltage_multiplier
            + self.voltage_offset
        )
        return {
            "channel": int(channel),
            "raw": int(raw),
            "adc_voltage": adc_voltage,
            "battery_voltage": adc_voltage / self.voltage_divider_factor,
        }


# Empirical voltage-only estimate from the 2026-09-25 discharge recording.
# It is indicative, not a substitute for current integration or a BMS.
DEFAULT_SOC_VOLTAGE_POINTS = (
    26.40, 26.20, 26.05, 25.95, 25.85, 25.70, 25.55,
    25.40, 25.20, 24.80, 24.00, 23.00, 22.00, 20.00,
)
DEFAULT_SOC_PERCENT_POINTS = (
    100.0, 70.8, 52.1, 39.8, 32.2, 24.4, 20.2,
    15.5, 7.9, 5.3, 3.0, 1.5, 0.6, 0.0,
)


def validate_soc_curve(voltage_points, percent_points):
    voltages = [float(value) for value in voltage_points]
    percents = [float(value) for value in percent_points]
    if len(voltages) != len(percents):
        raise ValueError("soc curve voltage and percentage lengths differ")
    if len(voltages) < 2:
        raise ValueError("soc curve needs at least two points")
    if not all(math.isfinite(value) for value in voltages + percents):
        raise ValueError("soc curve values must be finite")
    if any(value < 0.0 or value > 100.0 for value in percents):
        raise ValueError("soc curve percentages must be within 0..100")
    if all(a < b for a, b in zip(voltages, voltages[1:])):
        voltages.reverse()
        percents.reverse()
    elif not all(a > b for a, b in zip(voltages, voltages[1:])):
        raise ValueError("soc curve voltages must be strictly monotonic")
    if not all(a >= b for a, b in zip(percents, percents[1:])):
        raise ValueError("soc percentage must not rise as voltage falls")
    return tuple(voltages), tuple(percents)


def state_of_charge_percent(
    voltage, voltage_points=None, percent_points=None
):
    voltages = voltage_points or DEFAULT_SOC_VOLTAGE_POINTS
    percents = percent_points or DEFAULT_SOC_PERCENT_POINTS
    value = float(voltage)
    if not math.isfinite(value):
        raise ValueError("voltage must be finite")
    if value >= voltages[0]:
        return float(percents[0])
    if value <= voltages[-1]:
        return float(percents[-1])
    for index, (high, low) in enumerate(zip(voltages, voltages[1:])):
        if low <= value <= high:
            fraction = (value - low) / (high - low)
            result = percents[index + 1] + fraction * (
                percents[index] - percents[index + 1]
            )
            return max(0.0, min(100.0, float(result)))
    raise ValueError("voltage did not fall within the soc curve")


class MedianVoltageFilter:
    def __init__(self, window=30):
        if int(window) < 1:
            raise ValueError("median window must be at least 1")
        self._samples = deque(maxlen=int(window))

    @property
    def count(self):
        return len(self._samples)

    def reset(self):
        self._samples.clear()

    def update(self, value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("median filter requires a finite sample")
        self._samples.append(value)
        ordered = sorted(self._samples)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0

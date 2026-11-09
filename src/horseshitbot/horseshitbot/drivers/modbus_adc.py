"""Transport-free helpers for the N43IC04 Modbus acquisition module.

ROS serial ownership deliberately does not live here. The bus node reads one
raw register and battery_modbus_node uses this module only for conversion.

Battery calibration provenance
------------------------------
Both the raw->voltage conversion and the voltage->state-of-charge curve below
come from physical measurements taken on the robot on 2026-09-25, not from a
datasheet. The voltage calibration reproduces the discharge logger's
``(raw + 15.2) / 49.25``. The state-of-charge curve is derived from a single
21.4 h discharge run down to the BMS cutoff, so it is an empirical estimate
for that one load profile.

No battery current or coulomb counting is available, so state of charge is
inferred from voltage alone. Accuracy varies with load, temperature, battery
ageing and battery-to-battery variation, and the curve is very flat through
the middle of its range. Treat the percentage as indicative only. Safety
shutdown decisions are made from measured voltage, never from this estimate.
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


# Empirical HorseShitBot discharge calibration, 2026-09-25. Ordered from the
# top of the usable range downwards. Everything above the first voltage reads
# 100% because the pack spends only its first ~16 minutes there shedding
# surface charge; everything at or below the last reads 0%.
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

    Reports a median from the first sample onwards rather than waiting for a
    full window, so the percentage appears immediately after startup instead
    of staying blank for the length of the window.
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

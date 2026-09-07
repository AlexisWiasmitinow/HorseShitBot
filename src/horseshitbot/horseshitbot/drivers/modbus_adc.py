"""Transport-free helpers for the N43VD04-style Modbus ADC.

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


@dataclass(frozen=True)
class AdcRegisterMap:
    channel_base_register: int = 0x0000
    # These configuration defaults came from the older structured ADC driver.
    # Confirm them against the exact installed module before any write.
    address_register: int = 0x000E
    baudrate_register: int = 0x000F
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

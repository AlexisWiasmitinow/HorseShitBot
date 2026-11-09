"""Presentation helpers for the status screen's battery section.

Pure formatting over values published on /battery/status. Battery calibration,
state-of-charge estimation and safety decisions remain in battery_modbus_node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


DEFAULT_STALE_AFTER_SEC = 5.0
LOW_PERCENT = 20
CRITICAL_PERCENT = 5

UNKNOWN_PERCENT_TEXT = "--%"
UNKNOWN_VOLTAGE_TEXT = "--.- V"
CHARGE_TEXT = "Charge: unknown"

HEALTH_GOOD = 1
HEALTH_DEAD = 3
HEALTH_UNSPEC_FAILURE = 5

LEVEL_OK = "ok"
LEVEL_LOW = "low"
LEVEL_CRITICAL = "critical"
LEVEL_UNKNOWN = "unknown"

STATUS_OK = "ok"
STATUS_MISSING = "missing"
STATUS_STALE = "stale"
STATUS_INVALID = "invalid"

_LEVEL_RANK = {LEVEL_UNKNOWN: -1, LEVEL_OK: 0, LEVEL_LOW: 1, LEVEL_CRITICAL: 2}


@dataclass(frozen=True)
class BatteryReadout:
    percent: int | None
    percent_text: str
    voltage_text: str
    level: str
    status: str
    badge: str | None
    age_sec: float | None = None

    @property
    def available(self) -> bool:
        return self.status in (STATUS_OK, STATUS_INVALID)

    @property
    def bar_fraction(self) -> float:
        return 0.0 if self.percent is None else self.percent / 100.0


def display_percent(fraction) -> int | None:
    """Convert a BatteryState 0..1 fraction to an integer display percent."""
    if fraction is None or isinstance(fraction, bool):
        return None
    try:
        value = float(fraction)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or value < 0.0 or value > 1.0 + 1e-6:
        return None
    return int(max(0, min(100, math.floor(value * 100.0 + 0.5))))


def format_percent(percent: int | None) -> str:
    return UNKNOWN_PERCENT_TEXT if percent is None else f"{percent}%"


def valid_voltage(voltage) -> float | None:
    if voltage is None or isinstance(voltage, bool):
        return None
    try:
        value = float(voltage)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def format_voltage(voltage) -> str:
    value = valid_voltage(voltage)
    return UNKNOWN_VOLTAGE_TEXT if value is None else f"{value:.1f} V"


def health_level(power_supply_health) -> str:
    try:
        health = int(power_supply_health)
    except (TypeError, ValueError):
        return LEVEL_UNKNOWN
    if health == HEALTH_DEAD:
        return LEVEL_CRITICAL
    if health == HEALTH_UNSPEC_FAILURE:
        return LEVEL_LOW
    if health == HEALTH_GOOD:
        return LEVEL_OK
    return LEVEL_UNKNOWN


def percent_level(percent: int | None) -> str:
    if percent is None:
        return LEVEL_UNKNOWN
    if percent <= CRITICAL_PERCENT:
        return LEVEL_CRITICAL
    if percent <= LOW_PERCENT:
        return LEVEL_LOW
    return LEVEL_OK


def worst_level(*levels: str) -> str:
    return max(levels, key=lambda level: _LEVEL_RANK.get(level, -1))


def _badge_for(level: str) -> str | None:
    if level == LEVEL_CRITICAL:
        return "CRITICAL"
    if level == LEVEL_LOW:
        return "LOW"
    return None


def _format_age(age: float) -> str:
    if age < 120:
        return f"{int(age)}s"
    if age < 7200:
        return f"{int(age // 60)}m"
    return f"{int(age // 3600)}h"


def battery_readout(
    sample,
    received_at=None,
    now=0.0,
    stale_after=DEFAULT_STALE_AFTER_SEC,
) -> BatteryReadout:
    """Validate a recent BatteryState sample for truthful display."""
    if sample is None or received_at is None or not sample.get("present", True):
        return BatteryReadout(
            None,
            UNKNOWN_PERCENT_TEXT,
            UNKNOWN_VOLTAGE_TEXT,
            LEVEL_UNKNOWN,
            STATUS_MISSING,
            "NO DATA",
        )

    age = max(0.0, float(now) - float(received_at))
    if age > stale_after:
        return BatteryReadout(
            None,
            UNKNOWN_PERCENT_TEXT,
            UNKNOWN_VOLTAGE_TEXT,
            LEVEL_UNKNOWN,
            STATUS_STALE,
            f"STALE {_format_age(age)}",
            age,
        )

    percent = display_percent(sample.get("percentage"))
    voltage = valid_voltage(sample.get("voltage"))
    level = worst_level(health_level(sample.get("health")), percent_level(percent))
    if percent is None or voltage is None:
        return BatteryReadout(
            percent,
            format_percent(percent),
            format_voltage(voltage),
            level,
            STATUS_INVALID,
            _badge_for(level) or "INVALID",
            age,
        )

    return BatteryReadout(
        percent,
        format_percent(percent),
        format_voltage(voltage),
        level,
        STATUS_OK,
        _badge_for(level),
        age,
    )

"""Presentation helpers for the status screen's battery section.

Pure formatting over the values published on /battery/status. There is no ROS,
display or ADC access here, and deliberately no voltage-to-percentage
conversion: battery_modbus_node owns the calibration, the state-of-charge
curve, the smoothing and the shutdown decision. The screen only renders what
that node already published.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# battery_modbus_node publishes at ~1 Hz, so five missed messages means it
# stopped or restarted rather than a scheduling hiccup.
DEFAULT_STALE_AFTER_SEC = 5.0

# Display-only colour bands on the already-published percentage. The node's
# own low/critical voltage thresholds sit near 0 % on the SoC curve, so they
# alone would leave a nearly empty pack green. These never feed back into the
# low-voltage monitor or the shutdown path.
LOW_PERCENT = 20
CRITICAL_PERCENT = 5

UNKNOWN_PERCENT_TEXT = "--%"
UNKNOWN_VOLTAGE_TEXT = "--.- V"

# There is no BMS link, current sensor or charger-detect input on the robot.
# battery_modbus_node hardcodes POWER_SUPPLY_STATUS_DISCHARGING, which is not
# a measurement, so the charge direction is always shown as unknown.
CHARGE_TEXT = "Charge: unknown"

# sensor_msgs/BatteryState.POWER_SUPPLY_HEALTH_* as set by battery_modbus_node.
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
    """Everything the screen needs to draw the battery section."""

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
        """0.0..1.0 gauge fill; empty whenever the percentage is unknown."""
        if self.percent is None:
            return 0.0
        return self.percent / 100.0


def display_percent(fraction) -> int | None:
    """Convert a BatteryState 0..1 fraction to a whole 0..100 display percent.

    Returns None for anything that cannot be shown truthfully: missing, NaN
    (BatteryState's "unknown"), infinite, or outside 0..1. Out-of-range values
    are rejected rather than clamped so a 0..100 units bug is visible instead
    of silently pinned at 100 %.
    """
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
    if not math.isfinite(value) or value <= 0.0:
        return None
    return value


def format_voltage(voltage) -> str:
    value = valid_voltage(voltage)
    return UNKNOWN_VOLTAGE_TEXT if value is None else f"{value:.1f} V"


def health_level(power_supply_health) -> str:
    """Map BatteryState.power_supply_health onto a screen level."""
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
    return max(levels, key=lambda lvl: _LEVEL_RANK.get(lvl, -1))


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
    """Build the battery section from the last /battery/status message.

    ``sample`` is a dict with ``percentage``, ``voltage``, ``health`` and
    optionally ``present``; None until the first message arrives. A sample
    older than ``stale_after`` is reported as unknown rather than left frozen
    on screen, so a stopped battery node is visible instead of silently
    showing its last value forever.
    """
    if sample is None or received_at is None or not sample.get("present", True):
        return BatteryReadout(
            percent=None,
            percent_text=UNKNOWN_PERCENT_TEXT,
            voltage_text=UNKNOWN_VOLTAGE_TEXT,
            level=LEVEL_UNKNOWN,
            status=STATUS_MISSING,
            badge="NO DATA",
        )

    age = max(0.0, float(now) - float(received_at))
    if age > stale_after:
        return BatteryReadout(
            percent=None,
            percent_text=UNKNOWN_PERCENT_TEXT,
            voltage_text=UNKNOWN_VOLTAGE_TEXT,
            level=LEVEL_UNKNOWN,
            status=STATUS_STALE,
            badge=f"STALE {_format_age(age)}",
            age_sec=age,
        )

    percent = display_percent(sample.get("percentage"))
    voltage = valid_voltage(sample.get("voltage"))
    level = worst_level(health_level(sample.get("health")), percent_level(percent))

    if percent is None or voltage is None:
        return BatteryReadout(
            percent=percent,
            percent_text=format_percent(percent),
            voltage_text=format_voltage(voltage),
            level=level,
            status=STATUS_INVALID,
            badge=_badge_for(level) or "INVALID",
            age_sec=age,
        )

    return BatteryReadout(
        percent=percent,
        percent_text=format_percent(percent),
        voltage_text=format_voltage(voltage),
        level=level,
        status=STATUS_OK,
        badge=_badge_for(level),
        age_sec=age,
    )

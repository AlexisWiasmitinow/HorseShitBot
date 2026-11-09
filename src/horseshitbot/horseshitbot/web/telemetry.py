"""Pure helpers for dashboard telemetry (no ROS or FastAPI imports)."""

from __future__ import annotations

import math
from typing import Optional

# The RealSense streams run at 30 fps (15 fps minimum in practice), so 2 s is
# 30-60 missed frames: long enough to ride out DDS or CPU hiccups, short
# enough that an unplugged camera is reported within a couple of seconds.
CAMERA_STALE_SEC = 2.0

# Same default as status_screen_node's battery_stale_sec; the battery node
# publishes at 1 Hz.
BATTERY_STALE_SEC = 5.0


def _age(now: float, received_at: Optional[float]) -> Optional[float]:
    if received_at is None:
        return None
    return max(0.0, now - received_at)


def telemetry_health(
    key: str,
    value: dict,
    now: float,
    stale_sec: float | None = None,
) -> dict:
    """Classify receipt-time freshness without pretending cached data is live."""
    if stale_sec is None:
        stale_sec = 5.0 if key == "mks_bus" else 3.0
    received_at = value.get("_received_at")
    if not isinstance(received_at, (int, float)):
        return {
            "state": "no_data",
            "age_sec": None,
            "stale_after_sec": stale_sec,
        }
    age = _age(now, float(received_at))
    if age is not None and age > stale_sec:
        state = "stale"
    elif key == "lidar" and value.get("connected") is False:
        state = "error"
    elif key == "lidar" and value.get("scanning") is False:
        state = "disabled"
    elif key == "mks_bus" and value.get("bus_connected") is False:
        state = "error"
    elif key == "wheel_status" and (
        value.get("error") or value.get("estop_release_error")
    ):
        state = "error"
    else:
        state = "live"
    return {
        "state": state,
        "age_sec": None if age is None else round(age, 3),
        "stale_after_sec": stale_sec,
    }


def camera_status(
    now: float,
    color_received_at: Optional[float],
    depth_received_at: Optional[float],
    stale_sec: float = CAMERA_STALE_SEC,
) -> dict:
    """Robot-side camera availability from the arrival times of real frames.

    Times are monotonic seconds; None means no frame has ever arrived. This
    says nothing about whether a browser preview is open.
    """
    color_age = _age(now, color_received_at)
    depth_age = _age(now, depth_received_at)
    color_available = color_age is not None and color_age <= stale_sec
    depth_available = depth_age is not None and depth_age <= stale_sec
    ages = [a for a in (color_age, depth_age) if a is not None]
    connected = color_available or depth_available
    return {
        "connected": connected,
        "streaming": connected,
        "color_available": color_available,
        "depth_available": depth_available,
        "color_age_sec": None if color_age is None else round(color_age, 2),
        "depth_age_sec": None if depth_age is None else round(depth_age, 2),
        "last_frame_age_sec": round(min(ages), 2) if ages else None,
        "stale_after_sec": stale_sec,
        "source": "image frames on /camera/color/image_raw and "
                  "/camera/aligned_depth_to_color/image_raw",
    }


def battery_percent_from_fraction(value) -> Optional[float]:
    """BatteryState-style 0..1 fraction -> 0..100 percent, or None."""
    try:
        fraction = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(fraction) or fraction < 0.0:
        return None
    return round(min(fraction, 1.0) * 100.0, 1)


def battery_percent_from_json(data: dict) -> Optional[float]:
    """Percent from a /battery/status_json payload.

    "percentage" follows BatteryState (0..1 fraction); "percent" is already
    0..100. Each is scaled exactly once, never guessed from its magnitude.
    """
    if data.get("percentage") is not None:
        return battery_percent_from_fraction(data["percentage"])
    if data.get("percent") is not None:
        try:
            percent = float(data["percent"])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(percent) or percent < 0.0:
            return None
        return round(min(percent, 100.0), 1)
    return None


def battery_view(battery: dict, now: float, stale_sec: float = BATTERY_STALE_SEC) -> dict:
    """Copy of the merged battery state with its age and freshness.

    `battery["updated_at"]` is wall-clock time of the last battery message.
    """
    view = dict(battery)
    updated_at = view.get("updated_at")
    age = _age(now, updated_at) if isinstance(updated_at, (int, float)) else None
    view["age_sec"] = None if age is None else round(age, 2)
    view["stale_after_sec"] = stale_sec
    view["available"] = (
        view.get("available") is not False
        and view.get("percent") is not None
        and age is not None
        and age <= stale_sec
    )
    return view

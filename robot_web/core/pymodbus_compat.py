"""Feature-detected pymodbus 3.x compatibility for the standalone legacy app."""

from __future__ import annotations

import inspect
from typing import Callable

import pymodbus
from pymodbus.client import ModbusSerialClient


def select_rtu_framer(module):
    """Return the installed pymodbus RTU framer enum value."""
    for enum_name in ("FramerType", "Framer"):
        enum = getattr(module, enum_name, None)
        if enum is not None and hasattr(enum, "RTU"):
            return enum.RTU
    raise ImportError("installed pymodbus has no supported RTU framer enum")


RTU_FRAMER = select_rtu_framer(pymodbus)


def call_with_device(method: Callable, unit_id: int, **kwargs):
    """Call a pymodbus method with its supported device-address keyword."""
    parameters = inspect.signature(method).parameters
    for keyword in ("device_id", "slave", "unit"):
        if keyword in parameters:
            return method(**kwargs, **{keyword: int(unit_id)})
    raise TypeError(
        f"{getattr(method, '__name__', method)!r} has no supported "
        "Modbus device-address parameter"
    )

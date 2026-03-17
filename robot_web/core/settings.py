from __future__ import annotations
from dataclasses import dataclass

from .util import get_env_bool, get_env_float, get_env_int, get_env_str
from .bus import MODE_SR_VFOC

@dataclass(frozen=True)
class Settings:
    port: str
    baud: int

    id_left: int
    id_right: int
    invert_left_dir: bool
    invert_right_dir: bool
    max_wheel_rpm: float
    wheel_acc_reg: int
    wheel_update_hz: float
    wheel_accel_rpm_s: float
    wheel_decel_rpm_s: float
    stop_decel_rpm_s: float
    zero_hold_sec: float
    watchdog_sec: float

    id_lift_a: int
    id_lift_b: int
    invert_lift_dir: bool
    lift_max_rpm: float

    id_brush: int
    invert_brush_dir: bool
    brush_max_rpm: float

    id_side_door: int
    invert_side_door_dir: bool
    side_door_max_rpm: float

    aux_watchdog_sec: float
    aux_deadzone_pct: float

    web_host: str
    web_port: int
    camera_stream_url: str

    work_mode: int = MODE_SR_VFOC

def load_settings() -> Settings:
    return Settings(
        port=get_env_str("MKS_PORT", "/dev/ttyUSB0"),
        baud=get_env_int("MKS_BAUD", 38400),

        id_left=get_env_int("ID_LEFT", 1),
        id_right=get_env_int("ID_RIGHT", 2),
        invert_left_dir=get_env_bool("INVERT_LEFT_DIR", False),
        invert_right_dir=get_env_bool("INVERT_RIGHT_DIR", True),
        max_wheel_rpm=get_env_float("MAX_WHEEL_RPM", 500.0),
        wheel_acc_reg=get_env_int("WHEEL_ACC_REG", 3),
        wheel_update_hz=get_env_float("WHEEL_UPDATE_HZ", 50.0),
        wheel_accel_rpm_s=get_env_float("WHEEL_ACCEL_RPM_PER_S", 120.0),
        wheel_decel_rpm_s=get_env_float("WHEEL_DECEL_RPM_PER_S", 80.0),
        stop_decel_rpm_s=get_env_float("STOP_DECEL_RPM_PER_S", 500.0),
        zero_hold_sec=get_env_float("ZERO_HOLD_SEC", 0.10),
        watchdog_sec=get_env_float("WATCHDOG_SEC", 0.8),

        id_lift_a=get_env_int("ID_LIFT_A", 3),
        id_lift_b=get_env_int("ID_LIFT_B", 5),
        invert_lift_dir=get_env_bool("INVERT_LIFT_DIR", False),
        lift_max_rpm=get_env_float("LIFT_MAX_RPM", 450.0),

        id_brush=get_env_int("ID_BRUSH", 4),
        invert_brush_dir=get_env_bool("INVERT_BRUSH_DIR", False),
        brush_max_rpm=get_env_float("BRUSH_MAX_RPM", 450.0),

        id_side_door=get_env_int("ID_SIDE_DOOR", 6),
        invert_side_door_dir=get_env_bool("INVERT_SIDE_DOOR_DIR", False),
        side_door_max_rpm=get_env_float("SIDE_DOOR_MAX_RPM", 450.0),

        aux_watchdog_sec=get_env_float("AUX_WATCHDOG_SEC", 0.8),
        aux_deadzone_pct=get_env_float("AUX_DEADZONE_PCT", 2.0),

        web_host=get_env_str("WEB_HOST", "0.0.0.0"),
        web_port=get_env_int("WEB_PORT", 8000),
        camera_stream_url=get_env_str("CAMERA_STREAM_URL", ""),
    )

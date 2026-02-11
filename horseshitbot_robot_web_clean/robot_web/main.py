from __future__ import annotations

from pathlib import Path

from dotenv import load_dotenv
from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .core.bus import BusCfg, MksBus
from .core.settings import load_settings
from .core.router import build_router as build_core_router

from .wheels.controller import WheelsController
from .wheels.router import build_router as build_wheels_router

from .lift.controller import LiftController
from .lift.router import build_router as build_lift_router

from .brush.controller import AuxSpeedController as BrushController
from .brush.router import build_router as build_brush_router

from .side_door.controller import AuxSpeedController as SideDoorController
from .side_door.router import build_router as build_side_router


# Load .env if present
load_dotenv(dotenv_path=Path(".env"), override=False)

S = load_settings()

app = FastAPI(title="Horseshitbot Robot Web (modular)")

STATIC_DIR = Path(__file__).parent / "ui" / "static"
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

bus = MksBus(BusCfg(port=S.port, baud=S.baud, timeout=0.35, retries=3, inter_delay=0.002))

wheels = WheelsController(
    bus=bus,
    id_left=S.id_left,
    id_right=S.id_right,
    invert_left_dir=S.invert_left_dir,
    invert_right_dir=S.invert_right_dir,
    max_wheel_rpm=S.max_wheel_rpm,
    acc_reg=S.wheel_acc_reg,
    update_hz=S.wheel_update_hz,
    accel_rpm_s=S.wheel_accel_rpm_s,
    decel_rpm_s=S.wheel_decel_rpm_s,
    stop_decel_rpm_s=S.stop_decel_rpm_s,
    zero_hold_sec=S.zero_hold_sec,
    watchdog_sec=S.watchdog_sec,
)

lift = LiftController(
    bus=bus,
    id_a=S.id_lift_a,
    id_b=S.id_lift_b,
    max_rpm=S.lift_max_rpm,
    acc_reg=S.wheel_acc_reg,
    invert_dir_a=S.invert_lift_dir,
    watchdog_sec=S.aux_watchdog_sec,
    deadzone_pct=S.aux_deadzone_pct,
)

brush = BrushController(
    bus=bus,
    motor_id=S.id_brush,
    max_rpm=S.brush_max_rpm,
    acc_reg=S.wheel_acc_reg,
    invert_dir=S.invert_brush_dir,
    watchdog_sec=S.aux_watchdog_sec,
    deadzone_pct=S.aux_deadzone_pct,
)

side_door = SideDoorController(
    bus=bus,
    motor_id=S.id_side_door,
    max_rpm=S.side_door_max_rpm,
    acc_reg=S.wheel_acc_reg,
    invert_dir=S.invert_side_door_dir,
    watchdog_sec=S.aux_watchdog_sec,
    deadzone_pct=S.aux_deadzone_pct,
)

ALL_MOTORS = [S.id_left, S.id_right, S.id_lift_a, S.id_lift_b, S.id_brush, S.id_side_door]


@app.on_event("startup")
def _startup():
    bus.connect()
    for mid in ALL_MOTORS:
        try:
            bus.init_servo(int(mid), mode=S.work_mode, enable=True)
        except Exception:
            pass

    wheels.start()
    lift.start()
    brush.start()
    side_door.start()


@app.on_event("shutdown")
def _shutdown():
    try:
        wheels.set_stop_fast()
    except Exception:
        pass
    for ctrl in (lift, brush, side_door):
        try:
            ctrl.stop()
        except Exception:
            pass
    try:
        bus.close()
    except Exception:
        pass


@app.get("/")
def index():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.get("/api/config")
def config():
    return {"camera_stream_url": S.camera_stream_url}


app.include_router(build_wheels_router(wheels))
app.include_router(build_lift_router(lift))
app.include_router(build_brush_router(brush))
app.include_router(build_side_router(side_door))
app.include_router(build_core_router(bus, ALL_MOTORS))


@app.post("/api/aux/stop")
def stop_all_aux():
    lift.stop()
    brush.stop()
    side_door.stop()
    return {"ok": True}

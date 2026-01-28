import os
import time
import threading
from dataclasses import dataclass

from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from mks_bus import MksBus, BusCfg, clamp, COUNTS_PER_REV, MODE_SR_VFOC, MODE_SR_OPEN

# ---------------- CONFIG ----------------
MKS_PORT = os.getenv("MKS_PORT", "/dev/ttyUSB0")
MKS_BAUD = int(os.getenv("MKS_BAUD", "38400"))

# Modbus robustness
MODBUS_TIMEOUT = float(os.getenv("MODBUS_TIMEOUT", "0.35"))
MODBUS_RETRIES = int(os.getenv("MODBUS_RETRIES", "3"))
MODBUS_INTER_DELAY = float(os.getenv("MODBUS_INTER_DELAY", "0.002"))

# Wheels
ID_LEFT  = 1
ID_RIGHT = 2

# Website mapping (your request)
ID_LIFT  = 3   # Motor 3 = Lift
ID_BRUSH = 4   # Motor 4 = Brush
ID_SILO  = 8   # Motor 8 = Silo

# Wheel ID=2 rotates opposite to ID=1
INVERT_LEFT_DIR  = False
INVERT_RIGHT_DIR = True

# ---------------- Wheels ----------------
MAX_WHEEL_RPM = float(os.getenv("MAX_WHEEL_RPM", "500"))

WHEEL_ACC_REG = int(os.getenv("WHEEL_ACC_REG", "3"))

WHEEL_UPDATE_HZ = float(os.getenv("WHEEL_UPDATE_HZ", "50"))
WHEEL_ACCEL_RPM_PER_S = float(os.getenv("WHEEL_ACCEL_RPM_PER_S", "120"))

# Base decel when joystick released
WHEEL_DECEL_RPM_PER_S = float(os.getenv("WHEEL_DECEL_RPM_PER_S", "80"))

# Fast decel when STOP button pressed
STOP_DECEL_RPM_PER_S = float(os.getenv("STOP_DECEL_RPM_PER_S", "500"))

ZERO_HOLD_SEC = float(os.getenv("ZERO_HOLD_SEC", "0.10"))
WATCHDOG_SEC  = float(os.getenv("WATCHDOG_SEC", "0.8"))

# ---------------- Position motors ----------------
POS_SPEED_RPM = int(os.getenv("POS_SPEED_RPM", "600"))
POS_ACC       = int(os.getenv("POS_ACC", "2"))

POS_SPEED_RPM_SILO = int(os.getenv("POS_SPEED_RPM_SILO", "300"))

# Slider -> turns mapping
BRUSH_TURNS = int(os.getenv("BRUSH_TURNS", "5"))   # motor 4
LIFT_TURNS  = int(os.getenv("LIFT_TURNS", "11"))   # motor 3
SILO_TURNS  = int(os.getenv("SILO_TURNS", "1"))    # motor 8

TURNS_BY_ID = {
    ID_BRUSH: BRUSH_TURNS,
    ID_LIFT:  LIFT_TURNS,
    ID_SILO:  SILO_TURNS,
}

# ---------------- APP ----------------
app = FastAPI(title="Horseshitbot Robot Web")
app.mount("/static", StaticFiles(directory="static"), name="static")

bus = MksBus(BusCfg(
    port=MKS_PORT,
    baud=MKS_BAUD,
    timeout=MODBUS_TIMEOUT,
    retries=MODBUS_RETRIES,
    inter_delay=MODBUS_INTER_DELAY,
))

_state_lock = threading.Lock()
_last_drive_ts = 0.0

desired_left = 0.0
desired_right = 0.0

# STOP FAST latch (ignore joystick until both wheels are at 0)
_stop_fast_active = False

# For debugging / status
_last_err_left = ""
_last_err_right = ""
_last_err_ts = 0.0


def sign(x: float) -> int:
    if x > 0: return 1
    if x < 0: return -1
    return 0


def ramp_toward(current: float, target: float, rate_rpm_s: float, dt: float) -> float:
    max_delta = max(0.0, float(rate_rpm_s)) * dt
    delta = clamp(target - current, -max_delta, max_delta)
    nxt = current + delta
    if abs(nxt) < 0.5:
        nxt = 0.0
    return nxt


@dataclass
class WheelChannel:
    actual: float = 0.0
    desired: float = 0.0
    reversing: bool = False
    zero_until: float = 0.0

    def step(self, now: float, dt: float, decel_rate: float):
        cur_s = sign(self.actual)
        des_s = sign(self.desired)

        # reversing: go to 0 first
        if self.reversing:
            self.actual = ramp_toward(self.actual, 0.0, decel_rate, dt)
            if self.actual == 0.0:
                if now < self.zero_until:
                    return
                self.reversing = False
            else:
                return

        # start reversal if direction changes while moving
        if cur_s != 0 and des_s != 0 and cur_s != des_s:
            self.reversing = True
            self.zero_until = now + ZERO_HOLD_SEC
            self.actual = ramp_toward(self.actual, 0.0, decel_rate, dt)
            return

        # normal ramp
        rate = decel_rate if abs(self.desired) < abs(self.actual) else WHEEL_ACCEL_RPM_PER_S
        self.actual = ramp_toward(self.actual, self.desired, rate, dt)


wheel_L = WheelChannel()
wheel_R = WheelChannel()


def init_all():
    # Wheels + Brush + Lift in SR_vFOC
    for mid in (ID_LEFT, ID_RIGHT, ID_BRUSH, ID_LIFT):
        bus.init_servo(mid, mode=MODE_SR_VFOC)

    # Silo in SR_OPEN
    bus.init_servo(ID_SILO, mode=MODE_SR_OPEN)


def _send_wheels(left_rpm: float, right_rpm: float):
    """Send wheel speeds with independent error capture (don’t let one failure block the other)."""
    global _last_err_left, _last_err_right, _last_err_ts

    try:
        bus.set_speed_signed(ID_LEFT, left_rpm, acc=WHEEL_ACC_REG, invert_dir=INVERT_LEFT_DIR)
        _last_err_left = ""
    except Exception as e:
        _last_err_left = str(e)
        _last_err_ts = time.time()

    try:
        bus.set_speed_signed(ID_RIGHT, right_rpm, acc=WHEEL_ACC_REG, invert_dir=INVERT_RIGHT_DIR)
        _last_err_right = ""
    except Exception as e:
        _last_err_right = str(e)
        _last_err_ts = time.time()


def wheel_control_loop():
    global _last_drive_ts, desired_left, desired_right, _stop_fast_active

    period = 1.0 / max(1.0, WHEEL_UPDATE_HZ)
    last = time.time()

    while True:
        time.sleep(period)
        now = time.time()
        dt = now - last
        last = now

        with _state_lock:
            age = now - _last_drive_ts
            stop_fast = _stop_fast_active

            # STOP FAST priority: force targets to 0 no matter what
            if stop_fast:
                desired_left = 0.0
                desired_right = 0.0
                dl = 0.0
                dr = 0.0
            else:
                dl = desired_left
                dr = desired_right

        # Watchdog: no drive commands => base decel to 0
        if (not stop_fast) and age > WATCHDOG_SEC:
            dl = 0.0
            dr = 0.0
            with _state_lock:
                desired_left = 0.0
                desired_right = 0.0

        wheel_L.desired = dl
        wheel_R.desired = dr

        decel = STOP_DECEL_RPM_PER_S if stop_fast else WHEEL_DECEL_RPM_PER_S

        wheel_L.step(now, dt, decel)
        wheel_R.step(now, dt, decel)

        # Send commands
        _send_wheels(wheel_L.actual, wheel_R.actual)

        # Release STOP latch only when BOTH reached 0
        if stop_fast and wheel_L.actual == 0.0 and wheel_R.actual == 0.0:
            with _state_lock:
                _stop_fast_active = False


wheel_thread = threading.Thread(target=wheel_control_loop, daemon=True)


def pct_to_axis(motor_id: int, pct: float) -> int:
    pct = float(pct)
    pct = max(0.0, min(100.0, pct))
    turns = int(TURNS_BY_ID[motor_id])
    max_axis = turns * COUNTS_PER_REV
    axis = int(round((pct / 100.0) * max_axis))
    return max(0, min(max_axis, axis))


class DriveCmd(BaseModel):
    x: float = Field(0.0, ge=-1.0, le=1.0)
    y: float = Field(0.0, ge=-1.0, le=1.0)


class PosCmd(BaseModel):
    pct: float = Field(..., ge=0.0, le=100.0)


@app.on_event("startup")
def startup():
    bus.connect()
    init_all()
    if not wheel_thread.is_alive():
        wheel_thread.start()


@app.get("/")
def root():
    return FileResponse("static/index.html")


@app.get("/api/status")
def api_status():
    with _state_lock:
        return {
            "stop_fast_active": _stop_fast_active,
            "desired_left": desired_left,
            "desired_right": desired_right,
            "actual_left": wheel_L.actual,
            "actual_right": wheel_R.actual,
            "last_err_left": _last_err_left,
            "last_err_right": _last_err_right,
            "last_err_ts": _last_err_ts,
        }


@app.post("/api/drive")
def api_drive(cmd: DriveCmd):
    global desired_left, desired_right, _last_drive_ts, _stop_fast_active

    # Ignore joystick while STOP FAST is active
    with _state_lock:
        if _stop_fast_active:
            return {
                "ok": True,
                "stopping": True,
                "left_rpm": float(wheel_L.actual),
                "right_rpm": float(wheel_R.actual),
            }

    dl = (cmd.y - cmd.x) * MAX_WHEEL_RPM
    dr = (cmd.y + cmd.x) * MAX_WHEEL_RPM

    dl = clamp(dl, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)
    dr = clamp(dr, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)

    with _state_lock:
        desired_left = float(dl)
        desired_right = float(dr)
        _last_drive_ts = time.time()
        al = float(wheel_L.actual)
        ar = float(wheel_R.actual)

    return {"ok": True, "left_rpm": al, "right_rpm": ar}


@app.post("/api/stop")
def api_stop():
    """Soft stop: base decel (80 rpm/s by default)."""
    global desired_left, desired_right, _last_drive_ts, _stop_fast_active
    with _state_lock:
        desired_left = 0.0
        desired_right = 0.0
        _last_drive_ts = time.time()
        _stop_fast_active = False
    return {"ok": True}


@app.post("/api/stop_fast")
def api_stop_fast():
    """Fast stop: 500 rpm/s by default, latched until wheels reach 0."""
    global desired_left, desired_right, _last_drive_ts, _stop_fast_active
    with _state_lock:
        desired_left = 0.0
        desired_right = 0.0
        _last_drive_ts = time.time()
        _stop_fast_active = True

    # Extra reliability: immediately try to command 0 a few times
    for _ in range(3):
        _send_wheels(0.0, 0.0)
        time.sleep(0.02)

    return {"ok": True}


# ---------------- Motor 3 = Lift ----------------
@app.post("/api/motor/3/pos")
def m3_pos(cmd: PosCmd):
    axis = pct_to_axis(ID_LIFT, cmd.pct)
    bus.move_abs_axis(ID_LIFT, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
    return {"ok": True, "motor": 3, "pct": cmd.pct}


@app.post("/api/motor/3/zero")
def m3_zero():
    bus.axis_zero(ID_LIFT)
    return {"ok": True}


# ---------------- Motor 4 = Brush ----------------
@app.post("/api/motor/4/pos")
def m4_pos(cmd: PosCmd):
    axis = pct_to_axis(ID_BRUSH, cmd.pct)
    bus.move_abs_axis(ID_BRUSH, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
    return {"ok": True, "motor": 4, "pct": cmd.pct}


@app.post("/api/motor/4/zero")
def m4_zero():
    bus.axis_zero(ID_BRUSH)
    return {"ok": True}


# ---------------- Motor 8 = Silo ----------------
@app.post("/api/motor/8/pos")
def m8_pos(cmd: PosCmd):
    axis = pct_to_axis(ID_SILO, cmd.pct)
    bus.move_abs_axis(ID_SILO, axis, speed_rpm=POS_SPEED_RPM_SILO, acc=POS_ACC)
    return {"ok": True, "motor": 8, "pct": cmd.pct}


@app.post("/api/motor/8/zero")
def m8_zero():
    bus.axis_zero(ID_SILO)
    return {"ok": True}


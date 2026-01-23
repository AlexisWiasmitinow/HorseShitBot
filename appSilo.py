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

ID_LEFT  = 1
ID_RIGHT = 2
ID_BRUSH = 3
ID_LIFT  = 4
ID_SILO  = 8   # <-- NEW

# Wheel ID=2 rotates opposite to ID=1
INVERT_LEFT_DIR  = False
INVERT_RIGHT_DIR = True

# ---------------- Wheels: ULTRA SOFT RAMP ----------------
MAX_WHEEL_RPM = float(os.getenv("MAX_WHEEL_RPM", "500"))  # default 500 max

WHEEL_ACC_REG = int(os.getenv("WHEEL_ACC_REG", "3"))      # driver accel (small = smoother)

WHEEL_UPDATE_HZ = float(os.getenv("WHEEL_UPDATE_HZ", "50"))
WHEEL_ACCEL_RPM_PER_S = float(os.getenv("WHEEL_ACCEL_RPM_PER_S", "120"))
WHEEL_DECEL_RPM_PER_S = float(os.getenv("WHEEL_DECEL_RPM_PER_S", "120"))
ZERO_HOLD_SEC = float(os.getenv("ZERO_HOLD_SEC", "0.10"))

WATCHDOG_SEC  = float(os.getenv("WATCHDOG_SEC", "0.8"))

# ---------------- Position motors ----------------
POS_SPEED_RPM = int(os.getenv("POS_SPEED_RPM", "600"))
POS_ACC       = int(os.getenv("POS_ACC", "2"))

# Silo is SR_OPEN (open-loop) -> keep speed <= 400rpm (manual says SR_OPEN max 400)
POS_SPEED_RPM_SILO = int(os.getenv("POS_SPEED_RPM_SILO", "300"))

# ---------------- APP ----------------
app = FastAPI(title="Horseshitbot Robot Web")
app.mount("/static", StaticFiles(directory="static"), name="static")

bus = MksBus(BusCfg(port=MKS_PORT, baud=MKS_BAUD))

_state_lock = threading.Lock()
_last_drive_ts = 0.0

desired_left = 0.0
desired_right = 0.0


def sign(x: float) -> int:
    if x > 0: return 1
    if x < 0: return -1
    return 0


def ramp_toward(current: float, target: float, rate_rpm_s: float, dt: float) -> float:
    max_delta = max(0.0, rate_rpm_s) * dt
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

    def step(self, now: float, dt: float):
        cur_s = sign(self.actual)
        des_s = sign(self.desired)

        # If currently reversing: go to 0 smoothly first
        if self.reversing:
            self.actual = ramp_toward(self.actual, 0.0, WHEEL_DECEL_RPM_PER_S, dt)
            if self.actual == 0.0:
                if now < self.zero_until:
                    return
                self.reversing = False
            else:
                return

        # Start reversal if direction change requested while moving
        if cur_s != 0 and des_s != 0 and cur_s != des_s:
            self.reversing = True
            self.zero_until = now + ZERO_HOLD_SEC
            self.actual = ramp_toward(self.actual, 0.0, WHEEL_DECEL_RPM_PER_S, dt)
            return

        # Normal ramp
        rate = WHEEL_DECEL_RPM_PER_S if abs(self.desired) < abs(self.actual) else WHEEL_ACCEL_RPM_PER_S
        self.actual = ramp_toward(self.actual, self.desired, rate, dt)


wheel_L = WheelChannel()
wheel_R = WheelChannel()

pos_state = {
    ID_BRUSH: {"last_deg": 0.0, "last_axis": 0},
    ID_LIFT:  {"last_deg": 0.0, "last_axis": 0},
    ID_SILO:  {"last_deg": 0.0, "last_axis": 0},  # <-- NEW
}


def init_all():
    # Wheels + Brush + Lift in SR_vFOC
    for mid in (ID_LEFT, ID_RIGHT, ID_BRUSH, ID_LIFT):
        bus.init_servo(mid, mode=MODE_SR_VFOC)

    # Silo in SR_OPEN (open loop)
    bus.init_servo(ID_SILO, mode=MODE_SR_OPEN)


def wheel_control_loop():
    global _last_drive_ts, desired_left, desired_right
    period = 1.0 / max(1.0, WHEEL_UPDATE_HZ)
    last = time.time()

    while True:
        time.sleep(period)
        now = time.time()
        dt = now - last
        last = now

        with _state_lock:
            age = now - _last_drive_ts
            dl = desired_left
            dr = desired_right

        # Watchdog: if no drive commands, ramp down to zero
        if age > WATCHDOG_SEC:
            dl = 0.0
            dr = 0.0
            with _state_lock:
                desired_left = 0.0
                desired_right = 0.0

        wheel_L.desired = dl
        wheel_R.desired = dr

        wheel_L.step(now, dt)
        wheel_R.step(now, dt)

        try:
            bus.set_speed_signed(ID_LEFT, wheel_L.actual, acc=WHEEL_ACC_REG, invert_dir=INVERT_LEFT_DIR)
            bus.set_speed_signed(ID_RIGHT, wheel_R.actual, acc=WHEEL_ACC_REG, invert_dir=INVERT_RIGHT_DIR)
        except Exception:
            pass


wheel_thread = threading.Thread(target=wheel_control_loop, daemon=True)


def deg_to_counts(deg: float) -> int:
    d = float(deg)
    d = max(0.0, min(360.0, d))
    counts = int(round((d / 360.0) * COUNTS_PER_REV))
    if counts >= COUNTS_PER_REV:
        counts = 0
    return counts


def compute_consistent_axis(motor_id: int, new_deg: float) -> int:
    st = pos_state[motor_id]
    old_deg = float(st["last_deg"])
    old_axis = int(st["last_axis"])

    new_counts = deg_to_counts(new_deg)

    if old_axis == 0 and old_deg == 0.0:
        st["last_deg"] = float(new_deg)
        st["last_axis"] = int(new_counts)
        return int(new_counts)

    old_rem = old_axis % COUNTS_PER_REV

    if new_counts == old_rem:
        st["last_deg"] = float(new_deg)
        return old_axis

    delta_deg = float(new_deg) - old_deg
    base = old_axis - old_rem
    target = base + new_counts

    if delta_deg > 0:
        if new_counts <= old_rem:
            target += COUNTS_PER_REV
    elif delta_deg < 0:
        if new_counts >= old_rem:
            target -= COUNTS_PER_REV
    else:
        target = old_axis

    st["last_deg"] = float(new_deg)
    st["last_axis"] = int(target)
    return int(target)


class DriveCmd(BaseModel):
    x: float = Field(0.0, ge=-1.0, le=1.0)
    y: float = Field(0.0, ge=-1.0, le=1.0)


class PosCmd(BaseModel):
    deg: float = Field(..., ge=0.0, le=360.0)


@app.on_event("startup")
def startup():
    bus.connect()
    init_all()
    if not wheel_thread.is_alive():
        wheel_thread.start()


@app.get("/")
def root():
    return FileResponse("static/index.html")


@app.post("/api/drive")
def api_drive(cmd: DriveCmd):
    global desired_left, desired_right, _last_drive_ts

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
    global desired_left, desired_right, _last_drive_ts
    with _state_lock:
        desired_left = 0.0
        desired_right = 0.0
        _last_drive_ts = time.time()
    return {"ok": True}


# ---------------- Brush (ID 3) ----------------
@app.post("/api/motor/3/pos")
def m3_pos(cmd: PosCmd):
    axis = compute_consistent_axis(ID_BRUSH, cmd.deg)
    bus.move_abs_axis(ID_BRUSH, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
    return {"ok": True, "motor": 3, "deg": cmd.deg}


@app.post("/api/motor/3/zero")
def m3_zero():
    bus.axis_zero(ID_BRUSH)
    pos_state[ID_BRUSH]["last_deg"] = 0.0
    pos_state[ID_BRUSH]["last_axis"] = 0
    return {"ok": True}


# ---------------- Lift (ID 4) ----------------
@app.post("/api/motor/4/pos")
def m4_pos(cmd: PosCmd):
    axis = compute_consistent_axis(ID_LIFT, cmd.deg)
    bus.move_abs_axis(ID_LIFT, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
    return {"ok": True, "motor": 4, "deg": cmd.deg}


@app.post("/api/motor/4/zero")
def m4_zero():
    bus.axis_zero(ID_LIFT)
    pos_state[ID_LIFT]["last_deg"] = 0.0
    pos_state[ID_LIFT]["last_axis"] = 0
    return {"ok": True}


# ---------------- Silo (ID 8) NEW ----------------
@app.post("/api/motor/8/pos")
def m8_pos(cmd: PosCmd):
    axis = compute_consistent_axis(ID_SILO, cmd.deg)
    bus.move_abs_axis(ID_SILO, axis, speed_rpm=POS_SPEED_RPM_SILO, acc=POS_ACC)
    return {"ok": True, "motor": 8, "deg": cmd.deg}


@app.post("/api/motor/8/zero")
def m8_zero():
    bus.axis_zero(ID_SILO)
    pos_state[ID_SILO]["last_deg"] = 0.0
    pos_state[ID_SILO]["last_axis"] = 0
    return {"ok": True}

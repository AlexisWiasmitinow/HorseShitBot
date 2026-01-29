# Robot Web Modular (Fresh Raspberry Pi Setup)

This project runs a FastAPI server on a Raspberry Pi to control MKS SERVO57D motors over RS485 Modbus RTU.

Motor mapping:
- Tracks: ID 1 + ID 2
- Lift: ID 3 + ID 5
- Brush: ID 4
- Silo: ID 8

Lift motor ID 5 is set to counter-rotate relative to ID 3 using an environment variable.

## 1. System packages

```bash
sudo apt update
sudo apt install -y python3-venv git
```

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in (or reboot).

## 2. Project structure

```bash
mkdir -p ~/robot_web_modular/{setup,tracks,lift,brush,silo,ui/static}
cd ~/robot_web_modular
touch setup/__init__.py tracks/__init__.py lift/__init__.py brush/__init__.py silo/__init__.py
```

## 3. Python environment

```bash
cd ~/robot_web_modular
python3 -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip
```

```bash
pip install -U fastapi uvicorn pymodbus pyserial
```

```bash
cat > requirements.txt <<'REQ'
fastapi
uvicorn
pymodbus
pyserial
REQ
```

## 4. Create files

```bash
cat > .gitignore <<'GIT'
venv/
__pycache__/
*.pyc
*.pyo
*.pyd
*.log
.DS_Store
.idea/
.vscode/
log/
build/
dist/
GIT
```

```bash
cat > mks_bus.py <<'PY'
import inspect
import threading
import time
from dataclasses import dataclass
from pymodbus.client import ModbusSerialClient
try:
    from pymodbus import FramerType
    _FRAMER = FramerType.RTU
except Exception:
    _FRAMER = None
REG_WORKMODE = 130
REG_ENABLE = 243
REG_SPEED = 246
REG_POS_ABS = 245
REG_AXIS_ZERO = 146
REG_RELEASE_PROTECT = 61
MODE_SR_OPEN = 3
MODE_SR_VFOC = 5
COUNTS_PER_REV = 16384

def _id_kw(method):
    try:
        params = inspect.signature(method).parameters
        for key in ('device_id', 'unit', 'slave'):
            if key in params:
                return key
    except Exception:
        pass
    return None

def _with_unit(method, unit_id: int):
    kw = _id_kw(method)
    return {kw: unit_id} if kw else {}

def _split_i32_to_u16s(val: int):
    val &= 4294967295
    return [val >> 16 & 65535, val & 65535]

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

@dataclass
class BusCfg:
    port: str
    baud: int = 38400
    timeout: float = 0.35
    retries: int = 3
    inter_delay: float = 0.002

class MksBus:

    def __init__(self, cfg: BusCfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        if _FRAMER is not None:
            self.client = ModbusSerialClient(port=cfg.port, framer=_FRAMER, baudrate=cfg.baud, bytesize=8, parity='N', stopbits=1, timeout=cfg.timeout, retries=0)
        else:
            self.client = ModbusSerialClient(method='rtu', port=cfg.port, baudrate=cfg.baud, bytesize=8, parity='N', stopbits=1, timeout=cfg.timeout)

    def connect(self):
        if not self.client.connect():
            raise RuntimeError(f'Cannot open Modbus port: {self.cfg.port}')

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    def _ensure_connected(self):
        try:
            if hasattr(self.client, 'connected') and (not self.client.connected):
                self.client.connect()
        except Exception:
            pass

    def _retry(self, call, err_ctx: str):
        last_exc = None
        attempts = max(1, int(self.cfg.retries) + 1)
        for i in range(attempts):
            try:
                self._ensure_connected()
                with self.lock:
                    rr = call()
                if hasattr(rr, 'isError') and rr.isError():
                    raise RuntimeError(f'{err_ctx}: {rr}')
                if self.cfg.inter_delay and self.cfg.inter_delay > 0:
                    time.sleep(self.cfg.inter_delay)
                return rr
            except Exception as e:
                last_exc = e
                try:
                    self.client.close()
                except Exception:
                    pass
                try:
                    self.client.connect()
                except Exception:
                    pass
                time.sleep(0.02 * (i + 1))
        raise RuntimeError(f'{err_ctx}: {last_exc}')

    def write_reg(self, unit_id: int, addr: int, value: int):

        def _call():
            return self.client.write_register(address=addr, value=int(value) & 65535, **_with_unit(self.client.write_register, unit_id))
        return self._retry(_call, f'write_reg failed unit={unit_id} addr=0x{addr:04X}')

    def write_regs(self, unit_id: int, addr: int, values: list[int]):

        def _call():
            return self.client.write_registers(address=addr, values=[int(v) & 65535 for v in values], **_with_unit(self.client.write_registers, unit_id))
        return self._retry(_call, f'write_regs failed unit={unit_id} addr=0x{addr:04X}')

    def init_servo(self, unit_id: int, mode: int=MODE_SR_VFOC, enable: bool=True):
        self.write_reg(unit_id, REG_WORKMODE, int(mode))
        if enable:
            self.write_reg(unit_id, REG_ENABLE, 1)

    def set_enable(self, unit_id: int, enable: bool):
        self.write_reg(unit_id, REG_ENABLE, 1 if enable else 0)

    def axis_zero(self, unit_id: int):
        self.write_reg(unit_id, REG_AXIS_ZERO, 1)

    def release_locked_rotor(self, unit_id: int):
        self.write_reg(unit_id, REG_RELEASE_PROTECT, 1)

    def set_speed_signed(self, unit_id: int, rpm_signed: float, acc: int=10, invert_dir: bool=False):
        rpm_signed = float(rpm_signed)
        acc = int(clamp(acc, 0, 255))
        rpm_mag = int(clamp(abs(round(rpm_signed)), 0, 3000))
        dir_bit = 0 if rpm_signed >= 0 else 1
        if invert_dir:
            dir_bit ^= 1
        reg0 = (dir_bit & 255) << 8 | acc & 255
        self.write_regs(unit_id, REG_SPEED, [reg0, rpm_mag])

    def move_abs_axis(self, unit_id: int, abs_axis_i32: int, speed_rpm: int=600, acc: int=2):
        speed_rpm = int(clamp(speed_rpm, 0, 3000))
        acc = int(clamp(acc, 0, 65535))
        hi, lo = _split_i32_to_u16s(int(abs_axis_i32))
        self.write_regs(unit_id, REG_POS_ABS, [acc, speed_rpm, hi, lo])

    def clear_error_state(self, unit_id: int, mode: int | None=None):
        try:
            self.set_speed_signed(unit_id, 0.0, acc=0, invert_dir=False)
        except Exception:
            pass
        try:
            self.release_locked_rotor(unit_id)
        except Exception:
            pass
        try:
            self.axis_zero(unit_id)
        except Exception:
            pass
        try:
            self.set_enable(unit_id, False)
            time.sleep(0.05)
        except Exception:
            pass
        if mode is not None:
            try:
                self.write_reg(unit_id, REG_WORKMODE, int(mode))
            except Exception:
                pass
        try:
            self.set_enable(unit_id, True)
        except Exception:
            pass
PY
```

```bash
cat > setup/config.py <<'PY'
import os
from mks_bus import COUNTS_PER_REV, MODE_SR_OPEN, MODE_SR_VFOC

MKS_PORT = os.getenv("MKS_PORT", "/dev/ttyUSB0")
MKS_BAUD = int(os.getenv("MKS_BAUD", "38400"))

MODBUS_TIMEOUT = float(os.getenv("MODBUS_TIMEOUT", "0.35"))
MODBUS_RETRIES = int(os.getenv("MODBUS_RETRIES", "3"))
MODBUS_INTER_DELAY = float(os.getenv("MODBUS_INTER_DELAY", "0.002"))

ID_LEFT = 1
ID_RIGHT = 2
ID_LIFT = 3
ID_LIFT_B = 5
ID_BRUSH = 4
ID_SILO = 8

LIFT_B_SIGN = int(os.getenv("LIFT_B_SIGN", "-1"))

ALL_MOTORS = [ID_LEFT, ID_RIGHT, ID_BRUSH, ID_LIFT, ID_LIFT_B, ID_SILO]

INVERT_LEFT_DIR = False
INVERT_RIGHT_DIR = True

MAX_WHEEL_RPM = float(os.getenv("MAX_WHEEL_RPM", "500"))
WHEEL_ACC_REG = int(os.getenv("WHEEL_ACC_REG", "3"))
WHEEL_UPDATE_HZ = float(os.getenv("WHEEL_UPDATE_HZ", "50"))
WHEEL_ACCEL_RPM_PER_S = float(os.getenv("WHEEL_ACCEL_RPM_PER_S", "120"))
WHEEL_DECEL_RPM_PER_S = float(os.getenv("WHEEL_DECEL_RPM_PER_S", "80"))
STOP_DECEL_RPM_PER_S = float(os.getenv("STOP_DECEL_RPM_PER_S", "500"))

ZERO_HOLD_SEC = float(os.getenv("ZERO_HOLD_SEC", "0.10"))
WATCHDOG_SEC = float(os.getenv("WATCHDOG_SEC", "0.8"))

POS_SPEED_RPM = int(os.getenv("POS_SPEED_RPM", "600"))
POS_ACC = int(os.getenv("POS_ACC", "2"))
POS_SPEED_RPM_SILO = int(os.getenv("POS_SPEED_RPM_SILO", "300"))

BRUSH_TURNS = int(os.getenv("BRUSH_TURNS", "5"))
LIFT_TURNS = int(os.getenv("LIFT_TURNS", "11"))
SILO_TURNS = int(os.getenv("SILO_TURNS", "1"))

TURNS_BY_ID = {
    ID_BRUSH: BRUSH_TURNS,
    ID_LIFT: LIFT_TURNS,
    ID_LIFT_B: LIFT_TURNS,
    ID_SILO: SILO_TURNS,
}

MODE_BY_ID = {
    ID_LEFT: MODE_SR_VFOC,
    ID_RIGHT: MODE_SR_VFOC,
    ID_BRUSH: MODE_SR_VFOC,
    ID_LIFT: MODE_SR_VFOC,
    ID_LIFT_B: MODE_SR_VFOC,
    ID_SILO: MODE_SR_OPEN,
}

COUNTS_PER_REV_CONST = COUNTS_PER_REV
PY
```

```bash
cat > setup/models.py <<'PY'
from pydantic import BaseModel, Field

class DriveCmd(BaseModel):
    x: float = Field(0.0, ge=-1.0, le=1.0)
    y: float = Field(0.0, ge=-1.0, le=1.0)

class PosCmd(BaseModel):
    pct: float = Field(..., ge=0.0, le=100.0)
PY
```

```bash
cat > setup/utils.py <<'PY'
from setup.config import TURNS_BY_ID, COUNTS_PER_REV_CONST

def pct_to_axis(motor_id: int, pct: float) -> int:
    pct = max(0.0, min(100.0, float(pct)))
    turns = int(TURNS_BY_ID[motor_id])
    max_axis = turns * COUNTS_PER_REV_CONST
    axis = int(round((pct / 100.0) * max_axis))
    return max(0, min(max_axis, axis))
PY
```

```bash
cat > setup/bus.py <<'PY'
from mks_bus import MksBus, BusCfg
from setup.config import (
    MKS_PORT,
    MKS_BAUD,
    MODBUS_TIMEOUT,
    MODBUS_RETRIES,
    MODBUS_INTER_DELAY,
    ALL_MOTORS,
    MODE_BY_ID,
)

bus = MksBus(BusCfg(
    port=MKS_PORT,
    baud=MKS_BAUD,
    timeout=MODBUS_TIMEOUT,
    retries=MODBUS_RETRIES,
    inter_delay=MODBUS_INTER_DELAY,
))

_connected = False
_bus_error = ""
_init_errors = {}

def connect_and_init():
    global _connected, _bus_error, _init_errors
    _init_errors = {}
    try:
        bus.connect()
        _connected = True
        _bus_error = ""
    except Exception as e:
        _connected = False
        _bus_error = str(e)
        return
    for mid in ALL_MOTORS:
        try:
            bus.init_servo(mid, mode=MODE_BY_ID[mid])
        except Exception as e:
            _init_errors[str(mid)] = str(e)

def get_bus_status():
    return {
        "connected": _connected,
        "bus_error": _bus_error,
        "init_errors": _init_errors,
        "mks_port": MKS_PORT,
        "mks_baud": MKS_BAUD,
    }
PY
```

```bash
cat > tracks/controller.py <<'PY'
from __future__ import annotations

import time
import threading
from dataclasses import dataclass

from mks_bus import clamp
from setup.bus import bus
from setup.config import (
    ID_LEFT,
    ID_RIGHT,
    INVERT_LEFT_DIR,
    INVERT_RIGHT_DIR,
    MAX_WHEEL_RPM,
    WHEEL_ACC_REG,
    WHEEL_UPDATE_HZ,
    WHEEL_ACCEL_RPM_PER_S,
    WHEEL_DECEL_RPM_PER_S,
    STOP_DECEL_RPM_PER_S,
    ZERO_HOLD_SEC,
    WATCHDOG_SEC,
    ALL_MOTORS,
    MODE_BY_ID,
)

_state_lock = threading.Lock()

_last_drive_ts = 0.0
desired_left = 0.0
desired_right = 0.0
_stop_fast_active = False
_pause_wheel_tx_until = 0.0

_last_err_left = ""
_last_err_right = ""
_last_err_ts = 0.0


def sign(x: float) -> int:
    if x > 0:
        return 1
    if x < 0:
        return -1
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

        if self.reversing:
            self.actual = ramp_toward(self.actual, 0.0, decel_rate, dt)
            if self.actual == 0.0:
                if now < self.zero_until:
                    return
                self.reversing = False
            else:
                return

        if cur_s != 0 and des_s != 0 and cur_s != des_s:
            self.reversing = True
            self.zero_until = now + ZERO_HOLD_SEC
            self.actual = ramp_toward(self.actual, 0.0, decel_rate, dt)
            return

        rate = decel_rate if abs(self.desired) < abs(self.actual) else WHEEL_ACCEL_RPM_PER_S
        self.actual = ramp_toward(self.actual, self.desired, rate, dt)


wheel_L = WheelChannel()
wheel_R = WheelChannel()


def _send_wheels(left_rpm: float, right_rpm: float):
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
    global _last_drive_ts, desired_left, desired_right, _stop_fast_active, _pause_wheel_tx_until

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
            pause_tx = now < _pause_wheel_tx_until

            if stop_fast:
                desired_left = 0.0
                desired_right = 0.0
                dl = 0.0
                dr = 0.0
            else:
                dl = desired_left
                dr = desired_right

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

        if not pause_tx:
            _send_wheels(wheel_L.actual, wheel_R.actual)

        if stop_fast and wheel_L.actual == 0.0 and wheel_R.actual == 0.0:
            with _state_lock:
                _stop_fast_active = False


_wheel_thread = threading.Thread(target=wheel_control_loop, daemon=True)


def start_wheel_thread():
    if not _wheel_thread.is_alive():
        _wheel_thread.start()


def drive_from_joystick(x: float, y: float) -> dict:
    global desired_left, desired_right, _last_drive_ts

    with _state_lock:
        if _stop_fast_active:
            return {"ok": True, "stopping": True, "left_rpm": float(wheel_L.actual), "right_rpm": float(wheel_R.actual)}

    dl = (y - x) * MAX_WHEEL_RPM
    dr = (y + x) * MAX_WHEEL_RPM
    dl = clamp(dl, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)
    dr = clamp(dr, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)

    with _state_lock:
        desired_left = float(dl)
        desired_right = float(dr)
        _last_drive_ts = time.time()
        al = float(wheel_L.actual)
        ar = float(wheel_R.actual)

    return {"ok": True, "left_rpm": al, "right_rpm": ar}


def stop_soft() -> dict:
    global desired_left, desired_right, _last_drive_ts, _stop_fast_active
    with _state_lock:
        desired_left = 0.0
        desired_right = 0.0
        _last_drive_ts = time.time()
        _stop_fast_active = False
    return {"ok": True}


def stop_fast() -> dict:
    global desired_left, desired_right, _last_drive_ts, _stop_fast_active
    with _state_lock:
        desired_left = 0.0
        desired_right = 0.0
        _last_drive_ts = time.time()
        _stop_fast_active = True

    for _ in range(3):
        _send_wheels(0.0, 0.0)
        time.sleep(0.02)

    return {"ok": True}


def clear_errors_all() -> dict:
    global desired_left, desired_right, _last_drive_ts, _stop_fast_active, _pause_wheel_tx_until
    global _last_err_left, _last_err_right, _last_err_ts

    now = time.time()

    with _state_lock:
        desired_left = 0.0
        desired_right = 0.0
        wheel_L.actual = 0.0
        wheel_R.actual = 0.0
        wheel_L.reversing = False
        wheel_R.reversing = False
        _stop_fast_active = True
        _last_drive_ts = now
        _pause_wheel_tx_until = now + 0.9

    for _ in range(3):
        _send_wheels(0.0, 0.0)
        time.sleep(0.02)

    results: dict[str, str] = {}
    for mid in ALL_MOTORS:
        try:
            bus.clear_error_state(mid, mode=MODE_BY_ID[mid])
            results[str(mid)] = "ok"
        except Exception as e:
            results[str(mid)] = str(e)

    _last_err_left = ""
    _last_err_right = ""
    _last_err_ts = time.time()

    with _state_lock:
        _stop_fast_active = False
        _last_drive_ts = time.time()

    return {"ok": True, "results": results}


def get_tracks_status() -> dict:
    with _state_lock:
        return {
            "stop_fast_active": _stop_fast_active,
            "pause_wheel_tx_until": _pause_wheel_tx_until,
            "desired_left": desired_left,
            "desired_right": desired_right,
            "actual_left": wheel_L.actual,
            "actual_right": wheel_R.actual,
            "last_err_left": _last_err_left,
            "last_err_right": _last_err_right,
            "last_err_ts": _last_err_ts,
        }
PY
```

```bash
cat > tracks/api.py <<'PY'
from fastapi import APIRouter
from setup.models import DriveCmd
from setup.bus import get_bus_status
from tracks.controller import (
    drive_from_joystick,
    stop_soft,
    stop_fast,
    clear_errors_all,
    get_tracks_status,
)

router = APIRouter()

@router.get("/api/status")
def api_status():
    s = get_tracks_status()
    s.update(get_bus_status())
    return s

@router.post("/api/drive")
def api_drive(cmd: DriveCmd):
    return drive_from_joystick(cmd.x, cmd.y)

@router.post("/api/stop")
def api_stop():
    return stop_soft()

@router.post("/api/stop_fast")
def api_stop_fast():
    return stop_fast()

@router.post("/api/clear_errors")
def api_clear_errors():
    return clear_errors_all()
PY
```

```bash
cat > lift/api.py <<'PY'
from fastapi import APIRouter, HTTPException
from setup.models import PosCmd
from setup.utils import pct_to_axis
from setup.bus import bus
from setup.config import ID_LIFT, ID_LIFT_B, LIFT_B_SIGN, POS_SPEED_RPM, POS_ACC

router = APIRouter()

@router.post("/api/motor/3/pos")
def m3_pos(cmd: PosCmd):
    try:
        axis = pct_to_axis(ID_LIFT, cmd.pct)
        axis_b = int(LIFT_B_SIGN * axis)
        bus.move_abs_axis(ID_LIFT, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
        bus.move_abs_axis(ID_LIFT_B, axis_b, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
        return {"ok": True, "lift_pct": cmd.pct, "axis": axis, "axis_b": axis_b}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/api/motor/3/zero")
def m3_zero():
    try:
        bus.axis_zero(ID_LIFT)
        bus.axis_zero(ID_LIFT_B)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
PY
```

```bash
cat > brush/api.py <<'PY'
from fastapi import APIRouter, HTTPException
from setup.models import PosCmd
from setup.utils import pct_to_axis
from setup.bus import bus
from setup.config import ID_BRUSH, POS_SPEED_RPM, POS_ACC

router = APIRouter()

@router.post("/api/motor/4/pos")
def m4_pos(cmd: PosCmd):
    try:
        axis = pct_to_axis(ID_BRUSH, cmd.pct)
        bus.move_abs_axis(ID_BRUSH, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
        return {"ok": True, "motor": 4, "pct": cmd.pct}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/api/motor/4/zero")
def m4_zero():
    try:
        bus.axis_zero(ID_BRUSH)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
PY
```

```bash
cat > silo/api.py <<'PY'
from fastapi import APIRouter, HTTPException
from setup.models import PosCmd
from setup.utils import pct_to_axis
from setup.bus import bus
from setup.config import ID_SILO, POS_SPEED_RPM_SILO, POS_ACC

router = APIRouter()

@router.post("/api/motor/8/pos")
def m8_pos(cmd: PosCmd):
    try:
        axis = pct_to_axis(ID_SILO, cmd.pct)
        bus.move_abs_axis(ID_SILO, axis, speed_rpm=POS_SPEED_RPM_SILO, acc=POS_ACC)
        return {"ok": True, "motor": 8, "pct": cmd.pct}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/api/motor/8/zero")
def m8_zero():
    try:
        bus.axis_zero(ID_SILO)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
PY
```

```bash
cat > main.py <<'PY'
from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from setup.bus import connect_and_init
from tracks.controller import start_wheel_thread

from tracks.api import router as tracks_router
from lift.api import router as lift_router
from brush.api import router as brush_router
from silo.api import router as silo_router

app = FastAPI(title="Robot Web Modular")

app.mount("/static", StaticFiles(directory="ui/static"), name="static")

@app.on_event("startup")
def startup():
    connect_and_init()
    start_wheel_thread()

@app.get("/")
def root():
    return FileResponse("ui/static/index.html")

app.include_router(tracks_router)
app.include_router(lift_router)
app.include_router(brush_router)
app.include_router(silo_router)
PY
```

```bash
cat > ui/static/index.html <<'HTML'
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Robot Web Modular</title>
  <style>
    body { font-family: system-ui, Arial, sans-serif; margin: 18px; }
    .row { display:flex; gap:18px; flex-wrap: wrap; }
    .card { border: 1px solid #ddd; border-radius: 12px; padding: 14px; min-width: 290px; }
    .btns { display:flex; gap:10px; flex-wrap: wrap; margin-top:10px; }
    button { padding:10px 14px; border-radius:10px; border:1px solid #ccc; background:#f7f7f7; cursor:pointer; }
    button.danger { background:#ffecec; border-color:#ffb5b5; }
    button.primary { background:#eef6ff; border-color:#b7d6ff; }
    input[type="range"] { width: 100%; }
    pre { background:#111; color:#0f0; padding:10px; border-radius:10px; overflow:auto; max-height:260px; }
    .small { color:#666; font-size: 12px; }
  </style>
</head>
<body>
  <h1>Robot Web Modular</h1>
  <p class="small">
    Tracks (ID1+ID2) joystick X/Y, and sliders for Lift(ID3), Brush(ID4), Silo(ID8).
  </p>

  <div class="row">
    <div class="card">
      <h2>Tracks (Joystick)</h2>

      <label>X</label>
      <input id="jx" type="range" min="-1" max="1" step="0.01" value="0">

      <label>Y</label>
      <input id="jy" type="range" min="-1" max="1" step="0.01" value="0">

      <div class="btns">
        <button class="primary" id="btnStop">Stop</button>
        <button class="danger" id="btnStopFast">Stop Fast</button>
        <button class="danger" id="btnClear">Clear Errors</button>
      </div>
    </div>

    <div class="card">
      <h2>Lift (Motor 3)</h2>
      <input id="lift" type="range" min="0" max="100" step="1" value="0">
      <div class="btns">
        <button id="liftSend" class="primary">Send</button>
        <button id="liftZero">Zero</button>
      </div>
    </div>

    <div class="card">
      <h2>Brush (Motor 4)</h2>
      <input id="brush" type="range" min="0" max="100" step="1" value="0">
      <div class="btns">
        <button id="brushSend" class="primary">Send</button>
        <button id="brushZero">Zero</button>
      </div>
    </div>

    <div class="card">
      <h2>Silo (Motor 8)</h2>
      <input id="silo" type="range" min="0" max="100" step="1" value="0">
      <div class="btns">
        <button id="siloSend" class="primary">Send</button>
        <button id="siloZero">Zero</button>
      </div>
    </div>

    <div class="card" style="flex:1; min-width: 320px;">
      <h2>Status</h2>
      <pre id="status">{}</pre>
      <div class="btns">
        <button id="btnRefresh" class="primary">Refresh</button>
      </div>
    </div>
  </div>

<script>
const API = "";
let driveTimer = null;

async function postJSON(path, obj) {
  const r = await fetch(API + path, {
    method: "POST",
    headers: {"Content-Type":"application/json"},
    body: JSON.stringify(obj || {})
  });
  if(!r.ok) throw new Error(await r.text());
  return await r.json();
}

async function getJSON(path) {
  const r = await fetch(API + path);
  if(!r.ok) throw new Error(await r.text());
  return await r.json();
}

function startDriveLoop() {
  if (driveTimer) return;
  driveTimer = setInterval(async () => {
    const x = parseFloat(document.getElementById("jx").value);
    const y = parseFloat(document.getElementById("jy").value);
    try { await postJSON("/api/drive", {x, y}); } catch (e) {}
  }, 50);
}

async function refreshStatus() {
  try {
    const s = await getJSON("/api/status");
    document.getElementById("status").textContent = JSON.stringify(s, null, 2);
  } catch (e) {
    document.getElementById("status").textContent = String(e);
  }
}

document.getElementById("btnStop").onclick = async () => { await postJSON("/api/stop"); await refreshStatus(); };
document.getElementById("btnStopFast").onclick = async () => { await postJSON("/api/stop_fast"); await refreshStatus(); };
document.getElementById("btnClear").onclick = async () => { await postJSON("/api/clear_errors"); await refreshStatus(); };
document.getElementById("btnRefresh").onclick = refreshStatus;

document.getElementById("liftSend").onclick  = async () => { await postJSON("/api/motor/3/pos", {pct: parseFloat(lift.value)}); };
document.getElementById("brushSend").onclick = async () => { await postJSON("/api/motor/4/pos", {pct: parseFloat(brush.value)}); };
document.getElementById("siloSend").onclick  = async () => { await postJSON("/api/motor/8/pos", {pct: parseFloat(silo.value)}); };

document.getElementById("liftZero").onclick  = async () => { await postJSON("/api/motor/3/zero"); };
document.getElementById("brushZero").onclick = async () => { await postJSON("/api/motor/4/zero"); };
document.getElementById("siloZero").onclick  = async () => { await postJSON("/api/motor/8/zero"); };

document.getElementById("jx").addEventListener("input", startDriveLoop);
document.getElementById("jy").addEventListener("input", startDriveLoop);

setInterval(refreshStatus, 1000);
refreshStatus();
</script>
</body>
</html>
HTML
```

## 5. Run

```bash
ls -l /dev/serial/by-id/
```

```bash
cd ~/robot_web_modular
source venv/bin/activate

LIFT_B_SIGN=-1 MKS_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 MKS_BAUD=38400 uvicorn main:app --host 0.0.0.0 --port 8000
```

To switch the second lift motor direction:

```bash
LIFT_B_SIGN=1
```

## 6. GitHub

```bash
cd ~/robot_web_modular
git init
git add .
git commit -m "Initial commit"
git remote add origin <YOUR_GITHUB_REPO_URL>
git branch -M main
git push -u origin main
```

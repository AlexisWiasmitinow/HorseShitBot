# Horseshitbot — Robot Web Control (MKS SERVO57D Modbus + Differential Drive + Camera)

This project lets you control **4x MKS SERVO57D** motors via **RS485/Modbus RTU** from a **Raspberry Pi**, with a simple **web interface**:
- **Motor ID 1 & 2** = robot wheels (**differential drive**)
- **Motor ID 3** = **Brush** (position 0–360°)
- **Motor ID 4** = **Lift** (position 0–360°)
- **Camera live stream** embedded in the website (uStreamer on port 8080)

It includes:
- A FastAPI backend (`app.py`) that talks Modbus RTU over USB-RS485
- A static web UI (`static/index.html`) with joystick + sliders + camera
- A Modbus helper (`mks_bus.py`) compatible with pymodbus v2 + v3

---

## 0) Hardware checklist (Modbus RS485)
Make sure all motors share:
- **Same baudrate** (example: `38400`)
- **Unique Modbus addresses** (example: `1, 2, 3, 4`)
RS485 wiring:
- RS485 is a **bus**: connect all motors **A-to-A** and **B-to-B** (parallel bus).
- Use a **USB-RS485 adapter** to the Raspberry Pi.

Your detected RS485 device on the Pi (example):
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`

---

## 1) SSH into the Raspberry Pi

From your PC:

```bash
ssh horseshitbot@192.168.30.40
```

---

## 2) Create a NEW project folder (from scratch)

We will create a clean folder (new name):

```bash
mkdir -p ~/horseshitbot_robot_web_v2/static
cd ~/horseshitbot_robot_web_v2
```

Check it:

```bash
pwd
ls -la
```

---

## 3) Python environment + install packages

### create a new venv 
If you want a new venv inside the project:

```bash
sudo apt update
sudo apt install -y python3-venv
python3 -m venv .venv
source .venv/bin/activate
```

### Install required Python packages
```bash
pip install -U fastapi uvicorn pymodbus pyserial
```

---

## 4) Install and run camera streaming (uStreamer)

### Install (once)
```bash
sudo snap install ustreamer
sudo snap connect ustreamer:camera
```

### Run the camera (Terminal #1)
```bash
ustreamer --device=/dev/video1 --host=0.0.0.0 --port=8080
```

Test in your browser:
- `http://192.168.30.40:8080/stream`

---

## 5) Find your RS485 device path

On the Pi:

```bash
ls -l /dev/serial/by-id/
```

Example output:
```
usb-1a86_USB_Serial-if00-port0 -> ../../ttyUSB0
```

So the stable port is:
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`

---

## 6) Create the project files (beginner step-by-step)

### 6.1 Create `mks_bus.py`

Command:

```bash
nano ~/horseshitbot_robot_web_v2/mks_bus.py
```

Paste this FULL file:

```python
import inspect
import threading
from dataclasses import dataclass

from pymodbus.client import ModbusSerialClient

# pymodbus v3 uses FramerType, v2 uses method="rtu"
try:
    from pymodbus import FramerType
    _FRAMER = FramerType.RTU
except Exception:
    _FRAMER = None

# ----------------- MKS SERVO57D registers -----------------
REG_WORKMODE = 0x0082
MODE_SR_VFOC = 5

REG_ENABLE   = 0x00F3
REG_SPEED    = 0x00F6            # [dir<<8|acc, speed_rpm]
REG_POS_ABS  = 0x00F5            # [acc, speed, axis_hi, axis_lo]
REG_AXIS_ZERO = 0x0092

COUNTS_PER_REV = 0x4000          # 360° == 0x4000


def _id_kw(method):
    try:
        params = inspect.signature(method).parameters
        for key in ("device_id", "unit", "slave"):
            if key in params:
                return key
    except Exception:
        pass
    return None


def _with_unit(method, unit_id: int):
    kw = _id_kw(method)
    return {kw: unit_id} if kw else {}


def _split_i32_to_u16s(val: int):
    # signed i32 represented in two u16 words
    val &= 0xFFFFFFFF
    return [(val >> 16) & 0xFFFF, val & 0xFFFF]


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


@dataclass
class BusCfg:
    port: str
    baud: int = 38400
    timeout: float = 0.25
    retries: int = 1


class MksBus:
    def __init__(self, cfg: BusCfg):
        self.cfg = cfg
        self.lock = threading.Lock()

        if _FRAMER is not None:
            # pymodbus v3
            self.client = ModbusSerialClient(
                port=cfg.port,
                framer=_FRAMER,
                baudrate=cfg.baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=cfg.timeout,
                retries=cfg.retries,
            )
        else:
            # pymodbus v2
            self.client = ModbusSerialClient(
                method="rtu",
                port=cfg.port,
                baudrate=cfg.baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=cfg.timeout,
            )

    def connect(self):
        if not self.client.connect():
            raise RuntimeError(f"Cannot open Modbus port: {self.cfg.port}")

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    def write_reg(self, unit_id: int, addr: int, value: int):
        with self.lock:
            rr = self.client.write_register(
                address=addr,
                value=int(value) & 0xFFFF,
                **_with_unit(self.client.write_register, unit_id),
            )
        if rr.isError():
            raise RuntimeError(f"write_reg failed unit={unit_id} addr=0x{addr:04X}: {rr}")

    def write_regs(self, unit_id: int, addr: int, values: list[int]):
        with self.lock:
            rr = self.client.write_registers(
                address=addr,
                values=[int(v) & 0xFFFF for v in values],
                **_with_unit(self.client.write_registers, unit_id),
            )
        if rr.isError():
            raise RuntimeError(f"write_regs failed unit={unit_id} addr=0x{addr:04X}: {rr}")

    # --------- High-level helpers ---------
    def init_servo(self, unit_id: int):
        self.write_reg(unit_id, REG_WORKMODE, MODE_SR_VFOC)
        self.write_reg(unit_id, REG_ENABLE, 1)

    def axis_zero(self, unit_id: int):
        self.write_reg(unit_id, REG_AXIS_ZERO, 1)

    def set_speed_signed(self, unit_id: int, rpm_signed: float, acc: int = 10, invert_dir: bool = False):
        """
        Speed mode: REG_SPEED (0x00F6) write 2 regs:
          reg0 = (dir<<8) | acc
          reg1 = speed_rpm (0..3000)
        Convention here: rpm>=0 => dir=0, rpm<0 => dir=1
        """
        rpm_signed = float(rpm_signed)
        acc = int(clamp(acc, 0, 255))
        rpm_mag = int(clamp(abs(round(rpm_signed)), 0, 3000))

        dir_bit = 0 if rpm_signed >= 0 else 1
        if invert_dir:
            dir_bit ^= 1

        reg0 = ((dir_bit & 0xFF) << 8) | (acc & 0xFF)
        self.write_regs(unit_id, REG_SPEED, [reg0, rpm_mag])

    def move_abs_axis(self, unit_id: int, abs_axis_i32: int, speed_rpm: int = 600, acc: int = 2):
        """
        Position mode absolute axis: REG_POS_ABS (0x00F5) write 4 regs:
          [acc, speed_rpm, axis_hi, axis_lo]
        axis is signed i32.
        """
        speed_rpm = int(clamp(speed_rpm, 0, 3000))
        acc = int(clamp(acc, 0, 65535))
        hi, lo = _split_i32_to_u16s(int(abs_axis_i32))
        self.write_regs(unit_id, REG_POS_ABS, [acc, speed_rpm, hi, lo])
```

Save in nano:
- `CTRL+O` then Enter
- `CTRL+X` to exit

---

### 6.2 Create `app.py` (backend)

Command:

```bash
nano ~/horseshitbot_robot_web_v2/app.py
```

Paste this FULL file:

```python
import os
import time
import threading
from dataclasses import dataclass
from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from mks_bus import MksBus, BusCfg, clamp, COUNTS_PER_REV

# ---------------- CONFIG ----------------
MKS_PORT = os.getenv("MKS_PORT", "/dev/ttyUSB0")
MKS_BAUD = int(os.getenv("MKS_BAUD", "38400"))

ID_LEFT  = 1
ID_RIGHT = 2
ID_M3    = 3
ID_M4    = 4

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
    ID_M3: {"last_deg": 0.0, "last_axis": 0},
    ID_M4: {"last_deg": 0.0, "last_axis": 0},
}


def init_all():
    for mid in (ID_LEFT, ID_RIGHT, ID_M3, ID_M4):
        bus.init_servo(mid)


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


@app.post("/api/motor/3/pos")
def m3_pos(cmd: PosCmd):
    axis = compute_consistent_axis(ID_M3, cmd.deg)
    bus.move_abs_axis(ID_M3, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
    return {"ok": True, "motor": 3, "deg": cmd.deg}


@app.post("/api/motor/4/pos")
def m4_pos(cmd: PosCmd):
    axis = compute_consistent_axis(ID_M4, cmd.deg)
    bus.move_abs_axis(ID_M4, axis, speed_rpm=POS_SPEED_RPM, acc=POS_ACC)
    return {"ok": True, "motor": 4, "deg": cmd.deg}


@app.post("/api/motor/3/zero")
def m3_zero():
    bus.axis_zero(ID_M3)
    pos_state[ID_M3]["last_deg"] = 0.0
    pos_state[ID_M3]["last_axis"] = 0
    return {"ok": True}


@app.post("/api/motor/4/zero")
def m4_zero():
    bus.axis_zero(ID_M4)
    pos_state[ID_M4]["last_deg"] = 0.0
    pos_state[ID_M4]["last_axis"] = 0
    return {"ok": True}
```

Save (`CTRL+O`, Enter, `CTRL+X`)

---

### 6.3 Create the website file `static/index.html`

Command:

```bash
nano ~/horseshitbot_robot_web_v2/static/index.html
```

Paste this FULL file:

```html
<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Robot Control</title>

  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Montserrat:wght@600;700&family=Raleway:wght@400;500&display=swap" rel="stylesheet">

  <style>
    :root{
      --odan-blue:#005B9C; --odan-blue-dark:#00385F; --odan-yellow:#FFA500;
      --border: rgba(0, 56, 95, 0.18); --shadow: 0 18px 45px rgba(0,0,0,0.08);
      --radius: 18px;
    }
    *{ box-sizing:border-box; }
    body{
      margin:0;
      background:
        radial-gradient(1200px 600px at 10% 0%, rgba(0,91,156,0.10), transparent 60%),
        radial-gradient(900px 500px at 95% 5%, rgba(255,165,0,0.12), transparent 60%),
        #fff;
      color:#424242;
      font-family:"Raleway", system-ui, sans-serif;
    }
    .topbar{
      position:sticky; top:0; z-index:10;
      backdrop-filter: blur(10px);
      background: rgba(255,255,255,0.72);
      border-bottom: 1px solid var(--border);
    }
    .topbar-inner{
      max-width:1100px; margin:0 auto; padding:14px 18px;
      display:flex; align-items:center; justify-content:space-between; gap:12px;
    }
    .brand{ display:flex; align-items:center; gap:12px; min-width:0; }
    .brand img{ height:36px; width:auto; display:block; }
    .title{
      font-family:"Montserrat", system-ui, sans-serif;
      font-weight:700; color:var(--odan-blue-dark);
      letter-spacing:.3px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }

    .wrap{ max-width:1100px; margin:0 auto; padding:22px 18px 40px; }
    .grid{ display:grid; grid-template-columns: 1.15fr 0.85fr; gap:18px; align-items:start; }
    .card{
      background: rgba(255,255,255,0.9);
      border:1px solid var(--border);
      border-radius: var(--radius);
      box-shadow: var(--shadow);
      padding:18px;
    }
    h2{
      margin:0 0 12px 0;
      font-family:"Montserrat", system-ui, sans-serif;
      font-weight:700; color: var(--odan-blue-dark);
      font-size:18px;
    }

    .btn{
      border:0; border-radius:12px; padding:10px 14px;
      font-family:"Montserrat", system-ui, sans-serif; font-weight:700;
      cursor:pointer; user-select:none;
      background: linear-gradient(180deg, #ffffff, #f3f6fb);
      color: var(--odan-blue-dark);
      border: 1px solid var(--border);
    }

    /* Joystick */
    .joy-row{ display:flex; gap:18px; flex-wrap:wrap; align-items:center; }
    #pad{
      width:320px; height:320px; border-radius:22px;
      position:relative; touch-action:none; overflow:hidden;
      background:
        radial-gradient(180px 180px at 50% 50%, rgba(0,91,156,0.10), rgba(0,56,95,0.02)),
        linear-gradient(180deg, #fff, #f7fbff);
      border:1px solid var(--border);
    }
    #knob{
      width:84px; height:84px; border-radius:999px;
      position:absolute; left:50%; top:50%;
      transform: translate(-50%,-50%);
      background: radial-gradient(circle at 30% 30%, rgba(255,255,255,0.55), transparent 35%),
                  linear-gradient(180deg, var(--odan-blue), var(--odan-blue-dark));
      box-shadow: 0 18px 35px rgba(0,0,0,0.18);
      border: 1px solid rgba(255,255,255,0.25);
    }
    .readouts{ min-width:260px; display:flex; flex-direction:column; gap:10px; }
    .kv{
      display:flex; justify-content:space-between; gap:14px;
      border:1px solid var(--border); background:#fff;
      border-radius:14px; padding:10px 12px; font-size:14px;
    }
    .kv b{ font-family:"Montserrat", system-ui, sans-serif; font-weight:700; color:var(--odan-blue-dark); }
    .kv code{
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      background: rgba(0,91,156,0.07);
      padding:2px 8px; border-radius:10px;
      border:1px solid rgba(0,91,156,0.12);
    }

    /* Sliders */
    .slider{ margin-top:14px; padding-top:14px; border-top: 1px dashed rgba(0,56,95,0.20); }
    .slider:first-of-type{ border-top:none; padding-top:0; margin-top:0; }
    .slider label{
      display:block; margin-bottom:8px;
      font-family:"Montserrat", system-ui, sans-serif; font-weight:700;
      color: var(--odan-blue-dark);
    }
    input[type="range"]{ width:100%; accent-color: var(--odan-blue); }
    .slider-meta{
      display:flex; align-items:center; justify-content:space-between;
      gap:12px; margin-top:10px; opacity:.8; font-size:13px;
    }

    /* Camera */
    .cam{ width:100%; border-radius:16px; border:1px solid var(--border); overflow:hidden; background:#000; }
    .cam img{ width:100%; display:block; }

    @media (max-width: 940px){
      .grid{ grid-template-columns:1fr; }
      #pad{ width:min(360px, 100%); height:min(360px, 72vw); }
    }
  </style>
</head>

<body>
  <div class="topbar">
    <div class="topbar-inner">
      <div class="brand">
        <img src="/static/logo.png" alt="Logo" />
        <div class="title">Robot Control</div>
      </div>
    </div>
  </div>

  <div class="wrap">
    <div class="grid">
      <div class="card">
        <h2>Joystick</h2>

        <div class="joy-row">
          <div id="pad"><div id="knob"></div></div>

          <div class="readouts">
            <div class="kv"><b>Left RPM</b> <code id="lRpm">—</code></div>
            <div class="kv"><b>Right RPM</b> <code id="rRpm">—</code></div>
            <div class="kv"><b>Status</b> <code id="status">ready</code></div>
          </div>
        </div>
      </div>

      <div class="card">
        <h2>Position (0–360°)</h2>

        <div class="slider">
          <label for="m3">Motor 3 — Brush</label>
          <input id="m3" type="range" min="0" max="360" step="1" value="0">
          <div class="slider-meta">
            <span>Target: <b><span id="m3v">0</span>°</b></span>
            <div style="display:flex; gap:10px;">
              <button class="btn" id="m3Zero" type="button">Zero</button>
              <button class="btn" id="m3Go" type="button">Go</button>
            </div>
          </div>
        </div>

        <div class="slider">
          <label for="m4">Motor 4 — Lift</label>
          <input id="m4" type="range" min="0" max="360" step="1" value="0">
          <div class="slider-meta">
            <span>Target: <b><span id="m4v">0</span>°</b></span>
            <div style="display:flex; gap:10px;">
              <button class="btn" id="m4Zero" type="button">Zero</button>
              <button class="btn" id="m4Go" type="button">Go</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div style="height:18px;"></div>

    <div class="card">
      <h2>Camera Live</h2>
      <div class="cam"><img id="camImg" alt="camera stream"></div>
    </div>
  </div>

  <script>
    function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

    async function postJSON(url, obj){
      const r = await fetch(url, {
        method: "POST",
        headers: {"Content-Type":"application/json"},
        body: obj ? JSON.stringify(obj) : null
      });
      let data = {};
      try{ data = await r.json(); } catch(e){}
      if(!r.ok) throw new Error(data.error || r.statusText);
      return data;
    }

    // Camera URL = same host, port 8080
    const camImg = document.getElementById("camImg");
    camImg.src = `${location.protocol}//${location.hostname}:8080/stream`;

    const pad = document.getElementById("pad");
    const knob = document.getElementById("knob");

    const lRpm = document.getElementById("lRpm");
    const rRpm = document.getElementById("rRpm");
    const status = document.getElementById("status");

    const m3 = document.getElementById("m3");
    const m4 = document.getElementById("m4");
    const m3v = document.getElementById("m3v");
    const m4v = document.getElementById("m4v");
    const m3Zero = document.getElementById("m3Zero");
    const m4Zero = document.getElementById("m4Zero");
    const m3Go = document.getElementById("m3Go");
    const m4Go = document.getElementById("m4Go");

    let x = 0, y = 0;
    let dragging = false;
    let lastSend = 0;

    let sendTimer = null;

    function startSendLoop(){
      if (sendTimer) return;
      sendTimer = setInterval(() => {
        sendDrive(0);
      }, 100);
    }

    function stopSendLoop(){
      if (!sendTimer) return;
      clearInterval(sendTimer);
      sendTimer = null;
    }

    function setKnob(nx, ny){
      const rect = pad.getBoundingClientRect();
      const cx = rect.width / 2;
      const cy = rect.height / 2;
      const maxR = rect.width * 0.35;
      const px = cx + nx * maxR;
      const py = cy + ny * maxR;
      knob.style.left = `${px}px`;
      knob.style.top  = `${py}px`;
      knob.style.transform = "translate(-50%, -50%)";
    }

    async function sendDrive(throttleMs = 70){
      const now = performance.now();
      if (throttleMs > 0 && (now - lastSend < throttleMs)) return;
      lastSend = now;

      try{
        const res = await postJSON("/api/drive", {x, y});
        lRpm.textContent = (res.left_rpm ?? "—").toString();
        rRpm.textContent = (res.right_rpm ?? "—").toString();
        status.textContent = "ok";
      }catch(e){
        status.textContent = "ERR";
      }
    }

    function centerJoystick(send=true){
      x = 0; y = 0;
      setKnob(0, 0);
      if (send) sendDrive(0);
    }

    function updateFromPointer(e){
      const rect = pad.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top  + rect.height / 2;
      const dx = e.clientX - cx;
      const dy = e.clientY - cy;
      const maxR = rect.width * 0.35;

      let nx = clamp(dx / maxR, -1, 1);
      let ny = clamp(-dy / maxR, -1, 1);

      x = nx; y = ny;
      setKnob(x, -y);
    }

    pad.addEventListener("pointerdown", (e) => {
      dragging = true;
      pad.setPointerCapture(e.pointerId);
      updateFromPointer(e);
      startSendLoop();
      sendDrive(0);
    });

    pad.addEventListener("pointermove", (e) => {
      if (!dragging) return;
      updateFromPointer(e);
      sendDrive();
    });

    function releaseJoystick(){
      dragging = false;
      stopSendLoop();
      centerJoystick(true);
    }

    pad.addEventListener("pointerup", releaseJoystick);
    pad.addEventListener("pointercancel", releaseJoystick);

    window.addEventListener("blur", async () => {
      stopSendLoop();
      dragging = false;
      centerJoystick(false);
      try { await postJSON("/api/stop"); } catch(e){}
    });

    function refreshSliderLabels(){
      m3v.textContent = m3.value;
      m4v.textContent = m4.value;
    }
    m3.addEventListener("input", refreshSliderLabels);
    m4.addEventListener("input", refreshSliderLabels);

    m3Go.addEventListener("click", async () => {
      try{ await postJSON("/api/motor/3/pos", {deg: Number(m3.value)}); status.textContent = "M3 ok"; }
      catch(e){ status.textContent = "ERR"; }
    });

    m4Go.addEventListener("click", async () => {
      try{ await postJSON("/api/motor/4/pos", {deg: Number(m4.value)}); status.textContent = "M4 ok"; }
      catch(e){ status.textContent = "ERR"; }
    });

    m3Zero.addEventListener("click", async () => {
      try{ await postJSON("/api/motor/3/zero"); m3.value = 0; refreshSliderLabels(); status.textContent = "M3 zero"; }
      catch(e){ status.textContent = "ERR"; }
    });

    m4Zero.addEventListener("click", async () => {
      try{ await postJSON("/api/motor/4/zero"); m4.value = 0; refreshSliderLabels(); status.textContent = "M4 zero"; }
      catch(e){ status.textContent = "ERR"; }
    });

    centerJoystick(false);
    refreshSliderLabels();
  </script>
</body>
</html>
```

---

## 7) Upload your logo into the website folder

We expect the logo file name to be `logo.png` in:
`~/horseshitbot_robot_web_v2/static/logo.png`

### Windows PowerShell example
Replace the local path:

```powershell
scp "C:\PATH\TO\YOUR\logo.png" horseshitbot@192.168.30.40:~/horseshitbot_robot_web_v2/static/logo.png
```

### Mac/Linux example
```bash
scp "/path/to/logo.png" horseshitbot@192.168.30.40:~/horseshitbot_robot_web_v2/static/logo.png
```

---

## 8) Run the backend server (Terminal #2)

Activate your venv first:

```bash
cd ~/horseshitbot_robot_web_v2
source ~/mks_env/bin/activate
```

Run uvicorn (use YOUR RS485 device path):

```bash
MKS_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
MKS_BAUD=38400 \
MAX_WHEEL_RPM=500 \
uvicorn app:app --host 0.0.0.0 --port 8000
```

Open the website:
- `http://192.168.30.40:8000`

---

## 9) Common tuning (optional)

### Make it even smoother (slower ramps)
Example:

```bash
MKS_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
MKS_BAUD=38400 \
MAX_WHEEL_RPM=500 \
WHEEL_ACCEL_RPM_PER_S=80 \
WHEEL_DECEL_RPM_PER_S=80 \
ZERO_HOLD_SEC=0.15 \
uvicorn app:app --host 0.0.0.0 --port 8000
```

---

## 10) Troubleshooting

### “Could not import module app”
```bash
cd ~/horseshitbot_robot_web_v2
ls -la
python3 -c "import app; print('IMPORT OK')"
```

### Permission denied on ttyUSB0
```bash
sudo usermod -aG dialout horseshitbot
sudo reboot
```


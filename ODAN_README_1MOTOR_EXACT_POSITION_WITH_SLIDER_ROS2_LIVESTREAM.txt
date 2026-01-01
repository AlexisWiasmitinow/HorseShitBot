ODAN ROS2 (Ubuntu 24.04 on Raspberry Pi)
Motor (MKS Modbus) + USB Camera Live Stream + Website Slider
============================================================



For EVERY code file you will do:
1) run the exact nano command I give
2) paste the code (Ctrl+Shift+V to paste in terminal nano)
3) save: Ctrl+O then Enter
4) exit: Ctrl+X

What you will get
-----------------
Start everything with ONE command:
- Website UI (Brush slider + STOP):     http://<PI_IP>:8000
- Live camera stream (MJPEG):           http://<PI_IP>:8080/stream?topic=/camera/image_raw
- ROS bridge WebSocket:                  ws://<PI_IP>:9090

ROS interface
-------------
Website publishes:
- Topic: /odan/brush (std_msgs/msg/Float32) value 0..1
  Mapping: 0.0 -> 0°  |  1.0 -> 360°

STOP button calls:
- Service: /odan/stop (std_srvs/srv/Trigger)

Assumptions (edit later if different)
-------------------------------------
- OS: Ubuntu 24.04 on Raspberry Pi
- ROS2 distro: Jazzy
- USB webcam: /dev/video1 (we verify)
- Motor USB-RS485: /dev/ttyUSB0 (verify)
- Modbus baudrate: 38400
- MSTEP=16, full steps=200 -> pulses/rev = 200*16 = 3200
- No endstop: when motor node starts, current position is considered 0°.

SAFETY WARNING
--------------
A motor can move unexpectedly. Keep hands clear. Start with low accel/speed.

===============================================================================
STEP 0 — Update Ubuntu + install basics
===============================================================================

On the Pi:

  sudo apt update
  sudo apt install -y git nano curl wget unzip       python3-pip python3.12-venv       v4l-utils

===============================================================================
STEP 1 — Install ROS2 Jazzy (Ubuntu 24.04)
===============================================================================

1) Locales:

  sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

2) Add ROS2 apt repo:

  sudo apt install -y curl gnupg lsb-release
  sudo mkdir -p /usr/share/keyrings

  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

3) Install ROS + build tools:

  sudo apt update
  sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep

4) Init rosdep:

  sudo rosdep init || true
  rosdep update

5) Install camera + streaming + rosbridge packages:

  sudo apt install -y \
    ros-jazzy-v4l2-camera \
    ros-jazzy-web-video-server \
    ros-jazzy-rosbridge-suite

6) Auto-source ROS in every terminal:

  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  source /opt/ros/jazzy/setup.bash

===============================================================================
STEP 2 — Permissions (camera + serial)
===============================================================================

Add your user to groups:

  sudo usermod -aG video $USER
  sudo usermod -aG dialout $USER

Reboot:

  sudo reboot

After reboot, SSH back and check:

  groups | tr ' ' '\n' | egrep 'video|dialout'

===============================================================================
STEP 3 — Confirm device names (camera + motor)
===============================================================================

A) USB camera:

  v4l2-ctl --list-devices

You should see your USB cam:
  ... USB Webcam ...
      /dev/video1
      /dev/video2

Check formats to confirm which one is real:

  v4l2-ctl -d /dev/video1 --list-formats-ext | head -n 80
  v4l2-ctl -d /dev/video2 --list-formats-ext | head -n 80

Usually /dev/video1 has MJPG/YUYV formats. /dev/video2 is often empty.

B) Motor serial device:

  ls -la /dev/ttyUSB*

If you have /dev/ttyUSB1 instead of ttyUSB0, remember it for later.

===============================================================================
STEP 4 — Python venv for pymodbus (Ubuntu 24.04 safe)
===============================================================================

Ubuntu 24.04 blocks system-wide pip by default (PEP 668). Use a venv.

Create venv:

  mkdir -p ~/odan_ros_ws/.venvs
  python3 -m venv ~/odan_ros_ws/.venvs/ros

Activate venv:

  source ~/odan_ros_ws/.venvs/ros/bin/activate

IMPORTANT: after activating venv, source ROS:

  source /opt/ros/jazzy/setup.bash

Install libraries:

  python -m pip install --upgrade pip
  pip install pymodbus pyserial

Test:

  python -c "import pymodbus, serial; print('OK', pymodbus.__version__)"

IMPORTANT RULE (memorize this)
------------------------------
Before building/running, ALWAYS do:
  source ~/odan_ros_ws/.venvs/ros/bin/activate
  source /opt/ros/jazzy/setup.bash

===============================================================================
STEP 5 — Create workspace + packages
===============================================================================

(venv active) + ROS sourced:

  source ~/odan_ros_ws/.venvs/ros/bin/activate
  source /opt/ros/jazzy/setup.bash

Create workspace:

  mkdir -p ~/odan_ros_ws/src
  cd ~/odan_ros_ws/src

Create packages:

  ros2 pkg create odan_motor --build-type ament_python --dependencies rclpy std_msgs std_srvs
  ros2 pkg create odan_bringup --build-type ament_python --dependencies launch launch_ros

Create web folder:

  mkdir -p ~/odan_ros_ws/web

===============================================================================
STEP 6 — Create/modify code files (NANO + COPY-PASTE)
===============================================================================

---------------------------------------
6.1) Motor node: odan_motor/motor_node.py
---------------------------------------

Command:

  nano ~/odan_ros_ws/src/odan_motor/odan_motor/motor_node.py

Paste this FULL file:

#!/usr/bin/env python3
import inspect
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pymodbus.client import ModbusSerialClient

# ---- MKS registers (matches your working mapping) ----
REG_WORKMODE          = 0x0082
REG_MB_RTU_ENABLE     = 0x008E
REG_SET_AXIS_ZERO     = 0x0092
REG_MOTOR_ENABLE      = 0x00F3
REG_EMERGENCY_STOP    = 0x00F7
REG_POSMODE2_ABS_PULS = 0x00FE
MODE_SR_VFOC = 5

def _id_kw(method):
    try:
        params = inspect.signature(method).parameters
        for key in ("device_id", "unit", "slave"):
            if key in params:
                return key
    except Exception:
        pass
    return None

def _u32_to_regs_be(v: int):
    v &= 0xFFFFFFFF
    return [(v >> 16) & 0xFFFF, v & 0xFFFF]

class MKSServo:
    def __init__(self, port: str, baud: int, slave_id: int, timeout: float = 0.3):
        self.slave_id = slave_id
        self.client = ModbusSerialClient(
            port=port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=timeout
        )
        self._kw_wr  = _id_kw(self.client.write_register)
        self._kw_wrs = _id_kw(self.client.write_registers)
        self.lock = threading.Lock()
        self.last_pulses = None

    def connect(self):
        if not self.client.connect():
            raise RuntimeError(
                f"Cannot open Modbus port {self.client.port}. "
                f"Check /dev/ttyUSB* and that you are in the dialout group."
            )

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    def _wr(self, addr: int, value: int):
        kwargs = {self._kw_wr: self.slave_id} if self._kw_wr else {}
        return self.client.write_register(addr, value & 0xFFFF, **kwargs)

    def _wrs(self, addr: int, values):
        kwargs = {self._kw_wrs: self.slave_id} if self._kw_wrs else {}
        values16 = [int(v) & 0xFFFF for v in values]
        return self.client.write_registers(addr, values16, **kwargs)

    def init_and_zero_on_start(self):
        # No endstop: define current position as 0° at startup
        with self.lock:
            self._wr(REG_MB_RTU_ENABLE, 1)
            self._wr(REG_WORKMODE, MODE_SR_VFOC)
            self._wr(REG_MOTOR_ENABLE, 1)
            self._wr(REG_SET_AXIS_ZERO, 1)
            self.last_pulses = None

    def emergency_stop(self):
        with self.lock:
            self._wr(REG_EMERGENCY_STOP, 1)

    def goto_abs_pulses(self, pulses: int, acc: int, speed_rpm: int):
        pulses = int(max(0, min(0xFFFFFFFF, pulses)))
        acc = int(max(0, min(255, acc)))
        speed_rpm = int(max(0, min(3000, speed_rpm)))

        with self.lock:
            if self.last_pulses == pulses:
                return
            regs = [acc, speed_rpm] + _u32_to_regs_be(pulses)
            self._wrs(REG_POSMODE2_ABS_PULS, regs)
            self.last_pulses = pulses

class OdanMotorNode(Node):
    def __init__(self):
        super().__init__("odan_motor")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 38400)
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("mstep", 16)
        self.declare_parameter("acc", 30)
        self.declare_parameter("speed_rpm", 200)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        slave_id = int(self.get_parameter("slave_id").value)
        self.mstep = int(self.get_parameter("mstep").value)
        self.acc = int(self.get_parameter("acc").value)
        self.speed_rpm = int(self.get_parameter("speed_rpm").value)

        self.pulses_per_rev = 200 * self.mstep  # 3200 for mstep=16

        self.mks = MKSServo(port, baud, slave_id)
        self.mks.connect()
        self.mks.init_and_zero_on_start()

        self.get_logger().info(
            f"MKS OK: port={port}, baud={baud}, slave={slave_id}, mstep={self.mstep}, pulses/rev={self.pulses_per_rev}"
        )

        self.create_subscription(Float32, "/odan/brush", self.on_brush, 10)
        self.create_service(Trigger, "/odan/stop", self.on_stop)

    def on_brush(self, msg: Float32):
        v = float(msg.data)
        if v < 0.0: v = 0.0
        if v > 1.0: v = 1.0

        pulses = int(round(v * self.pulses_per_rev))
        pulses = max(0, min(self.pulses_per_rev, pulses))

        try:
            self.mks.goto_abs_pulses(pulses, acc=self.acc, speed_rpm=self.speed_rpm)
        except Exception as e:
            self.get_logger().error(f"Motor command failed: {e}")

    def on_stop(self, request, response):
        try:
            self.mks.emergency_stop()
            response.success = True
            response.message = "Emergency stop sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        try:
            self.mks.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = OdanMotorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

Save + Exit.

---------------------------------------
6.2) odan_motor/setup.py (REPLACE ENTIRE FILE)
---------------------------------------

Command:

  nano ~/odan_ros_ws/src/odan_motor/setup.py

Delete everything and paste this FULL file:

from setuptools import setup

package_name = 'odan_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odan',
    maintainer_email='odon@example.com',
    description='ODAN motor control node (MKS Modbus)',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = odan_motor.motor_node:main',
        ],
    },
)

Save + Exit.

---------------------------------------
6.3) Website file: ~/odan_ros_ws/web/index.html
---------------------------------------

Command:

  nano ~/odan_ros_ws/web/index.html

Paste this FULL file:

<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ODAN ROS Control</title>

  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Montserrat:wght@600;700&family=Raleway:wght@400;500&display=swap" rel="stylesheet">

  <script src="https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js"></script>

  <style>
    :root{
      --odan-blue:#005B9C;
      --odan-blue-dark:#00385F;
      --odan-yellow:#FFA500;
      --border:rgba(0,56,95,0.18);
      --shadow:0 18px 45px rgba(0,0,0,0.08);
      --radius:18px;
      --text:#424242;
    }
    *{ box-sizing:border-box; }
    body{
      margin:0;
      font-family: "Raleway", system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
      color: var(--text);
      background:
        radial-gradient(1200px 600px at 10% 0%, rgba(0,91,156,0.10), transparent 60%),
        radial-gradient(900px 500px at 95% 5%, rgba(255,165,0,0.12), transparent 60%),
        #fff;
    }
    .topbar{
      position:sticky; top:0;
      backdrop-filter: blur(10px);
      background: rgba(255,255,255,0.72);
      border-bottom:1px solid var(--border);
      z-index:10;
    }
    .topbar-inner{
      max-width:900px;
      margin:0 auto;
      padding:14px 18px;
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:14px;
    }
    .brand img{ height:38px; width:auto; display:block; }
    .btn{
      appearance:none; border:0;
      border-radius:12px;
      padding:10px 14px;
      font-family:"Montserrat",system-ui,sans-serif;
      font-weight:700;
      cursor:pointer;
      user-select:none;
      transition: transform .05s ease;
    }
    .btn:active{ transform: translateY(1px); }
    .btn.warn{
      background: linear-gradient(180deg, var(--odan-yellow), #ffb733);
      color:#2b2b2b;
    }
    .wrap{ max-width:900px; margin:0 auto; padding:22px 18px 40px; }
    .card{
      background: rgba(255,255,255,0.9);
      border:1px solid var(--border);
      border-radius: var(--radius);
      box-shadow: var(--shadow);
      padding:18px;
    }
    h2{
      margin:0 0 12px 0;
      font-family:"Montserrat",system-ui,sans-serif;
      font-weight:700;
      color: var(--odan-blue-dark);
      font-size:18px;
    }
    .row{
      display:flex;
      justify-content:space-between;
      align-items:baseline;
      gap:12px;
      margin-bottom:10px;
    }
    .label{
      font-family:"Montserrat",system-ui,sans-serif;
      font-weight:700;
      color: var(--odan-blue-dark);
    }
    .pill{
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
      background: rgba(0,91,156,0.07);
      border: 1px solid rgba(0,91,156,0.12);
      padding: 6px 10px;
      border-radius: 12px;
      min-width: 140px;
      text-align:right;
    }
    input[type="range"]{ width:100%; accent-color: var(--odan-blue); }
    .camBox{
      margin-top:16px;
      border:1px solid var(--border);
      border-radius:18px;
      overflow:hidden;
      background:#fff;
    }
    .camBox img{ width:100%; display:block; }
    .hint{
      margin-top:10px;
      font-size:12.5px;
      color:#6C757D;
      line-height:1.35;
    }
  </style>
</head>

<body>
  <div class="topbar">
    <div class="topbar-inner">
      <div class="brand">
        <img src="Logo_Odan.jpg" alt="ODAN logo">
      </div>
      <button class="btn warn" id="btnStop" type="button">STOP</button>
    </div>
  </div>

  <div class="wrap">
    <div class="card">
      <h2>Controls</h2>

      <div class="row">
        <div class="label">Brush</div>
        <div class="pill" id="angle">0°</div>
      </div>
      <input id="brush" type="range" min="0" max="1" step="0.001" value="0">

      <div class="row" style="margin-top:16px;">
        <div class="label">ROS Bridge</div>
        <div class="pill" id="rosStatus">offline</div>
      </div>

      <div class="row" style="margin-top:8px;">
        <div class="label">Live Camera</div>
        <div class="pill" id="camStatus">loading…</div>
      </div>

      <div class="camBox">
        <img id="cam" alt="Live camera">
      </div>

      <div class="hint">
        IMPORTANT: open this page using the Pi IP (not localhost).
        Example: http://192.168.30.40:8000
      </div>
    </div>
  </div>

  <script>
    const brush = document.getElementById("brush");
    const angle = document.getElementById("angle");
    const btnStop = document.getElementById("btnStop");
    const rosStatus = document.getElementById("rosStatus");
    const camStatus = document.getElementById("camStatus");
    const cam = document.getElementById("cam");

    function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }
    function updateAngle() {
      const v = clamp(parseFloat(brush.value || "0"), 0, 1);
      angle.textContent = `${Math.round(v * 360)}°`;
      return v;
    }

    // Camera stream from web_video_server
    const CAM_URL = `http://${location.hostname}:8080/stream?topic=/camera/image_raw`;
    cam.src = CAM_URL;
    cam.onload = () => { camStatus.textContent = "online"; };
    cam.onerror = () => { camStatus.textContent = "offline"; };

    // ROS bridge (rosbridge_websocket)
    // IMPORTANT: rosbridge uses ROS1-style type strings:
    //   "std_msgs/Float32" (NOT "std_msgs/msg/Float32")
    //   "std_srvs/Trigger" (NOT "std_srvs/srv/Trigger")
    const ros = new ROSLIB.Ros({ url: `ws://${location.hostname}:9090` });
    ros.on("connection", () => { rosStatus.textContent = "online"; });
    ros.on("close",      () => { rosStatus.textContent = "offline"; });
    ros.on("error",      () => { rosStatus.textContent = "offline"; });

    const brushPub = new ROSLIB.Topic({
      ros: ros,
      name: "/odan/brush",
      messageType: "std_msgs/Float32"
    });

    const stopSrv = new ROSLIB.Service({
      ros: ros,
      name: "/odan/stop",
      serviceType: "std_srvs/Trigger"
    });

    // publish at max ~20 Hz
    let sendTimer = null;
    let lastSent = null;

    function schedulePublish() {
      const v = updateAngle();
      if (lastSent !== null && Math.abs(v - lastSent) < 0.001) return;
      if (sendTimer) return;

      sendTimer = setTimeout(() => {
        sendTimer = null;
        lastSent = clamp(parseFloat(brush.value || "0"), 0, 1);
        brushPub.publish(new ROSLIB.Message({ data: lastSent }));
      }, 50);
    }

    brush.addEventListener("input", schedulePublish);

    btnStop.addEventListener("click", () => {
      brush.value = "0";
      updateAngle();
      stopSrv.callService(new ROSLIB.ServiceRequest({}), (res) => console.log("stop:", res));
    });

    updateAngle();
  </script>
</body>
</html>

Save + Exit.

---------------------------------------
6.4) Bringup launch: odan_bringup/launch/bringup.launch.py
---------------------------------------

First create the folder (if not already done):

  mkdir -p ~/odan_ros_ws/src/odan_bringup/launch

Command:

  nano ~/odan_ros_ws/src/odan_bringup/launch/bringup.launch.py

Paste this FULL file:

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Uses your home directory automatically (no hardcoded username)
    home = os.path.expanduser("~")
    web_dir = os.path.join(home, "odan_ros_ws", "web")

    motor = Node(
        package="odan_motor",
        executable="motor_node",
        name="odan_motor",
        output="screen",
        parameters=[{
            "port": "/dev/ttyUSB0",   # change to /dev/ttyUSB1 if needed
            "baud": 38400,
            "slave_id": 1,
            "mstep": 16,
            "acc": 30,
            "speed_rpm": 200,
        }]
    )

    # NOTE:
    # If your camera prefers MJPG, change:
    #   "pixel_format": "MJPG"
    # and keep output_encoding "bgr8"
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": "/dev/video1",  # change if needed
            "pixel_format": "YUYV",
            "output_encoding": "bgr8",
            "image_size": [640, 480],
        }],
        remappings=[
            ("image_raw", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ]
    )

    web_video = Node(
        package="web_video_server",
        executable="web_video_server",
        output="screen",
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
    )

    web = ExecuteProcess(
        cmd=["python3", "-m", "http.server", "8000", "--directory", web_dir],
        output="screen"
    )

    return LaunchDescription([motor, camera, web_video, rosbridge, web])

Save + Exit.

---------------------------------------
6.5) Bringup setup.py (REPLACE ENTIRE FILE)
---------------------------------------

Command:

  nano ~/odan_ros_ws/src/odan_bringup/setup.py

Delete everything and paste this FULL file:

import os
from glob import glob
from setuptools import setup

package_name = 'odan_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odan',
    maintainer_email='odon@example.com',
    description='ODAN bringup launch (motor + camera + rosbridge + web)',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)

Save + Exit.

===============================================================================
STEP 7 — Add the logo to the website folder
===============================================================================

Your website expects:

  ~/odan_ros_ws/web/Logo_Odan.jpg

Option A (from the Pi, if you already have it somewhere on the Pi):
  cp /path/to/Logo_Odan.jpg ~/odan_ros_ws/web/Logo_Odan.jpg

Option B (copy from Windows PC to the Pi using scp):
PowerShell on your PC:

  scp "C:\Users\NIZAR\Desktop\PYTHON\odan-control-ui\Logo_Odan.jpg" horseshitbot@<PI_IP>:~/odan_ros_ws/web/Logo_Odan.jpg

Replace:
- horseshitbot -> your Pi username
- <PI_IP> -> Pi IP (on Pi: hostname -I)

Check on Pi:

  ls -lh ~/odan_ros_ws/web/Logo_Odan.jpg

===============================================================================
STEP 8 — Build the workspace (IMPORTANT: venv must be active!)
===============================================================================

Run:

  source ~/odan_ros_ws/.venvs/ros/bin/activate
  source /opt/ros/jazzy/setup.bash

Build:

  cd ~/odan_ros_ws
  rosdep install --from-paths src -i -y
  colcon build --symlink-install

Source the built workspace:

  source ~/odan_ros_ws/install/setup.bash

===============================================================================
STEP 9 — Run everything (ONE command)
===============================================================================


In the SAME terminal:
sudo apt update
sudo apt install -y \
  python3-yaml \
  python3-tornado \
  python3-numpy \
  python3-pymongo \
  python3-pil

source ~/odan_ros_ws/.venvs/ros/bin/activate
pip install -U pyyaml tornado numpy pymongo pillow
deactivate


  source ~/odan_ros_ws/.venvs/ros/bin/activate
  source /opt/ros/jazzy/setup.bash
  source ~/odan_ros_ws/install/setup.bash

Run:

  ros2 launch odan_bringup bringup.launch.py

if rosbridge offline Other terminal:
	 connect to the PI and run:
		source /opt/ros/jazzy/setup.bash
                ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p address:=0.0.0.0


===============================================================================
STEP 10 — Open in your browser
===============================================================================

Get Pi IP:

  hostname -I

Example: 192.168.30.40

Open on your PC:
- Website:       http://192.168.30.40:8000
- Camera stream: http://192.168.30.40:8080/stream?topic=/camera/image_raw

===============================================================================
STEP 11 — Quick tests (no browser)
===============================================================================

Open a new terminal:

  source ~/odan_ros_ws/.venvs/ros/bin/activate
  source /opt/ros/jazzy/setup.bash
  source ~/odan_ros_ws/install/setup.bash

Test motor:

  ros2 topic pub /odan/brush std_msgs/msg/Float32 "{data: 0.5}" --once
  ros2 topic pub /odan/brush std_msgs/msg/Float32 "{data: 0.0}" --once

Test STOP:

  ros2 service call /odan/stop std_srvs/srv/Trigger "{}"

See website publishes:

  ros2 topic echo /odan/brush

===============================================================================
TROUBLESHOOTING
===============================================================================

A) Camera stream is black / frozen
----------------------------------
Most common: another process is using /dev/video1.

Check who uses it:

  sudo fuser -v /dev/video1

Kill unwanted processes (keep only what you need):

  sudo kill <PID>
  sudo kill -9 <PID>   # if needed

Confirm camera topic publishes:

  ros2 topic hz /camera/image_raw

B) Motor works from CLI but NOT from website
--------------------------------------------
Almost always roslibjs type strings.
In index.html you MUST have:

  messageType: "std_msgs/Float32"
  serviceType: "std_srvs/Trigger"

NOT:
  "std_msgs/msg/Float32"
  "std_srvs/srv/Trigger"

Check rosbridge is listening:

  ss -lntp | grep :9090

C) Motor serial port is different
---------------------------------
If your adapter is /dev/ttyUSB1, edit bringup.launch.py:
  "port": "/dev/ttyUSB1"

D) Check ports 8000/8080/9090
-----------------------------
  ss -lntp | egrep ':8000|:8080|:9090'

===============================================================================
CLEAN REMOVE (delete project only)
===============================================================================

Stop bringup (Ctrl+C), then:

  rm -rf ~/odan_ros_ws

END

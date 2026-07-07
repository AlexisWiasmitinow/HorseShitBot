# Training — ROS 2 Perception & Autonomous Navigation (Jazzy)

> **Objective**: Teach the trainee a complete autonomous robotics workflow, from field data collection to deployment of the navigation stack (Nav2) and real-time perception (YOLO on Jetson).
> 
> **Reference repo** : `HorseShitBot` (ROS 2, Raspberry Pi / Jetson Nano)
> 
> **Duration** : 1 day — **morning 9:00-12:00** / **afternoon 13:00-17:00**

---

## 1. Existing repo architecture (know the base)

### 1.1. ROS 2 Packages

| Package | Role |
|---------|------|
| `horseshitbot` | Main Python package (ament_python). Contains the nodes, drivers, launch files and the web UI. |
| `horseshitbot_interfaces` | Interface package (CMake). Custom messages and services for actuators and the MKS bus. |

### 1.2. Existing nodes (entry points)

```
mks_bus_node          → Modbus RTU bus (MKS SERVO57D)
wheel_driver_node     → /cmd_vel, odometry, switchable MKS/ODrive backend
lift_node / brush_node / bin_door_node  → actuators with stall-based homing
gamepad_teleop_node   → Bluetooth gamepad (evdev), publishes /cmd_vel
web_dashboard_node    → FastAPI on port 8080 (teleop, parameters, rosbags, video)
status_screen_node    → on-robot ILI9341 240×320 SPI TFT display
bag_recorder_node     → programmatic rosbag2 recording (MCAP format)
lidar_node            → LiDAR interface
```

### 1.3. Repo highlights for the training

- **Programmable rosbags**: two instances `perception_recorder` and `mapping_recorder`. Can be started/stopped via gamepad (button `Options` / `D-Pad Left`) or web dashboard.
- **Already published topics**:
  - `/camera/color/image_raw`
  - `/camera/aligned_depth_to_color/image_raw`
  - `/scan`
  - `/odom`
  - `/tf`
- **Web dashboard**: visualization, rosbag download, gamepad config, MJPEG streaming.
- **Test scripts**: in `test_scripts/`, to validate each sensor/actuator outside ROS.

---

## 2. Morning: Data Collection & Perception (3h00) — 9am-12pm

---

### 09:00 – 09:30 : Introduction & repo exploration (30 min)

**To do together:**
1. `colcon build --packages-select horseshitbot horseshitbot_interfaces`
2. `source install/setup.bash`
3. Launch the robot in "mapping" mode (camera + lidar):
   ```bash
   ros2 launch horseshitbot robot_launch.py enable_camera:=true enable_lidar:=true
   ```
4. Open the dashboard at `http://<ROBOT_IP>:8080` and identify the cards (wheel status, camera, LiDAR, recording).

**Trainee exercise:**
- Draw the ROS 2 architecture on paper: which nodes publish/consume which topics?
- Identify in `src/horseshitbot/launch/robot_launch.py` what each toggle does (`enable_camera`, `enable_mks`, `enable_lidar`).

---

### 09:30 – 10:30 : Teleoperation & rosbag recording (1h00)

**Context:**
The `bag_recorder_node` uses `rosbag2_py.SequentialWriter` to write timestamped MCAP files in `~/rosbags/`.

**Gamepad (Data Frog)**:
- Left stick: driving
- D-Pad Up/Down: lift
- RB / LB: brush / bin door (hold)
- A: E-STOP / B: resume
- Options (Start): `reference_all`
- **Select (Menu)**: toggle `perception` bag
- **D-Pad Left**: toggle `mapping` bag

**Hands-on exercise:**
1. Connect with the gamepad. Check the `/gamepad/status` topic:
   ```bash
   ros2 topic echo /gamepad/status
   ```
2. Drive the robot in the test environment.
3. Start a **mapping** rosbag (LiDAR + `/odom`) via the gamepad.
4. Start a **perception** rosbag (colour + depth camera) via the dashboard or gamepad.
5. Check the files in `~/rosbags/`.
6. **Useful commands**:
   ```bash
   ros2 bag info ~/rosbags/horseshitbot_YYYY-MM-DD_HH-MM-SS
   ros2 bag play ~/rosbags/horseshitbot_YYYY-MM-DD_HH-MM-SS --clock
   ```

**Teacher lecture / Demo:**
- Explain the difference between `perception_recorder` (camera) and `mapping_recorder` (LiDAR + odometry + TF).
- Show the content of `bag_recorder_node.py` (lines ~35-88): the `topic <-> type` mapping and topic groups.

---

### 10:30 – 10:45 : Break (15 min)

---

### 10:45 – 11:30 : Image extraction from rosbag for YOLO (45 min)

**Objective**: Turn the perception rosbag into an annotatable image dataset.

**Ready-to-use script**: `scripts/extract_images_from_bag.py`

Usage:
```bash
python3 scripts/extract_images_from_bag.py ~/rosbags/horseshitbot_2026-... \
  -o dataset/images/train
```

**Trainee exercise:**
- Read the script source to understand `rosbag2_py.SequentialReader`, `deserialize_message`, and the `sensor_msgs/Image` → OpenCV conversion.
- Run it on the perception rosbag recorded this morning.
- Check that a folder `dataset/images/train/frame_000000.jpg` is obtained...

**Expected YOLO format** (to prepare for annotation):
```
dataset/
├── images/
│   ├── train/...
│   └── val/...
└── labels/
    ├── train/...
    └── val/...
```

---

### 11:30 – 12:00 : YOLO annotation & training (30 min)

**Annotation (15 min)**:
- Use **LabelImg** or **Roboflow** to annotate objects in the environment (e.g. `cone`, `obstacle`, `door`).
- Export to YOLOv8 format (`.txt` files with `class x_center y_center width height` normalized).

**Training (15 min — demo / dedicated PC)**:
- Install Ultralytics:
  ```bash
  pip install ultralytics
  ```
- Run a short training (5-10 epochs) locally to validate the pipeline:
  ```python
  from ultralytics import YOLO
  model = YOLO("yolov8n.pt")
  model.train(data="dataset/dataset.yaml", epochs=10, imgsz=640)
  ```
- Discuss model size: `yolov8n` (nano) for fast Jetson inference vs `yolov8s/m/l`.

**Key point:** We do not do the full training here. We validate that the trainee knows how to:
1. Extract images from a rosbag.
2. Annotate in YOLO format.
3. Launch an Ultralytics `train`.
4. Understand metrics (`mAP50`, `precision`, `recall`).

---

### 12:00 – 13:00 : Lunch (1h00)

---

## 3. Afternoon: SLAM & Autonomous Navigation (4h00) — 1pm-5pm

---

### 13:00 – 14:00 : SLAM Toolbox — mapping from the mapping rosbag (1h00)

**Prerequisites** (install if not already done):
```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

**Dedicated technical guide**: `docs/slam_guide.md`
- Why SLAM Toolbox (and not Cartographer / gmapping)
- Optimized configuration for the **YDLidar T-MINI Plus**
- Specific tuning for the robot's **open-loop** odometry
- Troubleshooting

**"Offline mapping" mode** (from the rosbag recorded in the morning):
1. Launch SLAM Toolbox in mapping mode:
   ```bash
   ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
   ```
2. Play the mapping rosbag in `use_sim_time` mode:
   ```bash
   ros2 bag play ~/rosbags/horseshitbot_... --clock
   ```
3. Visualize in RViz2:
   - Add `/map`
   - Add `/scan` (LaserScan)
   - Check that the LiDAR aligns with the odometric trajectory.

**Teaching watchpoints:**
- The `wheel_driver_node` publishes **command-based** odometry (desired RPM → integration), not encoder feedback. This is "open-loop" odometry. Discuss drift and the LiDAR's role in closing it.
- The `base_link → laser` is already published in `robot_launch.py` (lines 106-113). Verify that the offsets (`z=0.15`, `yaw=3.14159`) match the real mount.

**Trainee exercise:**
- Save the map in `.pgm` + `.yaml` format via SLAM Toolbox:
  ```bash
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
    data: '/home/<user>/maps/salle_test'"
  ```
- Copy the `salle_test.pgm` and `salle_test.yaml` files into `src/horseshitbot/maps/` (see `maps/README.md`).
- Open the `.pgm` with an image viewer to understand how SLAM represents occupancy.

---

### 14:00 – 14:15 : Break (15 min)

---

### 14:15 – 15:15 : Nav2 — Autonomous Navigation (1h00)

**Prerequisites**:
```bash
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-nav2-amcl
```

**Teaching content:**

1. **Nav2 architecture** (whiteboard diagram):
   - `Planner Server` (global planner: NavFn / A*)
   - `Controller Server` (local planner: Regulated Pure Pursuit)
   - `Behavior Tree` (XML)
   - `AMCL` (localization on known map)
   - `Costmap 2D` (global + local)

2. **Files provided in the repo**:
   - `src/horseshitbot/config/nav2_params.yaml` — Nav2 servers config (**custom YOLO obstacles** in the `local_costmap`)
   - `src/horseshitbot/launch/nav2_bringup_launch.py` — inclusion of the official `bringup_launch.py`
   - `src/horseshitbot/maps/` — SLAM map storage

3. **Minimal configuration to adapt if needed**:
   - `amcl` : `scan_topic: /scan`, `odom_frame_id: odom`
   - `controller_server` : `FollowPath` (Regulated Pure Pursuit)
   - `global_costmap` : `static_layer` (map) + `obstacle_layer` (LiDAR)
   - `local_costmap` : `obstacle_layer` (LiDAR + **yolo_obstacles** PointCloud2)

4. **Launch Nav2 with the static map**:
   ```bash
   ros2 launch horseshitbot nav2_bringup_launch.py map:=$(pwd)/src/horseshitbot/maps/salle_test.yaml
   ```

**Trainee exercise:**
- Read `config/nav2_params.yaml` and identify the `local_costmap/obstacle_layer/yolo_obstacles` section.
- Launch Nav2 in localization AND navigation modes.
- Send a **Goal Pose** via RViz2 and observe the robot plan and follow the path.
- Understand the difference between the **static** map (`/map`) and the **costmap** (`/global_costmap`, `/local_costmap`).

---

### 15:15 – 16:00 : Perception + navigation integration (45 min)

**Concept**:
Nav2 uses the `local_costmap` to avoid obstacles. YOLO detection can inject 3D obstacle points into this costmap via a `sensor_msgs/PointCloud2` topic.

**Provided node**: `src/horseshitbot/nodes/yolo_detector_node.py`

Architecture:
```
Jetson / Pi
  ├─ yolo_detector_node
  │   ├─ subscribes: /camera/color/image_raw
  │   │             /camera/aligned_depth_to_color/image_raw
  │   │             /camera/aligned_depth_to_color/camera_info
  │   ├─ publishes: /yolo/detections    (String JSON)
  │   └─ publishes: /yolo/obstacles     (PointCloud2) → Nav2 obstacle_layer
  └─ Nav2 (planner + controller + AMCL)
```

The node performs **depth → colour alignment**: for each YOLO bounding box, it reads the depth at the pixel centre, projects to 3D via the intrinsic matrix `K`, and publishes a point cloud. The `nav2_params.yaml` already has the `yolo_obstacles` source configured in the `local_costmap`.

**Trainee exercise:**
- Explore the `yolo_detector_node.py` code.
- Identify the `_pixel_to_3d` function and explain the role of `fx`, `fy`, `cx`, `cy`.
- Launch the node by hand in simulation (if N/A, discuss the approach):
  ```bash
  ros2 run horseshitbot yolo_detector_node --ros-args -p model_path:=~/models/best.pt
  ```
- Check that `/yolo/obstacles` appears in the topic list.

---

### 16:00 – 16:15 : Break (15 min)

---

### 16:15 – 17:00 : Field deployment on Jetson (45 min)

**Objective**: everything must run on the on-robot computer with a single launch.

**1. Optimized YOLO model**

Provided script: `scripts/convert_yolo_tensorrt.sh`
```bash
# Export best.pt → ONNX → TensorRT FP16 (Jetson)
./scripts/convert_yolo_tensorrt.sh ~/models/best.pt
# Result: ~/models/best.engine
```

**Why?** The PyTorch `.pt` is slow on Jetson. The TensorRT `.engine` leverages GPU acceleration.

**2. Integrated launch file**

Provided file: `launch/autonomy_launch.py`

All-in-one field usage:
```bash
ros2 launch horseshitbot autonomy_launch.py map:=~/maps/salle_test.yaml
```

This launch simultaneously starts:
- `robot_launch.py` — base, camera, LiDAR, wheels, gamepad, dashboard, screen
- `yolo_detector_node` — real-time perception
- `nav2_bringup_launch.py` — AMCL, planner, controller, costmaps

**3. In-room deployment checklist**

| # | Check | Command / Method |
|---|--------------|-------------------|
| 1 | Complete TF tree | `ros2 run tf2_tools view_frames` |
| 2 | Active topics | `ros2 topic list` |
| 3 | Camera frequency | `ros2 topic hz /camera/color/image_raw` |
| 4 | LiDAR frequency | `ros2 topic hz /scan` |
| 5 | AMCL state | Check `/amcl_pose` in RViz2 |
| 6 | Autonomous goal | Send via RViz2, observe planning + tracking |

**Important teaching note:**
- The `wheel_driver_node` uses a **pass-through ramping**. Nav2 sends `/cmd_vel` at 20 Hz. The `watchdog_sec` (0.8 s) is compatible, but if the trainee sees stuttering, discuss the throttle on the Nav2 `velocity_smoother` side (`nav2_params.yaml`).
- Odometry is open-loop: AMCL is **essential** to correct drift on the map.

---

## 4. Files created / updated in the repo

| File | Role |
|---------|------|
| `scripts/extract_images_from_bag.py` | JPEG image extraction from perception rosbag |
| `src/horseshitbot/nodes/yolo_detector_node.py` | YOLO inference + depth projection → `PointCloud2` obstacles |
| `src/horseshitbot/config/nav2_params.yaml` | Nav2 configuration with `yolo_obstacles` in `local_costmap` |
| `src/horseshitbot/config/slam_toolbox_config.yaml` | Pre-tuned SLAM Toolbox config for YDLidar T-MINI + open-loop odo |
| `docs/slam_guide.md` | Complete SLAM guide (choices, tuning, troubleshooting) |
| `src/horseshitbot/launch/nav2_bringup_launch.py` | Launches AMCL + Nav2 via the official `bringup_launch.py` |
| `src/horseshitbot/launch/autonomy_launch.py` | **All-in-one field launch** |
| `src/horseshitbot/maps/` | Directory to store SLAM maps (`.pgm` + `.yaml`) |
| `scripts/convert_yolo_tensorrt.sh` | `.pt` → ONNX → TensorRT `.engine` export on Jetson |

---

## 5. Common trainee questions / pitfalls

1. **"Why does my `ros2 bag play` not publish anything in RViz?"**  
   → Check `use_sim_time:=true` on the SLAM/Nav2 side and `--clock` on the `ros2 bag play` side.

2. **"Why doesn't Nav2 receive my odom?"**  
   → Verify that `odom` is indeed the `frame_id` of the `Odometry` message and that the TF `odom → base_link` exists.

3. **"Why is my costmap empty?"**  
   → The LiDAR must publish on `/scan` with the correct `frame_id` (`laser`). Check with `ros2 topic echo /scan | head`.

4. **"Why doesn't the robot follow the path?"**  
   → The `controller_server` publishes on `/cmd_vel`. Check that `wheel_driver_node` is running and nothing else is publishing on `/cmd_vel`.

5. **"Why is YOLO slow on the Jetson?"**  
   → Do not use the raw PyTorch `.pt`. Use the `.engine` TensorRT generated by `convert_yolo_tensorrt.sh`.

6. **"Why don't YOLO obstacles appear in the local_costmap?"**  
   → Check that the `/yolo/obstacles` topic publishes a `PointCloud2` and that its `header.frame_id` is known in the TF tree (e.g. `camera_color_optical_frame`).

---

## 6. Additional resources

- **SLAM Toolbox** : https://github.com/SteveMacenski/slam_toolbox
- **Nav2** : https://navigation.ros.org/
- **Ultralytics YOLOv8** : https://docs.ultralytics.com/
- **RealSense ROS2** : already installed via `ros-$ROS_DISTRO-realsense2-camera`
- **Jetson inference / TensorRT** : `trtexec`, `jetson-inference`

---

## 7. Summary sheet "Morning / Afternoon"

| Phase (time) | Skills acquired | Deliverable |
|-------------------|----------------------|----------|
| 09:00 – 09:30 | ROS2 architecture, launch files, dashboard | Node/topic diagram |
| 09:30 – 10:30 | Gamepad, rosbags, dashboard / CLI | 2 rosbags (perception + mapping) |
| 10:45 – 11:30 | Python rosbag2, cv2, dataset | `dataset/images/` folder |
| 11:30 – 12:00 | LabelImg/Roboflow, Ultralytics | Annotated dataset + validated metrics |
| 13:00 – 14:00 | SLAM Toolbox, LiDAR/odom, TF | `map.pgm` + `map.yaml` |
| 14:15 – 15:15 | Nav2, AMCL, BT, costmap, RViz2 Goal | Autonomous in-room robot |
| 15:15 – 16:00 | YOLO, depth projection, PointCloud2, obstacle_layer | Perceived integration in costmap |
| 16:15 – 17:00 | TensorRT, integrated launch, field checklist | Operational `autonomy_launch.py` |

---

*Training prepared for the `HorseShitBot` repo — ROS 2 Jazzy / Humble.*

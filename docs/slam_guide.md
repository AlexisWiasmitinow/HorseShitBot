# SLAM Guide for HorseShitBot (YDLidar T-MINI Plus)

> **Recommended SLAM** : [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — native ROS 2, maintained by Steve Macenski (Nav2 maintainer).
> 
> **Why not Cartographer?** Cartographer works, but its configuration is verbose (Lua files, pbstream) and less pedagogical for a one-day training.
> **Why not gmapping / hector?** These are ROS 1 packages not officially ported to ROS 2.

---

## 1. Why SLAM Toolbox with the T-MINI Plus?

| Criterion | SLAM Toolbox | Advantage for the robot |
|---------|--------------|------------------------|
| **Compatibility** | Native ROS 2 (`sensor_msgs/LaserScan`) | The `lidar_node` already publishes on `/scan` |
| **Offline mode** | `online_sync_launch.py` + `--clock` | Allows generating a map from the rosbag recorded in the morning |
| **Open-loop odometry** | Robust scan-to-map matching | Corrects the drift of the command-integrated RPM odometry (no absolute encoders) |
| **Output** | `.pgm` + `.yaml` directly | Compatible Nav2 / `map_server` without conversion |
| **Lifelong mapping** | `lifelong_launch.py` available | For mapping large areas over time |

---

## 2. LiDAR Characteristics Used by SLAM

From `lidar_node.py`:

| Parameter | Value | Impact on SLAM |
|-----------|--------|-----------------|
| `frame_id` | `laser` | The TF `base_link → laser` must exist (already in `robot_launch.py`) |
| `topic` | `/scan` | SLAM Toolbox subscribes by default |
| `range_min` | 0.02 m | Very close to the robot → good for detecting rear obstacles |
| `range_max` | 12.0 m | Correct range for indoor / hangar use |
| `angle_offset` | -90° | Compensation for the physical sensor orientation (handled by the driver) |
| `scan_bins` | 720 | Angular resolution 0.5° — very good for a low-cost lidar |

> ⚠️ **Warning** : the `base_link → laser` in `robot_launch.py` has a `yaw=3.14159` (π rad). The lidar is mounted at 180° relative to the front of the robot. SLAM Toolbox does not care: it uses the TF tree to transform the scan into `odom`.

---

## 3. Installation

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

Check that launch files are present:
```bash
ros2 pkg prefix slam_toolbox
# Should output /opt/ros/$ROS_DISTRO/share/slam_toolbox
```

---

## 4. Recommended Configuration (`slam_toolbox_config.yaml`)

Create `src/horseshitbot/config/slam_toolbox_config.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------
    mode: "mapping"          # "mapping", "localization", "lifelong"
    solver_plugin: "solver_plugins::CeresSolver"
    ceres_linear_solver: "SPARSE_SCHUR"
    ceres_preconditioner: "SCHUR_JACOBI"
    ceres_trust_strategy: "LEVENBERG_MARQUARDT"
    ceres_dogleg_type: "TRADITIONAL_DOGLEG"
    ceres_loss_function: "None"

    # ------------------------------------------------------------------
    # Topics
    # ------------------------------------------------------------------
    odom_frame: "odom"
    map_frame: "map"
    base_frame: "base_link"
    scan_topic: "/scan"

    # ------------------------------------------------------------------
    # Scan matching
    # ------------------------------------------------------------------
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.15      # metre (small robot, open-loop odo)
    minimum_travel_heading: 0.15       # radian
    scan_buffer_size: 15
    scan_buffer_maximum_scan_distance: 10.0

    # ------------------------------------------------------------------
    # Loop closure
    # ------------------------------------------------------------------
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5    # tolerance for matching distant scans
    loop_search_maximum_distance: 5.0  # max distance to search a loop
    loop_search_maximum_angle: 3.14    # 180°
    do_loop_closing: true
    loop_match_minimum_chain_size: 3
    loop_match_maximum_variance_coarse: 0.4
    loop_match_minimum_response_coarse: 0.1
    loop_match_minimum_response_fine: 0.1

    # ------------------------------------------------------------------
    # Correspondence (scan-to-map)
    # ------------------------------------------------------------------
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # ------------------------------------------------------------------
    # Map
    # ------------------------------------------------------------------
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349   # ~0.2°
    coarse_search_angle_offset: 0.349    # ~20°
    coarse_angle_resolution: 0.0349       # ~2°
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true

    # ------------------------------------------------------------------
    # Rasterisation
    # ------------------------------------------------------------------
    resolution: 0.05        # 5 cm / pixel
    max_laser_range: 12.0   # do not exceed the lidar range_max
```

### Key points of this config for HorseShitBot

- `minimum_travel_distance: 0.15` : with an open-loop odo that drifts, we want ** lots of overlap** between scans. 15 cm guarantees good overlap without saturating the CPU.
- `link_scan_maximum_distance: 1.5` : the robot is small and operates in potentially narrow spaces; we allow wider matching than the default (0.3).
- `max_laser_range: 12.0` : identical to the driver `range_max`. Do not artificially increase it otherwise rays without return create noise in the map.
- `resolution: 0.05` : good precision / memory weight compromise for a Jetson.

---

## 5. Real-time Mapping (Live)

**Prerequisites** : the robot is powered on, the lidar is running (`/scan` publishes), odometry is active (`/odom`), and the TF `base_link → laser` exists.

```bash
# Launch the robot (if not already done)
ros2 launch horseshitbot robot_launch.py enable_camera:=false

# In another terminal, launch SLAM Toolbox
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=$(pwd)/src/horseshitbot/config/slam_toolbox_config.yaml \
  use_sim_time:=false
```

Visualize in RViz2:
- Topic `/map` (OccupancyGrid)
- Topic `/scan` (LaserScan), colour by intensity if needed
- TF : verify that `odom → base_link → laser` is consistent

Drive the robot slowly (the T-MINI can miss points in fast turns).

---

## 6. Offline Mapping from Rosbag (Recommended for the Training)

This is the mode used during the training (13:00-14:00) because it allows replaying, correcting, and not depending on the robot battery.

```bash
# 1. Launch SLAM Toolbox in simulated mode
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=$(pwd)/src/horseshitbot/config/slam_toolbox_config.yaml \
  use_sim_time:=true

# 2. Play the rosbag (the mapping bag, containing /scan + /odom + /tf)
ros2 bag play ~/rosbags/horseshitbot_YYYY-MM-DD_HH-MM-SS --clock

# 3. RViz2 (optional, to visualize the map being built)
rviz2
```

> ⏱️ A 5-10 minute rosbag of a 10×10 m room is more than enough.

---

## 7. Saving the Map

### Method A : ROS 2 Service (recommended)

```bash
# Create the destination folder
mkdir -p ~/maps

# Call the save service
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: '/home/$(whoami)/maps/salle_test'"
```

Result:
```
~/maps/
├── salle_test.pgm
└── salle_test.yaml
```

### Method B : CLI map_saver (legacy but functional)

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f ~/maps/salle_test
```

---

## 8. Nav2 Integration — From Map to Navigation

1. Copy the `.pgm` + `.yaml` files into the repo:
   ```bash
   cp ~/maps/salle_test.* src/horseshitbot/maps/
   ```
2. This map is then used by `nav2_bringup_launch.py` → `map_server` → AMCL.
   ```bash
   ros2 launch horseshitbot nav2_bringup_launch.py \
     map:=$(pwd)/src/horseshitbot/maps/salle_test.yaml
   ```

---

## 9. Troubleshooting SLAM Toolbox + T-MINI

| Symptom | Probable Cause | Solution |
|----------|----------------|----------|
| **“No laser scan received”** | Missing TF `base_link → laser` | Check `robot_launch.py` (lines 106-113) or `ros2 run tf2_tools view_frames` |
| **Blurry / foggy map** | `minimum_travel_distance` too small + robot too slow / stationary | Increase to 0.25 ; ensure continuous movement |
| **Map doubles** | Open-loop odometry drifts too much between two scans | Drive slower in straight lines ; verify that the lidar scan rate is stable |
| **Failed loop closure** | Space too little textured (white corridor) | Add distinctive objects ; lower `link_scan_maximum_distance` to 0.8 |
| **Ghost points behind the robot** | Very low `range_min` (0.02 m) and reflections | Increase `range_min` to 0.1 m in `lidar_node` (filter) or ignore very short distances |
| **Map too small / cut off** | The rosbag stopped before the end | Continue playing ; or use `--rate 0.5` to slow down and let SLAM catch up |
| **CPU saturated on Jetson** | `resolution: 0.05` is heavy | Switch to `resolution: 0.10` if precision is not critical |

---

## 10. Localization Mode (To Go Further)

Once the map is generated, SLAM Toolbox can be switched to **localization** mode (without modifying the map):

```bash
# Edit slam_toolbox_config.yaml : mode: "localization"
# Add:
# map_file_name: "/home/user/maps/salle_test"
# map_start_pose: [0.0, 0.0, 0.0]

ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=$(pwd)/src/horseshitbot/config/slam_toolbox_config.yaml \
  use_sim_time:=false
```

In this mode, SLAM behaves like AMCL + a slight scan-to-map correction. **Nav2 already provides AMCL**, so this mode is not necessary for the project's autonomous stack, but it is useful to know.

---

*Guide written for the **YDLidar T-MINI Plus** of HorseShitBot — baud 230400, frame `laser`, TF `base_link→laser` provided by the robot launch.*

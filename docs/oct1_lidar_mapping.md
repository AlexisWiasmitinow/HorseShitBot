# October 1, 2026 LiDAR Mapping

This workflow analyzes only:

`mapping_2026-10-01_14-36-31`

It never modifies the source MCAP. Runtime containers use `--network none`,
mount the bag directory read-only, expose no devices, drop Linux capabilities,
and communicate only inside their private network namespace.

## Historical configuration

The repository commit immediately preceding the recording (`a2f72fd`) defines:

- LaserScan frame: `laser`
- Scan geometry: 720 bins, `angle_min=0`, `angle_max=2*pi`
- Static transform: `base_link -> laser`, translation `(0, 0, 0.15) m`,
  RPY `(0, 0, 3.14159) rad`
- Dynamic recorded transform: `odom -> base_link`
- SLAM frames: `map`, `odom`, `base_link`
- SLAM mode: asynchronous online mapping with scan matching and loop closure
- Occupancy-grid resolution: `0.05 m/pixel`

The static transform was configured but `/tf_static` was not captured.

## Scan handedness

The driver bins native increasing sensor angles directly into an output
LaserScan with a positive angle increment. Empirical scan/IMU registration
shows that the physical rays progress clockwise:

`theta_corrected = -theta_recorded`

For an N-element LaserScan that retains a positive angle increment, the
equivalent lossless correction is:

`corrected[j] = recorded[(-j) mod N]`

Index zero and all range values are preserved. This is a reflection, not a yaw
rotation; changing only the static-transform yaw cannot correct it.

## Duplicate handling

The primary bag contains exactly two semantically identical messages for every
unique `/scan`, `/odom`, and `/tf` source identity. Playback retains the first
message for each:

- `/scan` and `/odom`: topic plus source header timestamp
- `/tf`: topic plus every transform's timestamp, parent, and child frame

Playback aborts if the same identity has a different semantic payload.

## Build and run in the isolated mapping container

From a checkout of this repository (with `BAG_PARENT` pointing to the directory
containing `mapping_2026-10-01_14-36-31`):

```bash
REPO_DIR=$PWD \
BAG_PARENT=/home/hsb/rosbags \
OUTPUT_DIR=/home/hsb/hsb_analysis/lidar_validation/oct1_2026 \
bash scripts/run_oct1_mapping_container.sh corrected_icp_odom 1.0
```

Available diagnostic modes:

- `raw`: original scan and recorded open-loop odometry
- `corrected`: reflected scan and recorded open-loop odometry
- `corrected_no_odom`: reflected scan with identity odometry
- `corrected_icp_odom`: reflected scan with diagnostic scan-to-scan ICP odometry

The last mode is intended for visualization and candidate geometry extraction.
Its odometry is derived from the LiDAR and is therefore not independent evidence
of LiDAR accuracy.

## RViz2

From an X11 graphical session:

```bash
REPO_DIR=$PWD \
BAG_PARENT=/home/hsb/rosbags \
OUTPUT_DIR=/home/hsb/hsb_analysis/lidar_validation/oct1_2026 \
bash scripts/run_oct1_rviz_container.sh corrected_icp_odom 1.0
```

This uses the same network-isolated container and scoped Xauthority file. It
does not use host networking or `xhost`, and it exposes no robot devices.

## Ground-truth comparison

Fill:

`src/horseshitbot/config/room_measurements_oct1_template.yaml`

Then run the validator with the selected map YAML:

```bash
python3 scripts/validate_lidar_and_map.py \
  --bag /home/hsb/rosbags/mapping_2026-10-01_14-36-31 \
  --map /path/to/generated-map.yaml \
  --measurements src/horseshitbot/config/room_measurements_oct1_template.yaml \
  --output-dir /home/hsb/hsb_analysis/lidar_validation/final_comparison
```

Raw-scan errors and occupancy-map errors are reported separately. Candidate map
dimensions are not calibration results until matched to identified physical
reference points.

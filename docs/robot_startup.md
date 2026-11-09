# Robot startup and shutdown

## Connect

From a Tailscale-connected computer:

```bash
ssh hsb@100.124.93.20
cd ~/HorseShitBot
```

`~/HorseShitBot` is the conventional production checkout. A controlled
deployment may instead use a dedicated worktree; run startup commands from the
checkout whose `install/` directory and source configuration should be used.

For manual ROS diagnostics, source ROS and the selected workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

The startup scripts source these environments themselves.

## Normal startup

Start without rebuilding:

```bash
./scripts/start.sh
```

This uses the existing `install/` workspace. If it does not exist, the script
builds first. Normal full startup enables the camera, LiDAR, shared MKS motor
bus, battery monitor, TFT screen, web dashboard, IMU, wheel driver, actuators,
gamepad and recorder nodes. Battery monitoring polls through the shared
`mks_bus_node`; it does not open a second serial connection.

After changing source or checking out a new version:

```bash
./scripts/buildstart.sh
```

This runs `colcon build`, sources the resulting workspace, and delegates to
`start.sh`. The equivalent alternative is:

```bash
./scripts/start.sh --rebuild
```

## Supported options

```text
--no-camera   Do not start the RealSense camera. Recorder nodes remain available.
--no-lidar    Do not start the LiDAR node.
--no-battery  Do not start battery_modbus_node; the TFT shows NO DATA.
--no-mks      Do not start the shared MKS bus. Battery monitoring is also disabled.
--drive-only  Start only wheel_driver_node and gamepad_teleop_node.
--rebuild     Build before launching.
```

Use `--no-battery` when diagnosing ADC or shared-bus errors. `--no-mks`
automatically disables battery polling because no second Modbus owner is
allowed. In the current launch architecture, MKS-backed wheel and actuator
nodes are not useful without `mks_bus_node`; `--no-mks` is a diagnostic option,
not a complete alternate-drive configuration.

`--drive-only` does not start the TFT, dashboard, battery monitor, camera,
LiDAR, IMU, recorders or actuators.

## Dashboard and diagnostics

The ROS dashboard is available at:

<http://100.124.93.20:8080>

Remote access to this address requires Tailscale connectivity.

Verify nodes and battery telemetry:

```bash
ros2 node list
ros2 topic echo /battery/status --once
```

`BatteryState.percentage` is a fraction from 0 to 1. It is an indicative,
voltage-derived estimate from an empirical discharge curve—not an independently
calibrated fuel gauge.

## Stop

For a foreground `start.sh` or `buildstart.sh` launch, press `Ctrl+C` in that
terminal and wait for the nodes to exit.

Do not start a second stack while one is active. `start.sh` detects an existing
`ros2 launch horseshitbot` or `ros2 run horseshitbot` process and asks whether
to quit or stop it. Choose quit unless a controlled restart is intended.

No HorseShitBot ROS systemd service is installed on `horseshitbot01` at the
time of this validation. The repository contains
`systemd/robot-web.service`, but it is a legacy Raspberry Pi service pointing
to `/home/pi/horseshitbot_robot_web_clean`, port 8000; it is not installed and
does not launch or restart the current ROS stack on port 8080.

## Troubleshooting

### Dashboard unavailable

Confirm Tailscale connectivity, verify `web_dashboard_node` in
`ros2 node list`, and check that port 8080—not the legacy port 8000—is used.

### TFT blank or unresponsive

Confirm exactly one `status_screen_node` is running. Check its logs for SPI,
GPIO or geometry errors. Do not run standalone TFT tests while the production
node owns the display.

### Battery percentage unavailable

Check:

```bash
ros2 node list | grep battery
ros2 topic echo /battery/status --once
```

The TFT intentionally shows `NO DATA`, `STALE` or `INVALID` rather than
retaining an unreliable value. Verify that `mks_bus_node` is running and that
startup was not passed `--no-battery` or `--no-mks`.

### Modbus errors

Do not open `/dev/mksbus` from another process. Stop standalone Modbus tests,
confirm there is one `mks_bus_node`, and use `--no-battery` to isolate the 1 Hz
ADC request if needed. Motor and actuator communication has priority over
displaying a percentage.

### Duplicate ROS nodes

Stop the extra foreground launch with `Ctrl+C`. Do not use `kill -9` unless a
normal controlled stop has failed and the robot is already safe.

### Wrong workspace sourced

Check:

```bash
echo "$AMENT_PREFIX_PATH"
ros2 pkg prefix horseshitbot
```

The first HorseShitBot prefix should belong to the checkout being deployed.
Open a new shell and source ROS plus the intended `install/setup.bash` if not.

### Source changed but behavior did not

Run `./scripts/buildstart.sh` after the controlled stop. `start.sh` without
`--rebuild` intentionally reuses the existing installation.

### Safe rollback

Do not reset or clean a checkout containing uncommitted work. Create a separate
worktree at the known-good commit, build it there, stop the active stack, and
start from that worktree. Keep the failed checkout intact for diagnosis.

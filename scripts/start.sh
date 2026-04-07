#!/usr/bin/env bash
#
# Start the HorseShitBot ROS 2 stack.
#
# Usage:
#   ./scripts/start.sh                  # launch everything
#   ./scripts/start.sh --no-camera      # skip RealSense + bag recorder
#   ./scripts/start.sh --no-mks         # skip MKS bus node (ODrive-only setup)
#   ./scripts/start.sh --drive-only     # just wheel driver + gamepad (no launch file)
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source the workspace overlay if built
if [ -f "$REPO_DIR/install/setup.bash" ]; then
  source "$REPO_DIR/install/setup.bash"
else
  echo "Workspace not built. Building now..."
  cd "$REPO_DIR"
  colcon build
  source "$REPO_DIR/install/setup.bash"
fi

PARAMS="$REPO_DIR/src/horseshitbot/config/params.yaml"

# Parse args
ENABLE_CAMERA=true
ENABLE_MKS=true
DRIVE_ONLY=false
for arg in "$@"; do
  case "$arg" in
    --no-camera)   ENABLE_CAMERA=false ;;
    --no-mks)      ENABLE_MKS=false ;;
    --drive-only)  DRIVE_ONLY=true ;;
  esac
done

if [ "$DRIVE_ONLY" = true ]; then
  echo "=== HorseShitBot — Drive Only ==="
  echo "  wheel_driver_node + gamepad_teleop_node"
  echo "  Ctrl+C to stop"
  echo ""

  ros2 run horseshitbot wheel_driver_node \
    --ros-args --params-file "$PARAMS" &
  PIDS="$!"

  ros2 run horseshitbot gamepad_teleop_node \
    --ros-args --params-file "$PARAMS" &
  PIDS="$PIDS $!"

  cleanup() {
    echo ""
    echo "Stopping..."
    for pid in $PIDS; do
      kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
  }
  trap cleanup EXIT INT TERM
  wait

else
  echo "=== HorseShitBot — Full Launch ==="
  [ "$ENABLE_CAMERA" = false ] && echo "  (camera disabled)"
  [ "$ENABLE_MKS" = false ]    && echo "  (MKS bus disabled)"
  echo "  Ctrl+C to stop"
  echo ""

  ros2 launch horseshitbot robot_launch.py \
    enable_camera:="$ENABLE_CAMERA" \
    enable_mks:="$ENABLE_MKS"
fi

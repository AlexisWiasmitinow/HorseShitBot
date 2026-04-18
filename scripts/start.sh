#!/usr/bin/env bash
#
# Start the HorseShitBot ROS 2 stack.
#
# Usage:
#   ./scripts/start.sh                  # launch everything
#   ./scripts/start.sh --no-camera      # skip RealSense + bag recorder
#   ./scripts/start.sh --no-mks         # skip MKS bus node (ODrive-only setup)
#   ./scripts/start.sh --no-lidar       # skip lidar node
#   ./scripts/start.sh --drive-only     # just wheel driver + gamepad (no launch file)
#   ./scripts/start.sh --rebuild        # force colcon build before launching
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# ── Source ROS 2 ─────────────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found."
  echo "Install ROS 2 Humble or run:  sudo ./scripts/install.sh"
  exit 1
fi

# ── Guard: check for existing instance ────────────────────────────
EXISTING_PID=$(pgrep -f "ros2 launch horseshitbot|ros2 run horseshitbot" | grep -v "$$" || true)
if [ -n "$EXISTING_PID" ]; then
  echo "HorseShitBot is already running (PID: $EXISTING_PID)."
  echo "  [k] Kill it and restart"
  echo "  [q] Quit"
  read -r -n1 -p "> " choice
  echo ""
  case "$choice" in
    k|K)
      echo "Killing existing instance..."
      echo "$EXISTING_PID" | xargs kill 2>/dev/null || true
      sleep 2
      # Force-kill anything that survived
      echo "$EXISTING_PID" | xargs kill -9 2>/dev/null || true
      sleep 1
      echo "Done."
      ;;
    *)
      echo "Aborted."
      exit 0
      ;;
  esac
fi

# ── Parse args ───────────────────────────────────────────────────
ENABLE_CAMERA=true
ENABLE_MKS=true
ENABLE_LIDAR=true
DRIVE_ONLY=false
FORCE_REBUILD=false
for arg in "$@"; do
  case "$arg" in
    --no-camera)   ENABLE_CAMERA=false ;;
    --no-mks)      ENABLE_MKS=false ;;
    --no-lidar)    ENABLE_LIDAR=false ;;
    --drive-only)  DRIVE_ONLY=true ;;
    --rebuild)     FORCE_REBUILD=true ;;
  esac
done

# ── Build if requested ────────────────────────────────────────────
if [ "$FORCE_REBUILD" = true ] || [ ! -f "$REPO_DIR/install/setup.bash" ]; then
  echo "Building workspace..."
  cd "$REPO_DIR"
  colcon build
  echo ""
fi

source "$REPO_DIR/install/setup.bash"

PARAMS="$REPO_DIR/src/horseshitbot/config/params.yaml"

# ── Launch ───────────────────────────────────────────────────────
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
  [ "$ENABLE_LIDAR" = false ]  && echo "  (lidar disabled)"
  echo "  Ctrl+C to stop"
  echo ""

  ros2 launch horseshitbot robot_launch.py \
    enable_camera:="$ENABLE_CAMERA" \
    enable_mks:="$ENABLE_MKS" \
    enable_lidar:="$ENABLE_LIDAR" \
    params_file:="$PARAMS"
fi

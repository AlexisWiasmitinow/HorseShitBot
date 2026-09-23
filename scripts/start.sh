#!/usr/bin/env bash
#
# Start the HorseShitBot ROS 2 stack.
#
# Usage:
#   ./scripts/start.sh                  # launch everything
#   ./scripts/start.sh --no-camera      # skip RealSense (recorders stay available)
#   ./scripts/start.sh --no-mks         # use ODrive; skip MKS bus and actuators
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
  if [ ! -t 0 ]; then
    echo "ERROR: Cannot replace it from a non-interactive startup."
    exit 1
  fi
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
    *)
      echo "ERROR: Unknown argument: $arg" >&2
      exit 2
      ;;
  esac
done

# ── Build when source changed ────────────────────────────────────
if [ "$FORCE_REBUILD" = true ]; then
  "$SCRIPT_DIR/build_workspace.sh" --force
else
  "$SCRIPT_DIR/build_workspace.sh"
fi

source "$REPO_DIR/install/setup.bash"

PARAMS="$REPO_DIR/src/horseshitbot/config/params.yaml"

# ── Launch ───────────────────────────────────────────────────────
if [ "$DRIVE_ONLY" = true ]; then
  echo "=== HorseShitBot — Drive Only ==="
  if [ "$ENABLE_MKS" = true ]; then
    echo "  mks_bus_node + wheel_driver_node + gamepad_teleop_node"
  else
    echo "  wheel_driver_node (ODrive) + gamepad_teleop_node"
  fi
  echo "  Ctrl+C to stop"
  echo ""

  PIDS=""
  if [ "$ENABLE_MKS" = true ]; then
    ros2 run horseshitbot mks_bus_node \
      --ros-args --params-file "$PARAMS" &
    PIDS="$!"
  fi

  WHEEL_ARGS=(--ros-args --params-file "$PARAMS")
  if [ "$ENABLE_MKS" = false ]; then
    WHEEL_ARGS+=(-p wheel_backend:=odrive)
  fi

  ros2 run horseshitbot wheel_driver_node \
    "${WHEEL_ARGS[@]}" &
  PIDS="$PIDS $!"

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
  [ "$ENABLE_MKS" = false ]    && echo "  (MKS disabled; ODrive wheel backend selected)"
  [ "$ENABLE_LIDAR" = false ]  && echo "  (lidar disabled)"
  echo "  Ctrl+C to stop"
  echo ""

  WHEEL_BACKEND="mks"
  [ "$ENABLE_MKS" = false ] && WHEEL_BACKEND="odrive"

  ros2 launch horseshitbot robot_launch.py \
    enable_camera:="$ENABLE_CAMERA" \
    enable_mks:="$ENABLE_MKS" \
    enable_lidar:="$ENABLE_LIDAR" \
    wheel_backend:="$WHEEL_BACKEND" \
    params_file:="$PARAMS"
fi

#!/usr/bin/env bash
#
# Start the HorseShitBot ROS 2 stack.
#
# Usage:
#   ./scripts/start.sh                  # launch everything
#   ./scripts/start.sh --no-camera      # skip RealSense + bag recorder
#   ./scripts/start.sh --no-mks         # skip MKS bus node (ODrive-only setup)
#   ./scripts/start.sh --no-lidar       # skip lidar node
#   ./scripts/start.sh --no-battery     # skip shared-bus battery monitoring
#   ./scripts/start.sh --drive-only     # just wheel driver + gamepad (no launch file)
#   ./scripts/start.sh --rebuild        # force colcon build before launching
#
set -eo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/start.sh [OPTIONS]

Options:
  --no-camera   Skip RealSense
  --no-mks      Skip the MKS bus node
  --no-lidar    Skip the LiDAR node
  --no-battery  Skip shared-bus battery monitoring
  --drive-only  Launch only the wheel driver and gamepad
  --rebuild     Build the workspace before launching
  -h, --help    Show this help and exit

This script does not implement --dry-run.
EOF
}

# Validate every argument before sourcing ROS, inspecting/killing processes,
# building the workspace, or touching hardware.
ENABLE_CAMERA=true
ENABLE_MKS=true
ENABLE_LIDAR=true
ENABLE_BATTERY=true
DRIVE_ONLY=false
FORCE_REBUILD=false
for arg in "$@"; do
  case "$arg" in
    --no-camera)   ENABLE_CAMERA=false ;;
    --no-mks)      ENABLE_MKS=false ;;
    --no-lidar)    ENABLE_LIDAR=false ;;
    --no-battery)  ENABLE_BATTERY=false ;;
    --drive-only)  DRIVE_ONLY=true ;;
    --rebuild)     FORCE_REBUILD=true ;;
    -h|--help)
      usage
      exit 0
      ;;
    --dry-run)
      echo "ERROR: --dry-run is not implemented; no processes were started." >&2
      exit 2
      ;;
    *)
      echo "ERROR: unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

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

# ── Guard: fail safely if any robot stack is already running ──────
EXISTING_PID=$(pgrep -f \
  "ros2 launch horseshitbot|ros2 run horseshitbot|/install/horseshitbot/lib/horseshitbot/" \
  || true)
if [ -n "$EXISTING_PID" ]; then
  echo "ERROR: HorseShitBot is already running (PID(s): $EXISTING_PID)." >&2
  echo "Stop the existing stack safely before starting another instance." >&2
  exit 3
fi

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
  # battery_modbus_node reads through mks_bus_node's shared service; it must
  # not run without the single MKS serial-bus owner.
  [ "$ENABLE_MKS" = false ] && ENABLE_BATTERY=false

  echo "=== HorseShitBot — Full Launch ==="
  [ "$ENABLE_CAMERA" = false ] && echo "  (camera disabled)"
  [ "$ENABLE_MKS" = false ]    && echo "  (MKS bus disabled)"
  [ "$ENABLE_LIDAR" = false ]  && echo "  (lidar disabled)"
  [ "$ENABLE_BATTERY" = false ] && echo "  (battery monitor disabled)"
  echo "  Ctrl+C to stop"
  echo ""

  ros2 launch horseshitbot robot_launch.py \
    enable_camera:="$ENABLE_CAMERA" \
    enable_mks:="$ENABLE_MKS" \
    enable_lidar:="$ENABLE_LIDAR" \
    enable_battery:="$ENABLE_BATTERY" \
    params_file:="$PARAMS"
fi

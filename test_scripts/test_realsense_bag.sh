#!/usr/bin/env bash
#
# Quick RealSense D415 test → rosbag (MCAP) on Ubuntu 22.04 / ROS 2 Humble.
#
# Usage:
#   chmod +x test_realsense_bag.sh
#   ./test_realsense_bag.sh              # record for 30 seconds (default)
#   ./test_realsense_bag.sh 60           # record for 60 seconds
#   ./test_realsense_bag.sh 0            # record until Ctrl-C
#
set -eo pipefail

DURATION_SEC="${1:-30}"
BAG_DIR="$HOME/rosbags"
STAMP=$(date +%Y-%m-%d_%H-%M-%S)
BAG_NAME="realsense_test_${STAMP}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

TOPICS=(
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/camera_info
)

# ── preflight ──────────────────────────────────────────────────────────────────

check_dep() {
  if ! dpkg -s "$1" &>/dev/null; then
    echo "❌  Missing package: $1"
    echo "   sudo apt install $1"
    return 1
  fi
}

echo "=== RealSense rosbag test ==="
echo ""

# Source ROS 2 if not already sourced
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  if [ -f /opt/ros/humble/setup.bash ]; then
    echo "Sourcing /opt/ros/humble/setup.bash ..."
    source /opt/ros/humble/setup.bash
  else
    echo "❌  Could not find ROS 2 Humble installation at /opt/ros/humble"
    exit 1
  fi
fi

MISSING=0
for pkg in ros-humble-realsense2-camera ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap; do
  check_dep "$pkg" || MISSING=1
done
if [ "$MISSING" -eq 1 ]; then
  echo ""
  echo "Install missing packages first, then re-run."
  exit 1
fi

# Check for a connected RealSense device
if ! command -v rs-enumerate-devices &>/dev/null; then
  echo "⚠  rs-enumerate-devices not found; skipping hardware check."
  echo "   (install librealsense2-utils for this)"
else
  echo ""
  echo "Detecting RealSense devices..."
  RS_DEV=$(rs-enumerate-devices -s 2>/dev/null || true)
  if [ -z "$RS_DEV" ]; then
    echo "❌  No RealSense device found. Is it plugged in (USB 3.x)?"
    exit 1
  fi
  echo "$RS_DEV"
fi

mkdir -p "$BAG_DIR"
echo ""
echo "Bag output:  ${BAG_PATH}"
echo "Topics:      ${TOPICS[*]}"
echo "Duration:    ${DURATION_SEC}s (0 = until Ctrl-C)"
echo ""

# ── launch RealSense node ─────────────────────────────────────────────────────

echo "Starting RealSense camera node..."
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  device_type:=d415 \
  enable_color:=true \
  enable_depth:=true \
  enable_pointcloud:=false \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x6 \
  depth_module.depth_profile:=640x480x6 &
RS_PID=$!

cleanup() {
  echo ""
  echo "Shutting down..."
  # Kill the entire process group so all child processes die
  kill -- -$$ 2>/dev/null || true
  kill "$RS_PID" 2>/dev/null || true
  pkill -f "ros2.bag.record" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Wait for camera topics to appear (max 15 s)
echo "Waiting for camera topics..."
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
    echo "✔  Camera topics detected (${i}×0.5s)"
    break
  fi
  if [ "$i" -eq 30 ]; then
    echo "❌  Timed out waiting for camera topics"
    exit 1
  fi
  sleep 0.5
done

# Quick sanity: print Hz for 3 seconds
echo ""
echo "Checking topic rates (3 s sample)..."
timeout 4 ros2 topic hz /camera/color/image_raw --window 10 || true
echo ""

# ── record rosbag ─────────────────────────────────────────────────────────────

TOPIC_ARGS=""
for t in "${TOPICS[@]}"; do
  TOPIC_ARGS+=" $t"
done

echo "Recording rosbag (${DURATION_SEC}s) ..."
if [ "$DURATION_SEC" -eq 0 ]; then
  ros2 bag record -o "$BAG_PATH" -s mcap $TOPIC_ARGS
else
  ros2 bag record -o "$BAG_PATH" -s mcap $TOPIC_ARGS &
  BAG_PID=$!
  sleep "${DURATION_SEC}"
  kill -INT "$BAG_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
fi

echo ""
echo "=== Done ==="
echo ""

# ── bag info ──────────────────────────────────────────────────────────────────

if [ -d "$BAG_PATH" ] || [ -f "${BAG_PATH}.mcap" ]; then
  ros2 bag info "$BAG_PATH" 2>/dev/null || ros2 bag info "${BAG_PATH}.mcap" 2>/dev/null || true
fi

echo ""
echo "Bag saved to: ${BAG_PATH}"
echo ""
echo "Replay later with:"
echo "  ros2 bag play ${BAG_PATH}"

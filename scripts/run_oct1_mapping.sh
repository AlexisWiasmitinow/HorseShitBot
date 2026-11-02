#!/usr/bin/env bash
# Run one raw or corrected October 1 slam_toolbox reconstruction.

set -eo pipefail

MODE="${1:?usage: run_oct1_mapping.sh raw|corrected|corrected_no_odom|corrected_icp_odom [playback_rate]}"
RATE="${2:-1.0}"
if [[ "$MODE" != "raw" && "$MODE" != "corrected" && "$MODE" != "corrected_no_odom" && "$MODE" != "corrected_icp_odom" ]]; then
  echo "Mode must be raw, corrected, corrected_no_odom, or corrected_icp_odom" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="/output/${MODE}/ros_logs"

BAG="/bags/mapping_2026-10-01_14-36-31"
OUTPUT="/output/${MODE}"
CONFIG="/workspace/src/horseshitbot/config/slam_toolbox_config.yaml"
mkdir -p "$OUTPUT" "$ROS_LOG_DIR"

SLAM_PID=""
TF_DIAG_PID=""
RVIZ_PID=""
service_ready() {
  ros2 service list 2>/dev/null | python3 -c \
    'import sys; raise SystemExit("/slam_toolbox/save_map" not in sys.stdin.read().splitlines())'
}

cleanup() {
  if [[ -n "$SLAM_PID" ]] && kill -0 "$SLAM_PID" 2>/dev/null; then
    kill -TERM "$SLAM_PID" 2>/dev/null || true
    wait "$SLAM_PID" 2>/dev/null || true
  fi
  if [[ -n "$TF_DIAG_PID" ]] && kill -0 "$TF_DIAG_PID" 2>/dev/null; then
    kill -TERM "$TF_DIAG_PID" 2>/dev/null || true
    wait "$TF_DIAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$CONFIG" \
  use_sim_time:=true \
  >"$OUTPUT/slam_toolbox.log" 2>&1 &
SLAM_PID=$!

for _ in $(seq 1 100); do
  if service_ready; then
    break
  fi
  if ! kill -0 "$SLAM_PID" 2>/dev/null; then
    echo "slam_toolbox exited before advertising save_map" >&2
    cat "$OUTPUT/slam_toolbox.log" >&2
    exit 3
  fi
  sleep 0.1
done
if ! service_ready; then
  echo "Timed out waiting for /slam_toolbox/save_map" >&2
  exit 4
fi

if [[ "${ENABLE_RVIZ:-0}" == "1" ]]; then
  rviz2 \
    -d /workspace/src/horseshitbot/config/lidar_mapping_validation.rviz \
    --ros-args -p use_sim_time:=true \
    >"$OUTPUT/rviz2.log" 2>&1 &
  RVIZ_PID=$!
fi

(
  sleep 5
  cd "$OUTPUT"
  timeout 15s ros2 run tf2_tools view_frames \
    --ros-args -p use_sim_time:=true
) >"$OUTPUT/view_frames.log" 2>&1 &
TF_DIAG_PID=$!

python3 /workspace/scripts/oct1_bag_player.py \
  --bag "$BAG" \
  --mode "$MODE" \
  --rate "$RATE" \
  --summary "$OUTPUT/playback_summary.json" \
  >"$OUTPUT/playback.log" 2>&1

# Let callbacks finish without advancing simulated time, then save the final map.
sleep 3
set +e
timeout 30s ros2 service call \
  /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '${OUTPUT}/oct1_${MODE}'}}" \
  >"$OUTPUT/save_map.log" 2>&1
SAVE_STATUS=$?
set -e
if [[ $SAVE_STATUS -ne 0 || ! -f "$OUTPUT/oct1_${MODE}.yaml" ]]; then
  echo "Map save failed for ${MODE}; see ${OUTPUT}/save_map.log" >&2
  exit 5
fi

python3 /workspace/scripts/render_occupancy_map.py \
  --map "$OUTPUT/oct1_${MODE}.yaml" \
  --output "$OUTPUT/oct1_${MODE}.png" \
  --title "October 1 ${MODE} slam_toolbox map (0.05 m configured resolution)"

ros2 topic list >"$OUTPUT/final_topics.txt" 2>&1 || true
wait "$TF_DIAG_PID" 2>/dev/null || true
TF_DIAG_PID=""

if [[ "${KEEP_RVIZ:-0}" == "1" && -n "$RVIZ_PID" ]]; then
  echo "Mapping complete; close RViz2 to exit."
  wait "$RVIZ_PID" || true
  RVIZ_PID=""
fi

cleanup
SLAM_PID=""
trap - EXIT INT TERM
echo "Completed ${MODE} mapping: $OUTPUT"

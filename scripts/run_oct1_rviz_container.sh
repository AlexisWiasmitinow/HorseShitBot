#!/usr/bin/env bash
# Launch isolated October 1 playback, slam_toolbox, and RViz2 in an X11 session.

set -euo pipefail

MODE="${1:-corrected_icp_odom}"
RATE="${2:-1.0}"
REPO_DIR="${REPO_DIR:-$PWD}"
BAG_PARENT="${BAG_PARENT:-/home/hsb/rosbags}"
OUTPUT_DIR="${OUTPUT_DIR:-/home/hsb/hsb_analysis/lidar_validation/oct1_2026}"
IMAGE="${IMAGE:-horseshitbot-lidar-mapping:humble}"

if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
  echo "Run this from a graphical X11 session; DISPLAY/X11 is unavailable." >&2
  exit 2
fi

XAUTHORITY_PATH="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ ! -f "$XAUTHORITY_PATH" ]]; then
  echo "Xauthority file not found: $XAUTHORITY_PATH" >&2
  exit 3
fi

mkdir -p "$OUTPUT_DIR/$MODE"
docker run --rm \
  --network none \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=2g \
  --tmpfs /home/analysis:rw,nosuid,nodev,size=64m \
  --user "$(id -u):$(id -g)" \
  --env HOME=/home/analysis \
  --env DISPLAY="$DISPLAY" \
  --env XAUTHORITY=/home/analysis/.Xauthority \
  --env ENABLE_RVIZ=1 \
  --env KEEP_RVIZ=1 \
  --env ROS_DOMAIN_ID=77 \
  --env ROS_LOCALHOST_ONLY=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --volume "$XAUTHORITY_PATH:/home/analysis/.Xauthority:ro" \
  --volume "$REPO_DIR:/workspace:ro" \
  --volume "$BAG_PARENT:/bags:ro" \
  --volume "$OUTPUT_DIR:/output:rw" \
  "$IMAGE" \
  bash /workspace/scripts/run_oct1_mapping.sh "$MODE" "$RATE"

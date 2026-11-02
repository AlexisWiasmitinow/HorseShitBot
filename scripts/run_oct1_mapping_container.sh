#!/usr/bin/env bash
# Build and run isolated October 1 mapping in a network-isolated container.

set -euo pipefail

MODE="${1:-all}"
RATE="${2:-1.0}"
if [[ "$MODE" != "all" && "$MODE" != "raw" && "$MODE" != "corrected" && "$MODE" != "corrected_no_odom" && "$MODE" != "corrected_icp_odom" ]]; then
  echo "usage: $0 [all|raw|corrected|corrected_no_odom|corrected_icp_odom] [playback_rate]" >&2
  exit 2
fi

IMAGE="${IMAGE:-horseshitbot-lidar-mapping:humble}"
REPO_DIR="${REPO_DIR:-$PWD}"
BAG_PARENT="${BAG_PARENT:-/home/hsb/rosbags}"
OUTPUT_DIR="${OUTPUT_DIR:-/home/hsb/hsb_analysis/lidar_validation/oct1_2026}"

if [[ ! -f "$REPO_DIR/docker/lidar-mapping/Dockerfile" ]]; then
  echo "REPO_DIR does not contain the LiDAR mapping Dockerfile: $REPO_DIR" >&2
  exit 3
fi
if [[ ! -d "$BAG_PARENT/mapping_2026-10-01_14-36-31" ]]; then
  echo "Primary October 1 bag not found below $BAG_PARENT" >&2
  exit 4
fi
mkdir -p "$OUTPUT_DIR"

docker build \
  --file "$REPO_DIR/docker/lidar-mapping/Dockerfile" \
  --tag "$IMAGE" \
  "$REPO_DIR"

run_mode() {
  local selected_mode="$1"
  mkdir -p "$OUTPUT_DIR/$selected_mode"
  docker run --rm \
    --network none \
    --cap-drop ALL \
    --security-opt no-new-privileges \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=2g \
    --tmpfs /home/analysis:rw,nosuid,nodev,size=64m \
    --user "$(id -u):$(id -g)" \
    --env HOME=/home/analysis \
    --env ROS_DOMAIN_ID=77 \
    --env ROS_LOCALHOST_ONLY=1 \
    --volume "$REPO_DIR:/workspace:ro" \
    --volume "$BAG_PARENT:/bags:ro" \
    --volume "$OUTPUT_DIR:/output:rw" \
    "$IMAGE" \
    bash /workspace/scripts/run_oct1_mapping.sh "$selected_mode" "$RATE"
}

if [[ "$MODE" == "all" || "$MODE" == "raw" ]]; then
  run_mode raw
fi
if [[ "$MODE" == "all" || "$MODE" == "corrected" ]]; then
  run_mode corrected
fi
if [[ "$MODE" == "all" || "$MODE" == "corrected_no_odom" ]]; then
  run_mode corrected_no_odom
fi
if [[ "$MODE" == "all" || "$MODE" == "corrected_icp_odom" ]]; then
  run_mode corrected_icp_odom
fi

echo "Outputs: $OUTPUT_DIR"

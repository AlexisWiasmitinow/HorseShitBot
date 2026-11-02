#!/usr/bin/env bash
# Start the browser-based October 1 LiDAR room-map viewer.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP="${MAP:-/home/hsb/hsb_analysis/lidar_validation/slam_toolbox/corrected_icp_odom/oct1_corrected_icp_odom.yaml}"
PORT="${PORT:-8765}"

exec python3 "$REPO_DIR/scripts/view_lidar_room_web.py" \
  --map "$MAP" \
  --bind 127.0.0.1 \
  --port "$PORT"

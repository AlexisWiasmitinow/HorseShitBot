#!/usr/bin/env bash
#
# Build and start the HorseShitBot ROS 2 stack.
# All arguments are forwarded to start.sh.
#
# Usage:
#   ./scripts/buildstart.sh                 # build + launch everything
#   ./scripts/buildstart.sh --no-camera     # build + launch without camera
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Reject unsupported arguments before building or inspecting hardware.
for arg in "$@"; do
  case "$arg" in
    --no-camera|--no-mks|--no-lidar|--no-battery|--drive-only|--rebuild)
      ;;
    -h|--help)
      exec "$SCRIPT_DIR/start.sh" --help
      ;;
    --dry-run)
      echo "ERROR: --dry-run is not implemented; no build or launch was performed." >&2
      exit 2
      ;;
    *)
      echo "ERROR: unknown argument: $arg" >&2
      echo "Run ./scripts/buildstart.sh --help for supported options." >&2
      exit 2
      ;;
  esac
done

# Do not modify the active install tree or create a duplicate stack.
EXISTING_PID=$(pgrep -f \
  "ros2 launch horseshitbot|ros2 run horseshitbot|/install/horseshitbot/lib/horseshitbot/" \
  || true)
if [ -n "$EXISTING_PID" ]; then
  echo "ERROR: HorseShitBot is already running (PID(s): $EXISTING_PID)." >&2
  echo "Stop the existing stack safely before building and starting it again." >&2
  exit 3
fi

source /opt/ros/humble/setup.bash

echo "=== Building workspace ==="
cd "$REPO_DIR"
colcon build
echo ""

source "$REPO_DIR/install/setup.bash"

exec "$SCRIPT_DIR/start.sh" "$@"

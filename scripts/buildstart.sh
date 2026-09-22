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

exec "$SCRIPT_DIR/start.sh" --rebuild "$@"

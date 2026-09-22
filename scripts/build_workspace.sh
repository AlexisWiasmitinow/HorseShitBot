#!/usr/bin/env bash
#
# Build the ROS 2 workspace only when build-relevant source content changed.
#
# Usage:
#   ./scripts/build_workspace.sh           # build if needed
#   ./scripts/build_workspace.sh --force   # always build
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
STAMP_FILE="$REPO_DIR/build/.horseshitbot-source.sha256"
FORCE_REBUILD=false

case "${1:-}" in
  "") ;;
  --force) FORCE_REBUILD=true ;;
  *)
    echo "Usage: $0 [--force]" >&2
    exit 2
    ;;
esac

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash

source_fingerprint() {
  find "$REPO_DIR/src" -type f \
    ! -path '*/__pycache__/*' \
    ! -name '*.pyc' \
    -print0 \
    | LC_ALL=C sort -z \
    | xargs -0 -r sha256sum \
    | sha256sum \
    | awk '{print $1}'
}

CURRENT_FINGERPRINT="$(source_fingerprint)"
SAVED_FINGERPRINT=""
if [ -f "$STAMP_FILE" ]; then
  SAVED_FINGERPRINT="$(cat "$STAMP_FILE")"
fi

if [ "$FORCE_REBUILD" = false ] \
    && [ -f "$REPO_DIR/install/setup.bash" ] \
    && [ "$CURRENT_FINGERPRINT" = "$SAVED_FINGERPRINT" ]; then
  echo "Workspace build is current."
  exit 0
fi

echo "=== Building workspace (symlink install) ==="
mkdir -p "$(dirname "$STAMP_FILE")"
rm -f "$STAMP_FILE"

cd "$REPO_DIR"
colcon build --symlink-install

BUILT_FINGERPRINT="$(source_fingerprint)"
if [ "$BUILT_FINGERPRINT" != "$CURRENT_FINGERPRINT" ]; then
  echo "ERROR: Source changed during the build; rebuild before starting." >&2
  exit 1
fi

TEMP_STAMP="$(mktemp "${STAMP_FILE}.tmp.XXXXXX")"
trap 'rm -f "$TEMP_STAMP"' EXIT
printf '%s\n' "$BUILT_FINGERPRINT" > "$TEMP_STAMP"
mv -f "$TEMP_STAMP" "$STAMP_FILE"
trap - EXIT
echo ""

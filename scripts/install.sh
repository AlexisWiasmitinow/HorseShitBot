#!/usr/bin/env bash
#
# Install all dependencies for HorseShitBot on a fresh Jetson / Pi.
# Run once after cloning, or again if dependencies change.
#
# Usage:
#   sudo ./scripts/install.sh          # full install
#   sudo ./scripts/install.sh --ros    # ROS 2 deps only (skip pip)
#
set -eo pipefail

if [ "$(id -u)" -ne 0 ] && [ "$1" != "--help" ]; then
  echo "Run with sudo:  sudo $0 $*"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

ROS_ONLY=false
for arg in "$@"; do
  case "$arg" in
    --ros) ROS_ONLY=true ;;
  esac
done

echo "=== HorseShitBot — Install ==="

# ── System packages ──────────────────────────────────────────────
echo ""
echo "--- APT packages ---"
apt-get update -qq

# ROS 2 build tools
apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  dos2unix

# ROS 2 Humble packages (ignore failures for packages not in all repos)
apt-get install -y --no-install-recommends \
  ros-humble-ros-base \
  || true

# RealSense (optional — only available on some architectures)
apt-get install -y --no-install-recommends \
  ros-humble-realsense2-camera \
  || echo "  (realsense2_camera not available — camera features disabled)"

# spidev for display tests
apt-get install -y --no-install-recommends \
  python3-spidev \
  || true

# hostapd + dnsmasq for WiFi AP mode and DHCP server
# NM manages these internally — we don't want them as standalone services
apt-get install -y --no-install-recommends \
  hostapd \
  dnsmasq-base \
  || true
systemctl disable --now dnsmasq 2>/dev/null || true
systemctl disable --now hostapd 2>/dev/null || true
systemctl unmask hostapd 2>/dev/null || true

# ── Python packages (system-wide, needed by ROS 2 nodes) ────────
if [ "$ROS_ONLY" = false ]; then
  echo ""
  echo "--- pip packages ---"
  pip3 install --break-system-packages \
    luma.lcd \
    Pillow \
    pymodbus \
    pyserial \
    evdev \
    2>/dev/null \
  || pip3 install \
    luma.lcd \
    Pillow \
    pymodbus \
    pyserial \
    evdev
fi

# ── User groups ──────────────────────────────────────────────────
REAL_USER="${SUDO_USER:-$USER}"
echo ""
echo "--- Adding $REAL_USER to required groups ---"
for grp in input dialout spi gpio i2c netdev; do
  if getent group "$grp" > /dev/null 2>&1; then
    usermod -aG "$grp" "$REAL_USER" && echo "  + $grp" || true
  fi
done

# ── NetworkManager permissions (WiFi, AP, IP config) ────────────
echo ""
echo "--- NetworkManager polkit policy ---"
POLKIT_FILE="/etc/polkit-1/localauthority/50-local.d/10-hsb-network.pkla"
mkdir -p "$(dirname "$POLKIT_FILE")"
cat > "$POLKIT_FILE" << EOF
[Allow $REAL_USER to manage NetworkManager]
Identity=unix-user:$REAL_USER
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOF
echo "  Created $POLKIT_FILE"

# ── Bluetooth restart permission (for br-connection-create-socket recovery) ──
echo ""
echo "--- Bluetooth restart sudoers rule ---"
SUDOERS_BT="/etc/sudoers.d/hsb-bluetooth"
cat > "$SUDOERS_BT" << EOF
$REAL_USER ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart bluetooth
EOF
chmod 0440 "$SUDOERS_BT"
echo "  Created $SUDOERS_BT"

# ── Fix line endings (repo may come from Windows) ────────────────
echo ""
echo "--- Fixing line endings ---"
find "$REPO_DIR" -name '*.py' -o -name '*.sh' -o -name '*.yaml' -o -name '*.xml' \
  | xargs dos2unix -q 2>/dev/null || true

# ── Build ROS 2 workspace ────────────────────────────────────────
echo ""
echo "--- Building ROS 2 workspace ---"
sudo -u "$REAL_USER" bash -c "
  source /opt/ros/humble/setup.bash
  cd '$REPO_DIR'
  colcon build
"

echo ""
echo "=== Install complete ==="
echo "Log out and back in for group changes to take effect."
echo "Then run:  ./scripts/start.sh --no-mks"

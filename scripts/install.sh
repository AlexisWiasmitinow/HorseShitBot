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

# Base packages — must succeed
apt-get install -y --no-install-recommends \
  python3-pip \
  python3-spidev \
  dos2unix \
  hostapd \
  dnsmasq

# ── ROS 2 Humble apt repo ────────────────────────────────────────
if ! apt-cache show ros-humble-ros-base &>/dev/null; then
  echo "--- Adding ROS 2 Humble apt repository ---"
  apt-get install -y --no-install-recommends curl gnupg lsb-release
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update -qq
fi

# ROS packages
apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-ros-base \
  ros-humble-realsense2-camera

# hostapd is managed by NM — keep the service disabled
systemctl disable --now hostapd 2>/dev/null || true
systemctl unmask hostapd 2>/dev/null || true

# systemd-resolved occupies port 53 and conflicts with dnsmasq.
# Disable it and point resolv.conf at 127.0.0.1 (dnsmasq).
systemctl disable --now systemd-resolved 2>/dev/null || true
rm -f /etc/resolv.conf
echo "nameserver 127.0.0.1" > /etc/resolv.conf

# dnsmasq runs as a systemd service for DHCP + DNS forwarding.
# Per-interface hsb-*.conf files control DHCP ranges.
systemctl enable dnsmasq 2>/dev/null || true

# ── Python packages (system-wide, needed by ROS 2 nodes) ────────
if [ "$ROS_ONLY" = false ]; then
  echo ""
  echo "--- pip packages ---"

  # Ensure pip is available (may not be installed yet)
  if ! python3 -m pip --version &>/dev/null; then
    apt-get install -y --no-install-recommends python3-pip
  fi

  # Remove stale apt pymodbus (too old for our codebase)
  apt-get remove -y python3-pymodbus 2>/dev/null || true

  python3 -m pip install --break-system-packages \
    luma.lcd \
    Pillow \
    "pymodbus>=3.10" \
    pyserial \
    evdev \
    2>/dev/null \
  || python3 -m pip install \
    luma.lcd \
    Pillow \
    "pymodbus>=3.10" \
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

# ── dnsmasq DHCP server permissions ──────────────────────────────
echo ""
echo "--- dnsmasq permissions ---"
mkdir -p /etc/dnsmasq.d
chown "$REAL_USER":"$REAL_USER" /etc/dnsmasq.d
echo "  /etc/dnsmasq.d owned by $REAL_USER"

# Base dnsmasq config: forward DNS to public resolvers so the
# resolvconf hook (nameserver 127.0.0.1) actually works, and AP
# clients also get working DNS.
# Per-interface hsb-*.conf files add DHCP ranges.
HSB_DNSMASQ_BASE="/etc/dnsmasq.d/00-hsb-base.conf"
cat > "$HSB_DNSMASQ_BASE" << 'EOF'
no-resolv
server=8.8.8.8
server=1.1.1.1
EOF
chown "$REAL_USER":"$REAL_USER" "$HSB_DNSMASQ_BASE"
echo "  Created $HSB_DNSMASQ_BASE (DNS forwarding to 8.8.8.8 / 1.1.1.1)"

systemctl restart dnsmasq 2>/dev/null || true

SUDOERS_DNSMASQ="/etc/sudoers.d/hsb-dnsmasq"
cat > "$SUDOERS_DNSMASQ" << EOF
$REAL_USER ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart dnsmasq
EOF
chmod 0440 "$SUDOERS_DNSMASQ"
echo "  Created $SUDOERS_DNSMASQ"

# ── udev rules (stable /dev/odrive, /dev/mksbus symlinks) ────────
echo ""
echo "--- udev rules ---"
cp "$SCRIPT_DIR/99-horseshitbot.rules" /etc/udev/rules.d/99-horseshitbot.rules
udevadm control --reload-rules
udevadm trigger
echo "  Installed /etc/udev/rules.d/99-horseshitbot.rules"
echo "  /dev/odrive → ODrive,  /dev/mksbus → RS485 adapter"

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

#!/usr/bin/env bash
#
# Build and load the CH341 USB-serial kernel module on Jetson (L4T / Ubuntu).
# Required for the USB-to-RS485 adapter (/dev/mksbus).
#
# The ch341 module is in the mainline kernel but is not always enabled in
# NVIDIA's Jetson defconfig. This script builds it as an out-of-tree module
# from the kernel source file matching the running kernel version.
#
# Usage:
#   ./scripts/install_ch340_driver.sh
#
set -eo pipefail

KVER="$(uname -r)"
KHEADERS="/lib/modules/$KVER/build"
BUILD_DIR="/tmp/ch341-build"
MODULE_DEST="/lib/modules/$KVER/kernel/drivers/usb/serial"

echo "=== CH341 driver install (kernel $KVER) ==="

# ── 0. Check if the module already loads ─────────────────────────
echo ""
echo "--- Checking existing module ---"
if sudo modprobe ch341 2>/dev/null; then
  echo "  ch341 module already available — nothing to build."
  echo "  Ensuring it loads on boot..."
  grep -qx ch341 /etc/modules 2>/dev/null || echo "ch341" | sudo tee -a /etc/modules > /dev/null
  exit 0
fi
echo "  ch341 not available — will build from source."

# ── 1. Kernel headers ─────────────────────────────────────────────
echo ""
echo "--- Installing kernel headers ---"
sudo apt-get install -y --no-install-recommends \
  build-essential \
  bc \
  libssl-dev \
  flex \
  bison

# Jetson (L4T) ships headers via nvidia-l4t-kernel-headers, not linux-headers-*
if [ ! -d "$KHEADERS" ]; then
  echo "  Standard headers not found — installing nvidia-l4t-kernel-headers ..."
  # Bypass TLS verification (same proxy issue as ROS repo)
  echo 'Acquire::https::repo.download.nvidia.com::Verify-Peer "false";' \
    > /etc/apt/apt.conf.d/99-nvidia-no-verify
  apt-get update -qq
  sudo apt-get install -y --no-install-recommends nvidia-l4t-kernel-headers

  # L4T installs headers under /usr/src/linux-headers-<kver>; symlink into /lib/modules
  L4T_HDR="$(find /usr/src -maxdepth 1 -name "linux-headers-${KVER}" -type d 2>/dev/null | head -1)"
  if [ -n "$L4T_HDR" ] && [ ! -e "$KHEADERS" ]; then
    sudo ln -sf "$L4T_HDR" "$KHEADERS"
    echo "  Linked $L4T_HDR → $KHEADERS"
  fi
fi

if [ ! -d "$KHEADERS" ]; then
  echo "ERROR: Kernel headers not found at $KHEADERS"
  echo "  Tried: linux-headers-$KVER and nvidia-l4t-kernel-headers"
  echo "  Check: apt-cache search linux-headers"
  exit 1
fi
echo "  Headers found at $KHEADERS"

# ── 2. Fetch ch341.c from upstream kernel matching running version ─
echo ""
echo "--- Fetching ch341.c source ---"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# Strip any distro suffix (e.g. -tegra, -generic) to get the upstream version
UPSTREAM_VER="$(echo "$KVER" | grep -oP '^\d+\.\d+\.\d+')"
SRC_URL="https://raw.githubusercontent.com/torvalds/linux/v${UPSTREAM_VER}/drivers/usb/serial/ch341.c"

echo "  Trying $SRC_URL ..."
if ! curl -kfsSL "$SRC_URL" -o "$BUILD_DIR/ch341.c"; then
  # Fall back to closest stable branch
  MINOR="$(echo "$UPSTREAM_VER" | cut -d. -f1-2)"
  SRC_URL="https://raw.githubusercontent.com/torvalds/linux/v${MINOR}/drivers/usb/serial/ch341.c"
  echo "  Exact tag not found, trying $SRC_URL ..."
  curl -kfsSL "$SRC_URL" -o "$BUILD_DIR/ch341.c"
fi
echo "  Downloaded ch341.c"

# ── 3. Write Makefile ─────────────────────────────────────────────
cat > "$BUILD_DIR/Makefile" << EOF
obj-m += ch341.o

all:
	make -C $KHEADERS M=\$(CURDIR) modules

clean:
	make -C $KHEADERS M=\$(CURDIR) clean
EOF

# ── 4. Build ──────────────────────────────────────────────────────
echo ""
echo "--- Building ch341.ko ---"
make -C "$BUILD_DIR"

# ── 5. Install and load ───────────────────────────────────────────
echo ""
echo "--- Installing and loading module ---"
sudo mkdir -p "$MODULE_DEST"
sudo cp "$BUILD_DIR/ch341.ko" "$MODULE_DEST/"
sudo depmod -a
sudo modprobe ch341

# ── 6. Persist across reboots ─────────────────────────────────────
grep -qx ch341 /etc/modules 2>/dev/null || echo "ch341" | sudo tee -a /etc/modules > /dev/null

echo ""
echo "=== CH341 driver installed and loaded ==="
echo "Verify with:  lsmod | grep ch341"
echo "Adapter will appear as /dev/mksbus once plugged in (udev rule applies)."

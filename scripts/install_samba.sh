#!/usr/bin/env bash
#
# Install and configure a minimal Samba file server on the Jetson.
# Shares the HorseShitBot repo as a guest-accessible network drive.
#
# Usage:
#   ./scripts/install_samba.sh
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REAL_USER="${SUDO_USER:-$USER}"

echo "=== HorseShitBot — Install Samba ==="

echo ""
echo "--- Installing samba ---"
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends samba

echo ""
echo "--- Deploying smb.conf ---"
sudo cp "$SCRIPT_DIR/smb.conf" /etc/samba/smb.conf
echo "  Installed /etc/samba/smb.conf"

echo ""
echo "--- Enabling and starting smbd / nmbd ---"
sudo systemctl enable smbd nmbd
sudo systemctl restart smbd nmbd

echo ""
echo "=== Samba install complete ==="
echo "Share is available at:  \\\\$(hostname)\\horseshitbot"

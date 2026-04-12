#!/bin/bash
# Grant the hsb user permission to manage NetworkManager connections
# (WiFi connect/disconnect, IP config changes) without requiring root.
#
# Run once:  sudo bash scripts/setup_network_permissions.sh

set -e

POLKIT_DIR="/etc/polkit-1/localauthority/50-local.d"
POLKIT_FILE="$POLKIT_DIR/10-hsb-network.pkla"

mkdir -p "$POLKIT_DIR"

cat > "$POLKIT_FILE" << 'EOF'
[Allow hsb to manage NetworkManager]
Identity=unix-user:hsb
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOF

echo "Created $POLKIT_FILE"

# Also add hsb to the netdev group (fallback for some distros)
usermod -aG netdev hsb 2>/dev/null || true

echo "Done. NetworkManager permissions granted to user 'hsb'."
echo "No reboot needed — takes effect immediately."

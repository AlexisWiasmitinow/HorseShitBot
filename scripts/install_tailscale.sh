#!/usr/bin/env bash
#
# Install Tailscale and bring up the node.
# After running, authenticate via the printed URL or the Tailscale admin panel.
#
# This node does not act as a router — no subnet routes, no exit node.
#
# Usage:
#   ./scripts/install_tailscale.sh [--ssh]
#
#   --ssh   Enable Tailscale SSH (access without managing SSH keys)
#
set -eo pipefail

TAILSCALE_UP_ARGS="--accept-routes=false"

for arg in "$@"; do
  case "$arg" in
    --ssh) TAILSCALE_UP_ARGS="$TAILSCALE_UP_ARGS --ssh" ;;
  esac
done

echo "=== HorseShitBot — Install Tailscale ==="

# ── Install ───────────────────────────────────────────────────────
echo ""
echo "--- Installing Tailscale ---"

if command -v tailscale &>/dev/null; then
  echo "  Already installed: $(tailscale version | head -1)"
else
  # Official install script handles repo + GPG key for all distros/arches
  curl -fsSL https://tailscale.com/install.sh | sh
fi

# ── Enable and start daemon ───────────────────────────────────────
echo ""
echo "--- Enabling tailscaled ---"
sudo systemctl enable --now tailscaled

# ── Bring up the node ─────────────────────────────────────────────
echo ""
echo "--- Bringing up Tailscale node ---"
echo "  Args: tailscale up $TAILSCALE_UP_ARGS"
echo ""

# shellcheck disable=SC2086
sudo tailscale up $TAILSCALE_UP_ARGS

# ── Status ────────────────────────────────────────────────────────
echo ""
echo "--- Tailscale status ---"
tailscale status

echo ""
echo "=== Tailscale install complete ==="
echo "Node name : $(tailscale status --json 2>/dev/null | python3 -c 'import sys,json; d=json.load(sys.stdin); print(d[\"Self\"][\"DNSName\"])' 2>/dev/null || hostname)"
echo "IP        : $(tailscale ip -4 2>/dev/null || echo '(not yet assigned)')"

#!/usr/bin/env bash
#
# Configure Jetson header pins for the ILI9341/ST7789 SPI screen:
#   - Enable SPI1          (pins 19/21/23/24/26  → /dev/spidev0.x or 1.x)
#   - Free pin 22 as GPIO  (DC,  spi3 group muxed to GPIO)
#   - Free pin 18 as GPIO  (RST, spi3 group muxed to GPIO)
#   - Free pin 12 as GPIO  (LED, i2s2 group muxed to GPIO)
#
# Uses jetson-io.py non-interactively via its config-by-pin interface.
# A reboot is required for the device-tree overlay to take effect.
#
# Usage:
#   ./scripts/configure_jetson_screen_pins.sh
#
set -eo pipefail

JETSON_IO="/opt/nvidia/jetson-io/jetson-io.py"

echo "=== Jetson screen pin configuration ==="

# ── Preflight ─────────────────────────────────────────────────────
if [ ! -f "$JETSON_IO" ]; then
  echo "ERROR: $JETSON_IO not found — is this a Jetson?"
  exit 1
fi

# ── Check if already configured ───────────────────────────────────
echo ""
echo "--- Current SPI devices ---"
SPI_DEVS=$(ls /dev/spidev* 2>/dev/null || true)
if [ -n "$SPI_DEVS" ]; then
  echo "  Found: $SPI_DEVS"
  echo ""
  read -rp "SPI already present. Re-apply overlay anyway? [y/N] " REPLY
  [[ "$REPLY" =~ ^[Yy]$ ]] || { echo "  Skipped."; exit 0; }
else
  echo "  No /dev/spidev* found — SPI not yet enabled."
fi

# ── Apply overlays via jetson-io ──────────────────────────────────
echo ""
echo "--- Applying jetson-io overlays ---"

# jetson-io.py supports a non-interactive config file interface:
#   sudo python3 jetson-io.py -o <overlay_name>
# The overlay names vary by board; try the known Orin/Nano names in order.

apply_overlay() {
  local name="$1"
  echo "  Trying overlay: '$name' ..."
  if sudo python3 "$JETSON_IO" -o "$name" 2>/dev/null; then
    echo "  Applied: $name"
    return 0
  fi
  echo "  Not available: $name"
  return 1
}

# Enable SPI1 (try both common overlay names across JetPack versions)
apply_overlay "Enable SPI1 CS0" \
  || apply_overlay "Enable SPI1" \
  || apply_overlay "SPI1" \
  || {
    echo ""
    echo "  Could not apply SPI1 overlay automatically."
    echo "  Run manually:  sudo python3 $JETSON_IO"
    echo "  → Configure → Select 'SPI1' → Save → Reboot"
    MANUAL=1
  }

# Free the spi3/i2s2 group pins as GPIO (pins 12, 18, 22)
apply_overlay "Configure spi3 as GPIO" \
  || apply_overlay "Configure i2s2 as GPIO" \
  || true   # these may not be separate overlays on all boards

# ── Save and summarise ────────────────────────────────────────────
echo ""
if [ "${MANUAL:-0}" = "1" ]; then
  echo "=== Manual step required ==="
  echo ""
  echo "  sudo python3 $JETSON_IO"
  echo ""
  echo "  In the TUI:"
  echo "    1. Configure Jetson 40pin Header"
  echo "    2. Enable SPI1 (pins 19/21/23/24/26)"
  echo "    3. Set 'spi3' group (13,16,18,22,37) → GPIO"
  echo "    4. Set 'i2s2' group (12,35,38,40)    → GPIO"
  echo "    5. Save and reboot"
else
  echo "=== Overlay applied — reboot required ==="
  echo ""
  read -rp "Reboot now? [y/N] " REPLY
  if [[ "$REPLY" =~ ^[Yy]$ ]]; then
    sudo reboot
  else
    echo "  Run 'sudo reboot' when ready."
  fi
fi

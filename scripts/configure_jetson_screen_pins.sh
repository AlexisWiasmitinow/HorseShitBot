#!/usr/bin/env bash
#
# Configure Jetson 40-pin header for the SPI screen (ILI9341 / ST7789).
# This is a manual step — jetson-io.py requires interactive TUI input.
#
# Usage:
#   ./scripts/configure_jetson_screen_pins.sh
#
echo ""
echo "=== Jetson screen pin configuration ==="
echo ""
echo "Current SPI devices:"
ls /dev/spidev* 2>/dev/null && echo "" || echo "  (none — SPI not yet enabled)" && echo ""

echo "Run the following and configure as shown:"
echo ""
echo "  sudo /opt/nvidia/jetson-io/jetson-io.py"
echo ""
echo "  Navigate to: Configure Jetson 40pin Header"
echo "               → Select desired functions for pins"
echo ""
echo "  Set these rows (arrow keys to move, Space to select):"
echo ""
echo "    [ ] unused [*] gpio [ ] i2s2   (12,35,38,40)   ← pin 12  = LED backlight"
echo "    [ ] unused [ ] gpio [*] spi1   (19,21,23,24,26) ← SPI bus"
echo "    [ ] unused [*] gpio [ ] spi3   (13,16,18,22,37) ← pin 18 = RST, pin 22 = DC"
echo ""
echo "  Then: Save pin changes → Save and reboot"
echo ""

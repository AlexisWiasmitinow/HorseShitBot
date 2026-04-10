#!/usr/bin/env python3
"""
Toggle a 40-pin header GPIO HIGH/LOW slowly so you can verify with a multimeter
or logic probe (D/C, RST, etc.). Jetson: physical pin number as used by Blinka board.D##.

  python3 test_scripts/jetson_gpio_wiggle.py 22
  python3 test_scripts/jetson_gpio_wiggle.py 18
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> None:
    p = argparse.ArgumentParser(description="Slow-toggle one header GPIO (Blinka board.D##)")
    p.add_argument("pin", type=int, help="Physical 40-pin number, e.g. 22 for D/C")
    p.add_argument("-n", type=int, default=20, help="Half-cycles (default 20 = ~10s at 0.5s each)")
    p.add_argument("-t", type=float, default=0.5, help="Seconds per half-cycle")
    args = p.parse_args()

    try:
        import board
        import digitalio
    except ImportError:
        print("Install: pip install adafruit-blinka", file=sys.stderr)
        sys.exit(1)

    name = f"D{args.pin}"
    if not hasattr(board, name):
        print(f"ERROR: board has no {name}", file=sys.stderr)
        sys.exit(1)

    pin = digitalio.DigitalInOut(getattr(board, name))
    pin.direction = digitalio.Direction.OUTPUT

    print(f"Toggling board.{name} (physical pin {args.pin}) ~{args.t}s HIGH/LOW, {args.n} half-cycles.")
    print("Probe the header pin; if voltage does not move, wrong pin or pinmux.")
    try:
        for i in range(args.n):
            pin.value = i & 1
            print(f"  {'HIGH' if pin.value else 'LOW'}")
            time.sleep(args.t)
    finally:
        pin.deinit()


if __name__ == "__main__":
    main()

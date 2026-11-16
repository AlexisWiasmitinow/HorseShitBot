#!/usr/bin/env python3
"""Render a ROS map_server occupancy grid with metric axes."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def read_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as handle:
        if handle.readline().strip() != b"P5":
            raise RuntimeError(f"Expected binary P5 PGM: {path}")
        line = handle.readline()
        while line.startswith(b"#"):
            line = handle.readline()
        width, height = map(int, line.split())
        maximum = int(handle.readline())
        image = np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width)
        if maximum != 255:
            image = np.rint(image.astype(float) * 255.0 / maximum).astype(np.uint8)
        return image


def render(map_yaml_path: Path, output_path: Path, title: str) -> None:
    metadata = yaml.safe_load(map_yaml_path.read_text())
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    image = read_pgm(image_path)
    resolution = float(metadata["resolution"])
    origin_x, origin_y = map(float, metadata["origin"][:2])
    extent = (
        origin_x,
        origin_x + image.shape[1] * resolution,
        origin_y,
        origin_y + image.shape[0] * resolution,
    )

    figure, axis = plt.subplots(figsize=(10, 8))
    axis.imshow(
        image,
        cmap="gray",
        origin="upper",
        extent=extent,
        vmin=0,
        vmax=255,
        interpolation="nearest",
    )
    axis.axhline(0.0, color="tab:green", linewidth=0.7, alpha=0.6)
    axis.axvline(0.0, color="tab:red", linewidth=0.7, alpha=0.6)
    axis.set_aspect("equal")
    axis.set_xlabel("map X (m)")
    axis.set_ylabel("map Y (m)")
    axis.set_title(title)
    axis.grid(alpha=0.2)
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--title", default="Occupancy-grid map")
    args = parser.parse_args()
    render(Path(args.map), Path(args.output), args.title)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

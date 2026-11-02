"""Backend and geometry tests for scripts/view_lidar_room_web.py."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest
from urllib.request import urlopen

import yaml
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "view_lidar_room_web.py"
SPEC = importlib.util.spec_from_file_location("view_lidar_room_web", SCRIPT)
VIEWER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = VIEWER
SPEC.loader.exec_module(VIEWER)
EXTRACT_SPEC = importlib.util.spec_from_file_location(
    "extract_oct1_lidar_web_data",
    REPO_ROOT / "scripts" / "extract_oct1_lidar_web_data.py",
)
EXTRACTOR = importlib.util.module_from_spec(EXTRACT_SPEC)
sys.modules[EXTRACT_SPEC.name] = EXTRACTOR
EXTRACT_SPEC.loader.exec_module(EXTRACTOR)


class TestLidarRoomWeb(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        directory = Path(self.temporary.name)
        self.pgm = directory / "test.pgm"
        # PGM rows are top-to-bottom: 4 columns x 3 rows.
        self.pixels = bytes(range(12))
        self.pgm.write_bytes(b"P5\n# synthetic\n4 3\n255\n" + self.pixels)
        self.yaml = directory / "test.yaml"
        self.yaml.write_text(
            yaml.safe_dump(
                {
                    "image": self.pgm.name,
                    "mode": "trinary",
                    "resolution": 0.05,
                    "origin": [-1.0, -2.0, 0.0],
                    "negate": 0,
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.25,
                }
            )
        )
        self.model = VIEWER.MapModel.load(self.yaml)

    def tearDown(self):
        self.temporary.cleanup()

    def test_loads_original_map_geometry_and_pixels(self):
        self.assertEqual((self.model.width, self.model.height), (4, 3))
        self.assertEqual(self.model.resolution, 0.05)
        self.assertEqual(self.model.pixels, self.pixels)
        self.assertTrue(self.model.png.startswith(b"\x89PNG\r\n\x1a\n"))
        width, height = struct.unpack(">II", self.model.png[16:24])
        self.assertEqual((width, height), (4, 3))

    def test_image_to_ros_coordinates_inverts_vertical_axis(self):
        # Top-left image boundary is the upper-left ROS map boundary.
        top = self.model.image_to_map(0, 0)
        bottom = self.model.image_to_map(0, 3)
        self.assertAlmostEqual(top[0], -1.0)
        self.assertAlmostEqual(top[1], -1.85)
        self.assertAlmostEqual(bottom[0], -1.0)
        self.assertAlmostEqual(bottom[1], -2.0)
        # Twenty pixels always equal one meter at 0.05 m/pixel.
        x0, y0 = self.model.image_to_map(0, 3)
        x1, y1 = self.model.image_to_map(20, 3)
        self.assertAlmostEqual(math.hypot(x1 - x0, y1 - y0), 1.0)

    def test_coordinate_conversion_round_trip_with_rotated_origin(self):
        rotated_yaml = Path(self.temporary.name) / "rotated.yaml"
        rotated_yaml.write_text(
            yaml.safe_dump(
                {
                    "image": self.pgm.name,
                    "resolution": 0.05,
                    "origin": [2.0, 3.0, math.pi / 2],
                }
            )
        )
        model = VIEWER.MapModel.load(rotated_yaml)
        world = model.image_to_map(2.25, 1.75)
        image = model.map_to_image(*world)
        self.assertAlmostEqual(image[0], 2.25)
        self.assertAlmostEqual(image[1], 1.75)

    def test_measurement_error_sign_and_percentage(self):
        result = VIEWER.measurement_error(3.31, 3.40)
        self.assertAlmostEqual(result["difference"], -0.09)
        self.assertAlmostEqual(result["percentage_error"], -2.6470588)
        self.assertEqual(
            VIEWER.measurement_error(3.31, None),
            {"difference": None, "percentage_error": None},
        )

    def test_wall_detection_finds_metric_straight_segment(self):
        x = np.linspace(-2.0, 2.0, 81)
        points = np.column_stack((x, np.full_like(x, 1.25)))
        walls = EXTRACTOR.detect_wall_segments(points)
        self.assertTrue(walls)
        self.assertAlmostEqual(walls[0]["sensor_distance_m"], 1.25, places=3)
        self.assertGreater(walls[0]["length_m"], 3.7)
        self.assertLess(walls[0]["rmse_m"], 0.001)

    def test_representative_scan_indices_include_recording_ends(self):
        selected = EXTRACTOR.representative_indices(802, 21)
        self.assertEqual(len(selected), 21)
        self.assertEqual(selected[0], 0)
        self.assertEqual(selected[-1], 801)
        self.assertEqual(EXTRACTOR.representative_indices(4, 0), [0, 1, 2, 3])

    def test_loads_physical_room_measurements_and_preserves_discrepancy(self):
        room_path = (
            REPO_ROOT
            / "src"
            / "horseshitbot"
            / "config"
            / "room_measurements_oct1.yaml"
        )
        room = VIEWER.load_room_measurements(room_path)
        self.assertEqual(room["room_metadata"]["units"], "meters")
        disc = room["discrepancies"][0]
        self.assertAlmostEqual(disc["left_wall_length_m"], 10.600)
        self.assertAlmostEqual(disc["right_wall_sum_m"], 10.322)
        self.assertAlmostEqual(disc["difference_m"], 0.278)
        widths = {item["id"]: item["measured_m"] for item in room["widths"]}
        self.assertEqual(widths, {"W1": 3.117, "W2": 3.272, "W3": 3.137})
        unassigned = {
            item["id"]: item["used_in_geometry"]
            for item in room["unassigned_or_uncertain_measurements"]
        }
        self.assertEqual(
            unassigned,
            {"circled_0_7": False, "sketch_327_3": False},
        )

    def test_rigid_transform_preserves_distance_without_scaling(self):
        p0 = VIEWER.rigid_transform_point(0.0, 0.0, -2.62, -4.15, math.radians(-11.0))
        p1 = VIEWER.rigid_transform_point(3.272, 4.400, -2.62, -4.15, math.radians(-11.0))
        self.assertAlmostEqual(
            math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
            math.hypot(3.272, 4.400),
            places=6,
        )

    def test_comparison_statistics_handles_incomplete_measurements(self):
        rows = [
            {"real_distance_m": 3.117, "lidar_scan_distance_m": 3.1156, "slam_map_distance_m": None},
            {"real_distance_m": 3.272, "lidar_scan_distance_m": 3.2710, "slam_map_distance_m": None},
            {"real_distance_m": 3.137, "lidar_scan_distance_m": 3.1376, "slam_map_distance_m": None},
            {"real_distance_m": 10.600, "lidar_scan_distance_m": None, "slam_map_distance_m": None},
        ]
        scan_stats = VIEWER.comparison_statistics(rows, "lidar_scan_distance_m")
        map_stats = VIEWER.comparison_statistics(rows, "slam_map_distance_m")
        self.assertEqual(scan_stats["count"], 3)
        self.assertAlmostEqual(scan_stats["mae_m"], 0.001, places=4)
        self.assertAlmostEqual(scan_stats["rmse_m"], 0.001058, places=4)
        self.assertEqual(map_stats, {"count": 0, "mae_m": None, "rmse_m": None})

    def test_http_server_serves_viewer_metadata_and_map(self):
        scan_payload = json.dumps(
            {"scans": [{"source_index": 0}], "trajectory": {"poses": []}}
        ).encode()
        server = VIEWER.create_server(
            self.model, "127.0.0.1", 0, scan_payload
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(base + "/metadata.json", timeout=3) as response:
                metadata = json.load(response)
            self.assertEqual(metadata["width"], 4)
            self.assertEqual(metadata["height"], 3)
            self.assertEqual(metadata["resolution"], 0.05)
            with urlopen(base + "/map.png", timeout=3) as response:
                self.assertEqual(response.headers["Content-Type"], "image/png")
                self.assertTrue(response.read().startswith(b"\x89PNG"))
            with urlopen(base + "/scans.json", timeout=3) as response:
                scans = json.load(response)
            self.assertEqual(scans["scans"][0]["source_index"], 0)
            with urlopen(base + "/room_measurements.json", timeout=3) as response:
                room = json.load(response)
            self.assertEqual(room["discrepancies"][0]["difference_m"], 0.278)
            with urlopen(base + "/", timeout=3) as response:
                html = response.read().decode()
            for required in (
                "Measure",
                "Export CSV",
                "Export PNG",
                "percentage_error",
                "imageToMap",
                "mapToImage",
                "LiDAR Scan",
                "Room Map",
                "Real Room Comparison",
                "Show Validated Room Geometry",
                "Measurements",
                "scans.json",
                "room_measurements.json",
                "Play",
                "Last 20 scans",
                "Recommended Scans for Measurement",
                "perpendicular wall separation",
            ):
                self.assertIn(required, html)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)


if __name__ == "__main__":
    unittest.main()

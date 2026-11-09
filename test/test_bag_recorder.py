"""
Unit and offline integration tests for BagRecorderNode and recorder launch files.

Verifies:
1. RELIABLE publisher messages are recorded exactly once.
2. BEST_EFFORT publisher messages are recorded exactly once.
3. The correct topics remain available for perception and mapping recorders
   (including TRANSIENT_LOCAL /tf_static).
4. Recorder launch configuration does not create unintended duplicate nodes.
5. Two recorders cannot silently write to the same MCAP file.
6. Generated MCAP files can be read back successfully.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.serialization import deserialize_message
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "horseshitbot"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.nodes import bag_recorder_node as bag_recorder_mod
from horseshitbot.nodes.bag_recorder_node import (
    BagRecorderNode,
    _TOPIC_GROUPS,
    _TOPIC_TYPE_MAP,
)


def _read_mcap_messages(bag_dir: str) -> dict[str, list[bytes]]:
    """Open an MCAP bag directory with SequentialReader and return serialized messages per topic."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    by_topic: dict[str, list[bytes]] = {}
    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        by_topic.setdefault(topic, []).append(data)
    return by_topic


def _spin_for(executor: SingleThreadedExecutor, duration_sec: float, step_sec: float = 0.02):
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=step_sec)


class TestBagRecorderFixes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._ros_log_dir = tempfile.TemporaryDirectory(prefix="hsb_ros_log_")
        os.environ["ROS_LOG_DIR"] = cls._ros_log_dir.name
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
        cls._ros_log_dir.cleanup()

    def setUp(self):
        self._tmp_dir = tempfile.TemporaryDirectory(prefix="hsb_bag_test_")
        self.tmp_path = Path(self._tmp_dir.name)
        self.config_dir = self.tmp_path / "config"
        self.bags_dir = self.tmp_path / "bags"
        self.config_dir.mkdir(parents=True, exist_ok=True)
        self.bags_dir.mkdir(parents=True, exist_ok=True)
        self._config_patcher = patch.object(
            bag_recorder_mod, "_CONFIG_DIR", self.config_dir
        )
        self._config_patcher.start()

    def tearDown(self):
        self._config_patcher.stop()
        self._tmp_dir.cleanup()

    def _make_recorder(
        self,
        *,
        node_name: str = "test_bag_recorder",
        bag_prefix: str = "test",
        topics: list[str] | None = None,
        topics_config_name: str = "test_topics.json",
    ) -> BagRecorderNode:
        params = [
            Parameter("output_dir", Parameter.Type.STRING, str(self.bags_dir)),
            Parameter("bag_prefix", Parameter.Type.STRING, bag_prefix),
            Parameter("storage_id", Parameter.Type.STRING, "mcap"),
            Parameter("topics_config_name", Parameter.Type.STRING, topics_config_name),
        ]
        if topics is not None:
            params.append(
                Parameter("default_topics", Parameter.Type.STRING_ARRAY, topics)
            )
        return BagRecorderNode(node_name=node_name, parameter_overrides=params)

    def test_1_reliable_publisher_recorded_exactly_once(self):
        """RELIABLE publishers (/odom, /scan) must be recorded once per message, not twice."""
        recorder = self._make_recorder(
            node_name="reliable_test_recorder",
            bag_prefix="reliable_test",
            topics=["/odom", "/scan"],
        )
        pub_node = rclpy.create_node("reliable_test_publisher")
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        odom_pub = pub_node.create_publisher(Odometry, "/odom", qos_reliable)
        scan_pub = pub_node.create_publisher(LaserScan, "/scan", qos_reliable)

        executor = SingleThreadedExecutor()
        executor.add_node(recorder)
        executor.add_node(pub_node)

        try:
            # Verify strictly one subscription per configured topic
            self.assertEqual(len(recorder._topic_subscriptions), 2)
            self.assertEqual(set(recorder._subscriptions_by_topic.keys()), {"/odom", "/scan"})

            _spin_for(executor, 0.25)

            start_res = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(start_res.success, start_res.message)
            bag_path = recorder._bag_path

            n_odom = 8
            n_scan = 5
            for i in range(n_odom):
                msg = Odometry()
                msg.header.frame_id = "odom"
                msg.child_frame_id = f"base_link_{i}"
                msg.pose.pose.position.x = float(i)
                odom_pub.publish(msg)
                _spin_for(executor, 0.04)

            for j in range(n_scan):
                scan = LaserScan()
                scan.header.frame_id = "laser"
                scan.ranges = [float(j + 1)] * 4
                scan_pub.publish(scan)
                _spin_for(executor, 0.04)

            _spin_for(executor, 0.2)

            stop_res = recorder._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(stop_res.success, stop_res.message)

            self.assertEqual(recorder._msg_counts["/odom"], n_odom)
            self.assertEqual(recorder._msg_counts["/scan"], n_scan)
            self.assertEqual(recorder._frame_count, n_odom + n_scan)

            recorded = _read_mcap_messages(bag_path)
            self.assertEqual(len(recorded.get("/odom", [])), n_odom)
            self.assertEqual(len(recorded.get("/scan", [])), n_scan)

            xs = [
                deserialize_message(raw, Odometry).pose.pose.position.x
                for raw in recorded["/odom"]
            ]
            self.assertEqual(xs, [float(i) for i in range(n_odom)])
        finally:
            if recorder._recording:
                recorder._close_bag()
            executor.remove_node(recorder)
            executor.remove_node(pub_node)
            recorder.destroy_node()
            pub_node.destroy_node()

    def test_2_best_effort_publisher_recorded_exactly_once(self):
        """BEST_EFFORT sensor-data publishers (/imu/data_raw) must be recorded once per message."""
        recorder = self._make_recorder(
            node_name="best_effort_test_recorder",
            bag_prefix="best_effort_test",
            topics=["/imu/data_raw"],
        )
        pub_node = rclpy.create_node("best_effort_test_publisher")
        imu_pub = pub_node.create_publisher(
            Imu, "/imu/data_raw", qos_profile_sensor_data
        )

        executor = SingleThreadedExecutor()
        executor.add_node(recorder)
        executor.add_node(pub_node)

        try:
            self.assertEqual(len(recorder._topic_subscriptions), 1)
            _spin_for(executor, 0.25)

            start_res = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(start_res.success, start_res.message)
            bag_path = recorder._bag_path

            n_imu = 10
            for i in range(n_imu):
                msg = Imu()
                msg.header.frame_id = "imu_link"
                msg.angular_velocity.z = float(i) * 0.1
                imu_pub.publish(msg)
                _spin_for(executor, 0.04)

            _spin_for(executor, 0.2)

            stop_res = recorder._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(stop_res.success, stop_res.message)

            self.assertEqual(recorder._msg_counts["/imu/data_raw"], n_imu)
            self.assertEqual(recorder._frame_count, n_imu)

            recorded = _read_mcap_messages(bag_path)
            self.assertEqual(len(recorded.get("/imu/data_raw", [])), n_imu)
            wz_values = [
                round(deserialize_message(raw, Imu).angular_velocity.z, 4)
                for raw in recorded["/imu/data_raw"]
            ]
            self.assertEqual(wz_values, [round(i * 0.1, 4) for i in range(n_imu)])
        finally:
            if recorder._recording:
                recorder._close_bag()
            executor.remove_node(recorder)
            executor.remove_node(pub_node)
            recorder.destroy_node()
            pub_node.destroy_node()

    def test_3_configured_topics_and_transient_local_tf_static_available(self):
        """Perception and mapping recorder topics in params.yaml must be valid and recordable."""
        params_path = PACKAGE_ROOT / "config" / "params.yaml"
        params_data = yaml.safe_load(params_path.read_text())

        perception_defaults = params_data["perception_recorder"]["ros__parameters"][
            "default_topics"
        ]
        mapping_defaults = params_data["mapping_recorder"]["ros__parameters"][
            "default_topics"
        ]

        expected_perception = {
            "/camera/color/image_raw",
            "/camera/aligned_depth_to_color/image_raw",
            "/camera/color/camera_info",
            "/camera/aligned_depth_to_color/camera_info",
        }
        expected_mapping = {
            "/scan",
            "/odom",
            "/imu/data_raw",
            "/imu/mag",
            "/imu/temperature",
            "/tf",
            "/tf_static",
        }
        self.assertEqual(set(perception_defaults), expected_perception)
        self.assertEqual(set(mapping_defaults), expected_mapping)

        for topic in perception_defaults + mapping_defaults:
            self.assertIn(topic, _TOPIC_TYPE_MAP)

        for group_topics in _TOPIC_GROUPS.values():
            for topic in group_topics:
                self.assertIn(topic, _TOPIC_TYPE_MAP)

        # Verify mapping recorder creates 1 subscription per topic and captures latched /tf_static
        recorder = self._make_recorder(
            node_name="mapping_topic_test_recorder",
            bag_prefix="mapping",
            topics=mapping_defaults,
            topics_config_name="mapping_bag_topics.json",
        )
        pub_node = rclpy.create_node("tf_static_test_publisher")
        tf_static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        tf_static_pub = pub_node.create_publisher(
            TFMessage, "/tf_static", tf_static_qos
        )

        executor = SingleThreadedExecutor()
        executor.add_node(recorder)
        executor.add_node(pub_node)

        try:
            self.assertEqual(len(recorder._topic_subscriptions), len(mapping_defaults))
            self.assertEqual(set(recorder._topics), expected_mapping)

            # Publish a latched /tf_static transform BEFORE recording starts
            tf_msg = TFMessage()
            ts = TransformStamped()
            ts.header.frame_id = "base_link"
            ts.child_frame_id = "laser"
            tf_msg.transforms.append(ts)
            tf_static_pub.publish(tf_msg)
            _spin_for(executor, 0.25)

            start_res = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(start_res.success, start_res.message)
            bag_path = recorder._bag_path

            stop_res = recorder._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(stop_res.success, stop_res.message)

            recorded = _read_mcap_messages(bag_path)
            self.assertEqual(len(recorded.get("/tf_static", [])), 1)
            deserialized = deserialize_message(recorded["/tf_static"][0], TFMessage)
            self.assertEqual(deserialized.transforms[0].child_frame_id, "laser")
        finally:
            if recorder._recording:
                recorder._close_bag()
            executor.remove_node(recorder)
            executor.remove_node(pub_node)
            recorder.destroy_node()
            pub_node.destroy_node()

    def test_4_launch_files_do_not_create_duplicate_recorder_nodes(self):
        """robot_launch.py and mapping_bag_launch.py must instantiate each recorder node only once."""
        import importlib.util
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, OpaqueFunction
        from launch_ros.actions import Node as LaunchNode

        def load_launch_module(filename: str):
            path = PACKAGE_ROOT / "launch" / filename
            spec = importlib.util.spec_from_file_location(filename[:-3], path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

        robot_launch = load_launch_module("robot_launch.py")
        context = LaunchContext()
        params_path = str(PACKAGE_ROOT / "config" / "params.yaml")
        for key, val in [
            ("enable_camera", "false"),
            ("enable_mks", "true"),
            ("enable_battery", "true"),
            ("enable_lidar", "true"),
            ("enable_imu", "true"),
            ("params_file", params_path),
            ("wheel_backend", ""),
        ]:
            context.launch_configurations[key] = val

        nodes = robot_launch._launch_setup(context)
        recorder_nodes = [
            n
            for n in nodes
            if isinstance(n, LaunchNode)
            and getattr(n, "_Node__node_executable", None) == "bag_recorder_node"
        ]
        recorder_names = [getattr(n, "_Node__node_name", None) for n in recorder_nodes]
        self.assertEqual(
            recorder_names,
            ["perception_recorder", "mapping_recorder"],
            f"Expected exactly one perception_recorder and one mapping_recorder, got {recorder_names}",
        )

        # Also check mapping_bag_launch.py
        mapping_launch = load_launch_module("mapping_bag_launch.py")
        mapping_context = LaunchContext()
        for key, val in [
            ("enable_mks", "true"),
            ("enable_lidar", "true"),
            ("enable_imu", "true"),
            ("params_file", params_path),
            ("wheel_backend", "mks"),
        ]:
            mapping_context.launch_configurations[key] = val
        mapping_nodes = mapping_launch._launch_setup(mapping_context)
        mapping_recorder_nodes = [
            e
            for e in mapping_nodes
            if isinstance(e, LaunchNode)
            and getattr(e, "_Node__node_executable", None) == "bag_recorder_node"
        ]
        mapping_recorder_names = [
            getattr(n, "_Node__node_name", None) for n in mapping_recorder_nodes
        ]
        self.assertEqual(mapping_recorder_names, ["mapping_recorder"])

    def test_5_two_recorders_cannot_write_to_same_mcap_file(self):
        """Two recorders with the same prefix/output_dir or explicit path cannot write to the same MCAP."""
        rec1 = self._make_recorder(
            node_name="collision_recorder_1",
            bag_prefix="mapping",
            topics=["/odom"],
        )
        rec2 = self._make_recorder(
            node_name="collision_recorder_2",
            bag_prefix="mapping",
            topics=["/odom"],
        )

        try:
            # First recorder starts successfully
            res1 = rec1._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(res1.success, res1.message)
            self.assertTrue(rec1._recording)
            first_bag_path = rec1._bag_path

            # Second recorder with same output_dir and bag_prefix must be rejected while rec1 is active
            res2 = rec2._srv_start(Trigger.Request(), Trigger.Response())
            self.assertFalse(res2.success)
            self.assertFalse(rec2._recording)
            self.assertIn("already active", res2.message)

            # Even if someone calls _open_bag(explicit_bag_path=first_bag_path) on another recorder,
            # it must fail rather than opening the same MCAP
            rec3 = self._make_recorder(
                node_name="collision_recorder_3",
                bag_prefix="other_prefix",
                topics=["/odom"],
            )
            try:
                with self.assertRaises((RuntimeError, FileExistsError)):
                    rec3._open_bag(explicit_bag_path=first_bag_path)
            finally:
                rec3.destroy_node()

            # Stop rec1; now starting rec2 within the same second (or with existing directory)
            # must allocate a distinct non-colliding bag directory rather than overwriting first_bag_path
            stop1 = rec1._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(stop1.success, stop1.message)

            res2_after = rec2._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(res2_after.success, res2_after.message)
            second_bag_path = rec2._bag_path
            self.assertNotEqual(first_bag_path, second_bag_path)

            stop2 = rec2._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(stop2.success, stop2.message)
        finally:
            for r in (rec1, rec2):
                if r._recording:
                    r._close_bag()
                r.destroy_node()

    def test_6_generated_mcap_files_readable_and_status_published(self):
        """End-to-end offline integration test: mixed QoS topics, status topic, and MCAP readability."""
        recorder = self._make_recorder(
            node_name="integration_mapping_recorder",
            bag_prefix="mapping",
            topics=["/odom", "/scan", "/imu/data_raw", "/tf", "/tf_static"],
        )
        helper_node = rclpy.create_node("integration_test_helper")

        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        odom_pub = helper_node.create_publisher(Odometry, "/odom", qos_rel)
        scan_pub = helper_node.create_publisher(LaserScan, "/scan", qos_rel)
        tf_pub = helper_node.create_publisher(TFMessage, "/tf", qos_rel)
        imu_pub = helper_node.create_publisher(
            Imu, "/imu/data_raw", qos_profile_sensor_data
        )

        status_msgs: list[dict] = []
        helper_node.create_subscription(
            String,
            "/integration_mapping_recorder/status",
            lambda m: status_msgs.append(json.loads(m.data)),
            10,
        )

        executor = SingleThreadedExecutor()
        executor.add_node(recorder)
        executor.add_node(helper_node)

        try:
            _spin_for(executor, 0.3)

            start_res = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(start_res.success, start_res.message)
            bag_path = recorder._bag_path

            for i in range(6):
                odom = Odometry()
                odom.twist.twist.linear.x = 0.25 * (i + 1)
                odom_pub.publish(odom)

                scan = LaserScan()
                scan.ranges = [1.0, 2.0, 3.0]
                scan_pub.publish(scan)

                imu = Imu()
                imu.angular_velocity.z = -0.15 * (i + 1)
                imu_pub.publish(imu)

                tf_msg = TFMessage()
                ts = TransformStamped()
                ts.header.frame_id = "odom"
                ts.child_frame_id = "base_link"
                tf_msg.transforms.append(ts)
                tf_pub.publish(tf_msg)

                _spin_for(executor, 0.05)

            recorder._publish_status()
            _spin_for(executor, 0.2)

            stop_res = recorder._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(stop_res.success, stop_res.message)

            # Verify status messages were published and well-formed
            self.assertGreaterEqual(len(status_msgs), 1)
            latest_status = status_msgs[-1]
            self.assertIn("recording", latest_status)
            self.assertIn("frame_count", latest_status)
            self.assertIn("selected_topics", latest_status)

            # Verify MCAP file exists and can be read cleanly with SequentialReader
            mcap_files = list(Path(bag_path).glob("*.mcap"))
            self.assertEqual(len(mcap_files), 1)
            self.assertGreater(mcap_files[0].stat().st_size, 0)

            recorded = _read_mcap_messages(bag_path)
            self.assertEqual(len(recorded.get("/odom", [])), 6)
            self.assertEqual(len(recorded.get("/scan", [])), 6)
            self.assertEqual(len(recorded.get("/imu/data_raw", [])), 6)
            self.assertEqual(len(recorded.get("/tf", [])), 6)
        finally:
            if recorder._recording:
                recorder._close_bag()
            executor.remove_node(recorder)
            executor.remove_node(helper_node)
            recorder.destroy_node()
            helper_node.destroy_node()

    def test_7_tf_static_multiple_publishers_and_repeated_start_stop_cycles(self):
        """/tf_static cache must retain >10 transforms across multiple publishers, deduplicate
        repeated (frame_id, child_frame_id) pairs, and write them on repeated start/stop cycles.
        """
        recorder = self._make_recorder(
            node_name="tf_static_multi_recorder",
            bag_prefix="tf_static_cycle",
            topics=["/tf_static"],
        )
        tf_static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_node_a = rclpy.create_node("tf_static_pub_a")
        pub_node_b = rclpy.create_node("tf_static_pub_b")
        pub_a = pub_node_a.create_publisher(TFMessage, "/tf_static", tf_static_qos)
        pub_b = pub_node_b.create_publisher(TFMessage, "/tf_static", tf_static_qos)

        executor = SingleThreadedExecutor()
        executor.add_node(recorder)
        executor.add_node(pub_node_a)
        executor.add_node(pub_node_b)

        try:
            _spin_for(executor, 0.25)

            # Publish 12 distinct static transforms across 2 publishers (>10 messages)
            for idx in range(6):
                msg_a = TFMessage()
                ts_a = TransformStamped()
                ts_a.header.frame_id = "base_link"
                ts_a.child_frame_id = f"sensor_a_{idx}"
                ts_a.transform.translation.x = float(idx)
                msg_a.transforms.append(ts_a)
                pub_a.publish(msg_a)
                _spin_for(executor, 0.03)

                msg_b = TFMessage()
                ts_b = TransformStamped()
                ts_b.header.frame_id = "base_link"
                ts_b.child_frame_id = f"sensor_b_{idx}"
                ts_b.transform.translation.y = float(idx)
                msg_b.transforms.append(ts_b)
                pub_b.publish(msg_b)
                _spin_for(executor, 0.03)

            # Re-publish sensor_a_0 with updated translation to verify deduplication (no duplicates)
            msg_update = TFMessage()
            ts_update = TransformStamped()
            ts_update.header.frame_id = "base_link"
            ts_update.child_frame_id = "sensor_a_0"
            ts_update.transform.translation.x = 99.0
            msg_update.transforms.append(ts_update)
            pub_a.publish(msg_update)
            _spin_for(executor, 0.15)

            # Cycle 1: start -> stop
            res_start_1 = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(res_start_1.success, res_start_1.message)
            bag_1 = recorder._bag_path
            res_stop_1 = recorder._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(res_stop_1.success, res_stop_1.message)

            # Cycle 2: start -> stop (without republishing /tf_static)
            res_start_2 = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(res_start_2.success, res_start_2.message)
            bag_2 = recorder._bag_path
            res_stop_2 = recorder._srv_stop(Trigger.Request(), Trigger.Response())
            self.assertTrue(res_stop_2.success, res_stop_2.message)

            self.assertNotEqual(bag_1, bag_2)

            for bag_path in (bag_1, bag_2):
                recorded = _read_mcap_messages(bag_path)
                self.assertEqual(len(recorded.get("/tf_static", [])), 1)
                tf_msg = deserialize_message(recorded["/tf_static"][0], TFMessage)
                # All 12 unique (frame_id, child_frame_id) transforms must be present without duplicates
                self.assertEqual(len(tf_msg.transforms), 12)
                by_child = {t.child_frame_id: t for t in tf_msg.transforms}
                self.assertEqual(len(by_child), 12)
                self.assertAlmostEqual(by_child["sensor_a_0"].transform.translation.x, 99.0)
        finally:
            if recorder._recording:
                recorder._close_bag()
            executor.remove_node(recorder)
            executor.remove_node(pub_node_a)
            executor.remove_node(pub_node_b)
            recorder.destroy_node()
            pub_node_a.destroy_node()
            pub_node_b.destroy_node()

    def test_8_lock_lifecycle_on_failure_repeated_start_destroy_and_kill(self):
        """Locks must be released on open failure, destroy_node(), and unexpected SIGKILL."""
        import signal
        import subprocess

        recorder = self._make_recorder(
            node_name="lock_lifecycle_recorder",
            bag_prefix="lifecycle",
            topics=["/odom"],
        )

        try:
            # 1. Initialization failure in SequentialWriter.open releases all locks
            with patch("rosbag2_py.SequentialWriter.open", side_effect=RuntimeError("simulated writer open failure")):
                fail_res = recorder._srv_start(Trigger.Request(), Trigger.Response())
                self.assertFalse(fail_res.success)
                self.assertIn("simulated writer open failure", fail_res.message)
                self.assertFalse(recorder._recording)
                self.assertEqual(len(recorder._lock_files), 0)

            # 2. Normal start after a failed start succeeds immediately (locks were not leaked)
            start_res = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(start_res.success, start_res.message)
            self.assertEqual(len(recorder._lock_files), 2)

            # 3. Repeated start request while already recording is rejected without disturbing locks
            repeat_res = recorder._srv_start(Trigger.Request(), Trigger.Response())
            self.assertFalse(repeat_res.success)
            self.assertEqual(repeat_res.message, "Already recording")
            self.assertTrue(recorder._recording)
            self.assertEqual(len(recorder._lock_files), 2)

            # 4. Calling destroy_node() while recording automatically closes bag and releases locks
            recorder.destroy_node()
            self.assertFalse(recorder._recording)
            self.assertEqual(len(recorder._lock_files), 0)
        finally:
            if recorder._recording:
                recorder._close_bag()

        # Verify another recorder can immediately start after destroy_node()
        recorder_after = self._make_recorder(
            node_name="lock_lifecycle_recorder_after",
            bag_prefix="lifecycle",
            topics=["/odom"],
        )
        try:
            res_after = recorder_after._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(res_after.success, res_after.message)
            recorder_after._srv_stop(Trigger.Request(), Trigger.Response())
        finally:
            recorder_after.destroy_node()

        # 5. Unexpected process termination (SIGKILL) releases OS flock so subsequent recorder succeeds
        lock_file_path = self.bags_dir / ".lifecycle.recording.lock"
        proc = subprocess.Popen(
            [
                sys.executable,
                "-c",
                (
                    "import fcntl, sys, time; "
                    f"f = open({str(lock_file_path)!r}, 'a+'); "
                    "fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB); "
                    "sys.stdout.write('LOCKED\\n'); sys.stdout.flush(); "
                    "time.sleep(30)"
                ),
            ],
            stdout=subprocess.PIPE,
            text=True,
        )
        try:
            line = proc.stdout.readline().strip()
            self.assertEqual(line, "LOCKED")
            # Kill child abruptly with SIGKILL
            os.kill(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        finally:
            if proc.poll() is None:
                proc.kill()
            if proc.stdout:
                proc.stdout.close()

        # Confirm kernel released the flock after SIGKILL and a new recorder can start
        recorder_post_kill = self._make_recorder(
            node_name="lock_lifecycle_post_kill",
            bag_prefix="lifecycle",
            topics=["/odom"],
        )
        try:
            res_post_kill = recorder_post_kill._srv_start(Trigger.Request(), Trigger.Response())
            self.assertTrue(res_post_kill.success, res_post_kill.message)
            recorder_post_kill._srv_stop(Trigger.Request(), Trigger.Response())
        finally:
            recorder_post_kill.destroy_node()


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Quick diagnostic: check if RealSense camera topics are being published
and whether we can subscribe to them.

Usage (on the Jetson, with the robot launch running):
    python3 camera_topic_check.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

TOPICS = [
    "/camera/color/image_raw",
    "/camera/aligned_depth_to_color/image_raw",
    "/camera/color/camera_info",
    "/camera/aligned_depth_to_color/camera_info",
]

QOS_PROFILES = {
    "best_effort": QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    ),
    "reliable": QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    ),
}


class CameraCheck(Node):
    def __init__(self):
        super().__init__("camera_topic_check")
        self._counts: dict[str, int] = {}
        self._first: dict[str, float] = {}

        print("\n=== Camera Topic Diagnostic ===\n")

        # List all published topics on the network
        time.sleep(1.0)  # let discovery settle
        published = self.get_topic_names_and_types()
        camera_topics = [(n, t) for n, t in published if "camera" in n.lower()]

        if camera_topics:
            print(f"Found {len(camera_topics)} camera-related topics on the network:")
            for name, types in sorted(camera_topics):
                print(f"  {name}  [{', '.join(types)}]")
        else:
            print("!! NO camera topics found on the network !!")
            print("   Is the RealSense node running?")
            print("   Check: ros2 topic list | grep camera")
            all_topics = [n for n, _ in published]
            print(f"\n   All topics ({len(all_topics)}):")
            for t in sorted(all_topics):
                print(f"     {t}")

        # Check publisher info for our target topics
        print(f"\n--- Publisher QoS info for target topics ---")
        for topic in TOPICS:
            infos = self.get_publishers_info_by_topic(topic)
            if not infos:
                print(f"  {topic}: NO publishers")
            else:
                for info in infos:
                    qos = info.qos_profile
                    print(
                        f"  {topic}: "
                        f"reliability={qos.reliability}, "
                        f"durability={qos.durability}, "
                        f"depth={qos.depth}"
                    )

        # Try importing sensor_msgs
        print(f"\n--- Import check ---")
        try:
            from sensor_msgs.msg import Image, CameraInfo
            print(f"  sensor_msgs.msg.Image: OK ({Image})")
            print(f"  sensor_msgs.msg.CameraInfo: OK ({CameraInfo})")
        except ImportError as e:
            print(f"  !! IMPORT FAILED: {e}")
            print(f"  Install: sudo apt install ros-humble-sensor-msgs")
            return

        # Subscribe with both QoS profiles
        print(f"\n--- Subscribing (waiting 10 seconds for messages) ---")
        from sensor_msgs.msg import Image, CameraInfo

        type_map = {
            "/camera/color/image_raw": Image,
            "/camera/aligned_depth_to_color/image_raw": Image,
            "/camera/color/camera_info": CameraInfo,
            "/camera/aligned_depth_to_color/camera_info": CameraInfo,
        }

        for topic in TOPICS:
            self._counts[topic] = 0
            msg_cls = type_map[topic]
            for label, qos in QOS_PROFILES.items():
                self.create_subscription(
                    msg_cls,
                    topic,
                    lambda msg, t=topic: self._on_msg(t, msg),
                    qos,
                )

        start = time.monotonic()
        while time.monotonic() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.monotonic() - start
            summary = " | ".join(
                f"{t.split('/')[-1]}={c}" for t, c in self._counts.items()
            )
            sys.stdout.write(f"\r  [{elapsed:.1f}s] {summary}   ")
            sys.stdout.flush()

        print("\n\n=== Results ===")
        for topic, count in self._counts.items():
            status = "OK" if count > 0 else "!! NO MESSAGES"
            print(f"  {topic}: {count} messages  {status}")

        total = sum(self._counts.values())
        if total == 0:
            print("\n  No messages received on any topic.")
            print("  Possible causes:")
            print("    1. Camera node not running or crashed")
            print("    2. Topic names don't match (check list above)")
            print("    3. DDS domain mismatch")
        else:
            print(f"\n  Total: {total} messages — camera is working!")

    def _on_msg(self, topic, msg):
        self._counts[topic] = self._counts.get(topic, 0) + 1
        if topic not in self._first:
            self._first[topic] = time.monotonic()
            print(f"\n  >> First msg on {topic} (type={type(msg).__name__})")


def main():
    rclpy.init()
    try:
        node = CameraCheck()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

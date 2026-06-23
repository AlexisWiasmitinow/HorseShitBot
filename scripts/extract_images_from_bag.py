#!/usr/bin/env python3
"""Extract color frames from a MCAP rosbag for YOLO training."""

import argparse
from pathlib import Path

import cv2
import numpy as np
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image


def extract_images(bag_path: str, output_dir: str, topic: str = "/camera/color/image_raw"):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="mcap"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        print(f"Topic {topic} not found in bag. Available topics:")
        for t in topic_types:
            print(" ", t)
        return

    msg_type = get_message(topic_types[topic])
    frame_idx = 0

    while reader.has_next():
        t_name, data, _ = reader.read_next()
        if t_name != topic:
            continue

        msg = deserialize_message(data, msg_type)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # RealSense publishes rgb8; OpenCV writes expects BGR
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        fname = output / f"frame_{frame_idx:06d}.jpg"
        cv2.imwrite(str(fname), img_bgr)
        frame_idx += 1
        if frame_idx % 50 == 0:
            print(f"Extracted {frame_idx} frames...")

    print(f"Done: {frame_idx} frames written to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract images from a ROS 2 rosbag")
    parser.add_argument("bag_path", help="Path to the rosbag directory")
    parser.add_argument("-o", "--output", default="dataset/images/train", help="Output directory")
    parser.add_argument("-t", "--topic", default="/camera/color/image_raw", help="Image topic")
    args = parser.parse_args()
    extract_images(args.bag_path, args.output, args.topic)

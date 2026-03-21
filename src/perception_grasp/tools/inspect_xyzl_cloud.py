#!/usr/bin/env python3
"""
Inspect a PointCloud2 topic that stores XYZL points.

This script subscribes once, decodes the cloud fields
("x", "y", "z", "label"), prints a short summary, then exits.
"""

import argparse

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class CloudInspector:
    def __init__(self, topic, sample_count, timeout_sec):
        self.topic = topic
        self.sample_count = sample_count
        self.timeout_sec = timeout_sec
        self.received = False

    def callback(self, msg):
        pts = list(
            pc2.read_points(
                msg,
                field_names=("x", "y", "z", "label"),
                skip_nans=True,
            )
        )

        print(f"topic: {self.topic}")
        print(f"frame_id: {msg.header.frame_id}")
        print(f"stamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}")
        print(f"width: {msg.width}")
        print(f"height: {msg.height}")
        print(f"point_step: {msg.point_step}")
        print(f"decoded_points: {len(pts)}")

        unique_labels = sorted({int(p[3]) for p in pts})
        print(f"instance_labels: {unique_labels}")
        print(f"first_{min(self.sample_count, len(pts))}_points:")
        for x, y, z, label in pts[: self.sample_count]:
            print(f"  x={x:.4f}, y={y:.4f}, z={z:.4f}, label={int(label)}")

        self.received = True
        rospy.signal_shutdown("decoded one cloud")

    def run(self):
        rospy.init_node("inspect_xyzl_cloud", anonymous=True)
        rospy.Subscriber(self.topic, PointCloud2, self.callback, queue_size=1)

        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.timeout_sec)
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown() and not self.received:
                if rospy.Time.now() > deadline:
                    print(f"timeout waiting for topic: {self.topic}")
                    break
                rate.sleep()
        except rospy.ROSInterruptException:
            # Expected when we intentionally stop after decoding one cloud.
            pass


def parse_args():
    parser = argparse.ArgumentParser(description="Decode an XYZL PointCloud2 topic once.")
    parser.add_argument(
        "--topic",
        default="/perception/yolo_bbox_instance_cloud",
        help="PointCloud2 topic to inspect.",
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=10,
        help="How many decoded points to print.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for one cloud before exiting.",
    )
    return parser.parse_args(rospy.myargv()[1:])


if __name__ == "__main__":
    args = parse_args()
    inspector = CloudInspector(args.topic, args.sample_count, args.timeout)
    inspector.run()
